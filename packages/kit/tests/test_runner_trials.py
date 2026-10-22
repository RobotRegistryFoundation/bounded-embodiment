# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""`be run ev02` and `be run ev05`: the trial tripwire, what the runner refuses before anything
moves, and the adapters' stop sources (trigger_stop, reset_stop, the simulated external stop).
The simulated runs themselves are in test_runner_trials_sim.py."""
from __future__ import annotations

import json
import sys
import threading
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import _runner_fakes as fakes
import _trial_gates
from _submission_support import be
from _trials_sim import PLANS, ROVER_ENVELOPE, ROVER_SPACE
from bounded_embodiment import envelope as envelope_mod
from bounded_embodiment.commandspace import load_command_space
from bounded_embodiment.report import load_stop_plan
from bounded_embodiment_kit.adapters.base import AdapterError
from bounded_embodiment_kit.adapters.lerobot import LeRobotAdapter
from bounded_embodiment_kit.adapters.sim_rover import SimRoverAdapter
from bounded_embodiment_kit.runner import run as runmod
from bounded_embodiment_kit.runner.kin import ArmFK
from bounded_embodiment_kit.runner.limits import DEFAULT
from bounded_embodiment_kit.runner.trials import TrialSegment, _between_trials
from bounded_embodiment_kit.runner.watchdog import StopLatch, TrialWatch, trial_trip

# ----- the trial tripwire (a pure function) ----------------------------------------------------------


def _samples(t0: float, t1: float, at, hz: float = 50.0) -> list[tuple[float, tuple]]:
    out, k = [], 0
    while t0 + k / hz <= t1:
        t = t0 + k / hz
        out.append((t, at(t)))
        k += 1
    return out


def test_the_trial_tripwire_waits_for_the_time_the_trial_allows():
    watch = TrialWatch("trial 1 (stop_message, drive x)", t_event=10.0, allowed_s=0.3)
    moving = _samples(9.0, 10.79, lambda t: (0.1 * t, 0.0, 0.0))
    assert trial_trip(moving, 10.79, watch) == (None, None)  # before 10.0 + 0.3 + 0.5


def test_a_machine_still_moving_after_the_allowed_time_trips_it():
    watch = TrialWatch("trial 1 (stop_message, drive x)", t_event=10.0, allowed_s=0.3)
    moving = _samples(9.0, 10.85, lambda t: (0.1 * t, 0.0, 0.0))  # 10 cm/s: 5 cm in 0.5 s
    reason, rested = trial_trip(moving, 10.85, watch)
    assert rested is None and reason.startswith("the tool point is not at rest 0.85 s after trial 1")
    assert "it moved 5.0 cm in the last 0.5 s" in reason


def test_a_machine_at_rest_is_remembered_and_must_stay_there():
    watch = TrialWatch("trial 2 (power_loss, drive y)", t_event=10.0, allowed_s=0.3)
    still = _samples(10.0, 10.9, lambda t: (0.2, 0.0, 0.1))
    assert trial_trip(still, 10.9, watch) == (None, (0.2, 0.0, 0.1))
    watch.rested_at = (0.2, 0.0, 0.1)
    drift = still + _samples(10.92, 12.0, lambda t: (0.2 + 0.03 * (t - 10.9), 0.0, 0.1))
    assert trial_trip(drift[:-30], 11.4, watch)[0] is None  # 1.5 cm: within 2 cm
    reason, rested = trial_trip(drift, 12.0, watch)
    assert rested == (0.2, 0.0, 0.1)
    assert reason.startswith("the tool point moved 3.3 cm from where it came to rest after trial 2")


def test_too_few_samples_is_no_judgement():
    watch = TrialWatch("trial 1", t_event=0.0, allowed_s=0.1)
    assert trial_trip([], 5.0, watch) == (None, None)
    assert trial_trip([(4.99, (0.0, 0.0, 0.0))], 5.0, watch) == (None, None)


def test_the_trial_numbers_live_with_the_runners_limits():
    assert (DEFAULT.trial_rest_grace_s, DEFAULT.trial_still_m, DEFAULT.trial_still_window_s,
            DEFAULT.trial_hold_trip_m, DEFAULT.trial_pause_extra_s) == (0.5, 0.02, 0.5, 0.02, 0.5)


# ----- the operator's NEXT between trials -------------------------------------------------------------


class _AsksOnce:
    """Stands in for TrialCommandThread: asks for NEXT before trial 2, and ends once it is given."""

    def __init__(self, segment: TrialSegment):
        self.segments, self.next_trial = (segment, segment), 1
        self.asking, self.next_go = threading.Event(), threading.Event()
        self.asking.set()

    def is_alive(self) -> bool:
        return not self.next_go.is_set()


@pytest.mark.parametrize(("kind", "source", "reset", "said"), [
    ("stop", "hardwired_stop", True, "Reset hardwired_stop first; the machine must not move"),
    ("stop", "power_loss", True, "Reset power_loss first; the machine must not move"),
    ("stop", "stop_message", True, "After NEXT the kit resets stop_message through the adapter."),
    ("stop", "stop_message", False, None),
    ("loss", "heartbeat_loss", True, "After NEXT the kit resets heartbeat_loss through the adapter."),
    ("loss", "comms_loss", False, None),
])
def test_the_next_prompt_says_who_resets_the_source(kind, source, reset, said):
    command = _AsksOnce(TrialSegment((), f"trial 1 ({source})", kind, source, 0.5, 1.0, reset))
    latch, asked = StopLatch(lambda: 0.0), []
    _between_trials(latch, command, lambda q: asked.append(q) or "NEXT", DEFAULT)
    assert command.next_go.is_set() and not latch.tripped
    assert len(asked) == 1 and asked[0].startswith(f"trial 1 ({source}) done (1 of 2).")
    if said is None:
        assert "eset" not in asked[0]
    else:
        assert said in asked[0]


def test_anything_but_next_ends_the_run_through_the_latch():
    latch = StopLatch(lambda: 0.0)
    command = _AsksOnce(TrialSegment((), "trial 1 (stop_message)", "stop", "stop_message", 0.5, 1.0, True))
    command.is_alive = lambda: not latch.tripped
    _between_trials(latch, command, lambda q: "next", DEFAULT)
    assert not command.next_go.is_set()
    assert latch.kind == "operator" and "before trial 2 of 2" in latch.reason


# ----- refusals before anything moves ---------------------------------------------------------------


def _write(path: Path, doc) -> Path:
    path.write_text(json.dumps(doc, indent=2) + "\n")
    return path


def _rover_argv(tmp_path: Path, test: str = "ev02", plan: dict | None = None, *extra) -> list:
    plan_doc = plan if plan is not None else PLANS[f"rover-{test}"]
    return ["run", test, "--mode", "full", "--adapter", "sim-rover", "--instrument", "sim",
            "--envelope", str(_write(tmp_path / "env.json", ROVER_ENVELOPE)),
            "--command-space", str(ROVER_SPACE), "--no-fk-tripwire-i-understand",
            "--plan", str(_write(tmp_path / "plan.json", plan_doc)),
            "--out", str(tmp_path / "run"), "--i-accept-the-risk", "--hardware-stop", "none (simulation)",
            "--ledger", str(tmp_path / "ledger.jsonl"), "--step-cap", "0.05", *extra]


def _refused(argv, tmp_path: Path) -> str:
    logs: list[str] = []
    code = runmod.execute_run(be(*argv), prompt=lambda q: "", log=logs.append)
    assert code == runmod.EXIT_REFUSED, logs
    assert not (tmp_path / "run").exists() or not any((tmp_path / "run" / "raw").glob("*.csv"))
    return "\n".join(logs)  # the refusal, and after a refusal once connected, the adapter's close


@pytest.mark.parametrize(("change", "words"), [
    (["--mode", "shakedown"], "--mode full --plan FILE"),
    (["--trials", "3"], "are for ev05"),
    (["--seconds", "5"], "--seconds and --target are for ev03"),
    (["--cycles", "2"], "are for be run ev10"),
])
def test_ev02_takes_a_full_run_with_its_own_flags(tmp_path, change, words):
    argv = _rover_argv(tmp_path)
    if change[0] == "--mode":
        argv[argv.index("--mode") + 1] = change[1]
    else:
        argv += change
    assert words in _refused(argv, tmp_path)


def test_trial_flags_belong_to_trial_runs(tmp_path):
    argv = ["run", "ev03", "--mode", "full", "--adapter", "sim-rover", "--instrument", "sim",
            "--envelope", str(fakes.ENVELOPES / "rover.envelope.json"), "--command-space", str(ROVER_SPACE),
            "--no-fk-tripwire-i-understand", "--out", str(tmp_path / "run"), "--i-accept-the-risk",
            "--hardware-stop", "none (simulation)", "--plan", "p.json"]
    assert "for be run ev02 and ev05" in _refused(argv, tmp_path)


def test_a_bad_plan_is_refused_before_anything_connects(tmp_path):
    plan = json.loads(json.dumps(PLANS["rover-ev02"]))
    plan["drives"][0]["command"]["linear"] = 9.0  # the command space allows 1.5
    assert "outside" in _refused(_rover_argv(tmp_path, "ev02", plan), tmp_path)
    plan = json.loads(json.dumps(PLANS["rover-ev02"]))
    plan["trials"].append({"source": "hardwired_stop", "drive": "forward"})
    assert "invalid plan" in _refused(_rover_argv(tmp_path, "ev02", plan), tmp_path)


def test_an_external_stop_needs_the_estop_command_and_a_reset(tmp_path):
    from _b6 import plant_shakedown

    args = be(*_rover_argv(tmp_path))
    plant_shakedown(args, tmp_path / "ledger.jsonl")
    assert "issues it with the lab's --estop-cmd" in _refused(_rover_argv(tmp_path), tmp_path)
    # with the command but without the simulated e-stop, sim-rover cannot resume after power loss
    argv = _rover_argv(tmp_path, "ev02", None, "--estop-cmd", "true")
    assert "cannot resume after it" in _refused(argv, tmp_path)


def test_a_full_trial_run_needs_a_shakedown_on_the_setup(tmp_path):
    argv = _rover_argv(tmp_path, "ev02", None, "--estop-cmd", "true", "--sim-estop-file",
                       str(tmp_path / "estop"))
    assert "needs a shakedown on this setup" in _refused(argv, tmp_path)


def test_ev05_refuses_a_silence_the_hold_cannot_fit_in_and_an_uncapped_walk(tmp_path):
    argv = _rover_argv(tmp_path, "ev05", None, "--silence-s", "2.0")
    assert "--silence-s 2 is shorter than" in _refused(argv, tmp_path)
    argv = _rover_argv(tmp_path, "ev05", None, "--restart-s", "1.0")
    assert "--restart-s 1 is shorter than the restart check (2 s)" in _refused(argv, tmp_path)
    argv = _rover_argv(tmp_path, "ev05")
    argv[argv.index("--step-cap") + 1] = "none"
    assert "none is refused" in _refused(argv, tmp_path)


def test_a_stop_message_needs_an_adapter_that_can_issue_one(tmp_path):
    """sim-so101 without a wrapper is LeRobot's bare follower: it has no stop request of its own."""
    from _b6 import SO101, plant_shakedown
    from _trials_sim import PLANS as P

    plan = _write(tmp_path / "plan.json", P["so101-ev02"])
    argv = ["run", "ev02", "--mode", "full", "--adapter", "sim-so101", "--instrument", "sim",
            "--envelope", str(SO101["envelope"]), "--command-space", str(SO101["command_space"]),
            "--kit-config", str(SO101["kit_config"]), "--urdf", str(SO101["urdf"]), "--plan", str(plan),
            "--out", str(tmp_path / "run"), "--i-accept-the-risk", "--hardware-stop", "none (simulation)",
            "--ledger", str(tmp_path / "ledger.jsonl"), "--step-cap", "1.5", "--estop-cmd", "true",
            "--sim-estop-file", str(tmp_path / "estop")]
    plant_shakedown(be(*argv), tmp_path / "ledger.jsonl")
    assert "which this adapter cannot issue" in _refused(argv, tmp_path)


def test_every_plan_pose_must_keep_the_arm_inside_its_zones():
    from _b6 import SO101

    envelope = envelope_mod.load(SO101["envelope"])
    kit = json.loads(Path(SO101["kit_config"]).read_text())
    fk = ArmFK.build(SO101["urdf"], kit.get("joint_map"), envelope)
    space = load_command_space(SO101["command_space"])
    from _trials_sim import PLANS as P

    runmod.check_plan_poses(fk, envelope, space, load_stop_plan(P["so101-ev02"]))
    doc = json.loads(json.dumps(P["so101-ev02"]))
    doc["drives"][0]["command"]["shoulder_lift.pos"] = 60.0  # down through the table
    with pytest.raises(runmod.RunError, match="the plan's /drives/0/command puts tool"):
        runmod.check_plan_poses(fk, envelope, space, load_stop_plan(doc))


def test_a_capped_walk_must_start_at_the_plans_start():
    space = load_command_space(fakes.SO101_SPACE)
    start = {a.key: 0.0 for a in space.axes}
    caps = dict.fromkeys(space.keys, 1.5)
    runmod.check_trial_start(space, caps, dict(start), start)
    with pytest.raises(runmod.RunError, match="more than one step from it now"):
        runmod.check_trial_start(space, caps, {**start, "elbow_flex.pos": 3.0}, start)
    runmod.check_trial_start(space, None, {**start, "elbow_flex.pos": 3.0}, start)  # no cap: no walk
    rover = load_command_space(ROVER_SPACE)
    runmod.check_trial_start(rover, {"linear": 0.05, "angular": 0.05}, None,
                             {"linear": 0.0, "angular": 0.0})  # a velocity starts from rest


# ----- the adapters' stop sources -------------------------------------------------------------------


def test_sim_rover_issues_its_own_stop_and_resumes_only_when_reset(tmp_path):
    estop = tmp_path / "estop"
    a = SimRoverAdapter(command_space_path=ROVER_SPACE, realtime=False, estop_file=estop)
    a.open()
    d = a.describe()
    assert d["stop_triggers"] == ["stop_message"]
    assert set(d["stop_resets"]) == {"stop_message", "heartbeat_loss", "comms_loss", "hardwired_stop", "power_loss"}
    assert a.send({"linear": 0.5, "angular": 0.0}).applied
    a.trigger_stop("stop_message")
    assert a.rover.cmd_v == 0.0 and not a.send({"linear": 0.5, "angular": 0.0}).applied
    a.reset_stop("stop_message")
    assert a.rover.cmd_v == 0.0  # a reset moves nothing
    assert a.send({"linear": 0.5, "angular": 0.0}).applied
    estop.touch()
    a.step(0.01)
    assert a.rover.cut and not a.send({"linear": 0.5, "angular": 0.0}).applied
    with pytest.raises(AdapterError, match="still open"):
        a.reset_stop("power_loss")
    estop.unlink()
    a.reset_stop("power_loss")
    assert not a.rover.cut and a.send({"linear": 0.1, "angular": 0.0}).applied
    with pytest.raises(AdapterError, match="issues stop_message only"):
        a.trigger_stop("power_loss")
    a.latch_stop()
    with pytest.raises(AdapterError, match="latched"):
        a.trigger_stop("stop_message")
    a.close()


def test_the_lerobot_adapter_issues_the_wrappers_stop_and_never_resets_an_external_one(tmp_path):
    bare = fakes.sim_adapter()
    bare.open()
    assert bare.describe()["stop_triggers"] == [] and bare.describe()["stop_resets"] == []
    with pytest.raises(AdapterError, match="needs a --wrapper with stop"):
        bare.trigger_stop("stop_message")
    bare.close()

    a = fakes.sim_adapter(wrapper="_trial_gates:stoppable", estop_file=tmp_path / "estop")
    a.open()
    assert a.describe()["stop_triggers"] == ["stop_message"]
    gate = a._target
    assert isinstance(gate, _trial_gates.TrialGate)
    a.trigger_stop("stop_message")
    assert gate.held and gate.events[:2] == ["stop", "hold:stop"]
    a.reset_stop("stop_message")
    assert not gate.held and gate.events[-1] == "resume"
    # LeRobot's own adapter never resumes after power was cut (only the simulator overrides it)
    with pytest.raises(AdapterError, match="cannot reset power_loss"):
        LeRobotAdapter.reset_stop(a, "power_loss")
    a.close()


def test_the_simulated_external_stop_cuts_the_servos_and_the_reset_syncs_before_torque(tmp_path):
    estop = tmp_path / "estop"
    a = fakes.sim_adapter(estop_file=estop, record=True)
    a.open()
    a.step(0.05)
    estop.touch()
    a.step(0.01)
    assert not a.servos.powered and set(a.servos.torque.values()) == {0}
    a.sim_robot.bus.enable_torque()  # an unpowered servo ignores torque writes
    assert set(a.servos.torque.values()) == {0}
    with pytest.raises(AdapterError, match="still open"):
        a.reset_stop("hardwired_stop")
    estop.unlink()
    calls_before = len(a.sim_robot.bus.calls)
    a.reset_stop("hardwired_stop")
    calls = a.sim_robot.bus.calls[calls_before:]
    goal = next(i for i, c in enumerate(calls) if c.method == "sync_write" and c.register == "Goal_Position")
    torque = next(i for i, c in enumerate(calls) if c.method == "enable_torque")
    assert goal < torque and a.servos.powered and set(a.servos.torque.values()) == {1}
    assert "hardwired_stop" in a.describe()["stop_resets"]
    a.close()
