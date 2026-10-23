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
from bounded_embodiment import report as core_report
from bounded_embodiment.commandspace import load_command_space
from bounded_embodiment.report import load_model_loss_plan, load_stop_plan
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
    assert trial_trip([(4.99, (0.0, 0.0, 0.0))], 5.0, watch) == (None, None)  # fresh, but too few


def test_a_watched_point_the_instrument_does_not_report_trips_the_watch():
    """Round-2 K2: once the trial's time has passed, no sample of the watched point since the
    instant, or a newest sample older than feed_silent_s, trips the watch: an instrument that
    does not see the plan's stop point leaves nothing to judge, and the watch must not sit idle."""
    watch = TrialWatch("trial 1", t_event=10.0, allowed_s=0.3, point="wrist")
    assert trial_trip([], 10.7, watch) == (None, None)  # before 10.0 + 0.3 + 0.5: no judgement yet
    reason, _ = trial_trip([], 10.81, watch)
    assert reason.startswith("no sample of the stop point (wrist) since trial 1's instant")
    before = _samples(9.0, 9.99, lambda t: (0.2, 0.0, 0.1))  # only samples from before the instant
    assert trial_trip(before, 10.81, watch)[0].startswith("no sample of the stop point (wrist)")
    stale = _samples(9.0, 10.2, lambda t: (0.2, 0.0, 0.1))  # it stopped reporting at 10.2 s
    assert trial_trip(stale, 10.69, watch)[0] is None  # 0.49 s old, and before the grace
    reason, _ = trial_trip(stale, 10.81, watch)
    assert reason.startswith("the newest sample of the stop point (wrist) is 0.61 s old")


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


@pytest.mark.parametrize(("kind", "source", "reset", "by_hand", "said"), [
    ("stop", "hardwired_stop", True, True, "Reset hardwired_stop first; the machine must not move"),
    ("stop", "power_loss", True, False, "Reset power_loss first; the machine must not move"),
    ("stop", "stop_message", True, False, "After NEXT the kit resets stop_message through the adapter."),
    ("stop", "stop_message", False, False, None),
    ("loss", "heartbeat_loss", True, False, "After NEXT the kit resets heartbeat_loss through the adapter."),
    ("loss", "heartbeat_loss", False, True, "Reset heartbeat_loss first; the machine must not move"),
    ("loss", "comms_loss", False, False, None),
])
def test_the_next_prompt_says_who_resets_the_source(kind, source, reset, by_hand, said):
    command = _AsksOnce(TrialSegment((), f"trial 1 ({source})", kind, source, 0.5, 1.0, reset,
                                     operator_reset=by_hand))
    latch, asked = StopLatch(lambda: 0.0), []
    _between_trials(latch, command, lambda q: asked.append(q) or "NEXT", DEFAULT)
    assert command.next_go.is_set() and not latch.tripped
    assert len(asked) == 1 and asked[0].startswith(f"trial 1 ({source}) done (1 of 2).")
    if said is None:
        assert "eset" not in asked[0]
    else:
        assert said in asked[0]


def _segments(tmp_path: Path, test: str, plan_doc: dict, desc: dict, estop_cmd: str | None):
    envelope = envelope_mod.load(_write(tmp_path / "env.json", ROVER_ENVELOPE))
    space = load_command_space(ROVER_SPACE)
    caps = {"linear": 0.05, "angular": 0.05}
    if test == "ev02":
        plan = load_stop_plan(plan_doc)
        params = core_report.streams.normalize(core_report.STOPS, {
            "hz": 50.0, "drive_s": 1.0, "return_s": 0.0, "after_s": 3.6, "step_cap": caps})
    else:
        plan = load_model_loss_plan(plan_doc)
        params = core_report.streams.normalize(core_report.MODEL_LOSS, {
            "hz": 50.0, "trials": 2, "motion_s": [1.0, 1.2], "silence_s": 3.0, "restart_s": 1.0,
            "step_cap": caps})
    cmds = {"hardwired_stop": estop_cmd, "power_loss": estop_cmd}
    segments, _ = runmod.trial_segments(test, space, plan, params, 7, envelope, desc, cmds, DEFAULT)
    return [(s.source, s.reset, s.operator_reset) for s in segments]


def test_each_trial_knows_who_resets_its_source(tmp_path):
    both = {"stop_triggers": ["stop_message"], "stop_resets": ["stop_message", "heartbeat_loss", "power_loss"]}
    ev02 = _segments(tmp_path, "ev02", PLANS["rover-ev02"], both, "touch /tmp/estop")
    assert ev02 == [("stop_message", True, False)] * 3 + [("power_loss", True, True)] * 3
    # EV-05, manual resume: the adapter resets the source after NEXT, or else the operator must
    assert _segments(tmp_path, "ev05", PLANS["rover-ev05"], both, None) == [("heartbeat_loss", True, False)] * 2
    assert (_segments(tmp_path, "ev05", PLANS["rover-ev05"], {"stop_triggers": [], "stop_resets": []}, None)
            == [("heartbeat_loss", False, True)] * 2)
    # ... and under a commands rule nobody resets anything
    doc = json.loads(json.dumps(PLANS["rover-ev05"]))
    doc["resume"] = {"rule": "commands", "after_ms": 500}
    assert _segments(tmp_path, "ev05", doc, both, None) == [("heartbeat_loss", False, False)] * 2


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


def test_an_external_stop_needs_its_own_command_and_a_reset(tmp_path):
    """D2: each external source has its own command; --estop-cmd is the CUT POWER escalation only
    and never issues a trial's stop."""
    from _b6 import plant_shakedown

    args = be(*_rover_argv(tmp_path))
    plant_shakedown(args, tmp_path / "ledger.jsonl")
    refused = _refused(_rover_argv(tmp_path), tmp_path)
    assert "the plan tests power_loss: give the command that issues it with --power-loss-cmd" in refused
    assert "--estop-cmd is the CUT POWER escalation only" in refused
    # --estop-cmd alone is not enough
    assert "--power-loss-cmd" in _refused(_rover_argv(tmp_path, "ev02", None, "--estop-cmd", "true"), tmp_path)
    # with the command but without the simulated e-stop, sim-rover cannot resume after power loss
    argv = _rover_argv(tmp_path, "ev02", None, "--power-loss-cmd", "true")
    assert "cannot resume after it" in _refused(argv, tmp_path)
    # the source commands are EV-02's
    ev05 = _rover_argv(tmp_path, "ev05", None, "--power-loss-cmd", "true")
    assert "--power-loss-cmd: for be run ev02" in _refused(ev05, tmp_path)


def test_a_full_trial_run_needs_a_shakedown_on_the_setup(tmp_path):
    argv = _rover_argv(tmp_path, "ev02", None, "--power-loss-cmd", "true", "--sim-estop-file",
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


# ----- review round 1: B1, S1-S3, S5, S7, N1, N3-N5, N8 ------------------------------------------------


def _bare_thread(limits=DEFAULT, **kw):
    """A TrialCommandThread with nothing behind it, to drive its helpers directly."""
    from bounded_embodiment_kit.runner.trials import TrialCommandThread
    from bounded_embodiment_kit.runner.watchdog import StateBox

    clock = __import__("time").monotonic
    return TrialCommandThread(adapter=None, latch=StopLatch(clock), state_box=StateBox(), fk=None,
                              files=None, clock=clock, stop_decided=lambda: True, limits=limits,
                              log=lambda m: None, segments=kw.pop("segments", ()), watchdog=None,
                              adapter_sources=(), **kw)


def _pending(cmd: str, thread) -> tuple[dict, object]:
    from bounded_embodiment_kit.runner.trials import spawn_source_command

    pend = {"p": None, "t_sent": thread.clock(), "k": 0, "done": threading.Event(), "result": None,
            "t_done": None}
    return pend, spawn_source_command(cmd)


def test_a_source_command_that_leaves_a_child_neither_blocks_nor_fails():
    """Review B1, scenario 3: with capture_output, `sh -c 'echo cut; sleep 30 &'` held stdout open
    for the whole 5 s and was taken for a failure. Its streams are closed now, so it ends at once."""
    import time

    thread = _bare_thread()
    pend, proc = _pending("sh -c 'echo cut; sleep 30 &'", thread)
    t0 = time.monotonic()
    thread._wait_for_command(pend, proc, "trial 1", "power_loss")
    assert time.monotonic() - t0 < 0.5
    assert pend["result"].applied and pend["result"].decision == "issued"
    assert not thread.latch.tripped


def test_a_failing_or_hung_source_command_trips_the_latch_off_the_command_thread():
    """Review B1: a non-zero exit or a timeout trips the latch, from the helper thread."""
    thread = _bare_thread()
    pend, proc = _pending("false", thread)
    thread._wait_for_command(pend, proc, "trial 1", "power_loss")
    assert pend["result"].error_kind == "external_returncode:1"
    assert thread.latch.kind == "error" and "external_returncode:1" in thread.latch.reason
    hung = _bare_thread(limits=DEFAULT.__class__(estop_timeout_s=0.7))
    pend, proc = _pending("sleep 10", hung)
    hung._wait_for_command(pend, proc, "trial 1", "hardwired_stop")
    assert pend["result"].error_kind == "external_timeout" and hung.latch.tripped
    proc.kill()


def test_a_source_command_that_outlives_its_deadline_is_ended_and_cannot_act_later(tmp_path):
    """Round-2 K1: at the deadline the command's process group is ended (SIGTERM, then SIGKILL),
    reaped, and recorded; the latch reason says it may have acted. Before, it ran on (in a session
    of its own) and could cut power after the run was over."""
    import time

    marker = tmp_path / "cut-late"
    hung = _bare_thread(limits=DEFAULT.__class__(estop_timeout_s=0.6))
    pend, proc = _pending(f"sh -c 'sleep 2; touch {marker}'", hung)
    hung._wait_for_command(pend, proc, "trial 1", "power_loss")
    assert pend["result"].error_kind == "external_timeout"
    assert pend["ended"] in ("SIGTERM", "SIGKILL") and proc.poll() is not None
    assert "may have acted" in hung.latch.reason and "ended" in hung.latch.reason
    time.sleep(2.5)
    assert not marker.exists()  # the command, and the sleep inside it, were ended


def test_a_source_command_still_running_when_the_run_ends_is_ended(tmp_path):
    """Round-2 K1: the run can end (a tripwire) before a command's deadline; the command does not
    outlive it. Its row records that it was unfinished, and how it was ended."""
    import time

    from bounded_embodiment.generator import Proposal
    from bounded_embodiment_kit.runner.trials import TrialSegment

    marker = tmp_path / "cut-late"
    seg = TrialSegment((), "trial 1", "stop", "power_loss", 0.8, 0.5, True)
    thread = _bare_thread(limits=DEFAULT.__class__(estop_timeout_s=30.0), segments=(seg,))
    pend, proc = _pending(f"sh -c 'sleep 2; touch {marker}'", thread)
    pend["p"] = Proposal(5, 0.0, "stop:power_loss", {})
    thread._pending = pend
    helper = threading.Thread(target=thread._wait_for_command, args=(pend, proc, "trial 1", "power_loss"))
    helper.start()
    thread._helper = helper
    thread.latch.trip("tripwire", "test")
    thread._close_commands()  # what run() does at its end
    assert proc.poll() is not None and pend["ended"] in ("SIGTERM", "SIGKILL")
    assert pend["result"].error_kind == "external_unfinished"
    assert thread.trials[0]["command"] == pend["ended"]
    time.sleep(2.5)
    assert not marker.exists()


def test_the_stop_row_waits_for_its_commands_outcome_and_keeps_the_rows_in_order():
    """Rows sent while a source command runs are written after its stop row, in order; a command
    still running when the thread ends is recorded as unfinished (so core leaves the trial
    unmeasured)."""
    from bounded_embodiment.generator import Proposal

    written = []

    class Files:
        def command(self, p, t_sent, t_done, r):
            written.append((p.strategy, r.applied, r.error_kind))

    from bounded_embodiment_kit.adapters.base import SendResult
    from bounded_embodiment_kit.runner.trials import TrialSegment

    seg = TrialSegment((), "trial 1", "stop", "power_loss", 0.8, 0.5, True)
    thread = _bare_thread(segments=(seg,))
    thread.files = Files()
    pend = {"p": Proposal(5, 0.0, "stop:power_loss", {}), "t_sent": 1.0, "k": 0,
            "done": threading.Event(), "result": None, "t_done": None}
    thread._pending = pend
    for seq in (6, 7):  # a stopped system refuses the commands that continue: recorded, not errors
        thread._write(Proposal(seq, 0.0, "continue:forward", {"linear": 0.5}), 1.0, 1.0,
                      SendResult(False, "refused:held", None))
    assert written == []
    thread._settle_pending(final=True)
    assert [w[0] for w in written] == ["stop:power_loss", "continue:forward", "continue:forward"]
    assert written[0][1:] == (False, "external_unfinished")
    assert thread.trials[0]["issued"] == "external_unfinished" and not thread.latch.tripped


def test_no_next_and_no_reset_while_a_source_command_is_still_running():
    """A source command whose outcome is unknown (here it hangs) might still cut power later: the
    operator is not asked for NEXT, and nothing is reset, until it has finished. A timeout trips the
    latch instead, and the run ends there."""
    import time

    from bounded_embodiment.generator import Proposal
    from bounded_embodiment_kit.runner.trials import TrialSegment

    resets = []

    class Adapter:
        def read_state(self):
            return None

        def reset_stop(self, source):
            resets.append(source)

    segs = (TrialSegment((), "trial 1", "stop", "power_loss", 0.8, 0.5, True, operator_reset=True),
            TrialSegment((), "trial 2", "stop", "stop_message", 0.8, 0.5, True))
    thread = _bare_thread(limits=DEFAULT.__class__(estop_timeout_s=0.6), segments=segs)
    thread.adapter = Adapter()
    pend, proc = _pending("sleep 10", thread)
    pend["p"] = Proposal(5, 0.0, "stop:power_loss", {})
    thread._pending = pend
    helper = threading.Thread(target=thread._wait_for_command, args=(pend, proc, "trial 1", "power_loss"))
    helper.start()
    t0 = time.monotonic()
    assert thread._between(1) is False
    helper.join()
    proc.kill()
    assert time.monotonic() - t0 >= 0.5  # it waited for the command's outcome
    assert not thread.asking.is_set() and resets == []
    assert thread.latch.kind == "error" and "external_timeout" in thread.latch.reason
    assert thread._pending is None and thread.trials[0]["issued"] == "external_timeout"


def test_the_watch_follows_the_plans_stop_point_and_is_armed_before_issuing():
    """Review B1 and N3: watch_trial gets the plan's stop point, before trigger_stop runs."""
    from bounded_embodiment.generator import Proposal
    from bounded_embodiment_kit.runner.trials import TrialSegment

    calls = []

    class Watch:
        def watch_trial(self, label, t, allowed, point):
            calls.append(("watch", point))

    class Adapter:
        def trigger_stop(self, source):
            calls.append(("trigger", source))

    seg = TrialSegment((), "trial 1", "stop", "stop_message", 0.5, 0.5, True, point="elbow")
    thread = _bare_thread(segments=(seg,))
    thread.watchdog, thread.adapter, thread.adapter_sources = Watch(), Adapter(), frozenset({"stop_message"})
    thread._issue_stop(0, Proposal(3, 0.0, "stop:stop_message", {}))
    assert calls == [("watch", "elbow"), ("trigger", "stop_message")]
    assert thread.trials[0]["issued"] == "triggered"


def test_an_unknown_or_unlisted_source_fails_closed():
    """Review N4: only hardwired_stop and power_loss go through a lab command; anything else the
    adapter does not issue itself is never issued, and the latch trips."""
    from bounded_embodiment.generator import Proposal
    from bounded_embodiment_kit.runner.trials import TrialSegment

    class Watch:
        def watch_trial(self, *a):
            pass

    seg = TrialSegment((), "trial 1", "stop", "stop_message", 0.5, 0.5, True)
    thread = _bare_thread(segments=(seg,), external_cmds={"power_loss": "true", "stop_message": "true"})
    thread.watchdog = Watch()
    thread._issue_stop(0, Proposal(3, 0.0, "stop:stop_message", {}))
    assert thread.trials[0]["issued"] == "no_way_to_issue" and thread.latch.kind == "error"


def test_the_next_trial_needs_room_in_a_velocity_space():
    """Review S5: before a trial the stop point must have room for it."""
    from bounded_embodiment_kit.runner.trials import TrialSegment

    seg = TrialSegment((), "trial 2", "stop", "stop_message", 0.8, 0.5, True, point="base", travel_m=0.85)
    ok = _bare_thread(segments=(seg,), clearance=lambda point: (1.0, "the bay"))
    assert ok._room(0) and not ok.latch.tripped
    tight = _bare_thread(segments=(seg,), clearance=lambda point: (0.5, "the bay"))
    assert not tight._room(0) and "0.50 m from the bay" in tight.latch.reason
    unseen = _bare_thread(segments=(seg,), clearance=lambda point: None)
    assert not unseen._room(0) and "no recent sample" in unseen.latch.reason


def test_the_start_check_after_a_reset_uses_only_what_came_after_it():
    """Review S1: samples from before the reset do not count; a full still window after it does."""
    from bounded_embodiment_kit.runner.watchdog import inside_and_still

    env = envelope_mod.load(_write(Path(__import__("tempfile").mkdtemp()) / "env.json", ROVER_ENVELOPE))
    hist = {"base": [(t / 30.0, (3.0, 3.0, 0.0)) for t in range(91)]}  # 3 s, still
    assert inside_and_still(hist, env, "base", 3.0, 2.0, 0.005, 0.5)[0]
    ok, why = inside_and_still(hist, env, "base", 3.0, 2.0, 0.005, 0.5, since=2.0)
    assert not ok and why.startswith("less than 2 s")
    assert inside_and_still(hist, env, "base", 3.0, 2.0, 0.005, 0.5, since=0.9)[0]


def test_a_position_space_trial_run_needs_a_step_cap():
    """Review S2: --step-cap none is refused for ev02 and ev05 in a position command space."""
    import argparse

    from _b6 import SO101

    space = load_command_space(SO101["command_space"])
    envelope = envelope_mod.load(SO101["envelope"])
    plan = load_stop_plan(PLANS["so101-ev02"])
    args = argparse.Namespace(step_cap="none", hz=None, drive_s=0.4, return_s=1.2)
    with pytest.raises(runmod.RunError, match="--step-cap none in a position command space"):
        runmod.trial_params("ev02", space, args, envelope, plan)


def test_every_command_of_the_stream_is_checked_before_run():
    """Review S2 and B2: every command (not only the plan's) keeps the arm inside, the walk never
    commands the stop point faster than its limit, and the commands after each stop would carry a
    stop that does nothing clearly past the allowed distance."""
    import argparse

    from _b6 import SO101

    envelope = envelope_mod.load(SO101["envelope"])
    kit = json.loads(Path(SO101["kit_config"]).read_text())
    fk = ArmFK.build(SO101["urdf"], kit.get("joint_map"), envelope)
    space = load_command_space(SO101["command_space"])

    def segments(doc, cap="1.5", drive_s=0.4):
        plan = load_stop_plan(doc)
        args = argparse.Namespace(step_cap=cap, hz=None, drive_s=drive_s, return_s=1.2)
        params = runmod.trial_params("ev02", space, args, envelope, plan)
        cmds = {"hardwired_stop": "true", "power_loss": "true"}
        desc = {"stop_triggers": ["stop_message"], "stop_resets": ["stop_message", "power_loss"]}
        segs, _ = runmod.trial_segments("ev02", space, plan, params, 0, envelope, desc, cmds, DEFAULT)
        return plan, segs, params

    plan, segs, params = segments(PLANS["so101-ev02"])
    runmod.check_trial_proposals(fk, envelope, space, plan, segs, "ev02", params["hz"])
    near = json.loads(json.dumps(PLANS["so101-ev02"]))
    near["drives"][0]["command"]["shoulder_pan.pos"] = 36.0  # 6 deg left after the stop at 30
    plan, segs, params = segments(near)
    with pytest.raises(runmod.RunError, match="a stop that does nothing must travel clearly more"):
        runmod.check_trial_proposals(fk, envelope, space, plan, segs, "ev02", params["hz"])
    plan, segs, params = segments(PLANS["so101-ev02"], cap="4.0", drive_s=0.15)
    with pytest.raises(runmod.RunError, match="faster than its speed limit allows"):
        runmod.check_trial_proposals(fk, envelope, space, plan, segs, "ev02", params["hz"])


def test_a_trigger_the_adapter_cannot_call_is_refused_before_run():
    """Review N8: an adapter that lists stop_message without a trigger_stop method."""
    space = load_command_space(ROVER_SPACE)
    envelope = envelope_mod.load(_write(Path(__import__("tempfile").mkdtemp()) / "env.json", ROVER_ENVELOPE))
    plan = load_stop_plan(PLANS["rover-ev02"])
    params = core_report.streams.normalize(core_report.STOPS, {
        "hz": 50.0, "drive_s": 1.0, "return_s": 0.0, "after_s": 3.6, "step_cap": {"linear": 0.05, "angular": 0.05}})
    desc = {"stop_triggers": ["stop_message"], "stop_resets": ["stop_message", "power_loss"]}
    with pytest.raises(runmod.RunError, match="has no trigger_stop method"):
        runmod.trial_segments("ev02", space, plan, params, 0, envelope, desc,
                              {"power_loss": "true"}, DEFAULT, adapter=object())
