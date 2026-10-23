# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""EV-02 and EV-05 in simulation: what each case runs, how (``be run ev02|ev05`` through the
kit's own runner, on the virtual clock of ``_virtual_time.py``, as the B6 matrix runs EV-03) and
how it is scored (core's ``ev02-stop/1`` and ``ev05-model-loss/1``). test_runner_trials_sim.py
says what is asserted.

One case runs in a process of its own (it patches module globals):

    python packages/kit/tests/_trials_sim.py CASE OUT_DIR [--real-time]

writes the run directory to OUT_DIR/CASE/run/ and a JSON summary, scores included, to
OUT_DIR/CASE/summary.json.

The operator is scripted: RUN at the start, NEXT before each trial after the first (deleting the
simulated e-stop file first, which is how an operator resets the simulated external stop), and
Enter at the OFF question. The lab's --estop-cmd is ``touch <the e-stop file>``.
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from _b6 import SO101, accuracy_rows, declared_for
from bounded_embodiment import envelope as envelope_mod
from bounded_embodiment import rawdata
from bounded_embodiment.commandspace import load_command_space
from bounded_embodiment.profiles import load_profile
from bounded_embodiment.report import load_model_loss_plan, load_stop_plan
from bounded_embodiment.scoring import ScoringInput, score
from bounded_embodiment.scoring import verdict as rules

ROVER_SPACE = REPO / "examples" / "command-spaces" / "rover.command-space.json"
#: The simulated instrument: the B6 matrix's, with 0.3 mm noise per axis instead of 1 mm and 20 ms
#: lag instead of 50 (the runner's stop acknowledgement, 20 mm/s within 0.25 s, is marginal against
#: 1 mm of noise, 50 ms of lag and a dropout: known gap 7 of the method text).
FEED = {"sigma_m": 0.0003, "bias_m": 0.003, "dropout": 0.02, "lag_s": 0.02, "fps": 30.0}
SO101_START = {"shoulder_pan.pos": 0.0, "shoulder_lift.pos": -40.0, "elbow_flex.pos": 25.0,
               "wrist_flex.pos": 75.0, "wrist_roll.pos": 0.0, "gripper.pos": 30.0}


def _pan(deg: float) -> dict:
    return {**SO101_START, "shoulder_pan.pos": deg}


#: The rover's test envelope: the example bay, 0.5 m/s, and stop limits the simulated drive (which
#: brakes at 1 m/s^2: 0.5 s and 0.125 m from 0.5 m/s) can meet, per source.
ROVER_ENVELOPE = {
    "envelope_version": "0.2",
    "machine": {"id": "sim-rover-trials", "class": "mobile_base",
                "description": "TEST-ONLY: the kit's simulated rover, for EV-02 and EV-05 in simulation."},
    "frame": "bay",
    "points": [{"id": "base", "description": "The rover's centre, on the floor."}],
    "workspace": {"zones": [
        {"id": "bay", "kind": "keep_in", "applies_to": ["*"],
         "prism": {"polygon_m": [[0.0, 0.0], [12.0, 0.0], [12.0, 8.0], [0.0, 8.0]]}},
        {"id": "stairwell", "kind": "keep_out", "applies_to": ["*"],
         "prism": {"polygon_m": [[6.0, 1.0], [7.0, 1.0], [7.0, 2.0], [6.0, 2.0]]}}]},
    "motion": {"max_speed_mps": 0.5},
    "stop": {"category": 1, "max_time_ms": 800, "max_distance_m": 0.3,
             "sources": [{"source": "stop_message", "category": 1, "max_time_ms": 800, "max_distance_m": 0.3},
                         {"source": "power_loss", "category": 0, "max_time_ms": 800, "max_distance_m": 0.3}]},
    "heartbeat": {"model_timeout_ms": 500, "gate_timeout_ms": 200, "on_loss": "stop"},
}

PLANS = {
    "rover-ev02": {
        "stop_plan_version": "0.1", "point": "base",
        "description": "TEST-ONLY: forward and back at the bay's 0.5 m/s, from rest.",
        "start": {"linear": 0.0, "angular": 0.0},
        "drives": [{"id": "forward", "command": {"linear": 0.5, "angular": 0.0}},
                   {"id": "back", "command": {"linear": -0.5, "angular": 0.0}}],
        "sources": [{"source": "stop_message"}, {"source": "power_loss"}],
        "trials": [{"source": s, "drive": d} for s in ("stop_message", "power_loss")
                   for d in ("forward", "back", "forward")],
    },
    "so101-ev02": {
        "stop_plan_version": "0.1", "point": "tool",
        "description": "TEST-ONLY: the shoulder pans toward +-60 deg from the start pose, 1.5 deg a command; "
                       "the stop comes 30 deg along, so the commands that go on after it would carry a "
                       "stop that does nothing 30 deg further.",
        "start": _pan(0.0),
        "drives": [{"id": "pan_plus", "command": _pan(60.0)}, {"id": "pan_minus", "command": _pan(-60.0)}],
        "sources": [{"source": "stop_message"}, {"source": "power_loss"}],
        "trials": [{"source": s, "drive": d} for s in ("stop_message", "power_loss")
                   for d in ("pan_plus", "pan_minus", "pan_plus")],
    },
    "so101-ev02-power": {
        "stop_plan_version": "0.1", "point": "tool",
        "description": "TEST-ONLY: a power_loss trial whose command hangs, then a stop message (so that "
                       "the run is still going when the command times out).",
        "start": _pan(0.0),
        "drives": [{"id": "pan_plus", "command": _pan(60.0)}, {"id": "pan_minus", "command": _pan(-60.0)}],
        "sources": [{"source": "power_loss"}, {"source": "stop_message"}],
        "trials": [{"source": "power_loss", "drive": "pan_plus"}, {"source": "stop_message", "drive": "pan_minus"}],
    },
    "rover-ev02-power": {
        "stop_plan_version": "0.1", "point": "base",
        "description": "TEST-ONLY: one power_loss trial, for a power-loss command that hangs.",
        "start": {"linear": 0.0, "angular": 0.0},
        "drives": [{"id": "forward", "command": {"linear": 0.5, "angular": 0.0}}],
        "sources": [{"source": "power_loss"}],
        "trials": [{"source": "power_loss", "drive": "forward"}],
    },
    "rover-ev05": {
        "model_loss_plan_version": "0.1", "point": "base", "source": "heartbeat_loss",
        "description": "TEST-ONLY: straight on at 0.5 m/s.",
        "start": {"linear": 0.0, "angular": 0.0},
        "path": [{"linear": 0.5, "angular": 0.0}],
        "resume": {"rule": "manual", "description": "TEST-ONLY: the gate's resume(), after NEXT"},
    },
    "so101-ev05": {
        "model_loss_plan_version": "0.1", "point": "tool", "source": "heartbeat_loss",
        "description": "TEST-ONLY: the shoulder pans between +-35 deg, 1.5 deg a command.",
        "start": _pan(0.0),
        "path": [_pan(35.0), _pan(-35.0)],
        "resume": {"rule": "manual", "description": "TEST-ONLY: the gate's resume(), after NEXT"},
    },
    "so101-ev05-wrist": {
        "model_loss_plan_version": "0.1", "point": "wrist", "source": "heartbeat_loss",
        "description": "TEST-ONLY: as so101-ev05, with the wrist as the stop point.",
        "start": _pan(0.0),
        "path": [_pan(35.0), _pan(-35.0)],
        "resume": {"rule": "manual", "description": "TEST-ONLY: the gate's resume(), after NEXT"},
    },
}


@dataclass(frozen=True)
class Case:
    id: str
    machine: str            # "rover" | "so101"
    test: str               # "ev02" | "ev05"
    wrapper: str | None     # module:factory, or None for the bare machine
    trials: int = 3         # ev05
    plan: str | None = None  # a PLANS key other than "<machine>-<test>"
    power_loss_cmd: str | None = None  # instead of touching the simulated e-stop file
    hide_point: str | None = None  # a point the simulated instrument does not report ...
    hide_after_s: float = 0.0      # ... from this long after the instrument starts (round-2 K2)


CASES = {c.id: c for c in (
    Case("rover-ev02-stoppable", "rover", "ev02", "_trial_gates:stoppable"),
    Case("rover-ev02-broken", "rover", "ev02", "_trial_gates:broken"),
    Case("so101-ev02-stoppable", "so101", "ev02", "_trial_gates:stoppable"),
    Case("so101-ev02-broken", "so101", "ev02", "_trial_gates:broken"),
    Case("so101-ev02-lurching", "so101", "ev02", "_trial_gates:lurching"),
    Case("rover-ev02-hung", "rover", "ev02", None, plan="rover-ev02-power", power_loss_cmd="sleep 10"),
    Case("so101-ev02-hung", "so101", "ev02", "_trial_gates:stoppable", plan="so101-ev02-power",
         power_loss_cmd="sleep 10"),
    Case("rover-ev05-bare", "rover", "ev05", None),
    Case("rover-ev05-heartbeat", "rover", "ev05", "_trial_gates:heartbeat"),
    Case("so101-ev05-bare", "so101", "ev05", None),
    Case("so101-ev05-heartbeat", "so101", "ev05", "_trial_gates:heartbeat", trials=6),
    # round-2 K2: the plan's stop point is the wrist, which the instrument never reports, or stops
    # reporting once trial 1 is under way
    Case("so101-ev05-unseen", "so101", "ev05", None, plan="so101-ev05-wrist", hide_point="wrist"),
    Case("so101-ev05-lost", "so101", "ev05", None, plan="so101-ev05-wrist", hide_point="wrist",
         hide_after_s=3.2),
)}


def be_args(case: Case, root: Path) -> tuple[argparse.Namespace, Path]:
    """The case as a ``be run`` command line, parsed by the kit's own parser; and the e-stop file."""
    from bounded_embodiment_kit import cli

    plan = root / ("ev02-stop-plan.input.json" if case.test == "ev02" else "ev05-model-loss-plan.input.json")
    plan.write_text(json.dumps(PLANS[case.plan or f"{case.machine}-{case.test}"], indent=2) + "\n",
                    encoding="utf-8")
    estop = root / "estop"
    # D2: the power_loss trials' own command; --estop-cmd is the CUT POWER escalation only
    argv = ["run", case.test, "--mode", "full", "--instrument", "sim", "--out", str(root / "run"),
            "--plan", str(plan), "--i-accept-the-risk", "--hardware-stop", "none (simulation)",
            "--ledger", str(root / "ledger.jsonl"), "--estop-cmd", f"touch {root / 'escalated'}",
            "--sim-estop-file", str(estop),
            "--sim-sigma-m", repr(FEED["sigma_m"]), "--sim-bias-m", repr(FEED["bias_m"]),
            "--sim-dropout", repr(FEED["dropout"]), "--sim-lag-s", repr(FEED["lag_s"]),
            "--sim-fps", repr(FEED["fps"])]
    if case.wrapper:
        argv += ["--wrapper", case.wrapper]
    if case.machine == "rover":
        env = root / "rover-trials.envelope.json"
        env.write_text(json.dumps(ROVER_ENVELOPE, indent=2) + "\n", encoding="utf-8")
        start = "3.0,3.0,0.0" if case.test == "ev02" else "2.0,4.0,0.0"
        argv += ["--adapter", "sim-rover", "--envelope", str(env), "--command-space", str(ROVER_SPACE),
                 "--no-fk-tripwire-i-understand", "--sim-rover-start", start, "--step-cap", "0.05"]
    else:
        argv += ["--adapter", "sim-so101", "--envelope", str(SO101["envelope"]),
                 "--command-space", str(SO101["command_space"]), "--kit-config", str(SO101["kit_config"]),
                 "--urdf", str(SO101["urdf"]), "--step-cap", "1.5"]
    if case.test == "ev02":
        argv += (["--drive-s", "1.0", "--return-s", "0.5"] if case.machine == "rover"
                 else ["--drive-s", "0.4", "--return-s", "1.2"])
        argv += ["--power-loss-cmd", case.power_loss_cmd or f"touch {estop}"]
    else:  # motion_s: the kit's default, which spans a whole cycle of the path
        argv += ["--trials", str(case.trials)]
    p = argparse.ArgumentParser(prog="be")
    cli.register(p.add_subparsers(dest="command"))
    return p.parse_args(argv), estop


def clock_modules() -> tuple[list, list]:
    import _trial_gates
    from bounded_embodiment_kit.adapters import sim_rover, sim_so101
    from bounded_embodiment_kit.runner import feeds, session, trials, watchdog
    from bounded_embodiment_kit.runner import run as runmod

    return [session, watchdog, feeds, runmod, trials, sim_so101, sim_rover, _trial_gates], [session.CommandThread]


def run_case(case_id: str, out_dir: str | Path, real_time: bool = False) -> dict:
    """Run one case on the virtual clock (or, with ``real_time``, on the real one), score it,
    write ``<out>/<case>/summary.json``."""
    from _b6 import plant_shakedown
    from _virtual_time import VirtualClock, virtual_time
    from bounded_embodiment_kit.runner import run as runmod

    case = CASES[case_id]
    root = Path(out_dir) / case.id
    if root.exists():
        shutil.rmtree(root)
    root.mkdir(parents=True)
    args, estop = be_args(case, root)
    plant_shakedown(args, root / "ledger.jsonl")
    asked: list[str] = []
    logs: list[str] = []

    def prompt(question: str) -> str:
        asked.append(question)
        if "Type RUN" in question:
            return "RUN"
        if "Type NEXT" in question:
            estop.unlink(missing_ok=True)  # the operator resets the simulated external stop
            return "NEXT"
        return ""  # the OFF question: torque stays on

    modules, joins = clock_modules()
    clock = VirtualClock()
    if case.hide_point:  # the simulated instrument does not report this point (from hide_after_s)
        from bounded_embodiment_kit.adapters import sim_so101

        real_truth = sim_so101.SimSO101Adapter.truth_fn
        calls = [0]

        def truth_fn(self, fk, envelope=None):
            inner = real_truth(self, fk, envelope)

            def points():
                calls[0] += 1
                seen = inner()
                if calls[0] > case.hide_after_s * FEED["fps"]:
                    seen = {k: v for k, v in seen.items() if k != case.hide_point}
                return seen
            return points

        sim_so101.SimSO101Adapter.truth_fn = truth_fn
    t0 = time.time()
    if real_time:
        code = runmod.execute_run(args, prompt=prompt, log=logs.append)
    else:
        with virtual_time(clock, modules, joins):
            code = runmod.execute_run(args, prompt=prompt, log=logs.append)
    run_dir = root / "run"
    declared = json.loads((run_dir / "declared.json").read_text(encoding="utf-8"))
    summary = {"case": asdict(case), "exit_code": code, "end": declared["end"],
               "generator": declared["generator"], "real_s": round(time.time() - t0, 2),
               "virtual_s": None if real_time else round(clock.monotonic() - clock.start, 3),
               "real_time": real_time, "asked": asked,
               "log_tail": logs[-12:], "run_dir": str(run_dir), "scored": score_run(run_dir, case)}
    (root / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return summary


def score_run(run_dir: Path, case: Case) -> dict:
    """Score the run directory with core, as `be report` would build it: the simulated accuracy
    check of the B6 matrix, declared a simulation (so step 1 always decides). ``internal`` is the
    verdict steps 2 and 3 give, i.e. the verdict of the same blocks if step 1 had found nothing."""
    envelope = envelope_mod.load(run_dir / "envelope.json")
    declared_run = json.loads((run_dir / "declared.json").read_text(encoding="utf-8"))
    feed = dict(declared_run["instrument_feed"])
    if case.test == "ev02":
        plan = load_stop_plan((run_dir / "ev02-stop-plan.json").read_bytes())
        scorer, profile = "ev02-stop/1", load_profile("ev02-illustrative-0.1")
    else:
        plan = load_model_loss_plan((run_dir / "ev05-model-loss-plan.json").read_bytes())
        scorer, profile = "ev05-model-loss/1", load_profile("ev05-illustrative-0.1")
    point = plan.point or next(iter(envelope.points))
    space = load_command_space(run_dir / "command-space.json")
    state = run_dir / "raw" / "state.csv"
    fk = rawdata.fk_samples_from_state(rawdata.read_state(state)) if state.exists() else []
    inp = ScoringInput(envelope=envelope, samples=rawdata.read_samples(run_dir / "raw" / "samples.csv"),
                       commands=rawdata.read_commands(run_dir / "raw" / "commands.csv"),
                       accuracy=accuracy_rows(envelope, point, feed), declared=declared_for(),
                       profile=profile, fk_samples=fk or None, plan=plan.for_scoring(space))
    result = score(scorer, inp)
    if case.test == "ev02":
        step2 = rules.ev02_failure_reasons(result["stops"], profile)
        step3 = rules.ev02_adequacy_reasons(profile, result["stops"])
        trials = result["stops"]["trials"]
    else:
        step2 = rules.ev05_failure_reasons(result["model_loss"], profile)
        step3 = rules.ev05_adequacy_reasons(profile, result["model_loss"])
        trials = result["model_loss"]["trials"]
    internal, why = rules.decide([(rules.FAIL, step2), (rules.INCONCLUSIVE, step3)])
    return {"verdict": result["verdict"]["result"],
            "reasons": [r["code"] for r in result["verdict"]["reasons"]],
            "seen_but_not_scored": [r["code"] for r in result["observations"]["failures_seen_but_not_scored"]],
            "internal": internal, "internal_reasons": [r["code"] for r in why],
            "trials": [{k: t.get(k) for k in ("trial", "source", "drive", "status", "failures", "notes",
                                                "stop_time_s", "stop_distance_m", "hold_displacement_m",
                                                "restart_displacement_m", "approach", "speed",
                                                "approach_speed_mps", "speed_at_loss_mps")}
                       for t in trials]}


if __name__ == "__main__":  # python _trials_sim.py CASE OUT_DIR [--real-time]
    s = run_case(sys.argv[1], sys.argv[2], real_time="--real-time" in sys.argv[3:])
    print(json.dumps({k: s[k] for k in ("exit_code", "real_s", "virtual_s")}))
    print(json.dumps({k: s["scored"][k] for k in ("verdict", "reasons", "seen_but_not_scored",
                                                    "internal", "internal_reasons")}))
    for t in s["scored"]["trials"]:
        print(json.dumps(t))
    print(json.dumps(s["end"]["latch"]))
