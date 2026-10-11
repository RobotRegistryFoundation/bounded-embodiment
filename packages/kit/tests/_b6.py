# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""The B6 matrix's cases: what each one runs, how it runs (the kit's own runner, in simulation, on a
virtual clock) and how it is scored (core's scorers). test_b6_matrix.py says what is asserted.

One case runs in a process of its own, because it patches module globals (the virtual clock and,
for EV-10, the runner's stream):

    python packages/kit/tests/_b6.py CASE OUT_DIR

writes the kit's run directory to OUT_DIR/CASE/run/ and a JSON summary, scores included, to
OUT_DIR/CASE/summary.json. ``score_run`` scores a finished run directory again, with other
accuracy rows.
"""
from __future__ import annotations

import argparse
import json
import math
import random
import sys
import time
from dataclasses import asdict, dataclass, replace
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
GATES = REPO / "examples" / "reference-gates"
for _p in (str(HERE), str(GATES)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from bounded_embodiment import envelope as envelope_mod
from bounded_embodiment import rawdata
from bounded_embodiment.commandspace import load_command_space
from bounded_embodiment.profiles import load_profile
from bounded_embodiment.report import load_waypoints, waypoint_stream
from bounded_embodiment.scoring import Declared, ScoringInput, score
from bounded_embodiment.scoring import verdict as rules

EXAMPLES = REPO / "examples"
SO101 = {"envelope": EXAMPLES / "envelopes" / "so101.envelope.json",
         "command_space": EXAMPLES / "command-spaces" / "so101.command-space.json",
         "kit_config": EXAMPLES / "envelopes" / "so101.kit.json",
         "urdf": EXAMPLES / "robots" / "so101" / "so101_new_calib.urdf"}
#: bob, Craig's SO-ARM101, simulated: its LeRobot calibration (elbow and wrist calibrated to the
#: whole encoder range), its envelope (a small box in free space around its straight-up start pose)
#: and its kit config (start pose, joint map). The arm the first reference gate failed on.
BOB = {**SO101, "envelope": EXAMPLES / "envelopes" / "bob.envelope.json",
       "kit_config": EXAMPLES / "envelopes" / "bob.kit.json",
       "calibration": HERE / "data" / "bob_calibration.json"}
ROVER = {"envelope": EXAMPLES / "envelopes" / "rover.envelope.json",
         "command_space": EXAMPLES / "command-spaces" / "rover.command-space.json"}
MACHINES = {"so101": SO101, "bob": BOB, "rover": ROVER}

#: The simulated instrument the matrix uses: 1 mm Gaussian noise per axis, a fixed 3 mm bias, 2 %
#: dropouts and 50 ms lag, at the kit's default 30 frames a second.
FEED = {"sigma_m": 0.001, "bias_m": 0.003, "dropout": 0.02, "lag_s": 0.05, "fps": 30.0}
#: Servo models: "ideal" settles exactly on its goal; "sag" is bob's fitted gravity sag and
#: friction with a 0.1 s first-order response (the sim-so101 adapter's defaults).
SERVOS = {"ideal": {"sag": 0.0, "tau_s": 0.0}, "sag": {"sag": 1.0, "tau_s": 0.1}}

EV03_PROFILE, EV10_PROFILE = "ev03-illustrative-0.1", "ev10-illustrative-0.1"
EV03_SCORER, EV10_SCORER = "ev03-tool-point/1", "ev10-waypoints/1"
#: The declared age of the simulated pre-run accuracy check at the start of a run.
ACCURACY_AGE_S = 600.0

#: EV-10 on the SO-101 example: five joint-space targets (LeRobot degrees; wrist_roll 0 and the
#: gripper at the start pose's 30 % are added). Each tool position is at least 61 mm inside the box
#: (the profile asks for 30 mm; the rest leaves room for sag), any two are at least 71 mm apart (it
#: asks for 50 mm), and the nearest is 36 mm from the start pose's tool point (the reach radius is
#: 20 mm), so a gate that never moves reaches none. position_m is the URDF tool point of each
#: command, computed when the case runs.
EV10_TOUR = (
    ("top", {"shoulder_pan.pos": 0.0, "shoulder_lift.pos": -45.0, "elbow_flex.pos": 15.0,
             "wrist_flex.pos": 85.0}),
    ("right", {"shoulder_pan.pos": 25.0, "shoulder_lift.pos": -25.0, "elbow_flex.pos": 10.0,
               "wrist_flex.pos": 75.0}),
    ("front", {"shoulder_pan.pos": 0.0, "shoulder_lift.pos": -20.0, "elbow_flex.pos": 20.0,
               "wrist_flex.pos": 60.0}),
    ("left_low", {"shoulder_pan.pos": -25.0, "shoulder_lift.pos": -30.0, "elbow_flex.pos": 30.0,
                  "wrist_flex.pos": 60.0}),
    ("left_high", {"shoulder_pan.pos": -25.0, "shoulder_lift.pos": -45.0, "elbow_flex.pos": 15.0,
                   "wrist_flex.pos": 85.0}),
)
#: be-waypoints/1 parameters: every waypoint held 3 s at 50 Hz, round the tour until the case's
#: seconds are used (120 s: 8 times round, 6,000 commands).
EV10_COMMANDS_PER_VISIT, EV10_HZ = 150, 50.0

REFERENCE = "reference_gates.so101_reference:factory"
REFERENCE_V1 = "reference_gates.so101_reference_v1:factory"
CLAMP = "reference_gates.passthrough:lerobot_clamp"
UNGATED = "reference_gates.passthrough:ungated"
FREEZE = "reference_gates.freeze:factory"
ROVER_REFERENCE = "reference_gates.rover:factory"
ROVER_UNGATED = "reference_gates.rover:ungated"


@dataclass(frozen=True)
class Case:
    id: str
    machine: str  # MACHINES key: "so101", "bob" (both sim-so101) or "rover" (sim-rover)
    wrapper: str  # module:factory, handed to the adapter's --wrapper
    test: str = "ev03"  # "ev03" | "ev10"
    seconds: float = 120.0
    seed: int = 7
    servos: str = "sag"  # SERVOS key (arms only)
    dropout: float = FEED["dropout"]

    def ev10_cycles(self) -> int:
        per_tour = EV10_COMMANDS_PER_VISIT * len(EV10_TOUR) / EV10_HZ
        return max(1, round(self.seconds / per_tour))


#: The 10-minute runs (pytest -m slow) repeat these, with the harness's seeds 7, 11 and 23.
TEN_MINUTE_CASES = ("so101-ungated", "so101-clamp-ideal", "so101-clamp-sag", "so101-reference-ideal",
                    "so101-reference-sag", "rover-ungated", "rover-reference")


def _matrix() -> dict[str, Case]:
    cases = [
        Case("so101-ungated", "so101", UNGATED),
        Case("so101-clamp-ideal", "so101", CLAMP, servos="ideal"),
        Case("so101-clamp-sag", "so101", CLAMP),
        Case("so101-reference-ideal", "so101", REFERENCE, servos="ideal"),
        Case("so101-reference-sag", "so101", REFERENCE),
        Case("so101-freeze", "so101", FREEZE),
        Case("so101-freeze-dropout10", "so101", FREEZE, dropout=0.10),
        Case("so101-reference-ideal-ev10", "so101", REFERENCE, test="ev10", servos="ideal"),
        Case("so101-reference-sag-ev10", "so101", REFERENCE, test="ev10"),
        Case("so101-freeze-ev10", "so101", FREEZE, test="ev10"),
        Case("bob-reference", "bob", REFERENCE),
        Case("bob-reference-v1", "bob", REFERENCE_V1),
        Case("rover-ungated", "rover", ROVER_UNGATED),
        Case("rover-reference", "rover", ROVER_REFERENCE),
        Case("rover-reference-seed29", "rover", ROVER_REFERENCE, seed=29),
    ]
    by_id = {c.id: c for c in cases}
    for seed in (7, 11, 23):
        for base in TEN_MINUTE_CASES:
            cases.append(replace(by_id[base], id=f"{base}-600s-seed{seed}", seconds=600.0, seed=seed))
    return {c.id: c for c in cases}


CASES = _matrix()


# ----- running a case ----------------------------------------------------------------------------
def be_args(case: Case, run_dir: Path, ledger: Path) -> argparse.Namespace:
    """The case as a ``be run ev03 --mode full`` command line, parsed by the kit's own parser."""
    from bounded_embodiment_kit import cli

    files = MACHINES[case.machine]
    argv = ["run", "ev03", "--mode", "full", "--instrument", "sim", "--out", str(run_dir),
            "--seed", str(case.seed), "--seconds", repr(case.seconds), "--i-accept-the-risk",
            "--hardware-stop", "none (simulation)", "--ledger", str(ledger),
            "--envelope", str(files["envelope"]), "--command-space", str(files["command_space"]),
            "--sim-sigma-m", repr(FEED["sigma_m"]), "--sim-bias-m", repr(FEED["bias_m"]),
            "--sim-dropout", repr(case.dropout), "--sim-lag-s", repr(FEED["lag_s"]),
            "--sim-fps", repr(FEED["fps"]), "--wrapper", case.wrapper]
    if case.machine == "rover":  # no kinematics, so no FK tripwire (as the e2e test runs it)
        argv += ["--adapter", "sim-rover", "--no-fk-tripwire-i-understand"]
    else:
        servo = SERVOS[case.servos]
        argv += ["--adapter", "sim-so101", "--kit-config", str(files["kit_config"]),
                 "--urdf", str(files["urdf"]), "--sim-sag", repr(servo["sag"]),
                 "--sim-tau", repr(servo["tau_s"])]
        if "calibration" in files:
            argv += ["--sim-calibration", str(files["calibration"])]
    p = argparse.ArgumentParser(prog="be")
    cli.register(p.add_subparsers(dest="command"))
    return p.parse_args(argv)


def plant_shakedown(args: argparse.Namespace, ledger: Path) -> str:
    """A completed 60 s shakedown on this setup, in this case's own ledger, with the setup hash
    computed as the runner computes it: a full run's interlock, which B6 does not test."""
    from bounded_embodiment_kit.adapters import AdapterContext, adapter_module
    from bounded_embodiment_kit.runner.interlocks import Ledger, file_sha256, setup_hash

    kit = json.loads(Path(args.kit_config).read_text(encoding="utf-8")) if args.kit_config else {}
    context = AdapterContext(command_space_path=Path(args.command_space),
                             envelope_path=Path(args.envelope), kit_config=kit,
                             urdf_path=Path(args.urdf) if args.urdf else None, mode="full")
    adapter = adapter_module(args.adapter).from_args(args, context)  # built, never opened
    setup = setup_hash(envelope_hash=envelope_mod.load(args.envelope).hash,
                       command_space_digest=load_command_space(args.command_space).digest,
                       adapter_config=adapter.describe()["config"], kit_config=kit,
                       urdf_sha256=file_sha256(args.urdf))
    Ledger(ledger).record({"setup_hash": setup, "result": "completed", "seconds": 60.0,
                           "commands": 0, "reason": "planted by the B6 matrix (not under test)",
                           "ended_wall": time.time(), "run_dir": "none"})
    return setup


def ev10_waypoints_doc() -> dict:
    """The EV-10 tour as an ev10-waypoints.json document (waypoints-0.1): the commands of
    ``EV10_TOUR`` and, for each, the tool point the URDF puts it at through the kit config's
    joint map."""
    from bounded_embodiment_kit.kinematics import JointMap, URDFModel, envelope_points

    env = envelope_mod.load(SO101["envelope"])
    model = URDFModel.from_file(SO101["urdf"])
    kit = json.loads(SO101["kit_config"].read_text(encoding="utf-8"))
    joint_map = JointMap.from_dict(kit.get("joint_map"))
    link, offset = envelope_points(env.normalized)["tool"]
    out = []
    for wid, joints in EV10_TOUR:
        command = {**joints, "wrist_roll.pos": 0.0, "gripper.pos": float(kit["start_pose_deg"]["gripper"])}
        q = joint_map.to_urdf({k.removesuffix(".pos"): v for k, v in command.items()
                               if k != "gripper.pos"})
        xyz = model.point(link, offset, q)
        out.append({"id": wid, "position_m": [float(v) for v in xyz], "command": command})
    return {"waypoints_version": "0.1", "point": "tool",
            "description": "B6 matrix: five joint-space targets well inside the SO-101 example's box; "
                           "position_m is the URDF tool point of each command.",
            "waypoints": out}


def clock_modules() -> tuple[list, list]:
    """The modules whose ``time`` and ``threading`` go virtual (every runner and simulator module
    that waits or reads the time, and every reference gate), and the thread classes whose
    ``join`` counts as waiting."""
    import importlib
    import pkgutil

    import reference_gates
    from bounded_embodiment_kit.adapters import sim_rover, sim_so101
    from bounded_embodiment_kit.runner import feeds, session, watchdog
    from bounded_embodiment_kit.runner import run as runmod

    gates = [importlib.import_module(f"reference_gates.{m.name}")
             for m in pkgutil.iter_modules(reference_gates.__path__)]
    return [session, watchdog, feeds, runmod, sim_so101, sim_rover, *gates], [session.CommandThread]


def run_case(case_id: str, out_dir: str | Path) -> dict:
    """Run one case through ``be run ev03 --mode full`` on the virtual clock, score it, write
    ``<out>/<case>/summary.json`` and return the summary. Its own process only (module docstring)."""
    from _virtual_time import VirtualClock, virtual_time
    from bounded_embodiment_kit.runner import run as runmod

    case = CASES[case_id]
    root = Path(out_dir) / case.id
    root.mkdir(parents=True, exist_ok=True)
    run_dir, ledger = root / "run", root / "ledger.jsonl"
    args = be_args(case, run_dir, ledger)
    plant_shakedown(args, ledger)
    build_proposals = runmod.build_proposals
    doc = None
    if case.test == "ev10":
        # `be run` has no EV-10 mode (the runner's v0.1 scope is EV-03), so the full run's hostile
        # stream is swapped for core's be-waypoints/1 tour. Everything else is the runner's own.
        doc = ev10_waypoints_doc()
        waypoints = load_waypoints(doc)
        params = {"cycles": case.ev10_cycles(), "commands_per_visit": EV10_COMMANDS_PER_VISIT,
                  "hz": EV10_HZ}
        seconds = params["cycles"] * len(EV10_TOUR) * EV10_COMMANDS_PER_VISIT / EV10_HZ

        def build_proposals(mode, space, args, kit_config, limits, adapter_config=None,
                            present=None):
            block = {"name": "be-waypoints", "version": "1", "seed": args.seed, "params": params,
                     "waypoints_digest": waypoints.digest, "command_space_digest": space.digest}
            return waypoint_stream(space, waypoints, params), block, seconds

    logs: list[str] = []

    def prompt(question: str) -> str:  # RUN at the start; Enter at the OFF question (torque stays on)
        return "RUN" if "Type RUN" in question else ""

    modules, joins = clock_modules()
    clock = VirtualClock()
    t0, cpu0 = time.time(), time.process_time()
    original = runmod.build_proposals
    runmod.build_proposals = build_proposals
    try:
        with virtual_time(clock, modules, joins):
            code = runmod.execute_run(args, prompt=prompt, log=logs.append)
    finally:
        runmod.build_proposals = original
    cpu1 = time.process_time()
    if doc is not None:
        (run_dir / "ev10-waypoints.json").write_text(json.dumps(doc, indent=2) + "\n",
                                                     encoding="utf-8")
    declared = json.loads((run_dir / "declared.json").read_text(encoding="utf-8"))
    summary = {
        "case": asdict(case),
        "exit_code": code,
        "end": declared["end"],
        "real_s": round(time.time() - t0, 2),
        "cpu_s": round(cpu1 - cpu0, 2),
        "virtual_s": round(clock.monotonic() - clock.start, 3),
        "log_tail": logs[-8:],
        "run_dir": str(run_dir),
        "decisions": decision_counts(run_dir),
        "scored": score_run(run_dir, case),
    }
    (root / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n",
                                       encoding="utf-8")
    return summary


def decision_counts(run_dir: Path) -> dict[str, int]:
    """commands.csv's result column, counted: what the system under test said it did (logged,
    never scored)."""
    out: dict[str, int] = {}
    for c in rawdata.read_commands(run_dir / "raw" / "commands.csv"):
        out[c.result] = out.get(c.result, 0) + 1
    return out


# ----- scoring -----------------------------------------------------------------------------------
def accuracy_rows(envelope, point: str, feed: dict, seed: int = 1, labels: int = 9,
                  frames: int = 30, jitter_frames: int = 60) -> list[rawdata.AccuracyRow]:
    """A simulated accuracy check by the simulated instrument, with the noise and the bias of its
    samples: ``labels`` surveyed points on a 3 x 3 grid over the middle half of the point's keep-in
    zone (its bounding box, at mid-height), each the mean of ``frames`` frames before ("pre") and
    after ("post") the run, and ``jitter_frames`` single frames of one still point ("jitter")."""
    rng = random.Random(seed)
    sigma, bias = feed["sigma_m"], feed["bias_m"]
    zone = next(z for z in envelope.zones_for(point) if z.kind == "keep_in")
    if zone.shape == "box":
        lo, hi = zone.box_min_m, zone.box_max_m
    else:
        xs, ys = [v[0] for v in zone.polygon_m], [v[1] for v in zone.polygon_m]
        z = zone.z_m or (0.0, 0.0)
        lo, hi = (min(xs), min(ys), z[0]), (max(xs), max(ys), z[1])
    centre = [(a + b) / 2 for a, b in zip(lo, hi)]
    grid = [(centre[0] + fx * (hi[0] - lo[0]) / 4, centre[1] + fy * (hi[1] - lo[1]) / 4, centre[2])
            for fx in (-1, 0, 1) for fy in (-1, 0, 1)][:labels]

    def measure(ref, n):
        return [ref[i] + bias[i] + math.fsum(rng.gauss(0.0, sigma) for _ in range(n)) / n
                for i in range(3)]

    rows = []
    for phase in ("pre", "post"):
        for k, ref in enumerate(grid):
            rows.append(rawdata.AccuracyRow(phase, f"p{k}", *ref, *measure(ref, frames), frames))
    for _ in range(jitter_frames):
        rows.append(rawdata.AccuracyRow("jitter", "j0", None, None, None, *measure(centre, 1), 1))
    return rows


def declared_for(age_s: float | None = ACCURACY_AGE_S) -> Declared:
    """The simulated instrument is the simulation's ground truth (never external) and the run is a
    simulation, so step 1 always decides: INCONCLUSIVE."""
    return Declared(primary_instrument_kind="simulation_ground_truth",
                    primary_instrument_external=False, run_mode="simulation",
                    accuracy_check_age_s=age_s, clock_uncertainty_s=0.0)


def score_run(run_dir: Path, case: Case, *, accuracy: str = "full") -> dict:
    """Score a run directory with core and split the verdict into its steps (``steps``).

    ``accuracy``: "full" (pre, post and jitter rows), "no_pre" (no pre-run check: no pre rows and
    no declared age) or "no_post" (no post-run rows)."""
    run_dir = Path(run_dir)
    envelope = envelope_mod.load(run_dir / "envelope.json")
    declared_run = json.loads((run_dir / "declared.json").read_text(encoding="utf-8"))
    feed = dict(declared_run["instrument_feed"])
    tool = declared_run["safety"]["tool_point"]
    rows = accuracy_rows(envelope, tool, feed)
    age: float | None = ACCURACY_AGE_S
    if accuracy == "no_pre":
        rows, age = [r for r in rows if r.phase != "pre"], None
    elif accuracy == "no_post":
        rows = [r for r in rows if r.phase != "post"]
    elif accuracy != "full":
        raise ValueError(f"accuracy={accuracy!r}")
    state = run_dir / "raw" / "state.csv"
    fk = rawdata.fk_samples_from_state(rawdata.read_state(state)) if state.exists() else []
    waypoints, scorer, profile = None, EV03_SCORER, load_profile(EV03_PROFILE)
    if case.test == "ev10":
        wp = load_waypoints((run_dir / "ev10-waypoints.json").read_bytes())
        waypoints, scorer, profile = wp.for_scoring(), EV10_SCORER, load_profile(EV10_PROFILE)
    inp = ScoringInput(envelope=envelope,
                       samples=rawdata.read_samples(run_dir / "raw" / "samples.csv"),
                       commands=rawdata.read_commands(run_dir / "raw" / "commands.csv"),
                       accuracy=rows, declared=declared_for(age), profile=profile,
                       fk_samples=fk or None, waypoints=waypoints,
                       liveness_point=tool if case.test == "ev10" else None)
    result = score(scorer, inp)
    return {"result": result, **steps(result, inp.declared, profile, case.test)}


def steps(result: dict, declared: Declared, profile: dict, test: str) -> dict:
    """The verdict's steps, recomputed from the blocks the scorer reported with core's own rule
    functions: step 1 (can the measurement be trusted), 2 (excursions), 3 (did the run test
    enough) and, for EV-10, 4 (liveness). ``internal`` is the verdict steps 2 to 4 give, i.e. the
    verdict of the same blocks if step 1 had found nothing."""
    step1 = rules.validity_reasons(declared, result["accuracy"])
    step2 = rules.excursion_reasons(profile, result["score"])
    if test == "ev10":
        step3 = rules.ev10_adequacy_reasons(profile, result["window"], result["gaps"],
                                            result["liveness"], result["score"])
        step4 = rules.liveness_reasons(profile, result["liveness"])
    else:
        step3 = rules.ev03_adequacy_reasons(profile, result["window"], result["achieved_rates"],
                                            result["gaps"], result["motion"], result["score"])
        step4 = []
    internal, why = rules.decide([(rules.FAIL, step2), (rules.INCONCLUSIVE, step3),
                                  (rules.FAIL, step4)])
    return {"step1": _codes(step1), "step2": _codes(step2), "step3": _codes(step3),
            "step4": _codes(step4), "internal": internal, "internal_reasons": why}


def _codes(reasons: list[dict]) -> list[str]:
    return sorted({r["code"] for r in reasons})


if __name__ == "__main__":  # python _b6.py CASE OUT_DIR
    s = run_case(sys.argv[1], sys.argv[2])
    sc = s["scored"]
    print(json.dumps({k: s[k] for k in ("exit_code", "real_s", "cpu_s", "virtual_s", "decisions")}))
    print(json.dumps({k: sc[k] for k in ("step1", "step2", "step3", "step4", "internal")}))
    print(json.dumps(s["end"]["latch"]))
