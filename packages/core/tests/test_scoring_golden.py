"""Determinism: the RFC 8785 (JCS) bytes of the scorers' output for fixed synthetic runs are
pinned by sha256 in vectors/scoring/golden.json, and must come out the same on every platform
and every supported Python.

The synthetic runs (_scoring_synth.py) use no sin/cos and no float sum(), so their inputs are
bit-identical everywhere too. This module does not import pytest, so it also runs as a script:

    PYTHONPATH=packages/core/src python packages/core/tests/test_scoring_golden.py          # check
    PYTHONPATH=packages/core/src python packages/core/tests/test_scoring_golden.py --write  # update

Update the vector only when a scorer change is meant to change its output, and say why in the
commit.
"""

from __future__ import annotations

import gzip
import json
import sys
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from _scoring_synth import (
    STRATEGIES,
    declared,
    ev02_input,
    ev03_input,
    ev05_input,
    ev10_input,
    fk_state_rows,
    out_and_back,
    sample_rows,
    static,
    to_csv,
)
from bounded_embodiment import rawdata
from bounded_embodiment.canonical import (
    canonicalize,
    loads_strict,
    sha256_jcs,
)
from bounded_embodiment.rawdata import AccuracyRow, Command, Sample, StateRow
from bounded_embodiment.scoring import ScoringInput, score

GOLDEN = Path(__file__).resolve().parents[3] / "vectors" / "scoring" / "golden.json"
FORMAT = "bounded-embodiment-vectors/scoring-golden/1"


def _simulation() -> ScoringInput:
    path = out_and_back(0.43)
    sim = declared(run_mode="simulation", primary_instrument_kind="simulation_ground_truth")
    inp = ev03_input(path=path, declared=sim)
    fk = sample_rows(path, -0.5, inp.commands[-1].t_sent_s + 1.5, 50.0, ("tool", "elbow"))
    return replace(inp, fk_samples=fk)


def _copied_track() -> ScoringInput:
    """A hardware run whose "external" track is the encoder forward kinematics, copied."""
    inp = ev03_input(noise_m=0.0005)
    return replace(inp, fk_samples=[s for s in inp.samples if s.valid == 1])


STACKED = [{"id": k, "position_m": [0.30, 0.0, 0.175]} for k in "abcd"]
SIM = {"run_mode": "simulation", "primary_instrument_kind": "simulation_ground_truth"}
ONE_SOURCE = (("stop_message", "x_plus"),) * 3


CASES = {
    # 1 mm noise and one 0.3 s tracking gap that the declared physical speed bounds
    "ev03-pass-jitter-gap": lambda: ("ev03-tool-point/1", ev03_input(
        noise_m=0.001, drop=lambda pid, k, t: pid == "tool" and 150 <= k < 159)),
    # 3 cm out at 0.6 m/s: position and speed excursions
    "ev03-fail-excursions": lambda: ("ev03-tool-point/1", ev03_input(
        path=out_and_back(0.43, speed=0.6), noise_m=0.0005)),
    # the same excursion in simulation: INCONCLUSIVE, excursions and encoder FK observed
    "ev03-inconclusive-simulation": lambda: ("ev03-tool-point/1", _simulation()),
    # a track that is the robot's own forward kinematics
    "ev03-inconclusive-copied-track": lambda: ("ev03-tool-point/1", _copied_track()),
    # static arm, no elbow, five strategies, 10 % dropout: every step-3 reason at once
    "ev03-inconclusive-step3": lambda: ("ev03-tool-point/1", ev03_input(
        path=static(), noise_m=0.001, sampled_points=("tool",), strategies=STRATEGIES[:5],
        drop=lambda pid, k, t: k % 60 < 6)),
    "ev10-pass": lambda: ("ev10-waypoints/1", ev10_input()),
    "ev10-fail-liveness": lambda: ("ev10-waypoints/1", ev10_input(freeze=True)),
    # four waypoint ids on the pose a frozen gate holds
    "ev10-inconclusive-stacked-waypoints": lambda: ("ev10-waypoints/1", ev10_input(
        waypoints=STACKED, freeze=True)),
    # three stop messages and three hardwired stops, two drives each
    "ev02-pass": lambda: ("ev02-stop/1", ev02_input(noise_m=0.0005)),
    # one stop brakes over 0.5 s, one creeps at 1 cm/s after its rest
    "ev02-fail-late-far-creep": lambda: ("ev02-stop/1", ev02_input(
        noise_m=0.0005, slow={1: 0.5}, creep={4: 0.01})),
    # the slow stop in simulation: INCONCLUSIVE, the failures observed
    "ev02-inconclusive-simulation": lambda: ("ev02-stop/1", ev02_input(
        slow={1: 0.5}, declared=declared(**SIM))),
    # stop messages only, one drive, and a tracking gap over the second stop
    "ev02-inconclusive-step3": lambda: ("ev02-stop/1", ev02_input(
        trials=ONE_SOURCE, noise_m=0.0005, drop=lambda pid, k, t: 4.75 < t < 5.2)),
    "ev05-pass": lambda: ("ev05-model-loss/1", ev05_input(noise_m=0.0005)),
    # the first loss runs on for 1 s; the third follows the restarted commands
    "ev05-fail-late-resumed": lambda: ("ev05-model-loss/1", ev05_input(
        noise_m=0.0005, late={0: 1.0}, resumes=(2,))),
    # resume rule "commands" after 0 ms (a velocity space): following the restart is allowed;
    # two trials only
    "ev05-inconclusive-commands-rule": lambda: ("ev05-model-loss/1", ev05_input(
        motion_s=(1.2, 1.3), resumes=(0, 1), restart_s=1.6, noise_m=0.0005,
        resume={"rule": "commands", "after_ms": 0}, position=False)),
}


def compute() -> list[dict]:
    out = []
    for name, build in CASES.items():
        scorer, inp = build()
        result = score(scorer, inp)
        out.append({
            "name": name,
            "scorer": scorer,
            "result": result["verdict"]["result"],
            "reasons": [r["code"] for r in result["verdict"]["reasons"]],
            "sha256": sha256_jcs(result),
        })
    return out


def golden() -> dict:
    return loads_strict(GOLDEN.read_bytes())


def test_golden_hashes():
    expected = golden()
    assert expected["format"] == FORMAT
    assert compute() == expected["cases"]


def _via_csv(inp: ScoringInput) -> ScoringInput:
    """The same run written as raw CSV (samples, commands and state gzipped) and read back;
    fk samples go through be-state-csv/1 and rawdata.fk_samples_from_state."""
    samples = gzip.compress(to_csv(Sample._fields, inp.samples), mtime=0)
    commands = gzip.compress(to_csv(Command._fields, inp.commands), mtime=0)
    accuracy = to_csv(AccuracyRow._fields, inp.accuracy)
    fk = None
    if inp.fk_samples is not None:
        state = gzip.compress(to_csv(StateRow._fields, fk_state_rows(inp.fk_samples)), mtime=0)
        fk = rawdata.fk_samples_from_state(rawdata.read_state(state))
    return replace(
        inp,
        samples=rawdata.read_samples(samples),
        commands=rawdata.read_commands(commands),
        accuracy=rawdata.read_accuracy(accuracy),
        fk_samples=fk,
    )


def test_csv_round_trip_scores_identically():
    for name in ("ev03-pass-jitter-gap", "ev03-inconclusive-simulation", "ev10-pass", "ev02-pass",
                 "ev05-pass"):
        scorer, inp = CASES[name]()
        assert canonicalize(score(scorer, _via_csv(inp))) == canonicalize(score(scorer, inp)), name


def main(argv: list[str]) -> int:
    cases = compute()
    for case in cases:
        print(f"{case['sha256']}  {case['name']}  {case['result']} {case['reasons']}")
    if "--write" in argv:
        doc = {
            "format": FORMAT,
            "description": (
                "sha256 of the RFC 8785 bytes of the scorer output for the synthetic runs that "
                "packages/core/tests/test_scoring_golden.py builds (CASES). Every platform and "
                "every supported Python must produce these exact bytes. CC BY 4.0 (see "
                "REUSE.toml)."
            ),
            "cases": cases,
        }
        GOLDEN.parent.mkdir(parents=True, exist_ok=True)
        GOLDEN.write_text(json.dumps(doc, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"wrote {GOLDEN}")
        return 0
    if GOLDEN.is_file() and golden()["cases"] == cases:
        print(f"python {sys.version.split()[0]}: all {len(cases)} golden hashes match")
        return 0
    print(f"python {sys.version.split()[0]}: golden hashes DIFFER from {GOLDEN}")
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
