# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""Stand-ins for the `be report` and `be submit` tests.

``runner_run`` turns one of core's golden runs (vectors/runs/, TEST-ONLY synthetic data in the
real formats) into a run directory as `be run` leaves it: declared.json, the setup files and the
robot's raw files, with the instrument's samples set aside on a clock of the test's choosing.
``fake_results_repo`` is a results repository with stand-in tools; ``real_results_repo`` finds a
clone of the real one (BE_RESULTS_REPO, or a sibling checkout) for the end-to-end test.
"""
from __future__ import annotations

import argparse
import calendar
import json
import os
import shutil
import time
from decimal import Decimal
from pathlib import Path

from bounded_embodiment import rawdata, signing
from bounded_embodiment_kit import cli

REPO = Path(__file__).resolve().parents[3]
RUNS = REPO / "vectors" / "runs"
GOLDEN_STARTED = "2026-10-10T18:00:00Z"
MONO_START = 5000.25
MONO_START_TEXT = "5000.25"


def be(*argv):
    """`be` with the kit's commands, as the entry point registers them."""
    p = argparse.ArgumentParser(prog="be")
    cli.register(p.add_subparsers(dest="command"))
    return p.parse_args([str(a) for a in argv])


def epoch(utc: str) -> float:
    return float(calendar.timegm(time.strptime(utc, "%Y-%m-%dT%H:%M:%SZ")))


# ----- keys ----------------------------------------------------------------------------------------
def make_key(path: Path, lab: str = "test-lab") -> tuple[Path, dict]:
    """A key file as `be keygen` writes it ({kid, kty, crv, x, d}, mode 0600) and its key-0.1
    entry."""
    private, public = signing.generate_key()
    entry = signing.key_entry(public, lab, "lab@example.org", "2026-10-01")
    path.write_text(json.dumps({"kid": entry["kid"], **private}, indent=2) + "\n")
    os.chmod(path, 0o600)
    return path, entry


# ----- a runner's run directory from a golden run --------------------------------------------------
def golden_report(name: str) -> dict:
    return json.loads((RUNS / name / "report.json").read_text())


def declared_json(name: str, *, simulation: bool = False, run_mode: str = "full",
                  generator: dict | None = None, timestamps: dict | None = None, **extra) -> dict:
    """declared.json for a golden run, in the shape runner/run.py writes."""
    rep = golden_report(name)
    gen = rep["generator"]
    if generator is None:
        generator = {"name": gen["name"], "version": gen["version"], "seed": gen["seed"],
                     "params": gen["params"],
                     "command_space_digest": rep["adapter"]["command_space_sha256"]}
    start = epoch(rep["run"]["started_at"])
    end = epoch(rep["run"]["ended_at"])
    doc = {
        "format": "be-run-declared/0.1", "test": rep["test"].replace("-", "").lower(),
        "run_mode": run_mode, "simulation": simulation,
        "adapter": {"adapter": "python", "simulation": simulation,
                    "config": {"adapter": "python"}},
        "generator": generator, "planned_seconds": 3.0,
        "command_space": {"path": "command-space.json",
                          "digest": rep["adapter"]["command_space_sha256"]},
        "envelope": {"path": "envelope.json", "hash": rep["envelope"]["hash"]},
        "kit_config": {"path": None, "sha256": None, "document": {}},
        "urdf": None,
        "instrument_feed": {"kind": "udp", "host": "0.0.0.0", "port": 5005},
        "safety": {"hardware_stop": "a relay on servo power, switch by the bench",
                   "fk_tripwire": "override", "estop_cmd": None, "tool_point": "tool"},
        "timestamps": timestamps if timestamps is not None else {
            "started_at": rep["run"]["started_at"].replace("Z", "+00:00"),
            "ended_at": rep["run"]["ended_at"].replace("Z", "+00:00"),
            "timebase": "time.monotonic() at run start = 0 s",
            "monotonic_start_s": MONO_START, "wall_start_s": start,
            "wall_start_uncertainty_s": 2e-07, "monotonic_clock": "clock_gettime(CLOCK_MONOTONIC)",
            "monotonic_resolution_s": 1e-09, "monotonic_end_s": MONO_START + (end - start),
            "wall_end_s": end, "wall_end_uncertainty_s": 2e-07},
        "end": {"status": "ended", "why": "the command stream ended", "started": True},
        "software": {"kit": {"name": "bounded-embodiment-kit", "version": "0.1.0.dev0"},
                     "core": {"name": "bounded-embodiment", "version": "0.1.0.dev0"},
                     "python": "3.12.0", "platform": "Linux"},
    }
    doc.update(extra)
    return doc


def shift_samples(data: bytes, offset: str) -> bytes:
    """be-samples-csv/1 with offset (decimal text) added to every t_s, exactly."""
    lines = data.decode().splitlines()
    out = [lines[0]]
    for line in lines[1:]:
        t, rest = line.split(",", 1)
        out.append(f"{Decimal(t) + Decimal(offset)},{rest}")
    return ("\n".join(out) + "\n").encode()


def runner_run(tmp: Path, name: str = "ev03-pass", *, declared: dict | None = None,
               samples_offset: str = MONO_START_TEXT, **declared_kw) -> Path:
    """A run directory as `be run` leaves it, from golden run ``name``; the golden samples are
    written to ``tmp/instrument/samples.csv`` with ``samples_offset`` added (default: this run's
    host CLOCK_MONOTONIC), and its accuracy rows to ``tmp/instrument/accuracy.csv``."""
    src = RUNS / name
    run = tmp / "run"
    (run / "raw").mkdir(parents=True)
    for rel in ("envelope.json", "command-space.json", "ev10-waypoints.json", "raw/commands.csv",
                "raw/state.csv"):
        if (src / rel).is_file():
            shutil.copyfile(src / rel, run / rel)
    doc = declared if declared is not None else declared_json(name, **declared_kw)
    (run / "declared.json").write_text(json.dumps(doc, indent=2, sort_keys=True) + "\n")
    inst = tmp / "instrument"
    inst.mkdir(exist_ok=True)
    (inst / "samples.csv").write_bytes(shift_samples((src / "raw" / "samples.csv").read_bytes(),
                                                     samples_offset))
    shutil.copyfile(src / "raw" / "accuracy.csv", inst / "accuracy.csv")
    if (src / "raw" / "instrument").is_dir():
        for f in (src / "raw" / "instrument").iterdir():
            shutil.copyfile(f, inst / f.name)
    return run


def lab_config(name: str = "ev03-pass", **overrides) -> dict:
    """A lab config with the golden run's declared fields."""
    rep = golden_report(name)
    doc = {"format": "be-lab-config/0.1",
           "machine": rep["system_under_test"]["machine"],
           "gate": rep["system_under_test"]["gate"],
           "instruments": rep["instruments"],
           "signer": {"name": "Test Signer", "organization": "TEST-ONLY lab", "role": "test"},
           "operator": "lab technician",
           "method": rep["method"]}
    doc.update(overrides)
    return doc


def write_json(path: Path, doc) -> Path:
    path.write_text(json.dumps(doc, indent=2) + "\n")
    return path


def report_args(tmp: Path, run: Path, key: Path, out: Path | None = None, *, clock="monotonic",
                lab: dict | None = None, extra=()) -> argparse.Namespace:
    """`be report` for a runner_run directory: the instrument's samples and accuracy rows, and
    the golden calibration file when the lab config lists one."""
    config = write_json(tmp / "lab-config.json", lab if lab is not None else lab_config())
    argv = ["report", run, "--lab-config", config, "--key", key, "--out", out or tmp / "submission",
            "--samples", tmp / "instrument" / "samples.csv", "--accuracy",
            tmp / "instrument" / "accuracy.csv"]
    if clock:
        argv += ["--clock", clock]
    if (tmp / "instrument" / "calibration.json").is_file():
        argv += ["--instrument-file", tmp / "instrument" / "calibration.json"]
    return be(*argv, *extra)


def read_samples_gz(path: Path) -> list:
    return rawdata.read_samples(path)


# ----- results repositories ------------------------------------------------------------------------
GUARD = '''import sys
MAX_FILE_BYTES = {file_bytes}
MAX_RUN_BYTES = {run_bytes}
paths = [p for p in sys.stdin.read().split() if p]
bad = [p for p in paths if not p.startswith("results/")]
if bad or {reject}:
    print("path-guard: rejected.")
    sys.exit(1)
print(f"path-guard: accepted. {{len(paths)}} paths")
'''
LINT = '''import re
import sys
from pathlib import Path
hits = [p for p in Path(sys.argv[1]).rglob("NOTES.md") if re.search(r"\\bsafe\\b", p.read_text(), re.I)]
if hits:
    print("claim-lint: 1 claim word(s) found")
    sys.exit(1)
print("claim-lint: no claim words in 1 run(s).")
'''
PIN = '''import sys
print("pin-core: refused" if {refuse} else "0.1.0")
sys.exit(1 if {refuse} else 0)
'''


def fake_results_repo(tmp: Path, lab: str, entries: list[dict], *, file_bytes: str = "20 * 1024 * 1024",
                      run_bytes: str = "50 * 1024 * 1024", reject: bool = False,
                      kits: dict | None = None, guard: bool = True, pin_refuses: bool = False) -> Path:
    """A results repository with stand-in tools: path_guard (limits as given; accepts results/
    paths unless ``reject``), claim_lint (flags the word "safe" in NOTES.md) and pin_core
    (prints 0.1.0, or refuses)."""
    repo = tmp / "results-repo"
    (repo / "tools").mkdir(parents=True)
    (repo / "results").mkdir()
    (repo / "labs" / lab).mkdir(parents=True)
    if guard:
        (repo / "tools" / "path_guard.py").write_text(
            GUARD.format(file_bytes=file_bytes, run_bytes=run_bytes, reject=reject))
    (repo / "tools" / "claim_lint.py").write_text(LINT)
    (repo / "tools" / "pin_core.py").write_text(PIN.format(refuse=pin_refuses))
    (repo / "POLICY.md").write_text("No file over 20 MB, and no run over 50 MB in total.\n")
    write_json(repo / "kits.lock", {"bounded-embodiment": kits or {}})
    write_json(repo / "labs" / lab / "lab.json",
               {"name": "Test lab", "github_handles": ["test-user"], "affiliation": "independent",
                "contact": "lab@example.org", "keys": entries})
    return repo


def real_results_repo() -> Path | None:
    """A clone of the results repository: $BE_RESULTS_REPO, else a sibling checkout named
    bounded-embodiment-results or be-results-public next to any parent of this repository."""
    env = os.environ.get("BE_RESULTS_REPO")
    candidates = [Path(env)] if env else []
    for parent in REPO.parents:
        candidates += [parent / "bounded-embodiment-results", parent / "be-results-public"]
    for c in candidates:
        if (c / "tools" / "path_guard.py").is_file() and (c / "tools" / "claim_lint.py").is_file():
            return c
    return None
