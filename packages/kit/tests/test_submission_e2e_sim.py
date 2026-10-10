# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""End to end in simulation: `be run` on the simulated SO-101 -> an instrument's samples on
another clock (the simulator's truth plus noise) -> `be keygen` -> `be report` -> `be verify` ->
`be submit` into a copy of the results repository, whose own tools then check it as CI would.

The results repository's tools run when a clone is found ($BE_RESULTS_REPO, or a sibling checkout
named bounded-embodiment-results or be-results-public); without one those steps skip.
"""
from __future__ import annotations

import builtins
import contextlib
import datetime as dt
import io
import json
import os
import random
import shutil
import subprocess
import sys
from decimal import Decimal
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import _runner_fakes as fakes
import _submission_support as sup
from bounded_embodiment import cli as core_cli
from bounded_embodiment import rawdata
from bounded_embodiment_kit.runner import run as runmod
from bounded_embodiment_kit.submission import ids

LAB = "test-e2e"
OTHER_CLOCK = Decimal("1234.5")


@pytest.fixture(autouse=True)
def _fixed_identity(monkeypatch):
    """Scrub as a machine named "benchhost" with the login "ana", whatever runs the tests (the
    real machine's names are test_submission_scrub's business)."""
    from bounded_embodiment_kit.submission.scrub import Identity

    monkeypatch.setattr(Identity, "this_machine", classmethod(
        lambda cls, redact=(): Identity(("benchhost",), ("ana",), ("/home/ana",), tuple(redact))))


def _core(*argv) -> tuple[int, str]:
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        code = core_cli.main([str(a) for a in argv])
    return code, out.getvalue()


@pytest.fixture(scope="module")
def sim(tmp_path_factory):
    """A 3 s shakedown on the simulated SO-101 (the runner's own e2e test runs the same), a key
    from `be keygen`, and a lab config for an external markerless instrument."""
    tmp = tmp_path_factory.mktemp("e2e")
    answers = iter(["RUN", ""])
    saved = builtins.input
    builtins.input = lambda question="": next(answers, "")
    try:
        args = sup.be("run", "ev03", "--mode", "shakedown", "--adapter", "sim-so101",
                      "--envelope", fakes.envelope_path("so101"), "--command-space", fakes.SO101_SPACE,
                      "--kit-config", fakes.ENVELOPES / "so101.kit.json", "--urdf", fakes.URDF,
                      "--instrument", "sim", "--sim-sigma-m", "0.0003", "--sim-lag-s", "0.02",
                      "--out", tmp / "run", "--seconds", "3", "--i-accept-the-risk",
                      "--hardware-stop", "none (simulation)", "--ledger", tmp / "ledger.jsonl")
        code = args.func(args)
    finally:
        builtins.input = saved
    # A loaded machine can make the simulated feed late and the watchdog stop the run early; a
    # run that started and sent commands is still a run to report (aborted: a stream prefix).
    assert code in (runmod.EXIT_OK, runmod.EXIT_STOPPED, runmod.EXIT_ESCALATED)
    declared = json.loads((tmp / "run" / "declared.json").read_text())
    assert declared["end"]["started"] and declared["end"]["commands_sent"] > 0
    stamps = declared["timestamps"]
    assert {"monotonic_start_s", "wall_start_s", "wall_end_s", "monotonic_end_s"} <= set(stamps)

    code, printed = _core("keygen", "--out", tmp / "lab.key.json", "--lab", LAB,
                          "--contact", "lab@example.org")
    assert code == 0
    entry = json.loads(printed)
    sup.write_json(tmp / "keys.json", [entry])

    now = dt.datetime.now(dt.timezone.utc)
    lab = sup.lab_config()
    lab["machine"] = {"make": "TEST-ONLY", "model": "simulated SO-101", "class": "arm",
                      "physical_max_tool_speed_mps": 1.0}
    lab["gate"] = {"name": "none", "version": "0", "placement": "none"}
    lab["instruments"] = [{
        "kind": "markerless_model_tracker", "external": True, "independent_of_robot_state": True,
        "make_model": "TEST-ONLY synthetic markerless fit", "rate_hz": 50.0,
        "measures": ["tool", "elbow", "wrist"],
        "calibration": {"method": "TEST-ONLY synthetic registration",
                        "files": ["raw/instrument/fit_meta.json"]},
        "accuracy_check": {"at": (now - dt.timedelta(hours=1)).strftime("%Y-%m-%dT%H:%M:%SZ"),
                           "points": 8, "method": "TEST-ONLY synthetic survey"},
        "post_run_check": {"at": (now + dt.timedelta(minutes=5)).strftime("%Y-%m-%dT%H:%M:%SZ"),
                           "method": "TEST-ONLY synthetic re-survey"},
        "primary": True}]
    sup.write_json(tmp / "lab-config.json", lab)
    rows = ["phase,label,ref_x_m,ref_y_m,ref_z_m,meas_x_m,meas_y_m,meas_z_m,n"]
    for i in range(8):
        x = 0.2 + 0.01 * i
        rows += [f"pre,p{i + 1},{x},0.0,0.1,{x + 0.001},0.0,0.1,30",
                 f"post,p{i + 1},{x},0.0,0.1,{x + 0.001},0.0005,0.1,30"]
    rows += [f"jitter,j1,,,,{0.2 + d},{d},{0.1 + d},1" for d in (-0.0005, 0.0, 0.0005)]
    (tmp / "accuracy.csv").write_text("\n".join(rows) + "\n")
    (tmp / "NOTES.md").write_text("A simulated shakedown; the instrument ran on /home/ana/rig.\n")
    return SimpleNamespace(tmp=tmp, run=tmp / "run", declared=declared, entry=entry,
                           key=tmp / "lab.key.json")


def _instrument(sim, at_run_start: Decimal, name: str) -> Path:
    """be fit's output shape: SESSION/fit/samples.csv, the simulator's truth (the FK of its
    joints, from raw/state.csv) plus 0.5 mm of noise, stamped on a clock that reads
    ``at_run_start`` at run start; and SESSION/fit/fit_meta.json."""
    fk: dict = {}
    for r in rawdata.read_state(sim.run / "raw" / "state.csv"):
        if r.source == "fk":
            point, axis = r.joint.rsplit(".", 1)
            fk.setdefault((r.t_s, point), {})[axis] = r.value
    rng = random.Random(7)
    lines = ["t_s,point,x_m,y_m,z_m,valid,residual_m"]
    for (t, point), xyz in sorted(fk.items()):
        stamp = (Decimal(repr(t)) + at_run_start).quantize(Decimal("0.000001"))
        noisy = [f"{xyz[a] + rng.gauss(0.0, 0.0005):.6f}" for a in "xyz"]
        lines.append(f"{stamp},{point},{','.join(noisy)},1,0.0005")
    fit = sim.tmp / name / "fit"
    fit.mkdir(parents=True)
    (fit / "samples.csv").write_text("\n".join(lines) + "\n")
    sup.write_json(fit / "fit_meta.json", {"session": {"path": "/home/ana/sessions/" + name},
                                           "outputs": {"t_s": "the recording host's CLOCK_MONOTONIC"}})
    return fit


def _clock_csv(sim, path: Path, at_run_start: Decimal) -> None:
    """The recorder's clock.csv: its monotonic clock reads at_run_start at the runner's wall
    start; its wall clock is the runner's, give or take 40 us."""
    w0 = sim.declared["timestamps"]["wall_start_s"]
    rows = ["kind,t_wall_s,t_mono_s,uncertainty_s"]
    for k, tau in enumerate(range(-20, 30, 5)):
        rows.append(f"{'start' if k == 0 else 'periodic'},{w0 + tau + (k % 3 - 1) * 4e-5:.6f},"
                    f"{float(at_run_start) + tau:.6f},0.000002000")
    path.write_text("\n".join(rows) + "\n")


def _report(sim, fit: Path, out: Path, *clock_args) -> dict:
    args = sup.be("report", sim.run, "--lab-config", sim.tmp / "lab-config.json", "--key", sim.key,
                  "--out", out, "--samples", fit / "samples.csv", *clock_args,
                  "--accuracy", sim.tmp / "accuracy.csv", "--instrument-file", fit / "fit_meta.json",
                  "--notes", sim.tmp / "NOTES.md")
    assert args.func(args) == 0
    code, printed = _core("verify", out, "--keys", sim.tmp / "keys.json", "--json")
    result = json.loads(printed)
    assert code == 0 and result["consistent"], result["findings"]
    return json.loads((out / "report.json").read_text())


def _aligned(sim, out: Path, report: dict) -> None:
    """The converted samples sit on the robot's own FK times, to within U."""
    u = Decimal(repr(report["run"]["clock"]["uncertainty_s"]))
    fk_times = sorted({Decimal(repr(r.t_s)) for r in rawdata.read_state(sim.run / "raw" / "state.csv")
                       if r.source == "fk"})
    times = sorted({Decimal(repr(s.t_s)) for s in rawdata.read_samples(out / "raw" / "samples.csv.gz")})
    assert len(times) == len(fk_times)
    assert max(abs(a - b) for a, b in zip(times, fk_times, strict=True)) <= u


def test_another_hosts_clock_by_wall_clock_pairs(sim):
    fit = _instrument(sim, OTHER_CLOCK, "wall-session")
    _clock_csv(sim, fit.parent / "clock.csv", OTHER_CLOCK)  # found beside fit/ by be report
    out = sim.tmp / "submission-wall"
    report = _report(sim, fit, out, "--clock", "wall", "--wall-clock-agreement-s", "0.002")
    clock = report["run"]["clock"]
    assert abs(Decimal(repr(clock["offset_s"])) + OTHER_CLOCK) <= Decimal(repr(clock["uncertainty_s"]))
    assert 0.002 < clock["uncertainty_s"] < 0.0021
    _aligned(sim, out, report)
    assert (report["role"], report["run"]["mode"], report["verdict"]["result"]) == \
        ("rehearsal", "simulation", "INCONCLUSIVE")
    assert (out / "notes/NOTES.md").read_text() == \
        "A simulated shakedown; the instrument ran on ~/rig.\n"
    assert json.loads((out / "raw/instrument/fit_meta.json").read_text())["session"]["path"] == \
        "~/.../wall-session"
    assert (out / "raw/instrument/recorder-clock.csv").is_file()


def test_this_hosts_monotonic_clock(sim):
    mono = Decimal(repr(sim.declared["timestamps"]["monotonic_start_s"]))
    fit = _instrument(sim, mono, "mono-session")
    _clock_csv(sim, fit.parent / "clock.csv", mono)  # the same host's pairs: the cross-check holds
    out = sim.tmp / "submission-mono"
    report = _report(sim, fit, out, "--clock", "monotonic")
    clock = report["run"]["clock"]
    assert Decimal(repr(clock["offset_s"])) == (-mono).quantize(Decimal("0.000001"))
    assert clock["uncertainty_s"] == 0.000001 and "cross-checked" in clock["method"]
    _aligned(sim, out, report)


def _git(repo: Path, *argv: str) -> str:
    env = {**os.environ, "GIT_CONFIG_NOSYSTEM": "1", "GIT_CONFIG_GLOBAL": os.devnull,
           "HOME": str(repo.parent)}
    proc = subprocess.run(["git", "-c", "user.name=Test", "-c", "user.email=test@example.org",
                           "-c", "commit.gpgsign=false", *argv], cwd=repo, env=env,
                          capture_output=True, text=True, check=True)
    return proc.stdout


def _tool(repo: Path, *argv) -> tuple[int, str]:
    proc = subprocess.run([sys.executable, *map(str, argv)], cwd=repo, capture_output=True, text=True,
                          check=False)
    return proc.returncode, proc.stdout + proc.stderr


def test_submit_into_the_results_repository_and_its_ci_tools_accept_it(sim, capsys):
    clone = sup.real_results_repo()
    if clone is None:
        pytest.skip("no clone of the results repository (set BE_RESULTS_REPO)")
    if shutil.which("git") is None:
        pytest.skip("git is not installed")
    fit = _instrument(sim, OTHER_CLOCK, "submit-session")
    _clock_csv(sim, fit.parent / "clock.csv", OTHER_CLOCK)
    out = sim.tmp / "submission-submit"
    report = _report(sim, fit, out, "--clock", "wall", "--wall-clock-agreement-s", "0.002")

    # The results repository as main has it, with this lab registered (keys are read from main).
    repo = sim.tmp / "results"
    shutil.copytree(clone, repo, ignore=shutil.ignore_patterns(".git", "__pycache__", ".pytest_cache"))
    (repo / "labs" / LAB).mkdir()
    sup.write_json(repo / "labs" / LAB / "lab.json",
                   {"name": "TEST-ONLY lab", "github_handles": ["test-user"],
                    "affiliation": "independent", "contact": "lab@example.org", "keys": [sim.entry]})
    _git(repo, "init", "-q")
    _git(repo, "checkout", "-q", "-b", "main")
    _git(repo, "add", "-A")
    _git(repo, "commit", "-q", "-m", "main")
    _git(repo, "update-ref", "refs/remotes/origin/main", "HEAD")
    _git(repo, "checkout", "-q", "-b", "submit")

    args = sup.be("submit", out, "--results-repo", repo, "--lab", LAB, "--allow-dev-core")
    assert args.func(args) == 0
    printed = capsys.readouterr()
    dest = repo / "results" / LAB / ids.run_dir_name(report)
    assert sorted(p.name for p in (repo / "results" / LAB).iterdir()) == [dest.name]
    assert dest.name.startswith("ev03-") and dest.name.endswith("-s7")
    assert "path-guard: accepted" in printed.err and "claim-lint: no claim words" in printed.err
    assert "gh pr create" in printed.out

    # Then CI's checks, from the committed branch: path guard (git mode), identity, be verify.
    _git(repo, "add", "results")
    _git(repo, "commit", "-q", "-m", "results")
    code, guard = _tool(repo, repo / "tools" / "path_guard.py", "--base", "origin/main", "--json")
    assert code == 0, guard
    assert json.loads(guard)["run_dirs"] == [f"results/{LAB}/{dest.name}"]
    code, identity = _tool(repo, repo / "tools" / "identity.py", f"results/{LAB}", "--author",
                           "test-user", "--base", "origin/main", "--keyring-out", sim.tmp / "ci-keys.json")
    assert code == 0, identity
    code, verified = _core("verify", dest, "--keys", sim.tmp / "ci-keys.json", "--json")
    assert code == 0 and json.loads(verified)["consistent"]
    code, lint = _tool(repo, repo / "tools" / "claim_lint.py", f"results/{LAB}/{dest.name}")
    assert code == 0, lint
    assert report["verdict"]["result"] == "INCONCLUSIVE"
