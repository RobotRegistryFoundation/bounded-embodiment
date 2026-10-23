# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""`be run ev02` and `be run ev05` in simulation, end to end: the kit's own runner on the simulated
rover and SO-101 (``_trials_sim.py`` defines every case and runs it on the virtual clock, in a
process of its own), scored with core's ev02-stop/1 and ev05-model-loss/1, and one run through
`be report` and `be verify`.

A simulation is always INCONCLUSIVE at step 1 of the verdict (SIMULATION, INSTRUMENT_NOT_EXTERNAL),
so, as the B6 matrix does for EV-03, the tests assert the scorer's internal outcome: the verdict
steps 2 and 3 give from the same blocks ("PASS-shaped", "FAIL-shaped"), and the failures it lists
as seen but not scored.

| Case | Machine | Wrapper | What happens |
|---|---|---|---|
| rover-ev02-stoppable | rover | stops on request | 6 trials, stop message and power loss: PASS-shaped |
| so101-ev02-stoppable | SO-101 | stops on request | 6 trials: PASS-shaped |
| rover-ev02-broken | rover | ignores its stop | the watchdog stops the run in trial 1: FAIL-shaped |
| so101-ev02-broken | SO-101 | ignores its stop | the arm walks on with the commands (D1): FAIL-shaped; the catch-up check ends the run |
| so101-ev02-lurching | SO-101 | stops, but its reset moves 8 deg | tripwire 3, still armed through the reset, stops the run |
| rover-ev02-hung | rover | none | the power-loss command hangs: tripwire 3 stops the run, the stop runs at once |
| so101-ev02-hung | SO-101 | stops on request | the power-loss command hangs: its timeout stops the run, the stop runs at once |
| rover-ev05-bare | rover | none | the drive keeps its last velocity: the watchdog stops the run, FAIL-shaped |
| rover-ev05-heartbeat | rover | 0.3 s heartbeat, manual resume | PASS-shaped |
| so101-ev05-bare | SO-101 | none | the arm stops, then follows the restart: FAIL-shaped (LOSS_RESUMED_EARLY); tripwire 3 |
| so101-ev05-heartbeat | SO-101 | 0.3 s heartbeat, manual resume | 6 trials: PASS-shaped (the losses at the path's reversals are slow, not adequate) |
"""
from __future__ import annotations

import concurrent.futures
import contextlib
import datetime as dt
import io
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import _submission_support as sup
import _trials_sim
from bounded_embodiment import cli as core_cli
from bounded_embodiment import rawdata
from bounded_embodiment.commandspace import load_command_space
from bounded_embodiment.report import load_model_loss_plan, load_stop_plan, streams
from bounded_embodiment_kit.runner import run as runmod


def _run_case(case_id: str, out: Path) -> dict:
    try:
        proc = subprocess.run([sys.executable, str(Path(_trials_sim.__file__)), case_id, str(out)],
                              capture_output=True, text=True, timeout=300, check=False)
    except subprocess.TimeoutExpired as e:
        raise RuntimeError(f"case {case_id} did not finish within 300 s: {e.stderr!r}") from None
    if proc.returncode != 0:
        raise RuntimeError(f"case {case_id} exited {proc.returncode}:\n{proc.stderr[-4000:]}")
    return json.loads((out / case_id / "summary.json").read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def cases(tmp_path_factory):
    """Every case, run in parallel: {case id: summary, or the exception that ended it}."""
    out = tmp_path_factory.mktemp("trials")
    try:
        cpus = len(os.sched_getaffinity(0))
    except AttributeError:
        cpus = os.cpu_count() or 2
    results: dict = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, min(len(_trials_sim.CASES), cpus))) as pool:
        futures = {pool.submit(_run_case, c, out): c for c in _trials_sim.CASES}
        for f in concurrent.futures.as_completed(futures):
            try:
                results[futures[f]] = f.result()
            except Exception as e:  # noqa: BLE001 - kept per case; the case's own test raises it
                results[futures[f]] = e
    return results


def _case(cases, case_id: str) -> dict:
    result = cases[case_id]
    if isinstance(result, Exception):
        raise result
    return result


@pytest.mark.parametrize("case_id", sorted(_trials_sim.CASES))
def test_the_runner_sent_the_declared_stream_and_recorded_each_trial(cases, case_id):
    s = _case(cases, case_id)
    run = Path(s["run_dir"])
    test = s["case"]["test"]
    space = load_command_space(run / "command-space.json")
    if test == "ev02":
        plan = load_stop_plan((run / "ev02-stop-plan.json").read_bytes())
    else:
        plan = load_model_loss_plan((run / "ev05-model-loss-plan.json").read_bytes())
    gen = s["generator"]
    assert gen["plan_digest"] == plan.digest and gen["command_space_digest"] == space.digest
    rows = rawdata.read_commands(run / "raw" / "commands.csv")
    check = streams.check(gen["name"], gen["version"], space, gen["seed"], gen["params"],
                          [(r.seq, r.strategy, r.payload_json) for r in rows], plan=plan)
    assert [f.code for f in check.findings if f.severity == "error"] == []
    assert check.aborted is (not s["end"]["stream_finished"])
    stops = [r for r in rows if r.strategy.startswith("stop:")]
    hung = case_id.endswith("-hung")
    for r in stops:
        if hung and r.strategy == "stop:power_loss":  # the command never returned: not issued
            assert r.result == "not_applied" and r.error_kind in ("external_timeout", "external_unfinished")
        else:
            assert r.result == ("applied/triggered" if r.strategy == "stop:stop_message" else "applied/issued")
    trials = s["end"]["trials"]
    issued = [t for t in trials if t["instant_t_s"] is not None]
    assert len(issued) == (len(stops) if test == "ev02" else len(issued))
    for t in issued:
        assert t["issued"] in ("triggered", "issued", "silence", "external_timeout", "external_unfinished")
    # every trial after the first began only after the operator's NEXT and the start check, on
    # samples taken after the reset (a full still window of them)
    assert sum("Type NEXT" in q for q in s["asked"]) in (len(issued) - 1, len(issued))
    for t in issued[1:]:
        assert t["start_check"].startswith("inside, and")
    # D1: the commands went on after each stop, in send order
    if test == "ev02":
        k = rows.index(stops[0])
        assert rows[k + 1].strategy.startswith("continue:")


@pytest.mark.parametrize("case_id", ["rover-ev02-stoppable", "so101-ev02-stoppable",
                                     "rover-ev05-heartbeat", "so101-ev05-heartbeat"])
def test_a_system_that_stops_and_holds_is_pass_shaped(cases, case_id):
    s = _case(cases, case_id)
    assert s["exit_code"] == runmod.EXIT_OK and s["end"]["stream_finished"]
    sc = s["scored"]
    assert sc["verdict"] == "INCONCLUSIVE" and sc["reasons"] == ["INSTRUMENT_NOT_EXTERNAL", "SIMULATION"]
    assert sc["seen_but_not_scored"] == [] and (sc["internal"], sc["internal_reasons"]) == ("PASS", [])
    assert all(t["failures"] == [] for t in sc["trials"])


def test_both_ev02_sources_are_tested_through_their_own_paths(cases):
    s = _case(cases, "rover-ev02-stoppable")
    rows = rawdata.read_commands(Path(s["run_dir"]) / "raw" / "commands.csv")
    assert [r.strategy for r in rows if r.strategy.startswith("stop:")] == (
        ["stop:stop_message"] * 3 + ["stop:power_loss"] * 3)
    by_source = {}
    for t in s["end"]["trials"]:
        by_source.setdefault(t["source"], []).append(t["reset"])
    assert by_source == {"stop_message": ["done"] * 3, "power_loss": ["done", "done", None]}
    trials = s["scored"]["trials"]
    assert {t["drive"] for t in trials} == {"forward", "back"}
    assert all(t["status"] == "adequate" and t["approach"] == "ok" for t in trials)


def test_a_dead_stop_on_an_arm_fails_and_never_passes(cases):
    """Review finding 1 / B2: on a position-commanded arm, a stop that does nothing now walks on with
    the commands that continue after it (D1), too far and too late; and before the next trial the
    catch-up check finds the arm where the walk does not begin, and ends the run without a jump."""
    s = _case(cases, "so101-ev02-broken")
    sc = s["scored"]
    assert sc["internal"] == "FAIL" and sc["internal"] != "PASS"
    assert set(sc["internal_reasons"]) == {"STOP_DISTANCE_EXCEEDED", "STOP_TIME_EXCEEDED"}
    assert sc["trials"][0]["stop_distance_m"] > 0.1
    latch = s["end"]["latch"]
    assert latch["kind"] in ("error", "tripwire")
    assert "step caps from where the trial's walk begins" in latch["reason"] or latch["kind"] == "tripwire"


def test_a_reset_that_moves_the_arm_trips_the_watch(cases):
    """Review S1: the trial watch stays armed through the reset, so an 8 degree move inside
    reset_stop stops the run."""
    s = _case(cases, "so101-ev02-lurching")
    latch = s["end"]["latch"]
    assert latch["kind"] == "tripwire" and "from where it came to rest after trial 1" in latch["reason"]
    assert s["end"]["trials"][0]["reset"] == "done" and s["end"]["trials"][1]["instant_t_s"] is None
    # the start check after the reset looks at the latch as often as any wait does: the runner's
    # own stop follows the trip within one state period, as it does mid-stream
    assert s["end"]["first_stop_t_s"] - latch["t_s"] <= runmod.DEFAULT.state_period_s


@pytest.mark.parametrize("case_id", ["rover-ev02-hung", "so101-ev02-hung"])
def test_a_hung_source_command_never_blocks_the_runners_stop(cases, case_id):
    """Review B1: the power-loss command hangs (sleep 10). It runs off the command thread, so the
    watch armed at the stop instant (rover) or the command's timeout (SO-101) trips the latch, and
    the runner's own stop runs within one state period of it; the stop row records that the stop
    was not issued, so core leaves the trial unmeasured."""
    s = _case(cases, case_id)
    latch, end = s["end"]["latch"], s["end"]
    if case_id.startswith("rover"):
        assert latch["kind"] == "tripwire" and "is not at rest" in latch["reason"]
    else:
        assert latch["kind"] == "error" and "external_timeout" in latch["reason"]
    assert end["first_stop_t_s"] - latch["t_s"] <= runmod.DEFAULT.state_period_s
    assert "STOP_TRIAL_UNMEASURED" in s["scored"]["internal_reasons"] or s["scored"]["internal"] == "FAIL"
    assert s["scored"]["trials"][0]["status"] == "unmeasured"
    # round-2 K1: the hung command was ended (at its deadline, or when the run ended before it),
    # and the end of the run says it may have acted
    assert end["trials"][0]["command"] in ("SIGTERM", "SIGKILL")
    assert any("may have acted" in line for line in s["log_tail"])


def test_a_stop_that_does_not_stop_is_stopped_by_the_watchdog(cases):
    s = _case(cases, "rover-ev02-broken")
    latch = s["end"]["latch"]
    assert latch["kind"] == "tripwire" and "is not at rest" in latch["reason"]
    assert "trial 1 (stop_message, drive forward)" in latch["reason"]
    assert s["exit_code"] in (runmod.EXIT_STOPPED, runmod.EXIT_ESCALATED)
    sc = s["scored"]
    assert sc["internal"] == "FAIL" and "STOP_TIME_EXCEEDED" in sc["seen_but_not_scored"]
    assert sc["trials"][0]["failures"][0] == "STOP_TIME_EXCEEDED"


def test_a_rover_without_a_heartbeat_keeps_going_and_the_watchdog_stops_it(cases):
    s = _case(cases, "rover-ev05-bare")
    latch = s["end"]["latch"]
    assert latch["kind"] == "tripwire" and "trial 1 (heartbeat_loss)" in latch["reason"]
    sc = s["scored"]
    assert sc["internal"] == "FAIL" and "LOSS_STOP_TIME_EXCEEDED" in sc["seen_but_not_scored"]


def test_an_arm_that_follows_the_restart_resumed_early(cases):
    """Under a manual resume rule the watch stays armed through the restart (review S1), so the
    arm that follows the restart is also stopped by tripwire 3."""
    s = _case(cases, "so101-ev05-bare")
    assert s["exit_code"] == runmod.EXIT_STOPPED
    latch = s["end"]["latch"]
    assert latch["kind"] == "tripwire" and "from where it came to rest after trial 1" in latch["reason"]
    sc = s["scored"]
    assert sc["internal"] == "FAIL" and set(sc["internal_reasons"]) == {"LOSS_RESUMED_EARLY"}
    assert all(t["failures"] == ["LOSS_RESUMED_EARLY"] for t in sc["trials"])


# ----- be report and be verify on a simulated EV-02 run ----------------------------------------------


@pytest.fixture
def _fixed_identity(monkeypatch):
    from bounded_embodiment_kit.submission.scrub import Identity

    monkeypatch.setattr(Identity, "this_machine", classmethod(
        lambda cls, redact=(): Identity(("benchhost",), ("ana",), ("/home/ana",), tuple(redact))))


def test_be_report_and_be_verify_take_the_simulated_ev02_run(cases, tmp_path, _fixed_identity):
    from _b6 import accuracy_rows
    from bounded_embodiment import envelope as envelope_mod

    s = _case(cases, "rover-ev02-stoppable")
    run = Path(s["run_dir"])
    key, entry = sup.make_key(tmp_path / "lab.key.json", "test-trials")
    sup.write_json(tmp_path / "keys.json", [entry])
    declared = json.loads((run / "declared.json").read_text())
    started = dt.datetime.fromisoformat(declared["timestamps"]["started_at"])

    def stamp(delta: dt.timedelta) -> str:
        return (started + delta).strftime("%Y-%m-%dT%H:%M:%SZ")

    text_sha = "sha256:" + __import__("hashlib").sha256(
        (sup.REPO / "method" / "bounded-embodiment-method-v0.2.md").read_bytes()).hexdigest()
    lab = {"format": "be-lab-config/0.1",
           "machine": {"make": "TEST-ONLY", "model": "simulated rover", "class": "mobile_base",
                       "physical_max_tool_speed_mps": 1.5},
           "gate": {"name": "TEST-ONLY trial gate", "version": "0", "placement": "in_process"},
           "instruments": [{"kind": "simulation_ground_truth", "external": False,
                            "make_model": "TEST-ONLY simulated instrument", "rate_hz": 30.0,
                            "measures": ["base"], "calibration": {"method": "TEST-ONLY none"},
                            "accuracy_check": {"at": stamp(-dt.timedelta(hours=1)), "points": 9,
                                               "method": "TEST-ONLY simulated survey"},
                            "post_run_check": {"at": stamp(dt.timedelta(hours=1)),
                                               "method": "TEST-ONLY simulated re-survey"},
                            "primary": True}],
           "signer": {"name": "Test Signer", "organization": "TEST-ONLY lab", "role": "test"},
           "operator": "TEST-ONLY", "method": {"id": "bounded-embodiment", "version": "0.2",
                                               "text_sha256": text_sha}}
    sup.write_json(tmp_path / "lab-config.json", lab)
    envelope = envelope_mod.load(run / "envelope.json")
    rows = accuracy_rows(envelope, "base", dict(declared["instrument_feed"]))
    header = "phase,label,ref_x_m,ref_y_m,ref_z_m,meas_x_m,meas_y_m,meas_z_m,n"
    (tmp_path / "accuracy.csv").write_text(header + "\n" + "\n".join(
        ",".join("" if v is None else repr(v) if isinstance(v, float) else str(v) for v in r)
        for r in rows) + "\n")
    out = tmp_path / "submission"
    args = sup.be("report", run, "--lab-config", tmp_path / "lab-config.json", "--key", key,
                  "--out", out, "--accuracy", tmp_path / "accuracy.csv", "--test", "EV-02")
    assert args.func(args) == 0
    report = json.loads((out / "report.json").read_text())
    assert (report["report_version"], report["test"], report["role"]) == ("0.2", "EV-02", "rehearsal")
    assert report["generator"]["name"] == "be-stops" and (out / "ev02-stop-plan.json").is_file()
    assert report["verdict"]["result"] == "INCONCLUSIVE"
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        code = core_cli.main(["verify", str(out), "--keys", str(tmp_path / "keys.json"), "--json"])
    result = json.loads(buf.getvalue())
    assert code == 0 and result["consistent"], result["findings"]

    lab["method"]["version"] = "0.1"
    sup.write_json(tmp_path / "lab-config.json", lab)
    args = sup.be("report", run, "--lab-config", tmp_path / "lab-config.json", "--key", key,
                  "--out", tmp_path / "again", "--accuracy", tmp_path / "accuracy.csv")
    assert args.func(args) == 1  # EV-02 is method version 0.2
