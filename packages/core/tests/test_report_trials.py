"""EV-02 and EV-05 runs (method v0.2): test report 0.2, the plan files, and what build and verify
refuse."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from _report_helpers import (
    copy_run,
    golden,
    in_repo,
    keyring,
    make_run,
    read_report,
    resign,
)
from bounded_embodiment import report as rep
from bounded_embodiment import schema
from bounded_embodiment.report import derive

pytestmark = in_repo

EV03_ONLY = ("window", "achieved_rates", "score", "gaps", "coverage")


def _codes(exc: pytest.ExceptionInfo) -> list[str]:
    return sorted(f.code for f in exc.value.findings)


def _ev02_declared(**changes) -> dict:
    g = golden()
    doc = g.declared(test="EV-02", generator=g.ev02_generator(), method_version="0.2",
                     profile="ev02-illustrative-0.1")
    doc.update(changes)
    return doc


def _ev02_run(tmp_path: Path, **file_changes) -> Path:
    g = golden()
    run = tmp_path / "ev02"
    run.mkdir()
    files = g.ev02_files(list(g.EV02_TRIALS))
    for rel, data in file_changes.items():
        if data is None:
            files.pop(rel)
        else:
            files[rel] = data
    g.write_files(run, files)
    return run


# --------------------------------------------------------------------------- reports


@pytest.mark.parametrize(
    ("name", "block", "other"),
    [("ev02-pass", "stops", "model_loss"), ("ev05-pass", "model_loss", "stops")],
)
def test_ev02_and_ev05_reports_are_0_2(name: str, block: str, other: str) -> None:
    report = read_report(golden().OUT / name)
    assert report["report_version"] == "0.2" and report["method"]["version"] == "0.2"
    assert schema.validate(report, "test-report-0.2") == []
    assert schema.validate(report, "test-report-0.1") != []  # 0.1 has no EV-02 or EV-05
    assert block in report and other not in report
    for member in (*EV03_ONLY, "motion", "liveness"):
        assert member not in report, member
    assert report["scoring"]["method"] == derive.SCORER_FOR_TEST[report["test"]]
    roles = {e["role"]: e["path"] for e in report["raw_data"]}
    plan_role = "stop_plan" if block == "stops" else "model_loss_plan"
    assert roles[plan_role] == derive.CANONICAL_PATH[plan_role]


def test_report_version_by_test() -> None:
    assert [rep.report_version_for(t) for t in ("EV-02", "EV-03", "EV-05", "EV-10")] == [
        "0.2",
        "0.1",
        "0.2",
        "0.1",
    ]
    assert rep.REPORT_VERSIONS == ("0.1", "0.2")


def test_the_0_2_schema_holds_the_derived_blocks_closed() -> None:
    report = read_report(golden().OUT / "ev02-pass")
    report["stops"]["trials"][0]["extra"] = 1
    assert [f.code for f in schema.validate(report, "test-report-0.2")] == ["SCHEMA_INVALID"]
    report = read_report(golden().OUT / "ev02-pass")
    report["window"] = {"start_s": 0.0}
    assert schema.validate(report, "test-report-0.2") != []
    report = read_report(golden().OUT / "ev05-pass")
    report["model_loss"]["trials"][0]["failures"] = ["STOP_NOT_HELD"]
    assert schema.validate(report, "test-report-0.2") != []


def test_an_ev03_report_may_be_0_2(tmp_path: Path) -> None:
    g = golden()
    run = tmp_path / "ev03"
    run.mkdir()
    g.write_files(run, g.ev03_files(g.square_loop()))
    doc = g.declared()
    doc["report_version"] = "0.2"
    report = g.finish(run, doc)
    assert report["report_version"] == "0.2"
    assert schema.validate(report, "test-report-0.2") == []
    result = rep.verify(run, keyring())
    assert result.consistent and result.verdict["result"] == "PASS"


def test_an_ev02_report_cannot_be_0_1(tmp_path: Path) -> None:
    run = _ev02_run(tmp_path)
    with pytest.raises(rep.ReportError) as exc:
        rep.build(_ev02_declared(report_version="0.1"), run)
    assert "SCHEMA_INVALID" in _codes(exc)
    with pytest.raises(rep.ReportError) as exc:
        rep.build(_ev02_declared(report_version="0.3"), run)
    assert _codes(exc) == ["REPORT_VERSION_UNSUPPORTED"]


# --------------------------------------------------------------------------- the plan files


def test_an_ev02_run_needs_its_plan(tmp_path: Path) -> None:
    run = _ev02_run(tmp_path, **{"ev02-stop-plan.json": None})
    with pytest.raises(rep.ReportError) as exc:
        rep.build(_ev02_declared(), run)
    assert "REQUIRED_FILE_MISSING" in _codes(exc)


def test_a_plan_belongs_to_its_own_test(tmp_path: Path) -> None:
    g = golden()
    run = tmp_path / "ev03"
    run.mkdir()
    files = g.ev03_files(g.square_loop())
    files["ev02-stop-plan.json"] = g._json(g.stop_plan_doc(list(g.EV02_TRIALS)))
    g.write_files(run, files)
    with pytest.raises(rep.ReportError) as exc:
        rep.build(g.declared(), run)
    found = [f for f in exc.value.findings if f.code == "UNEXPECTED_FILE"]
    assert found and "belongs to EV-02 runs only" in found[0].detail
    ev05 = tmp_path / "ev05"
    ev05.mkdir()
    files = g.ev05_files()
    files["ev02-stop-plan.json"] = g._json(g.stop_plan_doc(list(g.EV02_TRIALS)))
    g.write_files(ev05, files)
    with pytest.raises(rep.ReportError) as exc:
        rep.build(
            g.declared(test="EV-05", generator=g.ev05_generator(), method_version="0.2",
                       profile="ev05-illustrative-0.1"),
            ev05,
        )
    assert "UNEXPECTED_FILE" in _codes(exc)


def test_a_plan_outside_the_command_space_is_refused(tmp_path: Path) -> None:
    g = golden()
    doc = g.stop_plan_doc(list(g.EV02_TRIALS))
    doc["drives"][0]["command"]["x"] = 0.5
    run = _ev02_run(tmp_path, **{"ev02-stop-plan.json": g._json(doc)})
    with pytest.raises(rep.ReportError) as exc:
        rep.build(_ev02_declared(), run)
    found = [f for f in exc.value.findings if f.code == "PLAN_COMMAND_OUTSIDE_LIMITS"]
    assert found and found[0].where == "ev02-stop-plan.json /drives/0/command"


def test_a_plan_that_is_not_json_is_refused(tmp_path: Path) -> None:
    run = _ev02_run(tmp_path, **{"ev02-stop-plan.json": b"{not json"})
    with pytest.raises(rep.ReportError) as exc:
        rep.build(_ev02_declared(), run)
    found = [f for f in exc.value.findings if f.code == "PLAN_PARSE_ERROR"]
    assert found and found[0].where == "ev02-stop-plan.json"


def test_the_generator_must_be_the_tests(tmp_path: Path) -> None:
    g = golden()
    run = _ev02_run(tmp_path)
    doc = _ev02_declared(generator={"name": "be-model-loss", "version": "1", "seed": 5,
                                    "params": dict(g.LOSS_PARAMS)})
    with pytest.raises(rep.ReportError) as exc:
        rep.build(doc, run)
    assert "GENERATOR_TEST_MISMATCH" in _codes(exc)


def test_a_simulated_stop_run_is_a_rehearsal(tmp_path: Path) -> None:
    g = golden()
    run = _ev02_run(tmp_path)
    sim = g.instrument(kind="simulation_ground_truth", external=False)
    with pytest.raises(rep.ReportError) as exc:
        rep.build(_ev02_declared(run=dict(g.declared()["run"], mode="simulation"),
                                 instruments=[sim]), run)
    assert "SIMULATION_NOT_REHEARSAL" in _codes(exc)
    report = g.finish(run, _ev02_declared(role="rehearsal",
                                          run=dict(g.declared()["run"], mode="simulation"),
                                          instruments=[sim]))
    assert report["verdict"]["result"] == "INCONCLUSIVE"
    codes = [r["code"] for r in report["verdict"]["reasons"]]
    assert "SIMULATION" in codes and "INSTRUMENT_NOT_EXTERNAL" in codes


# --------------------------------------------------------------------------- verify


def test_verify_recomputes_the_stop_block(tmp_path: Path) -> None:
    run = copy_run("ev02-fail", tmp_path)
    report = read_report(run)
    report["stops"]["trials"][4]["failures"] = []
    resign(run, report)
    result = rep.verify(run, keyring())
    assert not result.consistent
    assert "DERIVED_MISMATCH" in [f.code for f in result.findings]
    assert result.verdict["result"] == "FAIL"


def test_verify_reads_both_versions() -> None:
    for name in ("ev03-pass", "ev02-pass", "ev05-pass", "ev10-pass"):
        result = rep.verify(golden().OUT / name, keyring())
        assert result.consistent, (name, [f.as_dict() for f in result.findings])


def test_make_run_round_trips(tmp_path: Path) -> None:
    for name in ("ev02-inconclusive-one-source", "ev05-inconclusive-short-silence"):
        run = make_run(name, tmp_path)
        want = json.loads((golden().OUT / "expect.json").read_text())["runs"][name]
        result = rep.verify(run, keyring())
        assert result.consistent and result.verdict["result"] == want["verdict"]
