"""report.build and report.sign_report: a run directory becomes a signed report that verify finds
consistent, and build refuses what verify would not accept."""

from __future__ import annotations

import copy
import gzip
import json
from pathlib import Path

import pytest
from _report_helpers import golden, in_repo, keyring, make_run, read_report
from bounded_embodiment import __version__, canonical, schema, signing
from bounded_embodiment import report as rep
from bounded_embodiment.rawdata import StateRow

pytestmark = in_repo


def _declared(name: str = "ev03-pass", **changes) -> dict:
    """The declared fields the golden script uses for a run kind (EV-03 hostile by default)."""
    g = golden()
    doc = g.declared(**changes)
    if name == "ev03-pass":
        doc["instruments"] = [g.instrument(calibration_files=["raw/instrument/calibration.json"])]
    return doc


def _codes(exc: pytest.ExceptionInfo) -> list[str]:
    return sorted(f.code for f in exc.value.findings)


# --------------------------------------------------------------------------- round trips


@pytest.mark.parametrize("name", sorted(golden().RUNS))
def test_build_sign_verify_round_trip(name: str, tmp_path: Path) -> None:
    run = make_run(name, tmp_path)
    result = rep.verify(run, keyring())
    want = json.loads((golden().OUT / "expect.json").read_text())["runs"][name]
    assert result.consistent, [f.as_dict() for f in result.findings]
    assert result.verdict["result"] == want["verdict"]
    assert [r["code"] for r in result.verdict["reasons"]] == want["reasons"]
    report = read_report(run)
    assert report["report_version"] == rep.report_version_for(report["test"])
    assert schema.validate(report, rep.SCHEMA_NAMES[report["report_version"]]) == []


def test_build_returns_an_unsigned_report_and_sign_adds_one_entry(tmp_path: Path) -> None:
    run = make_run("ev03-pass", tmp_path)
    unsigned = rep.build(_declared(), run)
    assert "signature" not in unsigned
    assert schema.validate(unsigned, rep.SCHEMA_NAME) == []
    signed = rep.sign_report(unsigned, golden().LAB_PRIVATE)
    assert [e["kid"] for e in signed["signature"]] == [golden().LAB_KID]
    assert signing.verify(signed, keyring()) == []
    assert {k: v for k, v in signed.items() if k != "signature"} == unsigned


def test_build_is_deterministic(tmp_path: Path) -> None:
    run = make_run("ev03-fail", tmp_path)
    first = canonical.canonicalize(rep.build(_declared("ev03-fail"), run))
    second = canonical.canonicalize(rep.build(_declared("ev03-fail"), run))
    assert first == second


def test_report_has_the_fields_the_results_repository_reads(tmp_path: Path) -> None:
    """tools/build_table.py, pin_core.py and summary.py read exactly these paths."""
    report = read_report(make_run("ev03-pass", tmp_path))
    assert report["role"] in ("test", "control", "rehearsal")
    assert report["test"] == "EV-03"
    assert isinstance(report["method"]["version"], str)
    assert report["run"]["started_at"].endswith("Z")
    assert report["verdict"]["result"] in ("PASS", "FAIL", "INCONCLUSIVE")
    assert report["envelope"]["hash"].startswith("sha256:")
    assert report["provenance"]["core_version"] == __version__
    assert "reproduces" not in report  # optional
    assert report["statements"] == {
        "not_certification": True,
        "thresholds_are_illustrative": True,
        "measurement_source": "external_instrument",
    }
    assert report["license"] == "CC-BY-4.0" and report["rights_affirmed"] is True


def test_report_order_puts_declared_members_first(tmp_path: Path) -> None:
    keys = list(read_report(make_run("ev03-pass", tmp_path)))
    assert keys[:3] == ["report_version", "method", "test"]
    assert (
        keys.index("signer")
        < keys.index("envelope")
        < keys.index("verdict")
        < keys.index("signature")
    )


def test_any_json_formatting_of_the_report_verifies(tmp_path: Path) -> None:
    run = make_run("ev03-pass", tmp_path)
    report = read_report(run)
    (run / "report.json").write_text(
        json.dumps(report, sort_keys=True, separators=(",", ":")), encoding="utf-8"
    )
    assert rep.verify(run, keyring()).consistent


def test_gzip_raw_files(tmp_path: Path) -> None:
    run = tmp_path / "gz"
    run.mkdir()
    g = golden()
    files = g.ev03_files(g.square_loop())
    for rel in ("raw/samples.csv", "raw/commands.csv"):
        files[rel + ".gz"] = gzip.compress(files.pop(rel), mtime=0)
    g.write_files(run, files)
    report = g.finish(run, g.declared())
    paths = [e["path"] for e in report["raw_data"]]
    assert "raw/samples.csv.gz" in paths and "raw/commands.csv.gz" in paths
    assert [e["format"] for e in report["raw_data"] if e["path"].endswith(".gz")] == [
        "be-commands-csv/1",
        "be-samples-csv/1",
    ]
    result = rep.verify(run, keyring())
    assert result.consistent and result.verdict["result"] == "PASS"


def test_accuracy_age_comes_from_the_declared_times(tmp_path: Path) -> None:
    report = read_report(make_run("ev03-pass", tmp_path))
    assert report["accuracy"]["age_s"] == 3600.0  # 18:00:00Z - 17:00:00Z


def test_the_envelope_declared_level_is_never_copied(tmp_path: Path) -> None:
    run = tmp_path / "level"
    run.mkdir()
    g = golden()
    files = g.ev03_files(g.square_loop())
    envelope = g.envelope_doc()
    envelope["declared_level"] = "BE-2"
    files["envelope.json"] = g._json(envelope)
    g.write_files(run, files)
    g.finish(run, g.declared())
    assert b"BE-2" not in (run / "report.json").read_bytes()
    assert rep.verify(run, keyring()).consistent


# --------------------------------------------------------------------------- what build refuses


def _setup(tmp_path: Path, name: str = "ev03-pass") -> Path:
    run = make_run(name, tmp_path)
    (run / "report.json").unlink()
    return run


@pytest.mark.parametrize(
    "member", ["verdict", "score", "raw_data", "statements", "envelope", "signature"]
)
def test_a_derived_member_in_the_declarations_is_refused(member: str, tmp_path: Path) -> None:
    run = _setup(tmp_path)
    doc = _declared()
    doc[member] = {"result": "PASS"}
    with pytest.raises(rep.ReportError) as exc:
        rep.build(doc, run)
    assert _codes(exc) == ["DECLARED_HAS_DERIVED"]


def test_derived_generator_members_are_refused(tmp_path: Path) -> None:
    doc = _declared()
    doc["generator"]["stream_sha256"] = "sha256:" + "0" * 64
    with pytest.raises(rep.ReportError) as exc:
        rep.build(doc, _setup(tmp_path))
    assert _codes(exc) == ["DECLARED_HAS_DERIVED"]


def test_an_unexpected_file_is_refused(tmp_path: Path) -> None:
    run = _setup(tmp_path)
    (run / "raw" / "extra.csv").write_text("x\n")
    with pytest.raises(rep.ReportError) as exc:
        rep.build(_declared(), run)
    assert _codes(exc) == ["UNEXPECTED_FILE"]


def test_a_missing_required_file_is_refused(tmp_path: Path) -> None:
    run = _setup(tmp_path)
    (run / "raw" / "samples.csv").unlink()
    with pytest.raises(rep.ReportError) as exc:
        rep.build(_declared(), run)
    assert _codes(exc) == ["REQUIRED_FILE_MISSING"]


def test_the_command_space_digest_must_match(tmp_path: Path) -> None:
    doc = _declared()
    doc["adapter"]["command_space_sha256"] = "sha256:" + "1" * 64
    with pytest.raises(rep.ReportError) as exc:
        rep.build(doc, _setup(tmp_path))
    assert _codes(exc) == ["COMMAND_SPACE_DIGEST_MISMATCH"]


def test_a_simulation_must_be_a_rehearsal(tmp_path: Path) -> None:
    g = golden()
    doc = g.declared(
        role="test",
        mode="simulation",
        instruments=[g.instrument(kind="simulation_ground_truth", external=False)],
    )
    with pytest.raises(rep.ReportError) as exc:
        rep.build(doc, _setup(tmp_path, "ev03-rehearsal-simulation"))
    assert _codes(exc) == ["SIMULATION_NOT_REHEARSAL"]


def test_core_version_must_be_this_core(tmp_path: Path) -> None:
    doc = _declared()
    doc["provenance"]["core_version"] = "9.9.9"
    with pytest.raises(rep.ReportError) as exc:
        rep.build(doc, _setup(tmp_path))
    assert _codes(exc) == ["CORE_VERSION_DIFFERS"]


def test_bad_generator_params_are_refused(tmp_path: Path) -> None:
    doc = _declared()
    doc["generator"]["params"]["strategies"] = ["teleport"]
    with pytest.raises(rep.ReportError) as exc:
        rep.build(doc, _setup(tmp_path))
    assert _codes(exc) == ["STREAM_PARAMS_INVALID"]


def test_commands_that_are_not_the_declared_stream_are_refused(tmp_path: Path) -> None:
    doc = _declared()
    doc["generator"]["seed"] = 12
    with pytest.raises(rep.ReportError) as exc:
        rep.build(doc, _setup(tmp_path))
    assert _codes(exc) == ["STREAM_MISMATCH"]


def test_the_scorer_must_be_the_tests(tmp_path: Path) -> None:
    doc = _declared()
    doc["scoring"]["method"] = "ev10-waypoints/1"
    with pytest.raises(rep.ReportError) as exc:
        rep.build(doc, _setup(tmp_path))
    assert _codes(exc) == ["DECLARED_INVALID"]


def test_unknown_declared_members_are_refused_by_the_schema(tmp_path: Path) -> None:
    doc = _declared()
    doc["level"] = "BE-3"
    with pytest.raises(rep.ReportError) as exc:
        rep.build(doc, _setup(tmp_path))
    assert _codes(exc) == ["SCHEMA_INVALID"]


def test_build_needs_a_mapping() -> None:
    with pytest.raises(TypeError):
        rep.build(["not", "a", "mapping"], ".")  # type: ignore[arg-type]


# --------------------------------------------------------------------------- profiles


def test_profile_by_name_by_object_and_by_default(tmp_path: Path) -> None:
    name = "ev03-inconclusive-short"
    run = _setup(tmp_path, name)
    by_name = rep.build(_declared(name, profile="ev03-illustrative-0.1"), run)
    assert by_name["scoring"]["profile"]["profile"] == "ev03-illustrative-0.1"
    assert by_name["flags"] == []
    doc = _declared(name)
    del doc["scoring"]
    by_default = rep.build(doc, run)
    assert by_default["scoring"] == by_name["scoring"]
    assert by_default["scoring"]["method"] == "ev03-tool-point/1"
    custom = copy.deepcopy(by_name["scoring"]["profile"])
    custom["min_duration_s"] = 1
    by_object = rep.build(_declared(name), run, profile=custom)
    assert by_object["scoring"]["profile"] == custom
    assert [f["code"] for f in by_object["flags"]] == ["NONSTANDARD_PARAMS"]
    assert by_object["verdict"]["result"] == "PASS"


def test_an_unknown_profile_name_is_refused(tmp_path: Path) -> None:
    with pytest.raises(rep.ReportError) as exc:
        rep.build(_declared(), _setup(tmp_path), profile="ev03-made-up-9")
    assert _codes(exc) == ["DECLARED_INVALID"]


# --------------------------------------------------------------------------- signing


def test_sign_report_needs_the_signers_key(tmp_path: Path) -> None:
    unsigned = rep.build(_declared(), _setup(tmp_path))
    with pytest.raises(ValueError, match="signer.kid"):
        rep.sign_report(unsigned, golden().OUTSIDER_PRIVATE)
    with pytest.raises(ValueError):
        rep.sign_report({"report_version": "9"}, golden().LAB_PRIVATE)


def test_a_key_file_from_be_keygen_signs(tmp_path: Path) -> None:
    """`be keygen` writes {"kid": ..., kty, crv, x, d}; the extra kid member is fine."""
    unsigned = rep.build(_declared(), _setup(tmp_path))
    key_file = {"kid": golden().LAB_KID, **golden().LAB_PRIVATE}
    assert signing.verify(rep.sign_report(unsigned, key_file), keyring()) == []


# --------------------------------------------------------------------------- encoder forward kinematics


def test_fk_samples_from_state() -> None:
    rows = [
        StateRow(0.0, "encoder", "j1", 0.5),
        StateRow(0.0, "fk", "tool.x", 0.3),
        StateRow(0.0, "fk", "tool.y", 0.0),
        StateRow(0.0, "fk", "tool.z", 0.2),
        StateRow(0.1, "fk", "tool.z", 0.21),
        StateRow(0.1, "fk", "tool.x", 0.31),
        StateRow(0.1, "fk", "tool.y", 0.01),
    ]
    samples = rep.fk_samples_from_state(rows)
    assert [(s.t_s, s.point, s.x_m, s.y_m, s.z_m, s.valid) for s in samples] == [
        (0.0, "tool", 0.3, 0.0, 0.2, 1),
        (0.1, "tool", 0.31, 0.01, 0.21, 1),
    ]
    assert rep.fk_samples_from_state([StateRow(0.0, "encoder", "j1", 0.5)]) is None
    from bounded_embodiment.rawdata import RawDataError

    with pytest.raises(RawDataError):
        rep.fk_samples_from_state([StateRow(0.0, "fk", "tool.x", 0.3)])  # no y, z


def test_build_with_encoder_state_rows(tmp_path: Path) -> None:
    report = read_report(make_run("ev03-pass", tmp_path))
    assert "raw/state.csv" in [e["path"] for e in report["raw_data"]]
    assert report["observations"]["encoder_fk_observation"] is None  # encoder rows only, no fk
