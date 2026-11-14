"""report.verify: the golden runs, every tamper case with its own code, hostile run directories
(it never raises and never follows a link), determinism and wording."""

from __future__ import annotations

import gzip
import importlib
import json
import os
import random
import sys
from pathlib import Path

import pytest
from _report_helpers import (
    BANNED,
    RUNS,
    codes,
    copy_run,
    golden,
    in_repo,
    keyring,
    read_report,
    resign,
    text_of,
)
from bounded_embodiment import report as rep
from bounded_embodiment.report import derive as derive_module
from bounded_embodiment.report import files as files_module

pytestmark = in_repo

EXPECT = (
    json.loads((RUNS / "expect.json").read_text(encoding="utf-8"))["runs"] if RUNS.is_dir() else {}
)
RELAYED_PREFIXES = ("ENVELOPE_", "COMMAND_SPACE_", "WAYPOINTS_", "RAW_", "SCORING_")


def _check_result(result: rep.VerifyResult) -> None:
    """What holds for every result: no internal error, no claim word, every code known."""
    assert "INTERNAL_ERROR" not in codes(result), [f.as_dict() for f in result.findings]
    text = text_of(result)
    assert not BANNED.search(text.replace(result.run_dir, "")), BANNED.search(text)
    json.loads(text)
    for f in result.findings:
        if f.severity == "warn" and f.where.startswith("/flags/"):
            continue  # scorer flags keep the scorer's codes
        assert f.code in rep.CODES or f.code.startswith(RELAYED_PREFIXES), f.code


# --------------------------------------------------------------------------- golden runs


@pytest.mark.parametrize("name", sorted(EXPECT))
def test_golden_run(name: str) -> None:
    want = EXPECT[name]
    result = rep.verify(RUNS / name, keyring())
    got = golden().outcome(result)
    assert got == {k: want[k] for k in got}
    if want["tamper_code"]:
        assert want["tamper_code"] in got["errors"]
    _check_result(result)


def test_every_golden_directory_has_an_expectation() -> None:
    assert sorted(p.name for p in RUNS.iterdir() if p.is_dir()) == sorted(EXPECT)


def test_goldens_regenerate_byte_identically(tmp_path: Path) -> None:
    """The goldens are a pure function of the script and core: same bytes on every platform and
    Python (this also pins the determinism of build, the generators and the scorers)."""
    fresh = tmp_path / "runs"
    assert golden().make_all(fresh) == []
    want = {
        p.relative_to(fresh).as_posix(): p.read_bytes()
        for p in sorted(fresh.rglob("*"))
        if p.is_file()
    }
    have = {
        p.relative_to(RUNS).as_posix(): p.read_bytes()
        for p in sorted(RUNS.rglob("*"))
        if p.is_file()
    }
    assert sorted(want) == sorted(have)
    assert [rel for rel in want if want[rel] != have[rel]] == []


def test_verify_is_deterministic() -> None:
    for name in ("ev03-pass", "tamper-envelope-swapped", "tamper-embedded-key", "ev10-fail-freeze"):
        a, b = rep.verify(RUNS / name, keyring()), rep.verify(RUNS / name, keyring())
        assert text_of(a) == text_of(b)
        assert a.diffs == b.diffs


def test_the_result_carries_recomputed_blocks_and_diffs() -> None:
    result = rep.verify(RUNS / "tamper-pass-over-fail", keyring())
    assert set(result.derived) >= {
        "envelope",
        "score",
        "verdict",
        "raw_data",
        "statements",
        "generator",
    }
    assert result.verdict["result"] == "FAIL"
    assert (
        '/verdict/result: the report has "PASS"; the files give "FAIL"' in result.diffs["verdict"]
    )
    assert result.report["verdict"]["result"] == "PASS"


def test_a_mismatch_detail_is_a_readable_diff() -> None:
    result = rep.verify(RUNS / "tamper-envelope-swapped", keyring())
    found = {f.code: f for f in result.findings}
    assert "/envelope/hash: the report has" in found["ENVELOPE_HASH_MISMATCH"].detail
    assert found["FILE_HASH_MISMATCH"].where == "envelope.json"


# --------------------------------------------------------------------------- hostile report.json


@pytest.mark.parametrize(
    ("content", "code"),
    [
        (b"", "JSON_INVALID"),
        (b"not json at all", "JSON_INVALID"),
        (b"\xff\xfe\x00garbage", "JSON_INVALID"),
        (b'\xef\xbb\xbf{"report_version": "0.1"}', "JSON_INVALID"),
        (b'{"report_version": "0.1", "report_version": "0.1"}', "JSON_DUPLICATE_KEY"),
        (b'{"x": NaN}', "JSON_INVALID"),
        (b'{"x": Infinity}', "JSON_INVALID"),
        (b'{"x": 1e400}', "JSON_INVALID"),
        (b"[" * 100000 + b"]" * 100000, "JSON_INVALID"),
        (b"[1, 2, 3]", "REPORT_NOT_OBJECT"),
        (b'"a string"', "REPORT_NOT_OBJECT"),
        (b'{"report_version": "0.3"}', "REPORT_VERSION_UNSUPPORTED"),
        (b"{}", "REPORT_VERSION_UNSUPPORTED"),
    ],
    # Short ids: pytest names tmp_path after the test id, and a 200,000-character id is longer
    # than a Windows path may be.
    ids=[
        "empty", "not-json", "binary", "bom", "duplicate-key", "nan", "infinity", "huge-number",
        "deep-nesting", "array", "string", "version-0.3", "no-version",
    ],
)
def test_hostile_report_json(content: bytes, code: str, tmp_path: Path) -> None:
    run = copy_run("ev03-pass", tmp_path)
    (run / "report.json").write_bytes(content)
    result = rep.verify(run, keyring())
    assert not result.consistent
    assert code in codes(result)
    _check_result(result)


def test_huge_numbers_in_a_report(tmp_path: Path) -> None:
    run = copy_run("ev03-pass", tmp_path)
    report = read_report(run)
    report["generator"]["seed"] = 2**80
    report["score"]["sigma_jitter_m"] = 10**400
    raw = json.dumps(report).encode()
    (run / "report.json").write_bytes(raw)
    result = rep.verify(run, keyring())
    assert {"SCHEMA_INVALID", "BAD_SIGNATURE"} <= set(codes(result))
    _check_result(result)


def test_a_report_with_any_member_mangled_never_raises(tmp_path: Path) -> None:
    """Deterministic fuzzing: drop or retype members of a valid report, re-sign, verify."""
    run = copy_run("ev03-pass", tmp_path)
    base = read_report(run)
    rng = random.Random(7)
    values = [None, True, -1, 2**60, 1e300, "", "x" * 300, [], {}, [None], {"kty": "OKP"}]
    paths: list[list] = []

    def walk(node: object, path: list) -> None:
        paths.append(path)
        if isinstance(node, dict):
            for k, v in node.items():
                walk(v, [*path, k])
        elif isinstance(node, list) and node:
            walk(node[0], [*path, 0])

    walk(base, [])
    paths = [p for p in paths if p and p[0] != "signature"]  # resign() replaces the signature
    for i in range(120):
        doc = json.loads(json.dumps(base))
        path = rng.choice(paths)
        parent = doc
        for key in path[:-1]:
            parent = parent[key]
        if rng.random() < 0.3 and isinstance(parent, dict):
            del parent[path[-1]]
        else:
            parent[path[-1]] = rng.choice(values)
        try:
            resign(run, doc)
        except ValueError:  # 2**60 cannot be signed (RFC 8785); write it as it is
            (run / "report.json").write_text(json.dumps(doc), encoding="utf-8")
        result = rep.verify(run, keyring())
        # A mangled report may stay consistent only if the change touched an optional DECLARED
        # member (dropping software[].role, say): nothing derived from the data may change.
        if result.consistent:
            for key in (*rep.DERIVED_BLOCKS, "envelope", "generator"):
                assert doc.get(key) == base.get(key), (i, path)
        _check_result(result)


# --------------------------------------------------------------------------- hostile run directories


def test_missing_and_non_directory_run_dirs(tmp_path: Path) -> None:
    for target in (tmp_path / "nothing-here", tmp_path / "a-file"):
        if target.name == "a-file":
            target.write_text("x")
        result = rep.verify(target, keyring())
        assert codes(result) == ["RUN_DIR_INVALID"]
        _check_result(result)


def test_an_empty_run_dir(tmp_path: Path) -> None:
    result = rep.verify(tmp_path, keyring())
    assert codes(result) == ["REPORT_MISSING"]


@pytest.mark.skipif(
    not hasattr(os, "symlink") or sys.platform == "win32", reason="needs POSIX symlinks"
)
class TestSymlinks:
    def test_a_linked_raw_file_is_refused_and_never_read(self, tmp_path: Path) -> None:
        run = copy_run("ev03-pass", tmp_path)
        outside = tmp_path / "outside.csv"
        outside.write_bytes((run / "raw" / "samples.csv").read_bytes())
        (run / "raw" / "samples.csv").unlink()
        os.symlink(outside, run / "raw" / "samples.csv")
        result = rep.verify(run, keyring())
        assert codes(result) == ["SYMLINK_REFUSED"]
        assert result.verdict is None or "samples" not in str(result.derived.get("raw_data"))
        _check_result(result)

    def test_a_link_to_a_system_file_is_not_followed(self, tmp_path: Path) -> None:
        run = copy_run("ev03-pass", tmp_path)
        os.symlink("/etc/passwd", run / "notes" / "photos.jpg")
        result = rep.verify(run, keyring())
        assert codes(result) == ["SYMLINK_REFUSED"]
        assert "root:" not in text_of(result)

    def test_a_linked_directory_is_not_entered(self, tmp_path: Path) -> None:
        run = copy_run("ev03-pass", tmp_path)
        os.symlink("/", run / "raw" / "instrument" / "rootfs")
        result = rep.verify(run, keyring())
        assert codes(result) == ["SYMLINK_REFUSED"]

    def test_a_linked_report_is_refused(self, tmp_path: Path) -> None:
        run = copy_run("ev03-pass", tmp_path)
        real = tmp_path / "elsewhere.json"
        (run / "report.json").rename(real)
        os.symlink(real, run / "report.json")
        result = rep.verify(run, keyring())
        assert codes(result) == ["SYMLINK_REFUSED"]
        assert result.report_sha256 is None

    def test_a_linked_run_dir_is_refused(self, tmp_path: Path) -> None:
        run = copy_run("ev03-pass", tmp_path)
        os.symlink(run, tmp_path / "alias")
        assert codes(rep.verify(tmp_path / "alias", keyring())) == ["RUN_DIR_INVALID"]


@pytest.mark.skipif(not hasattr(os, "mkfifo"), reason="needs FIFOs")
def test_a_fifo_is_refused_without_blocking(tmp_path: Path) -> None:
    run = copy_run("ev03-pass", tmp_path)
    os.mkfifo(run / "raw" / "instrument" / "pipe")
    result = rep.verify(run, keyring())
    assert codes(result) == ["NOT_A_REGULAR_FILE"]


def test_a_hidden_file_is_unexpected_and_unlisted(tmp_path: Path) -> None:
    run = copy_run("ev03-pass", tmp_path)
    (run / ".DS_Store").write_bytes(b"\x00\x01")
    assert codes(rep.verify(run, keyring())) == ["UNEXPECTED_FILE", "UNLISTED_FILE"]


@pytest.mark.skipif(
    sys.platform in ("win32", "darwin"), reason="needs arbitrary bytes in file names"
)
def test_a_file_name_that_is_not_utf8(tmp_path: Path) -> None:
    run = copy_run("ev03-pass", tmp_path)
    (run / "raw" / "instrument").joinpath(os.fsdecode(b"bad\xffname\x1b[31m")).write_bytes(b"x")
    result = rep.verify(run, keyring())
    assert codes(result) == ["UNEXPECTED_FILE", "UNLISTED_FILE"]
    text = json.dumps(result.as_json(), ensure_ascii=True)
    assert "\\\\xff" in text and "\\u001b" not in text
    _check_result(result)


def test_a_listed_file_that_is_missing(tmp_path: Path) -> None:
    run = copy_run("ev03-pass", tmp_path)
    (run / "notes" / "NOTES.md").unlink()
    assert codes(rep.verify(run, keyring())) == ["FILE_MISSING"]


def test_a_missing_required_file_listed_in_raw_data(tmp_path: Path) -> None:
    run = copy_run("ev03-pass", tmp_path)
    (run / "raw" / "commands.csv").unlink()
    assert codes(rep.verify(run, keyring())) == ["FILE_MISSING"]


def test_two_files_for_one_role(tmp_path: Path) -> None:
    run = copy_run("ev03-pass", tmp_path)
    data = (run / "raw" / "samples.csv").read_bytes()
    (run / "raw" / "samples.csv.gz").write_bytes(gzip.compress(data, mtime=0))
    assert codes(rep.verify(run, keyring())) == ["DUPLICATE_RAW_FILE", "UNLISTED_FILE"]


def test_raw_data_out_of_order(tmp_path: Path) -> None:
    run = copy_run("ev03-pass", tmp_path)
    report = read_report(run)
    report["raw_data"].reverse()
    resign(run, report)
    assert codes(rep.verify(run, keyring())) == ["RAW_DATA_MISMATCH"]


def test_raw_data_listing_report_json(tmp_path: Path) -> None:
    run = copy_run("ev03-pass", tmp_path)
    report = read_report(run)
    report["raw_data"].append(
        {
            "path": "report.json",
            "sha256": "sha256:" + "0" * 64,
            "bytes": 1,
            "role": "notes",
            "format": "json",
        }
    )
    resign(run, report)
    assert "RAW_DATA_MISMATCH" in codes(rep.verify(run, keyring()))


def test_raw_data_naming_a_path_outside(tmp_path: Path) -> None:
    run = copy_run("ev03-pass", tmp_path)
    report = read_report(run)
    report["raw_data"].append(
        {
            "path": "../../etc/passwd",
            "sha256": "sha256:" + "0" * 64,
            "bytes": 1,
            "role": "notes",
            "format": "bytes",
        }
    )
    resign(run, report)
    result = rep.verify(run, keyring())
    assert {"SCHEMA_INVALID", "FILE_MISSING"} <= set(codes(result))
    _check_result(result)


# --------------------------------------------------------------------------- hostile raw files


def test_a_gzip_bomb_is_refused(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    run = copy_run("ev03-pass", tmp_path)
    samples = run / "raw" / "samples.csv"
    samples.write_bytes(gzip.compress(samples.read_bytes() + b"\n" * 5000, mtime=0))
    samples.rename(run / "raw" / "samples.csv.gz")
    report = read_report(run)
    for entry in report["raw_data"]:
        if entry["path"] == "raw/samples.csv":
            entry["path"] = "raw/samples.csv.gz"
    monkeypatch.setattr(derive_module, "MAX_DECOMPRESSED_BYTES", 4096)
    resign(run, report)
    result = rep.verify(run, keyring())
    assert "RAW_TOO_LARGE" in codes(result)
    _check_result(result)


@pytest.mark.parametrize(
    ("name", "content", "code"),
    [
        ("raw/samples.csv", b"t_s,point\n1,2\n", "RAW_HEADER"),
        ("raw/samples.csv", b"\x1f\x8b\x08\x00broken gzip", "RAW_UNREADABLE"),
        ("raw/accuracy.csv", b"\xff\xfe", "RAW_ENCODING"),
        (
            "raw/commands.csv",
            b'seq,t_sent_s,t_done_s,strategy,payload_json,result,error_kind\n0,1e999,,x,"{}",ok,\n',
            "RAW_VALUE",
        ),
        (
            "raw/commands.csv",
            b"seq,t_sent_s,t_done_s,strategy,payload_json,result,error_kind\n"
            + b"9" * 5000
            + b",0,,x,,,\n",
            "RAW_VALUE",
        ),
        (
            "envelope.json",
            b'{"envelope_version": "0.2", "envelope_version": "0.2"}',
            "ENVELOPE_PARSE_ERROR",
        ),
        ("command-space.json", b"[]", "COMMAND_SPACE_NOT_OBJECT"),
    ],
)
def test_hostile_raw_files(name: str, content: bytes, code: str, tmp_path: Path) -> None:
    run = copy_run("ev03-pass", tmp_path)
    (run / name).write_bytes(content)
    golden()._relist(run, report := read_report(run), name)
    resign(run, report)
    result = rep.verify(run, keyring())
    assert code in codes(result), [f.as_dict() for f in result.findings]
    _check_result(result)


def test_a_gz_name_that_is_not_gzip(tmp_path: Path) -> None:
    run = copy_run("ev03-pass", tmp_path)
    (run / "raw" / "commands.csv").rename(run / "raw" / "commands.csv.gz")
    report = read_report(run)
    for entry in report["raw_data"]:
        if entry["path"] == "raw/commands.csv":
            entry["path"] = "raw/commands.csv.gz"
    resign(run, report)
    assert "RAW_UNREADABLE" in codes(rep.verify(run, keyring()))


def test_a_report_over_the_size_limit(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(files_module, "MAX_REPORT_BYTES", 1000)
    result = rep.verify(copy_run("ev03-pass", tmp_path), keyring())
    assert codes(result) == ["FILE_TOO_LARGE"]


# --------------------------------------------------------------------------- keys and signatures


def test_keyring_must_be_a_mapping() -> None:
    result = rep.verify(RUNS / "ev03-pass", ["not", "a", "keyring"])  # type: ignore[arg-type]
    assert "KEYRING_INVALID" in codes(result)


def test_an_empty_keyring() -> None:
    assert codes(rep.verify(RUNS / "ev03-pass", {})) == ["UNKNOWN_KID"]


def test_a_keyring_entry_whose_kid_is_not_its_thumbprint() -> None:
    """The keyring maps the lab's kid to some other key: the thumbprint is recomputed."""
    ring = {golden().LAB_KID: dict(golden().OUTSIDER_PUBLIC)}
    assert codes(rep.verify(RUNS / "ev03-pass", ring)) == ["UNKNOWN_KID"]


def test_an_unsigned_report(tmp_path: Path) -> None:
    run = copy_run("ev03-pass", tmp_path)
    report = read_report(run)
    del report["signature"]
    (run / "report.json").write_bytes(rep.to_json(report))
    assert codes(rep.verify(run, keyring())) == ["NO_SIGNATURE"]


def test_signer_kid_must_be_a_valid_signer(tmp_path: Path) -> None:
    run = copy_run("ev03-pass", tmp_path)
    report = read_report(run)
    from bounded_embodiment import signing

    report["signer"]["kid"] = signing.thumbprint(golden().OUTSIDER_PUBLIC)
    resign(run, report)  # signed by the lab key, but signer.kid names another key
    assert codes(rep.verify(run, keyring())) == ["SIGNER_KID_MISMATCH"]


def test_a_cosignature_by_an_unregistered_key_is_an_error(tmp_path: Path) -> None:
    from bounded_embodiment import signing

    run = copy_run("ev03-pass", tmp_path)
    cosigned = signing.sign(read_report(run), golden().OUTSIDER_PRIVATE)
    (run / "report.json").write_bytes(rep.to_json(cosigned))
    assert codes(rep.verify(run, keyring())) == ["UNKNOWN_KID"]


def test_a_tampered_signature(tmp_path: Path) -> None:
    run = copy_run("ev03-pass", tmp_path)
    report = read_report(run)
    report["run"]["operator"] = "someone else"
    (run / "report.json").write_bytes(rep.to_json(report))
    assert codes(rep.verify(run, keyring())) == ["BAD_SIGNATURE"]


# --------------------------------------------------------------------------- declarations and derived members


def _edit(tmp_path: Path, name: str, change) -> rep.VerifyResult:
    run = copy_run(name, tmp_path)
    report = read_report(run)
    change(report)
    resign(run, report)
    result = rep.verify(run, keyring())
    _check_result(result)
    return result


def test_simulation_without_rehearsal(tmp_path: Path) -> None:
    result = _edit(tmp_path, "ev03-rehearsal-simulation", lambda r: r.update(role="test"))
    assert codes(result) == ["SIMULATION_NOT_REHEARSAL"]


def test_run_times_inverted(tmp_path: Path) -> None:
    result = _edit(
        tmp_path, "ev03-pass", lambda r: r["run"].update(ended_at="2026-10-10T17:59:59Z")
    )
    assert codes(result) == ["RUN_TIMES_INVERTED"]


def test_accuracy_check_after_the_run_start(tmp_path: Path) -> None:
    result = _edit(
        tmp_path,
        "ev03-pass",
        lambda r: r["instruments"][0]["accuracy_check"].update(at="2026-10-10T18:00:01Z"),
    )
    assert codes(result) == ["ACCURACY_CHECK_AFTER_RUN_START"]


def test_accuracy_points_must_match_the_file(tmp_path: Path) -> None:
    result = _edit(
        tmp_path, "ev03-pass", lambda r: r["instruments"][0]["accuracy_check"].update(points=12)
    )
    assert codes(result) == ["ACCURACY_CHECK_POINTS_MISMATCH"]


def test_a_stale_check_changes_the_verdict(tmp_path: Path) -> None:
    result = _edit(
        tmp_path,
        "ev03-pass",
        lambda r: r["instruments"][0]["accuracy_check"].update(at="2026-10-09T17:00:00Z"),
    )
    assert codes(result) == ["DERIVED_MISMATCH", "VERDICT_MISMATCH"]
    assert result.verdict["result"] == "INCONCLUSIVE"
    assert [r["code"] for r in result.verdict["reasons"]] == ["ACCURACY_CHECK_STALE"]


def test_calibration_file_missing(tmp_path: Path) -> None:
    result = _edit(
        tmp_path,
        "ev03-pass",
        lambda r: r["instruments"][0]["calibration"].update(files=["raw/instrument/gone.yaml"]),
    )
    assert codes(result) == ["CALIBRATION_FILE_MISSING"]


def test_control_stream_on_a_test_run(tmp_path: Path) -> None:
    result = _edit(tmp_path, "ev03-control", lambda r: r.update(role="test"))
    assert codes(result) == ["GENERATOR_ROLE_MISMATCH"]


def test_waypoint_stream_on_ev03(tmp_path: Path) -> None:
    def change(r: dict) -> None:
        r["generator"].update(
            name="be-waypoints", params={"cycles": 1, "commands_per_visit": 1, "hz": 50.0}
        )

    result = _edit(tmp_path, "ev03-pass", change)
    assert "GENERATOR_TEST_MISMATCH" in codes(result)


def test_a_stream_too_large_to_regenerate(tmp_path: Path) -> None:
    result = _edit(
        tmp_path,
        "ev03-pass",
        lambda r: r["generator"]["params"].update(seconds=86400.0, base_hz=10000.0),
    )
    assert codes(result) == ["STREAM_TOO_LARGE"]


def test_generator_params_core_refuses(tmp_path: Path) -> None:
    result = _edit(
        tmp_path, "ev03-pass", lambda r: r["generator"]["params"].update(center={"q": 1.0})
    )
    assert codes(result) == ["STREAM_PARAMS_INVALID"]


def test_a_different_seed(tmp_path: Path) -> None:
    result = _edit(tmp_path, "ev03-pass", lambda r: r["generator"].update(seed=12))
    assert codes(result) == ["STREAM_DIGEST_MISMATCH", "STREAM_MISMATCH"]


def test_statements_follow_the_instrument(tmp_path: Path) -> None:
    result = _edit(
        tmp_path,
        "ev03-pass",
        lambda r: r["statements"].update(measurement_source="internal_instrument"),
    )
    assert codes(result) == ["STATEMENTS_MISMATCH"]


def test_flags_removed(tmp_path: Path) -> None:
    result = _edit(tmp_path, "ev03-pass", lambda r: r.update(flags=[]))
    assert codes(result) == ["DERIVED_MISMATCH"]
    assert [f.where for f in result.findings if f.code == "DERIVED_MISMATCH"] == ["/flags"]


def test_commands_compared_wrong(tmp_path: Path) -> None:
    result = _edit(tmp_path, "ev03-pass", lambda r: r["generator"].update(commands_compared=1))
    assert codes(result) == ["STREAM_COUNT_MISMATCH"]


def test_a_report_from_another_core(monkeypatch: pytest.MonkeyPatch) -> None:
    verify_module = importlib.import_module("bounded_embodiment.report.verify")
    monkeypatch.setattr(verify_module, "__version__", "0.2.0")
    result = rep.verify(RUNS / "ev03-pass", keyring())
    assert result.consistent
    assert codes(result, "warn") == ["CORE_VERSION_DIFFERS", "NONSTANDARD_PARAMS"]


def test_every_code_is_documented() -> None:
    doc = (RUNS.parents[1] / "docs" / "seams" / "report.md").read_text(encoding="utf-8")
    missing = [code for code in rep.CODES if code not in doc]
    assert missing == []
    assert all(severity in ("error", "warn", "info") for severity, _ in rep.CODES.values())
