"""``be verify``: exit status, the --json document, the text output, keyring files, and the
output shape the results repository's tools/summary.py reads."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any

import pytest
from _report_helpers import BANNED, RUNS, golden, in_repo
from bounded_embodiment import cli, signing
from bounded_embodiment import report as rep

pytestmark = in_repo

KEYS = RUNS / "keyring.json"


def run_cli(capsys: pytest.CaptureFixture[str], *argv: str) -> tuple[int, str, str]:
    code = cli.main(["verify", *argv])
    out = capsys.readouterr()
    return code, out.out, out.err


def test_exit_0_when_consistent(capsys: pytest.CaptureFixture[str]) -> None:
    code, out, _ = run_cli(capsys, str(RUNS / "ev03-fail"), "--keys", str(KEYS), "--json")
    assert code == 0
    doc = json.loads(out)
    assert doc["consistent"] is True
    assert doc["verdict"]["result"] == "FAIL"  # consistent is not passed


def test_exit_1_when_not_consistent(capsys: pytest.CaptureFixture[str]) -> None:
    code, out, _ = run_cli(
        capsys, str(RUNS / "tamper-pass-over-fail"), "--keys", str(KEYS), "--json"
    )
    assert code == 1
    assert json.loads(out)["consistent"] is False


def test_json_shape(capsys: pytest.CaptureFixture[str]) -> None:
    _, out, _ = run_cli(
        capsys, str(RUNS / "tamper-envelope-swapped"), "--keys", str(KEYS), "--json"
    )
    doc = json.loads(out)
    assert set(doc) == {
        "consistent",
        "core_version",
        "run_dir",
        "report_sha256",
        "verdict",
        "findings",
    }
    assert doc["run_dir"] == str(RUNS / "tamper-envelope-swapped")
    assert doc["report_sha256"].startswith("sha256:") and len(doc["report_sha256"]) == 71
    assert set(doc["verdict"]) == {"result", "reasons", "points_measured", "points_unmeasured"}
    for f in doc["findings"]:
        assert set(f) == {"code", "severity", "detail", "where"}
        assert f["severity"] in ("error", "warn", "info")
    assert out.isascii()


def test_text_output(capsys: pytest.CaptureFixture[str]) -> None:
    code, out, _ = run_cli(capsys, str(RUNS / "ev03-pass"), "--keys", str(KEYS))
    assert code == 0
    assert "consistent: yes" in out
    assert "verdict the files support: PASS" in out
    assert "warn: NONSTANDARD_PARAMS" in out
    code, out, _ = run_cli(capsys, str(RUNS / "tamper-unlisted-file"), "--keys", str(KEYS))
    assert code == 1
    assert "consistent: no" in out and "error: UNLISTED_FILE" in out


@pytest.mark.parametrize(
    "name", sorted(p.name for p in RUNS.iterdir() if p.is_dir()) if RUNS.is_dir() else []
)
def test_no_claim_words_in_any_output(name: str, capsys: pytest.CaptureFixture[str]) -> None:
    for flags in ([], ["--json"]):
        _, out, err = run_cli(capsys, str(RUNS / name), "--keys", str(KEYS), *flags)
        text = (out + err).replace(str(RUNS), "")
        assert not BANNED.search(text), BANNED.search(text)


def test_help_has_no_claim_words(capsys: pytest.CaptureFixture[str]) -> None:
    code, out, _ = run_cli(capsys, "--help")
    assert code == 0
    assert not BANNED.search(out)


def test_usage_errors_exit_2(capsys: pytest.CaptureFixture[str]) -> None:
    code, _, err = run_cli(capsys, str(RUNS / "ev03-pass"))
    assert code == 2 and "--keys" in err


def test_an_unusable_keyring_exits_2_with_a_finding(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    bad = tmp_path / "keys.json"
    bad.write_text('{"a": 1, "a": 2}')
    code, out, _ = run_cli(capsys, str(RUNS / "ev03-pass"), "--keys", str(bad), "--json")
    assert code == 2
    doc = json.loads(out)
    assert doc["consistent"] is False
    assert [f["code"] for f in doc["findings"]] == ["KEYRING_INVALID"]
    code, _, err = run_cli(
        capsys, str(RUNS / "ev03-pass"), "--keys", str(tmp_path / "missing.json")
    )
    assert code == 2 and "KEYRING_INVALID" in err


def test_keyring_shapes(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    g = golden()
    entry = signing.key_entry(g.LAB_PUBLIC, "golden-lab", "lab@example.org", "2026-10-01")
    revoked = dict(
        signing.key_entry(g.OUTSIDER_PUBLIC, "golden-lab", "lab@example.org", "2026-10-01"),
        revoked="2026-10-05",
    )
    shapes = {
        "map.json": {g.LAB_KID: dict(g.LAB_PUBLIC)},
        "entries.json": [entry],
        "lab.json": {"name": "Golden lab", "github_handles": ["x"], "keys": [entry, revoked]},
    }
    for name, doc in shapes.items():
        (tmp_path / name).write_text(json.dumps(doc))
        assert rep.load_keyring(tmp_path / name) == {g.LAB_KID: dict(g.LAB_PUBLIC)}
        code, _, _ = run_cli(capsys, str(RUNS / "ev03-pass"), "--keys", str(tmp_path / name))
        assert code == 0, name


def test_a_symlinked_keyring_is_refused(tmp_path: Path) -> None:
    if not hasattr(os, "symlink") or sys.platform == "win32":
        pytest.skip("needs POSIX symlinks")
    os.symlink(KEYS, tmp_path / "keys.json")
    with pytest.raises(rep.KeyringError):
        rep.load_keyring(tmp_path / "keys.json")


# --------------------------------------------------------------------------- results repository


def _warnings(obj: Any) -> list[dict]:
    """tools/summary.py's rule in the results repository: every finding-shaped object with
    severity "warn", wherever it sits in the output, is a flag."""
    found = []
    if isinstance(obj, dict):
        if obj.get("severity") == "warn" and isinstance(obj.get("code"), str):
            found.append({"code": obj["code"], "detail": str(obj.get("detail", ""))[:200]})
        for value in obj.values():
            found += _warnings(value)
    elif isinstance(obj, list):
        for value in obj:
            found += _warnings(value)
    return found


def _outputs(tmp_path: Path, capsys: pytest.CaptureFixture[str], names: list[str]) -> Path:
    out_dir = tmp_path / "be-verify"
    out_dir.mkdir()
    for name in names:
        _, out, _ = run_cli(capsys, str(RUNS / name), "--keys", str(KEYS), "--json")
        (out_dir / f"golden-lab__{name}.json").write_text(out)
    return out_dir


def test_flags_are_what_summary_py_collects(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    out_dir = _outputs(
        tmp_path, capsys, ["ev03-pass", "ev03-inconclusive-short", "tamper-unknown-key"]
    )
    flags = {p.stem: _warnings(json.loads(p.read_text())) for p in sorted(out_dir.glob("*.json"))}
    assert [f["code"] for f in flags["golden-lab__ev03-pass"]] == ["NONSTANDARD_PARAMS"]
    assert flags["golden-lab__ev03-inconclusive-short"] == []
    # errors are never flags: a run that is not consistent fails the check instead
    assert [f["code"] for f in flags["golden-lab__tamper-unknown-key"]] == ["NONSTANDARD_PARAMS"]


def _results_repo() -> Path | None:
    candidates = [
        os.environ.get("BE_RESULTS_REPO", ""),
        str(Path(__file__).resolve().parents[4] / "bounded-embodiment-results"),
    ]
    for c in candidates:
        if c and (Path(c) / "tools" / "summary.py").is_file():
            return Path(c)
    return None


def test_summary_py_flags_reads_the_output(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """Runs the results repository's own tools/summary.py when a checkout is next to this one
    (or named by BE_RESULTS_REPO)."""
    results = _results_repo()
    if results is None:
        pytest.skip("no bounded-embodiment-results checkout found")
    out_dir = _outputs(tmp_path, capsys, ["ev03-pass", "ev03-inconclusive-short", "ev10-pass"])
    proc = subprocess.run(
        [sys.executable, str(results / "tools" / "summary.py"), "flags", str(out_dir)],
        capture_output=True,
        text=True,
        check=True,
    )
    line = proc.stdout.strip()
    assert line.startswith("flags=")
    flags = json.loads(line[len("flags=") :])
    assert sorted(flags) == ["golden-lab/ev03-pass", "golden-lab/ev10-pass"]
    assert [f["code"] for f in flags["golden-lab/ev03-pass"]] == ["NONSTANDARD_PARAMS"]
