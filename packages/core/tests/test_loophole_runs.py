"""The loophole-search runs (vectors/loopholes/runs, made by tools/make_loophole_runs.py): each
pins what `be verify` finds today. "regression" runs get the intended verdict; "defect" runs do
not, and each names a defect written up in docs/loopholes.md. A new rule-set version that fixes
a defect gets its own vectors; these keep pinning the released rule sets."""

from __future__ import annotations

import importlib.util
import json
import re
import sys
from pathlib import Path

import pytest
from _report_helpers import BANNED, text_of

from bounded_embodiment import report as rep

REPO = Path(__file__).resolve().parents[3]
SCRIPT = REPO / "tools" / "make_loophole_runs.py"
RUNS = REPO / "vectors" / "loopholes" / "runs"
DOC = REPO / "docs" / "loopholes.md"

in_repo = pytest.mark.skipif(not SCRIPT.is_file() or not RUNS.is_dir(), reason="not in a repository checkout")
EXPECT = json.loads((RUNS / "expect.json").read_text("utf-8"))["runs"] if RUNS.is_dir() else {}
VERDICTS = ("PASS", "FAIL", "INCONCLUSIVE")


def loophole_runs():
    name = "make_loophole_runs"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    previous, sys.dont_write_bytecode = sys.dont_write_bytecode, True
    try:
        sys.modules[name] = module
        spec.loader.exec_module(module)
    finally:
        sys.dont_write_bytecode = previous
    return module


def keyring() -> dict:
    return json.loads((RUNS / "keyring.json").read_text("utf-8"))


@in_repo
@pytest.mark.parametrize("name", sorted(EXPECT))
def test_loophole_run(name: str) -> None:
    want = EXPECT[name]
    result = rep.verify(RUNS / name, keyring())
    got = loophole_runs().g.outcome(result)
    assert got == {k: want[k] for k in got}
    if want["tamper_code"]:
        assert want["tamper_code"] in got["errors"]
    assert "INTERNAL_ERROR" not in got["errors"]
    text = text_of(result)
    assert not BANNED.search(text.replace(result.run_dir, ""))


@in_repo
def test_every_directory_has_an_expectation() -> None:
    assert sorted(p.name for p in RUNS.iterdir() if p.is_dir()) == sorted(EXPECT)


@in_repo
def test_kinds_and_defect_ids() -> None:
    doc = DOC.read_text("utf-8")
    documented = set(re.findall(r"^### (L[0-9]+)\b", doc, flags=re.MULTILINE))
    used = set()
    for name, want in EXPECT.items():
        assert want["kind"] in ("regression", "defect"), name
        assert name.startswith(want["kind"]) or name.startswith("tamper-"), name
        if want["kind"] == "defect":
            assert want["intended_verdict"] in VERDICTS
            assert (want["intended_verdict"], want["intended_reasons"]) != (
                want["verdict"], want["reasons"]
            ) or name in ("defect-excursion-before-window", "defect-envelope-signature-unchecked")
            assert want["defect"] in documented, f"{want['defect']} is not in docs/loopholes.md"
            used.add(want["defect"])
            assert f"vectors/loopholes/runs/{name}" in doc, f"{name} is not cited in docs/loopholes.md"
    assert used, "no defect vectors"


@in_repo
def test_loophole_runs_regenerate_byte_identically(tmp_path: Path) -> None:
    fresh = tmp_path / "runs"
    assert loophole_runs().make_all(fresh) == []
    tree = lambda d: {  # noqa: E731
        p.relative_to(d).as_posix(): p.read_bytes() for p in sorted(d.rglob("*")) if p.is_file()
    }
    assert tree(fresh) == tree(RUNS)
