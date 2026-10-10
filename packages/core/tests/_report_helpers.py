"""Shared pieces of the report tests: the golden-run script (tools/make_golden_runs.py, which
builds TEST-ONLY synthetic runs with core's own generators) and small helpers to copy, edit and
re-sign runs."""

from __future__ import annotations

import importlib.util
import json
import re
import shutil
import sys
from pathlib import Path

import pytest
from bounded_embodiment import report as rep
from bounded_embodiment import signing

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = REPO_ROOT / "tools" / "make_golden_runs.py"
RUNS = REPO_ROOT / "vectors" / "runs"

in_repo = pytest.mark.skipif(
    not SCRIPT.is_file() or not RUNS.is_dir(), reason="not running from a repository checkout"
)

# Words `be verify` must never print (CI says "consistent", never these).
BANNED = re.compile(
    r"(?<![A-Za-z])(passed|verified|certified|compliant|safe|approved)(?![A-Za-z])", re.IGNORECASE
)


def golden():
    """tools/make_golden_runs.py as a module (loaded once)."""
    name = "make_golden_runs"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    previous, sys.dont_write_bytecode = (
        sys.dont_write_bytecode,
        True,
    )  # keep tools/ free of __pycache__
    try:
        sys.modules[name] = module
        spec.loader.exec_module(module)
    finally:
        sys.dont_write_bytecode = previous
    return module


def keyring() -> dict:
    return json.loads((RUNS / "keyring.json").read_text(encoding="utf-8"))


def copy_run(name: str, tmp_path: Path) -> Path:
    """A copy of a golden run directory to edit."""
    dst = tmp_path / name
    shutil.copytree(RUNS / name, dst)
    return dst


def make_run(name: str, tmp_path: Path) -> Path:
    """A fresh golden-kind run built by the script into tmp_path."""
    run = tmp_path / name
    run.mkdir()
    golden().RUNS[name](run)
    return run


def read_report(run: Path) -> dict:
    return json.loads((run / "report.json").read_text(encoding="utf-8"))


def resign(run: Path, report: dict, key: dict | None = None) -> None:
    """Write report signed again (signature replaced) with the TEST-ONLY lab key, or key."""
    unsigned = {k: v for k, v in report.items() if k != "signature"}
    (run / "report.json").write_bytes(
        rep.to_json(signing.sign(unsigned, key or golden().LAB_PRIVATE))
    )


def codes(result: rep.VerifyResult, severity: str = "error") -> list[str]:
    return sorted(f.code for f in result.findings if f.severity == severity)


def text_of(result: rep.VerifyResult) -> str:
    return json.dumps(result.as_json(), sort_keys=True)
