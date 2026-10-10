"""The package's schemas/ and profiles/ are byte-identical copies of the repository's.

If this fails, run ``python tools/sync_package_data.py`` from the repository root.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
PACKAGE = REPO_ROOT / "packages" / "core" / "src" / "bounded_embodiment"
SCRIPT = REPO_ROOT / "tools" / "sync_package_data.py"

pytestmark = pytest.mark.skipif(
    not (REPO_ROOT / "schemas").is_dir() or not SCRIPT.is_file(),
    reason="not running from a repository checkout",
)


def _tree(directory: Path) -> dict[str, bytes]:
    if not directory.is_dir():
        return {}
    out = {}
    for path in sorted(directory.rglob("*")):
        rel = path.relative_to(directory)
        if any(part.startswith(".") or part == "__pycache__" for part in rel.parts):
            continue
        if path.is_file():
            out[rel.as_posix()] = path.read_bytes()
    return out


def _load_script():
    spec = importlib.util.spec_from_file_location("sync_package_data", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    previous, sys.dont_write_bytecode = sys.dont_write_bytecode, True  # keep tools/ free of __pycache__
    try:
        spec.loader.exec_module(module)
    finally:
        sys.dont_write_bytecode = previous
    return module


@pytest.mark.parametrize("name", ["schemas", "profiles"])
def test_package_copy_is_identical(name: str) -> None:
    source, packaged = _tree(REPO_ROOT / name), _tree(PACKAGE / name)
    missing = sorted(set(source) - set(packaged))
    stale = sorted(set(packaged) - set(source))
    differ = sorted(rel for rel in set(source) & set(packaged) if source[rel] != packaged[rel])
    assert not (missing or stale or differ), (
        f"{name}: missing {missing}, stale {stale}, differ {differ}; "
        "run python tools/sync_package_data.py"
    )


def test_schemas_are_json_files() -> None:
    assert _tree(REPO_ROOT / "schemas"), "no schemas found"
    assert all(rel.endswith(".json") for rel in _tree(REPO_ROOT / "schemas"))


def test_sync_script_is_idempotent_and_removes_stale(tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
                                                      capsys: pytest.CaptureFixture[str]) -> None:
    script = _load_script()
    src, dst = tmp_path / "root" / "schemas", tmp_path / "pkg" / "schemas"
    src.mkdir(parents=True)
    (src / "a.json").write_bytes(b'{"a": 1}\n')
    (src / "b.json").write_bytes(b"{}\r\n")  # bytes, not text: line endings survive
    (src / ".DS_Store").write_bytes(b"junk")
    dst.mkdir(parents=True)
    (dst / "old.json").write_bytes(b"{}")
    (dst / "a.json").write_bytes(b"{}")
    monkeypatch.setattr(script, "ROOT", tmp_path)
    monkeypatch.setattr(script, "PAIRS", ((src, dst), (tmp_path / "root" / "profiles", tmp_path / "pkg" / "profiles")))

    assert script.main(["--check"]) == 1
    assert script.main([]) == 0
    assert _tree(dst) == {"a.json": b'{"a": 1}\n', "b.json": b"{}\r\n"}
    capsys.readouterr()
    assert script.main([]) == 0
    assert "(0 change(s))" in capsys.readouterr().out
    assert script.main(["--check"]) == 0
