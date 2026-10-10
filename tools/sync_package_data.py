#!/usr/bin/env python3
"""Mirror the repository's schemas/ and profiles/ into the core package.

The core package ships its schemas and profiles as package data
(bounded_embodiment/schemas/, bounded_embodiment/profiles/). The copies in the
repository root are the ones people edit; this script makes the package copies
byte-identical to them, removing package files that no longer exist at the
root. Running it twice changes nothing the second time.

    python tools/sync_package_data.py           # sync
    python tools/sync_package_data.py --check   # report drift, exit 1 if any

packages/core/tests/test_package_data.py fails while the copies have drifted.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "packages" / "core" / "src" / "bounded_embodiment"
PAIRS = (
    (ROOT / "schemas", PACKAGE / "schemas"),
    (ROOT / "profiles", PACKAGE / "profiles"),
)


def _files(directory: Path) -> dict[str, Path]:
    """Regular files under directory by relative POSIX path, skipping dotfiles and caches."""
    if not directory.is_dir():
        return {}
    found = {}
    for path in sorted(directory.rglob("*")):
        rel = path.relative_to(directory)
        if any(part.startswith(".") or part == "__pycache__" for part in rel.parts):
            continue
        if path.is_file():
            found[rel.as_posix()] = path
    return found


def plan(src: Path, dst: Path) -> tuple[list[str], list[str]]:
    """(files to copy or update, stale files to remove) for one pair of directories."""
    source, target = _files(src), _files(dst)
    to_copy = [
        rel for rel, path in source.items()
        if rel not in target or target[rel].read_bytes() != path.read_bytes()
    ]
    stale = [rel for rel in target if rel not in source]
    return to_copy, stale


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="report drift without changing anything")
    args = parser.parse_args(argv)

    drift = 0
    for src, dst in PAIRS:
        to_copy, stale = plan(src, dst)
        drift += len(to_copy) + len(stale)
        for rel in to_copy:
            print(f"{'differs' if args.check else 'copy'}: {src.name}/{rel} -> {dst.relative_to(ROOT)}/{rel}")
            if not args.check:
                target = dst / rel
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes((src / rel).read_bytes())
        for rel in stale:
            print(f"{'stale' if args.check else 'remove'}: {dst.relative_to(ROOT)}/{rel}")
            if not args.check:
                (dst / rel).unlink()
    if args.check:
        if drift:
            print(f"{drift} package data file(s) out of date; run python tools/sync_package_data.py")
            return 1
        print("package data is in sync")
        return 0
    print(f"package data synced ({drift} change(s))")
    return 0


if __name__ == "__main__":
    sys.exit(main())
