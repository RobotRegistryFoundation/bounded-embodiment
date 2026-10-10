#!/usr/bin/env python3
"""Fetch the meshes a robot folder's URDF needs, from the upstream commit pinned in its assets.lock.json.

The meshes are large and belong to their upstream project, so the repository keeps only the URDF and
a lock file with the sha256 of every mesh. This puts the meshes in <robot folder>/assets/ and refuses
any file whose hash differs from the lock.

    python tools/fetch_robot_assets.py examples/robots/so101
    python tools/fetch_robot_assets.py examples/robots/so101 --from-dir ~/src/SO-ARM100/Simulation/SO101

--from-dir copies from a local checkout instead of downloading (same hash check).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
import urllib.request
from pathlib import Path


def raw_url(source: dict, rel: str) -> str:
    repo = source["repository"].rstrip("/")
    if not repo.startswith("https://github.com/"):
        raise SystemExit(f"only GitHub sources are supported, got {repo}")
    slug = repo[len("https://github.com/"):]
    return f"https://raw.githubusercontent.com/{slug}/{source['commit']}/{source['path']}/{rel}"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("robot_dir", type=Path)
    ap.add_argument("--from-dir", type=Path, help="copy from a local checkout of the upstream folder")
    args = ap.parse_args(argv)

    lock = json.loads((args.robot_dir / "assets.lock.json").read_text())
    source, files = lock["source"], lock["files"]
    bad = 0
    for rel, want in sorted(files.items()):
        dst = args.robot_dir / rel
        if dst.is_file() and hashlib.sha256(dst.read_bytes()).hexdigest() == want["sha256"]:
            print(f"ok      {rel}")
            continue
        if args.from_dir:
            data = (args.from_dir / rel).read_bytes()
        else:
            with urllib.request.urlopen(raw_url(source, rel), timeout=60) as r:  # noqa: S310 (pinned https)
                data = r.read()
        got = hashlib.sha256(data).hexdigest()
        if got != want["sha256"]:
            print(f"BAD     {rel}: sha256 {got} != {want['sha256']}", file=sys.stderr)
            bad += 1
            continue
        dst.parent.mkdir(parents=True, exist_ok=True)
        tmp = dst.with_suffix(dst.suffix + ".part")
        tmp.write_bytes(data)
        shutil.move(tmp, dst)
        print(f"fetched {rel} ({len(data)} bytes)")
    if bad:
        print(f"{bad} file(s) did not match the lock; nothing was written for them", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
