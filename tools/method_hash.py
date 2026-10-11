#!/usr/bin/env python3
"""Print the SHA-256 of the Bounded Embodiment method text, or compare a report's with it.

A test report cites the method it followed in ``method.text_sha256``: ``"sha256:"`` followed by
the lowercase hex SHA-256 of the method file's bytes, exactly as published (UTF-8, LF line
endings). This script computes that value from the file in this repository.

    python tools/method_hash.py                       # prints sha256:<hex> and the file's path
    python tools/method_hash.py --check REPORT.json   # does the report cite this text?
    python tools/method_hash.py --text FILE           # hash another copy or version of the text

``--check`` takes a report.json, or a run directory that holds one. Exit status: 0 when the
hash is printed, or when the report's ``method.text_sha256`` equals it; 1 when it differs or the
report has none; 2 when a file cannot be read or the report is not strict JSON.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
METHOD_TEXT = ROOT / "method" / "bounded-embodiment-method-v0.1.md"

EXIT_MATCH, EXIT_DIFFERS, EXIT_ERROR = 0, 1, 2


def text_sha256(path: Path) -> str:
    """``"sha256:"`` + the lowercase hex SHA-256 of the file's bytes."""
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def _object(pairs: list[tuple[str, object]]) -> dict:
    out: dict = {}
    for key, value in pairs:
        if key in out:
            raise ValueError(f"duplicate member {key!r}")
        out[key] = value
    return out


def _constant(name: str) -> object:
    raise ValueError(f"{name} is not JSON")


def report_method(path: Path) -> tuple[Path, object]:
    """The report's path and its ``method`` member (None when absent). ``path`` is a report.json
    or a run directory that holds one. The report is parsed strictly: a repeated member name,
    NaN or an infinity is an error (ValueError)."""
    if path.is_dir():
        path = path / "report.json"
    doc = json.loads(
        path.read_text(encoding="utf-8"), object_pairs_hook=_object, parse_constant=_constant
    )
    if not isinstance(doc, dict):
        # A report of the wrong shape is bad input, reported like a JSON error.
        raise ValueError("a report is a JSON object")  # noqa: TRY004
    return path, doc.get("method")


def _show(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return str(path)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="method_hash.py",
        description="Print the SHA-256 of the method text, or compare a report's "
        "method.text_sha256 with it.",
    )
    parser.add_argument(
        "--text",
        type=Path,
        default=METHOD_TEXT,
        help=f"the method text to hash (default: {_show(METHOD_TEXT)})",
    )
    parser.add_argument(
        "--check",
        type=Path,
        metavar="REPORT",
        help="a report.json, or a run directory holding one, whose method.text_sha256 to compare",
    )
    args = parser.parse_args(argv)

    try:
        digest = text_sha256(args.text)
    except OSError as e:
        print(f"method_hash: cannot read {args.text}: {e.strerror or e}", file=sys.stderr)
        return EXIT_ERROR
    if args.check is None:
        print(f"{digest}  {_show(args.text)}")
        return EXIT_MATCH

    try:
        report, method = report_method(args.check)
    except OSError as e:
        print(f"method_hash: cannot read {args.check}: {e.strerror or e}", file=sys.stderr)
        return EXIT_ERROR
    except ValueError as e:  # json.JSONDecodeError is a ValueError
        print(f"method_hash: {args.check} is not a strict JSON report: {e}", file=sys.stderr)
        return EXIT_ERROR

    cited = method.get("text_sha256") if isinstance(method, dict) else None
    print(f"method text  {digest}  {_show(args.text)}")
    if not isinstance(cited, str):
        print(f"report       no method.text_sha256  {report}")
        print("match: no")
        return EXIT_DIFFERS
    label = ""
    if isinstance(method, dict):
        label = f" (method {method.get('id')!s} {method.get('version')!s})"
    print(f"report       {cited}  {report}{label}")
    if cited == digest:
        print("match: yes")
        return EXIT_MATCH
    print("match: no")
    return EXIT_DIFFERS


if __name__ == "__main__":
    sys.exit(main())
