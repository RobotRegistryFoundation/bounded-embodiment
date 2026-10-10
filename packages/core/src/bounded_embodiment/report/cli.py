"""``be verify RUN_DIR --keys KEYRING [--json]``: the one check the kit, ``be submit`` and the
results CI all run.

Exit status: 0 consistent, 1 not consistent, 2 usage error or an unusable keyring. With
``--json`` the output is one JSON object: ``consistent``, ``core_version``, ``run_dir``,
``report_sha256``, ``verdict`` (recomputed from the files; null when the run could not be
scored) and ``findings`` (``code``, ``severity``, ``detail``, ``where``).
"""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Mapping
from pathlib import Path

from .. import __version__, canonical, signing
from ..findings import Finding
from .files import show
from .verify import VerifyResult, verify

__all__ = ["KeyringError", "cli_verify", "load_keyring"]

EXIT_CONSISTENT, EXIT_NOT_CONSISTENT, EXIT_USAGE = 0, 1, 2
MAX_KEYRING_BYTES = 4 * 1024 * 1024


class KeyringError(ValueError):
    """The keyring file cannot be used."""


def load_keyring(path: str | Path) -> dict[str, dict]:
    """A verify keyring (kid -> public JWK) from a JSON file holding one of: ``{kid: public JWK}``
    (what the results CI writes from the base branch), a list of key-0.1 entries, or a lab.json
    (``{"keys": [key-0.1 entries], ...}``; revoked keys are left out). Raises KeyringError."""
    p = Path(path)
    try:
        if p.is_symlink():
            raise KeyringError(f"{show(str(p))} is a symbolic link")
        data = p.read_bytes()
    except OSError as e:
        raise KeyringError(f"cannot read {show(str(p))}: {e.strerror or e}") from None
    if len(data) > MAX_KEYRING_BYTES:
        raise KeyringError(f"{show(str(p))} is larger than {MAX_KEYRING_BYTES} bytes")
    try:
        doc = canonical.loads_strict(data)
    except canonical.CanonicalError as e:
        raise KeyringError(f"{show(str(p))} is not strict JSON: {e}") from None
    try:
        if isinstance(doc, Mapping) and isinstance(doc.get("keys"), list):
            return signing.keyring_from_entries(doc["keys"])
        if isinstance(doc, list):
            return signing.keyring_from_entries(doc)
    except ValueError as e:
        raise KeyringError(str(e)) from None
    if isinstance(doc, Mapping):
        ring = {}
        for kid, jwk in doc.items():
            if not isinstance(jwk, Mapping):
                raise KeyringError(f"keyring entry {kid!r} is not a JWK object")
            ring[kid] = dict(jwk)
        return ring
    raise KeyringError("a keyring is {kid: public JWK}, a list of key entries, or a lab.json")


def _write(text: str) -> None:
    """Write to stdout whatever its encoding (a hostile file name must not crash the output)."""
    stream = sys.stdout
    encoding = getattr(stream, "encoding", None) or "utf-8"
    stream.write(text.encode(encoding, "backslashreplace").decode(encoding, "replace"))


def _text(result: VerifyResult) -> str:
    lines = [f"be verify {result.run_dir}", f"core {result.core_version}"]
    if result.report_sha256:
        lines.append(f"report.json {result.report_sha256}")
    lines.append("consistent: yes" if result.consistent else "consistent: no")
    v = result.verdict
    if isinstance(v, dict):
        codes = ", ".join(str(r.get("code")) for r in v.get("reasons", []) if isinstance(r, dict))
        lines.append(
            f"verdict the files support: {v.get('result')}" + (f" ({codes})" if codes else "")
        )
    for f in result.findings:
        at = f" at {f.where}" if f.where else ""
        lines.append(f"{f.severity}: {f.code}{at}: {f.detail}")
    if result.consistent:
        lines.append(
            "Consistent means the signature, the files and the derived results agree; it does not mean "
            "the machine met any requirement."
        )
    return "\n".join(lines) + "\n"


def cli_verify(argv: list[str]) -> int:
    """Run ``be verify`` with ``argv`` (the arguments after ``verify``); return the exit status."""
    parser = argparse.ArgumentParser(
        prog="be verify",
        description=(
            "Check that a run directory's signed report agrees with its files: signatures against "
            "the keyring, file hashes, the regenerated command stream, and the score and verdict "
            "recomputed from the raw data. Exit 0 when consistent. Consistent is not the same as "
            "the machine meeting a requirement: a FAIL report is consistent when its data supports FAIL."
        ),
    )
    parser.add_argument("run_dir", help="the run directory (report.json and its files)")
    parser.add_argument(
        "--keys",
        required=True,
        metavar="KEYRING",
        help="JSON file: {kid: public JWK}, a list of key-0.1 entries, or a lab.json; the only keys trusted",
    )
    parser.add_argument("--json", action="store_true", help="print one JSON object")
    try:
        args = parser.parse_args(argv)
    except SystemExit as e:  # argparse: 0 after --help, 2 for a usage error
        return e.code if isinstance(e.code, int) else EXIT_USAGE

    try:
        keyring = load_keyring(args.keys)
    except KeyringError as e:
        finding = Finding("KEYRING_INVALID", "error", str(e), "--keys")
        if args.json:
            out = {
                "consistent": False,
                "core_version": __version__,
                "run_dir": show(args.run_dir),
                "report_sha256": None,
                "verdict": None,
                "findings": [
                    {
                        "code": finding.code,
                        "severity": finding.severity,
                        "detail": finding.detail,
                        "where": finding.where,
                    }
                ],
            }
            _write(json.dumps(out, indent=2, sort_keys=True, ensure_ascii=True) + "\n")
        else:
            print(f"be verify: {finding.code}: {finding.detail}", file=sys.stderr)
        return EXIT_USAGE

    result = verify(args.run_dir, keyring)
    if args.json:
        _write(json.dumps(result.as_json(), indent=2, sort_keys=True, ensure_ascii=True) + "\n")
    else:
        _write(_text(result))
    return EXIT_CONSISTENT if result.consistent else EXIT_NOT_CONSISTENT
