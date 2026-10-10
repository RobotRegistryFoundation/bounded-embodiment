"""Check a run directory: is its signed report consistent with its files?

``verify(run_dir, keyring)`` never raises. It checks, in this order:

1. the run directory: no symbolic links (refused, never followed), only regular files, sizes;
2. report.json: strict JSON (duplicate keys refused), ``report_version`` 0.1, the schema;
3. keys: no key carried in the report is used; every signature entry against the keyring only
   (kid = the RFC 7638 thumbprint of the keyring key, recomputed), at least one valid Ed25519
   signature, and ``signer.kid`` the kid of a valid one;
4. files: every file ``raw_data`` lists is there with its sha256 and size, no other file is
   (report.json aside), and the layout allows each one;
5. declarations: a simulation is a rehearsal, the stream suits the test and the role, the
   pre-run check precedes the run, calibration files are present, the command space digest;
6. derived members, recomputed from the files with the core that runs: envelope hash, the
   regenerated command stream against raw/commands.csv (a prefix only when
   ``generator.aborted`` says so), every scorer block and the verdict, raw_data, statements;
   each compared as RFC 8785 bytes, with a readable difference when they differ.

Consistent means no error finding. Warnings (severity "warn") are flags: the scorer's flags,
CORE_VERSION_DIFFERS and UNSUPPORTED_ALG. Consistent is not passed: a FAIL report is consistent
when its data supports FAIL.
"""

from __future__ import annotations

import hashlib
import os
from collections.abc import Mapping
from dataclasses import dataclass, field

from .. import __version__, canonical, schema, signing
from ..findings import Finding
from . import derive as _derive
from .build import REPORT_VERSION, SCHEMA_NAME
from .files import REPORT_FILE, RunFiles, raw_data_entries, scan, show

__all__ = ["CODES", "VerifyResult", "verify"]

# Every code verify can report, with its severity and what it means (docs/seams/report.md).
CODES: dict[str, tuple[str, str]] = {
    # run directory and files
    "RUN_DIR_INVALID": (
        "error",
        "the run directory is missing, unreadable, a link or not a directory",
    ),
    "RUN_DIR_TOO_LARGE": ("error", "more than 10,000 entries, or nested more than 12 levels"),
    "SYMLINK_REFUSED": ("error", "a symbolic link in the run directory; links are never followed"),
    "NOT_A_REGULAR_FILE": ("error", "a device, FIFO, socket or other special file"),
    "FILE_UNREADABLE": ("error", "a file or directory that cannot be read"),
    "FILE_TOO_LARGE": ("error", "report.json over 20 MiB or another file over 64 MiB"),
    "REPORT_MISSING": ("error", "no report.json"),
    "UNEXPECTED_FILE": ("error", "a file the run directory layout does not allow"),
    "DUPLICATE_RAW_FILE": (
        "error",
        "two files for one role, such as samples.csv and samples.csv.gz",
    ),
    "REQUIRED_FILE_MISSING": (
        "error",
        "envelope.json, command-space.json, samples, commands, or (EV-10) ev10-waypoints.json is absent",
    ),
    "FILE_MISSING": ("error", "a file raw_data lists is absent"),
    "FILE_HASH_MISMATCH": ("error", "a file's sha256 or size differs from raw_data"),
    "UNLISTED_FILE": ("error", "a file raw_data does not list"),
    "RAW_DATA_MISMATCH": (
        "error",
        "raw_data lists the right files but not as recomputed (order, role, format, repeats)",
    ),
    # report.json
    "JSON_INVALID": (
        "error",
        "report.json is not strict JSON (syntax, NaN, Infinity, numbers out of range, bad UTF-8, BOM)",
    ),
    "JSON_DUPLICATE_KEY": ("error", "report.json repeats a member name"),
    "REPORT_NOT_OBJECT": ("error", "report.json is not a JSON object"),
    "REPORT_VERSION_UNSUPPORTED": ("error", "report_version is not 0.1"),
    "SCHEMA_INVALID": (
        "error",
        "the report (or ev10-waypoints.json, envelope.json, command-space.json) breaks its schema",
    ),
    "DECLARED_INVALID": ("error", "declared fields derivation needs are unusable"),
    # keys and signatures
    "KEYRING_INVALID": ("error", "the keyring cannot be used"),
    "EMBEDDED_KEY": ("error", "the report carries a key (an object with kty); it is never used"),
    "NO_SIGNATURE": ("error", "no signature, or no valid Ed25519 signature"),
    "MALFORMED_SIGNATURE": ("error", "a signature entry is malformed"),
    "UNKNOWN_KID": (
        "error",
        "a signature's kid is not in the keyring, or the keyring key's thumbprint is not its kid",
    ),
    "BAD_SIGNATURE": ("error", "a signature does not match the report"),
    "UNSUPPORTED_ALG": ("warn", "a signature entry this core cannot check was skipped"),
    "SIGNER_KID_MISMATCH": ("error", "signer.kid is not the kid of a valid Ed25519 signature"),
    # declarations
    "SIMULATION_NOT_REHEARSAL": ("error", "run.mode simulation without role rehearsal"),
    "RUN_TIMES_INVERTED": ("error", "run.ended_at before run.started_at"),
    "ACCURACY_CHECK_AFTER_RUN_START": (
        "error",
        "the primary instrument's accuracy_check.at is after run.started_at",
    ),
    "ACCURACY_CHECK_POINTS_MISMATCH": (
        "error",
        "accuracy_check.points is not the number of pre-run points in raw/accuracy.csv",
    ),
    "CALIBRATION_FILE_MISSING": (
        "error",
        "a calibration file an instrument names is not in the run directory",
    ),
    "GENERATOR_TEST_MISMATCH": (
        "error",
        "the generator does not belong to the test (EV-03: be-hostile, be-control; EV-10: be-waypoints)",
    ),
    "GENERATOR_ROLE_MISMATCH": ("error", "be-control on a run whose role is test"),
    "COMMAND_SPACE_DIGEST_MISMATCH": (
        "error",
        "adapter.command_space_sha256 is not the digest of command-space.json",
    ),
    "CORE_VERSION_DIFFERS": ("warn", "provenance.core_version is not the core that ran this check"),
    # the command stream
    "GENERATOR_UNSUPPORTED": ("error", "a generator name or version core cannot regenerate"),
    "STREAM_PARAMS_INVALID": ("error", "the seed or params are refused"),
    "STREAM_TOO_LARGE": (
        "error",
        "the declared stream is longer than core regenerates (2,000,000 commands)",
    ),
    "STREAM_SEQ_GAP": ("error", "commands.csv seq is not 0, 1, 2, ..."),
    "STREAM_TOO_LONG": ("error", "commands.csv goes on past the stream"),
    "STREAM_MISMATCH": ("error", "a commands.csv row differs from the regenerated command"),
    "STREAM_PREFIX": (
        "info",
        "commands.csv is a strict prefix of the stream (the run stopped early)",
    ),
    "STREAM_DIGEST_MISMATCH": (
        "error",
        "generator.stream_sha256 is not the regenerated stream's digest",
    ),
    "STREAM_COUNT_MISMATCH": ("error", "generator.commands_compared is not the number of rows"),
    "STREAM_ABORTED_MISMATCH": (
        "error",
        "generator.aborted does not say whether the rows stop short of the stream",
    ),
    # derived members
    "ENVELOPE_HASH_MISMATCH": ("error", "envelope.hash is not the hash of envelope.json"),
    "VERDICT_MISMATCH": ("error", "the verdict does not follow from the files"),
    "DERIVED_MISMATCH": (
        "error",
        "a derived block (score, gaps, motion, ...) does not follow from the files",
    ),
    "STATEMENTS_MISMATCH": (
        "error",
        "statements are not the fixed statements and the measurement source",
    ),
    "RAW_TOO_LARGE": ("error", "a gzip raw file decompresses to more than 512 MiB"),
    "INTERNAL_ERROR": ("error", "verify met something it did not expect; please report it"),
}
"""Codes raised by verify itself. Findings relayed from the loaders keep their own codes
(ENVELOPE_*, COMMAND_SPACE_*, WAYPOINTS_*, RAW_*, SCORING_*) and are errors; the scorer's flags
(NONSTANDARD_PARAMS, REQUIRED_POINTS_OMIT_ZONED, SPEED_UNMEASURED, UNDECLARED_POINT_SAMPLES, and
any it adds) are relayed as warnings."""

_SPECIFIC = {
    "envelope": "ENVELOPE_HASH_MISMATCH",
    "verdict": "VERDICT_MISMATCH",
    "statements": "STATEMENTS_MISMATCH",
}
_GENERATOR_CODES = (
    ("stream_sha256", "STREAM_DIGEST_MISMATCH"),
    ("commands_compared", "STREAM_COUNT_MISMATCH"),
    ("aborted", "STREAM_ABORTED_MISMATCH"),
)
_MISSING = object()
_DIFF_LINES = 6


@dataclass(frozen=True)
class VerifyResult:
    """What ``verify`` found. ``derived`` holds the recomputed derived members (its
    ``"generator"`` only the generator's derived members); ``diffs`` a readable difference for
    each derived member that does not match; ``verdict`` the recomputed verdict (None when the
    run could not be scored)."""

    consistent: bool
    findings: tuple[Finding, ...]
    run_dir: str
    core_version: str
    report_sha256: str | None
    verdict: dict | None
    derived: dict = field(default_factory=dict, repr=False)
    diffs: dict = field(default_factory=dict, repr=False)
    report: dict | None = field(default=None, repr=False)

    def as_json(self) -> dict:
        """The ``be verify --json`` document."""
        return {
            "consistent": self.consistent,
            "core_version": self.core_version,
            "run_dir": self.run_dir,
            "report_sha256": self.report_sha256,
            "verdict": self.verdict,
            "findings": [
                {"code": f.code, "severity": f.severity, "detail": f.detail, "where": f.where}
                for f in self.findings
            ],
        }


def _err(code: str, detail: str, where: str = "") -> Finding:
    return Finding(code, "error", detail, where)


def _pointer(parts: list) -> str:
    return "".join("/" + str(p).replace("~", "~0").replace("/", "~1") for p in parts)


def _jcs(value: object) -> bytes | None:
    try:
        return canonical.canonicalize(value)
    except (canonical.CanonicalError, RecursionError):
        return None


def _same(a: object, b: object) -> bool:
    ca = _jcs(a)
    return ca is not None and ca == _jcs(b)


def _short(value: object, limit: int = 120) -> str:
    if value is _MISSING:
        return "nothing"
    raw = _jcs(value)
    text = raw.decode("utf-8") if raw is not None else repr(value)
    return text if len(text) <= limit else text[: limit - 3] + "..."


def diff(claimed: object, actual: object, pointer: str = "", limit: int = _DIFF_LINES) -> str:
    """A readable difference: up to ``limit`` lines ``<pointer>: the report has X; the files give Y``."""
    lines: list[str] = []
    more = [0]

    def walk(a: object, b: object, ptr: str) -> None:
        if _same(a, b):
            return
        if len(lines) >= limit:
            more[0] += 1
            return
        if isinstance(a, dict) and isinstance(b, dict):
            for key in sorted(set(a) | set(b)):
                walk(a.get(key, _MISSING), b.get(key, _MISSING), ptr + _pointer([key]))
            return
        if isinstance(a, list) and isinstance(b, list):
            if len(a) != len(b):
                lines.append(
                    f"{ptr or '/'}: the report has {len(a)} entries; the files give {len(b)}"
                )
            for i in range(min(len(a), len(b))):
                walk(a[i], b[i], f"{ptr}/{i}")
            return
        lines.append(f"{ptr or '/'}: the report has {_short(a)}; the files give {_short(b)}")

    walk(claimed, actual, pointer)
    if more[0]:
        lines.append(f"... and {more[0]} more difference(s)")
    return "; ".join(lines)


def _embedded_keys(doc: object) -> list[Finding]:
    found = []
    stack: list[tuple[object, list]] = [(doc, [])]
    while stack:
        node, path = stack.pop()
        if isinstance(node, dict):
            if "kty" in node:
                found.append(
                    _err(
                        "EMBEDDED_KEY",
                        "the report carries a key (an object with kty); keys come only from the keyring, "
                        "so it was not used",
                        _pointer(path) or "/",
                    )
                )
            for key, value in node.items():
                stack.append((value, [*path, key]))
        elif isinstance(node, list):
            for i, value in enumerate(node):
                stack.append((value, [*path, i]))
    return sorted(found, key=lambda f: f.where)


def _signatures(report: dict, keyring: Mapping) -> list[Finding]:
    findings = list(signing.verify(report, dict(keyring)))
    entries = report.get("signature")
    valid: set[str] = set()
    if isinstance(entries, list):
        for entry in entries:
            ed25519 = isinstance(entry, dict) and entry.get("alg") == signing.ED25519
            if ed25519 and not signing.verify({**report, "signature": [entry]}, dict(keyring)):
                valid.add(entry.get("kid"))  # each Ed25519 entry that verifies on its own
    signer = report.get("signer")
    kid = signer.get("kid") if isinstance(signer, dict) else None
    if not isinstance(kid, str):
        kid = None  # a hostile report can put anything here; only a string can name a key
    if valid and kid not in valid:
        findings.append(
            _err(
                "SIGNER_KID_MISMATCH",
                f"signer.kid is {kid!r}; the valid Ed25519 signatures are by {', '.join(sorted(valid))}",
                "/signer/kid",
            )
        )
    return findings


def _listing(report: dict, files: RunFiles, recomputed: list[dict]) -> list[Finding]:
    claimed = report.get("raw_data")
    if not isinstance(claimed, list) or not all(
        isinstance(e, dict) and isinstance(e.get("path"), str) for e in claimed
    ):
        return []  # the schema reports it
    findings = []
    listed: dict[str, dict] = {}
    for i, entry in enumerate(claimed):
        path = entry["path"]
        if path in listed:
            findings.append(
                _err("RAW_DATA_MISMATCH", f"raw_data lists {show(path)} twice", f"/raw_data/{i}")
            )
            continue
        listed[path] = entry
    for path, entry in listed.items():
        if path == REPORT_FILE:
            findings.append(
                _err(
                    "RAW_DATA_MISMATCH",
                    "raw_data lists report.json, which is never listed",
                    "/raw_data",
                )
            )
            continue
        f = files.files.get(path)
        if f is None:
            if path not in files.refused:
                findings.append(
                    _err(
                        "FILE_MISSING",
                        f"raw_data lists {show(path)}, which is not in the run directory",
                        show(path),
                    )
                )
            continue
        if entry.get("sha256") != f.sha256 or entry.get("bytes") != f.size:
            findings.append(
                _err(
                    "FILE_HASH_MISMATCH",
                    f"{path} is {f.size} bytes with {f.sha256}; raw_data says {entry.get('bytes')!r} bytes "
                    f"with {entry.get('sha256')!r}",
                    path,
                )
            )
    for path in sorted(files.files):
        if path not in listed:
            findings.append(
                _err(
                    "UNLISTED_FILE",
                    f"{show(path)} is in the run directory but not in raw_data",
                    show(path),
                )
            )
    readable = [
        e for e in claimed if e["path"] not in files.refused
    ]  # refused files are reported already
    if not findings and not _same(readable, recomputed):
        findings.append(
            _err(
                "RAW_DATA_MISMATCH",
                "raw_data is not as recomputed: " + diff(readable, recomputed, "/raw_data"),
                "/raw_data",
            )
        )
    return findings


def _compare(report: dict, blocks: dict, diffs: dict) -> list[Finding]:
    findings = []
    gen = report.get("generator") if isinstance(report.get("generator"), dict) else {}
    for member, code in _GENERATOR_CODES:
        if member in blocks.get("generator", {}):
            actual = blocks["generator"][member]
            claimed = gen.get(member, _MISSING)
            if not _same(claimed, actual):
                text = diff(claimed, actual, f"/generator/{member}")
                diffs[f"generator.{member}"] = text
                findings.append(_err(code, text, f"/generator/{member}"))
    for block in _derive.DERIVED_BLOCKS:
        if block == "raw_data" or block not in blocks:
            continue
        claimed = report.get(block, _MISSING)
        if not _same(claimed, blocks[block]):
            text = diff(claimed, blocks[block], "/" + block)
            diffs[block] = text
            code = _SPECIFIC.get(block, "DERIVED_MISMATCH")
            findings.append(
                _err(code, f"{block} does not follow from the files: {text}", "/" + block)
            )
    return findings


def _report_sha256(files: RunFiles) -> str | None:
    if files.report is None or files.report.data is None:
        return None
    return "sha256:" + hashlib.sha256(files.report.data).hexdigest()


def _parse(files: RunFiles) -> tuple[object, list[Finding]]:
    assert files.report is not None and files.report.data is not None
    try:
        return canonical.loads_strict(files.report.data), []
    except canonical.CanonicalError as e:
        code = "JSON_DUPLICATE_KEY" if "duplicate object member name" in str(e) else "JSON_INVALID"
        return None, [_err(code, f"report.json: {e}", REPORT_FILE)]


def _flags(blocks: dict) -> list[Finding]:
    out = []
    for i, flag in enumerate(blocks.get("flags", [])):
        if isinstance(flag, dict):
            out.append(
                Finding(str(flag.get("code")), "warn", str(flag.get("detail", "")), f"/flags/{i}")
            )
    return out


def _result(run_dir: str, findings: list[Finding], **kw) -> VerifyResult:
    consistent = not any(f.severity == "error" for f in findings)
    return VerifyResult(
        consistent=consistent,
        findings=tuple(findings),
        run_dir=run_dir,
        core_version=__version__,
        **kw,
    )


def verify(run_dir: str | os.PathLike, keyring: Mapping[str, Mapping]) -> VerifyResult:
    """Check the run directory against its report.json, with ``keyring`` (kid -> public JWK) the
    only keys trusted. Never raises; anything unexpected becomes an INTERNAL_ERROR finding."""
    try:
        shown = show(os.fsdecode(run_dir))
    except TypeError:
        shown = show(repr(run_dir))
    try:
        return _verify(run_dir, keyring, shown)
    except Exception as e:  # noqa: BLE001 - verify never raises: anything unexpected is reported
        detail = f"{type(e).__name__}: {e}"[:400]
        return _result(shown, [_err("INTERNAL_ERROR", detail)], report_sha256=None, verdict=None)


def _verify(run_dir: str | os.PathLike, keyring: Mapping, shown: str) -> VerifyResult:
    findings: list[Finding] = []
    if not isinstance(keyring, Mapping):
        findings.append(_err("KEYRING_INVALID", "the keyring must map kid to public JWK"))
        keyring = {}
    files = scan(run_dir)
    findings += files.findings
    sha = _report_sha256(files)
    if files.report is None:
        return _result(shown, findings, report_sha256=sha, verdict=None)
    report, problems = _parse(files)
    findings += problems
    if problems:
        return _result(shown, findings, report_sha256=sha, verdict=None)
    if not isinstance(report, dict):
        findings.append(_err("REPORT_NOT_OBJECT", "report.json is not a JSON object", REPORT_FILE))
        return _result(shown, findings, report_sha256=sha, verdict=None)

    if report.get("report_version") != REPORT_VERSION:
        findings.append(
            _err(
                "REPORT_VERSION_UNSUPPORTED",
                f"report_version is {_short(report.get('report_version', _MISSING))}; this core reads {REPORT_VERSION}",
                "/report_version",
            )
        )
        findings += _embedded_keys(report)
        findings += _signatures(report, keyring)
        return _result(shown, findings, report_sha256=sha, verdict=None, report=report)

    try:
        schema_findings = schema.validate(report, SCHEMA_NAME)
    except RecursionError:
        schema_findings = [_err("SCHEMA_INVALID", "the report is nested too deeply to validate")]
    findings += schema_findings
    findings += _embedded_keys(report)
    findings += _signatures(report, keyring)

    recomputed_listing = raw_data_entries(files)
    test = report.get("test") if report.get("test") in _derive.SCORER_FOR_TEST else None
    listed = report.get("raw_data")
    listed_paths = (
        frozenset(
            e["path"] for e in listed if isinstance(e, dict) and isinstance(e.get("path"), str)
        )
        if isinstance(listed, list)
        else frozenset()
    )
    findings += _derive.layout_checks(files, test, listed_paths)
    findings += _listing(report, files, recomputed_listing)

    if schema_findings:
        return _result(shown, findings, report_sha256=sha, verdict=None, report=report)

    view, problems = _derive.view_of(report)
    findings += problems
    if view is None:
        return _result(shown, findings, report_sha256=sha, verdict=None, report=report)
    derivation = _derive.derive(view, files)
    findings += derivation.findings
    blocks = derivation.blocks
    diffs: dict = {}
    findings += _compare(report, blocks, diffs)

    has_error = any(f.severity == "error" for f in findings)
    missing = [k for k in _derive.expected_blocks(view.test) if k not in blocks]
    missing += [
        f"generator.{k}" for k in _derive.GENERATOR_DERIVED if k not in blocks.get("generator", {})
    ]
    if missing and not has_error:  # a member is left out only when an error says why
        findings.append(
            _err("INTERNAL_ERROR", f"derived members were not recomputed: {', '.join(missing)}")
        )

    findings += _flags(blocks)
    stated = report.get("provenance", {}).get("core_version")
    if stated != __version__:
        findings.append(
            Finding(
                "CORE_VERSION_DIFFERS",
                "warn",
                f"the report was built with core {stated}; this check ran core {__version__}",
                "/provenance/core_version",
            )
        )
    return _result(
        shown,
        findings,
        report_sha256=sha,
        verdict=blocks.get("verdict"),
        derived=blocks,
        diffs=diffs,
        report=report,
    )
