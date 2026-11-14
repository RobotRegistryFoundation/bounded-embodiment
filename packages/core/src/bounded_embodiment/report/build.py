"""Build and sign a test report (0.1, or 0.2 for EV-02 and EV-05) from a run directory.

``build(declared, run_dir)`` returns the unsigned report: the declared fields as given (with
``scoring.profile`` written out, ``scoring.method`` and ``provenance.core_version`` filled in
and the generator's params normalized) plus every derived member, computed from the run
directory by :func:`derive.derive`, the same function ``be verify`` runs. It refuses
(ReportError) a run that ``be verify`` would find not consistent for any reason but the
signature, so a report it returns needs only ``sign_report``.

Report versions. Test report 0.2 (``schemas/test-report-0.2.json``) is 0.1 with the tests EV-02
and EV-05 added. Unless the declarations name a ``report_version``, build writes 0.2 for EV-02
and EV-05, which 0.1 cannot express, and 0.1 for EV-03 and EV-10, exactly as before, so an EV-03
or EV-10 report and the golden runs do not change. An EV-03 or EV-10 report may declare 0.2; its
members are the same.
"""

from __future__ import annotations

import copy
import json
import os
from collections.abc import Mapping

from .. import __version__, canonical, schema, signing
from ..findings import Finding, errors
from ..profiles import load_profile
from . import derive as _derive
from . import streams
from .files import scan

__all__ = [
    "REPORT_VERSION",
    "REPORT_VERSIONS",
    "SCHEMA_NAME",
    "SCHEMA_NAMES",
    "ReportError",
    "build",
    "report_version_for",
    "sign_report",
    "to_json",
]

REPORT_VERSION = "0.1"
"""The version build writes for EV-03 and EV-10 (``report_version_for``)."""
SCHEMA_NAME = "test-report-0.1"
REPORT_VERSIONS = ("0.1", "0.2")
"""The report versions this core builds and checks."""
SCHEMA_NAMES = {"0.1": "test-report-0.1", "0.2": "test-report-0.2"}
_TESTS_NEEDING_0_2 = ("EV-02", "EV-05")


def report_version_for(test: object) -> str:
    """The report version build writes when the declarations name none: 0.2 for EV-02 and
    EV-05, 0.1 otherwise."""
    return "0.2" if test in _TESTS_NEEDING_0_2 else REPORT_VERSION

DECLARED_ORDER = (
    "report_version",
    "method",
    "test",
    "role",
    "reproduces",
    "control_run",
    "run",
    "system_under_test",
    "adapter",
    "generator",
    "instruments",
    "scoring",
    "safety",
    "provenance",
    "license",
    "rights_affirmed",
    "signer",
)


class ReportError(ValueError):
    """The run cannot be made into a consistent report; ``findings`` says why."""

    def __init__(self, findings: list[Finding]):
        self.findings = list(findings)
        errs = errors(self.findings) or self.findings
        head = "; ".join(
            f"{f.code}{' at ' + f.where if f.where else ''}: {f.detail}" for f in errs[:3]
        )
        more = f" (+{len(errs) - 3} more)" if len(errs) > 3 else ""
        super().__init__(
            f"cannot build the report: {head}{more}" if head else "cannot build the report"
        )


def _err(code: str, detail: str, where: str = "") -> Finding:
    return Finding(code, "error", detail, where)


def _profile(arg: object, declared: object, test: object) -> dict:
    chosen = arg if arg is not None else declared
    if chosen is None:
        chosen = _derive.DEFAULT_PROFILE.get(test) if isinstance(test, str) else None
        if chosen is None:
            raise ReportError(
                [
                    _err(
                        "DECLARED_INVALID",
                        "no profile, and no default for this test",
                        "/scoring/profile",
                    )
                ]
            )
    if isinstance(chosen, str):
        try:
            return load_profile(chosen)
        except KeyError as e:
            raise ReportError(
                [_err("DECLARED_INVALID", str(e.args[0] if e.args else e), "/scoring/profile")]
            ) from None
    if isinstance(chosen, Mapping):
        return copy.deepcopy(dict(chosen))
    raise ReportError(
        [
            _err(
                "DECLARED_INVALID",
                "the profile is a packaged profile's name or the profile itself",
                "/scoring/profile",
            )
        ]
    )


def _prepare(declared: Mapping, profile: object) -> dict:
    """The declared fields with build's additions, or ReportError."""
    doc = copy.deepcopy(dict(declared))
    problems = []
    for key in sorted(set(doc) & {*_derive.DERIVED_BLOCKS, "signature"}):
        problems.append(
            _err(
                "DECLARED_HAS_DERIVED",
                f"{key} is derived from the run directory by build; leave it out",
                "/" + key,
            )
        )
    gen = doc.get("generator")
    if isinstance(gen, Mapping):
        for key in _derive.GENERATOR_DERIVED:
            if key in gen:
                problems.append(
                    _err(
                        "DECLARED_HAS_DERIVED",
                        f"generator.{key} is derived by build; leave it out",
                        f"/generator/{key}",
                    )
                )
    if problems:
        raise ReportError(problems)
    test = doc.get("test")
    doc.setdefault("report_version", report_version_for(test))
    if doc["report_version"] not in REPORT_VERSIONS:
        raise ReportError(
            [
                _err(
                    "REPORT_VERSION_UNSUPPORTED",
                    f"report_version {doc['report_version']!r}: this core builds "
                    f"{' and '.join(REPORT_VERSIONS)}",
                    "/report_version",
                )
            ]
        )

    scoring = doc.get("scoring", {})
    if isinstance(scoring, Mapping):
        method = scoring.get(
            "method", _derive.SCORER_FOR_TEST.get(test) if isinstance(test, str) else None
        )
        filled = {"method": method} if method is not None else {}
        filled["profile"] = _profile(profile, scoring.get("profile"), test)
        doc["scoring"] = {**filled, **{k: v for k, v in scoring.items() if k not in filled}}

    provenance = doc.get("provenance")
    if isinstance(provenance, Mapping):
        stated = provenance.get("core_version")
        if stated is not None and stated != __version__:
            raise ReportError(
                [
                    _err(
                        "CORE_VERSION_DIFFERS",
                        f"provenance.core_version is {stated!r}, but this is core {__version__}",
                        "/provenance/core_version",
                    )
                ]
            )
        doc["provenance"] = {**provenance, "core_version": __version__}

    if isinstance(gen, Mapping) and gen.get("name") in streams.GENERATORS:
        name = gen["name"]
        params = gen.get("params", {} if name == streams.HOSTILE else None)
        try:
            doc["generator"] = {**gen, "params": streams.normalize(name, params)}
        except ValueError as e:
            raise ReportError(
                [_err("STREAM_PARAMS_INVALID", str(e), "/generator/params")]
            ) from None
    return doc


def assemble(doc: Mapping, blocks: Mapping) -> dict:
    """Declared fields in report order, then the derived members."""
    report: dict = {}
    for key in DECLARED_ORDER:
        if key in doc:
            report[key] = doc[key]
    for key in doc:
        if key not in report:
            report[key] = doc[key]
    if isinstance(report.get("generator"), Mapping):
        report["generator"] = {**report["generator"], **blocks.get("generator", {})}
    for key in _derive.DERIVED_BLOCKS:
        if key in blocks:
            report[key] = blocks[key]
    return report


def build(
    declared: Mapping, run_dir: str | os.PathLike, *, profile: str | Mapping | None = None
) -> dict:
    """The unsigned report for the run in ``run_dir``.

    ``declared`` holds the declared members (schemas/test-report-0.1.json, or 0.2 for EV-02 and
    EV-05); a derived member in it is refused. ``profile`` is a packaged profile's name or a profile object; without it,
    ``declared["scoring"]["profile"]`` (a name or an object), else the packaged illustrative
    profile of the test. ``scoring.method`` defaults to the test's scorer.

    Raises ReportError with findings when the run directory or the declarations would not make
    a consistent report (signatures aside), TypeError when ``declared`` is not a mapping."""
    if not isinstance(declared, Mapping):
        raise TypeError(f"declared must be a mapping, not {type(declared).__name__}")
    doc = _prepare(declared, profile)
    view, problems = _derive.view_of(doc)
    if view is None:
        raise ReportError(problems)
    files = scan(run_dir, read_report=False)
    findings = list(files.findings)
    findings += _derive.layout_checks(files, view.test)
    derivation = _derive.derive(view, files)
    findings += derivation.findings
    if errors(findings):
        raise ReportError(findings)
    report = assemble(doc, derivation.blocks)
    missing = [k for k in _derive.expected_blocks(view.test) if k not in report]
    missing += [
        f"generator.{k}"
        for k in _derive.GENERATOR_DERIVED
        if k not in derivation.blocks.get("generator", {})
    ]
    if missing:  # pragma: no cover - derive always fills these when it finds no error
        raise ReportError(
            [_err("INTERNAL_ERROR", f"derived members not computed: {', '.join(missing)}")]
        )
    problems = schema.validate(report, SCHEMA_NAMES[report["report_version"]])
    if problems:
        raise ReportError(problems)
    try:
        canonical.canonicalize(report)
    except canonical.CanonicalError as e:
        raise ReportError(
            [_err("REPORT_NOT_CANONICAL", f"the report cannot be written as RFC 8785 JSON: {e}")]
        ) from None
    return report


def sign_report(report: Mapping, private_jwk: Mapping) -> dict:
    """A copy of ``report`` with an Ed25519 signature entry appended (bounded_embodiment.signing).

    The key must be the one ``signer.kid`` names (ValueError otherwise), so the report says who
    signed it. A further signature (a co-signer's) can be added with ``signing.sign``."""
    if not isinstance(report, Mapping) or report.get("report_version") not in REPORT_VERSIONS:
        raise ValueError("sign_report() takes a test report 0.1 or 0.2")
    kid = signing.thumbprint(dict(private_jwk))
    signer = report.get("signer")
    if not isinstance(signer, Mapping) or signer.get("kid") != kid:
        stated = signer.get("kid") if isinstance(signer, Mapping) else None
        raise ValueError(f"signer.kid is {stated!r}, but the signing key's kid is {kid}")
    return signing.sign(dict(report), dict(private_jwk))


def to_json(report: Mapping) -> bytes:
    """report.json bytes: indented UTF-8 JSON, members in report order, ending in a newline.
    Any JSON text of the same values verifies the same: signatures cover the RFC 8785 form."""
    return (json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n").encode(
        "utf-8"
    )
