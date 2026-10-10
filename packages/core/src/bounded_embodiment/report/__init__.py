"""Test report 0.1: build it from a run directory, sign it, and check it (``be verify``).

    from bounded_embodiment import report

    doc = report.build(declared, "runs/ev03-20261013T183000Z-s7")     # unsigned; ReportError if not
    doc = report.sign_report(doc, private_jwk)                        # the key signer.kid names
    Path("runs/.../report.json").write_bytes(report.to_json(doc))
    result = report.verify("runs/ev03-20261013T183000Z-s7", keyring)  # never raises
    result.consistent, result.findings, result.verdict, result.diffs

The kit, ``be submit`` and the results CI all run the same ``verify``; ``build`` derives every
derived member with the same code, so a report it returns is consistent once signed.
docs/seams/report.md describes the fields, the run directory, the streams and every code.
"""

from .build import REPORT_VERSION, SCHEMA_NAME, ReportError, build, sign_report, to_json
from .cli import KeyringError, cli_verify, load_keyring
from .derive import DERIVED_BLOCKS, GENERATOR_DERIVED, fk_samples_from_state
from .files import (
    COMMAND_SPACE_FILE,
    ENVELOPE_FILE,
    REPORT_FILE,
    WAYPOINTS_FILE,
    classify,
)
from .streams import (
    CONTROL,
    HOSTILE,
    WAYPOINTS,
    control_stream,
    stream_digest,
    waypoint_stream,
)
from .verify import CODES, VerifyResult, verify
from .waypoints import Waypoints, WaypointsError, load_waypoints

__all__ = [
    "CODES",
    "COMMAND_SPACE_FILE",
    "CONTROL",
    "DERIVED_BLOCKS",
    "ENVELOPE_FILE",
    "GENERATOR_DERIVED",
    "HOSTILE",
    "REPORT_FILE",
    "REPORT_VERSION",
    "SCHEMA_NAME",
    "WAYPOINTS",
    "WAYPOINTS_FILE",
    "KeyringError",
    "ReportError",
    "VerifyResult",
    "Waypoints",
    "WaypointsError",
    "build",
    "classify",
    "cli_verify",
    "control_stream",
    "fk_samples_from_state",
    "load_keyring",
    "load_waypoints",
    "sign_report",
    "stream_digest",
    "to_json",
    "verify",
    "waypoint_stream",
]
