"""Test reports 0.1 and 0.2: build one from a run directory, sign it, and check it (``be verify``).

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

from .build import (
    REPORT_VERSION,
    REPORT_VERSIONS,
    SCHEMA_NAME,
    SCHEMA_NAMES,
    ReportError,
    build,
    report_version_for,
    sign_report,
    to_json,
)
from .cli import KeyringError, cli_verify, load_keyring
from .derive import DERIVED_BLOCKS, GENERATOR_DERIVED, fk_samples_from_state
from .files import (
    COMMAND_SPACE_FILE,
    ENVELOPE_FILE,
    MODEL_LOSS_PLAN_FILE,
    REPORT_FILE,
    STOP_PLAN_FILE,
    WAYPOINTS_FILE,
    classify,
)
from .plans import (
    ModelLossPlan,
    PlanError,
    StopPlan,
    check_plan_commands,
    load_model_loss_plan,
    load_stop_plan,
)
from .streams import (
    CONTROL,
    HOSTILE,
    MODEL_LOSS,
    STOPS,
    WAYPOINTS,
    control_stream,
    model_loss_stream,
    split_segments,
    stop_stream,
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
    "MODEL_LOSS",
    "MODEL_LOSS_PLAN_FILE",
    "REPORT_FILE",
    "REPORT_VERSION",
    "REPORT_VERSIONS",
    "SCHEMA_NAME",
    "SCHEMA_NAMES",
    "STOPS",
    "STOP_PLAN_FILE",
    "WAYPOINTS",
    "WAYPOINTS_FILE",
    "KeyringError",
    "ModelLossPlan",
    "PlanError",
    "ReportError",
    "StopPlan",
    "VerifyResult",
    "Waypoints",
    "WaypointsError",
    "build",
    "check_plan_commands",
    "classify",
    "cli_verify",
    "control_stream",
    "fk_samples_from_state",
    "load_keyring",
    "load_model_loss_plan",
    "load_stop_plan",
    "load_waypoints",
    "model_loss_stream",
    "report_version_for",
    "sign_report",
    "split_segments",
    "stop_stream",
    "stream_digest",
    "to_json",
    "verify",
    "waypoint_stream",
]
