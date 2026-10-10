"""EV-10 waypoints: ``ev10-waypoints.json`` in the run directory (``schemas/waypoints-0.1.json``).

Each waypoint has an id, a position (where the liveness point must arrive, metres in the
envelope frame) and a command (the command-space values sent while visiting it). The file is
declared by the lab, listed and hashed in the report's ``raw_data`` like every other file, and
read by both the ``be-waypoints/1`` generator (the commands) and the ``ev10-waypoints/1`` scorer
(the positions).

Finding codes (errors): WAYPOINTS_PARSE_ERROR, WAYPOINTS_NOT_OBJECT,
WAYPOINTS_VERSION_UNSUPPORTED, WAYPOINTS_DUPLICATE_ID, WAYPOINTS_COMMAND_UNKNOWN_AXIS,
WAYPOINTS_COMMAND_MISSING_AXIS, WAYPOINTS_COMMAND_OUTSIDE_LIMITS; schema violations keep
SCHEMA_INVALID.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from types import MappingProxyType

from .. import canonical, schema
from ..commandspace import CommandSpace
from ..findings import Finding, errors

__all__ = [
    "SCHEMA_NAME",
    "VERSION",
    "Waypoint",
    "Waypoints",
    "WaypointsError",
    "check_commands",
    "load_waypoints",
]

SCHEMA_NAME = "waypoints-0.1"
VERSION = "0.1"


class WaypointsError(ValueError):
    """The document is not a usable waypoints file; ``findings`` says why."""

    def __init__(self, findings: list[Finding]):
        self.findings = list(findings)
        head = "; ".join(
            f"{f.code}{' at ' + f.where if f.where else ''}: {f.detail}" for f in self.findings[:3]
        )
        super().__init__(f"invalid waypoints: {head}" if head else "invalid waypoints")


@dataclass(frozen=True)
class Waypoint:
    id: str
    position_m: tuple[float, float, float]
    command: Mapping[str, float]  # read-only; every value a float


@dataclass(frozen=True, eq=False)
class Waypoints:
    """A loaded waypoints file. ``digest`` is ``sha256_jcs`` of the document."""

    point: str | None
    waypoints: tuple[Waypoint, ...]
    document: dict = field(repr=False)
    digest: str

    def for_scoring(self) -> list[dict]:
        """The scorer's ``waypoints`` input: ``[{"id", "position_m"}]`` in declared order."""
        return [{"id": w.id, "position_m": list(w.position_m)} for w in self.waypoints]


def _err(code: str, detail: str, where: str = "") -> Finding:
    return Finding(code, "error", detail, where)


def load_waypoints(src: bytes | str | dict) -> Waypoints:
    """Load a waypoints document from its bytes or text (parsed strictly) or a dict; raise
    WaypointsError with findings when it is not usable."""
    if isinstance(src, dict):
        doc: object = src
    else:
        try:
            doc = canonical.loads_strict(src)
        except ValueError as e:
            raise WaypointsError(
                [_err("WAYPOINTS_PARSE_ERROR", f"ev10-waypoints.json is not strict JSON: {e}")]
            ) from None
    if not isinstance(doc, dict):
        raise WaypointsError([_err("WAYPOINTS_NOT_OBJECT", "a waypoints file is a JSON object")])
    if doc.get("waypoints_version") != VERSION:
        raise WaypointsError(
            [
                _err(
                    "WAYPOINTS_VERSION_UNSUPPORTED",
                    f"waypoints_version must be {VERSION!r}",
                    "/waypoints_version",
                )
            ]
        )
    findings = schema.validate(doc, SCHEMA_NAME)
    if errors(findings):
        raise WaypointsError(findings)
    items: list[Waypoint] = []
    seen: set[str] = set()
    for i, w in enumerate(doc["waypoints"]):
        if w["id"] in seen:
            findings.append(
                _err(
                    "WAYPOINTS_DUPLICATE_ID",
                    f"waypoint {w['id']!r} is declared twice",
                    f"/waypoints/{i}/id",
                )
            )
            continue
        seen.add(w["id"])
        position = tuple(float(v) + 0.0 for v in w["position_m"])
        command = MappingProxyType({k: float(v) + 0.0 for k, v in w["command"].items()})
        items.append(Waypoint(w["id"], (position[0], position[1], position[2]), command))
    if errors(findings):
        raise WaypointsError(findings)
    try:
        digest = canonical.sha256_jcs(doc)
    except canonical.CanonicalError as e:
        raise WaypointsError(
            [_err("WAYPOINTS_PARSE_ERROR", f"the document cannot be hashed as RFC 8785 JSON: {e}")]
        ) from None
    return Waypoints(doc.get("point"), tuple(items), doc, digest)


def check_commands(waypoints: Waypoints, space: CommandSpace) -> list[Finding]:
    """Each waypoint's command must name every axis of the command space and nothing else, each
    value within the axis's limits: a liveness tour only asks for what the machine may do."""
    findings = []
    keys = space.keys
    for i, w in enumerate(waypoints.waypoints):
        where = f"ev10-waypoints.json /waypoints/{i}/command"
        unknown = [k for k in w.command if k not in keys]
        missing = [k for k in keys if k not in w.command]
        if unknown:
            findings.append(
                _err(
                    "WAYPOINTS_COMMAND_UNKNOWN_AXIS",
                    f"waypoint {w.id}: {', '.join(unknown)} not axes of the command space",
                    where,
                )
            )
        if missing:
            findings.append(
                _err(
                    "WAYPOINTS_COMMAND_MISSING_AXIS",
                    f"waypoint {w.id}: no value for {', '.join(missing)}",
                    where,
                )
            )
        for axis in space.axes:
            value = w.command.get(axis.key)
            if value is not None and not axis.lo <= value <= axis.hi:
                findings.append(
                    _err(
                        "WAYPOINTS_COMMAND_OUTSIDE_LIMITS",
                        f"waypoint {w.id}: {axis.key}={value!r} is outside [{axis.lo!r}, {axis.hi!r}]",
                        where,
                    )
                )
    return findings
