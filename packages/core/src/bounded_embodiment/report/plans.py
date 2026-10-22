"""The EV-02 stop plan and the EV-05 model-loss plan: ``ev02-stop-plan.json`` and
``ev05-model-loss-plan.json`` in the run directory (``schemas/stop-plan-0.1.json``,
``schemas/model-loss-plan-0.1.json``).

Each plan is declared by the lab before the run, listed and hashed in the report's ``raw_data``
like every other file, and read by both its generator (``be-stops/1``, ``be-model-loss/1``: the
commands) and its scorer (``ev02-stop/1``, ``ev05-model-loss/1``: the stop point and, for EV-05,
the stop source and the resume rule).

Finding codes (errors): PLAN_PARSE_ERROR, PLAN_NOT_OBJECT, PLAN_VERSION_UNSUPPORTED,
PLAN_DUPLICATE_ID, PLAN_DUPLICATE_SOURCE, PLAN_UNKNOWN_DRIVE, PLAN_UNKNOWN_SOURCE,
PLAN_COMMAND_UNKNOWN_AXIS, PLAN_COMMAND_MISSING_AXIS, PLAN_COMMAND_OUTSIDE_LIMITS; schema
violations keep SCHEMA_INVALID.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from types import MappingProxyType

from .. import canonical, schema
from ..commandspace import CommandSpace
from ..findings import Finding, errors
from .files import MODEL_LOSS_PLAN_FILE, STOP_PLAN_FILE

__all__ = [
    "MODEL_LOSS_PLAN_FILE",
    "MODEL_LOSS_SCHEMA",
    "STOP_PLAN_FILE",
    "STOP_SCHEMA",
    "VERSION",
    "Drive",
    "ModelLossPlan",
    "PlanError",
    "StopPlan",
    "check_plan_commands",
    "load_model_loss_plan",
    "load_stop_plan",
]

VERSION = "0.1"
STOP_SCHEMA = "stop-plan-0.1"
MODEL_LOSS_SCHEMA = "model-loss-plan-0.1"
DEFAULT_LOSS_SOURCE = "heartbeat_loss"


class PlanError(ValueError):
    """The document is not a usable plan; ``findings`` says why."""

    def __init__(self, findings: list[Finding]):
        self.findings = list(findings)
        head = "; ".join(
            f"{f.code}{' at ' + f.where if f.where else ''}: {f.detail}" for f in self.findings[:3]
        )
        super().__init__(f"invalid plan: {head}" if head else "invalid plan")


def _err(code: str, detail: str, where: str = "") -> Finding:
    return Finding(code, "error", detail, where)


def _command(raw: Mapping) -> Mapping[str, float]:
    """A read-only command with every value a float (-0 read as +0)."""
    return MappingProxyType({k: float(v) + 0.0 for k, v in raw.items()})


@dataclass(frozen=True)
class Drive:
    id: str
    command: Mapping[str, float]  # read-only; every value a float


@dataclass(frozen=True, eq=False)
class StopPlan:
    """A loaded EV-02 stop plan. ``digest`` is ``sha256_jcs`` of the document; ``trials`` holds
    (source, drive id) pairs in declared order."""

    point: str | None
    start: Mapping[str, float]
    drives: tuple[Drive, ...]
    sources: tuple[str, ...]
    trials: tuple[tuple[str, str], ...]
    document: dict = field(repr=False)
    digest: str

    def drive(self, drive_id: str) -> Drive:
        for d in self.drives:
            if d.id == drive_id:
                return d
        raise KeyError(f"no drive {drive_id!r}; the plan declares {[d.id for d in self.drives]}")

    def commands(self) -> list[tuple[str, Mapping[str, float]]]:
        """Every command the plan holds, with a JSON Pointer to it: the start, then each drive."""
        out = [("/start", self.start)]
        out += [(f"/drives/{i}/command", d.command) for i, d in enumerate(self.drives)]
        return out

    def for_scoring(self) -> dict:
        """The scorer's ``plan`` input: ``{"point", "sources"}``."""
        return {"point": self.point, "sources": list(self.sources)}


@dataclass(frozen=True, eq=False)
class ModelLossPlan:
    """A loaded EV-05 model-loss plan. ``digest`` is ``sha256_jcs`` of the document; ``resume``
    is ``{"rule": "manual"}`` or ``{"rule": "commands", "after_ms": n}`` (descriptions dropped)."""

    point: str | None
    source: str
    start: Mapping[str, float]
    path: tuple[Mapping[str, float], ...]
    resume: Mapping
    document: dict = field(repr=False)
    digest: str

    def commands(self) -> list[tuple[str, Mapping[str, float]]]:
        out = [("/start", self.start)]
        out += [(f"/path/{i}", c) for i, c in enumerate(self.path)]
        return out

    def for_scoring(self) -> dict:
        """The scorer's ``plan`` input: ``{"point", "source", "resume"}``."""
        return {"point": self.point, "source": self.source, "resume": dict(self.resume)}


def _document(src: bytes | str | dict, what: str, version_key: str) -> dict:
    if isinstance(src, dict):
        doc: object = src
    else:
        try:
            doc = canonical.loads_strict(src)
        except ValueError as e:
            raise PlanError([_err("PLAN_PARSE_ERROR", f"{what} is not strict JSON: {e}")]) from None
    if not isinstance(doc, dict):
        raise PlanError([_err("PLAN_NOT_OBJECT", f"{what} is a JSON object")])
    if doc.get(version_key) != VERSION:
        raise PlanError(
            [_err("PLAN_VERSION_UNSUPPORTED", f"{version_key} must be {VERSION!r}", f"/{version_key}")]
        )
    return doc


def _digest(doc: dict) -> str:
    try:
        return canonical.sha256_jcs(doc)
    except canonical.CanonicalError as e:
        raise PlanError(
            [_err("PLAN_PARSE_ERROR", f"the document cannot be hashed as RFC 8785 JSON: {e}")]
        ) from None


def load_stop_plan(src: bytes | str | dict) -> StopPlan:
    """Load an EV-02 stop plan from its bytes or text (parsed strictly) or a dict; raise
    PlanError with findings when it is not usable."""
    doc = _document(src, STOP_PLAN_FILE, "stop_plan_version")
    findings = schema.validate(doc, STOP_SCHEMA)
    if errors(findings):
        raise PlanError(findings)
    drives: list[Drive] = []
    for i, d in enumerate(doc["drives"]):
        if any(d["id"] == seen.id for seen in drives):
            findings.append(
                _err("PLAN_DUPLICATE_ID", f"drive {d['id']!r} is declared twice", f"/drives/{i}/id")
            )
            continue
        drives.append(Drive(d["id"], _command(d["command"])))
    sources: list[str] = []
    for i, s in enumerate(doc["sources"]):
        if s["source"] in sources:
            findings.append(
                _err(
                    "PLAN_DUPLICATE_SOURCE",
                    f"source {s['source']!r} is declared twice",
                    f"/sources/{i}/source",
                )
            )
            continue
        sources.append(s["source"])
    trials: list[tuple[str, str]] = []
    for i, t in enumerate(doc["trials"]):
        if t["source"] not in sources:
            findings.append(
                _err(
                    "PLAN_UNKNOWN_SOURCE",
                    f"trial {i} names source {t['source']!r}, which the plan does not declare",
                    f"/trials/{i}/source",
                )
            )
        if not any(d.id == t["drive"] for d in drives):
            findings.append(
                _err(
                    "PLAN_UNKNOWN_DRIVE",
                    f"trial {i} names drive {t['drive']!r}, which the plan does not declare",
                    f"/trials/{i}/drive",
                )
            )
        trials.append((t["source"], t["drive"]))
    if errors(findings):
        raise PlanError(findings)
    return StopPlan(
        doc.get("point"),
        _command(doc["start"]),
        tuple(drives),
        tuple(sources),
        tuple(trials),
        doc,
        _digest(doc),
    )


def load_model_loss_plan(src: bytes | str | dict) -> ModelLossPlan:
    """Load an EV-05 model-loss plan from its bytes or text (parsed strictly) or a dict; raise
    PlanError with findings when it is not usable."""
    doc = _document(src, MODEL_LOSS_PLAN_FILE, "model_loss_plan_version")
    findings = schema.validate(doc, MODEL_LOSS_SCHEMA)
    if errors(findings):
        raise PlanError(findings)
    resume = doc["resume"]
    rule = (
        {"rule": "manual"}
        if resume["rule"] == "manual"
        else {"rule": "commands", "after_ms": resume["after_ms"]}
    )
    return ModelLossPlan(
        doc.get("point"),
        doc.get("source", DEFAULT_LOSS_SOURCE),
        _command(doc["start"]),
        tuple(_command(c) for c in doc["path"]),
        MappingProxyType(rule),
        doc,
        _digest(doc),
    )


def check_plan_commands(plan: StopPlan | ModelLossPlan, space: CommandSpace, file: str) -> list[Finding]:
    """Every command of the plan must name every axis of the command space and nothing else,
    each value within the axis's limits: a stop or model-loss test only asks for what the machine
    may do."""
    findings = []
    keys = space.keys
    for pointer, command in plan.commands():
        where = f"{file} {pointer}"
        unknown = [k for k in command if k not in keys]
        missing = [k for k in keys if k not in command]
        if unknown:
            findings.append(
                _err(
                    "PLAN_COMMAND_UNKNOWN_AXIS",
                    f"{', '.join(unknown)} not axes of the command space",
                    where,
                )
            )
        if missing:
            findings.append(
                _err("PLAN_COMMAND_MISSING_AXIS", f"no value for {', '.join(missing)}", where)
            )
        for axis in space.axes:
            value = command.get(axis.key)
            if value is not None and not axis.lo <= value <= axis.hi:
                findings.append(
                    _err(
                        "PLAN_COMMAND_OUTSIDE_LIMITS",
                        f"{axis.key}={value!r} is outside [{axis.lo!r}, {axis.hi!r}]",
                        where,
                    )
                )
    return findings
