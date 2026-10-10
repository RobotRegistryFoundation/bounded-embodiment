"""The command space: the command vocabulary a lab declares for the machine under test.

A command space lists the keys a command may carry (``axes``), each with its unit, its limits
and its rest value, plus optional far targets the lab names and the malformed commands the
transport can carry. ``schemas/command-space-0.1.json`` is the format. The hostile command
generator (:mod:`bounded_embodiment.generator`) builds every command it sends from it.

Why it is a declared file: the limits must come from the machine's design data (for an arm, the
URDF joint limits), never from bus calibration alone. A servo calibrated to its full encoder
range would otherwise be aimed far past its mechanical stops (bob's elbow and wrist would be
aimed at about +/-171 degrees).

Why ``digest`` matters: it is ``sha256_jcs`` of the document, and it goes into the stream digest,
so a run's command stream is pinned to the exact vocabulary it was generated from. Two documents
with the same digest always give the same stream: every number is read as a float, and negative
zero (which RFC 8785 writes as ``0``) is read as zero.

Finding codes raised here (all errors):

- COMMAND_SPACE_UNREADABLE, COMMAND_SPACE_PARSE_ERROR, COMMAND_SPACE_NOT_OBJECT,
  COMMAND_SPACE_VERSION_UNSUPPORTED, COMMAND_SPACE_NOT_FINITE, COMMAND_SPACE_NOT_CANONICAL: the
  document cannot be read, parsed or hashed.
- COMMAND_SPACE_DUPLICATE_AXIS, COMMAND_SPACE_RESERVED_KEY, COMMAND_SPACE_RANGE_INVERTED,
  COMMAND_SPACE_HOME_OUTSIDE: axes.
- COMMAND_SPACE_DUPLICATE_FAR_TARGET, COMMAND_SPACE_FAR_TARGET_UNKNOWN_AXIS,
  COMMAND_SPACE_FAR_TARGET_MISSING_AXIS: far_targets.

Schema violations keep the codes :mod:`bounded_embodiment.schema` gives them (SCHEMA_INVALID).
"""

from __future__ import annotations

import copy
import math
import os
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType

from . import canonical, schema
from .findings import Finding, errors

__all__ = [
    "KINDS",
    "MALFORMED_KINDS",
    "SCHEMA_NAME",
    "UNITS",
    "UNKNOWN_AXIS_KEY",
    "Axis",
    "CommandSpace",
    "CommandSpaceError",
    "FarTarget",
    "load_command_space",
]

SCHEMA_NAME = "command-space-0.1"
VERSION = "0.1"
KINDS = ("joint_position", "cartesian_position", "velocity")
UNITS = ("deg", "rad", "m", "m/s", "rad/s", "percent")

#: Every malformed kind, in the fixed order the generator draws from. A command space that
#: declares a subset keeps this order whatever order the document lists them in.
MALFORMED_KINDS = (
    "nan",
    "inf",
    "neg_inf",
    "string",
    "null",
    "bool",
    "missing_key",
    "unknown_key",
    "empty",
)

#: The key the malformed ``unknown_key`` command adds; no axis may use it.
UNKNOWN_AXIS_KEY = "unknown_axis"


class CommandSpaceError(ValueError):
    """The document is not a usable command space. ``findings`` lists everything the loader
    found; filter on ``severity``."""

    def __init__(self, findings: list[Finding], message: str | None = None):
        self.findings = list(findings)
        if message is None:
            errs = errors(self.findings) or self.findings
            head = "; ".join(
                f"{f.code}{' at ' + f.where if f.where else ''}: {f.detail}" for f in errs[:3]
            )
            more = f" (+{len(errs) - 3} more)" if len(errs) > 3 else ""
            message = f"invalid command space: {head}{more}" if head else "invalid command space"
        super().__init__(message)


@dataclass(frozen=True)
class Axis:
    """One key a command may carry, with its unit, its limits and its rest value."""

    key: str
    unit: str
    lo: float
    hi: float
    home: float


@dataclass(frozen=True)
class FarTarget:
    """A target the lab declares as somewhere the machine must not end up; one value per axis.
    ``values`` is read-only, so the stream cannot drift away from the digest."""

    id: str
    values: Mapping[str, float]
    description: str = ""


@dataclass(frozen=True, eq=False)
class CommandSpace:
    """A validated command space. Build it with :func:`load_command_space`.

    ``axes`` and ``far_targets`` keep the document's order; ``malformed`` lists the declared
    malformed kinds in :data:`MALFORMED_KINDS` order (all of them when the document leaves the
    member out). ``document`` is the dict as given, and ``digest`` is ``sha256_jcs`` of it."""

    kind: str
    axes: tuple[Axis, ...]
    far_targets: tuple[FarTarget, ...]
    malformed: tuple[str, ...]
    document: dict
    digest: str = field(init=False)

    def __post_init__(self) -> None:
        object.__setattr__(self, "digest", canonical.sha256_jcs(self.document))

    @property
    def keys(self) -> tuple[str, ...]:
        """The axis keys, in declared order."""
        return tuple(a.key for a in self.axes)

    def axis(self, key: str) -> Axis:
        for a in self.axes:
            if a.key == key:
                return a
        raise KeyError(f"unknown axis {key!r}; this command space declares {list(self.keys)}")

    def far_target(self, target_id: str) -> FarTarget:
        for ft in self.far_targets:
            if ft.id == target_id:
                return ft
        declared = [ft.id for ft in self.far_targets]
        raise KeyError(
            f"unknown far target {target_id!r}; this command space declares "
            f"{declared if declared else 'no far targets'}"
        )


def load_command_space(src: str | os.PathLike | dict) -> CommandSpace:
    """Load a command space from a path or a dict; raise CommandSpaceError with findings if it is
    not usable. A dict is deep-copied, so later changes to it do not reach the CommandSpace."""
    doc = _read(src)
    if not isinstance(doc, dict):
        raise CommandSpaceError(
            [
                _err(
                    "COMMAND_SPACE_NOT_OBJECT",
                    f"a command space is a JSON object, not {type(doc).__name__}",
                )
            ]
        )
    version = doc.get("command_space_version")
    if version != VERSION:
        detail = (
            "command_space_version is missing"
            if "command_space_version" not in doc
            else f"command_space_version {version!r} is not {VERSION!r}"
        )
        raise CommandSpaceError(
            [_err("COMMAND_SPACE_VERSION_UNSUPPORTED", detail, "/command_space_version")]
        )

    findings = _non_finite(doc, "")
    if findings:
        raise CommandSpaceError(findings)
    findings = list(schema.validate(doc, SCHEMA_NAME))
    if errors(findings):
        raise CommandSpaceError(findings)
    findings.extend(check_rules(doc))
    if errors(findings):
        raise CommandSpaceError(findings)

    axes = tuple(
        Axis(
            key=a["key"],
            unit=a["unit"],
            lo=_number(a["lo"]),
            hi=_number(a["hi"]),
            home=_number(a["home"]),
        )
        for a in doc["axes"]
    )
    far_targets = tuple(
        FarTarget(
            id=ft["id"],
            values=MappingProxyType({a.key: _number(ft["values"][a.key]) for a in axes}),
            description=ft.get("description", ""),
        )
        for ft in doc.get("far_targets", [])
    )
    declared = set(doc.get("malformed", MALFORMED_KINDS))
    try:
        return CommandSpace(
            kind=doc["kind"],
            axes=axes,
            far_targets=far_targets,
            malformed=tuple(k for k in MALFORMED_KINDS if k in declared),
            document=doc,
        )
    except canonical.CanonicalError as e:
        raise CommandSpaceError(
            findings
            + [
                _err(
                    "COMMAND_SPACE_NOT_CANONICAL",
                    f"the document cannot be hashed as RFC 8785 JSON: {e}",
                )
            ]
        ) from None


def check_rules(doc: dict) -> list[Finding]:
    """The rules a schema cannot express, over a schema-valid document."""
    out: list[Finding] = []
    index: dict[str, int] = {}
    for i, a in enumerate(doc["axes"]):
        key, where = a["key"], f"/axes/{i}"
        if key in index:
            out.append(
                _err(
                    "COMMAND_SPACE_DUPLICATE_AXIS",
                    f"axis key {key!r} is declared twice (axes {index[key]} and {i})",
                    where + "/key",
                )
            )
        else:
            index[key] = i
        if key == UNKNOWN_AXIS_KEY:
            out.append(
                _err(
                    "COMMAND_SPACE_RESERVED_KEY",
                    f"{UNKNOWN_AXIS_KEY!r} is reserved for the malformed unknown_key command",
                    where + "/key",
                )
            )
        if not a["lo"] < a["hi"]:
            out.append(
                _err(
                    "COMMAND_SPACE_RANGE_INVERTED",
                    f"axis {key!r}: lo {a['lo']} must be below hi {a['hi']}",
                    where,
                )
            )
        elif not a["lo"] <= a["home"] <= a["hi"]:
            out.append(
                _err(
                    "COMMAND_SPACE_HOME_OUTSIDE",
                    f"axis {key!r}: home {a['home']} is outside [{a['lo']}, {a['hi']}]",
                    where + "/home",
                )
            )

    seen: dict[str, int] = {}
    for i, ft in enumerate(doc.get("far_targets", [])):
        where = f"/far_targets/{i}"
        if ft["id"] in seen:
            out.append(
                _err(
                    "COMMAND_SPACE_DUPLICATE_FAR_TARGET",
                    f"far target id {ft['id']!r} is declared twice (far targets "
                    f"{seen[ft['id']]} and {i})",
                    where + "/id",
                )
            )
        else:
            seen[ft["id"]] = i
        unknown = [k for k in ft["values"] if k not in index]
        if unknown:
            out.append(
                _err(
                    "COMMAND_SPACE_FAR_TARGET_UNKNOWN_AXIS",
                    f"far target {ft['id']!r} sets {unknown}, which are not declared axes",
                    where + "/values",
                )
            )
        missing = [k for k in index if k not in ft["values"]]
        if missing:
            out.append(
                _err(
                    "COMMAND_SPACE_FAR_TARGET_MISSING_AXIS",
                    f"far target {ft['id']!r} does not set {missing}; a far target names every "
                    "axis",
                    where + "/values",
                )
            )
    return out


# ----- helpers ---------------------------------------------------------------------------------
def _number(value: float) -> float:
    """A JSON number as the float the generator computes with. Adding 0.0 turns -0.0 into 0.0,
    so the stream cannot depend on a sign RFC 8785 (and so the digest) does not record."""
    return float(value) + 0.0


def _err(code: str, detail: str, where: str = "") -> Finding:
    return Finding(code, "error", detail, where)


def _pointer(*parts) -> str:
    return "".join("/" + str(p).replace("~", "~0").replace("/", "~1") for p in parts)


def _non_finite(node, where: str) -> list[Finding]:
    """NaN and infinities, which JSON cannot carry (a file never has them: loads_strict refuses
    them; a dict can)."""
    if isinstance(node, float) and not math.isfinite(node):
        return [_err("COMMAND_SPACE_NOT_FINITE", f"{node!r} is not a JSON number", where)]
    out: list[Finding] = []
    if isinstance(node, dict):
        for k, v in node.items():
            out.extend(_non_finite(v, where + _pointer(k)))
    elif isinstance(node, (list, tuple)):
        for i, v in enumerate(node):
            out.extend(_non_finite(v, where + _pointer(i)))
    return out


def _read(src) -> object:
    if isinstance(src, dict):
        return copy.deepcopy(src)
    if isinstance(src, (str, os.PathLike)):
        path = Path(src)
        try:
            data = path.read_bytes()
        except OSError as e:
            raise CommandSpaceError(
                [_err("COMMAND_SPACE_UNREADABLE", f"cannot read {path}: {e.strerror or e}")]
            ) from None
        try:
            return canonical.loads_strict(data)
        except ValueError as e:  # CanonicalError and JSON syntax errors alike
            raise CommandSpaceError(
                [_err("COMMAND_SPACE_PARSE_ERROR", f"{path} is not strict JSON: {e}")]
            ) from None
    raise TypeError(f"load_command_space() takes a path or a dict, not {type(src).__name__}")
