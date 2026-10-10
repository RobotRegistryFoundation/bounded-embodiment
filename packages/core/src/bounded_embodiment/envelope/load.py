"""Load an envelope: parse strictly, validate it against the schema of its version, upgrade a 0.1
document to the 0.2 form in memory, check the rules a schema cannot express, and hash it.

Finding codes raised here (all errors unless noted):

- ENVELOPE_UNREADABLE, ENVELOPE_PARSE_ERROR, ENVELOPE_NOT_OBJECT, ENVELOPE_VERSION_UNSUPPORTED,
  ENVELOPE_NOT_FINITE, ENVELOPE_NOT_CANONICAL: the document cannot be read, parsed or hashed.
- ENVELOPE_DUPLICATE_POINT_ID, ENVELOPE_DUPLICATE_ZONE_ID, ENVELOPE_UNKNOWN_POINT: ids.
- ENVELOPE_BOX_INVERTED, ENVELOPE_Z_RANGE_INVERTED, ENVELOPE_POLYGON_DEGENERATE,
  ENVELOPE_POLYGON_NOT_SIMPLE: zone geometry.
- ENVELOPE_STOP_SOURCE_EXCEEDS, ENVELOPE_STOP_SOURCE_DUPLICATE: stop.sources[].
- ENVELOPE_DUPLICATE_JOINT, ENVELOPE_JOINT_RANGE_INVERTED: motion.joints[].
- ENVELOPE_POINT_NO_KEEP_IN (warn): no keep-in zone bounds a declared point.

Schema violations keep the codes bounded_embodiment.schema gives them.
"""

from __future__ import annotations

import copy
import math
import os
from pathlib import Path

from .. import canonical, schema
from ..findings import Finding, errors
from . import geometry
from .model import Envelope, EnvelopeError, Point, Zone

SCHEMA_FOR_VERSION = {"0.1": "envelope-0.1", "0.2": "envelope-0.2"}

#: The single point a 0.1 document is upgraded with (CONTRACT.md, "Upgrading 0.1 to 0.2").
MACHINE_POINT_0_1 = {
    "id": "machine",
    "description": "the machine's reference point as declared in 0.1",
}

_BLOCKS_KEPT_FROM_0_1 = (
    "motion",
    "force",
    "proximity",
    "sensing",
    "stop",
    "heartbeat",
    "authority",
    "signature",
)


def load(src: str | os.PathLike | dict) -> Envelope:
    """Load an envelope 0.1 or 0.2 from a path or a dict; raise EnvelopeError with findings if it
    is not usable. A dict is deep-copied, so later changes to it do not reach the Envelope."""
    doc = _read(src)
    if not isinstance(doc, dict):
        raise EnvelopeError(
            [_err("ENVELOPE_NOT_OBJECT", f"an envelope is a JSON object, not {type(doc).__name__}")]
        )
    version = doc.get("envelope_version")
    schema_name = SCHEMA_FOR_VERSION.get(version) if isinstance(version, str) else None
    if schema_name is None:
        detail = (
            "envelope_version is missing"
            if "envelope_version" not in doc
            else f"envelope_version {version!r} is not one of {', '.join(SCHEMA_FOR_VERSION)}"
        )
        raise EnvelopeError([_err("ENVELOPE_VERSION_UNSUPPORTED", detail, "/envelope_version")])

    findings = _non_finite(doc, "")
    if findings:
        raise EnvelopeError(findings)
    findings = list(schema.validate(doc, schema_name))
    if errors(findings):
        raise EnvelopeError(findings)

    if version == "0.1":
        normalized, paths = upgrade_0_1(doc), _Paths(from_0_1=True)
    else:
        normalized, paths = doc, _Paths(from_0_1=False)
    findings.extend(_dedupe(check_rules(normalized, paths)))
    if errors(findings):
        raise EnvelopeError(findings)

    try:
        digest = canonical.sha256_jcs({k: v for k, v in doc.items() if k != "signature"})
    except canonical.CanonicalError as e:
        raise EnvelopeError(
            findings
            + [
                _err(
                    "ENVELOPE_NOT_CANONICAL", f"the document cannot be hashed as RFC 8785 JSON: {e}"
                )
            ]
        )
    return Envelope(
        version="0.2",
        document=doc,
        hash=digest,
        points={p["id"]: Point.from_dict(p) for p in normalized["points"]},
        zones=[Zone.from_dict(z) for z in normalized["workspace"]["zones"]],
        normalized=normalized,
        upgraded_from="0.1" if version == "0.1" else None,
        findings=tuple(findings),
    )


def upgrade_0_1(doc: dict) -> dict:
    """The 0.2 form of a valid 0.1 envelope, built as CONTRACT.md specifies. Nothing is written
    back: the 0.1 document stays what gets hashed and signed. The free-text 0.1 machine.class is
    mapped onto the 0.2 enum (unknown wording becomes "other"), and the original wording is kept
    in machine.description when the class changed and no description was given."""
    ws = doc["workspace"]
    z = ws.get("z_range_m")

    def prism(polygon: list) -> dict:
        p = {"polygon_m": copy.deepcopy(polygon)}
        if z is not None:
            p["z_m"] = list(z)
        return p

    zones = [
        {"id": "keep_in", "kind": "keep_in", "applies_to": ["*"], "prism": prism(ws["keep_in"])}
    ]
    for i, polygon in enumerate(ws.get("keep_out", [])):
        zones.append(
            {
                "id": f"keep_out_{i}",
                "kind": "keep_out",
                "applies_to": ["*"],
                "prism": prism(polygon),
            }
        )
    machine = copy.deepcopy(doc["machine"])
    original_class = str(machine.get("class", ""))
    machine["class"] = _class_0_2(original_class)
    if machine["class"] != original_class and "description" not in machine:
        machine["description"] = f"0.1 class: {original_class}"
    out: dict = {"envelope_version": "0.2", "machine": machine}
    if "level" in doc:
        out["declared_level"] = doc["level"]
    out["frame"] = ws["frame"]
    out["points"] = [dict(MACHINE_POINT_0_1)]
    out["workspace"] = {"zones": zones}
    for key in _BLOCKS_KEPT_FROM_0_1:
        if key in doc:
            out[key] = copy.deepcopy(doc[key])
    return out


def check_rules(doc: dict, paths: _Paths | None = None) -> list[Finding]:
    """The rules a schema cannot express, over a schema-valid document in the 0.2 form."""
    paths = paths or _Paths(from_0_1=False)
    out: list[Finding] = []

    point_index: dict[str, int] = {}
    for i, p in enumerate(doc["points"]):
        if p["id"] in point_index:
            out.append(
                _err(
                    "ENVELOPE_DUPLICATE_POINT_ID",
                    f"point id {p['id']!r} is declared twice (points {point_index[p['id']]} "
                    f"and {i})",
                    paths.point(i, "id"),
                )
            )
        else:
            point_index[p["id"]] = i

    zones = doc["workspace"]["zones"]
    zone_index: dict[str, int] = {}
    for i, zone in enumerate(zones):
        if zone["id"] in zone_index:
            out.append(
                _err(
                    "ENVELOPE_DUPLICATE_ZONE_ID",
                    f"zone id {zone['id']!r} is declared twice (zones {zone_index[zone['id']]} "
                    f"and {i})",
                    paths.zone(i, "id"),
                )
            )
        else:
            zone_index[zone["id"]] = i
        for k, ref in enumerate(zone["applies_to"]):
            if ref != "*" and ref not in point_index:
                out.append(
                    _err(
                        "ENVELOPE_UNKNOWN_POINT",
                        f"zone {zone['id']!r} applies to {ref!r}, which is not a declared point",
                        paths.zone(i, "applies_to", k),
                    )
                )
        if "box" in zone:
            lo, hi = zone["box"]["min_m"], zone["box"]["max_m"]
            bad = [axis for axis in range(3) if not lo[axis] < hi[axis]]
            if bad:
                out.append(
                    _err(
                        "ENVELOPE_BOX_INVERTED",
                        f"zone {zone['id']!r}: min_m must be below max_m on every axis, and is "
                        f"not on {', '.join('xyz'[a] for a in bad)}",
                        paths.zone(i, "box"),
                    )
                )
            continue
        prism = zone["prism"]
        for kind, detail in geometry.polygon_problems(prism["polygon_m"]):
            code = (
                "ENVELOPE_POLYGON_DEGENERATE"
                if kind == "degenerate"
                else "ENVELOPE_POLYGON_NOT_SIMPLE"
            )
            out.append(_err(code, f"zone {zone['id']!r}: {detail}", paths.polygon(i)))
        z = prism.get("z_m")
        if z is not None and not z[0] < z[1]:
            out.append(
                _err(
                    "ENVELOPE_Z_RANGE_INVERTED",
                    f"zone {zone['id']!r}: height band [{z[0]}, {z[1]}] must have its minimum "
                    "below its maximum",
                    paths.z_range(i),
                )
            )

    for pid, i in point_index.items():
        if not any(
            zone["kind"] == "keep_in" and ("*" in zone["applies_to"] or pid in zone["applies_to"])
            for zone in zones
        ):
            out.append(
                Finding(
                    "ENVELOPE_POINT_NO_KEEP_IN",
                    "warn",
                    f"no keep_in zone applies to point {pid!r}, so its position is not bounded",
                    paths.point(i),
                )
            )

    joint_index: dict[str, int] = {}
    for i, joint in enumerate(doc["motion"].get("joints", [])):
        where = f"/motion/joints/{i}"
        if joint["name"] in joint_index:
            out.append(
                _err(
                    "ENVELOPE_DUPLICATE_JOINT",
                    f"joint {joint['name']!r} is listed twice (joints {joint_index[joint['name']]} "
                    f"and {i})",
                    where + "/name",
                )
            )
        else:
            joint_index[joint["name"]] = i
        r = joint.get("position_rad")
        if r is not None and not r[0] < r[1]:
            out.append(
                _err(
                    "ENVELOPE_JOINT_RANGE_INVERTED",
                    f"joint {joint['name']!r}: position_rad [{r[0]}, {r[1]}] must have its "
                    "minimum below its maximum",
                    where + "/position_rad",
                )
            )

    stop = doc["stop"]
    source_index: dict[str, int] = {}
    for k, src in enumerate(stop.get("sources", [])):
        where = f"/stop/sources/{k}"
        if src["source"] in source_index:
            out.append(
                _err(
                    "ENVELOPE_STOP_SOURCE_DUPLICATE",
                    f"stop source {src['source']!r} is listed twice (sources "
                    f"{source_index[src['source']]} and {k})",
                    where + "/source",
                )
            )
        else:
            source_index[src["source"]] = k
        # IEC 60204-1 stop categories name kinds of stop, not magnitudes, so only time and
        # distance are compared against the worst case.
        for key in ("max_time_ms", "max_distance_m"):
            if src[key] > stop[key]:
                out.append(
                    _err(
                        "ENVELOPE_STOP_SOURCE_EXCEEDS",
                        f"stop source {src['source']!r} declares {key} {src[key]}, above the "
                        f"top-level {stop[key]}; the top-level stop must be the worst case "
                        "over every source",
                        f"{where}/{key}",
                    )
                )
    return out


# ----- helpers ---------------------------------------------------------------------------------
class _Paths:
    """JSON Pointers for findings, into the document as given (a 0.1 document has no zones[])."""

    def __init__(self, from_0_1: bool):
        self.from_0_1 = from_0_1

    def point(self, i: int, *rest) -> str:
        if self.from_0_1:
            return ""  # the upgrade's default point is not in the 0.1 document
        return _pointer("points", i, *rest)

    def zone(self, i: int, *rest) -> str:
        if self.from_0_1:
            return "/workspace/keep_in" if i == 0 else f"/workspace/keep_out/{i - 1}"
        return _pointer("workspace", "zones", i, *rest)

    def polygon(self, i: int) -> str:
        return self.zone(i) if self.from_0_1 else self.zone(i, "prism", "polygon_m")

    def z_range(self, i: int) -> str:
        return "/workspace/z_range_m" if self.from_0_1 else self.zone(i, "prism", "z_m")


def _pointer(*parts) -> str:
    return "".join("/" + str(p).replace("~", "~0").replace("/", "~1") for p in parts)


def _err(code: str, detail: str, where: str = "") -> Finding:
    return Finding(code, "error", detail, where)


def _dedupe(findings: list[Finding]) -> list[Finding]:
    seen, out = set(), []
    for f in findings:
        key = (f.code, f.severity, f.detail, f.where)
        if key not in seen:
            seen.add(key)
            out.append(f)
    return out


def _non_finite(node, where: str) -> list[Finding]:
    """NaN and infinities, which JSON cannot carry (a file never has them: loads_strict refuses
    them; a dict can)."""
    if isinstance(node, float) and not math.isfinite(node):
        return [_err("ENVELOPE_NOT_FINITE", f"{node!r} is not a JSON number", where)]
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
            raise EnvelopeError(
                [_err("ENVELOPE_UNREADABLE", f"cannot read {path}: {e.strerror or e}")]
            )
        try:
            return canonical.loads_strict(data)
        except ValueError as e:  # CanonicalError and JSON syntax errors alike
            raise EnvelopeError([_err("ENVELOPE_PARSE_ERROR", f"{path} is not strict JSON: {e}")])
    raise TypeError(f"load() takes a path or a dict, not {type(src).__name__}")


_CLASS_WORDS_0_2 = (
    ("mobile_manipulator", ("mobile_manipulator", "mobile-manipulator", "mobile manipulator")),
    ("legged", ("legged", "quadruped", "biped", "humanoid", "hexapod")),
    ("arm", ("arm", "manipulator", "cobot")),
    ("mobile_base", ("mobile", "rover", "wheeled", "ground", "agv", "amr", "drone", "aerial")),
)


def _class_0_2(text: str) -> str:
    """The 0.2 machine.class for a free-text 0.1 class; the first matching family wins."""
    t = text.strip().lower()
    if t in ("arm", "mobile_base", "mobile_manipulator", "legged", "other"):
        return t
    for cls, words in _CLASS_WORDS_0_2:
        if any(w in t for w in words):
            return cls
    return "other"
