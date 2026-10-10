"""The loaded envelope: points, zones and the queries scoring and the kit run against them.

Build these with :func:`bounded_embodiment.envelope.load`, which validates the document first; the
constructors here assume a valid document.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from typing import Any

from ..findings import Finding, errors
from . import geometry

FaceFn = Callable[[Sequence[float]], float]


class EnvelopeError(ValueError):
    """The document is not a usable envelope. ``findings`` lists everything the loader found,
    errors and warnings alike; filter on ``severity``."""

    def __init__(self, findings: list[Finding], message: str | None = None):
        self.findings = list(findings)
        if message is None:
            errs = errors(self.findings) or self.findings
            head = "; ".join(
                f"{f.code}{' at ' + f.where if f.where else ''}: {f.detail}" for f in errs[:3]
            )
            more = f" (+{len(errs) - 3} more)" if len(errs) > 3 else ""
            message = f"invalid envelope: {head}{more}" if head else "invalid envelope"
        super().__init__(message)


@dataclass(frozen=True)
class Point:
    """A named point of the machine whose position zones bound and an instrument measures."""

    id: str
    description: str
    link: str | None = None
    offset_m: tuple[float, float, float] | None = None
    max_speed_mps: float | None = None

    @classmethod
    def from_dict(cls, d: dict) -> Point:
        off = d.get("offset_m")
        return cls(
            id=d["id"],
            description=d["description"],
            link=d.get("link"),
            offset_m=tuple(off) if off is not None else None,
            max_speed_mps=d.get("max_speed_mps"),
        )


@dataclass(frozen=True)
class Zone:
    """A keep-in or keep-out solid (box or vertical prism) and the points it applies to."""

    id: str
    kind: str  # "keep_in" | "keep_out"
    applies_to: tuple[str, ...]  # point ids, or ("*",)
    box_min_m: tuple[float, float, float] | None = None
    box_max_m: tuple[float, float, float] | None = None
    polygon_m: tuple[tuple[float, float], ...] | None = None
    z_m: tuple[float, float] | None = None
    ccw: bool = field(default=True, compare=False)  # polygon winding (prisms only)

    @classmethod
    def from_dict(cls, d: dict) -> Zone:
        common = {"id": d["id"], "kind": d["kind"], "applies_to": tuple(d["applies_to"])}
        if "box" in d:
            return cls(
                **common, box_min_m=tuple(d["box"]["min_m"]), box_max_m=tuple(d["box"]["max_m"])
            )
        prism = d["prism"]
        poly = tuple((v[0], v[1]) for v in prism["polygon_m"])
        z = prism.get("z_m")
        return cls(
            **common,
            polygon_m=poly,
            z_m=tuple(z) if z is not None else None,
            ccw=geometry.polygon_twice_area_exact(poly) > 0,
        )

    @property
    def shape(self) -> str:
        return "box" if self.box_min_m is not None else "prism"

    def applies(self, point_id: str) -> bool:
        return "*" in self.applies_to or point_id in self.applies_to

    def signed_distance(self, xyz: Sequence[float]) -> float:
        """Metres from xyz to the zone's surface: positive inside the solid, negative outside it."""
        if self.box_min_m is not None:
            return geometry.box_signed_distance(self.box_min_m, self.box_max_m, xyz)
        return geometry.prism_signed_distance(self.polygon_m, self.z_m, xyz)

    def clearance(self, xyz: Sequence[float]) -> float:
        """Signed metres of compliance with this zone alone: for a keep-in zone the distance inside
        it (negative outside); for a keep-out zone the distance outside it (negative inside)."""
        d = self.signed_distance(xyz)
        return d if self.kind == "keep_in" else 0.0 - d

    def faces(self) -> list[tuple[str, FaceFn]]:
        """The zone's faces as ``(face_id, fn)``; ``fn(xyz)`` is the distance to that face,
        positive on the zone's side. Box: x_min, x_max, y_min, y_max, z_min, z_max (distance to
        the face's plane). Prism: edge_<i> for polygon edge i (vertex i to i + 1; see
        geometry.prism_edge_face_distance), then z_min and z_max when it has a height band."""
        out: list[tuple[str, FaceFn]] = []
        if self.box_min_m is not None:
            lo, hi = self.box_min_m, self.box_max_m
            for name, axis, side in geometry.BOX_FACES:
                out.append((name, _box_face(lo, hi, axis, side)))
            return out
        for i in range(len(self.polygon_m)):
            out.append((f"edge_{i}", _edge_face(self.polygon_m, self.z_m, self.ccw, i)))
        if self.z_m is not None:
            out.append(("z_min", _cap_face(self.z_m, -1)))
            out.append(("z_max", _cap_face(self.z_m, 1)))
        return out


def _box_face(lo, hi, axis, side) -> FaceFn:
    return lambda xyz: geometry.box_face_distance(lo, hi, axis, side, xyz)


def _edge_face(poly, z, ccw, i) -> FaceFn:
    return lambda xyz: geometry.prism_edge_face_distance(poly, z, ccw, i, xyz)


def _cap_face(z, side) -> FaceFn:
    return lambda xyz: geometry.prism_cap_face_distance(z, side, xyz)


@dataclass(frozen=True, eq=False)
class Envelope:
    """A validated envelope, always in its 0.2 form.

    ``document`` is the dict as given (for a 0.1 document, the 0.1 dict) and ``hash`` is
    ``sha256_jcs`` of it minus ``signature``. ``normalized`` is the 0.2 form the queries use: the
    in-memory upgrade of a 0.1 document, or the document itself. ``findings`` holds the warnings
    the loader raised without rejecting the document."""

    version: str
    document: dict
    hash: str
    points: dict[str, Point]
    zones: list[Zone]
    normalized: dict
    upgraded_from: str | None = None
    findings: tuple[Finding, ...] = ()

    # ----- declared blocks, from the 0.2 form ------------------------------------------------
    @property
    def frame(self) -> str:
        return self.normalized["frame"]

    @property
    def machine(self) -> dict:
        return self.normalized["machine"]

    @property
    def declared_level(self) -> str | None:
        return self.normalized.get("declared_level")

    @property
    def motion(self) -> dict:
        return self.normalized["motion"]

    @property
    def stop(self) -> dict:
        return self.normalized["stop"]

    @property
    def heartbeat(self) -> dict:
        return self.normalized["heartbeat"]

    def get(self, key: str, default: Any = None) -> Any:
        """Any other top-level member of the 0.2 form (force, proximity, sensing, notes...)."""
        return self.normalized.get(key, default)

    # ----- queries ---------------------------------------------------------------------------
    def _point(self, point_id: str) -> Point:
        try:
            return self.points[point_id]
        except KeyError:
            raise KeyError(
                f"unknown point {point_id!r}; this envelope declares {sorted(self.points)}"
            ) from None

    def zones_for(self, point_id: str) -> list[Zone]:
        """The zones whose applies_to names the point or "*", in document order."""
        self._point(point_id)
        return [z for z in self.zones if z.applies(point_id)]

    def clearance(self, point_id: str, xyz: Sequence[float]) -> float:
        """Signed metres: the minimum over the point's zones of the distance inside each keep-in
        zone and outside each keep-out zone. Positive means compliant; negative means outside the
        allowed region by that much. ``math.inf`` when no zone applies to the point."""
        best = math.inf
        for z in self.zones_for(point_id):
            best = min(best, z.clearance(xyz))
        return best

    def faces(self, point_id: str) -> list[tuple[str, str, FaceFn]]:
        """``[(face_id, zone_id, fn)]`` over the keep-in zones that apply to the point, for
        coverage: ``fn(xyz)`` is the distance to that face, positive on the inside. Keep-out zones
        are covered as wholes, through ``zone.clearance``, because a plane through one face of an
        obstacle says nothing about approaching it."""
        out = []
        for z in self.zones_for(point_id):
            if z.kind != "keep_in":
                continue
            for face_id, fn in z.faces():
                out.append((face_id, z.id, fn))
        return out

    def speed_limit_mps(self, point_id: str) -> float:
        """The point's max_speed_mps, falling back to motion.max_speed_mps."""
        p = self._point(point_id)
        if p.max_speed_mps is not None:
            return p.max_speed_mps
        return self.motion["max_speed_mps"]
