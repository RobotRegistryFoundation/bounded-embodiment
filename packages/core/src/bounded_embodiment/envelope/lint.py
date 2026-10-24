"""Semantic lint for envelopes: what the schema and the loader accept but the method cannot use.

The loader (``envelope.load``) rejects documents that are malformed: unknown point ids,
inverted boxes and height bands, self-intersecting or zero-area polygons, stop sources above the
top-level stop. An envelope can pass all of that and still be unsafe, contradictory or vacuous:
a keep-out zone that covers the whole keep-in zone, a keep-out zone so thin that a point can sit
anywhere inside it without a position excursion, a stop distance longer than any point is from
the edge, a proximity rule that lets the machine go faster when a person is closer. ``lint``
finds these and reports each with a stable ``LINT_*`` code.

Lint never changes a verdict and is not part of any rule set: ``be verify`` and the scorers do
not run it. It reads the envelope's 0.2 form (a 0.1 document is linted after its in-memory
upgrade) and the published profiles, for the tolerance and the motion and EV-10 thresholds that
make a geometric fact matter.

Region depth. For a point with zones, the *depth* of its allowed region is the largest clearance
any position has (method section 4.3). A branch-and-bound search over cells, bounding boxes
and height bands exactly axis by axis and a polygon's distance by its 1-Lipschitz property,
narrows the depth until every threshold that matters (tolerance_m, the EV-10 inside minimum,
the stop distances) is decided, or reports LINT_DEPTH_UNDECIDED when its budget runs out. A region
counts as empty when no position with positive clearance is found and none can be tolerance_m
inside. Every step is deterministic.

Only the standard library is used (CONTRACT.md: core imports jsonschema and cryptography only).
"""

from __future__ import annotations

import heapq
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

from ..findings import Finding
from ..profiles import load_profile
from . import geometry
from .load import _Paths
from .model import Envelope, Zone

__all__ = [
    "DEPTH_EPS_M",
    "LINT_CODES",
    "SPEED_IMPLAUSIBLE_MPS",
    "STOP_SOURCES",
    "Depth",
    "lint",
    "region_depth",
    "zone_depth",
]

#: Every lint code, with its severity and what it means. Codes are stable once released.
#: Severity: "error" means no run can give a meaningful verdict against this envelope, or a
#: rule is contradicted; "warn" means a limit is vacuous or a test cannot be judged; "info"
#: means redundant or unchecked.
LINT_CODES: dict[str, tuple[str, str]] = {
    "LINT_KEEP_IN_DISJOINT": (
        "error",
        "two keep_in zones of one point do not overlap, so every position is outside one of them",
    ),
    "LINT_KEEP_OUT_COVERS_KEEP_IN": (
        "error",
        "keep_out zones cover the whole of a point's keep_in region, so no position is allowed",
    ),
    "LINT_KEEP_OUT_UNENFORCEABLE": (
        "error",
        "a keep_out zone is no deeper than tolerance_m, so a point can be anywhere inside it "
        "without a position excursion",
    ),
    "LINT_ALLOWED_REGION_THINNER_THAN_TOLERANCE": (
        "warn",
        "no position of the point's allowed region is tolerance_m inside it",
    ),
    "LINT_ALLOWED_REGION_TOO_THIN_FOR_EV10": (
        "warn",
        "no position is as far inside as an EV-10 waypoint must be, so every EV-10 run is "
        "INCONCLUSIVE",
    ),
    "LINT_KEEP_IN_UNBOUNDED_Z": (
        "warn",
        "no keep_in zone of the point bounds z (prisms without z_m), so it may go up or down "
        "without limit",
    ),
    "LINT_KEEP_OUT_UNREACHABLE": (
        "info",
        "a keep_out zone does not meet the point's keep_in region, so it excludes nothing",
    ),
    "LINT_STOP_DISTANCE_EXCEEDS_DEPTH": (
        "warn",
        "the declared worst-case stop distance is longer than the depth of the allowed region, "
        "so from every position a worst-case stop may leave it",
    ),
    "LINT_STOP_DISTANCE_ZERO": (
        "warn",
        "stop.max_distance_m is 0 while the machine may move: no stop is that short",
    ),
    "LINT_STOP_DISTANCE_ABOVE_SPEED_TIME": (
        "info",
        "the stop distance is longer than speed limit x stop time, so it bounds nothing",
    ),
    "LINT_STOP_SOURCE_MISSING": (
        "warn",
        "stop.sources itemizes the stop sources but leaves this one out",
    ),
    "LINT_SPEED_LIMIT_IMPLAUSIBLE": (
        "warn",
        "a speed limit above SPEED_IMPLAUSIBLE_MPS; the speed test is vacuous at that limit",
    ),
    "LINT_SPEED_LIMIT_ABOVE_PHYSICAL_MAX": (
        "warn",
        "a speed limit above the declared physical maximum speed, so the machine can never "
        "exceed it",
    ),
    "LINT_SPEED_LIMIT_BLOCKS_MOTION": (
        "error",
        "every required point's speed limit is too low for the EV-03 motion rule, so no run "
        "that keeps the limit can be PASS",
    ),
    "LINT_SPEED_LIMIT_ON_UNREQUIRED_POINT": (
        "warn",
        "a point with its own speed limit has no zones, so no instrument has to measure it",
    ),
    "LINT_PROXIMITY_CLOSER_FASTER": (
        "error",
        "a proximity rule for a closer person allows a higher speed than a rule for a farther one",
    ),
    "LINT_PROXIMITY_DUPLICATE_DISTANCE": (
        "warn",
        "two proximity rules have the same distance",
    ),
    "LINT_PROXIMITY_SPEED_NOT_A_LIMIT": (
        "info",
        "a proximity speed is at or above every point's speed limit, so it limits nothing",
    ),
    "LINT_JOINT_SPEED_ABOVE_CAP": (
        "warn",
        "a joint's max_speed_radps is above motion.max_joint_speed_radps",
    ),
    "LINT_JOINT_RANGE_VACUOUS": (
        "warn",
        "a joint's position_rad range is wider than a full turn, so it bounds nothing",
    ),
    "LINT_ZONE_DUPLICATE": (
        "info",
        "two zones of the same kind have the same shape",
    ),
    "LINT_DEPTH_UNDECIDED": (
        "info",
        "the region search ran out of budget before it could decide a depth question",
    ),
}

STOP_SOURCES = (
    "stop_message",
    "hardwired_stop",
    "heartbeat_loss",
    "comms_loss",
    "power_loss",
    "gate_loss",
)

SPEED_IMPLAUSIBLE_MPS = 10.0
"""Above this a declared linear speed limit is not a plausible bound for a machine under test."""

DEPTH_EPS_M = 1e-4
"""The resolution of depth questions: a keep_out zone "meets" a keep_in region when some
position is this far inside both, and is unenforceable when no position is more than
tolerance_m + this inside it."""

MAX_EVALUATIONS = 40_000
"""Function evaluations per region search."""

_Z_MARGIN_M = 1.0
"""How far past every finite height the search looks when no keep_in zone bounds z."""


# --------------------------------------------------------------------------- region search


@dataclass(frozen=True)
class Depth:
    """The result of a region search: ``best`` is the largest value found (at ``at``), and
    ``bound`` an upper bound on the true maximum. The search stops as soon as every threshold
    it was given is decided (``best >= t`` or ``bound < t``); ``exact`` is False when its budget
    ran out first."""

    best: float
    bound: float
    at: tuple[float, float, float]
    exact: bool


Term = tuple[Zone, int]
"""One term of a function to maximize: ``sign`` x the zone's signed distance (+1 is the depth
inside the zone, -1 the distance outside it). The function is the minimum of its terms."""


def _value(terms: Sequence[Term], xyz: Sequence[float]) -> float:
    best = math.inf
    for zone, sign in terms:
        d = zone.signed_distance(xyz)
        best = min(best, d if sign > 0 else 0.0 - d)
    return best


def _tent_max(a: float, b: float, lo: float, hi: float) -> float:
    """max over x in [a, b] of min(x - lo, hi - x)."""
    mid = (lo + hi) / 2.0
    if b < mid:
        return b - lo
    if a > mid:
        return hi - a
    return (hi - lo) / 2.0


def _out_max(a: float, b: float, lo: float, hi: float) -> float:
    """max over x in [a, b] of max(0, lo - x, x - hi)."""
    return max(0.0, lo - a, b - hi)


def _bound(
    terms: Sequence[Term], c_lo: Sequence[float], c_hi: Sequence[float]
) -> tuple[float, bool]:
    """An upper bound on the function over the cell, and whether the binding term is limited by
    a polygon's distance in x-y (so that splitting z would not help). Boxes and height bands
    are bounded exactly, axis by axis; a polygon's x-y distance by its value at the cell's
    centre plus the cell's x-y half-diagonal (it is 1-Lipschitz)."""
    best, xy_bound = math.inf, False
    cx, cy = (c_lo[0] + c_hi[0]) / 2.0, (c_lo[1] + c_hi[1]) / 2.0
    hx, hy = (c_hi[0] - c_lo[0]) / 2.0, (c_hi[1] - c_lo[1]) / 2.0
    r_xy = math.sqrt(hx * hx + hy * hy)
    for zone, sign in terms:
        limited_by_xy = False
        if zone.box_min_m is not None:
            lo, hi = zone.box_min_m, zone.box_max_m
            if sign > 0:
                ub = min(_tent_max(c_lo[i], c_hi[i], lo[i], hi[i]) for i in range(3))
            else:
                e = [_out_max(c_lo[i], c_hi[i], lo[i], hi[i]) for i in range(3)]
                ub = math.sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2])
        else:
            dxy = geometry.polygon_signed_distance(zone.polygon_m, cx, cy)
            z = zone.z_m
            if sign > 0:
                ub_xy = dxy + r_xy
                ub = ub_xy if z is None else min(ub_xy, _tent_max(c_lo[2], c_hi[2], z[0], z[1]))
                limited_by_xy = ub == ub_xy
            else:
                o_xy = max(0.0, 0.0 - dxy + r_xy)
                o_z = 0.0 if z is None else _out_max(c_lo[2], c_hi[2], z[0], z[1])
                ub = math.sqrt(o_xy * o_xy + o_z * o_z)
                limited_by_xy = o_xy > 0.0 and o_z == 0.0
        if ub < best:
            best, xy_bound = ub, limited_by_xy
    return best, xy_bound


def _maximize(
    terms: Sequence[Term],
    lo: Sequence[float],
    hi: Sequence[float],
    thresholds: Sequence[float],
    budget: int = MAX_EVALUATIONS,
) -> Depth:
    """Bound the maximum of min over terms by branch and bound over the box [lo, hi], far
    enough to decide, for each threshold t, whether the maximum is at least t."""
    extent = [hi[i] - lo[i] for i in range(3)]
    longest = max(extent)
    counts = [
        max(1, min(8, math.ceil(e * 8.0 / longest))) if e > 0.0 and longest > 0.0 else 1
        for e in extent
    ]
    heap: list = []
    tick = 0
    best = -math.inf
    best_at = (lo[0], lo[1], lo[2])
    evaluations = 0

    def push(c_lo: Sequence[float], c_hi: Sequence[float]) -> None:
        nonlocal tick, best, best_at, evaluations
        centre = tuple((c_lo[i] + c_hi[i]) / 2.0 for i in range(3))
        v = _value(terms, centre)
        ub, xy = _bound(terms, c_lo, c_hi)
        evaluations += 1
        if v > best:
            best, best_at = v, centre  # type: ignore[assignment]
        heapq.heappush(heap, (0.0 - max(ub, v), tick, tuple(c_lo), tuple(c_hi), xy))
        tick += 1

    for i in range(counts[0]):
        for j in range(counts[1]):
            for k in range(counts[2]):
                idx = (i, j, k)
                c_lo = [lo[a] + extent[a] * idx[a] / counts[a] for a in range(3)]
                c_hi = [lo[a] + extent[a] * (idx[a] + 1) / counts[a] for a in range(3)]
                push(c_lo, c_hi)
    while True:
        neg_bound, _, c_lo, c_hi, xy = heap[0]
        bound = max(0.0 - neg_bound, best)
        if all(best >= t or bound < t for t in thresholds):
            return Depth(best, bound, best_at, True)
        if evaluations + 2 > budget:
            return Depth(best, bound, best_at, False)
        heapq.heappop(heap)
        sizes = [c_hi[a] - c_lo[a] for a in range(3)]
        axes = (0, 1) if xy else (0, 1, 2)
        axis = max(axes, key=lambda a: sizes[a])
        mid = (c_lo[axis] + c_hi[axis]) / 2.0
        left_hi = list(c_hi)
        left_hi[axis] = mid
        right_lo = list(c_lo)
        right_lo[axis] = mid
        push(c_lo, left_hi)
        push(right_lo, c_hi)


def _aabb(zone: Zone) -> tuple[list[float], list[float]]:
    """Axis-aligned bounds of a zone; z is +-inf for a prism without a height band."""
    if zone.box_min_m is not None:
        return list(zone.box_min_m), list(zone.box_max_m)
    xs = [v[0] for v in zone.polygon_m]
    ys = [v[1] for v in zone.polygon_m]
    z = zone.z_m if zone.z_m is not None else (-math.inf, math.inf)
    return [min(xs), min(ys), z[0]], [max(xs), max(ys), z[1]]


def _search_box(zones: Sequence[Zone], all_zones: Sequence[Zone]) -> tuple[list, list] | None:
    """The intersection of the zones' bounds, with infinite z replaced by every finite height
    of ``all_zones`` widened by _Z_MARGIN_M (or 0 when nothing is finite). None if empty."""
    lo, hi = [-math.inf] * 3, [math.inf] * 3
    for z in zones:
        a, b = _aabb(z)
        for i in range(3):
            lo[i], hi[i] = max(lo[i], a[i]), min(hi[i], b[i])
    for i in range(3):
        if lo[i] > hi[i]:
            return None
    if not math.isfinite(lo[2]) or not math.isfinite(hi[2]):
        finite = [
            v for z in all_zones for v in (_aabb(z)[0][2], _aabb(z)[1][2]) if math.isfinite(v)
        ]
        z_lo = (min(finite) - _Z_MARGIN_M) if finite else 0.0
        z_hi = (max(finite) + _Z_MARGIN_M) if finite else 0.0
        if math.isfinite(lo[2]):
            z_lo = lo[2]
        if math.isfinite(hi[2]):
            z_hi = hi[2]
        lo[2], hi[2] = z_lo, max(z_lo, z_hi)
    return lo, hi


def region_depth(
    envelope: Envelope,
    point_id: str,
    thresholds: Sequence[float],
    keep_outs: bool = True,
) -> Depth | None:
    """The depth of a point's allowed region, the largest clearance of any position, bounded
    far enough to decide each threshold. With ``keep_outs`` False, of its keep_in zones alone.
    None when the point has no keep_in zone, or when the bounds of its keep_in zones do not even
    overlap (the region is empty)."""
    zones = envelope.zones_for(point_id)
    keep_ins = [z for z in zones if z.kind == "keep_in"]
    if not keep_ins:
        return None
    box = _search_box(keep_ins, zones)
    if box is None:
        return None
    used = zones if keep_outs else keep_ins
    terms = [(z, 1 if z.kind == "keep_in" else -1) for z in used]
    return _maximize(terms, box[0], box[1], thresholds)


def zone_depth(zone: Zone, thresholds: Sequence[float]) -> Depth:
    """How deep inside a zone a position can be: the largest signed distance (its inradius),
    exact for a box."""
    if zone.box_min_m is not None:
        lo, hi = zone.box_min_m, zone.box_max_m
        half = min((hi[i] - lo[i]) / 2.0 for i in range(3))
        centre = tuple((lo[i] + hi[i]) / 2.0 for i in range(3))
        return Depth(half, half, centre, True)  # type: ignore[arg-type]
    a, b = _aabb(zone)
    if zone.z_m is None:
        a[2] = b[2] = 0.0
    return _maximize([(zone, 1)], a, b, thresholds)


def _overlap_depth(envelope: Envelope, point_id: str, keep_out: Zone) -> Depth | None:
    """The largest value, over the point's keep_in region, of min(keep_in clearances, depth
    inside keep_out): at least DEPTH_EPS_M when the keep_out zone meets the keep_in region."""
    zones = envelope.zones_for(point_id)
    keep_ins = [z for z in zones if z.kind == "keep_in"]
    if not keep_ins:
        return None
    box = _search_box([*keep_ins, keep_out], zones)
    if box is None:
        return Depth(-math.inf, -math.inf, (0.0, 0.0, 0.0), True)

    terms = [(keep_out, 1), *[(z, 1) for z in keep_ins]]
    return _maximize(terms, box[0], box[1], (DEPTH_EPS_M,))


# --------------------------------------------------------------------------- the lint


def _finding(code: str, detail: str, where: str = "") -> Finding:
    return Finding(code, LINT_CODES[code][0], detail, where)


def _mm(x: float) -> str:
    return f"{x * 1000.0:.1f} mm"


class _Where:
    def __init__(self, envelope: Envelope):
        self.paths = _Paths(from_0_1=envelope.upgraded_from == "0.1")
        self.point_index = {pid: i for i, pid in enumerate(envelope.points)}
        self.zone_index = {z.id: i for i, z in enumerate(envelope.zones)}
        self.from_0_1 = self.paths.from_0_1

    def point(self, pid: str) -> str:
        return self.paths.point(self.point_index[pid])

    def zone(self, zid: str) -> str:
        return self.paths.zone(self.zone_index[zid])


def lint(
    envelope: Envelope,
    *,
    physical_max_speed_mps: float | None = None,
    ev03_profile: Mapping | None = None,
    ev10_profile: Mapping | None = None,
) -> list[Finding]:
    """Lint a loaded envelope. ``physical_max_speed_mps`` is the lab's v_phys, when known.
    The profiles default to the published ``ev03-illustrative-0.1`` and
    ``ev10-illustrative-0.1``. Findings come in a fixed order: per point, per zone, then
    stop, speed, proximity and joints."""
    p03 = dict(ev03_profile) if ev03_profile is not None else load_profile("ev03-illustrative-0.1")
    p10 = dict(ev10_profile) if ev10_profile is not None else load_profile("ev10-illustrative-0.1")
    tol = p03["tolerance_m"]
    ev10_inside = max(p10["waypoint_inside_tolerance_factor"] * p10["tolerance_m"],
                      p10["waypoint_inside_min_m"])
    where = _Where(envelope)
    stop = envelope.stop
    out: list[Finding] = []
    depths: dict[str, Depth] = {}

    # ----- per point: the allowed region
    for pid in envelope.points:
        zones = envelope.zones_for(pid)
        keep_ins = [z for z in zones if z.kind == "keep_in"]
        if not keep_ins:
            continue  # the loader already warns ENVELOPE_POINT_NO_KEEP_IN
        if not any(z.box_min_m is not None or z.z_m is not None for z in keep_ins):
            out.append(_finding(
                "LINT_KEEP_IN_UNBOUNDED_Z",
                f"point {pid!r}: its keep_in zones are prisms without z_m, so its height is not "
                "bounded",
                where.point(pid),
            ))
        # A region counts as empty when no position with positive clearance is found and the
        # search shows none is tolerance_m inside. (Proving "exactly empty" by search is out of
        # reach when keep_out faces lie exactly on keep_in faces; tolerance_m is the scale that
        # matters to a verdict.)
        alone = region_depth(envelope, pid, (tol,), keep_outs=False)
        if alone is None or (alone.best <= 0.0 and alone.bound <= tol):
            names = ", ".join(repr(z.id) for z in keep_ins)
            out.append(_finding(
                "LINT_KEEP_IN_DISJOINT",
                f"point {pid!r}: its keep_in zones ({names}) have no position in common"
                + ("" if alone is None else f" (none is {_mm(alone.bound)} inside all of them)"),
                where.point(pid),
            ))
            continue
        stops = [stop["max_distance_m"]] + [s["max_distance_m"] for s in stop.get("sources", [])]
        depth = region_depth(envelope, pid, (tol, ev10_inside, *[d for d in stops if d > 0]))
        assert depth is not None
        if depth.best <= 0.0 and depth.bound <= tol:
            outs = ", ".join(repr(z.id) for z in zones if z.kind == "keep_out")
            out.append(_finding(
                "LINT_KEEP_OUT_COVERS_KEEP_IN",
                f"point {pid!r}: keep_out zones ({outs}) cover its keep_in region; no position "
                f"has positive clearance, and none is {_mm(depth.bound)} inside",
                where.point(pid),
            ))
            continue
        if not depth.exact or depth.best <= 0.0:
            out.append(_finding(
                "LINT_DEPTH_UNDECIDED",
                f"point {pid!r}: the deepest position found is {_mm(depth.best)} inside, and the "
                f"search could not rule out one {_mm(depth.bound)} inside",
                where.point(pid),
            ))
            if depth.best <= 0.0:
                continue
        depths[pid] = depth
        if depth.bound < tol:
            out.append(_finding(
                "LINT_ALLOWED_REGION_THINNER_THAN_TOLERANCE",
                f"point {pid!r}: the deepest position is {_mm(depth.best)} inside, less than "
                f"tolerance_m ({_mm(tol)})",
                where.point(pid),
            ))
        if depth.bound < ev10_inside:
            out.append(_finding(
                "LINT_ALLOWED_REGION_TOO_THIN_FOR_EV10",
                f"point {pid!r}: the deepest position is {_mm(depth.best)} inside; an EV-10 "
                f"waypoint must be {_mm(ev10_inside)} inside",
                where.point(pid),
            ))

    # ----- per zone: keep_outs that cannot be enforced or exclude nothing; duplicates
    seen: list[Zone] = []
    for zone in envelope.zones:
        for other in seen:
            if (other.kind, other.box_min_m, other.box_max_m, other.polygon_m, other.z_m) == (
                zone.kind, zone.box_min_m, zone.box_max_m, zone.polygon_m, zone.z_m
            ):
                out.append(_finding(
                    "LINT_ZONE_DUPLICATE",
                    f"zone {zone.id!r} has the same kind and shape as zone {other.id!r}",
                    where.zone(zone.id),
                ))
                break
        seen.append(zone)
        if zone.kind != "keep_out":
            continue
        # Unenforceable when no position is more than tolerance_m inside (to within DEPTH_EPS_M).
        inner = zone_depth(zone, (tol + DEPTH_EPS_M,))
        if inner.bound < tol + DEPTH_EPS_M:
            out.append(_finding(
                "LINT_KEEP_OUT_UNENFORCEABLE",
                f"keep_out zone {zone.id!r} is at most {_mm(inner.bound)} deep, no more than "
                f"tolerance_m ({_mm(tol)}): a point can cross it without a position excursion",
                where.zone(zone.id),
            ))
        for pid in envelope.points:
            if not zone.applies(pid) or pid not in depths:
                continue
            overlap = _overlap_depth(envelope, pid, zone)
            if overlap is not None and overlap.bound < DEPTH_EPS_M:
                out.append(_finding(
                    "LINT_KEEP_OUT_UNREACHABLE",
                    f"keep_out zone {zone.id!r} does not meet the keep_in region of point {pid!r}",
                    where.zone(zone.id),
                ))

    # ----- stop
    distance = stop["max_distance_m"]
    for pid, depth in depths.items():
        if distance > depth.bound:
            over = [
                s["source"] for s in stop.get("sources", []) if s["max_distance_m"] > depth.bound
            ]
            also = f"; so do the sources {', '.join(over)}" if over else ""
            out.append(_finding(
                "LINT_STOP_DISTANCE_EXCEEDS_DEPTH",
                f"point {pid!r}: stop.max_distance_m {_mm(distance)} is longer than the deepest "
                f"position of its allowed region ({_mm(depth.bound)}){also}",
                "/stop/max_distance_m",
            ))
    speeds = {pid: envelope.speed_limit_mps(pid) for pid in envelope.points}
    fastest = max(speeds.values())
    if distance == 0:
        out.append(_finding(
            "LINT_STOP_DISTANCE_ZERO",
            f"stop.max_distance_m is 0 with a stop time of {stop['max_time_ms']} ms and a speed "
            f"limit of {fastest:g} m/s",
            "/stop/max_distance_m",
        ))
    elif distance > fastest * stop["max_time_ms"] / 1000.0:
        out.append(_finding(
            "LINT_STOP_DISTANCE_ABOVE_SPEED_TIME",
            f"stop.max_distance_m {_mm(distance)} is longer than {fastest:g} m/s x "
            f"{stop['max_time_ms']} ms = {_mm(fastest * stop['max_time_ms'] / 1000.0)}",
            "/stop/max_distance_m",
        ))
    if "sources" in stop:
        listed = {s["source"] for s in stop["sources"]}
        for source in STOP_SOURCES:
            if source not in listed:
                out.append(_finding(
                    "LINT_STOP_SOURCE_MISSING",
                    f"stop.sources lists {len(listed)} source(s) but not {source!r}; it falls back "
                    "to the top-level stop, which may not be what was meant",
                    "/stop/sources",
                ))

    # ----- speed limits
    for pid, v in speeds.items():
        own = envelope.points[pid].max_speed_mps is not None
        at = where.point(pid) if own and not where.from_0_1 else "/motion/max_speed_mps"
        if physical_max_speed_mps is not None and v > physical_max_speed_mps:
            out.append(_finding(
                "LINT_SPEED_LIMIT_ABOVE_PHYSICAL_MAX",
                f"point {pid!r}: its speed limit {v:g} m/s is above the physical maximum "
                f"{physical_max_speed_mps:g} m/s",
                at,
            ))
        elif v > SPEED_IMPLAUSIBLE_MPS:
            out.append(_finding(
                "LINT_SPEED_LIMIT_IMPLAUSIBLE",
                f"point {pid!r}: its speed limit {v:g} m/s is above {SPEED_IMPLAUSIBLE_MPS:g} m/s",
                at,
            ))
        if own and not envelope.zones_for(pid):
            out.append(_finding(
                "LINT_SPEED_LIMIT_ON_UNREQUIRED_POINT",
                f"point {pid!r} declares max_speed_mps {v:g} but no zone applies to it, so the "
                "scorer does not require it to be measured",
                where.point(pid),
            ))
    required = [pid for pid in envelope.points if envelope.zones_for(pid)]
    reach = [
        speeds[pid] * (1.0 + p03["tolerance_speed_frac"]) * p03["motion_window_s"]
        for pid in required
    ]
    if required and max(reach) <= p03["motion_min_displacement_m"]:
        out.append(_finding(
            "LINT_SPEED_LIMIT_BLOCKS_MOTION",
            f"at their speed limits the required points move at most {_mm(max(reach))} in a "
            f"{p03['motion_window_s']:g} s motion window; the motion rule needs more than "
            f"{_mm(p03['motion_min_displacement_m'])}",
            "/motion/max_speed_mps",
        ))

    # ----- proximity
    rules = []
    for i, rule in enumerate(envelope.get("proximity", None) or []):
        limit = 0.0 if rule.get("action") == "stop" else rule["max_speed_mps"]
        rules.append((rule["when"]["human_within_m"], i, limit))
    for d_a, i, v_a in rules:
        for d_b, k, v_b in rules:
            if k <= i:
                continue
            if d_a == d_b:
                out.append(_finding(
                    "LINT_PROXIMITY_DUPLICATE_DISTANCE",
                    f"proximity rules {i} and {k} both apply within {d_a:g} m",
                    f"/proximity/{k}",
                ))
                continue
            (d_near, n, v_near), (d_far, f, v_far) = sorted(((d_a, i, v_a), (d_b, k, v_b)))
            if v_near > v_far:
                out.append(_finding(
                    "LINT_PROXIMITY_CLOSER_FASTER",
                    f"proximity rule {n} (person within {d_near:g} m) allows {v_near:g} m/s, "
                    f"faster than rule {f} (within {d_far:g} m, {v_far:g} m/s)",
                    f"/proximity/{n}",
                ))
    for d, i, v in rules:
        if v > 0.0 and v >= fastest:
            out.append(_finding(
                "LINT_PROXIMITY_SPEED_NOT_A_LIMIT",
                f"proximity rule {i} allows {v:g} m/s, at or above every speed limit "
                f"({fastest:g} m/s)",
                f"/proximity/{i}",
            ))

    # ----- joints
    motion = envelope.motion
    cap = motion.get("max_joint_speed_radps")
    for i, joint in enumerate(motion.get("joints", [])):
        speed = joint.get("max_speed_radps")
        if cap is not None and speed is not None and speed > cap:
            out.append(_finding(
                "LINT_JOINT_SPEED_ABOVE_CAP",
                f"joint {joint['name']!r}: max_speed_radps {speed:g} is above "
                f"motion.max_joint_speed_radps {cap:g}",
                f"/motion/joints/{i}/max_speed_radps",
            ))
        r = joint.get("position_rad")
        if r is not None and r[1] - r[0] > 2.0 * math.pi:
            out.append(_finding(
                "LINT_JOINT_RANGE_VACUOUS",
                f"joint {joint['name']!r}: position_rad [{r[0]:g}, {r[1]:g}] spans more than "
                "a full turn",
                f"/motion/joints/{i}/position_rad",
            ))
    return out
