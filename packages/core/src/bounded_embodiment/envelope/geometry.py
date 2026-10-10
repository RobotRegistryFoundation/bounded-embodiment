"""Exact Euclidean geometry for envelope zones.

Points are ``(x, y, z)`` in metres, in the envelope's frame. A *signed distance* is positive
inside a solid (the distance to its surface) and negative outside it (minus the distance to it);
it is 0.0 on the surface. Boxes are axis-aligned; prisms are a simple polygon in x-y,
optionally limited to a height band ``[zmin, zmax]``.

Only ``+ - * /`` and ``math.sqrt`` touch coordinates, each in a fixed order, so results are
bit-identical on every IEEE-754 platform. Validity predicates (orientation, intersection, area)
fall back to exact rational arithmetic whenever floating point cannot decide the sign.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from fractions import Fraction

BOX_FACES = (
    ("x_min", 0, -1),
    ("x_max", 0, 1),
    ("y_min", 1, -1),
    ("y_max", 1, 1),
    ("z_min", 2, -1),
    ("z_max", 2, 1),
)


# ----- boxes -----------------------------------------------------------------------------------
def box_signed_distance(lo: Sequence[float], hi: Sequence[float], p: Sequence[float]) -> float:
    """Signed distance from p to the surface of the box [lo, hi]: the depth inside, or minus the
    Euclidean distance to the box from outside."""
    outside_sq = 0.0
    outside = False
    depth = math.inf
    for i in range(3):
        below = lo[i] - p[i]
        above = p[i] - hi[i]
        if below > 0.0:
            outside = True
            outside_sq += below * below
        elif above > 0.0:
            outside = True
            outside_sq += above * above
        else:
            depth = min(depth, -below, -above)
    if outside:
        return -math.sqrt(outside_sq)
    return depth + 0.0  # normalises -0.0 to 0.0 on the surface


def box_face_distance(
    lo: Sequence[float], hi: Sequence[float], axis: int, side: int, p: Sequence[float]
) -> float:
    """Distance from p to the plane of one box face, positive on the box's side of that plane."""
    if side < 0:
        return p[axis] - lo[axis] + 0.0
    return hi[axis] - p[axis] + 0.0


# ----- exact predicates ------------------------------------------------------------------------
def _orient(a: Sequence[float], b: Sequence[float], c: Sequence[float]) -> int:
    """Sign of the cross product (b - a) x (c - a): +1 left turn, -1 right turn, 0 collinear.
    A float filter decides the clear cases; the rest are computed exactly."""
    t1 = (b[0] - a[0]) * (c[1] - a[1])
    t2 = (b[1] - a[1]) * (c[0] - a[0])
    det = t1 - t2
    bound = 1e-12 * (abs(t1) + abs(t2))
    if det > bound:
        return 1
    if det < -bound:
        return -1
    fa = (Fraction(a[0]), Fraction(a[1]))
    e = (Fraction(b[0]) - fa[0]) * (Fraction(c[1]) - fa[1]) - (Fraction(b[1]) - fa[1]) * (
        Fraction(c[0]) - fa[0]
    )
    return (e > 0) - (e < 0)


def _on_segment(a: Sequence[float], b: Sequence[float], p: Sequence[float]) -> bool:
    """For p collinear with a-b: whether p lies on the closed segment a-b."""
    return min(a[0], b[0]) <= p[0] <= max(a[0], b[0]) and min(a[1], b[1]) <= p[1] <= max(a[1], b[1])


def segments_intersect(
    p1: Sequence[float], p2: Sequence[float], p3: Sequence[float], p4: Sequence[float]
) -> bool:
    """Whether the closed segments p1-p2 and p3-p4 share any point (touching counts)."""
    if (
        max(p1[0], p2[0]) < min(p3[0], p4[0])
        or max(p3[0], p4[0]) < min(p1[0], p2[0])
        or max(p1[1], p2[1]) < min(p3[1], p4[1])
        or max(p3[1], p4[1]) < min(p1[1], p2[1])
    ):
        return False
    d1 = _orient(p3, p4, p1)
    d2 = _orient(p3, p4, p2)
    d3 = _orient(p1, p2, p3)
    d4 = _orient(p1, p2, p4)
    if d1 * d2 < 0 and d3 * d4 < 0:
        return True
    return (
        (d1 == 0 and _on_segment(p3, p4, p1))
        or (d2 == 0 and _on_segment(p3, p4, p2))
        or (d3 == 0 and _on_segment(p1, p2, p3))
        or (d4 == 0 and _on_segment(p1, p2, p4))
    )


def polygon_twice_area_exact(poly: Sequence[Sequence[float]]) -> Fraction:
    """Twice the signed area of the polygon (shoelace), exactly: positive for counter-clockwise."""
    n = len(poly)
    acc = Fraction(0)
    for i in range(n):
        x0, y0 = poly[i][0], poly[i][1]
        x1, y1 = poly[(i + 1) % n][0], poly[(i + 1) % n][1]
        acc += Fraction(x0) * Fraction(y1) - Fraction(x1) * Fraction(y0)
    return acc


def polygon_problems(poly: Sequence[Sequence[float]]) -> list[tuple[str, str]]:
    """Why a polygon cannot bound a zone, as ``[(kind, detail)]`` with kind ``"degenerate"`` or
    ``"not_simple"``; empty when the polygon is simple and has positive area. Polygons are closed
    implicitly, so repeating the first vertex at the end is a zero-length edge (degenerate)."""
    n = len(poly)
    if n < 3:
        return [("degenerate", f"a polygon needs at least 3 vertices, got {n}")]
    out: list[tuple[str, str]] = []
    for i in range(n):
        j = (i + 1) % n
        if poly[i][0] == poly[j][0] and poly[i][1] == poly[j][1]:
            if j == 0:
                out.append(
                    (
                        "degenerate",
                        (
                            f"the last vertex ({n - 1}) repeats vertex 0; polygons "
                            "are closed implicitly, so do not repeat the first vertex"
                        ),
                    )
                )
            else:
                out.append(("degenerate", f"vertex {j} repeats vertex {i} (zero-length edge)"))
    if out:
        return out
    if all(_orient(poly[0], poly[1], poly[k]) == 0 for k in range(2, n)):
        return [("degenerate", "all vertices are collinear, so the polygon has no area")]
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        for j in range(i + 1, n):
            c, d = poly[j], poly[(j + 1) % n]
            if j == i + 1 or (i == 0 and j == n - 1):
                # Adjacent edges share one vertex; they must not fold back over each other.
                shared, other1, other2 = (b, a, d) if j == i + 1 else (a, b, c)
                if (
                    _orient(other1, shared, other2) == 0
                    and (other1[0] - shared[0]) * (other2[0] - shared[0])
                    + (other1[1] - shared[1]) * (other2[1] - shared[1])
                    > 0
                ):
                    out.append(("not_simple", f"edges {i} and {j} fold back over each other"))
                    return out
                continue
            if segments_intersect(a, b, c, d):
                out.append(("not_simple", f"edges {i} and {j} intersect"))
                return out
    if polygon_twice_area_exact(poly) == 0:  # unreachable for a simple polygon; kept as a guard
        out.append(("degenerate", "the polygon has zero area"))
    return out


# ----- polygons and prisms ---------------------------------------------------------------------
def _segment_distance_sq(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    dx, dy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    length_sq = dx * dx + dy * dy
    t = wx * dx + wy * dy
    if length_sq == 0.0 or t <= 0.0:
        return wx * wx + wy * wy
    if t >= length_sq:
        ex, ey = px - bx, py - by
        return ex * ex + ey * ey
    cross = dx * wy - dy * wx
    return cross * cross / length_sq


def point_in_polygon(px: float, py: float, poly: Sequence[Sequence[float]]) -> bool:
    """Even-odd rule. Points on the boundary may go either way; callers that need a sign use
    polygon_signed_distance, which returns 0.0 there."""
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i][0], poly[i][1]
        xj, yj = poly[j][0], poly[j][1]
        if (yi > py) != (yj > py):
            x_cross = xi + (py - yi) * (xj - xi) / (yj - yi)
            if px < x_cross:
                inside = not inside
        j = i
    return inside


def polygon_signed_distance(poly: Sequence[Sequence[float]], px: float, py: float) -> float:
    """Signed distance in the x-y plane from (px, py) to the polygon's boundary: positive inside."""
    n = len(poly)
    best = math.inf
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        best = min(best, _segment_distance_sq(px, py, a[0], a[1], b[0], b[1]))
    dist = math.sqrt(best)
    if dist == 0.0:
        return 0.0
    return dist if point_in_polygon(px, py, poly) else -dist


def prism_signed_distance(
    poly: Sequence[Sequence[float]], z_range: Sequence[float] | None, p: Sequence[float]
) -> float:
    """Signed distance from p to the surface of the vertical prism poly x z_range (unbounded in z
    when z_range is None)."""
    dxy = polygon_signed_distance(poly, p[0], p[1])
    if z_range is None:
        return dxy
    dz = min(p[2] - z_range[0], z_range[1] - p[2])
    if dxy >= 0.0 and dz >= 0.0:
        return min(dxy, dz) + 0.0
    ox = -dxy if dxy < 0.0 else 0.0
    oz = -dz if dz < 0.0 else 0.0
    return -math.sqrt(ox * ox + oz * oz)


def prism_edge_face_distance(
    poly: Sequence[Sequence[float]],
    z_range: Sequence[float] | None,
    ccw: bool,
    edge: int,
    p: Sequence[float],
) -> float:
    """Distance from p to the side face over polygon edge ``edge`` (vertex edge -> edge + 1).

    Inside the prism it is the Euclidean distance to that face (never negative, also next to the
    reflex corners of a non-convex polygon). Outside it is the signed distance to the face's plane,
    positive on the prism's side, so a negative value says how far p is past that plane. For a
    rectangle this is exactly the box face distance."""
    n = len(poly)
    a, b = poly[edge], poly[(edge + 1) % n]
    if prism_signed_distance(poly, z_range, p) >= 0.0:
        return math.sqrt(_segment_distance_sq(p[0], p[1], a[0], a[1], b[0], b[1])) + 0.0
    dx, dy = b[0] - a[0], b[1] - a[1]
    cross = dx * (p[1] - a[1]) - dy * (p[0] - a[0])
    d = cross / math.sqrt(dx * dx + dy * dy)
    return (d if ccw else -d) + 0.0


def prism_cap_face_distance(z_range: Sequence[float], side: int, p: Sequence[float]) -> float:
    """Distance from p to the plane of the bottom (side -1) or top (side +1) cap, positive on the
    prism's side."""
    if side < 0:
        return p[2] - z_range[0] + 0.0
    return z_range[1] - p[2] + 0.0
