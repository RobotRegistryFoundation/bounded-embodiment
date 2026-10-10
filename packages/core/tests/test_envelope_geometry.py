"""Zone geometry: exact signed distances, checked against brute-force sampling of each surface,
and the validity predicates for polygons."""

from __future__ import annotations

import math
import random
from fractions import Fraction

import pytest

from bounded_embodiment.envelope import Zone
from bounded_embodiment.envelope import geometry as g


# ----- independent helpers (no code shared with geometry.py) -----------------------------------
def dist(a, b) -> float:
    s = 0.0
    for i in range(len(a)):
        d = a[i] - b[i]
        s += d * d
    return math.sqrt(s)


def winding_inside(poly, x, y) -> bool:
    """Winding-number point-in-polygon, independent of geometry.point_in_polygon (even-odd)."""
    wn = 0
    n = len(poly)
    for i in range(n):
        (x0, y0), (x1, y1) = poly[i], poly[(i + 1) % n]
        cross = (x1 - x0) * (y - y0) - (x - x0) * (y1 - y0)
        if y0 <= y:
            if y1 > y and cross > 0:
                wn += 1
        elif y1 <= y and cross < 0:
            wn -= 1
    return wn != 0


def ticks(a, b, h):
    n = max(1, math.ceil((b - a) / h))
    return [a + (b - a) * k / n for k in range(n + 1)]


def box_surface(lo, hi, h):
    t = [ticks(lo[i], hi[i], h) for i in range(3)]
    pts = []
    for axis in range(3):
        u, v = [i for i in range(3) if i != axis]
        for c in (lo[axis], hi[axis]):
            for a in t[u]:
                for b in t[v]:
                    p = [0.0, 0.0, 0.0]
                    p[axis], p[u], p[v] = c, a, b
                    pts.append(p)
    return pts


def edge_samples(poly, h):
    out = []
    n = len(poly)
    for i in range(n):
        (x0, y0), (x1, y1) = poly[i], poly[(i + 1) % n]
        length = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
        k = max(1, math.ceil(length / h))
        out.extend((x0 + (x1 - x0) * j / k, y0 + (y1 - y0) * j / k) for j in range(k))
    return out


def prism_surface(poly, z, h):
    rim = edge_samples(poly, h)
    pts = [(x, y, zz) for (x, y) in rim for zz in ticks(z[0], z[1], h)]
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    cap = [
        (x, y)
        for x in ticks(min(xs), max(xs), h)
        for y in ticks(min(ys), max(ys), h)
        if winding_inside(poly, x, y)
    ]
    for zz in z:
        pts.extend((x, y, zz) for (x, y) in cap)
    return pts


def queries(lo, hi, n, rng, margin):
    return [[rng.uniform(lo[i] - margin, hi[i] + margin) for i in range(3)] for _ in range(n)]


BOX = ((0.20, -0.15, 0.05), (0.40, 0.15, 0.30))
CONVEX = [(0.0, 0.0), (0.30, -0.05), (0.42, 0.12), (0.25, 0.30), (0.02, 0.22)]  # CCW pentagon
U_SHAPE = [
    (0.0, 0.0),
    (0.3, 0.0),
    (0.3, 0.3),
    (0.2, 0.3),
    (0.2, 0.1),
    (0.1, 0.1),
    (0.1, 0.3),
    (0.0, 0.3),
]
L_SHAPE_CW = [(0, 0), (0, 6), (3, 6), (3, 3), (8, 3), (8, 0)]


# ----- signed distances against brute force ----------------------------------------------------
def test_box_signed_distance_matches_brute_force():
    lo, hi = BOX
    h = 0.01
    surface = box_surface(lo, hi, h)
    rng = random.Random(1)
    for q in queries(lo, hi, 150, rng, 0.08) + [list(p) for p in rng.sample(surface, 20)]:
        sd = g.box_signed_distance(lo, hi, q)
        brute = min(dist(q, s) for s in surface)
        assert abs(sd) <= brute + 1e-12
        assert brute <= abs(sd) + h / math.sqrt(2) + 1e-12
        inside = all(lo[i] <= q[i] <= hi[i] for i in range(3))
        assert (sd >= 0) == inside or sd == 0.0


@pytest.mark.parametrize(
    "poly", [CONVEX, U_SHAPE, list(reversed(U_SHAPE))], ids=["convex", "u", "u-cw"]
)
def test_prism_signed_distance_matches_brute_force(poly):
    z = (0.05, 0.25)
    h = 0.01
    surface = prism_surface(poly, z, h)
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    lo, hi = (min(xs), min(ys), z[0]), (max(xs), max(ys), z[1])
    rng = random.Random(2)
    for q in queries(lo, hi, 120, rng, 0.06):
        sd = g.prism_signed_distance(poly, z, q)
        brute = min(dist(q, s) for s in surface)
        assert abs(sd) <= brute + 1e-12, (q, sd, brute)
        assert brute <= abs(sd) + h + 1e-12, (q, sd, brute)
        inside = winding_inside(poly, q[0], q[1]) and z[0] <= q[2] <= z[1]
        assert (sd > 0) == inside or abs(sd) < 1e-12


def test_unbounded_prism_matches_2d_brute_force_on_a_nonconvex_clockwise_polygon():
    h = 0.01
    rim = edge_samples(L_SHAPE_CW, h)
    rng = random.Random(3)
    for _ in range(150):
        q = (rng.uniform(-1, 9), rng.uniform(-1, 7), rng.uniform(-50, 50))
        sd = g.prism_signed_distance(L_SHAPE_CW, None, q)
        brute = min(dist(q[:2], s) for s in rim)
        assert abs(sd) <= brute + 1e-12
        assert brute <= abs(sd) + h / 2 + 1e-12
        assert (sd > 0) == winding_inside(L_SHAPE_CW, q[0], q[1]) or abs(sd) < 1e-12


@pytest.mark.parametrize("shape", ["box", "convex", "u"])
def test_signed_distance_is_1_lipschitz(shape):
    rng = random.Random(4)
    if shape == "box":

        def f(p):
            return g.box_signed_distance(*BOX, p)
    else:
        poly = CONVEX if shape == "convex" else U_SHAPE

        def f(p):
            return g.prism_signed_distance(poly, (0.05, 0.25), p)

    for _ in range(400):
        p = [rng.uniform(-0.1, 0.5) for _ in range(3)]
        q = [c + rng.uniform(-0.03, 0.03) for c in p]
        assert abs(f(p) - f(q)) <= dist(p, q) + 1e-12


def test_signed_distance_is_zero_on_the_surface_and_never_negative_zero():
    lo, hi = BOX
    assert math.copysign(1.0, g.box_signed_distance(lo, hi, (0.20, 0.0, 0.1))) == 1.0
    assert g.box_signed_distance(lo, hi, (0.20, 0.0, 0.1)) == 0.0
    assert g.prism_signed_distance(CONVEX, (0.05, 0.25), (0.0, 0.0, 0.1)) == 0.0
    z = Zone(id="o", kind="keep_out", applies_to=("*",), box_min_m=lo, box_max_m=hi)
    assert math.copysign(1.0, z.clearance((0.20, 0.0, 0.1))) == 1.0


# ----- faces -----------------------------------------------------------------------------------
def test_box_faces_are_plane_distances_in_harness_order():
    lo, hi = BOX
    zone = Zone(id="b", kind="keep_in", applies_to=("*",), box_min_m=lo, box_max_m=hi)
    names = [n for n, _ in zone.faces()]
    assert names == ["x_min", "x_max", "y_min", "y_max", "z_min", "z_max"]
    rng = random.Random(5)
    for q in queries(lo, hi, 50, rng, 0.1):
        values = {n: fn(q) for n, fn in zone.faces()}
        assert values["x_min"] == q[0] - lo[0] and values["x_max"] == hi[0] - q[0]
        assert values["z_min"] == q[2] - lo[2] and values["z_max"] == hi[2] - q[2]
        if all(lo[i] <= q[i] <= hi[i] for i in range(3)):
            assert min(values.values()) == pytest.approx(zone.signed_distance(q), abs=1e-15)


@pytest.mark.parametrize("winding", ["ccw", "cw"])
def test_rectangle_prism_faces_equal_the_box_faces(winding):
    lo, hi = BOX
    rect = [(lo[0], lo[1]), (hi[0], lo[1]), (hi[0], hi[1]), (lo[0], hi[1])]
    # CCW edges: 0 = y_min, 1 = x_max, 2 = y_max, 3 = x_min. Reversed: 0 = y_max, 1 = x_max, ...
    names = ["y_min", "x_max", "y_max", "x_min"]
    if winding == "cw":
        rect = rect[::-1]
        names = ["y_max", "x_max", "y_min", "x_min"]
    prism = Zone.from_dict(
        {
            "id": "p",
            "kind": "keep_in",
            "applies_to": ["*"],
            "prism": {"polygon_m": [list(v) for v in rect], "z_m": [lo[2], hi[2]]},
        }
    )
    box = Zone(id="b", kind="keep_in", applies_to=("*",), box_min_m=lo, box_max_m=hi)
    box_faces = dict(box.faces())
    rng = random.Random(6)
    for q in queries(lo, hi, 200, rng, 0.1):
        assert prism.signed_distance(q) == pytest.approx(box.signed_distance(q), abs=1e-12)
        for name, fn in prism.faces():
            twin = names[int(name[5:])] if name.startswith("edge_") else name
            assert fn(q) == pytest.approx(box_faces[twin](q), abs=1e-12), (name, q)


@pytest.mark.parametrize("poly", [U_SHAPE, L_SHAPE_CW], ids=["u", "l-cw"])
def test_prism_faces_inside_are_distances_to_the_face_never_negative(poly):
    zone = Zone.from_dict(
        {
            "id": "p",
            "kind": "keep_in",
            "applies_to": ["*"],
            "prism": {"polygon_m": [list(v) for v in poly], "z_m": [0.0, 1.0]},
        }
    )
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    rng = random.Random(7)
    checked = 0
    while checked < 150:
        q = (rng.uniform(min(xs), max(xs)), rng.uniform(min(ys), max(ys)), rng.uniform(0.0, 1.0))
        sd = zone.signed_distance(q)
        if sd < 0:
            continue
        values = [fn(q) for _, fn in zone.faces()]
        assert min(values) >= 0.0
        assert min(values) == pytest.approx(sd, abs=1e-12)
        checked += 1


def test_prism_faces_outside_are_signed_plane_distances():
    zone = Zone.from_dict(
        {
            "id": "p",
            "kind": "keep_in",
            "applies_to": ["*"],
            "prism": {"polygon_m": [list(v) for v in CONVEX], "z_m": [0.05, 0.25]},
        }
    )
    faces = dict(zone.faces())
    # 0.1 m straight out through the middle of edge 0 (CONVEX is counter-clockwise).
    (x0, y0), (x1, y1) = CONVEX[0], CONVEX[1]
    length = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
    nx, ny = (y1 - y0) / length, -(x1 - x0) / length  # outward normal of a CCW edge
    q = ((x0 + x1) / 2 + 0.1 * nx, (y0 + y1) / 2 + 0.1 * ny, 0.15)
    assert faces["edge_0"](q) == pytest.approx(-0.1, abs=1e-12)
    assert zone.signed_distance(q) == pytest.approx(-0.1, abs=1e-12)
    assert faces["z_max"]((0.2, 0.1, 0.30)) == pytest.approx(-0.05, abs=1e-15)


def test_unbounded_prism_has_no_caps():
    zone = Zone.from_dict(
        {
            "id": "p",
            "kind": "keep_in",
            "applies_to": ["*"],
            "prism": {"polygon_m": [list(v) for v in CONVEX]},
        }
    )
    assert [n for n, _ in zone.faces()] == [f"edge_{i}" for i in range(len(CONVEX))]


# ----- polygon validity ------------------------------------------------------------------------
@pytest.mark.parametrize(
    "poly",
    [
        [(0, 0), (1, 0), (1, 1), (0, 1)],
        [(0, 0), (0, 1), (1, 1), (1, 0)],  # clockwise is fine
        U_SHAPE,
        L_SHAPE_CW,
        [(0, 0), (2, 0), (2, 1), (1, 1), (0, 1)],  # a straight (180 degree) vertex is fine
        [(0, 0), (1e-9, 0), (0, 1e-9)],  # tiny but not degenerate
        [(0, 0), (1, 0), (0.5, 1e-300)],  # area is positive, exactly
    ],
)
def test_simple_polygons_have_no_problems(poly):
    assert g.polygon_problems(poly) == []


@pytest.mark.parametrize(
    "poly, kind",
    [
        ([(0, 0), (1, 1), (1, 0), (0, 1)], "not_simple"),  # bow tie
        (
            [(0, 0), (4, 0), (4, 4), (4, 2), (0, 4)],
            "not_simple",
        ),  # edge folds back on the previous one
        (
            [(0, 0), (4, 0), (4, 4), (2, 0), (0, 4)],
            "not_simple",
        ),  # vertex touches a non-adjacent edge
        (
            [(0, 0), (2, 0), (1, 1), (2, 2), (0, 2), (1, 1)],
            "not_simple",
        ),  # pinched: a vertex repeated
        ([(0, 0), (1, 1), (2, 2)], "degenerate"),  # collinear
        ([(0.1, 0.1), (0.2, 0.2), (0.3, 0.3)], "degenerate"),  # collinear in floating point too
        ([(0, 0), (1, 0), (1, 0), (0, 1)], "degenerate"),  # repeated consecutive vertex
        ([(0, 0), (1, 0), (1, 1), (0, 0)], "degenerate"),  # first vertex repeated at the end
    ],
)
def test_bad_polygons_are_reported(poly, kind):
    problems = g.polygon_problems(poly)
    assert problems and problems[0][0] == kind, problems


def test_closing_vertex_gets_a_helpful_message():
    ((kind, detail),) = g.polygon_problems([(0, 0), (1, 0), (1, 1), (0, 0)])
    assert kind == "degenerate" and "closed implicitly" in detail


def test_orientation_predicate_is_exact_on_nearly_collinear_points():
    rng = random.Random(8)
    for _ in range(2000):
        a = (rng.uniform(-10, 10), rng.uniform(-10, 10))
        b = (rng.uniform(-10, 10), rng.uniform(-10, 10))
        t = rng.uniform(-2, 3)
        c = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))  # collinear up to rounding
        exact = (Fraction(b[0]) - Fraction(a[0])) * (Fraction(c[1]) - Fraction(a[1])) - (
            Fraction(b[1]) - Fraction(a[1])
        ) * (Fraction(c[0]) - Fraction(a[0]))
        assert g._orient(a, b, c) == (exact > 0) - (exact < 0)


def test_winding_is_read_from_the_exact_area():
    ccw = Zone.from_dict(
        {
            "id": "p",
            "kind": "keep_in",
            "applies_to": ["*"],
            "prism": {"polygon_m": [[0, 0], [1, 0], [0, 1]]},
        }
    )
    cw = Zone.from_dict(
        {
            "id": "p",
            "kind": "keep_in",
            "applies_to": ["*"],
            "prism": {"polygon_m": [[0, 0], [0, 1], [1, 0]]},
        }
    )
    assert ccw.ccw and not cw.ccw
    for q in [(0.2, 0.2, 0), (0.6, 0.6, 0), (-0.3, 0.4, 0)]:
        bottom_ccw = dict(ccw.faces())["edge_0"]  # (0, 0) -> (1, 0)
        bottom_cw = dict(cw.faces())["edge_2"]  # (1, 0) -> (0, 0)
        assert bottom_ccw(q) == pytest.approx(bottom_cw(q))
