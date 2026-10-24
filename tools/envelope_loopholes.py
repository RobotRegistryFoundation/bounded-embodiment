#!/usr/bin/env python3
"""Seeded search for envelopes that the schema accepts but that are unsafe, contradictory or
vacuous, checked against the loader and `be envelope lint`.

    python tools/envelope_loopholes.py --seed 7 --count 400          # search, print a table
    python tools/envelope_loopholes.py --seed 7 --count 50 --out DIR  # also write the cases
    python tools/envelope_loopholes.py --write-vectors                # (re)write vectors/envelope/lint/
    python tools/envelope_loopholes.py --check                        # compare with vectors/envelope/lint/

Every case is a random clean envelope (0.1 or 0.2) with one problem class applied. Each case
must pass the JSON schema of its version; then either the loader must reject it with the
expected ENVELOPE_* codes (the classes the loader already covers), or the lint must report the
expected LINT_* codes. A clean case must get no lint error or warning. The run exits 1 if any
case is missed, so the search doubles as a regression test (packages/core/tests/
test_envelope_lint.py runs it for a few seeds).

Randomness is random.Random(seed) and only its random() method, as in core's generators, so a
seed gives the same envelopes on every platform and Python. Numbers are rounded to 0.1 mm.
TEST-ONLY synthetic data; the vectors this writes are CC BY 4.0 (see REUSE.toml).
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import random
import sys
import tempfile
from collections.abc import Callable
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages" / "core" / "src"))

from bounded_embodiment import schema  # noqa: E402
from bounded_embodiment.envelope import EnvelopeError, load  # noqa: E402
from bounded_embodiment.envelope.lint import LINT_CODES, STOP_SOURCES, lint  # noqa: E402

VECTORS = ROOT / "vectors" / "envelope" / "lint"
VECTOR_SEED = 1

Case = dict  # {"name", "class", "version", "doc", "expect", "load_error", "options"}


# --------------------------------------------------------------------------- randomness


class Rng:
    def __init__(self, seed: int):
        self._r = random.Random(seed)

    def u(self, a: float, b: float) -> float:
        return a + (b - a) * self._r.random()

    def i(self, n: int) -> int:
        return min(int(self._r.random() * n), n - 1)

    def coin(self) -> bool:
        return self._r.random() < 0.5


def r4(x: float) -> float:
    return round(x, 4) + 0.0


# --------------------------------------------------------------------------- clean envelopes


def _convex_polygon(rng: Rng, cx: float, cy: float, radius: float) -> list[list[float]]:
    """Vertices on a circle in angular order: always convex and simple."""
    n = 4 + rng.i(5)
    # One vertex in each of n equal sectors, jittered: consecutive vertices are 0.5/n to 1.5/n
    # of a turn apart, so the polygon is convex, simple and holds its centre with room to spare.
    # Coordinates are rounded to 0.1 mm, which hides any last-bit difference in cos and sin.
    out = []
    for k in range(n):
        t = 2.0 * math.pi * (k + 0.5 * rng.u(0.0, 1.0)) / n
        out.append([r4(cx + radius * math.cos(t)), r4(cy + radius * math.sin(t))])
    return out


def _common(rng: Rng) -> dict:
    speed = r4(rng.u(0.25, 1.0))
    return {
        "motion": {"max_speed_mps": speed},
        "stop": {
            "category": 1,
            "max_time_ms": 300,
            "max_distance_m": r4(rng.u(0.005, 0.02)),
        },
        "heartbeat": {"model_timeout_ms": 500, "gate_timeout_ms": 200, "on_loss": "stop"},
        "proximity": [
            {"when": {"human_within_m": r4(rng.u(1.0, 2.0))}, "max_speed_mps": r4(speed * 0.5)},
            {"when": {"human_within_m": r4(rng.u(0.2, 0.6))}, "action": "stop"},
        ],
    }


def clean_0_2(rng: Rng) -> dict:
    """A clean 0.2 envelope: a tool keep_in box (or prism), an elbow in a larger cell, and a
    keep_out block well inside the tool's region."""
    c = [rng.u(-0.4, 0.4), rng.u(-0.4, 0.4), rng.u(0.3, 0.6)]
    h = [rng.u(0.08, 0.25) for _ in range(3)]
    if rng.coin():
        tool_zone = {"box": {"min_m": [r4(c[i] - h[i]) for i in range(3)],
                             "max_m": [r4(c[i] + h[i]) for i in range(3)]}}
    else:
        radius = min(h[0], h[1]) * 1.4
        tool_zone = {"prism": {"polygon_m": _convex_polygon(rng, c[0], c[1], radius),
                               "z_m": [r4(c[2] - h[2]), r4(c[2] + h[2])]}}
        h[0] = h[1] = radius * 0.5  # the inscribed square of the polygon's circle, roughly
    o = [rng.u(0.03, 0.4 * h[i]) for i in range(3)]
    oc = [c[i] + rng.u(-0.3, 0.3) * h[i] for i in range(3)]
    doc = {
        "envelope_version": "0.2",
        "machine": {"id": "loophole-search", "class": "arm",
                    "description": "TEST-ONLY synthetic envelope"},
        "frame": "base_link",
        "points": [
            {"id": "tool", "description": "TEST-ONLY tool point",
             "max_speed_mps": r4(rng.u(0.2, 0.25))},
            {"id": "elbow", "description": "TEST-ONLY elbow point"},
        ],
        "workspace": {"zones": [
            {"id": "tool_region", "kind": "keep_in", "applies_to": ["tool"], **tool_zone},
            {"id": "cell", "kind": "keep_in", "applies_to": ["elbow"],
             "box": {"min_m": [r4(c[i] - h[i] - 0.5) for i in range(3)],
                     "max_m": [r4(c[i] + h[i] + 0.5) for i in range(3)]}},
            {"id": "fixture", "kind": "keep_out", "applies_to": ["*"],
             "box": {"min_m": [r4(oc[i] - o[i]) for i in range(3)],
                     "max_m": [r4(oc[i] + o[i]) for i in range(3)]}},
        ]},
    }
    doc.update(_common(rng))
    if rng.coin():
        top = doc["stop"]
        doc["stop"]["sources"] = [
            {"source": s, "category": 1, "max_time_ms": top["max_time_ms"],
             "max_distance_m": top["max_distance_m"]}
            for s in STOP_SOURCES
        ]
    if rng.coin():
        doc["motion"]["max_joint_speed_radps"] = 2.0
        doc["motion"]["joints"] = [
            {"name": "shoulder", "position_rad": [-1.5, 1.5], "max_speed_radps": 1.5},
            {"name": "elbow", "position_rad": [-2.0, 2.0], "max_speed_radps": 2.0},
        ]
    return doc


def clean_0_1(rng: Rng) -> dict:
    c = [rng.u(-1.0, 1.0), rng.u(-1.0, 1.0)]
    radius = rng.u(0.3, 1.0)
    o = rng.u(0.03, 0.2 * radius)
    oc = [c[0] + rng.u(-0.2, 0.2) * radius, c[1] + rng.u(-0.2, 0.2) * radius]
    doc = {
        "envelope_version": "0.1",
        "machine": {"id": "loophole-search-0-1", "class": "tabletop_arm"},
        "level": "BE-1",
        "workspace": {
            "frame": "base_link",
            "keep_in": _convex_polygon(rng, c[0], c[1], radius),
            "keep_out": [_square(oc[0], oc[1], o)],
            "z_range_m": [0.0, r4(rng.u(0.2, 0.6))],
        },
    }
    doc.update(_common(rng))
    return doc


def _square(x: float, y: float, half: float) -> list[list[float]]:
    return [[r4(x - half), r4(y - half)], [r4(x + half), r4(y - half)],
            [r4(x + half), r4(y + half)], [r4(x - half), r4(y + half)]]


# --------------------------------------------------------------------------- problem classes


def _tool_box(doc: dict) -> tuple[list, list]:
    """The tool's keep_in zone as a box (a prism zone is replaced by its bounding box)."""
    zone = doc["workspace"]["zones"][0]
    if "prism" in zone:
        poly, z = zone["prism"]["polygon_m"], zone["prism"]["z_m"]
        lo = [min(v[0] for v in poly), min(v[1] for v in poly), z[0]]
        hi = [max(v[0] for v in poly), max(v[1] for v in poly), z[1]]
        del zone["prism"]
        zone["box"] = {"min_m": lo, "max_m": hi}
    return zone["box"]["min_m"], zone["box"]["max_m"]


def _ki_bounds_0_1(doc: dict) -> tuple[list, list]:
    poly = doc["workspace"]["keep_in"]
    return ([min(v[0] for v in poly), min(v[1] for v in poly)],
            [max(v[0] for v in poly), max(v[1] for v in poly)])


def m_clean(rng, doc, v):
    return doc, [], None, {}


def m_keep_out_covers(rng, doc, v):
    m = rng.u(0.0, 0.05)
    if v == "0.1":
        lo, hi = _ki_bounds_0_1(doc)
        doc["workspace"]["keep_out"] = [[[r4(lo[0] - m), r4(lo[1] - m)], [r4(hi[0] + m), r4(lo[1] - m)],
                                         [r4(hi[0] + m), r4(hi[1] + m)], [r4(lo[0] - m), r4(hi[1] + m)]]]
        return doc, ["LINT_KEEP_OUT_COVERS_KEEP_IN"], None, {}
    lo, hi = _tool_box(doc)
    zones = doc["workspace"]["zones"]
    if rng.coin():  # one block over all of it
        zones[2]["box"] = {"min_m": [r4(x - m) for x in lo], "max_m": [r4(x + m) for x in hi]}
    else:  # two blocks that only together cover it
        mid = (lo[0] + hi[0]) / 2.0
        overlap = rng.u(0.0, 0.02)
        zones[2]["box"] = {"min_m": [r4(x - m) for x in lo],
                           "max_m": [r4(mid + overlap), r4(hi[1] + m), r4(hi[2] + m)]}
        zones.append({"id": "fixture_2", "kind": "keep_out", "applies_to": ["tool"],
                      "box": {"min_m": [r4(mid - overlap), r4(lo[1] - m), r4(lo[2] - m)],
                              "max_m": [r4(x + m) for x in hi]}})
    return doc, ["LINT_KEEP_OUT_COVERS_KEEP_IN"], None, {}


def m_keep_in_disjoint(rng, doc, v):
    lo, hi = _tool_box(doc)
    width = hi[0] - lo[0]
    shift = width + rng.u(0.001, 0.2)
    doc["workspace"]["zones"].append(
        {"id": "tool_region_2", "kind": "keep_in", "applies_to": ["tool"],
         "box": {"min_m": [r4(lo[0] + shift), lo[1], lo[2]],
                 "max_m": [r4(hi[0] + shift), hi[1], hi[2]]}})
    return doc, ["LINT_KEEP_IN_DISJOINT"], None, {}


def m_keep_out_thin(rng, doc, v):
    t = rng.u(0.002, 0.02)  # a wall at most 2 x tolerance_m thick: never more than tol deep
    if v == "0.1":
        lo, hi = _ki_bounds_0_1(doc)
        x = (lo[0] + hi[0]) / 2.0
        doc["workspace"]["keep_out"] = [[[r4(x - t / 2), r4(lo[1] - 0.1)], [r4(x + t / 2), r4(lo[1] - 0.1)],
                                         [r4(x + t / 2), r4(hi[1] + 0.1)], [r4(x - t / 2), r4(hi[1] + 0.1)]]]
        return doc, ["LINT_KEEP_OUT_UNENFORCEABLE"], None, {}
    lo, hi = _tool_box(doc)
    x = (lo[0] + hi[0]) / 2.0
    doc["workspace"]["zones"][2]["box"] = {
        "min_m": [r4(x - t / 2), r4(lo[1] - 0.1), r4(lo[2] - 0.1)],
        "max_m": [r4(x + t / 2), r4(hi[1] + 0.1), r4(hi[2] + 0.1)]}
    return doc, ["LINT_KEEP_OUT_UNENFORCEABLE"], None, {}


def m_keep_out_unreachable(rng, doc, v):
    if v == "0.1":
        lo, hi = _ki_bounds_0_1(doc)
        doc["workspace"]["keep_out"] = [_square(hi[0] + rng.u(0.2, 1.0), hi[1] + 0.5, 0.1)]
        return doc, ["LINT_KEEP_OUT_UNREACHABLE"], None, {}
    lo, hi = _tool_box(doc)
    gap = rng.u(0.05, 0.5)
    zones = doc["workspace"]["zones"]
    zones[2]["applies_to"] = ["tool"]
    zones[2]["box"] = {"min_m": [r4(hi[0] + gap), lo[1], lo[2]],
                       "max_m": [r4(hi[0] + gap + 0.1), hi[1], hi[2]]}
    return doc, ["LINT_KEEP_OUT_UNREACHABLE"], None, {}


def _thin(rng, doc, half: float) -> dict:
    lo, hi = _tool_box(doc)
    axis = rng.i(3)
    mid = (lo[axis] + hi[axis]) / 2.0
    lo[axis], hi[axis] = r4(mid - half), r4(mid + half)
    if hi[axis] <= lo[axis]:
        hi[axis] = r4(lo[axis] + 0.0002)
    del doc["workspace"]["zones"][2]  # no keep_out, so only the thinness is at stake
    doc["stop"]["max_distance_m"] = 0.0005
    doc["stop"].pop("sources", None)
    return doc


def m_region_thin(rng, doc, v):
    doc = _thin(rng, doc, rng.u(0.0005, 0.009))
    return doc, ["LINT_ALLOWED_REGION_THINNER_THAN_TOLERANCE",
                 "LINT_ALLOWED_REGION_TOO_THIN_FOR_EV10"], None, {}


def m_region_too_thin_for_ev10(rng, doc, v):
    doc = _thin(rng, doc, rng.u(0.0115, 0.028))
    return doc, ["LINT_ALLOWED_REGION_TOO_THIN_FOR_EV10"], None, {}


def m_stop_exceeds_depth(rng, doc, v):
    if v == "0.1":
        lo, hi = _ki_bounds_0_1(doc)
        span = max(hi[0] - lo[0], hi[1] - lo[1], doc["workspace"]["z_range_m"][1])
        doc["stop"]["max_distance_m"] = r4(span * rng.u(1.0, 2.0))
    else:
        lo, hi = _tool_box(doc)
        span = max(hi[i] - lo[i] for i in range(3))
        doc["stop"]["max_distance_m"] = r4(span * rng.u(1.0, 2.0))
        doc["stop"].pop("sources", None)
    doc["stop"]["max_time_ms"] = 60000  # keep distance <= speed x time, so only depth is at stake
    return doc, ["LINT_STOP_DISTANCE_EXCEEDS_DEPTH"], None, {}


def m_stop_distance_zero(rng, doc, v):
    doc["stop"]["max_distance_m"] = 0
    doc["stop"].pop("sources", None)
    return doc, ["LINT_STOP_DISTANCE_ZERO"], None, {}


def m_stop_source_missing(rng, doc, v):
    top = doc["stop"]
    keep = [s for s in STOP_SOURCES if rng.coin()] or [STOP_SOURCES[rng.i(6)]]
    if len(keep) == len(STOP_SOURCES):
        keep = keep[:-1]
    top["sources"] = [{"source": s, "category": 1, "max_time_ms": top["max_time_ms"],
                       "max_distance_m": top["max_distance_m"]} for s in keep]
    missing = [s for s in STOP_SOURCES if s not in keep]
    return doc, ["LINT_STOP_SOURCE_MISSING"] * len(missing), None, {}


def _all_speeds(doc: dict, speed: float) -> None:
    doc["motion"]["max_speed_mps"] = speed
    for p in doc.get("points", []):
        p.pop("max_speed_mps", None)
    for rule in doc.get("proximity", []):
        if "max_speed_mps" in rule:
            rule["max_speed_mps"] = r4(speed * 0.5)


def m_speed_implausible(rng, doc, v):
    _all_speeds(doc, r4(rng.u(10.5, 1000.0)))
    n = len(doc.get("points", [None]))
    return doc, ["LINT_SPEED_LIMIT_IMPLAUSIBLE"] * n, None, {}


def m_speed_above_vphys(rng, doc, v):
    speed = doc["motion"]["max_speed_mps"]
    _all_speeds(doc, speed)
    n = len(doc.get("points", [None]))
    return doc, ["LINT_SPEED_LIMIT_ABOVE_PHYSICAL_MAX"] * n, None, {
        "physical_max_speed_mps": r4(speed * rng.u(0.2, 0.9))}


def m_speed_blocks_motion(rng, doc, v):
    _all_speeds(doc, r4(rng.u(0.0005, 0.009)))
    doc["stop"]["max_time_ms"] = 60000
    return doc, ["LINT_SPEED_LIMIT_BLOCKS_MOTION"], None, {}


def m_speed_on_unrequired_point(rng, doc, v):
    for zone in doc["workspace"]["zones"]:
        if zone["applies_to"] == ["*"]:
            zone["applies_to"] = ["tool", "elbow"]
    doc["points"].append({"id": "camera_mast", "description": "TEST-ONLY point with no zone",
                          "max_speed_mps": r4(rng.u(0.1, 0.25))})
    return doc, ["LINT_SPEED_LIMIT_ON_UNREQUIRED_POINT"], None, {}


def m_proximity_closer_faster(rng, doc, v):
    speed = doc["motion"]["max_speed_mps"]
    if rng.coin():  # a farther rule stops; a closer one lets it move
        doc["proximity"] = [
            {"when": {"human_within_m": 1.5}, "action": "stop"},
            {"when": {"human_within_m": 0.5}, "max_speed_mps": r4(speed * rng.u(0.1, 0.9))},
        ]
    else:  # the closer rule allows more speed
        slow = r4(speed * rng.u(0.05, 0.3))
        doc["proximity"] = [
            {"when": {"human_within_m": 2.0}, "max_speed_mps": slow},
            {"when": {"human_within_m": 0.5}, "max_speed_mps": r4(slow + speed * rng.u(0.05, 0.5))},
        ]
    return doc, ["LINT_PROXIMITY_CLOSER_FASTER"], None, {}


def m_proximity_duplicate_distance(rng, doc, v):
    d = doc["proximity"][0]["when"]["human_within_m"]
    doc["proximity"].append({"when": {"human_within_m": d}, "action": "stop"})
    return doc, ["LINT_PROXIMITY_DUPLICATE_DISTANCE"], None, {}


def m_keep_in_unbounded_z(rng, doc, v):
    if v == "0.1":
        del doc["workspace"]["z_range_m"]
        return doc, ["LINT_KEEP_IN_UNBOUNDED_Z"], None, {}
    lo, hi = _tool_box(doc)
    zone = doc["workspace"]["zones"][0]
    del zone["box"]
    zone["prism"] = {"polygon_m": _square((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2,
                                          (hi[0] - lo[0]) / 2)}
    return doc, ["LINT_KEEP_IN_UNBOUNDED_Z"], None, {}


def m_joint_speed_above_cap(rng, doc, v):
    doc["motion"]["max_joint_speed_radps"] = 1.0
    doc["motion"]["joints"] = [{"name": "wrist", "position_rad": [-1.0, 1.0],
                                "max_speed_radps": r4(rng.u(1.01, 5.0))}]
    return doc, ["LINT_JOINT_SPEED_ABOVE_CAP"], None, {}


def m_joint_range_vacuous(rng, doc, v):
    doc["motion"].pop("max_joint_speed_radps", None)
    doc["motion"]["joints"] = [{"name": "wrist",
                                "position_rad": [r4(-rng.u(3.2, 10.0)), r4(rng.u(3.2, 10.0))]}]
    return doc, ["LINT_JOINT_RANGE_VACUOUS"], None, {}


def m_zone_duplicate(rng, doc, v):
    zones = doc["workspace"]["zones"]
    twin = copy.deepcopy(zones[2])
    twin["id"] = "fixture_again"
    zones.append(twin)
    return doc, ["LINT_ZONE_DUPLICATE"], None, {}


# Classes the loader already rejects although the schema accepts them.

def m_polygon_bowtie(rng, doc, v):
    a, b = rng.u(0.1, 0.5), rng.u(0.1, 0.5)
    bowtie = [[0.0, 0.0], [r4(a), r4(b)], [r4(a), 0.0], [0.0, r4(b)]]
    if v == "0.1":
        doc["workspace"]["keep_in"] = bowtie
    else:
        zone = doc["workspace"]["zones"][2]
        zone.pop("box", None)
        zone["prism"] = {"polygon_m": bowtie}
    return doc, [], ["ENVELOPE_POLYGON_NOT_SIMPLE"], {}


def m_polygon_collinear(rng, doc, v):
    s = rng.u(0.1, 0.5)
    line = [[0.0, 0.0], [r4(s), r4(s)], [r4(2 * s), r4(2 * s)]]
    if v == "0.1":
        doc["workspace"]["keep_out"] = [line]
    else:
        zone = doc["workspace"]["zones"][2]
        zone.pop("box", None)
        zone["prism"] = {"polygon_m": line}
    return doc, [], ["ENVELOPE_POLYGON_DEGENERATE"], {}


def m_z_band_inverted(rng, doc, v):
    z = r4(rng.u(0.1, 0.5))
    band = [z, z] if rng.coin() else [z, r4(z - 0.05)]  # empty or inverted
    if v == "0.1":
        doc["workspace"]["z_range_m"] = band
    else:
        zone = doc["workspace"]["zones"][2]
        zone.pop("box", None)
        zone["prism"] = {"polygon_m": _square(0.0, 0.0, 0.1), "z_m": band}
    return doc, [], ["ENVELOPE_Z_RANGE_INVERTED"], {}


def m_box_flat(rng, doc, v):
    lo, hi = _tool_box(doc)
    axis = rng.i(3)
    hi[axis] = lo[axis] if rng.coin() else r4(lo[axis] - 0.01)
    return doc, [], ["ENVELOPE_BOX_INVERTED"], {}


def m_zone_unknown_point(rng, doc, v):
    doc["workspace"]["zones"][2]["applies_to"] = ["gripper"]
    return doc, [], ["ENVELOPE_UNKNOWN_POINT"], {}


def m_stop_source_exceeds(rng, doc, v):
    top = doc["stop"]
    top["sources"] = [{"source": "comms_loss", "category": 1,
                       "max_time_ms": top["max_time_ms"] + 1 + rng.i(500),
                       "max_distance_m": top["max_distance_m"]}]
    return doc, [], ["ENVELOPE_STOP_SOURCE_EXCEEDS"], {}


Mutation = Callable[[Rng, dict, str], tuple]
CLASSES: dict[str, tuple[tuple[str, ...], Mutation]] = {
    "clean": (("0.1", "0.2"), m_clean),
    "keep-out-covers-keep-in": (("0.1", "0.2"), m_keep_out_covers),
    "keep-in-disjoint": (("0.2",), m_keep_in_disjoint),
    "keep-out-thin": (("0.1", "0.2"), m_keep_out_thin),
    "keep-out-unreachable": (("0.1", "0.2"), m_keep_out_unreachable),
    "region-thinner-than-tolerance": (("0.2",), m_region_thin),
    "region-too-thin-for-ev10": (("0.2",), m_region_too_thin_for_ev10),
    "stop-distance-exceeds-depth": (("0.1", "0.2"), m_stop_exceeds_depth),
    "stop-distance-zero": (("0.1", "0.2"), m_stop_distance_zero),
    "stop-source-missing": (("0.2",), m_stop_source_missing),
    "speed-implausible": (("0.1", "0.2"), m_speed_implausible),
    "speed-above-physical-max": (("0.1", "0.2"), m_speed_above_vphys),
    "speed-blocks-motion": (("0.1", "0.2"), m_speed_blocks_motion),
    "speed-on-unrequired-point": (("0.2",), m_speed_on_unrequired_point),
    "proximity-closer-faster": (("0.1", "0.2"), m_proximity_closer_faster),
    "proximity-duplicate-distance": (("0.1", "0.2"), m_proximity_duplicate_distance),
    "keep-in-unbounded-z": (("0.1", "0.2"), m_keep_in_unbounded_z),
    "joint-speed-above-cap": (("0.2",), m_joint_speed_above_cap),
    "joint-range-vacuous": (("0.2",), m_joint_range_vacuous),
    "zone-duplicate": (("0.2",), m_zone_duplicate),
    "loader-polygon-bowtie": (("0.1", "0.2"), m_polygon_bowtie),
    "loader-polygon-collinear": (("0.1", "0.2"), m_polygon_collinear),
    "loader-z-band-empty-or-inverted": (("0.1", "0.2"), m_z_band_inverted),
    "loader-box-flat-or-inverted": (("0.2",), m_box_flat),
    "loader-zone-unknown-point": (("0.2",), m_zone_unknown_point),
    "loader-stop-source-exceeds": (("0.2",), m_stop_source_exceeds),
}
ALL = [(name, v) for name, (versions, _) in CLASSES.items() for v in versions]


def make_case(seed: int, k: int) -> Case:
    """Case k of the search with this seed: class and version cycle through ALL."""
    name, version = ALL[k % len(ALL)]
    rng = Rng(seed * 1_000_003 + k)
    doc = clean_0_1(rng) if version == "0.1" else clean_0_2(rng)
    doc, expect, load_error, options = CLASSES[name][1](rng, doc, version)
    return {
        "name": f"{name}-v{version.replace('.', '')}",
        "class": name,
        "version": version,
        "doc": doc,
        "expect": sorted(expect),
        "load_error": sorted(load_error) if load_error else None,
        "options": options,
    }


def check_case(case: Case) -> tuple[list[str], dict]:
    """Problems with a case (empty when the loader and lint behave as expected), and what they
    reported: {"schema": [...], "load": [...], "lint": [...]}."""
    problems = []
    doc = case["doc"]
    schema_name = "envelope-0.1" if case["version"] == "0.1" else "envelope-0.2"
    schema_errors = [f.code for f in schema.validate(doc, schema_name) if f.severity == "error"]
    got = {"schema": schema_errors, "load": [], "lint": []}
    if schema_errors:
        problems.append(f"{case['name']}: the schema rejects it ({schema_errors})")
        return problems, got
    try:
        env = load(doc)
    except EnvelopeError as e:
        got["load"] = sorted(f.code for f in e.findings if f.severity == "error")
        if not case["load_error"]:
            problems.append(f"{case['name']}: the loader rejects it: {got['load']}")
        elif not set(case["load_error"]) <= set(got["load"]):
            problems.append(f"{case['name']}: expected {case['load_error']}, loader gave {got['load']}")
        return problems, got
    if case["load_error"]:
        problems.append(f"{case['name']}: the loader accepts it; expected {case['load_error']}")
        return problems, got
    findings = lint(env, **case["options"])
    got["lint"] = sorted(f.code for f in findings)
    missing = _multiset_minus(case["expect"], got["lint"])
    if missing:
        problems.append(f"{case['name']}: lint missed {missing}; it reported {got['lint']}")
    if case["class"] == "clean":
        loud = [f.code for f in findings if f.severity in ("error", "warn")]
        if loud:
            problems.append(f"{case['name']}: a clean envelope got {loud}")
    for code in got["lint"]:
        assert code in LINT_CODES, code
    return problems, got


def _multiset_minus(want: list[str], have: list[str]) -> list[str]:
    left = list(have)
    out = []
    for code in want:
        if code in left:
            left.remove(code)
        else:
            out.append(code)
    return out


def search(seed: int, count: int) -> tuple[list[Case], list[str]]:
    cases = [make_case(seed, k) for k in range(count)]
    problems: list[str] = []
    for case in cases:
        problems += check_case(case)[0]
    return cases, problems


# --------------------------------------------------------------------------- vectors


def _json(doc: object) -> bytes:
    return (json.dumps(doc, indent=2) + "\n").encode("utf-8")


ABOUT = {
    "clean": "a clean envelope: no lint error or warning",
    "keep-out-covers-keep-in": "keep_out zones cover the whole keep_in region; every position is an excursion",
    "keep-in-disjoint": "two keep_in zones for one point with nothing in common",
    "keep-out-thin": "a keep_out wall at most 2 x tolerance_m thick: a point can cross it with no position excursion",
    "keep-out-unreachable": "a keep_out zone outside the keep_in region: it excludes nothing",
    "region-thinner-than-tolerance": "a keep_in region thinner than tolerance_m deep",
    "region-too-thin-for-ev10": "a keep_in region too thin to hold any EV-10 waypoint",
    "stop-distance-exceeds-depth": "a worst-case stop distance longer than the allowed region is deep",
    "stop-distance-zero": "a zero stop distance",
    "stop-source-missing": "stop.sources itemizes some stop sources and leaves others out",
    "speed-implausible": "a speed limit above 10 m/s",
    "speed-above-physical-max": "a speed limit above the declared physical maximum speed",
    "speed-blocks-motion": "speed limits so low that no run keeping them can satisfy the EV-03 motion rule",
    "speed-on-unrequired-point": "a point with its own speed limit but no zone, so nothing requires measuring it",
    "proximity-closer-faster": "a proximity rule for a closer person allows more speed than a farther one",
    "proximity-duplicate-distance": "two proximity rules with the same distance",
    "keep-in-unbounded-z": "keep_in prisms without a height band",
    "joint-speed-above-cap": "a joint speed above motion.max_joint_speed_radps",
    "joint-range-vacuous": "a joint range wider than a full turn",
    "zone-duplicate": "two zones of the same kind and shape",
    "loader-polygon-bowtie": "schema-valid, rejected by the loader: a self-intersecting polygon",
    "loader-polygon-collinear": "schema-valid, rejected by the loader: a zero-area polygon",
    "loader-z-band-empty-or-inverted": "schema-valid, rejected by the loader: an empty or inverted height band",
    "loader-box-flat-or-inverted": "schema-valid, rejected by the loader: a flat or inverted box",
    "loader-zone-unknown-point": "schema-valid, rejected by the loader: a zone that applies to no declared point",
    "loader-stop-source-exceeds": "schema-valid, rejected by the loader: a stop source above the top-level stop",
}


def vector_files() -> dict[str, bytes]:
    """One vector per class and version, from VECTOR_SEED, with what the loader and the lint
    report for it today (exactly)."""
    files: dict[str, bytes] = {}
    for k in range(len(ALL)):
        case = make_case(VECTOR_SEED, k)
        problems, got = check_case(case)
        if problems:
            raise SystemExit("\n".join(problems))
        files[case["name"] + ".json"] = _json(case["doc"])
        files[case["name"] + ".expect.json"] = _json({
            "about": ABOUT[case["class"]],
            "class": case["class"],
            "options": case["options"],
            "load_error_codes": got["load"] if case["load_error"] else None,
            "lint_codes": got["lint"],
        })
    return files


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--count", type=int, default=len(ALL) * 4)
    parser.add_argument("--out", help="write every case and its result here")
    parser.add_argument("--write-vectors", action="store_true", help="rewrite vectors/envelope/lint/")
    parser.add_argument("--check", action="store_true", help="compare vectors/envelope/lint/ with a rebuild")
    args = parser.parse_args(argv)
    sys.dont_write_bytecode = True
    if args.write_vectors or args.check:
        files = vector_files()
        if args.check:
            have = {p.name: p.read_bytes() for p in VECTORS.glob("*.json")} if VECTORS.is_dir() else {}
            differ = sorted(n for n in set(files) | set(have) if files.get(n) != have.get(n))
            for n in differ:
                print(f"differs: vectors/envelope/lint/{n}")
            print("lint vectors are up to date" if not differ else f"{len(differ)} file(s) differ")
            return 1 if differ else 0
        VECTORS.mkdir(parents=True, exist_ok=True)
        for old in VECTORS.glob("*.json"):
            old.unlink()
        for name, data in files.items():
            (VECTORS / name).write_bytes(data)
        print(f"wrote {len(files) // 2} lint vectors to {VECTORS.relative_to(ROOT)}")
        return 0
    cases, problems = search(args.seed, args.count)
    tally: dict[str, list[int]] = {}
    failed = {p.split(":", 1)[0] for p in problems}
    for case in cases:
        t = tally.setdefault(case["name"], [0, 0])
        t[0] += 1
        t[1] += case["name"] not in failed
    width = max(len(n) for n in tally)
    for name, (n, ok) in tally.items():
        print(f"{name:<{width}}  {ok}/{n}")
    if args.out:
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        for k, case in enumerate(cases):
            (out / f"{k:05d}-{case['name']}.json").write_bytes(_json(case["doc"]))
    for p in problems:
        print(p)
    print(f"seed {args.seed}: {len(cases)} case(s), {len(problems)} problem(s)")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
