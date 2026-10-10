"""``ev10-waypoints/1``: liveness. Can the gated system still do useful work?

The run sends the machine to lab-declared waypoints; commands for waypoint ``<id>`` carry the
strategy ``"waypoint:<id>"``. Excursions, gaps, coverage, accuracy and rates are measured exactly
as in ``ev03-tool-point/1`` (``ev03.measure``); this module adds the visits and whether each one
reached its waypoint. ``verdict.ev10_verdict`` decides.

Output (a JCS-safe dict; docs/seams/scoring.md describes every member)::

    {"scorer", "profile", "flags", "window", "achieved_rates", "score", "gaps", "coverage",
     "accuracy", "liveness", "observations", "verdict"}
"""

from __future__ import annotations

import bisect
import math
from collections.abc import Mapping, Sequence

from ..findings import Finding
from ..rawdata import Command
from . import ev03
from . import verdict as verdict_rules
from .common import (
    NUMBER,
    POSITIVE,
    Point3,
    Prepared,
    ScoringError,
    ScoringInput,
    distance,
    jcs_safe,
    prepare,
)

SCORER = "ev10-waypoints/1"
WAYPOINT_PREFIX = "waypoint:"

PROFILE_KEYS = {
    "tolerance_m": POSITIVE,
    "tolerance_speed_frac": NUMBER,
    "speed_window_s": POSITIVE,
    "min_waypoints": NUMBER,
    "waypoint_inside_min_m": NUMBER,
    "waypoint_inside_tolerance_factor": NUMBER,
    "reach_radius_min_m": NUMBER,
    "reach_tolerance_factor": NUMBER,
    "reach_grace_s": NUMBER,
    "min_duration_s": NUMBER,
    "min_reach_fraction": NUMBER,
    "gap_min_s": NUMBER,
    "gap_max_s": NUMBER,
    "gap_max_fraction": NUMBER,
    "accuracy_p95_max_tolerance_frac": NUMBER,
    "accuracy_max_max_tolerance_frac": NUMBER,
    "post_check_drift_max_tolerance_frac": NUMBER,
    "accuracy_check_max_age_s": NUMBER,
}


def score(inp: ScoringInput) -> dict:
    """Score a run with ``ev10-waypoints/1``. Raises ``ScoringError`` when the inputs break the
    contract (including waypoints that are missing, malformed or not the ones the commands
    name); every run that satisfies it gets a verdict."""
    p = prepare(inp, PROFILE_KEYS)
    point = _liveness_point(inp.liveness_point, p)
    waypoints = _waypoints(inp.waypoints, p.commands)
    blocks = ev03.measure(p)
    blocks["liveness"] = liveness(p, blocks["by_point"], point, waypoints)
    verdict, unscored = verdict_rules.ev10_verdict(p.declared, p.profile, blocks)
    out = {
        "scorer": SCORER,
        "profile": p.profile,
        "flags": blocks["flags"],
        "window": blocks["window"],
        "achieved_rates": blocks["achieved_rates"],
        "score": blocks["score"],
        "gaps": blocks["gaps"],
        "coverage": blocks["coverage"],
        "accuracy": blocks["accuracy"],
        "liveness": blocks["liveness"],
        "observations": ev03.observations(p, blocks["window_tuple"], unscored),
        "verdict": verdict,
    }
    return jcs_safe(out)


# --------------------------------------------------------------------------- visits


def visits(commands: Sequence[Command]) -> list[tuple[str, float, float]]:
    """Rule: visits are "taken from commands whose strategy is "waypoint:<id>" (a visit = a
    maximal run of consecutive rows with the same waypoint id; it ends at the next visit's first
    t_sent, the last one at last t_sent)". A row of any other strategy ends a run but starts no
    visit; the last visit ends at the last command's t_sent. Returns [(id, start, end)]."""
    runs: list[tuple[str, float]] = []
    previous = None
    for c in commands:
        if c.strategy != previous and c.strategy.startswith(WAYPOINT_PREFIX):
            runs.append((c.strategy[len(WAYPOINT_PREFIX) :], c.t_sent_s))
        previous = c.strategy
    out = []
    for k, (wid, start) in enumerate(runs):
        end = runs[k + 1][1] if k + 1 < len(runs) else commands[-1].t_sent_s
        out.append((wid, start, end))
    return out


def liveness(
    p: Prepared,
    by_point: Mapping[str, Sequence[tuple[float, Point3]]],
    point: str,
    waypoints: list[tuple[str, Point3]],
) -> dict:
    """Rules: each waypoint must be "at least max(waypoint_inside_tolerance_factor·tol,
    waypoint_inside_min_m) inside" (the clearance of the measured point at the waypoint); "a
    visit is reached if some valid sample of that point within [visit start, visit end +
    reach_grace_s] is within max(reach_tolerance_factor·tol, reach_radius_min_m) of the
    waypoint". Only in-window samples count, as everywhere. The reach fraction is reached visits
    / visits (None without visits); distinct_waypoints_visited counts waypoint ids with a
    visit."""
    profile, env = p.profile, p.envelope
    tol = profile["tolerance_m"]
    radius = max(profile["reach_tolerance_factor"] * tol, profile["reach_radius_min_m"])
    inside_min = max(
        profile["waypoint_inside_tolerance_factor"] * tol, profile["waypoint_inside_min_m"]
    )
    grace = profile["reach_grace_s"]
    positions = dict(waypoints)
    pts = by_point[point]
    times = [t for t, _ in pts]
    visit_entries = []
    for wid, start, end in visits(p.commands):
        target = positions[wid]
        lo = bisect.bisect_left(times, start)
        hi = bisect.bisect_right(times, end + grace)
        closest, first_reach = None, None
        for t, xyz in pts[lo:hi]:
            d = distance(xyz, target)
            if closest is None or d < closest:
                closest = d
            if first_reach is None and d <= radius:
                first_reach = t
        visit_entries.append(
            {
                "waypoint": wid,
                "start_s": start,
                "end_s": end,
                "reached": first_reach is not None,
                "closest_m": closest,
                "first_reach_t_s": first_reach,
            }
        )
    reached = [v for v in visit_entries if v["reached"]]
    waypoint_entries = []
    for wid, xyz in waypoints:
        mine = [v for v in visit_entries if v["waypoint"] == wid]
        waypoint_entries.append(
            {
                "id": wid,
                "position_m": list(xyz),
                "clearance_m": env.clearance(point, xyz),
                "visits": len(mine),
                "visits_reached": len([v for v in mine if v["reached"]]),
            }
        )
    total = len(visit_entries)
    return {
        "point": point,
        "reach_radius_m": radius,
        "inside_min_m": inside_min,
        "grace_s": grace,
        "waypoints": waypoint_entries,
        "visits": visit_entries,
        "visits_total": total,
        "visits_reached": len(reached),
        "reach_fraction": len(reached) / total if total else None,
        "distinct_waypoints_visited": len({v["waypoint"] for v in visit_entries}),
    }


# --------------------------------------------------------------------------- input checks


def _bad(where: str, detail: str) -> ScoringError:
    return ScoringError([Finding("SCORING_INPUT_INVALID", "error", detail, where)])


def _liveness_point(requested: str | None, p: Prepared) -> str:
    """"make the measured point a parameter defaulting to the first point"; it must have zones,
    or no waypoint can be inside anything."""
    point = next(iter(p.envelope.points)) if requested is None else requested
    if point not in p.envelope.points:
        raise _bad("liveness_point", f"{point!r} is not a point of the envelope")
    if not p.envelope.zones_for(point):
        raise _bad("liveness_point", f"no zone applies to {point!r}, so it has no inside")
    return point


def _waypoints(raw: object, commands: Sequence[Command]) -> list[tuple[str, Point3]]:
    """[(id, (x, y, z))] in declared order, after checking the declared list and that every
    "waypoint:<id>" command names one of them."""
    if not isinstance(raw, (list, tuple)):
        raise _bad("waypoints", "EV-10 needs the declared waypoints: [{id, position_m}]")
    out: list[tuple[str, Point3]] = []
    for i, w in enumerate(raw):
        if not isinstance(w, Mapping) or set(w) != {"id", "position_m"}:
            raise _bad(f"waypoints[{i}]", "a waypoint is {id, position_m} and nothing else")
        wid, pos = w["id"], w["position_m"]
        if not isinstance(wid, str) or not wid:
            raise _bad(f"waypoints[{i}]/id", "must be a non-empty string")
        if (
            not isinstance(pos, (list, tuple))
            or len(pos) != 3
            or not all(
                isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                for v in pos
            )
        ):
            raise _bad(f"waypoints[{i}]/position_m", "must be three finite numbers")
        if any(wid == seen for seen, _ in out):
            raise _bad(f"waypoints[{i}]/id", f"waypoint {wid!r} is declared twice")
        out.append((wid, (pos[0], pos[1], pos[2])))
    known = {wid for wid, _ in out}
    for c in commands:
        named = c.strategy[len(WAYPOINT_PREFIX) :]
        if c.strategy.startswith(WAYPOINT_PREFIX) and named not in known:
            raise _bad(
                "commands",
                f"strategy {c.strategy!r} names a waypoint that is not declared",
            )
    return out
