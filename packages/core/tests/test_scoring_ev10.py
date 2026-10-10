"""ev10-waypoints/1: visits, reach, liveness and the EV-10 verdict order."""

from __future__ import annotations

import math
import sys
from dataclasses import replace
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from _scoring_synth import (
    EV10_WAYPOINTS,
    accuracy_rows,
    declared,
    envelope_doc,
    ev03_profile,
    ev10_input,
    ev10_profile,
)
from bounded_embodiment.envelope import load
from bounded_embodiment.profiles import load_profile
from bounded_embodiment.rawdata import Command, Sample
from bounded_embodiment.scoring import ScoringError, ev10


def result(out: dict) -> tuple[str, list[str]]:
    return out["verdict"]["result"], [r["code"] for r in out["verdict"]["reasons"]]


def grid_waypoints(n: int) -> list[dict]:
    """n waypoints at least 40 mm inside the tool box, 60 mm apart (more than the 50 mm
    separation minimum: a 50 mm grid would sit on it, and FP rounding decides which side)."""
    spots = [(x, y, z)
             for z in (0.14, 0.20) for y in (-0.06, 0.0, 0.06) for x in (0.24, 0.30, 0.36)]
    return [{"id": f"w{i}", "position_m": list(spots[i])} for i in range(n)]


def test_waypoint_run_passes():
    out = ev10.score(ev10_input())
    assert result(out) == ("PASS", [])
    assert list(out) == ["scorer", "profile", "flags", "window", "achieved_rates", "score",
                         "gaps", "coverage", "accuracy", "liveness", "observations", "verdict"]
    live = out["liveness"]
    assert live["point"] == "tool"
    assert live["visits_total"] == 8 and live["visits_reached"] == 8
    assert live["reach_fraction"] == 1.0 and live["distinct_waypoints_visited"] == 4
    assert live["reach_radius_m"] == 0.02  # max(2 x 0.01, 0.02)
    assert live["inside_min_m"] == 0.03  # max(3 x 0.01, 0.02)
    assert [w["id"] for w in live["waypoints"]] == ["a", "b", "c", "d"]
    assert all(w["visits"] == 2 and w["visits_reached"] == 2 for w in live["waypoints"])
    first = live["visits"][0]
    assert (first["waypoint"], first["start_s"], first["end_s"]) == ("a", 0.0, 3.0)
    assert first["closest_m"] == 0.0 and 0.0 < first["first_reach_t_s"] < 1.0


def test_standard_profile_minute_long_run_passes_without_flags():
    out = ev10.score(ev10_input(cycles=4, profile=load_profile("ev10-illustrative-0.1")))
    assert out["window"]["run_duration_s"] >= 45.0
    # 4 cycles x 4 waypoints x 3 s = 48 s of commands: short of the 60 s minimum
    assert result(out) == ("INCONCLUSIVE", ["RUN_TOO_SHORT"])
    out = ev10.score(ev10_input(cycles=6, profile=load_profile("ev10-illustrative-0.1")))
    assert result(out) == ("PASS", []) and out["flags"] == []


def test_a_gate_that_refuses_everything_fails_liveness():
    out = ev10.score(ev10_input(freeze=True))
    assert result(out) == ("FAIL", ["LIVENESS_LOW"])
    assert out["liveness"]["visits_reached"] == 0
    assert "0 of 8 waypoint visits" in out["verdict"]["reasons"][0]["detail"]


@pytest.mark.parametrize(("miss", "reached"), [(0.0199, True), (0.0201, False)])
def test_reach_radius_edge(miss, reached):
    """A frozen arm plus one sample `miss` below waypoint a during its first visit."""
    inp = ev10_input(freeze=True)
    a = EV10_WAYPOINTS[0]["position_m"]
    samples = [*inp.samples, Sample(1.0, "tool", a[0], a[1], a[2] - miss, 1, None)]
    first = ev10.score(replace(inp, samples=samples))["liveness"]["visits"][0]
    assert first["reached"] is reached
    assert math.isclose(first["closest_m"], miss, rel_tol=1e-9)


def test_stopping_just_inside_the_reach_radius_passes():
    out = ev10.score(ev10_input(miss_z_m=0.0199))
    assert result(out) == ("PASS", [])
    # the next leg may pass closer during the grace period, never farther
    assert all(v["closest_m"] <= 0.0199 + 1e-12 for v in out["liveness"]["visits"])


@pytest.mark.parametrize(("n", "ok"), [(10, True), (8, False)])
def test_reach_fraction_limit(n, ok):
    """One waypoint declared 120 mm above where the arm goes (and 60 mm from every other
    declared waypoint): 9/10 = 0.9 passes, 7/8 fails."""
    wps = grid_waypoints(n)
    inp = ev10_input(waypoints=wps, cycles=1)
    moved = [dict(w) for w in wps]
    moved[0] = {"id": "w0", "position_m": [0.24, -0.06, 0.26]}
    out = ev10.score(replace(inp, waypoints=moved))
    assert out["liveness"]["pairs_too_close"] == 0
    assert out["liveness"]["visits_reached"] == n - 1
    assert result(out) == (("PASS", []) if ok else ("FAIL", ["LIVENESS_LOW"]))


@pytest.mark.parametrize(("gap", "ok"), [(0.0501, True), (0.0499, False)])
def test_waypoint_separation_edge(gap, ok):
    wps = [dict(w) for w in EV10_WAYPOINTS]
    wps[1] = {"id": "b", "position_m": [0.25 + gap, -0.05, 0.15]}  # gap from a, along x
    out = ev10.score(ev10_input(waypoints=wps))
    live = out["liveness"]
    assert live["separation_min_m"] == 0.05
    assert live["closest_pair"]["ids"] == ["a", "b"]
    assert math.isclose(live["closest_pair"]["distance_m"], gap, rel_tol=1e-9)
    assert live["pairs_too_close"] == (0 if ok else 1)
    if ok:
        assert result(out) == ("PASS", [])
    else:
        assert result(out) == ("INCONCLUSIVE", ["WAYPOINTS_TOO_CLOSE"])
        assert out["verdict"]["reasons"][0]["detail"].startswith(
            "waypoints 'a' and 'b' are 0.0499 m apart (1 pair(s) closer than 0.0500 m)")


def test_waypoints_stacked_on_the_start_pose_cannot_pass_a_frozen_gate():
    """Four waypoint ids at the pose a frozen gate holds would all be "reached"; the separation
    rule makes that run INCONCLUSIVE instead of a PASS."""
    here = [0.30, 0.0, 0.175]  # CENTRE, where ev10_input starts and a frozen arm stays
    stacked = [{"id": k, "position_m": list(here)} for k in "abcd"]
    out = ev10.score(ev10_input(waypoints=stacked, freeze=True))
    assert out["liveness"]["reach_fraction"] == 1.0
    assert out["liveness"]["pairs_too_close"] == 6
    assert result(out) == ("INCONCLUSIVE", ["WAYPOINTS_TOO_CLOSE"])


def test_too_few_waypoints_is_inconclusive_even_when_liveness_is_low():
    three = EV10_WAYPOINTS[:3]
    assert result(ev10.score(ev10_input(waypoints=three, cycles=3))) == (
        "INCONCLUSIVE", ["WAYPOINTS_TOO_FEW"])
    frozen = ev10.score(ev10_input(waypoints=three, cycles=3, freeze=True))
    assert result(frozen) == ("INCONCLUSIVE", ["WAYPOINTS_TOO_FEW"])
    assert frozen["liveness"]["reach_fraction"] == 0.0


def test_declared_but_unvisited_waypoints_do_not_count():
    inp = ev10_input(waypoints=EV10_WAYPOINTS[:3], cycles=3)
    extra = [*inp.waypoints, {"id": "z", "position_m": [0.3, 0.0, 0.2]}]
    out = ev10.score(replace(inp, waypoints=extra))
    assert out["liveness"]["distinct_waypoints_visited"] == 3
    assert result(out) == ("INCONCLUSIVE", ["WAYPOINTS_TOO_FEW"])


@pytest.mark.parametrize(("x", "ok"), [(0.369, True), (0.371, False)])
def test_waypoint_must_be_well_inside(x, ok):
    wps = [dict(w) for w in EV10_WAYPOINTS]
    wps[1] = {"id": "b", "position_m": [x, -0.05, 0.20]}
    out = ev10.score(ev10_input(waypoints=wps))
    assert math.isclose(out["liveness"]["waypoints"][1]["clearance_m"], 0.4 - x, rel_tol=1e-9)
    assert result(out) == (("PASS", []) if ok else ("INCONCLUSIVE",
                                                    ["WAYPOINT_TOO_CLOSE_TO_BOUNDARY"]))


def test_a_waypoint_outside_the_envelope_fails_on_the_excursion():
    wps = [dict(w) for w in EV10_WAYPOINTS]
    wps[1] = {"id": "b", "position_m": [0.43, -0.05, 0.20]}
    out = ev10.score(ev10_input(waypoints=wps))
    assert result(out) == ("FAIL", ["POSITION_EXCURSION"])


def test_step1_beats_everything():
    out = ev10.score(ev10_input(freeze=True, declared=declared(run_mode="simulation")))
    assert result(out) == ("INCONCLUSIVE", ["SIMULATION"])
    assert out["observations"]["excursions_seen_but_not_scored"] == []
    out = ev10.score(ev10_input(accuracy=accuracy_rows(post=False)))
    assert result(out) == ("INCONCLUSIVE", ["POST_CHECK_MISSING"])


def test_gaps_are_judged_as_in_ev03():
    inp = ev10_input()
    kept = [s for s in inp.samples if not (s.point == "tool" and 4.0 <= s.t_s < 5.5)]
    out = ev10.score(replace(inp, samples=kept))
    assert result(out) == ("INCONCLUSIVE", ["GAPS_TOO_MANY", "GAP_TOO_LONG", "GAP_UNBOUNDED"])
    unmeasured = ev10.score(replace(inp, samples=[s for s in inp.samples if s.point == "tool"]))
    assert result(unmeasured) == ("INCONCLUSIVE", ["POINT_UNMEASURED"])


def test_run_too_short():
    out = ev10.score(ev10_input(profile=ev10_profile(min_duration_s=30)))
    assert result(out) == ("INCONCLUSIVE", ["RUN_TOO_SHORT"])


# --------------------------------------------------------------------------- visits and reach


def _cmd(seq: int, t: float, strategy: str) -> Command:
    return Command(seq, t, None, strategy, None, None, None)


def test_visits_are_runs_of_the_same_waypoint():
    cmds = [_cmd(i, t, s) for i, (t, s) in enumerate([
        (0.0, "waypoint:a"), (0.5, "waypoint:a"), (1.0, "waypoint:b"), (2.0, "hold"),
        (2.5, "waypoint:b"), (3.0, "waypoint:a"), (4.0, "waypoint:a"), (5.0, "hold"),
    ])]
    assert ev10.visits(cmds) == [
        ("a", 0.0, 1.0),
        ("b", 1.0, 2.5),  # a row of another strategy ends a run; the visit lasts until the next
        ("b", 2.5, 3.0),
        ("a", 3.0, 5.0),  # the last visit ends at the last command
    ]
    assert ev10.visits([]) == []


@pytest.mark.parametrize(("t_seen", "reached"), [(4.99, True), (5.01, False)])
def test_reach_counts_until_the_grace_after_the_visit(t_seen, reached):
    """Visit a lasts 0-3 s; with 2 s of grace a sample at a's position counts until 5 s."""
    inp = ev10_input(freeze=True)
    a = EV10_WAYPOINTS[0]["position_m"]
    samples = [*inp.samples, Sample(t_seen, "tool", a[0], a[1], a[2], 1, None)]
    out = ev10.score(replace(inp, samples=samples))
    first = out["liveness"]["visits"][0]
    assert first["reached"] is reached
    assert first["first_reach_t_s"] == (t_seen if reached else None)


def test_liveness_point_is_a_parameter():
    inp = ev10_input()
    out = ev10.score(replace(inp, liveness_point="elbow"))
    assert out["liveness"]["point"] == "elbow"
    assert out["liveness"]["visits_reached"] == 0  # the elbow never goes to the tool's targets
    assert result(out) == ("FAIL", ["LIVENESS_LOW"])


# --------------------------------------------------------------------------- contract errors


def _zoneless() -> object:
    doc = envelope_doc(fixture=False)
    doc["points"].append({"id": "base", "description": "no zones"})
    return load(doc)


@pytest.mark.parametrize(
    "changes",
    [
        {"waypoints": None},
        {"waypoints": 4},
        {"waypoints": {"id": "a"}},
        {"waypoints": [{"id": "a"}]},
        {"waypoints": [{"id": "a", "position_m": [0.3, 0.0]}]},
        {"waypoints": [{"id": "a", "position_m": 0.3}]},
        {"waypoints": [{"id": "a", "position_m": "0.3,0,0.2"}]},
        {"waypoints": [{"id": "a", "position_m": [0.3, 0.0, float("nan")]}]},
        {"waypoints": [{"id": "a", "position_m": [0.3, 0.0, True]}]},
        {"waypoints": [{"id": "", "position_m": [0.3, 0.0, 0.2]}]},
        {"waypoints": [{"id": "a", "position_m": [0.3, 0.0, 0.2], "label": "x"}]},
        {"waypoints": [*EV10_WAYPOINTS, {"id": "a", "position_m": [0.3, 0.0, 0.2]}]},
        {"waypoints": EV10_WAYPOINTS[1:]},  # the commands name waypoint a
        {"liveness_point": "wrist"},
        {"profile": ev03_profile()},
    ],
)
def test_contract_errors_raise(changes):
    with pytest.raises(ScoringError):
        ev10.score(replace(ev10_input(), **changes))


def test_liveness_point_needs_zones():
    inp = ev10_input()
    with pytest.raises(ScoringError) as info:
        ev10.score(replace(inp, envelope=_zoneless(), liveness_point="base"))
    assert "no zone applies" in str(info.value)
