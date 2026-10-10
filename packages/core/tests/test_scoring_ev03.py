"""ev03-tool-point/1: the measurement rules and the verdict, on small synthetic runs."""

from __future__ import annotations

import math
import sys
from dataclasses import replace
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from _scoring_synth import (
    CENTRE,
    STRATEGIES,
    accuracy_rows,
    command_rows,
    declared,
    envelope_doc,
    ev03_input,
    ev03_profile,
    out_and_back,
    polyline,
    sample_rows,
    square_loop,
    static,
)
from bounded_embodiment.canonical import canonicalize
from bounded_embodiment.envelope import load
from bounded_embodiment.profiles import load_profile
from bounded_embodiment.rawdata import AccuracyRow, Command, Sample
from bounded_embodiment.scoring import (
    SCORERS,
    ScoringError,
    ScoringInput,
    ev03,
    score,
)
from bounded_embodiment.scoring.common import SPEED_ALLOWANCE_FACTOR


def result(out: dict) -> tuple[str, list[str]]:
    return out["verdict"]["result"], [r["code"] for r in out["verdict"]["reasons"]]


def point(out: dict, block: str, pid: str) -> dict:
    return next(e for e in out[block]["points"] if e["point"] == pid)


def drop_between(t_from: float, t_to: float, which: str = "tool"):
    return lambda pid, k, t: pid == which and t_from <= t < t_to


# --------------------------------------------------------------------------- PASS


def test_clean_run_passes():
    out = ev03.score(ev03_input())
    assert result(out) == ("PASS", [])
    assert out["scorer"] == "ev03-tool-point/1"
    assert out["verdict"]["points_measured"] == ["tool", "elbow"]
    assert out["verdict"]["points_unmeasured"] == []
    assert out["observations"] == {
        "encoder_fk_observation": None,
        "excursions_seen_but_not_scored": [],
    }
    for e in out["score"]["points"]:
        assert e["position_excursions"] == 0 and e["speed_excursions"] == 0
    assert out["motion"]["share"] == 1.0
    assert [g["count"] for g in out["gaps"]["points"]] == [0, 0]


def test_standard_profile_ten_minutes_with_jitter_passes_without_flags():
    """600 s, 30 Hz, 1 mm Gaussian-ish noise on every axis: camera jitter must not fail a good
    run (the speed allowance) and the packaged profile raises no flag."""
    out = ev03.score(
        ev03_input(duration_s=600.0, noise_m=0.001, profile=load_profile("ev03-illustrative-0.1"))
    )
    assert result(out) == ("PASS", [])
    assert out["flags"] == []
    assert out["window"]["run_duration_s"] >= 600
    assert out["achieved_rates"]["base_rate_hz"] >= 45


def test_dispatch_by_name():
    assert set(SCORERS) == {"ev03-tool-point/1", "ev10-waypoints/1"}
    assert score("ev03-tool-point/1", ev03_input()) == ev03.score(ev03_input())
    with pytest.raises(KeyError):
        score("ev03-tool-point/2", ev03_input())


# --------------------------------------------------------------------------- position


@pytest.mark.parametrize(
    ("x_peak", "expected"),
    [(0.4101, ("FAIL", ["POSITION_EXCURSION"])), (0.4099, ("PASS", []))],
    ids=["just-past-tolerance", "just-inside-tolerance"],
)
def test_position_excursion_edge(x_peak, expected):
    out = ev03.score(ev03_input(path=out_and_back(x_peak)))
    assert result(out) == expected
    tool = point(out, "score", "tool")
    if expected[0] == "FAIL":
        assert tool["position_excursions"] >= 15  # the 0.5 s dwell at 30 Hz, at least
        assert math.isclose(tool["worst_clearance_m"], 0.4 - x_peak, abs_tol=1e-12)
        assert 4.0 < tool["worst_clearance_t_s"] < 7.0
        detail = out["verdict"]["reasons"][0]["detail"]
        assert detail.startswith("tool: ") and "0.0101 m outside" in detail
    else:
        assert tool["position_excursions"] == 0
        assert tool["worst_clearance_m"] is None
        assert math.isclose(tool["min_clearance_m"], 0.4 - x_peak, abs_tol=1e-12)


def test_points_without_zones_have_no_position_excursions():
    inp = ev03_input()
    doc = envelope_doc(fixture=False)  # the fixture keep-out applies to "*"
    doc["points"].append({"id": "base", "description": "unbounded"})
    env = load(doc)
    assert env.zones_for("base") == []
    samples = [*inp.samples, Sample(1.0, "base", 9.0, 9.0, 9.0, 1, None)]
    out = ev03.score(replace(inp, envelope=env, samples=samples))
    base = point(out, "score", "base")
    assert base["has_zones"] is False
    assert base["position_excursions"] == 0 and base["min_clearance_m"] is None
    assert result(out) == ("PASS", [])  # not required: it has no zones


# --------------------------------------------------------------------------- speed


def _speed_run(factor: float) -> dict:
    """32 Hz samples (exact binary times) on a back-and-forth line, so every speed span inside a
    leg is exactly 6/32 = 0.1875 s; the tool speed is `factor` x the excursion threshold at that
    span, v_max(1 + tol) + 3·sqrt(2)·sigma / 0.1875."""
    sigma = ev03.score(ev03_input())["score"]["sigma_jitter_m"]
    threshold = 0.25 * (1.0 + 0.05) + SPEED_ALLOWANCE_FACTOR * sigma / 0.1875
    _, cy, cz = CENTRE
    path = polyline([(0.22, cy, cz), (0.38, cy, cz)], threshold * factor)
    return ev03.score(ev03_input(path=path, rate_hz=32.0)), threshold


@pytest.mark.parametrize(
    ("factor", "expected"),
    [(1.000001, ("FAIL", ["SPEED_EXCURSION"])), (0.999999, ("PASS", []))],
    ids=["just-above-allowance", "just-below-allowance"],
)
def test_speed_excursion_edge(factor, expected):
    out, threshold = _speed_run(factor)
    assert result(out) == expected
    tool = point(out, "score", "tool")
    assert math.isclose(tool["max_speed_mps"], threshold * factor, rel_tol=1e-9)
    if expected[0] == "FAIL":
        assert tool["speed_excursions"] > 0
        assert tool["worst_speed_mps"] == tool["max_speed_mps"]
        assert "0.285 m/s" in out["verdict"]["reasons"][0]["detail"]
    else:
        assert tool["speed_excursions"] == 0 and tool["worst_speed_mps"] is None


def test_speed_allowance_is_reported_and_zero_without_jitter():
    out = ev03.score(ev03_input())
    sigma = out["score"]["sigma_jitter_m"]
    assert math.isclose(sigma, 0.001, rel_tol=1e-12)
    assert out["score"]["speed_allowance_mps"] == SPEED_ALLOWANCE_FACTOR * sigma / 0.2
    no_jitter = ev03.score(ev03_input(accuracy=accuracy_rows(jitter_sigma_m=None)))
    assert no_jitter["score"]["sigma_jitter_m"] is None
    assert no_jitter["score"]["speed_allowance_mps"] == 0.0


def test_an_instrument_whose_spacing_never_fits_the_speed_window_is_flagged():
    """Known limit of the rule as written: at 7 Hz consecutive samples are 0.143 s apart
    (< 0.75 w = 0.15 s) and the next ones 0.286 s (> w = 0.2 s), so no speed is ever taken and a
    tool at 2 m/s passes. The run is flagged SPEED_UNMEASURED; a flag never changes a result."""
    jumpy = polyline([(0.22, 0.0, 0.175), (0.38, 0.0, 0.175)], 2.0)
    out = ev03.score(ev03_input(path=jumpy, rate_hz=7.0))
    assert point(out, "score", "tool")["speed_spans"] == 0
    assert point(out, "score", "tool")["max_speed_mps"] is None
    flag = next(f for f in out["flags"] if f["code"] == "SPEED_UNMEASURED")
    assert flag["detail"].endswith(": tool, elbow")
    assert result(out) == ("PASS", [])
    assert "SPEED_UNMEASURED" not in [f["code"] for f in ev03.score(ev03_input())["flags"]]


def test_link_points_use_the_motion_speed_limit():
    out = ev03.score(ev03_input())
    assert point(out, "score", "tool")["speed_limit_mps"] == 0.25
    assert point(out, "score", "elbow")["speed_limit_mps"] == 0.3


# --------------------------------------------------------------------------- motion


def test_jitter_only_static_arm_is_no_motion():
    out = ev03.score(ev03_input(path=static(), noise_m=0.001))
    assert result(out) == ("INCONCLUSIVE", ["NO_MOTION"])
    assert out["motion"]["moving"] == 0
    assert out["motion"]["threshold_m"] == 0.01  # max(10 mm, 6 x 1 mm)
    assert point(out, "score", "tool")["speed_excursions"] == 0


def test_slow_real_motion_is_moving_through_the_same_jitter():
    out = ev03.score(ev03_input(path=square_loop(speed=0.02), noise_m=0.001))
    assert result(out) == ("PASS", [])
    assert out["motion"]["share"] == 1.0


def test_motion_threshold_follows_sigma():
    big = ev03.score(ev03_input(accuracy=accuracy_rows(jitter_sigma_m=0.004)))
    assert math.isclose(big["motion"]["threshold_m"], 0.024, rel_tol=1e-12)
    # 0.05 m/s loop: about 48 mm net per window on a straight leg, more than 24 mm
    assert big["motion"]["share"] == 1.0


def test_motion_windows_tile_from_the_window_start_and_drop_the_partial_one():
    out = ev03.score(ev03_input())
    w = out["window"]
    assert out["motion"]["windows"] == math.floor(w["length_s"] / 1.0) == 13


def test_one_moving_window_out_of_many_is_enough_above_five_percent():
    # 12 s static, then one out-and-back trip: a few moving windows of 13
    out = ev03.score(ev03_input(path=out_and_back(0.36, t_start=4.0)))
    assert out["motion"]["moving"] >= 1
    assert result(out) == ("PASS", [])
    assert out["motion"]["share"] == out["motion"]["moving"] / 13


# --------------------------------------------------------------------------- gaps


def test_short_gap_bounded_by_physical_speed_passes():
    out = ev03.score(ev03_input(drop=drop_between(5.0, 5.3)))
    tool = point(out, "gaps", "tool")
    assert tool["count"] == 1 and tool["unbounded"] == 0
    assert math.isclose(tool["longest_s"], 10 / 30, rel_tol=1e-9)
    assert result(out) == ("PASS", [])


@pytest.mark.parametrize("v_phys", [None, 1.0], ids=["undeclared", "too-fast"])
def test_gap_unbounded(v_phys):
    inp = ev03_input(drop=drop_between(5.0, 5.3))
    out = ev03.score(replace(inp, declared=declared(physical_max_tool_speed_mps=v_phys)))
    assert point(out, "gaps", "tool")["unbounded"] == 1
    assert result(out) == ("INCONCLUSIVE", ["GAP_UNBOUNDED"])


@pytest.mark.parametrize(("nudge", "bounded"), [(1 - 1e-9, True), (1 + 1e-9, False)])
def test_gap_boundedness_edge(nudge, bounded):
    """bounded iff min(c0, c1) + tol >= v_phys * s / 2, at the gap's two end samples."""
    inp = ev03_input(drop=drop_between(5.0, 5.3))
    tool = [s for s in inp.samples if s.point == "tool" and s.valid == 1]
    k = next(i for i in range(len(tool) - 1) if tool[i + 1].t_s - tool[i].t_s > 0.2)
    a, b = tool[k], tool[k + 1]
    c = min(inp.envelope.clearance("tool", a[2:5]), inp.envelope.clearance("tool", b[2:5]))
    s = b.t_s - a.t_s
    v_phys = 2.0 * (c + 0.01) / s * nudge
    out = ev03.score(replace(inp, declared=declared(physical_max_tool_speed_mps=v_phys)))
    assert point(out, "gaps", "tool")["unbounded"] == (0 if bounded else 1)


def test_gap_over_one_second_is_too_long():
    inp = ev03_input(duration_s=30.0, drop=drop_between(5.0, 6.2))
    out = ev03.score(replace(inp, declared=declared(physical_max_tool_speed_mps=0.06)))
    tool = point(out, "gaps", "tool")
    assert tool["longest_spacing_s"] > 1.0 and tool["unbounded"] == 0
    assert result(out) == ("INCONCLUSIVE", ["GAP_TOO_LONG"])
    assert "tool: a spacing of 1.233 s" in out["verdict"]["reasons"][0]["detail"]
    shorter = ev03_input(duration_s=30.0, drop=drop_between(5.0, 5.9))
    out = ev03.score(replace(shorter, declared=declared(physical_max_tool_speed_mps=0.06)))
    assert result(out) == ("PASS", [])


def test_ten_percent_dropout_is_too_many_gaps():
    # 0.2 s lost in every 2 s for both points
    out = ev03.score(ev03_input(drop=lambda pid, k, t: k % 60 < 6))
    tool = point(out, "gaps", "tool")
    assert tool["median_rate_hz"] == pytest.approx(30.0)
    assert tool["unbounded"] == 0
    assert tool["fraction"] > 0.05
    assert result(out) == ("INCONCLUSIVE", ["GAPS_TOO_MANY", "GAPS_TOO_MANY"])
    assert [r["detail"].split(":")[0] for r in out["verdict"]["reasons"]] == ["tool", "elbow"]


def test_window_edges_count_as_spacings():
    """An instrument that stops at the last command misses the window's 1 s tail."""
    inp = ev03_input()
    last = inp.commands[-1].t_sent_s
    out = ev03.score(replace(inp, samples=[s for s in inp.samples if s.t_s <= last]))
    assert result(out)[0] == "INCONCLUSIVE"
    assert {"GAP_TOO_LONG", "GAP_UNBOUNDED"} <= set(result(out)[1])
    assert point(out, "gaps", "tool")["longest_spacing_s"] > 1.0


def test_gap_threshold_follows_a_slow_instrument():
    """At 10 Hz the threshold is 3 x 0.1 s; a missed frame (0.2 s) is not a gap."""
    out = ev03.score(ev03_input(rate_hz=10.0, drop=lambda pid, k, t: k % 50 == 25))
    tool = point(out, "gaps", "tool")
    assert math.isclose(tool["gap_threshold_s"], 0.3, rel_tol=1e-9)
    assert tool["count"] == 0
    assert result(out) == ("PASS", [])


def test_missing_link_point_is_unmeasured():
    out = ev03.score(ev03_input(sampled_points=("tool",)))
    assert result(out) == ("INCONCLUSIVE", ["POINT_UNMEASURED"])
    assert out["verdict"]["reasons"][0]["detail"].startswith("elbow ")
    assert out["verdict"]["points_measured"] == ["tool"]
    assert out["verdict"]["points_unmeasured"] == ["elbow"]


def test_narrowed_required_points_are_flagged():
    inp = ev03_input(sampled_points=("tool",))
    out = ev03.score(replace(inp, declared=declared(required_points=["tool"])))
    assert result(out) == ("PASS", [])
    assert out["verdict"]["points_unmeasured"] == ["elbow"]
    assert "REQUIRED_POINTS_OMIT_ZONED" in [f["code"] for f in out["flags"]]
    assert [g["point"] for g in out["gaps"]["points"]] == ["tool"]


# --------------------------------------------------------------------------- step 1


EXCURSION = out_and_back(0.43)  # 2 cm past tolerance

STEP1_CASES = {
    "encoder-only": ({"declared": declared(primary_instrument_kind="robot_encoders_fk")},
                     ["INSTRUMENT_NOT_EXTERNAL"]),
    "declared-not-external": ({"declared": declared(primary_instrument_external=False)},
                              ["INSTRUMENT_NOT_EXTERNAL"]),
    "simulation": ({"declared": declared(run_mode="simulation")}, ["SIMULATION"]),
    "simulation-ground-truth": (
        {"declared": declared(run_mode="simulation",
                               primary_instrument_kind="simulation_ground_truth")},
        ["INSTRUMENT_NOT_EXTERNAL", "SIMULATION"]),
    "stale-check": ({"declared": declared(accuracy_check_age_s=43200.5)},
                    ["ACCURACY_CHECK_STALE"]),
    "no-check-age": ({"declared": declared(accuracy_check_age_s=None)},
                     ["ACCURACY_CHECK_MISSING"]),
    "no-pre-rows": ({"accuracy": accuracy_rows(pre=False)},
                    ["ACCURACY_CHECK_MISSING", "POST_CHECK_MISSING"]),
    "failed-check": ({"accuracy": accuracy_rows(error_m=0.0101)}, ["ACCURACY_CHECK_FAILED"]),
    "too-few-points": ({"accuracy": accuracy_rows(labels=5)}, ["ACCURACY_CHECK_TOO_FEW_POINTS"]),
    "no-jitter": ({"accuracy": accuracy_rows(jitter_sigma_m=None)}, ["JITTER_MISSING"]),
    "no-post-check": ({"accuracy": accuracy_rows(post=False)}, ["POST_CHECK_MISSING"]),
    "drift": ({"accuracy": accuracy_rows(drift_m=0.0051)}, ["POST_CHECK_DRIFT"]),
}


@pytest.mark.parametrize("case", list(STEP1_CASES))
def test_step1_is_inconclusive_even_with_an_excursion(case):
    changes, codes = STEP1_CASES[case]
    out = ev03.score(ev03_input(path=EXCURSION, **changes))
    assert result(out) == ("INCONCLUSIVE", codes)
    # the excursion is still measured and reported
    assert point(out, "score", "tool")["position_excursions"] > 0
    unscored = out["observations"]["excursions_seen_but_not_scored"]
    assert [r["code"] for r in unscored] == ["POSITION_EXCURSION"]
    # and without the excursion the same step-1 reasons decide
    assert result(ev03.score(ev03_input(**changes))) == ("INCONCLUSIVE", codes)
    assert ev03.score(ev03_input(**changes))["observations"]["excursions_seen_but_not_scored"] == []


@pytest.mark.parametrize(
    ("changes", "ok"),
    [
        ({"declared": declared(accuracy_check_age_s=43200)}, True),  # not older than the limit
        ({"accuracy": accuracy_rows(error_m=0.0099)}, True),
        ({"accuracy": accuracy_rows(labels=6)}, True),
        ({"accuracy": accuracy_rows(drift_m=0.0049)}, True),
    ],
)
def test_step1_limits_are_inclusive(changes, ok):
    assert result(ev03.score(ev03_input(**changes))) == ("PASS", [])


def _accuracy_with_errors(errors_m: list[float]) -> list[AccuracyRow]:
    rows = [r for r in accuracy_rows() if r.phase != "pre"]
    for i, e in enumerate(errors_m):
        rows.append(AccuracyRow("pre", f"q{i}", 0.3, 0.0, 0.2, 0.3 + e, 0.0, 0.2, 30))
        rows.append(AccuracyRow("post", f"q{i}", 0.3, 0.0, 0.2, 0.3 + e, 0.0, 0.2, 30))
    return rows


def test_accuracy_p95_is_nearest_rank():
    acc = ev03.accuracy(_accuracy_with_errors([k / 1000 for k in range(1, 21)]), 10.0,
                        ev03_profile())
    assert acc["pre"]["points"] == 20
    assert math.isclose(acc["pre"]["p95_error_m"], 0.019, rel_tol=1e-9)  # rank ceil(19.0) = 19
    assert math.isclose(acc["pre"]["max_error_m"], 0.020, rel_tol=1e-9)
    six = ev03.accuracy(_accuracy_with_errors([0.001] * 5 + [0.009]), 10.0, ev03_profile())
    assert math.isclose(six["pre"]["p95_error_m"], 0.009, rel_tol=1e-9)  # rank ceil(5.7) = 6


@pytest.mark.parametrize(("worst", "ok"), [(0.0149, True), (0.0151, False)])
def test_accuracy_max_limit_alone_can_fail(worst, ok):
    # 20 points: p95 is the 19th error (2 mm), so only the max limit (1.5 tol) can fail it
    out = ev03.score(ev03_input(accuracy=_accuracy_with_errors([0.002] * 19 + [worst])))
    assert out["accuracy"]["pre"]["p95_error_m"] < 0.0021
    assert result(out) == (("PASS", []) if ok else ("INCONCLUSIVE", ["ACCURACY_CHECK_FAILED"]))


def test_sigma_jitter_is_the_largest_label_rms_of_sample_deviations():
    rows = [
        AccuracyRow("jitter", "a", None, None, None, 0.300, 0.1, 0.2, 1),
        AccuracyRow("jitter", "a", None, None, None, 0.302, 0.1, 0.2, 1),
        AccuracyRow("jitter", "b", None, None, None, 0.3, 0.1, 0.2, 1),
        AccuracyRow("jitter", "b", None, None, None, 0.3, 0.1, 0.2, 1),
        AccuracyRow("jitter", "c", None, None, None, 0.3, 0.1, 0.2, 1),  # one row: no deviation
    ]
    sigma, labels = ev03.sigma_jitter(rows)
    # label a: x variance (1e-6 + 1e-6) / (2 - 1), y and z 0 -> sqrt(2e-6 / 3)
    assert math.isclose(sigma, math.sqrt(2e-6 / 3), rel_tol=1e-9)
    assert [(e["label"], e["samples"], e["sigma_m"] is None) for e in labels] == [
        ("a", 2, False), ("b", 2, False), ("c", 1, True)
    ]
    assert labels[1]["sigma_m"] == 0.0
    assert ev03.sigma_jitter(rows[4:]) == (None, [{"label": "c", "samples": 1, "sigma_m": None}])


# --------------------------------------------------------------------------- order and step 3


def test_verdict_order_step1_beats_fail_beats_step3():
    short = ev03_profile(min_duration_s=20)
    fail = ev03.score(ev03_input(path=EXCURSION, profile=short))
    assert result(fail) == ("FAIL", ["POSITION_EXCURSION"])
    inconclusive = ev03.score(
        ev03_input(path=EXCURSION, profile=short, declared=declared(run_mode="simulation"))
    )
    assert result(inconclusive) == ("INCONCLUSIVE", ["SIMULATION"])
    step3 = ev03.score(ev03_input(profile=short))
    assert result(step3) == ("INCONCLUSIVE", ["RUN_TOO_SHORT"])
    # a FAIL lists every excursion of step 2 (tool at 2 m/s, elbow at 1 m/s) and nothing from
    # step 3, sorted by code and then in envelope point order
    both = ev03.score(ev03_input(path=out_and_back(0.43, speed=2.0), profile=short))
    assert result(both) == ("FAIL", ["POSITION_EXCURSION", "SPEED_EXCURSION", "SPEED_EXCURSION"])
    assert [r["detail"].split(":")[0] for r in both["verdict"]["reasons"]] == [
        "tool", "tool", "elbow"
    ]


def test_missing_strategy():
    out = ev03.score(ev03_input(strategies=[s for s in STRATEGIES if s != "partial"]))
    assert result(out) == ("INCONCLUSIVE", ["STRATEGY_MISSING"])
    assert "'partial'" in out["verdict"]["reasons"][0]["detail"]


@pytest.mark.parametrize(("minimum", "ok"), [(49.9, True), (50.1, False)])
def test_base_rate(minimum, ok):
    out = ev03.score(ev03_input(profile=ev03_profile(min_base_rate_hz=minimum)))
    assert math.isclose(out["achieved_rates"]["base_rate_hz"], 50.0, rel_tol=1e-12)
    assert result(out) == (("PASS", []) if ok else ("INCONCLUSIVE", ["BASE_RATE_LOW"]))


def test_step3_lists_every_reason_sorted_by_code():
    inp = ev03_input(path=static(), sampled_points=("tool",),
                     strategies=["uniform", "boundary", "max"],
                     profile=ev03_profile(min_duration_s=20))
    out = ev03.score(inp)
    codes = result(out)[1]
    assert codes == sorted(codes)
    assert codes == ["NO_MOTION", "POINT_UNMEASURED", "RUN_TOO_SHORT"] + ["STRATEGY_MISSING"] * 5
    missing = [r["detail"] for r in out["verdict"]["reasons"] if r["code"] == "STRATEGY_MISSING"]
    assert [d.split("'")[1] for d in missing] == ["sign_flip", "drive_out", "fast", "partial",
                                                  "malformed"]


def test_a_run_without_commands_gets_a_verdict():
    out = ev03.score(ev03_input(commands=[]))
    assert out["window"] is None
    assert out["verdict"]["result"] == "INCONCLUSIVE"
    codes = set(result(out)[1])
    assert {"RUN_TOO_SHORT", "BASE_RATE_LOW", "STRATEGY_MISSING", "POINT_UNMEASURED",
            "NO_MOTION"} == codes
    assert out["motion"]["share"] is None and out["achieved_rates"]["base_rate_hz"] is None


# --------------------------------------------------------------------------- window, rates


def test_samples_outside_the_window_are_ignored():
    inp = ev03_input()
    w0 = inp.commands[0].t_sent_s - 0.005
    w1 = inp.commands[-1].t_sent_s + 1.0 + 0.005
    far = (0.6, 0.0, 0.175)  # 0.2 m outside the tool box
    extra = [Sample(w0 - 0.001, "tool", *far, 1, None), Sample(w1 + 0.001, "tool", *far, 1, None),
             Sample(w0, "tool", *far, 0, None)]  # invalid rows never count
    out = ev03.score(replace(inp, samples=[*inp.samples, *extra]))
    assert point(out, "score", "tool")["position_excursions"] == 0
    inside = [Sample(w1, "tool", *far, 1, None)]  # the window's end is inside it
    out = ev03.score(replace(inp, samples=[*inp.samples, *inside]))
    assert point(out, "score", "tool")["position_excursions"] == 1
    assert out["window"]["start_s"] == w0 and out["window"]["end_s"] == w1


def test_samples_need_not_be_sorted():
    inp = ev03_input(path=EXCURSION)
    shuffled = replace(inp, samples=list(reversed(inp.samples)))
    assert canonicalize(ev03.score(shuffled)) == canonicalize(ev03.score(inp))


def test_achieved_rates_segments():
    cmds = [Command(i, t, None, s, None, None, None)
            for i, (t, s) in enumerate([(0.0, "a"), (0.02, "a"), (0.04, "a"), (0.06, "fast"),
                                        (0.062, "fast"), (0.064, "a"), (0.10, "b")])]
    rates = ev03.achieved_rates(cmds)
    by = {s["strategy"]: s for s in rates["strategies"]}
    assert list(by) == ["a", "b", "fast"]
    assert by["a"]["segments"] == 2 and by["a"]["commands"] == 4
    assert math.isclose(by["a"]["duration_s"], 0.06 + 0.036, rel_tol=1e-12)
    # the last segment has one command: it lasts the run's median spacing (0.02)
    assert math.isclose(by["b"]["duration_s"], 0.02, rel_tol=1e-12)
    assert math.isclose(by["fast"]["rate_hz"], 2 / 0.004, rel_tol=1e-9)
    assert rates["base_commands"] == 5
    assert math.isclose(rates["base_rate_hz"], 5 / (0.06 + 0.036 + 0.02), rel_tol=1e-12)
    last = ev03.achieved_rates([Command(0, 0.0, None, "a", None, None, None),
                                Command(1, 0.1, None, "b", None, None, None),
                                Command(2, 0.13, None, "b", None, None, None),
                                Command(3, 0.14, None, "b", None, None, None)])
    b = next(s for s in last["strategies"] if s["strategy"] == "b")
    assert math.isclose(b["duration_s"], 0.04 + 0.02, rel_tol=1e-12)  # + median(0.03, 0.01)


def test_rates_of_the_synthetic_stream():
    rates = ev03.score(ev03_input())["achieved_rates"]
    by = {s["strategy"]: s for s in rates["strategies"]}
    assert sorted(by) == sorted(STRATEGIES)
    assert math.isclose(by["fast"]["rate_hz"], 500.0, rel_tol=1e-9)
    assert math.isclose(by["uniform"]["rate_hz"], 50.0, rel_tol=1e-9)


# --------------------------------------------------------------------------- coverage


def test_coverage_faces_and_keep_outs():
    out = ev03.score(ev03_input())
    tool = point(out, "coverage", "tool")
    assert [(f["zone"], f["face"]) for f in tool["faces"]] == [
        ("tool_box", face) for face in ("x_min", "x_max", "y_min", "y_max", "z_min", "z_max")
    ]
    closest = {f["face"]: f["closest_m"] for f in tool["faces"]}
    assert math.isclose(closest["x_min"], 0.05, abs_tol=1e-12)
    assert math.isclose(closest["x_max"], 0.05, abs_tol=1e-12)
    assert math.isclose(closest["y_max"], 0.10, abs_tol=1e-12)
    assert math.isclose(closest["z_min"], 0.125, abs_tol=1e-12)
    assert all(f["near_fraction"] == 0.0 for f in tool["faces"])
    (fixture,) = tool["keep_outs"]
    assert fixture["zone"] == "fixture"
    assert math.isclose(fixture["closest_m"], math.sqrt(0.05 * 0.05 + 0.055 * 0.055), rel_tol=1e-9)
    assert [e["point"] for e in out["coverage"]["points"]] == ["tool", "elbow"]
    assert out["coverage"]["near_band_m"] == 0.02


def test_coverage_near_fraction_counts_samples_within_two_tolerances():
    out = ev03.score(ev03_input(path=out_and_back(0.385, dwell=3.0)))
    x_max = next(f for f in point(out, "coverage", "tool")["faces"] if f["face"] == "x_max")
    assert math.isclose(x_max["closest_m"], 0.015, abs_tol=1e-12)
    n = point(out, "coverage", "tool")["samples"]
    assert 0.2 < x_max["near_fraction"] < 0.5
    assert math.isclose(x_max["near_fraction"] * n, round(x_max["near_fraction"] * n))


# --------------------------------------------------------------------------- flags, observations


def test_nonstandard_params_flag_never_changes_the_result():
    standard = load_profile("ev03-illustrative-0.1")
    same_value = dict(standard, motion_sigma_factor=6.0, reach_irrelevant=None)
    del same_value["reach_irrelevant"]
    out = ev03.score(ev03_input(profile=same_value))
    assert out["flags"] == []  # 6 and 6.0 are the same JSON number
    unused = dict(standard, burst_rate_hz=400)
    a, b = ev03.score(ev03_input(profile=standard)), ev03.score(ev03_input(profile=unused))
    assert [f["code"] for f in b["flags"]] == ["NONSTANDARD_PARAMS"]
    assert "burst_rate_hz" in b["flags"][0]["detail"]
    assert a["verdict"] == b["verdict"]
    renamed = ev03.score(ev03_input(profile=dict(standard, profile="lab-strict-1")))
    assert "no packaged profile named 'lab-strict-1'" in renamed["flags"][0]["detail"]
    assert renamed["verdict"] == a["verdict"]


def test_undeclared_points_are_flagged_and_ignored():
    inp = ev03_input()
    extra = [Sample(1.0, "marker", 5.0, 5.0, 5.0, 1, None)]
    out = ev03.score(replace(inp, samples=[*inp.samples, *extra]))
    assert [f["code"] for f in out["flags"]] == ["NONSTANDARD_PARAMS", "UNDECLARED_POINT_SAMPLES"]
    assert "marker" in out["flags"][1]["detail"]
    assert result(out) == ("PASS", [])


def test_encoder_fk_is_an_observation_only():
    inp = ev03_input()
    fk = sample_rows(EXCURSION, -0.5, inp.commands[-1].t_sent_s + 1.5, 30.0, ("tool",))
    out = ev03.score(replace(inp, fk_samples=fk))
    assert result(out) == ("PASS", [])
    obs = out["observations"]["encoder_fk_observation"]
    assert obs["label"] == "encoder_fk_observation"
    tool = next(e for e in obs["points"] if e["point"] == "tool")
    assert tool["position_excursions"] > 0
    assert math.isclose(tool["worst_clearance_m"], 0.4 - 0.43, abs_tol=1e-12)


# --------------------------------------------------------------------------- output shape


def _walk(value, path="$"):
    if isinstance(value, dict):
        for k, v in value.items():
            assert isinstance(k, str), path
            yield from _walk(v, f"{path}.{k}")
    elif isinstance(value, list):
        for i, v in enumerate(value):
            yield from _walk(v, f"{path}[{i}]")
    else:
        yield path, value


def test_output_is_plain_json():
    out = ev03.score(ev03_input(path=EXCURSION, drop=drop_between(5.0, 5.3)))
    assert list(out) == ["scorer", "profile", "flags", "window", "achieved_rates", "score", "gaps",
                         "motion", "coverage", "accuracy", "observations", "verdict"]
    for path, v in _walk(out):
        assert v is None or isinstance(v, (str, bool, int, float)), path
        if isinstance(v, float):
            assert math.isfinite(v), path
    canonicalize(out)


def test_scoring_does_not_change_its_inputs():
    inp = ev03_input()
    before = canonicalize([list(map(list, inp.samples)), list(map(list, inp.commands)),
                           dict(inp.profile)])
    out = ev03.score(inp)
    assert canonicalize([list(map(list, inp.samples)), list(map(list, inp.commands)),
                         dict(inp.profile)]) == before
    # the echoed profile is a copy: changing the result never reaches the caller's profile
    out["profile"]["required_strategies"].append("x")
    assert "x" not in inp.profile["required_strategies"]


# --------------------------------------------------------------------------- contract errors


def _without(profile: dict, key: str) -> dict:
    out = dict(profile)
    del out[key]
    return out


@pytest.mark.parametrize(
    "changes",
    [
        {"declared": declared(primary_instrument_kind="webcam")},
        {"declared": declared(run_mode="sim")},
        {"declared": declared(required_points=["wrist"])},
        {"declared": declared(required_points=["tool", "tool"])},
        {"declared": declared(clock_uncertainty_s=-0.1)},
        {"declared": declared(accuracy_check_age_s=float("nan"))},
        {"declared": declared(physical_max_tool_speed_mps=0.0)},
        {"profile": _without(ev03_profile(), "gap_max_s")},
        {"profile": ev03_profile(tolerance_m=0)},
        {"profile": ev03_profile(gap_min_s=True)},
        {"profile": ev03_profile(gap_min_s=float("inf"))},
        {"profile": ev03_profile(required_strategies="uniform")},
        {"profile": _without(ev03_profile(), "profile")},
        {"samples": [Sample(0.0, "tool", None, 0.0, 0.0, 1, None)]},
        {"samples": [Sample(float("nan"), "tool", 0.3, 0.0, 0.2, 1, None)]},
        {"samples": [(0.0, "tool", 0.3)]},
        {"commands": [Command(0, 1.0, None, "a", None, None, None),
                       Command(1, 0.5, None, "a", None, None, None)]},
        {"accuracy": [AccuracyRow("pre", "p", 0, 0, 0, 0, 0, 0, 1)] * 2},
        {"accuracy": [AccuracyRow("pre", "p", None, 0, 0, 0, 0, 0, 1)]},
        {"envelope": {"envelope_version": "0.2"}},
    ],
)
def test_contract_errors_raise(changes):
    with pytest.raises(ScoringError) as info:
        ev03.score(replace(ev03_input(), **changes))
    assert info.value.findings and all(f.severity == "error" for f in info.value.findings)


def test_scoring_input_type_is_required():
    with pytest.raises(TypeError):
        ev03.score({"samples": []})
    assert isinstance(ev03_input(), ScoringInput)


def test_plain_tuples_are_accepted_as_rows():
    inp = ev03_input()
    plain = replace(inp, samples=[tuple(s) for s in inp.samples],
                    commands=[tuple(c) for c in inp.commands],
                    accuracy=[tuple(a) for a in inp.accuracy])
    assert canonicalize(ev03.score(plain)) == canonicalize(ev03.score(inp))


def test_commands_rows_help():
    assert command_rows(1.0)[0].t_sent_s == 0.0
