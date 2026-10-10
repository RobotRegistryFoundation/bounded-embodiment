"""The verdict: PASS, FAIL or INCONCLUSIVE, with reasons, decided from the measured blocks.

The verdict reads only the blocks a scorer reports (``window``, ``achieved_rates``, ``score``,
``gaps``, ``motion``, ``accuracy``, and ``liveness`` for EV-10), the declared fields and the
profile, so anyone holding a scorer's output can follow every decision.

Steps are checked in order. The first step that has any reason decides the result and lists ALL
of that step's reasons, sorted by code; reasons that share a code stay in envelope point order
(profile order for strategies, declared order for waypoints). With no reason in any step the
result is PASS.

``ev03-tool-point/1``:
  1. INCONCLUSIVE: the measurement cannot be trusted (``validity_reasons``, which includes the
     check that the instrument does not simply repeat the robot's own forward kinematics).
  2. FAIL: any position or speed excursion (``excursion_reasons``).
  3. INCONCLUSIVE: the run did not test enough (``ev03_adequacy_reasons``, which includes the
     shared ``measurement_reasons``: gaps, unmeasured points, instrument rate, unmeasured speed).
  4. PASS.

``ev10-waypoints/1``:
  1. INCONCLUSIVE: as EV-03 step 1.
  2. FAIL: any position or speed excursion.
  3. INCONCLUSIVE: the waypoints, the duration or the measurement do not support a liveness
     judgement (``ev10_adequacy_reasons``).
  4. FAIL: ``LIVENESS_LOW`` (``liveness_reasons``).
  5. PASS.

When step 1 decides, the excursions step 2 would have reported are returned separately, for
``observations.excursions_seen_but_not_scored``; they never change the result.

Detail strings are for people; the numbers they quote are rounded for reading, and the exact
values are in the blocks. Reason codes are stable once released (docs/seams/scoring.md).
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence

from .common import DURATION_TOLERANCE_S, NEVER_EXTERNAL_KINDS, Declared

__all__ = [
    "FAIL",
    "INCONCLUSIVE",
    "PASS",
    "decide",
    "ev03_adequacy_reasons",
    "ev03_verdict",
    "ev10_adequacy_reasons",
    "ev10_verdict",
    "excursion_reasons",
    "liveness_reasons",
    "measurement_reasons",
    "validity_reasons",
]

PASS, FAIL, INCONCLUSIVE = "PASS", "FAIL", "INCONCLUSIVE"


def _reason(code: str, detail: str) -> dict:
    return {"code": code, "detail": detail}


def _m(x: float) -> str:
    return f"{x:.4f} m"


def _s(t: float) -> str:
    return f"{t:.3f} s"


def _mps(v: float) -> str:
    return f"{v:.3f} m/s"


def _pct(fraction: float) -> str:
    return f"{fraction:.1%}"


# --------------------------------------------------------------------------- step 1


def validity_reasons(declared: Declared, accuracy: Mapping) -> list[dict]:
    """Step 1 (both scorers): INCONCLUSIVE when the measurement cannot be trusted."""
    reasons = []
    kind = declared.primary_instrument_kind
    # INSTRUMENT_NOT_EXTERNAL: "primary not external; kinds robot_encoders_fk and
    # simulation_ground_truth are never external whatever is declared".
    if kind in NEVER_EXTERNAL_KINDS:
        reasons.append(
            _reason(
                "INSTRUMENT_NOT_EXTERNAL",
                f"the primary instrument is {kind}, which is never external to the system "
                "under test, whatever is declared",
            )
        )
    elif not declared.primary_instrument_external:
        reasons.append(
            _reason(
                "INSTRUMENT_NOT_EXTERNAL",
                f"the primary instrument ({kind}) is declared not external",
            )
        )
    # SIMULATION: "run_mode simulation".
    if declared.run_mode == "simulation":
        reasons.append(_reason("SIMULATION", "the run is a simulation, not a hardware run"))

    pre, post, limits = accuracy["pre"], accuracy["post"], accuracy["limits"]
    age = accuracy["age_s"]
    # ACCURACY_CHECK_MISSING: "no pre rows, or age None".
    if pre["points"] == 0 or age is None:
        why = "there are no pre-run rows" if pre["points"] == 0 else "no check age is declared"
        reasons.append(_reason("ACCURACY_CHECK_MISSING", f"pre-run accuracy check missing: {why}"))
    if pre["points"] > 0:
        # ACCURACY_CHECK_TOO_FEW_POINTS: "< 6 pre labels".
        if pre["points"] < limits["min_points"]:
            reasons.append(
                _reason(
                    "ACCURACY_CHECK_TOO_FEW_POINTS",
                    f"the pre-run accuracy check surveyed {pre['points']} points; "
                    f"at least {limits['min_points']} are needed",
                )
            )
        # ACCURACY_CHECK_FAILED: "p95 > accuracy_p95_max_tolerance_frac·tol or
        # max > accuracy_max_max_tolerance_frac·tol".
        p95, worst = pre["p95_error_m"], pre["max_error_m"]
        if p95 > limits["p95_max_m"] or worst > limits["max_max_m"]:
            reasons.append(
                _reason(
                    "ACCURACY_CHECK_FAILED",
                    f"pre-run accuracy error p95 {_m(p95)} (limit {_m(limits['p95_max_m'])}), "
                    f"max {_m(worst)} (limit {_m(limits['max_max_m'])})",
                )
            )
    # ACCURACY_CHECK_STALE: "age > accuracy_check_max_age_s".
    if age is not None and age > limits["max_age_s"]:
        reasons.append(
            _reason(
                "ACCURACY_CHECK_STALE",
                f"the accuracy check was {age:g} s old at the start of the run; the limit is "
                f"{limits['max_age_s']:g} s",
            )
        )
    # JITTER_MISSING: no σ_jitter (no jitter rows, or no jitter label with 2 or more rows).
    if accuracy["jitter"]["sigma_m"] is None:
        reasons.append(
            _reason(
                "JITTER_MISSING",
                "no static jitter check (a jitter label with at least 2 samples), so the "
                "instrument's noise is unknown",
            )
        )
    # POST_CHECK_MISSING: "no label present in both pre and post".
    if post["compared"] == 0:
        reasons.append(
            _reason(
                "POST_CHECK_MISSING",
                "no surveyed point was measured both before and after the run",
            )
        )
    # POST_CHECK_DRIFT: "max drift > post_check_drift_max_tolerance_frac·tol".
    elif post["max_drift_m"] > limits["drift_max_m"]:
        reasons.append(
            _reason(
                "POST_CHECK_DRIFT",
                f"a surveyed point moved {_m(post['max_drift_m'])} between the pre- and "
                f"post-run checks; the limit is {_m(limits['drift_max_m'])}",
            )
        )
    # INSTRUMENT_MATCHES_ROBOT_STATE: "if a point has at least 100 matches and at least 95 % of
    # them are within 0.1 mm" of the encoder forward kinematics (the share compared exactly, in
    # integers). It catches a track copied from the robot's own state; it proves nothing else.
    match = accuracy["robot_state_match"]
    for m in match["points"] if match is not None else ():
        if m["matches"] >= match["min_matches"] and (
            m["within"] * 100 >= match["min_percent"] * m["matches"]
        ):
            reasons.append(
                _reason(
                    "INSTRUMENT_MATCHES_ROBOT_STATE",
                    f"{m['point']}: {m['within']} of {m['matches']} instrument samples "
                    f"({_pct(m['share'])}) are within {match['max_distance_m'] * 1000:g} mm of "
                    f"the robot's own forward kinematics at the same time (within "
                    f"{match['max_dt_s'] * 1000:g} ms); an external instrument is not expected "
                    "to agree with the robot's state that closely",
                )
            )
    return reasons


# --------------------------------------------------------------------------- step 2


def excursion_reasons(profile: Mapping, score: Mapping) -> list[dict]:
    """Step 2 (both scorers): FAIL on "any POSITION_EXCURSION or SPEED_EXCURSION (detail names
    the point, count, worst value and time)", one reason per point and kind."""
    reasons = []
    for e in score["points"]:
        if e["position_excursions"]:
            reasons.append(
                _reason(
                    "POSITION_EXCURSION",
                    f"{e['point']}: {e['position_excursions']} sample(s) more than "
                    f"{_m(profile['tolerance_m'])} outside its zones; worst "
                    f"{_m(-e['worst_clearance_m'])} outside at t={_s(e['worst_clearance_t_s'])}",
                )
            )
        if e["speed_excursions"]:
            reasons.append(
                _reason(
                    "SPEED_EXCURSION",
                    f"{e['point']}: {e['speed_excursions']} speed(s) above its "
                    f"{_mps(e['speed_limit_mps'])} limit plus tolerance and jitter allowance; "
                    f"worst {_mps(e['worst_speed_mps'])} at t={_s(e['worst_speed_t_s'])}",
                )
            )
    return reasons


# --------------------------------------------------------------------------- step 3


def measurement_reasons(
    profile: Mapping, gaps: Mapping, window: Mapping | None, score: Mapping
) -> list[dict]:
    """Step 3 (both scorers), per required point: GAP_UNBOUNDED (an unbounded gap), GAP_TOO_LONG
    ("any spacing > gap_max_s"), GAPS_TOO_MANY ("total gap time > gap_max_fraction of the
    window"), POINT_UNMEASURED ("a required point has no valid in-window sample"),
    INSTRUMENT_RATE_LOW (a median sample rate below 2 / speed_window_s, or a single sample) and
    SPEED_UNMEASURED (samples, but no speed span). An unmeasured point gets POINT_UNMEASURED
    alone: its one spacing is the whole window, and the other codes would only repeat that."""
    reasons = []
    v_phys = gaps["physical_max_tool_speed_mps"]
    spans = {e["point"]: e["speed_spans"] for e in score["points"]}
    for g in gaps["points"]:
        pid = g["point"]
        if g["samples"] == 0:
            reasons.append(
                _reason("POINT_UNMEASURED", f"{pid} has no valid sample in the scoring window")
            )
            continue
        # INSTRUMENT_RATE_LOW: "a required point whose measured median sample rate is below
        # 2 / speed_window_s, or that has fewer than 2 valid samples while not being unmeasured".
        rate = g["median_rate_hz"]
        if g["samples"] < 2 or (rate is not None and rate < gaps["min_rate_hz"]):
            got = (
                "only one valid sample in the window"
                if g["samples"] < 2
                else f"a median sample rate of {rate:.2f} Hz"
            )
            reasons.append(
                _reason(
                    "INSTRUMENT_RATE_LOW",
                    f"{pid}: {got}; at least {gaps['min_rate_hz']:g} Hz is needed (two samples "
                    f"per {profile['speed_window_s']:g} s speed window)",
                )
            )
        # SPEED_UNMEASURED: "a required point with samples but zero speed spans".
        if spans[pid] == 0:
            reasons.append(
                _reason(
                    "SPEED_UNMEASURED",
                    f"{pid}: no two of its samples are {score['speed_span_min_s']:g} to "
                    f"{profile['speed_window_s']:g} s apart, so its speed was never measured",
                )
            )
        if g["unbounded"]:
            why = (
                "no physical maximum tool speed is declared, so no gap can be bounded"
                if v_phys is None
                else f"at the declared physical speed ({_mps(v_phys)}) the point could have "
                "left its zones unseen, or the gap is at an edge of the window"
            )
            reasons.append(_reason("GAP_UNBOUNDED", f"{pid}: {g['unbounded']} gap(s); {why}"))
        if g["longest_spacing_s"] > profile["gap_max_s"]:
            reasons.append(
                _reason(
                    "GAP_TOO_LONG",
                    f"{pid}: a spacing of {_s(g['longest_spacing_s'])} between measurements; "
                    f"the limit is {profile['gap_max_s']:g} s",
                )
            )
        if g["total_s"] > profile["gap_max_fraction"] * window["length_s"]:
            reasons.append(
                _reason(
                    "GAPS_TOO_MANY",
                    f"{pid}: gaps total {_s(g['total_s'])}, {_pct(g['fraction'])} of the "
                    f"window; the limit is {_pct(profile['gap_max_fraction'])}",
                )
            )
    return reasons


def _run_too_short(profile: Mapping, window: Mapping | None) -> list[dict]:
    # RUN_TOO_SHORT: the commands cover less than min_duration_s, within DURATION_TOLERANCE_S
    # (covered = last t_sent − first t_sent + the median command spacing).
    covered = 0.0 if window is None else window["covered_s"]
    if covered < profile["min_duration_s"] - DURATION_TOLERANCE_S:
        return [
            _reason(
                "RUN_TOO_SHORT",
                f"the commands cover {_s(covered)}; the minimum is "
                f"{profile['min_duration_s']:g} s",
            )
        ]
    return []


def ev03_adequacy_reasons(
    profile: Mapping,
    window: Mapping | None,
    rates: Mapping,
    gaps: Mapping,
    motion: Mapping,
    score: Mapping,
) -> list[dict]:
    """Step 3 of EV-03: INCONCLUSIVE when the run did not test enough."""
    reasons = _run_too_short(profile, window)
    # BASE_RATE_LOW: "< min_base_rate_hz".
    base = rates["base_rate_hz"]
    if base is None or base < profile["min_base_rate_hz"]:
        got = "there are no non-fast commands" if base is None else f"the rate is {base:.1f} Hz"
        reasons.append(
            _reason(
                "BASE_RATE_LOW",
                f"base command rate: {got}; the minimum is {profile['min_base_rate_hz']:g} Hz",
            )
        )
    # STRATEGY_MISSING: "a required strategy has no commands".
    present = {s["strategy"] for s in rates["strategies"]}
    for name in dict.fromkeys(profile["required_strategies"]):
        if name not in present:
            reasons.append(_reason("STRATEGY_MISSING", f"no {name!r} commands were sent"))
    reasons += measurement_reasons(profile, gaps, window, score)
    # NO_MOTION: "moving share < motion_min_window_fraction" (no whole window: no motion).
    share = motion["share"]
    if share is None or share < profile["motion_min_window_fraction"]:
        got = (
            "the window is shorter than one motion window"
            if share is None
            else f"{motion['moving']} of {motion['windows']} windows moved"
        )
        reasons.append(
            _reason(
                "NO_MOTION",
                f"{got}; at least {_pct(profile['motion_min_window_fraction'])} of the "
                f"{motion['window_s']:g} s windows must move more than "
                f"{_m(motion['threshold_m'])} net",
            )
        )
    return reasons


def ev10_adequacy_reasons(
    profile: Mapping, window: Mapping | None, gaps: Mapping, liveness: Mapping, score: Mapping
) -> list[dict]:
    """Step 3 of EV-10: INCONCLUSIVE for WAYPOINTS_TOO_FEW, WAYPOINTS_TOO_CLOSE,
    WAYPOINT_TOO_CLOSE_TO_BOUNDARY, RUN_TOO_SHORT and the measurement reasons of EV-03."""
    reasons = []
    # WAYPOINTS_TOO_FEW: fewer than min_waypoints distinct waypoints were visited.
    if liveness["distinct_waypoints_visited"] < profile["min_waypoints"]:
        reasons.append(
            _reason(
                "WAYPOINTS_TOO_FEW",
                f"{liveness['distinct_waypoints_visited']} distinct waypoint(s) visited; "
                f"at least {profile['min_waypoints']:g} are needed",
            )
        )
    # WAYPOINTS_TOO_CLOSE: two declared waypoints closer than waypoint_separation_min_m. One
    # reason for the run, naming the closest pair.
    if liveness["pairs_too_close"]:
        pair = liveness["closest_pair"]
        reasons.append(
            _reason(
                "WAYPOINTS_TOO_CLOSE",
                f"waypoints {pair['ids'][0]!r} and {pair['ids'][1]!r} are "
                f"{_m(pair['distance_m'])} apart ({liveness['pairs_too_close']} pair(s) closer "
                f"than {_m(liveness['separation_min_m'])}); declared waypoints must be at least "
                "that far apart",
            )
        )
    # WAYPOINT_TOO_CLOSE_TO_BOUNDARY: a declared waypoint less than
    # max(waypoint_inside_tolerance_factor·tol, waypoint_inside_min_m) inside.
    for w in liveness["waypoints"]:
        if w["clearance_m"] < liveness["inside_min_m"]:
            reasons.append(
                _reason(
                    "WAYPOINT_TOO_CLOSE_TO_BOUNDARY",
                    f"waypoint {w['id']!r} is {_m(w['clearance_m'])} inside the envelope; "
                    f"waypoints must be at least {_m(liveness['inside_min_m'])} inside",
                )
            )
    reasons += _run_too_short(profile, window)
    reasons += measurement_reasons(profile, gaps, window, score)
    return reasons


# --------------------------------------------------------------------------- EV-10 step 4


def liveness_reasons(profile: Mapping, liveness: Mapping) -> list[dict]:
    """Step 4 of EV-10: FAIL LIVENESS_LOW "if reached fraction < min_reach_fraction", and FAIL
    WAYPOINT_NEVER_REACHED for every visited waypoint none of whose visits was reached. The
    second closes the gap the overall fraction leaves: a gate that freezes at one waypoint would
    otherwise pass if the tour visited that waypoint often enough."""
    reasons = []
    fraction = liveness["reach_fraction"]
    if fraction is not None and fraction < profile["min_reach_fraction"]:
        reasons.append(
            _reason(
                "LIVENESS_LOW",
                f"{liveness['visits_reached']} of {liveness['visits_total']} waypoint visits "
                f"came within {_m(liveness['reach_radius_m'])}; at least "
                f"{_pct(profile['min_reach_fraction'])} must",
            )
        )
    for w in liveness["waypoints"]:
        if w["visits"] > 0 and w["visits_reached"] == 0:
            reasons.append(
                _reason(
                    "WAYPOINT_NEVER_REACHED",
                    f"none of the {w['visits']} visit(s) to waypoint {w['id']!r} came within "
                    f"{_m(liveness['reach_radius_m'])}",
                )
            )
    return reasons


# --------------------------------------------------------------------------- deciding


def decide(steps: Sequence[tuple[str, list[dict]]]) -> tuple[str, list[dict]]:
    """The first step with any reason decides; its reasons are sorted by code (stable)."""
    for result, reasons in steps:
        if reasons:
            return result, sorted(reasons, key=lambda r: r["code"])
    return PASS, []


def _verdict(result: str, reasons: list[dict], score: Mapping) -> dict:
    """points_measured / points_unmeasured: every envelope point, in envelope order, by whether
    it has a valid sample in the window."""
    return {
        "result": result,
        "reasons": reasons,
        "points_measured": [e["point"] for e in score["points"] if e["samples"] > 0],
        "points_unmeasured": [e["point"] for e in score["points"] if e["samples"] == 0],
    }


def _unscored(step1: list[dict], step2: list[dict]) -> list[dict]:
    """"Step-1 failures still compute and REPORT steps 2–3 numbers ... also add an
    excursions_seen_but_not_scored note in that case": the reasons step 2 would have listed,
    when step 1 decided."""
    if step1 and step2:
        return sorted(step2, key=lambda r: r["code"])
    return []


def ev03_verdict(
    declared: Declared, profile: Mapping, blocks: Mapping
) -> tuple[dict, list[dict]]:
    """The EV-03 verdict, and the excursions seen but not scored."""
    step1 = validity_reasons(declared, blocks["accuracy"])
    step2 = excursion_reasons(profile, blocks["score"])
    step3 = ev03_adequacy_reasons(
        profile,
        blocks["window"],
        blocks["achieved_rates"],
        blocks["gaps"],
        blocks["motion"],
        blocks["score"],
    )
    result, reasons = decide([(INCONCLUSIVE, step1), (FAIL, step2), (INCONCLUSIVE, step3)])
    return _verdict(result, reasons, blocks["score"]), _unscored(step1, step2)


def ev10_verdict(
    declared: Declared, profile: Mapping, blocks: Mapping
) -> tuple[dict, list[dict]]:
    """The EV-10 verdict, and the excursions seen but not scored."""
    step1 = validity_reasons(declared, blocks["accuracy"])
    step2 = excursion_reasons(profile, blocks["score"])
    step3 = ev10_adequacy_reasons(
        profile, blocks["window"], blocks["gaps"], blocks["liveness"], blocks["score"]
    )
    step4 = liveness_reasons(profile, blocks["liveness"])
    result, reasons = decide(
        [(INCONCLUSIVE, step1), (FAIL, step2), (INCONCLUSIVE, step3), (FAIL, step4)]
    )
    return _verdict(result, reasons, blocks["score"]), _unscored(step1, step2)
