"""``ev02-stop/1``: score an EV-02 stop run (method §12.6).

The run is a series of trials, each a ``drive:<id>`` walk that ends in one ``stop:<source>``
command row: the instant the bench issued the stop through that source (its ``t_sent_s``). For
each trial ``stops.measure_stop`` finds the reference (the last measurement of the stop point at
or before the instant less U), the approach speed there, the rest, the stop time and distance and
the hold; this module judges them against the envelope's limits for the source and
``verdict.ev02_verdict`` decides.

Output (a JCS-safe dict; docs/seams/scoring.md describes every member)::

    {"scorer", "profile", "flags", "accuracy", "stops", "observations", "verdict"}
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

from ..findings import Finding
from ..rawdata import Command
from . import ev03
from . import verdict as verdict_rules
from .common import (
    GROUPS,
    NUMBER,
    POSITIVE,
    Prepared,
    ScoringError,
    ScoringInput,
    common_flags,
    in_window_by_point,
    jcs_safe,
    prepare,
)
from .stops import (
    Measurements,
    StopRules,
    StopTrial,
    by_source_order,
    measure_stop,
    plan_point,
    rate_block,
    stop_limits,
)

SCORER = "ev02-stop/1"
DRIVE_PREFIX, STOP_PREFIX = "drive:", "stop:"
SOURCES = ("stop_message", "hardwired_stop", "power_loss")

PROFILE_KEYS = {
    "tolerance_m": POSITIVE,
    "tolerance_speed_frac": NUMBER,
    "speed_window_s": POSITIVE,
    "approach_speed_min_frac": NUMBER,
    "still_radius_m": POSITIVE,
    "still_sigma_factor": NUMBER,
    "still_window_s": POSITIVE,
    "hold_s": NUMBER,
    "min_trials_per_source": NUMBER,
    "min_drives_per_source": NUMBER,
    "required_source_groups": GROUPS,
    "gap_min_s": NUMBER,
    "gap_max_s": NUMBER,
    "accuracy_p95_max_tolerance_frac": NUMBER,
    "accuracy_max_max_tolerance_frac": NUMBER,
    "post_check_drift_max_tolerance_frac": NUMBER,
    "accuracy_check_max_age_s": NUMBER,
}

FAILURES = ("STOP_TIME_EXCEEDED", "STOP_DISTANCE_EXCEEDED", "STOP_NOT_HELD")


@dataclass(frozen=True)
class Trial:
    """One trial as the commands show it: its segment's first command, its drive (the id of the
    last drive row before the stop; None without one), the stop row and the next row after it."""

    index: int
    source: str
    drive: str | None
    first_t: float
    stop_t: float
    next_t: float | None


def split_trials(commands: Sequence[Command]) -> list[Trial]:
    """Rule: "a trial is the run of rows from the first row after the previous stop row (or the
    first row) up to and including a stop:<source> row". Rows after the last stop row belong to
    no trial (a run stopped early)."""
    trials: list[Trial] = []
    first = None
    drive = None
    for k, c in enumerate(commands):
        if first is None:
            first = c.t_sent_s
        if c.strategy.startswith(DRIVE_PREFIX):
            drive = c.strategy[len(DRIVE_PREFIX) :]
        elif c.strategy.startswith(STOP_PREFIX):
            nxt = commands[k + 1].t_sent_s if k + 1 < len(commands) else None
            trials.append(
                Trial(len(trials), c.strategy[len(STOP_PREFIX) :], drive, first, c.t_sent_s, nxt)
            )
            first, drive = None, None
    return trials


def score(inp: ScoringInput) -> dict:
    """Score a run with ``ev02-stop/1``. Raises ``ScoringError`` when the inputs break the
    contract (including a missing or malformed plan); every run that satisfies it gets a
    verdict."""
    p = prepare(inp, PROFILE_KEYS)
    plan = _plan(inp.plan, p)
    point = plan["point"]
    profile, u = p.profile, p.declared.clock_uncertainty_s
    window = None if not p.commands else (p.commands[0].t_sent_s - u, math.inf)
    by_point, undeclared = in_window_by_point(p.samples, window, p.envelope)
    fk_by_point = None
    if p.fk_samples is not None:
        fk_by_point, _ = in_window_by_point(p.fk_samples, window, p.envelope)
    acc = ev03.accuracy(p.accuracy, p.declared.accuracy_check_age_s, profile)
    acc["robot_state_match"] = ev03.robot_state_match(by_point, fk_by_point, [point])
    sigma = acc["jitter"]["sigma_m"]
    sigma_used = 0.0 if sigma is None else sigma
    meas = Measurements.of(by_point[point])
    rate = rate_block(meas, profile["gap_min_s"], profile["speed_window_s"])
    radius = max(profile["still_radius_m"], profile["still_sigma_factor"] * sigma_used)
    v_max = p.envelope.speed_limit_mps(point)
    rules = StopRules(
        clock_uncertainty_s=u,
        speed_window_s=profile["speed_window_s"],
        gap_threshold_s=rate["gap_threshold_s"],
        gap_max_s=profile["gap_max_s"],
        still_radius_m=radius,
        still_window_s=profile["still_window_s"],
        hold_s=profile["hold_s"],
        speed_limit_mps=v_max,
        speed_min_frac=profile["approach_speed_min_frac"],
        tolerance_speed_frac=profile["tolerance_speed_frac"],
        sigma_m=sigma_used,
    )
    trials = []
    for t in split_trials(p.commands):
        max_time, max_distance, _ = stop_limits(p.envelope, t.source)
        end_t = math.inf if t.next_t is None else t.next_t - u
        m = measure_stop(meas, t_event=t.stop_t, end_t=end_t, allowed_time_s=max_time, rules=rules)
        trials.append(_trial_entry(t, m, max_time, max_distance, profile["tolerance_m"], radius))
    stops = {
        "point": point,
        "speed_limit_mps": v_max,
        "approach_speed_min_mps": profile["approach_speed_min_frac"] * v_max,
        "sigma_jitter_m": sigma,
        "still_radius_m": radius,
        "still_window_s": profile["still_window_s"],
        "hold_s": profile["hold_s"],
        "clock_uncertainty_s": u,
        **rate,
        "sources": _sources(p, plan["sources"], trials),
        "trials": trials,
    }
    verdict, unscored = verdict_rules.ev02_verdict(p.declared, profile, acc, stops)
    out = {
        "scorer": SCORER,
        "profile": profile,
        "flags": common_flags(p, undeclared),
        "accuracy": acc,
        "stops": stops,
        "observations": {"failures_seen_but_not_scored": unscored},
        "verdict": verdict_rules.points_verdict(verdict, p.envelope, by_point),
    }
    return jcs_safe(out)


def _status(m: StopTrial) -> str:
    """unmeasured: the measurements cannot judge this stop. not_comparable: the approach was
    faster than the point's speed limit allows (the declared stop limits hold at the top speed).
    adequate: measured, approach at least the least approach speed, at rest, the hold measured to
    its end. inadequate: measured and comparable, but not adequate."""
    if not m.measured:
        return "unmeasured"
    if m.speed_class == "fast":
        return "not_comparable"
    if m.speed_class == "ok" and m.rest_t_s is not None and m.hold_observed:
        return "adequate"
    return "inadequate"


def _failures(m: StopTrial, max_distance: float, tolerance: float, radius: float) -> list[str]:
    if not m.measured:
        return []
    out = []
    if m.late:
        out.append("STOP_TIME_EXCEEDED")
    if m.stop_distance_m is not None and m.stop_distance_m > max_distance + tolerance:
        out.append("STOP_DISTANCE_EXCEEDED")
    if m.hold_displacement_m is not None and m.hold_displacement_m > radius:
        out.append("STOP_NOT_HELD")
    return out


def _trial_entry(
    t: Trial, m: StopTrial, max_time: float, max_distance: float, tolerance: float, radius: float
) -> dict:
    notes = list(m.notes)
    if m.measured and m.speed_class == "slow":
        notes.append("the approach was slower than the least approach speed")
    if m.measured and m.speed_class == "fast":
        notes.append("the approach was faster than the speed limit allows")
    if m.measured and m.rest_t_s is None:
        notes.append("the point did not come to rest")
    return {
        "trial": t.index,
        "source": t.source,
        "drive": t.drive,
        "first_command_s": t.first_t,
        "stop_s": t.stop_t,
        "next_command_s": t.next_t,
        "max_time_s": max_time,
        "max_distance_m": max_distance,
        "reference_t_s": m.reference_t_s,
        "approach_speed_mps": m.speed_mps,
        "approach_span_s": m.speed_span_s,
        "approach": m.speed_class,
        "rest_t_s": m.rest_t_s,
        "settled_t_s": m.settled_t_s,
        "stop_time_s": m.stop_time_s,
        "stop_distance_m": m.stop_distance_m,
        "hold_until_s": m.hold_until_s,
        "hold_displacement_m": m.hold_displacement_m,
        "longest_spacing_s": m.longest_spacing_s,
        "status": _status(m),
        "notes": notes,
        "failures": _failures(m, max_distance, tolerance, radius),
    }


def _sources(p: Prepared, declared: Sequence[str], trials: list[dict]) -> list[dict]:
    """Per tested source, in the plan's order: its limits and where they come from, its trials,
    its adequate trials and the distinct drives among them (sorted)."""
    names = by_source_order(list(dict.fromkeys(t["source"] for t in trials)), declared)
    out = []
    for name in names:
        mine = [t for t in trials if t["source"] == name]
        good = [t for t in mine if t["status"] == "adequate"]
        max_time, max_distance, where = stop_limits(p.envelope, name)
        out.append(
            {
                "source": name,
                "limits_from": where,
                "max_time_s": max_time,
                "max_distance_m": max_distance,
                "trials": len(mine),
                "trials_adequate": len(good),
                "drives_adequate": sorted({t["drive"] for t in good if t["drive"] is not None}),
            }
        )
    return out


def _bad(where: str, detail: str) -> ScoringError:
    return ScoringError([Finding("SCORING_INPUT_INVALID", "error", detail, where)])


def _plan(raw: object, p: Prepared) -> dict:
    """The plan's scoring part: {"point": envelope point or None, "sources": [declared
    sources]}; the stop point defaults to the envelope's first point."""
    if not isinstance(raw, Mapping) or set(raw) != {"point", "sources"}:
        raise _bad("plan", "EV-02 needs the stop plan's scoring part: {point, sources}")
    point = raw["point"]
    if point is not None and (not isinstance(point, str) or point not in p.envelope.points):
        raise _bad("plan/point", f"{point!r} is not a point of the envelope")
    sources = raw["sources"]
    if (
        not isinstance(sources, (list, tuple))
        or not all(isinstance(s, str) and s in SOURCES for s in sources)
        or len(set(sources)) != len(sources)
    ):
        raise _bad("plan/sources", f"sources must be distinct names from {', '.join(SOURCES)}")
    for c in p.commands:
        if c.strategy.startswith(STOP_PREFIX) and c.strategy[len(STOP_PREFIX) :] not in SOURCES:
            raise _bad("commands", f"strategy {c.strategy!r} names no EV-02 stop source")
    return {"point": plan_point(raw, p.envelope), "sources": list(sources)}
