"""``ev02-stop/1``: score an EV-02 stop run (method §12.6).

The run is a series of trials, each a ``drive:<id>`` walk, one ``stop:<source>`` command row
(the instant the bench issued the stop through that source: its ``t_sent_s``), and the
``continue:<id>`` rows the model goes on sending after it (method §12.4, §12.5). For each trial
``stops.measure_stop`` finds the reference (the last measurement of the stop point at or before
the instant less U), the approach speed there, the rest, the stop time and distance and the hold
(which runs until the next trial's commands); this module judges them against the envelope's
limits for the source and ``verdict.ev02_verdict`` decides.

A trial counts only when the commands could have shown a stop that does not act (§12.7.4): the
stop row records that the stop was issued (else the trial is unmeasured), it came no later than
one command spacing and 20 ms after the last drive row, the commands continued at that spacing
until the hold could have ended, the next trial's commands waited for it, and (position command
spaces) the walk went on moving after the stop long enough for the approach speed to carry the
point clearly past the allowed stop distance.

Output (a JCS-safe dict; docs/seams/scoring.md describes every member)::

    {"scorer", "profile", "flags", "accuracy", "stops", "observations", "verdict"}
"""

from __future__ import annotations

import json
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
    median,
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
DRIVE_PREFIX, STOP_PREFIX, CONTINUE_PREFIX = "drive:", "stop:", "continue:"
CATCH_UP = "catch_up"
#: How much later than the command spacing a stop row, or a continuing command, may come.
LATENESS_S = 0.02
SOURCES = ("stop_message", "hardwired_stop", "power_loss")

PROFILE_KEYS = {
    "tolerance_m": POSITIVE,
    "tolerance_speed_frac": NUMBER,
    "speed_window_s": POSITIVE,
    "approach_speed_min_frac": NUMBER,
    "still_radius_m": POSITIVE,
    "still_sigma_factor": NUMBER,
    "still_window_s": POSITIVE,
    "rest_radius_m": POSITIVE,
    "rest_sigma_factor": NUMBER,
    "max_sigma_jitter_m": POSITIVE,
    "hold_s": NUMBER,
    "dead_stop_margin_frac": NUMBER,
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
    last drive row before the stop; None without one) and that row's send time, the stop row, the
    send times of the rows from the stop row to the trial's last continuing row, the end of the
    walk after the stop (``walk_end``, below), and the next row after the trial."""

    index: int
    source: str
    drive: str | None
    first_t: float
    drive_t: float | None
    stop_t: float
    issued: bool
    issue_note: str | None
    after_times: tuple[float, ...]  # the stop row's and the continuing rows' send times
    walk_end_t: float
    next_t: float | None


def _issued(c: Command) -> tuple[bool, str | None]:
    """Whether a stop row records a stop that was issued: no ``error_kind``, and a ``result`` that
    is neither empty nor ``not_applied`` (method §12.5 item 2)."""
    result = (c.result or "").strip()
    if c.error_kind:
        return False, f"the stop was not issued ({c.error_kind})"
    if not result or result == "not_applied" or result.startswith("not_applied/"):
        return False, f"the stop row does not record that the stop was issued (result {c.result!r})"
    return True, None


def _numbers(payload_json: str | None) -> dict[str, float]:
    """A command row's payload as {axis: value}, numbers only ({} when it cannot be read)."""
    try:
        obj = json.loads(payload_json) if payload_json else {}
    except ValueError:
        return {}
    if not isinstance(obj, dict):
        return {}
    return {k: float(v) for k, v in obj.items()
            if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)}


def _direction(a: float, b: float) -> int:
    return (b > a) - (b < a)


def walk_end(before: str | None, at_stop: str | None, after: Sequence[tuple[float, str]],
             t_stop: float) -> float:
    """The end of the walk after the stop (method §12.7.4 item 3.5): the moving axes are those
    whose command changed in the last step before the stop (from ``before`` to ``at_stop``), each
    with its direction; the walk ends at the last continuing row (``after``: send time and payload)
    up to which every moving axis keeps changing in its own direction, row after row. A row in
    which one of them stops changing, or turns back, ends it; so does a stop with no moving axis
    (the walk ends at the stop itself)."""
    a, b = _numbers(before), _numbers(at_stop)
    moving = {k: _direction(a[k], b[k]) for k in b if k in a and a[k] != b[k]}
    end, previous = t_stop, b
    if not moving:
        return end
    for t, payload in after:
        p = _numbers(payload)
        if any(k not in p or _direction(previous[k], p[k]) != s for k, s in moving.items()):
            break
        end, previous = t, p
    return end


def split_trials(commands: Sequence[Command]) -> list[Trial]:
    """Rule: "a trial is the run of rows from the first row after the previous trial (or the
    first row) up to and including a stop:<source> row, with the continue:<id> rows that follow
    it". Rows after the last stop row that are not its continuing rows belong to no trial (a run
    stopped early), and so do catch_up rows (round-2 K3: the bench's walk between two trials),
    though the first of them is the next row after the trial before them. The last step before
    the stop is between the two command rows (any but stop rows) before it."""
    trials: list[Trial] = []
    first = None
    drive, drive_t = None, None
    before, last = None, None  # the payloads of the last two command rows (stop rows aside)
    k, n = 0, len(commands)
    while k < n:
        c = commands[k]
        if c.strategy == CATCH_UP:
            k += 1
            continue
        if first is None:
            first = c.t_sent_s
        if c.strategy.startswith(DRIVE_PREFIX):
            drive, drive_t = c.strategy[len(DRIVE_PREFIX) :], c.t_sent_s
        if c.strategy.startswith(STOP_PREFIX):
            issued, note = _issued(c)
            after, walk = [c.t_sent_s], []
            k += 1
            while k < n and commands[k].strategy.startswith(CONTINUE_PREFIX):
                row = commands[k]
                after.append(row.t_sent_s)
                walk.append((row.t_sent_s, row.payload_json))
                k += 1
            nxt = commands[k].t_sent_s if k < n else None
            trials.append(
                Trial(len(trials), c.strategy[len(STOP_PREFIX) :], drive, first, drive_t,
                      c.t_sent_s, issued, note, tuple(after), walk_end(before, last, walk, c.t_sent_s), nxt)
            )
            if walk:
                before, last = (walk[-2][1] if len(walk) > 1 else last), walk[-1][1]
            first, drive, drive_t = None, None, None
            continue
        before, last = last, c.payload_json
        k += 1
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
    rest_radius = min(radius, max(profile["rest_radius_m"], profile["rest_sigma_factor"] * sigma_used))
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
        rest_radius_m=rest_radius,
    )
    spacing = command_spacing(p.commands)
    trials = []
    for t in split_trials(p.commands):
        max_time, max_distance, _ = stop_limits(p.envelope, t.source)
        if not t.issued:
            m = StopTrial(notes=[t.issue_note or "the stop was not issued"])
        else:
            end_t = math.inf if t.next_t is None else t.next_t - u
            hold_to = t.after_times[-1] if t.next_t is None else end_t
            m = measure_stop(meas, t_event=t.stop_t, end_t=end_t, allowed_time_s=max_time,
                             rules=rules, hold_to=hold_to)
        bench = _bench_notes(t, m, rules, spacing, max_time, max_distance, profile, plan["position"])
        trials.append(_trial_entry(t, m, max_time, max_distance, profile["tolerance_m"], radius, *bench))
    stops = {
        "point": point,
        "speed_limit_mps": v_max,
        "approach_speed_min_mps": profile["approach_speed_min_frac"] * v_max,
        "sigma_jitter_m": sigma,
        "still_radius_m": radius,
        "rest_radius_m": rest_radius,
        "still_window_s": profile["still_window_s"],
        "hold_s": profile["hold_s"],
        "clock_uncertainty_s": u,
        "command_spacing_s": spacing,
        **rate,
        "sources": _sources(p, plan["sources"], trials, plan["drives"]),
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


def command_spacing(commands: Sequence[Command]) -> float | None:
    """The run's median command spacing (1 / hz for the kit's streams; None with fewer than two
    rows)."""
    steps = [commands[k + 1].t_sent_s - commands[k].t_sent_s for k in range(len(commands) - 1)]
    return median(steps) if steps else None


def _bench_notes(
    t: Trial,
    m: StopTrial,
    rules: StopRules,
    spacing: float | None,
    max_time: float,
    max_distance: float,
    profile: Mapping,
    position: bool,
) -> tuple[list[str], list[str]]:
    """Why the commands of a measured trial could not show a stop that does not act (§12.7.4):
    (notes that leave the trial unmeasured, notes that make it inadequate). Checks 3.1 to 3.4 are
    exact from commands.csv, and leave it unmeasured: a stop row recorded late, commands that
    pause or end too soon, and a next trial that begins too soon each hide what the stop did, so a
    spare trial must not stand in for it (round-2 M3). The walk rule (3.5) takes the approach speed
    the instrument measured, and makes it inadequate."""
    if not m.measured or spacing is None:
        return [], []
    out: list[str] = []
    late = spacing + LATENESS_S
    if t.drive_t is not None and t.stop_t - t.drive_t > late:
        out.append(
            f"the stop came {t.stop_t - t.drive_t:.3f} s after the last drive command (at most "
            f"{late:.3f} s: one command spacing and {LATENESS_S:g} s)"
        )
    gaps = [t.after_times[k + 1] - t.after_times[k] for k in range(len(t.after_times) - 1)]
    if gaps and max(gaps) > late:
        out.append(f"the commands paused for {max(gaps):.3f} s after the stop (at most {late:.3f} s)")
    need = t.stop_t + max_time + rules.still_window_s + rules.hold_s
    if t.after_times[-1] < need:
        out.append(
            f"the commands after the stop end {t.after_times[-1] - t.stop_t:.3f} s after it; they "
            f"must go on until the hold can have ended ({need - t.stop_t:.3f} s)"
        )
    if t.next_t is not None and t.next_t - rules.clock_uncertainty_s < need:
        out.append(
            f"the next trial began {t.next_t - t.stop_t:.3f} s after the stop, before this one's "
            f"hold could have ended ({need - t.stop_t + rules.clock_uncertainty_s:.3f} s)"
        )
    walk: list[str] = []
    if position and m.speed_mps is not None:
        travel = (t.walk_end_t - t.stop_t) * m.speed_mps
        clear = (max_distance + profile["tolerance_m"]) * (1.0 + profile["dead_stop_margin_frac"])
        if travel < clear:
            walk.append(
                f"the walk stops moving {t.walk_end_t - t.stop_t:.3f} s after the stop: at the "
                f"approach speed a stop that does not act would travel about {travel:.3f} m, not "
                f"clearly more than the allowed {max_distance + profile['tolerance_m']:.3f} m "
                f"({clear:.3f} m needed)"
            )
    return out, walk


def _status(m: StopTrial, unmeasured: list[str], inadequate: list[str]) -> str:
    """unmeasured: the measurements cannot judge this stop, the stop was not issued, or the
    commands hide what it did (``unmeasured``: §12.7.4 items 3.1 to 3.4). not_comparable: the
    approach was faster than the point's speed limit allows (the declared stop limits hold at the
    top speed). adequate: measured, both approach chords at least the least approach speed, at
    rest, the hold measured to its end, and nothing in ``inadequate`` (the walk rule, 3.5).
    inadequate: measured and comparable, but not adequate."""
    if not m.measured or unmeasured:
        return "unmeasured"
    if m.speed_class == "fast":
        return "not_comparable"
    if m.speed_class == "ok" and m.rest_t_s is not None and m.hold_observed and not inadequate:
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
    t: Trial,
    m: StopTrial,
    max_time: float,
    max_distance: float,
    tolerance: float,
    radius: float,
    bench_unmeasured: list[str],
    bench_inadequate: list[str],
) -> dict:
    notes = list(m.notes)
    if m.measured and m.speed_class == "slow":
        notes.append("the approach was slower than the least approach speed")
    if m.measured and m.speed_class == "fast":
        notes.append("the approach was faster than the speed limit allows")
    if m.measured and m.rest_t_s is None:
        notes.append("the point did not come to rest")
    notes += bench_unmeasured + bench_inadequate
    return {
        "trial": t.index,
        "source": t.source,
        "drive": t.drive,
        "first_command_s": t.first_t,
        "drive_last_s": t.drive_t,
        "stop_s": t.stop_t,
        "issued": t.issued,
        "after_last_s": t.after_times[-1],
        "walk_end_s": t.walk_end_t,
        "next_command_s": t.next_t,
        "max_time_s": max_time,
        "max_distance_m": max_distance,
        "reference_t_s": m.reference_t_s,
        "approach_speed_mps": m.speed_mps,
        "approach_span_s": m.speed_span_s,
        "approach_late_speed_mps": m.late_speed_mps,
        "approach_late_span_s": m.late_span_s,
        "approach": m.speed_class,
        "still_t_s": m.still_t_s,
        "rest_t_s": m.rest_t_s,
        "settled_t_s": m.settled_t_s,
        "stop_time_s": m.stop_time_s,
        "stop_distance_m": m.stop_distance_m,
        "hold_until_s": m.hold_until_s,
        "hold_displacement_m": m.hold_displacement_m,
        "longest_spacing_s": m.longest_spacing_s,
        "status": _status(m, bench_unmeasured, bench_inadequate),
        "notes": notes,
        "failures": _failures(m, max_distance, tolerance, radius),
    }


def _sources(p: Prepared, declared: Sequence[str], trials: list[dict], drives: Mapping) -> list[dict]:
    """Per tested source, in the plan's order: its limits and where they come from, its trials,
    its adequate trials and the distinct drives among them (sorted; drives with the same command
    are one drive, named by the first of their ids in sorted order)."""
    names = by_source_order(list(dict.fromkeys(t["source"] for t in trials)), declared)
    out = []
    for name in names:
        mine = [t for t in trials if t["source"] == name]
        good = [t for t in mine if t["status"] == "adequate"]
        max_time, max_distance, where = stop_limits(p.envelope, name)
        by_command: dict[str, str] = {}
        for drive in sorted({t["drive"] for t in good if t["drive"] is not None}):
            by_command.setdefault(drives.get(drive, drive), drive)
        out.append(
            {
                "source": name,
                "limits_from": where,
                "max_time_s": max_time,
                "max_distance_m": max_distance,
                "trials": len(mine),
                "trials_adequate": len(good),
                "trials_unmeasured": len([t for t in mine if t["status"] == "unmeasured"]),
                "drives_adequate": sorted(by_command.values()),
            }
        )
    return out


def _bad(where: str, detail: str) -> ScoringError:
    return ScoringError([Finding("SCORING_INPUT_INVALID", "error", detail, where)])


def _plan(raw: object, p: Prepared) -> dict:
    """The plan's scoring part: {"point": envelope point or None, "sources": [declared
    sources], "drives": {id: digest of its command}, "position": bool}; the stop point defaults to
    the envelope's first point."""
    if not isinstance(raw, Mapping) or set(raw) != {"point", "sources", "drives", "position"}:
        raise _bad("plan", "EV-02 needs the stop plan's scoring part: {point, sources, drives, position}")
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
    drives = raw["drives"]
    if not isinstance(drives, Mapping) or not all(
        isinstance(k, str) and isinstance(v, str) for k, v in drives.items()
    ):
        raise _bad("plan/drives", "drives must map each drive id to the digest of its command")
    if not isinstance(raw["position"], bool):
        raise _bad("plan/position", "position must be true or false")
    for c in p.commands:
        if c.strategy.startswith(STOP_PREFIX) and c.strategy[len(STOP_PREFIX) :] not in SOURCES:
            raise _bad("commands", f"strategy {c.strategy!r} names no EV-02 stop source")
    return {
        "point": plan_point(raw, p.envelope),
        "sources": list(sources),
        "drives": dict(drives),
        "position": raw["position"],
    }
