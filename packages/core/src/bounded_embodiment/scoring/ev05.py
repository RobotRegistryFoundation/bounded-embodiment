"""``ev05-model-loss/1``: score an EV-05 model-loss run (method §13.6).

The run is a series of trials, each a run of ``motion`` command rows, a silence (the model is
lost: no command, no goodbye), then ``restart`` rows (the model is back). The loss instant is the
last motion row's ``t_sent_s``, the last command the system under test received. For each trial
``stops.measure_stop`` measures the stop that follows the loss against the envelope's
``heartbeat.model_timeout_ms`` plus the stop limits of the plan's source; this module adds the
restart check (the point stays at rest while the commands return, until the plan's resume rule
lets it move) and ``verdict.ev05_verdict`` decides.

Output (a JCS-safe dict; docs/seams/scoring.md describes every member)::

    {"scorer", "profile", "flags", "accuracy", "model_loss", "observations", "verdict"}
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
    DURATION_TOLERANCE_S,
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
    displacement_in,
    measure_stop,
    plan_point,
    rate_block,
    stop_limits,
)

SCORER = "ev05-model-loss/1"
MOTION, RESTART = "motion", "restart"
SOURCES = ("heartbeat_loss", "comms_loss")

PROFILE_KEYS = {
    "tolerance_m": POSITIVE,
    "tolerance_speed_frac": NUMBER,
    "speed_window_s": POSITIVE,
    "loss_speed_min_frac": NUMBER,
    "still_radius_m": POSITIVE,
    "still_sigma_factor": NUMBER,
    "still_window_s": POSITIVE,
    "rest_radius_m": POSITIVE,
    "rest_sigma_factor": NUMBER,
    "max_sigma_jitter_m": POSITIVE,
    "hold_s": NUMBER,
    "restart_check_s": NUMBER,
    "min_resume_check_s": NUMBER,
    "min_trials": NUMBER,
    "gap_min_s": NUMBER,
    "gap_max_s": NUMBER,
    "accuracy_p95_max_tolerance_frac": NUMBER,
    "accuracy_max_max_tolerance_frac": NUMBER,
    "post_check_drift_max_tolerance_frac": NUMBER,
    "accuracy_check_max_age_s": NUMBER,
}

FAILURES = (
    "LOSS_STOP_TIME_EXCEEDED",
    "LOSS_STOP_DISTANCE_EXCEEDED",
    "LOSS_STOP_NOT_HELD",
    "LOSS_RESUMED_EARLY",
)


@dataclass(frozen=True)
class Trial:
    """One trial as the commands show it: its motion rows' first and last send times, its restart
    rows' send times (empty when the run stopped before they came) and the next row after them."""

    index: int
    first_t: float
    loss_t: float
    restart_times: tuple[float, ...]
    next_t: float | None


def split_trials(commands: Sequence[Command]) -> list[Trial]:
    """Rule: "a trial is a maximal run of motion rows and the restart rows that follow it; a
    motion row after a restart row starts the next trial". Restart rows before any motion row
    belong to no trial, and so do catch_up rows (round-2 K3: the bench's walk between two trials);
    a trial's next row is the row after its last motion or restart row."""
    groups: list[tuple[list[Command], list[Command], int]] = []  # (motion, restart, next index)
    for k, c in enumerate(commands):
        if c.strategy == MOTION:
            if not groups or groups[-1][1]:
                groups.append(([], [], k))
            groups[-1][0].append(c)
        elif c.strategy == RESTART and groups:
            groups[-1][1].append(c)
        else:
            continue
        groups[-1] = (groups[-1][0], groups[-1][1], k + 1)
    out = []
    for i, (motion, restart, nxt) in enumerate(groups):
        out.append(
            Trial(
                i,
                motion[0].t_sent_s,
                motion[-1].t_sent_s,
                tuple(c.t_sent_s for c in restart),
                commands[nxt].t_sent_s if nxt < len(commands) else None,
            )
        )
    return out


def score(inp: ScoringInput) -> dict:
    """Score a run with ``ev05-model-loss/1``. Raises ``ScoringError`` when the inputs break the
    contract (including a missing or malformed plan); every run that satisfies it gets a
    verdict."""
    p = prepare(inp, PROFILE_KEYS)
    plan = _plan(inp.plan, p)
    point, source, resume = plan["point"], plan["source"], plan["resume"]
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
    timeout = p.envelope.heartbeat["model_timeout_ms"] / 1000.0
    max_time, max_distance, where = stop_limits(p.envelope, source)
    allowed_time = timeout + max_time
    allowed_distance = v_max * timeout + max_distance
    restart_check = (
        profile["restart_check_s"] if resume["rule"] == "manual" else resume["after_ms"] / 1000.0
    )
    # Round-2 M4: in a position space the stop half is met by construction, so the restart check
    # is all that EV-05 tests; under a commands rule it judges only to the restart + c - U.
    short_check = (
        plan["position"]
        and resume["rule"] == "commands"
        and restart_check - u < profile["min_resume_check_s"]
    )
    rules = StopRules(
        clock_uncertainty_s=u,
        speed_window_s=profile["speed_window_s"],
        gap_threshold_s=rate["gap_threshold_s"],
        gap_max_s=profile["gap_max_s"],
        still_radius_m=radius,
        still_window_s=profile["still_window_s"],
        hold_s=profile["hold_s"],
        speed_limit_mps=v_max,
        speed_min_frac=profile["loss_speed_min_frac"],
        tolerance_speed_frac=profile["tolerance_speed_frac"],
        sigma_m=sigma_used,
        rest_radius_m=rest_radius,
    )
    steps = [p.commands[k + 1].t_sent_s - p.commands[k].t_sent_s for k in range(len(p.commands) - 1)]
    run_spacing = median(steps) if steps else 0.0
    trials = []
    for t in split_trials(p.commands):
        trials.append(
            _trial(
                t,
                meas,
                rules,
                allowed_time=allowed_time,
                allowed_distance=allowed_distance,
                restart_check=restart_check,
                commands_rule=resume["rule"] == "commands",
                short_check=profile["min_resume_check_s"] if short_check else None,
                tolerance=profile["tolerance_m"],
                run_spacing=run_spacing,
            )
        )
    model_loss = {
        "point": point,
        "source": source,
        "limits_from": where,
        "speed_limit_mps": v_max,
        "model_timeout_s": timeout,
        "max_time_s": max_time,
        "max_distance_m": max_distance,
        "allowed_time_s": allowed_time,
        "allowed_distance_m": allowed_distance,
        "resume": dict(resume),
        "restart_check_s": restart_check,
        "loss_speed_min_mps": profile["loss_speed_min_frac"] * v_max,
        "sigma_jitter_m": sigma,
        "still_radius_m": radius,
        "rest_radius_m": rest_radius,
        "still_window_s": profile["still_window_s"],
        "hold_s": profile["hold_s"],
        "clock_uncertainty_s": u,
        **rate,
        "trials": trials,
        "trials_adequate": len([t for t in trials if t["status"] == "adequate"]),
        "trials_unmeasured": len([t for t in trials if t["status"] == "unmeasured"]),
    }
    verdict, unscored = verdict_rules.ev05_verdict(p.declared, profile, acc, model_loss)
    out = {
        "scorer": SCORER,
        "profile": profile,
        "flags": common_flags(p, undeclared),
        "accuracy": acc,
        "model_loss": model_loss,
        "observations": {"failures_seen_but_not_scored": unscored},
        "verdict": verdict_rules.points_verdict(verdict, p.envelope, by_point),
    }
    return jcs_safe(out)


def _covered(times: tuple[float, ...], run_spacing: float) -> float:
    """The time the restart commands cover: last less first send time plus their median spacing
    (the run's median command spacing for a single row), as §7.8.9 counts a run's duration."""
    if not times:
        return 0.0
    steps = [times[k + 1] - times[k] for k in range(len(times) - 1)]
    spacing = median(steps) if steps else run_spacing
    return (times[-1] - times[0]) + spacing


def _trial(
    t: Trial,
    meas: Measurements,
    rules: StopRules,
    *,
    allowed_time: float,
    allowed_distance: float,
    restart_check: float,
    commands_rule: bool,
    short_check: float | None,
    tolerance: float,
    run_spacing: float,
) -> dict:
    u = rules.clock_uncertainty_s
    restart = t.restart_times[0] if t.restart_times else None
    end_stop = restart - u if restart is not None else (math.inf if t.next_t is None else t.next_t - u)
    # the hold runs until the commands come back (less U), and at least hold_s
    m = measure_stop(meas, t_event=t.loss_t, end_t=end_stop, allowed_time_s=allowed_time, rules=rules,
                     hold_to=None if restart is None else end_stop)
    covered = _covered(t.restart_times, run_spacing)
    restart_until = displacement = None
    restart_ok = restart_check == 0.0
    notes = list(m.notes)
    if restart is not None and restart_check > 0.0 and m.measured and m.settled is not None:
        # measured to restart + c + U, but under a commands rule a move is judged only up to
        # restart + c - U: the machine may move once the rule allows, and U blurs when that is
        restart_until = restart + restart_check + u
        judged_until = restart + restart_check - u if commands_rule else restart_until
        later = meas.upto(math.inf if t.next_t is None else t.next_t - u)
        displacement, longest, observed = displacement_in(
            later, m.settled, restart - u, restart_until, judged_until
        )
        gap = longest is not None and (
            longest > rules.gap_threshold_s or longest > rules.gap_max_s
        )
        restart_ok = observed and not gap
        if gap:
            notes.append(f"a spacing of {longest:.3f} s between measurements in the restart window")
        elif not observed:
            notes.append("the restart window was not measured to its end")
        elif displacement is None:  # nothing was judged, so nothing was checked (round-2 M4)
            notes.append("no measurement in the judged part of the restart window")
            restart_ok = False
    if restart is None:
        notes.append("the commands never restarted")
        restart_ok = restart_check == 0.0 and restart_ok
    elif restart_check > 0.0 and covered < restart_check - DURATION_TOLERANCE_S:
        notes.append("the restart commands are shorter than the restart check")
        restart_ok = False
    if short_check is not None:
        notes.append(
            f"the restart check judges {restart_check - u:.3f} s after the restart (after_ms less "
            f"U), less than min_resume_check_s ({short_check:g} s): in a position command space "
            "it is all that EV-05 tests"
        )
        restart_ok = False
    status = _status(m, restart_ok)
    if m.measured and m.speed_class == "slow":
        notes.append("the point was slower than the least speed at the loss")
    if m.measured and m.speed_class == "fast":
        notes.append("the point was faster than its speed limit allows at the loss")
    if m.measured and m.rest_t_s is None:
        notes.append("the point did not come to rest before the commands restarted")
    failures = []
    if m.measured:
        if m.late:
            failures.append("LOSS_STOP_TIME_EXCEEDED")
        if m.stop_distance_m is not None and m.stop_distance_m > allowed_distance + tolerance:
            failures.append("LOSS_STOP_DISTANCE_EXCEEDED")
        if m.hold_displacement_m is not None and m.hold_displacement_m > rules.still_radius_m:
            failures.append("LOSS_STOP_NOT_HELD")
        if displacement is not None and displacement > rules.still_radius_m:
            failures.append("LOSS_RESUMED_EARLY")
    return {
        "trial": t.index,
        "first_command_s": t.first_t,
        "loss_s": t.loss_t,
        "restart_s": restart,
        "restart_last_s": t.restart_times[-1] if t.restart_times else None,
        "next_command_s": t.next_t,
        "silence_s": None if restart is None else restart - t.loss_t,
        "restart_covered_s": covered,
        "reference_t_s": m.reference_t_s,
        "speed_at_loss_mps": m.speed_mps,
        "speed_span_s": m.speed_span_s,
        "speed_late_mps": m.late_speed_mps,
        "speed_late_span_s": m.late_span_s,
        "speed": m.speed_class,
        "still_t_s": m.still_t_s,
        "rest_t_s": m.rest_t_s,
        "settled_t_s": m.settled_t_s,
        "stop_time_s": m.stop_time_s,
        "stop_distance_m": m.stop_distance_m,
        "hold_until_s": m.hold_until_s,
        "hold_displacement_m": m.hold_displacement_m,
        "restart_until_s": restart_until,
        "restart_displacement_m": displacement,
        "longest_spacing_s": m.longest_spacing_s,
        "status": status,
        "notes": notes,
        "failures": failures,
    }


def _status(m: StopTrial, restart_ok: bool) -> str:
    """As ev02's, with the restart check: adequate also needs the restart window measured to its
    end and covered by restart commands (or a resume rule that makes the check empty)."""
    if not m.measured:
        return "unmeasured"
    if m.speed_class == "fast":
        return "not_comparable"
    if m.speed_class == "ok" and m.rest_t_s is not None and m.hold_observed and restart_ok:
        return "adequate"
    return "inadequate"


def _bad(where: str, detail: str) -> ScoringError:
    return ScoringError([Finding("SCORING_INPUT_INVALID", "error", detail, where)])


def _plan(raw: object, p: Prepared) -> dict:
    """The plan's scoring part: {"point", "source", "resume", "position"}; the stop point
    defaults to the envelope's first point."""
    if not isinstance(raw, Mapping) or set(raw) != {"point", "source", "resume", "position"}:
        raise _bad(
            "plan", "EV-05 needs the model-loss plan's scoring part: {point, source, resume, position}"
        )
    if not isinstance(raw["position"], bool):
        raise _bad("plan/position", "position must be true or false")
    point = raw["point"]
    if point is not None and (not isinstance(point, str) or point not in p.envelope.points):
        raise _bad("plan/point", f"{point!r} is not a point of the envelope")
    if raw["source"] not in SOURCES:
        raise _bad("plan/source", f"source must be one of {', '.join(SOURCES)}")
    resume = raw["resume"]
    manual = isinstance(resume, Mapping) and dict(resume) == {"rule": "manual"}
    after = resume.get("after_ms") if isinstance(resume, Mapping) else None
    commands = (
        isinstance(resume, Mapping)
        and set(resume) == {"rule", "after_ms"}
        and resume.get("rule") == "commands"
        and isinstance(after, int)
        and not isinstance(after, bool)
        and 0 <= after <= 3_600_000
    )
    if not (manual or commands):
        raise _bad("plan/resume", 'resume must be {"rule": "manual"} or {"rule": "commands", "after_ms": n}')
    return {
        "point": plan_point(raw, p.envelope),
        "source": raw["source"],
        "resume": dict(resume),
        "position": raw["position"],
    }
