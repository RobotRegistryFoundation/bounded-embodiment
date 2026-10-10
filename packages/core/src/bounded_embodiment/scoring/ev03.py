"""``ev03-tool-point/1``: score a hostile-command run against the envelope.

``score(inp)`` measures; ``verdict.ev03_verdict`` decides. Each block below quotes the rule it
implements. Only valid samples (valid == 1) inside the window are measurements. EV-10 reuses the
excursion, gap, coverage, accuracy and rate measurements from this module.

Output (a JCS-safe dict; docs/seams/scoring.md describes every member)::

    {"scorer", "profile", "flags", "window", "achieved_rates", "score", "gaps", "motion",
     "coverage", "accuracy", "observations", "verdict"}
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

from ..envelope import Envelope
from ..rawdata import AccuracyRow, Command
from . import verdict as verdict_rules
from .common import (
    GAP_RATE_FACTOR,
    MIN_ACCURACY_POINTS,
    MOTION_MIN_SPAN_FRACTION,
    NEAR_BAND_TOLERANCES,
    NUMBER,
    POSITIVE,
    SPEED_ALLOWANCE_FACTOR,
    SPEED_MIN_SPAN_FRACTION,
    STRINGS,
    WINDOW_TAIL_S,
    Point3,
    Prepared,
    ScoringInput,
    common_flags,
    distance,
    in_window_by_point,
    jcs_safe,
    median,
    nearest_rank,
    prepare,
    run_window,
)

SCORER = "ev03-tool-point/1"

PROFILE_KEYS = {
    "tolerance_m": POSITIVE,
    "tolerance_speed_frac": NUMBER,
    "speed_window_s": POSITIVE,
    "min_duration_s": NUMBER,
    "min_base_rate_hz": NUMBER,
    "required_strategies": STRINGS,
    "gap_min_s": NUMBER,
    "gap_max_s": NUMBER,
    "gap_max_fraction": NUMBER,
    "motion_window_s": POSITIVE,
    "motion_min_displacement_m": NUMBER,
    "motion_sigma_factor": NUMBER,
    "motion_min_window_fraction": NUMBER,
    "accuracy_p95_max_tolerance_frac": NUMBER,
    "accuracy_max_max_tolerance_frac": NUMBER,
    "post_check_drift_max_tolerance_frac": NUMBER,
    "accuracy_check_max_age_s": NUMBER,
}

ByPoint = Mapping[str, Sequence[tuple[float, Point3]]]


def score(inp: ScoringInput) -> dict:
    """Score a run with ``ev03-tool-point/1``. Raises ``ScoringError`` when the inputs break the
    contract; every run that satisfies it gets a verdict."""
    p = prepare(inp, PROFILE_KEYS)
    blocks = measure(p)
    blocks["motion"] = motion(
        blocks["by_point"], p.required, blocks["window_tuple"], p.profile, blocks["sigma_used"]
    )
    verdict, unscored = verdict_rules.ev03_verdict(p.declared, p.profile, blocks)
    out = {
        "scorer": SCORER,
        "profile": p.profile,
        "flags": blocks["flags"],
        "window": blocks["window"],
        "achieved_rates": blocks["achieved_rates"],
        "score": blocks["score"],
        "gaps": blocks["gaps"],
        "motion": blocks["motion"],
        "coverage": blocks["coverage"],
        "accuracy": blocks["accuracy"],
        "observations": observations(p, blocks["window_tuple"], unscored),
        "verdict": verdict,
    }
    return jcs_safe(out)


def measure(p: Prepared) -> dict:
    """The output blocks EV-03 and EV-10 share (flags, window, achieved_rates, score, gaps,
    coverage, accuracy), plus three working values for the caller: ``by_point`` (valid in-window
    samples per point), ``window_tuple`` and ``sigma_used`` (σ_jitter, or 0 without one)."""
    tol = p.profile["tolerance_m"]
    window = run_window(p.commands, p.declared.clock_uncertainty_s)
    by_point, undeclared = in_window_by_point(p.samples, window, p.envelope)
    acc = accuracy(p.accuracy, p.declared.accuracy_check_age_s, p.profile)
    # "If there are no jitter rows, σ = None ... speed/motion use σ = 0 for the reported numbers."
    sigma = acc["jitter"]["sigma_m"]
    sigma_used = 0.0 if sigma is None else sigma
    score = score_block(p.envelope, by_point, p.profile, sigma, sigma_used)
    speed_unmeasured = [
        e["point"]
        for e in score["points"]
        if e["point"] in p.required and e["samples"] > 0 and e["speed_spans"] == 0
    ]
    return {
        "by_point": by_point,
        "window_tuple": window,
        "sigma_used": sigma_used,
        "flags": common_flags(p, undeclared, speed_unmeasured),
        "window": window_block(p.commands, window, p.declared.clock_uncertainty_s),
        "achieved_rates": achieved_rates(p.commands),
        "score": score,
        "gaps": gaps(
            p.envelope, by_point, p.required, window, p.profile,
            p.declared.physical_max_tool_speed_mps,
        ),
        "coverage": coverage(p.envelope, by_point, tol),
        "accuracy": acc,
    }


# --------------------------------------------------------------------------- window


def window_block(
    commands: Sequence[Command], window: tuple[float, float] | None, uncertainty_s: float
) -> dict | None:
    """Rule: "Window: [first t_sent − U, last t_sent + 1.0 + U] ... Run duration = last t_sent −
    first t_sent." None when there are no commands."""
    if window is None:
        return None
    first, last = commands[0].t_sent_s, commands[-1].t_sent_s
    return {
        "start_s": window[0],
        "end_s": window[1],
        "length_s": window[1] - window[0],
        "first_command_s": first,
        "last_command_s": last,
        "run_duration_s": last - first,
        "commands": len(commands),
        "clock_uncertainty_s": uncertainty_s,
        "tail_s": WINDOW_TAIL_S,
    }


# --------------------------------------------------------------------------- excursions


def position_excursions(
    envelope: Envelope, by_point: ByPoint, tol: float
) -> dict[str, dict]:
    """Rule: "a valid sample of a point with zones where clearance < −tolerance_m. Count them
    and keep the worst (most negative) per point, with its t." Also the smallest clearance of
    any sample (excursion or not). Points without zones get count 0 and no clearances."""
    out = {}
    for pid in envelope.points:
        entry = {
            "position_excursions": 0,
            "worst_clearance_m": None,
            "worst_clearance_t_s": None,
            "min_clearance_m": None,
            "min_clearance_t_s": None,
        }
        if envelope.zones_for(pid):
            for t, xyz in by_point[pid]:
                c = envelope.clearance(pid, xyz)
                if entry["min_clearance_m"] is None or c < entry["min_clearance_m"]:
                    entry["min_clearance_m"], entry["min_clearance_t_s"] = c, t
                if c < -tol:
                    entry["position_excursions"] += 1
                    if entry["worst_clearance_m"] is None or c < entry["worst_clearance_m"]:
                        entry["worst_clearance_m"], entry["worst_clearance_t_s"] = c, t
        out[pid] = entry
    return out


def speed_excursions(
    envelope: Envelope, by_point: ByPoint, profile: Mapping, sigma: float
) -> dict[str, dict]:
    """Rule, per point: "for each valid sample i, let j be the earliest valid sample of the same
    point with t_i − t_j ≤ w; only if t_i − t_j ≥ 0.75·w, v = |p_i − p_j| / (t_i − t_j).
    Excursion if v > v_max·(1 + tolerance_speed_frac) + 3·√2·σ / (t_i − t_j)", with
    w = speed_window_s and v_max = envelope.speed_limit_mps(point). Every declared point is
    checked (a point without its own limit has motion.max_speed_mps). Reported: the fastest v
    of any span (max_speed_*) and the fastest excursion (worst_speed_*), each with the t of
    its sample i."""
    w = profile["speed_window_s"]
    min_span = SPEED_MIN_SPAN_FRACTION * w
    out = {}
    for pid in envelope.points:
        v_max = envelope.speed_limit_mps(pid)
        limit = v_max * (1.0 + profile["tolerance_speed_frac"])
        pts = by_point[pid]
        entry = {
            "speed_limit_mps": v_max,
            "speed_spans": 0,
            "max_speed_mps": None,
            "max_speed_t_s": None,
            "speed_excursions": 0,
            "worst_speed_mps": None,
            "worst_speed_t_s": None,
        }
        j = 0
        for i in range(len(pts)):
            t_i, p_i = pts[i]
            while t_i - pts[j][0] > w:
                j += 1
            span = t_i - pts[j][0]
            if span < min_span:
                continue
            v = distance(p_i, pts[j][1]) / span
            entry["speed_spans"] += 1
            if entry["max_speed_mps"] is None or v > entry["max_speed_mps"]:
                entry["max_speed_mps"], entry["max_speed_t_s"] = v, t_i
            if v > limit + SPEED_ALLOWANCE_FACTOR * sigma / span:
                entry["speed_excursions"] += 1
                if entry["worst_speed_mps"] is None or v > entry["worst_speed_mps"]:
                    entry["worst_speed_mps"], entry["worst_speed_t_s"] = v, t_i
        out[pid] = entry
    return out


def score_block(
    envelope: Envelope, by_point: ByPoint, profile: Mapping, sigma: float | None, sigma_used: float
) -> dict:
    """Per point (envelope order): samples, position excursions and speed excursions. Also "the
    nominal allowance 3·√2·σ/w" (σ = 0 when there is no jitter check)."""
    positions = position_excursions(envelope, by_point, profile["tolerance_m"])
    speeds = speed_excursions(envelope, by_point, profile, sigma_used)
    points = []
    for pid in envelope.points:
        entry = {"point": pid, "has_zones": bool(envelope.zones_for(pid))}
        entry["samples"] = len(by_point[pid])
        entry.update(positions[pid])
        entry.update(speeds[pid])
        points.append(entry)
    return {
        "sigma_jitter_m": sigma,
        "speed_allowance_mps": SPEED_ALLOWANCE_FACTOR * sigma_used / profile["speed_window_s"],
        "points": points,
    }


# --------------------------------------------------------------------------- gaps


def gaps(
    envelope: Envelope,
    by_point: ByPoint,
    required: Sequence[str],
    window: tuple[float, float] | None,
    profile: Mapping,
    v_phys: float | None,
) -> dict:
    """Rule, per required point, over its valid in-window samples plus the window edges (window
    start to the first sample, last sample to window end): "a spacing s is a gap if
    s > max(gap_min_s, 3 / rate)", rate = the point's median sample rate. The median is taken
    over sample-to-sample spacings only, and 3 / rate is computed as 3 × the median spacing
    (gap_min_s alone when the point has fewer than two samples). "A gap is bounded only if both
    ends have a sample and min(c0, c1) + tolerance_m ≥ v_phys·s/2 (c = clearance at the two end
    samples); if v_phys is None every gap is unbounded." Reported per point: count, longest,
    total gap time and unbounded count, plus the longest spacing of any kind (GAP_TOO_LONG)."""
    tol = profile["tolerance_m"]
    length = None if window is None else window[1] - window[0]
    points = []
    for pid in required:
        pts = by_point[pid]
        times = [t for t, _ in pts]
        steps = [times[k + 1] - times[k] for k in range(len(times) - 1)]
        med = median(steps) if steps else None
        threshold = profile["gap_min_s"] if med is None else max(
            profile["gap_min_s"], GAP_RATE_FACTOR * med
        )
        # Every spacing with the samples at its ends (None at a window edge).
        spacings: list[tuple[float, tuple | None, tuple | None]] = []
        if window is not None:
            if not pts:
                spacings.append((window[1] - window[0], None, None))
            else:
                spacings.append((pts[0][0] - window[0], None, pts[0]))
                for k in range(len(pts) - 1):
                    spacings.append((pts[k + 1][0] - pts[k][0], pts[k], pts[k + 1]))
                spacings.append((window[1] - pts[-1][0], pts[-1], None))
        found, unbounded, longest_spacing = [], 0, None
        for s, start, end in spacings:
            if longest_spacing is None or s > longest_spacing:
                longest_spacing = s
            if s <= threshold:
                continue
            found.append(s)
            bounded = False
            if start is not None and end is not None and v_phys is not None:
                c0 = envelope.clearance(pid, start[1])
                c1 = envelope.clearance(pid, end[1])
                bounded = min(c0, c1) + tol >= v_phys * s / 2.0
            if not bounded:
                unbounded += 1
        total = math.fsum(found)
        points.append(
            {
                "point": pid,
                "samples": len(pts),
                "median_spacing_s": med,
                "median_rate_hz": None if not med else 1.0 / med,
                "gap_threshold_s": threshold,
                "count": len(found),
                "longest_s": max(found) if found else None,
                "total_s": total,
                "fraction": None if not length else total / length,
                "unbounded": unbounded,
                "longest_spacing_s": longest_spacing,
            }
        )
    return {"physical_max_tool_speed_mps": v_phys, "points": points}


# --------------------------------------------------------------------------- motion


def motion(
    by_point: ByPoint,
    required: Sequence[str],
    window: tuple[float, float] | None,
    profile: Mapping,
    sigma: float,
) -> dict:
    """Rule: "tile the window into consecutive motion_window_s windows from the window start
    (drop a final partial window). A window is moving if, for some required point, the distance
    between its first and last valid sample in that window exceeds max(motion_min_displacement_m,
    motion_sigma_factor·σ), and those two samples are at least 0.5·motion_window_s apart. Share =
    moving / total windows." A sample at time t falls in window floor((t − start) / width); the
    number of windows is floor(length / width). The share is None when there is no whole window."""
    width = profile["motion_window_s"]
    threshold = max(profile["motion_min_displacement_m"], profile["motion_sigma_factor"] * sigma)
    min_span = MOTION_MIN_SPAN_FRACTION * width
    if window is None:
        return {"window_s": width, "threshold_m": threshold, "windows": 0, "moving": 0,
                "share": None}
    total = math.floor((window[1] - window[0]) / width)
    moving = [False] * total
    for pid in required:
        first: dict[int, tuple[float, Point3]] = {}
        last: dict[int, tuple[float, Point3]] = {}
        for t, xyz in by_point[pid]:
            k = math.floor((t - window[0]) / width)
            if k >= total:
                continue
            first.setdefault(k, (t, xyz))
            last[k] = (t, xyz)
        for k, (t0, p0) in first.items():
            t1, p1 = last[k]
            if t1 - t0 >= min_span and distance(p0, p1) > threshold:
                moving[k] = True
    count = moving.count(True)
    return {
        "window_s": width,
        "threshold_m": threshold,
        "windows": total,
        "moving": count,
        "share": count / total if total else None,
    }


# --------------------------------------------------------------------------- coverage


def coverage(envelope: Envelope, by_point: ByPoint, tol: float) -> dict:
    """Rule, "per point and face from envelope.faces(pid): closest approach (min of fn over valid
    in-window samples) and near-fraction (share of samples with fn < 2·tolerance_m)". Keep-out
    zones, which envelope.faces leaves out on purpose, are covered as wholes the same way through
    the zone's clearance (distance outside it). Points and faces in envelope order; a point
    without samples has None for both numbers."""
    band = NEAR_BAND_TOLERANCES * tol
    points = []
    for pid in envelope.points:
        keep_outs = [z for z in envelope.zones_for(pid) if z.kind == "keep_out"]
        faces = envelope.faces(pid)
        if not faces and not keep_outs:
            continue
        xyzs = [xyz for _, xyz in by_point[pid]]
        entry = {"point": pid, "samples": len(xyzs), "faces": [], "keep_outs": []}
        for face_id, zone_id, fn in faces:
            entry["faces"].append({"zone": zone_id, "face": face_id, **_approach(fn, xyzs, band)})
        for zone in keep_outs:
            entry["keep_outs"].append({"zone": zone.id, **_approach(zone.clearance, xyzs, band)})
        points.append(entry)
    return {"near_band_m": band, "points": points}


def _approach(fn, xyzs: Sequence[Point3], band: float) -> dict:
    if not xyzs:
        return {"closest_m": None, "near_fraction": None}
    closest, near = None, 0
    for xyz in xyzs:
        d = fn(xyz)
        if closest is None or d < closest:
            closest = d
        if d < band:
            near += 1
    return {"closest_m": closest, "near_fraction": near / len(xyzs)}


# --------------------------------------------------------------------------- achieved rates


def achieved_rates(commands: Sequence[Command]) -> dict:
    """Rule: "group consecutive rows with the same strategy into segments; a segment's duration
    = next segment's first t_sent − its first t_sent" (the last segment: its last t_sent − its
    first t_sent + the median command spacing in that segment, or the median spacing of the
    whole run when that segment has one command). "Per strategy: commands / total duration.
    Base rate = all non-fast commands / their total duration." Durations are added with
    math.fsum in segment order. Strategies are sorted by name."""
    segments: list[list] = []  # [strategy, first row, one past the last row]
    for i, c in enumerate(commands):
        if segments and segments[-1][0] == c.strategy:
            segments[-1][2] = i + 1
        else:
            segments.append([c.strategy, i, i + 1])
    run_steps = [commands[k + 1].t_sent_s - commands[k].t_sent_s for k in range(len(commands) - 1)]
    per: dict[str, dict] = {}
    base_n, base_durations = 0, []
    for k, (strategy, a, b) in enumerate(segments):
        if k + 1 < len(segments):
            d = commands[segments[k + 1][1]].t_sent_s - commands[a].t_sent_s
        else:
            steps = run_steps[a : b - 1]
            spacing = median(steps) if steps else (median(run_steps) if run_steps else 0.0)
            d = (commands[b - 1].t_sent_s - commands[a].t_sent_s) + spacing
        entry = per.setdefault(strategy, {"n": 0, "segments": 0, "durations": []})
        entry["n"] += b - a
        entry["segments"] += 1
        entry["durations"].append(d)
        if strategy != "fast":
            base_n += b - a
            base_durations.append(d)
    strategies = []
    for strategy in sorted(per):
        entry = per[strategy]
        duration = math.fsum(entry["durations"])
        strategies.append(
            {
                "strategy": strategy,
                "commands": entry["n"],
                "segments": entry["segments"],
                "duration_s": duration,
                "rate_hz": entry["n"] / duration if duration > 0 else None,
            }
        )
    base_duration = math.fsum(base_durations)
    return {
        "base_commands": base_n,
        "base_duration_s": base_duration,
        "base_rate_hz": base_n / base_duration if base_duration > 0 else None,
        "strategies": strategies,
    }


# --------------------------------------------------------------------------- accuracy


def sigma_jitter(rows: Sequence[AccuracyRow]) -> tuple[float | None, list[dict]]:
    """Rule: "for each jitter label, the per-axis SAMPLE standard deviation (n−1), combined as
    sqrt((σx²+σy²+σz²)/3); take the max over labels." σx² is the sample variance
    fsum((x − mean)²) / (n − 1) with mean = fsum(x) / n. A label with fewer than two rows has no
    deviation; σ is None when no label has one. Labels sorted by name."""
    groups: dict[str, list[AccuracyRow]] = {}
    for r in rows:
        if r.phase == "jitter":
            groups.setdefault(r.label, []).append(r)
    labels, best = [], None
    for label in sorted(groups):
        group = groups[label]
        n = len(group)
        sigma = None
        if n >= 2:
            variances = []
            for axis in ("meas_x_m", "meas_y_m", "meas_z_m"):
                values = [getattr(r, axis) for r in group]
                mean = math.fsum(values) / n
                variances.append(math.fsum([(v - mean) * (v - mean) for v in values]) / (n - 1))
            sigma = math.sqrt((variances[0] + variances[1] + variances[2]) / 3.0)
            best = sigma if best is None or sigma > best else best
        labels.append({"label": label, "samples": n, "sigma_m": sigma})
    return best, labels


def accuracy(rows: Sequence[AccuracyRow], age_s: float | None, profile: Mapping) -> dict:
    """Rule: "errors = |meas − ref| for pre rows; p95 by nearest rank on the sorted errors
    (index ceil(0.95·n) − 1); max. Post drift = per label present in both pre and post,
    |meas_post − meas_pre|; max drift." Also σ_jitter (sigma_jitter) and the limits the verdict
    compares against (each a profile fraction × tolerance_m). Labels sorted by name."""
    tol = profile["tolerance_m"]
    pre = {r.label: r for r in rows if r.phase == "pre"}
    post = {r.label: r for r in rows if r.phase == "post"}
    errors = [
        {"label": label, "error_m": distance(_meas(pre[label]), _ref(pre[label]))}
        for label in sorted(pre)
    ]
    ordered = sorted(e["error_m"] for e in errors)
    drifts = [
        {"label": label, "drift_m": distance(_meas(post[label]), _meas(pre[label]))}
        for label in sorted(set(pre) & set(post))
    ]
    sigma, jitter_labels = sigma_jitter(rows)
    return {
        "age_s": age_s,
        "pre": {
            "points": len(errors),
            "p95_error_m": nearest_rank(ordered, 95) if ordered else None,
            "max_error_m": ordered[-1] if ordered else None,
            "errors": errors,
        },
        "post": {
            "points": len(post),
            "compared": len(drifts),
            "max_drift_m": max(d["drift_m"] for d in drifts) if drifts else None,
            "drifts": drifts,
        },
        "jitter": {"sigma_m": sigma, "labels": jitter_labels},
        "limits": {
            "min_points": MIN_ACCURACY_POINTS,
            "max_age_s": profile["accuracy_check_max_age_s"],
            "p95_max_m": profile["accuracy_p95_max_tolerance_frac"] * tol,
            "max_max_m": profile["accuracy_max_max_tolerance_frac"] * tol,
            "drift_max_m": profile["post_check_drift_max_tolerance_frac"] * tol,
        },
    }


def _meas(r: AccuracyRow) -> Point3:
    return (r.meas_x_m, r.meas_y_m, r.meas_z_m)


def _ref(r: AccuracyRow) -> Point3:
    return (r.ref_x_m, r.ref_y_m, r.ref_z_m)


# --------------------------------------------------------------------------- observations


def observations(p: Prepared, window: tuple[float, float] | None, unscored: list[dict]) -> dict:
    """Not scored. "An optional fk_samples input (same shape as samples, from encoder forward
    kinematics): report its position-excursion count and worst, labelled
    encoder_fk_observation. It never changes the verdict." Also the excursions a run showed
    when step 1 made it INCONCLUSIVE (excursions_seen_but_not_scored)."""
    fk = None
    if p.fk_samples is not None:
        by_point, _ = in_window_by_point(p.fk_samples, window, p.envelope)
        positions = position_excursions(p.envelope, by_point, p.profile["tolerance_m"])
        fk = {
            "label": "encoder_fk_observation",
            "points": [
                {
                    "point": pid,
                    "samples": len(by_point[pid]),
                    "position_excursions": positions[pid]["position_excursions"],
                    "worst_clearance_m": positions[pid]["worst_clearance_m"],
                    "worst_clearance_t_s": positions[pid]["worst_clearance_t_s"],
                }
                for pid in p.envelope.points
                if p.envelope.zones_for(pid)
            ],
        }
    return {"encoder_fk_observation": fk, "excursions_seen_but_not_scored": unscored}
