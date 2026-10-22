"""What ``ev02-stop/1`` and ``ev05-model-loss/1`` share: how a stop after an instant is measured.

An EV-02 trial ends in a stop the bench issues at the stop instant; an EV-05 trial in a model
that stops sending at the loss instant. Either way the scorer asks the same questions of the stop
point's measurements after that instant (method §12.6, §13.6):

- the reference: the last measurement at or before the instant less the clock uncertainty U;
- the speed there, by the speed rule of EV-03 (§7.8.4), over a span of 0.5 w to w;
- the rest: the first measurement from the reference on that every measurement in the next
  ``still_window_s`` stays within the still radius of (and the instrument kept measuring to the
  end of that window); the settled position: the mean of the measurements from the rest to the
  end of that window, and its time the last of them;
- the stop time (rest time less the instant, plus U; 0 if that is negative), the stop distance (the farthest any
  measurement from the reference to the settled position lies from the reference), and the hold
  (the farthest any measurement in the ``hold_s`` after the settled position lies from it).

The rest is the first measurement from which the point stays within the still radius r, so the
last r or less of the travel comes after it: the stop time is early by the time the point takes
to cover that (for a uniform deceleration a, at most sqrt(2 r / a)). The stop distance and the
hold are measured from and to the settled position, so they are not shortened by it. The settled
position is a mean so that the hold compares each measurement with a steadier reference than one
noisy measurement.
- the trial's window: no spacing between consecutive measurements in it longer than the gap
  threshold or ``gap_max_s``, and measurements to its end.

Everything here is a pure function of the measurements; arithmetic is ``+ - * /``, ``math.sqrt``
and comparisons in a fixed order, as everywhere in core.
"""

from __future__ import annotations

import bisect
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field

from ..envelope import Envelope
from .common import (
    GAP_RATE_FACTOR,
    MIN_SAMPLES_PER_SPEED_WINDOW,
    SPEED_ALLOWANCE_FACTOR,
    SPEED_MIN_SPAN_FRACTION,
    Point3,
    distance,
    median,
)

__all__ = [
    "Measurements",
    "StopRules",
    "StopTrial",
    "classify_speed",
    "measure_stop",
    "rate_block",
    "stop_limits",
]

Meas = Sequence[tuple[float, Point3]]


@dataclass(frozen=True)
class Measurements:
    """One point's measurements, sorted by time, with their times for bisection."""

    items: tuple[tuple[float, Point3], ...]
    times: tuple[float, ...]

    @classmethod
    def of(cls, meas: Meas) -> Measurements:
        items = tuple(meas)
        return cls(items, tuple(t for t, _ in items))

    def upto(self, t_end: float) -> Measurements:
        """The measurements at or before ``t_end`` (all of them when it is infinite)."""
        if t_end == math.inf:
            return self
        k = bisect.bisect_right(self.times, t_end)
        return Measurements(self.items[:k], self.times[:k])

    def __len__(self) -> int:
        return len(self.items)


@dataclass(frozen=True)
class StopRules:
    """The numbers a stop is judged by (from the profile, the envelope and the declared U)."""

    clock_uncertainty_s: float
    speed_window_s: float
    gap_threshold_s: float
    gap_max_s: float
    still_radius_m: float
    still_window_s: float
    hold_s: float
    speed_limit_mps: float
    speed_min_frac: float
    tolerance_speed_frac: float
    sigma_m: float  # σ_jitter, 0 when there is no jitter record


@dataclass
class StopTrial:
    """What :func:`measure_stop` found for one instant. ``measured`` is False when the
    measurements cannot support a judgement of this stop (``notes`` says why); then only the
    reference, the speed there and the longest spacing may be set."""

    measured: bool = False
    notes: list[str] = field(default_factory=list)
    reference_t_s: float | None = None
    reference: Point3 | None = None
    speed_mps: float | None = None
    speed_span_s: float | None = None
    speed_class: str | None = None  # "ok" | "slow" | "fast"
    rest_t_s: float | None = None
    rest: Point3 | None = None
    settled_t_s: float | None = None
    settled: Point3 | None = None
    stop_time_s: float | None = None
    stop_distance_m: float | None = None
    hold_until_s: float | None = None
    hold_displacement_m: float | None = None
    hold_observed: bool = False
    longest_spacing_s: float | None = None
    late: bool = False  # not at rest by the allowed time


def _unmeasured(out: StopTrial, note: str) -> StopTrial:
    """``out`` with ``note`` added and everything found after the reference dropped: a rest found
    across a gap in the measurements is no rest."""
    return StopTrial(
        measured=False,
        notes=[*out.notes, note],
        reference_t_s=out.reference_t_s,
        reference=out.reference,
        speed_mps=out.speed_mps,
        speed_span_s=out.speed_span_s,
        speed_class=out.speed_class,
        longest_spacing_s=out.longest_spacing_s,
    )


def stop_limits(envelope: Envelope, source: str) -> tuple[float, float, str]:
    """(max_time_s, max_distance_m, where they come from) for a stop source: its
    ``stop.sources[]`` entry when the envelope declares one, else the top-level ``stop`` block,
    which is the worst case over every source (§4.2)."""
    stop = envelope.stop
    for s in stop.get("sources", ()):
        if s["source"] == source:
            return s["max_time_ms"] / 1000.0, float(s["max_distance_m"]) + 0.0, "stop.sources"
    return stop["max_time_ms"] / 1000.0, float(stop["max_distance_m"]) + 0.0, "stop"


def rate_block(meas: Measurements, gap_min_s: float, speed_window_s: float) -> dict:
    """The stop point's sampling over the whole run: its measurements, their median spacing and
    rate (the mean of the two middle spacings for an even count; None with fewer than two
    measurements, the rate also when the spacing is 0), the least rate the speed rule needs
    (2 / w) and the gap threshold, max(gap_min_s, 3 x the median spacing)."""
    times = meas.times
    steps = [times[k + 1] - times[k] for k in range(len(times) - 1)]
    med = median(steps) if steps else None
    threshold = gap_min_s if med is None else max(gap_min_s, GAP_RATE_FACTOR * med)
    return {
        "samples": len(meas),
        "median_spacing_s": med,
        "median_rate_hz": None if not med else 1.0 / med,
        "min_rate_hz": MIN_SAMPLES_PER_SPEED_WINDOW / speed_window_s,
        "gap_threshold_s": threshold,
    }


def _speed_at(meas: Measurements, i: int, w: float) -> tuple[float | None, float | None]:
    """The speed rule of §7.8.4 at measurement i: against the earliest measurement j with
    t_i - t_j <= w, only over a span of at least 0.5 w."""
    t_i, x_i = meas.items[i]
    times = meas.times
    # The earliest j with t_i - t_j <= w, exactly as §7.8.4 states it: bisect on t_i - w (which
    # is rounded) can land one off either way, so step to the exact boundary.
    j = bisect.bisect_left(times, t_i - w)
    while j > 0 and t_i - times[j - 1] <= w:
        j -= 1
    while t_i - times[j] > w:
        j += 1
    span = t_i - times[j]
    if span < SPEED_MIN_SPAN_FRACTION * w:
        return None, None
    return distance(x_i, meas.items[j][1]) / span, span


def classify_speed(v: float, span: float, rules: StopRules) -> str:
    """"fast" when the speed would be a speed excursion under §7.8.4 (above v_max x (1 +
    tolerance_speed_frac) plus the jitter allowance 3√2 x σ / span), "slow" when it is below
    ``speed_min_frac`` x v_max, else "ok"."""
    limit = rules.speed_limit_mps * (1.0 + rules.tolerance_speed_frac)
    if v > limit + SPEED_ALLOWANCE_FACTOR * rules.sigma_m / span:
        return "fast"
    if v < rules.speed_min_frac * rules.speed_limit_mps:
        return "slow"
    return "ok"


def _observed_to(meas: Measurements, t: float) -> bool:
    """Whether the instrument measured at or after ``t`` (within the measurements given)."""
    return bool(meas.times) and meas.times[-1] >= t


def _at_rest(meas: Measurements, i: int, rules: StopRules) -> bool:
    t_i, x_i = meas.items[i]
    t_end = t_i + rules.still_window_s
    if not _observed_to(meas, t_end):
        return False
    stop = bisect.bisect_right(meas.times, t_end)
    for m in range(i + 1, stop):
        if distance(meas.items[m][1], x_i) > rules.still_radius_m:
            return False
    return True


def _mean(meas: Measurements, i_from: int, i_to: int) -> Point3:
    """The mean position of measurements i_from to i_to (both included), each axis an exactly
    rounded sum (math.fsum) divided by their number."""
    pts = [meas.items[m][1] for m in range(i_from, i_to + 1)]
    n = len(pts)
    return (
        math.fsum(p[0] for p in pts) / n,
        math.fsum(p[1] for p in pts) / n,
        math.fsum(p[2] for p in pts) / n,
    )


def _farthest(meas: Measurements, x0: Point3, i_from: int, t_to: float) -> float:
    """The largest distance from x0 of the measurements from index i_from up to time t_to."""
    worst = 0.0
    for m in range(i_from, len(meas)):
        t, x = meas.items[m]
        if t > t_to:
            break
        d = distance(x, x0)
        worst = max(worst, d)
    return worst


def longest_spacing(meas: Measurements, i_from: int, t_to: float) -> tuple[float | None, bool]:
    """The longest spacing between consecutive measurements from index i_from to the first
    measurement at or after t_to (the pair that straddles t_to included), and whether there is
    such a measurement (the window was measured to its end)."""
    longest = None
    j = i_from
    n = len(meas)
    while j + 1 < n and meas.times[j] < t_to:
        s = meas.times[j + 1] - meas.times[j]
        if longest is None or s > longest:
            longest = s
        j += 1
    return longest, j < n and meas.times[j] >= t_to


def measure_stop(
    meas: Measurements,
    *,
    t_event: float,
    end_t: float,
    allowed_time_s: float,
    rules: StopRules,
) -> StopTrial:
    """Measure the stop that follows ``t_event`` (a stop instant or a loss instant, on the run
    timebase), from the measurements before ``end_t`` (the next segment's first command less U,
    or infinity for the last one).

    The rest and the settled position are as the module docstring says; the hold runs from the
    settled position for ``hold_s``.

    Unmeasured (no judgement either way) when: there is no measurement at or before
    ``t_event - U``; the speed there has no span; a spacing in the trial's window is longer than
    the gap threshold or ``gap_max_s``; or the measurements end before the window does. The
    window runs from the reference to the end of the hold, or, when the point never came to rest,
    to ``allowed_time_s + still_window_s`` after ``t_event - U``, by when a rest in time would
    have shown."""
    out = StopTrial()
    u = rules.clock_uncertainty_s
    data = meas.upto(end_t)
    i_ref = bisect.bisect_right(data.times, t_event - u) - 1
    if i_ref < 0:
        out.notes.append("no measurement at or before the instant (less the clock uncertainty)")
        return out
    out.reference_t_s, out.reference = data.items[i_ref]
    v, span = _speed_at(data, i_ref, rules.speed_window_s)
    if v is None or span is None:
        out.notes.append("the speed at the instant has no span of 0.5 w to w")
        return out
    out.speed_mps, out.speed_span_s = v, span
    out.speed_class = classify_speed(v, span, rules)

    i_rest = None
    for i in range(i_ref, len(data)):
        if _at_rest(data, i, rules):
            i_rest = i
            break
    limit_t = t_event - u + allowed_time_s
    if i_rest is not None:
        out.rest_t_s, out.rest = data.items[i_rest]
        # _at_rest saw a measurement at or after the window's end, so this is i_rest or later.
        i_settled = bisect.bisect_right(data.times, out.rest_t_s + rules.still_window_s) - 1
        out.settled_t_s = data.times[i_settled]
        out.settled = _mean(data, i_rest, i_settled)
        # 0 when the rest is the reference itself (the point stopped within r of where it was)
        out.stop_time_s = max(0.0, out.rest_t_s - t_event + u)
        out.stop_distance_m = _farthest(data, out.reference, i_ref, out.settled_t_s)
        out.hold_until_s = out.settled_t_s + rules.hold_s
        out.hold_displacement_m = _farthest(data, out.settled, i_settled, out.hold_until_s)
        window_end = out.hold_until_s
        out.late = out.rest_t_s > limit_t
    else:
        window_end = limit_t + rules.still_window_s
        out.stop_distance_m = _farthest(data, out.reference, i_ref, math.inf)
        out.late = True
    longest, observed = longest_spacing(data, i_ref, window_end)
    out.longest_spacing_s = longest
    out.hold_observed = i_rest is not None and observed
    if longest is not None and (longest > rules.gap_threshold_s or longest > rules.gap_max_s):
        return _unmeasured(
            out,
            f"a spacing of {longest:.3f} s between measurements in the trial's window (gap "
            f"threshold {rules.gap_threshold_s:.3f} s, longest allowed {rules.gap_max_s:g} s)",
        )
    if i_rest is None and not observed:
        return _unmeasured(out, "the measurements end before a stop in time would have shown")
    out.measured = True
    if i_rest is not None and not observed:
        out.notes.append("the hold was not measured to its end")
    return out


def displacement_in(
    meas: Measurements, x0: Point3, t_from: float, t_to: float
) -> tuple[float | None, float | None, bool]:
    """Over the measurements from t_from to t_to: the largest distance from x0 (None without a
    measurement there), the longest spacing from the last measurement at or before t_from to the
    first at or after t_to, and whether one at or after t_to exists."""
    if not meas.items:
        return None, None, False
    i = max(0, bisect.bisect_right(meas.times, t_from) - 1)
    worst = None
    for m in range(bisect.bisect_left(meas.times, t_from), len(meas)):
        t, x = meas.items[m]
        if t > t_to:
            break
        d = distance(x, x0)
        if worst is None or d > worst:
            worst = d
    longest, observed = longest_spacing(meas, i, t_to)
    return worst, longest, observed


def by_source_order(names: Sequence[str], declared: Sequence[str]) -> list[str]:
    """Source names in declared order, then any others in the order given."""
    out = [s for s in declared if s in names]
    out += [s for s in names if s not in out]
    return out


def plan_point(plan: Mapping | None, envelope: Envelope) -> str:
    point = plan.get("point") if isinstance(plan, Mapping) else None
    return next(iter(envelope.points)) if point is None else point
