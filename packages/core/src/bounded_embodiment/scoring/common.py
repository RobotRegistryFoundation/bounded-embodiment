"""Shared pieces of the scorers: the inputs, the method constants, input checks, flags and a few
exact numeric helpers.

Every number a verdict depends on is computed with ``+ - * /``, ``math.sqrt`` and ``math.fsum``
in a fixed order, so it is bit-identical on every IEEE 754 platform and every supported Python.
Built-in ``sum`` (it changed in Python 3.12), ``statistics``, numpy and ``math.hypot``/``dist``
are never used (``tests/test_no_float_sum.py`` enforces it).
"""

from __future__ import annotations

import copy
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

from ..canonical import CanonicalError, canonicalize
from ..envelope import Envelope
from ..findings import Finding
from ..profiles import load_profile
from ..rawdata import AccuracyRow, Command, Sample

__all__ = [
    "INSTRUMENT_KINDS",
    "MIN_ACCURACY_POINTS",
    "MOTION_MIN_SPAN_FRACTION",
    "NEAR_BAND_TOLERANCES",
    "NEVER_EXTERNAL_KINDS",
    "RUN_MODES",
    "SPEED_ALLOWANCE_FACTOR",
    "SPEED_MIN_SPAN_FRACTION",
    "WINDOW_TAIL_S",
    "Declared",
    "Prepared",
    "ScoringError",
    "ScoringInput",
    "distance",
    "in_window_by_point",
    "jcs_safe",
    "median",
    "nearest_rank",
    "prepare",
    "run_window",
]

# --------------------------------------------------------------------------- method constants
# Fixed by the method text, not by a profile: changing one changes the scorer's name.

WINDOW_TAIL_S = 1.0
"""The scoring window runs to the last command + this + the clock uncertainty."""

SPEED_MIN_SPAN_FRACTION = 0.75
"""A speed is taken only over a span of at least this fraction of speed_window_s."""

SPEED_ALLOWANCE_FACTOR = 3.0 * math.sqrt(2.0)
"""The jitter allowance on speed is this x sigma / span (3 sigma of a difference of two samples)."""

MOTION_MIN_SPAN_FRACTION = 0.5
"""A motion window counts only if its first and last samples are this fraction of it apart."""

NEAR_BAND_TOLERANCES = 2.0
"""Coverage counts a sample as near a face when it is within this many tolerances of it."""

MIN_ACCURACY_POINTS = 6
"""The pre-run accuracy check needs at least this many surveyed points (labels)."""

GAP_RATE_FACTOR = 3.0
"""A spacing is a gap above max(gap_min_s, this / median rate)."""

INSTRUMENT_KINDS = (
    "markerless_model_tracker",
    "optical_marker_tracker",
    "motion_capture",
    "laser_tracker",
    "depth_camera",
    "robot_encoders_fk",
    "simulation_ground_truth",
    "other",
)
NEVER_EXTERNAL_KINDS = ("robot_encoders_fk", "simulation_ground_truth")
RUN_MODES = ("hardware", "simulation")


# --------------------------------------------------------------------------- inputs


class ScoringError(ValueError):
    """The inputs break the scorer's contract (not a verdict about the machine)."""

    def __init__(self, findings: list[Finding]):
        self.findings = list(findings)
        head = "; ".join(
            f"{f.where + ': ' if f.where else ''}{f.detail}" for f in self.findings[:3]
        )
        more = f" (+{len(self.findings) - 3} more)" if len(self.findings) > 3 else ""
        super().__init__(f"cannot score: {head}{more}" if head else "cannot score")


@dataclass(frozen=True)
class Declared:
    """What the signed report declares. The scorer uses these for the verdict's preconditions
    and never as measurements.

    ``required_points``: the points whose measurement the run must cover (gaps, motion,
    ``POINT_UNMEASURED``); ``None`` means every point that has zones."""

    primary_instrument_kind: str
    primary_instrument_external: bool
    run_mode: str  # "hardware" | "simulation"
    accuracy_check_age_s: float | None  # age at run start; None: no check
    clock_uncertainty_s: float
    physical_max_tool_speed_mps: float | None = None
    required_points: Sequence[str] | None = None


@dataclass(frozen=True)
class ScoringInput:
    """Everything a scorer reads. Rows are ``bounded_embodiment.rawdata`` rows (or plain tuples
    in the same field order). ``fk_samples`` (encoder forward kinematics, same shape as
    ``samples``) is reported as an observation and never scored. ``waypoints`` (EV-10 only) is
    ``[{"id": str, "position_m": [x, y, z]}]``; ``liveness_point`` is the point EV-10 measures
    (default: the envelope's first point)."""

    envelope: Envelope
    samples: Sequence[Sample]
    commands: Sequence[Command]
    accuracy: Sequence[AccuracyRow]
    declared: Declared
    profile: Mapping[str, Any]
    fk_samples: Sequence[Sample] | None = None
    waypoints: Sequence[Mapping[str, Any]] | None = None
    liveness_point: str | None = None


@dataclass(frozen=True)
class Prepared:
    """The checked inputs: rows as rawdata NamedTuples, the profile as a plain dict, and the
    required points in envelope order."""

    envelope: Envelope
    samples: list[Sample]
    commands: list[Command]
    accuracy: list[AccuracyRow]
    fk_samples: list[Sample] | None
    declared: Declared
    profile: dict
    required: list[str]


# --------------------------------------------------------------------------- input checks

POSITIVE, NUMBER, STRINGS, NAME = "positive", "number", "strings", "name"


def _is_number(x: object) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


class _Checks:
    def __init__(self) -> None:
        self.findings: list[Finding] = []

    def bad(self, where: str, detail: str) -> None:
        if len(self.findings) < 20:
            self.findings.append(Finding("SCORING_INPUT_INVALID", "error", detail, where))

    def done(self) -> None:
        if self.findings:
            raise ScoringError(self.findings)


def _check_profile(profile: object, keys: Mapping[str, str], checks: _Checks) -> dict:
    if not isinstance(profile, Mapping):
        checks.bad("profile", f"the profile must be a mapping, not {type(profile).__name__}")
        return {}
    out = copy.deepcopy(dict(profile))  # the result echoes it; never share the caller's lists
    if not isinstance(out.get("profile"), str) or not out.get("profile"):
        checks.bad("profile/profile", "the profile needs a non-empty string name in 'profile'")
    for key, kind in keys.items():
        where = f"profile/{key}"
        if key not in out:
            checks.bad(where, f"the profile has no {key!r}")
            continue
        value = out[key]
        if kind == STRINGS:
            if not isinstance(value, list) or not all(isinstance(s, str) for s in value):
                checks.bad(where, f"{key} must be a list of strings")
        elif not _is_number(value) or value < 0 or (kind == POSITIVE and value == 0):
            need = "a finite number above 0" if kind == POSITIVE else "a finite number >= 0"
            checks.bad(where, f"{key} must be {need}, not {value!r}")
    try:
        canonicalize(out)
    except CanonicalError as e:
        checks.bad("profile", f"the profile is not JSON-safe: {e}")
    return out


def _check_declared(declared: object, envelope: Envelope, checks: _Checks) -> None:
    if not isinstance(declared, Declared):
        checks.bad("declared", f"declared must be a Declared, not {type(declared).__name__}")
        return
    d = declared
    if d.primary_instrument_kind not in INSTRUMENT_KINDS:
        checks.bad(
            "declared/primary_instrument_kind",
            f"{d.primary_instrument_kind!r} is not one of {', '.join(INSTRUMENT_KINDS)}",
        )
    if not isinstance(d.primary_instrument_external, bool):
        checks.bad("declared/primary_instrument_external", "must be true or false")
    if d.run_mode not in RUN_MODES:
        checks.bad("declared/run_mode", f"{d.run_mode!r} is not one of {', '.join(RUN_MODES)}")
    if d.accuracy_check_age_s is not None and (
        not _is_number(d.accuracy_check_age_s) or d.accuracy_check_age_s < 0
    ):
        checks.bad("declared/accuracy_check_age_s", "must be None or a finite number >= 0")
    if not _is_number(d.clock_uncertainty_s) or d.clock_uncertainty_s < 0:
        checks.bad("declared/clock_uncertainty_s", "must be a finite number >= 0")
    if d.physical_max_tool_speed_mps is not None and (
        not _is_number(d.physical_max_tool_speed_mps) or d.physical_max_tool_speed_mps <= 0
    ):
        checks.bad("declared/physical_max_tool_speed_mps", "must be None or a finite number > 0")
    if d.required_points is not None:
        if isinstance(d.required_points, str) or not all(
            isinstance(p, str) for p in d.required_points
        ):
            checks.bad("declared/required_points", "must be None or a list of point ids")
            return
        unknown = [p for p in d.required_points if p not in envelope.points]
        if unknown:
            checks.bad(
                "declared/required_points",
                f"not points of the envelope: {', '.join(map(repr, unknown))}",
            )
        if len(set(d.required_points)) != len(d.required_points):
            checks.bad("declared/required_points", "lists a point more than once")


def _sample_rows(rows: object, name: str, checks: _Checks) -> list[Sample]:
    out: list[Sample] = []
    if rows is None or isinstance(rows, (str, bytes)):
        checks.bad(name, "must be a sequence of rows")
        return out
    for i, row in enumerate(rows):
        try:
            s = Sample(*row)
        except TypeError:
            checks.bad(f"{name}[{i}]", "a sample row has 7 fields")
            continue
        if not _is_number(s.t_s) or not isinstance(s.point, str) or s.valid not in (0, 1):
            checks.bad(f"{name}[{i}]", "needs a finite t_s, a point id and valid 0 or 1")
            continue
        if s.valid == 1 and not (_is_number(s.x_m) and _is_number(s.y_m) and _is_number(s.z_m)):
            checks.bad(f"{name}[{i}]", "a valid sample needs finite x_m, y_m and z_m")
            continue
        out.append(s)
    return out


def _command_rows(rows: object, checks: _Checks) -> list[Command]:
    out: list[Command] = []
    if rows is None or isinstance(rows, (str, bytes)):
        checks.bad("commands", "must be a sequence of rows")
        return out
    for i, row in enumerate(rows):
        try:
            c = Command(*row)
        except TypeError:
            checks.bad(f"commands[{i}]", "a command row has 7 fields")
            continue
        if not _is_number(c.t_sent_s) or not isinstance(c.strategy, str) or not c.strategy:
            checks.bad(f"commands[{i}]", "needs a finite t_sent_s and a strategy")
            continue
        if out and c.t_sent_s < out[-1].t_sent_s:
            checks.bad(f"commands[{i}]", "t_sent_s goes back in time; rows must be in send order")
            continue
        out.append(c)
    return out


def _accuracy_rows(rows: object, checks: _Checks) -> list[AccuracyRow]:
    out: list[AccuracyRow] = []
    if rows is None or isinstance(rows, (str, bytes)):
        checks.bad("accuracy", "must be a sequence of rows")
        return out
    seen: set[tuple[str, str]] = set()
    for i, row in enumerate(rows):
        try:
            r = AccuracyRow(*row)
        except TypeError:
            checks.bad(f"accuracy[{i}]", "an accuracy row has 9 fields")
            continue
        meas = (r.meas_x_m, r.meas_y_m, r.meas_z_m)
        if r.phase not in ("pre", "post", "jitter") or not isinstance(r.label, str):
            checks.bad(f"accuracy[{i}]", "needs a phase (pre, post, jitter) and a label")
            continue
        if not all(_is_number(v) for v in meas):
            checks.bad(f"accuracy[{i}]", "needs finite meas_x_m, meas_y_m, meas_z_m")
            continue
        if r.phase != "jitter":
            if not all(_is_number(v) for v in (r.ref_x_m, r.ref_y_m, r.ref_z_m)):
                checks.bad(
                    f"accuracy[{i}]", f"a {r.phase} row needs finite ref_x_m, ref_y_m, ref_z_m"
                )
                continue
            if (r.phase, r.label) in seen:
                checks.bad(f"accuracy[{i}]", f"{r.phase} label {r.label!r} appears twice")
                continue
            seen.add((r.phase, r.label))
        out.append(r)
    return out


def prepare(inp: ScoringInput, profile_keys: Mapping[str, str]) -> Prepared:
    """Check a ScoringInput against the scorer's contract; raise ScoringError listing every
    problem (at most 20)."""
    checks = _Checks()
    if not isinstance(inp, ScoringInput):
        raise TypeError(f"expected a ScoringInput, not {type(inp).__name__}")
    if not isinstance(inp.envelope, Envelope):
        checks.bad("envelope", "must be a loaded bounded_embodiment.envelope.Envelope")
        checks.done()
    profile = _check_profile(inp.profile, profile_keys, checks)
    _check_declared(inp.declared, inp.envelope, checks)
    samples = _sample_rows(inp.samples, "samples", checks)
    fk = None if inp.fk_samples is None else _sample_rows(inp.fk_samples, "fk_samples", checks)
    commands = _command_rows(inp.commands, checks)
    accuracy = _accuracy_rows(inp.accuracy, checks)
    checks.done()
    env = inp.envelope
    if inp.declared.required_points is None:
        # "default: every point that has zones"
        required = [pid for pid in env.points if env.zones_for(pid)]
    else:
        wanted = set(inp.declared.required_points)
        required = [pid for pid in env.points if pid in wanted]
    return Prepared(env, samples, commands, accuracy, fk, inp.declared, profile, required)


# --------------------------------------------------------------------------- flags


def nonstandard_params_flag(profile: Mapping[str, Any]) -> dict | None:
    """NONSTANDARD_PARAMS when any profile value differs from the packaged profile of the same
    name, compared as JCS bytes (so 2 and 2.0 are the same value). It never changes a result."""
    name = profile.get("profile")
    try:
        packaged = load_profile(name)
    except KeyError:
        return {
            "code": "NONSTANDARD_PARAMS",
            "detail": f"there is no packaged profile named {name!r}, so every value is nonstandard",
        }
    if canonicalize(dict(profile)) == canonicalize(packaged):
        return None
    differ = sorted(
        key
        for key in set(profile) | set(packaged)
        if key not in profile
        or key not in packaged
        or canonicalize(profile[key]) != canonicalize(packaged[key])
    )
    return {
        "code": "NONSTANDARD_PARAMS",
        "detail": f"differs from the packaged profile {name!r} in: {', '.join(differ)}",
    }


def common_flags(
    p: Prepared, undeclared_points: list[str], speed_unmeasured: list[str]
) -> list[dict]:
    """The flags both scorers raise, sorted by code. Flags never change a result.

    NONSTANDARD_PARAMS: see nonstandard_params_flag. REQUIRED_POINTS_OMIT_ZONED: the declared
    required_points leave out a point that has zones. SPEED_UNMEASURED: a required point has
    samples but no pair of them is 0.75·w to w apart, so its speed was never checked (an
    instrument at 6.7-10 Hz or 13.3-15 Hz never produces such a pair). UNDECLARED_POINT_SAMPLES:
    samples name a point the envelope does not declare (they are ignored)."""
    flags = []
    nonstandard = nonstandard_params_flag(p.profile)
    if nonstandard is not None:
        flags.append(nonstandard)
    zoned = [pid for pid in p.envelope.points if p.envelope.zones_for(pid)]
    omitted = [pid for pid in zoned if pid not in p.required]
    if omitted:
        flags.append(
            {
                "code": "REQUIRED_POINTS_OMIT_ZONED",
                "detail": "declared required_points leave out points that have zones, so their "
                f"gaps and coverage do not decide the verdict: {', '.join(omitted)}",
            }
        )
    if speed_unmeasured:
        flags.append(
            {
                "code": "SPEED_UNMEASURED",
                "detail": "no two samples of these required points are 0.75 to 1 speed window "
                f"apart, so their speed was never checked: {', '.join(speed_unmeasured)}",
            }
        )
    if undeclared_points:
        flags.append(
            {
                "code": "UNDECLARED_POINT_SAMPLES",
                "detail": "samples name points the envelope does not declare; they were "
                f"ignored: {', '.join(undeclared_points)}",
            }
        )
    flags.sort(key=lambda f: f["code"])
    return flags


# --------------------------------------------------------------------------- windows and samples


def run_window(commands: Sequence[Command], uncertainty_s: float) -> tuple[float, float] | None:
    """Rule: the window is [first t_sent − U, last t_sent + 1.0 + U] (U = clock uncertainty).
    None when there are no commands."""
    if not commands:
        return None
    first, last = commands[0].t_sent_s, commands[-1].t_sent_s
    return (first - uncertainty_s, last + WINDOW_TAIL_S + uncertainty_s)


Point3 = tuple[float, float, float]


def in_window_by_point(
    samples: Sequence[Sample], window: tuple[float, float] | None, envelope: Envelope
) -> tuple[dict[str, list[tuple[float, Point3]]], list[str]]:
    """Valid samples (valid == 1) inside the window (ends included), per envelope point, as
    [(t, (x, y, z))] sorted by t (a stable sort: equal times keep file order). Also the sorted
    point ids samples name that the envelope does not declare."""
    by_point: dict[str, list[tuple[float, Point3]]] = {pid: [] for pid in envelope.points}
    undeclared: set[str] = set()
    for s in samples:
        bucket = by_point.get(s.point)
        if bucket is None:
            undeclared.add(s.point)
            continue
        if s.valid != 1 or window is None or s.t_s < window[0] or s.t_s > window[1]:
            continue
        bucket.append((s.t_s, (s.x_m, s.y_m, s.z_m)))
    for bucket in by_point.values():
        bucket.sort(key=_time)
    return by_point, sorted(undeclared)


def _time(item: tuple[float, Point3]) -> float:
    return item[0]


# --------------------------------------------------------------------------- numeric helpers


def distance(a: Sequence[float], b: Sequence[float]) -> float:
    """Euclidean distance: sqrt of an explicit sum of squares, x then y then z."""
    dx, dy, dz = a[0] - b[0], a[1] - b[1], a[2] - b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def median(values: Sequence[float]) -> float:
    """The median by sorting: the middle value, or the mean of the two middle values."""
    if not values:
        raise ValueError("median of no values")
    s = sorted(values)
    m = len(s) // 2
    if len(s) % 2:
        return s[m]
    return (s[m - 1] + s[m]) / 2.0


def nearest_rank(sorted_values: Sequence[float], percent: int) -> float:
    """Nearest-rank percentile of ascending values: index ceil(percent/100 · n) − 1, with the
    ceiling taken exactly in integers."""
    n = len(sorted_values)
    if n == 0:
        raise ValueError("percentile of no values")
    rank = (percent * n + 99) // 100
    return sorted_values[rank - 1]


def jcs_safe(result: dict) -> dict:
    """Return result after checking that RFC 8785 can serialize it (no NaN, no infinity)."""
    try:
        canonicalize(result)
    except CanonicalError as e:
        raise ScoringError(
            [Finding("SCORING_OUTPUT_NOT_JSON", "error", f"the result is not JSON-safe: {e}")]
        ) from None
    return result
