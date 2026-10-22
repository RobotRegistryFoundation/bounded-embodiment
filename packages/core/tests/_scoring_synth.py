"""Synthetic runs for the scoring tests.

Everything here is deterministic on every platform and Python: paths are piecewise linear (no
sin/cos, whose last bits differ between C libraries), noise comes from an integer LCG, and float
totals are never formed with built-in ``sum`` (which changed in Python 3.12). So the golden
hashes in vectors/scoring/golden.json hold everywhere.

Geometry (metres, base frame): the tool is kept in TOOL_BOX, the elbow in LINK_BOX, and every
point is kept out of FIXTURE. The default tool path is a square loop of side 0.1 m around
CENTRE at 0.05 m/s, which stays at least 0.05 m from every boundary.
"""

from __future__ import annotations

import csv
import io
import math
from collections.abc import Callable, Sequence
from dataclasses import replace

from bounded_embodiment.envelope import Envelope, load
from bounded_embodiment.profiles import load_profile
from bounded_embodiment.rawdata import AccuracyRow, Command, Sample, StateRow
from bounded_embodiment.scoring import Declared, ScoringInput

STRATEGIES = (
    "uniform", "boundary", "max", "sign_flip", "drive_out", "fast", "partial", "malformed"
)
TOOL_BOX = ((0.20, -0.15, 0.05), (0.40, 0.15, 0.30))
LINK_BOX = ((-0.30, -0.40, 0.02), (0.50, 0.40, 0.60))
FIXTURE = ((0.34, 0.10, 0.05), (0.40, 0.15, 0.12))
CENTRE = (0.30, 0.0, 0.175)
ELBOW_CENTRE = (0.10, 0.0, 0.30)

Xyz = tuple[float, float, float]
Path = Callable[[float], Xyz]


# --------------------------------------------------------------------------- envelope


def envelope_doc(points: Sequence[str] = ("tool", "elbow"), tool_max_speed: float = 0.25,
                 motion_max_speed: float = 0.3, fixture: bool = True,
                 stop_sources: Sequence[dict] | None = None) -> dict:
    pts = []
    for pid in points:
        p = {"id": pid, "description": f"synthetic {pid} point"}
        if pid == "tool":
            p["max_speed_mps"] = tool_max_speed
        pts.append(p)
    zones = []
    if "tool" in points:
        zones.append({"id": "tool_box", "kind": "keep_in", "applies_to": ["tool"],
                      "box": {"min_m": list(TOOL_BOX[0]), "max_m": list(TOOL_BOX[1])}})
    links = [pid for pid in points if pid != "tool"]
    if links:
        zones.append({"id": "cell", "kind": "keep_in", "applies_to": links,
                      "box": {"min_m": list(LINK_BOX[0]), "max_m": list(LINK_BOX[1])}})
    if fixture:
        zones.append({"id": "fixture", "kind": "keep_out", "applies_to": ["*"],
                      "box": {"min_m": list(FIXTURE[0]), "max_m": list(FIXTURE[1])}})
    doc = {
        "envelope_version": "0.2",
        "machine": {"id": "synthetic-arm", "class": "arm"},
        "frame": "base_link",
        "points": pts,
        "workspace": {"zones": zones},
        "motion": {"max_speed_mps": motion_max_speed},
        "stop": {"category": 1, "max_time_ms": 300, "max_distance_m": 0.05},
        "heartbeat": {"model_timeout_ms": 500, "gate_timeout_ms": 200, "on_loss": "stop"},
    }
    if stop_sources is not None:
        doc["stop"]["sources"] = [dict(s) for s in stop_sources]
    return doc


def envelope(**kw) -> Envelope:
    return load(envelope_doc(**kw))


# --------------------------------------------------------------------------- noise and paths


class Lcg:
    """Numerical Recipes LCG. gauss() is Irwin-Hall (12 uniforms - 6), added up in integers."""

    def __init__(self, seed: int):
        self.state = seed & 0xFFFFFFFF

    def next(self) -> int:
        self.state = (1664525 * self.state + 1013904223) & 0xFFFFFFFF
        return self.state

    def gauss(self) -> float:
        total = 0
        for _ in range(12):
            total += self.next()
        return total / 4294967296.0 - 6.0


def polyline(corners: Sequence[Xyz], speed: float, closed: bool = True) -> Path:
    """Constant-speed motion along straight segments through corners (looping when closed)."""
    legs = []
    for k in range(len(corners) if closed else len(corners) - 1):
        a, b = corners[k], corners[(k + 1) % len(corners)]
        dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        legs.append((a, b, math.sqrt(dx * dx + dy * dy + dz * dz)))
    total = 0.0
    for leg in legs:
        total += leg[2]

    def at(t: float) -> Xyz:
        s = t * speed
        s = s % total if closed else min(max(s, 0.0), total)
        for a, b, length in legs:
            if s <= length:
                f = s / length
                return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f,
                        a[2] + (b[2] - a[2]) * f)
            s -= length
        return legs[-1][1]

    return at


def square_loop(half: float = 0.05, speed: float = 0.05, centre: Xyz = CENTRE) -> Path:
    cx, cy, cz = centre
    return polyline([(cx + half, cy - half, cz), (cx + half, cy + half, cz),
                     (cx - half, cy + half, cz), (cx - half, cy - half, cz)], speed)


def static(at: Xyz = CENTRE) -> Path:
    return lambda t: at


def out_and_back(x_peak: float, t_start: float = 4.0, speed: float = 0.05,
                 dwell: float = 0.5) -> Path:
    """Static at CENTRE except one trip along +x to x_peak at `speed`, a dwell, and back."""
    cx, cy, cz = CENTRE
    t1 = t_start + (x_peak - cx) / speed
    t2 = t1 + dwell
    t3 = t2 + (x_peak - cx) / speed

    def at(t: float) -> Xyz:
        if t <= t_start or t >= t3:
            x = cx
        elif t < t1:
            x = cx + (t - t_start) * speed
        elif t <= t2:
            x = x_peak
        else:
            x = x_peak - (t - t2) * speed
        return (x, cy, cz)

    return at


def elbow_of(tool: Xyz) -> Xyz:
    return (ELBOW_CENTRE[0] + 0.5 * (tool[0] - CENTRE[0]),
            ELBOW_CENTRE[1] + 0.5 * (tool[1] - CENTRE[1]),
            ELBOW_CENTRE[2] + 0.5 * (tool[2] - CENTRE[2]))


# --------------------------------------------------------------------------- rows


def sample_rows(path: Path, t0: float, t1: float, rate_hz: float = 30.0,
                points: Sequence[str] = ("tool", "elbow"), noise_m: float = 0.0, seed: int = 1,
                drop: Callable[[str, int, float], bool] | None = None) -> list[Sample]:
    """One row per point per frame from t0 to t1 (frames at t0 + k / rate_hz). A dropped frame
    is written as valid=0 with empty coordinates, as an instrument would."""
    rng = Lcg(seed)
    rows = []
    k = 0
    while True:
        t = t0 + k / rate_hz
        if t > t1:
            return rows
        tool = path(t)
        for pid in points:
            base = tool if pid == "tool" else elbow_of(tool)
            if drop is not None and drop(pid, k, t):
                rows.append(Sample(t, pid, None, None, None, 0, None))
                continue
            if noise_m:
                xyz = (base[0] + noise_m * rng.gauss(), base[1] + noise_m * rng.gauss(),
                       base[2] + noise_m * rng.gauss())
            else:
                xyz = base
            rows.append(Sample(t, pid, xyz[0], xyz[1], xyz[2], 1, 0.0005))
        k += 1


def command_rows(duration_s: float, strategies: Sequence[str] = STRATEGIES, t0: float = 0.0,
                 segment: int = 25, fast_segment: int = 50) -> list[Command]:
    """Cycle through the strategies: `segment` commands at 50 Hz each, `fast` bursts of
    `fast_segment` at 500 Hz. Times are whole ticks of 1/500 s from t0, up to t0 + duration_s."""
    rows, tick, seq, i = [], 0, 0, 0
    while True:
        strategy = strategies[i % len(strategies)]
        i += 1
        n, step = (fast_segment, 1) if strategy == "fast" else (segment, 10)
        for _ in range(n):
            t = t0 + tick / 500.0
            if tick / 500.0 > duration_s:
                return rows
            rows.append(Command(seq, t, t + 0.004, strategy, '{"q":[0,0,0]}', "ok", None))
            seq += 1
            tick += step


def waypoint_command_rows(ids: Sequence[str], dwell_s: float, cycles: int,
                          t0: float = 0.0) -> list[Command]:
    """Commands for EV-10: each waypoint in turn for dwell_s at 50 Hz, `cycles` times round."""
    rows, tick, seq = [], 0, 0
    per = round(dwell_s * 50)
    for _ in range(cycles):
        for wid in ids:
            for _ in range(per):
                t = t0 + tick / 500.0
                rows.append(Command(seq, t, t + 0.004, f"waypoint:{wid}", "{}", "ok", None))
                seq += 1
                tick += 10
    return rows


SURVEY = [(CENTRE[0] + sx * 0.05, CENTRE[1] + sy * 0.05, CENTRE[2] + sz * 0.05)
          for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]


def accuracy_rows(error_m: float = 0.002, labels: int = 8, drift_m: float = 0.001,
                  jitter_sigma_m: float | None = 0.001, jitter_labels: int = 2,
                  post: bool = True, pre: bool = True) -> list[AccuracyRow]:
    """pre: one row per surveyed point, error_m off along one axis; post: the pre measurement
    moved by drift_m; jitter: per label the exact pattern (-s, 0, +s) on every axis, so each
    label's sigma is jitter_sigma_m."""
    rows = []
    for i in range(labels if pre else 0):
        ref = SURVEY[i % len(SURVEY)]
        axis = i % 3
        sign = 1.0 if i % 2 == 0 else -1.0
        meas = list(ref)
        meas[axis] = ref[axis] + sign * error_m
        rows.append(AccuracyRow("pre", f"p{i + 1}", ref[0], ref[1], ref[2],
                                meas[0], meas[1], meas[2], 30))
        if post:
            rows.append(AccuracyRow("post", f"p{i + 1}", ref[0], ref[1], ref[2],
                                    meas[0], meas[1] + drift_m, meas[2], 30))
    if jitter_sigma_m is not None:
        for j in range(jitter_labels):
            base = SURVEY[j]
            for d in (-jitter_sigma_m, 0.0, jitter_sigma_m):
                rows.append(AccuracyRow("jitter", f"j{j + 1}", None, None, None,
                                        base[0] + d, base[1] + d, base[2] + d, 1))
    return rows


# --------------------------------------------------------------------------- inputs


def ev03_profile(**changes) -> dict:
    """The packaged EV-03 profile with min_duration_s 10 (so runs stay short), plus changes."""
    profile = load_profile("ev03-illustrative-0.1")
    profile["min_duration_s"] = 10
    profile.update(changes)
    return profile


def ev10_profile(**changes) -> dict:
    profile = load_profile("ev10-illustrative-0.1")
    profile["min_duration_s"] = 20
    profile.update(changes)
    return profile


def declared(**changes) -> Declared:
    base = Declared(
        primary_instrument_kind="optical_marker_tracker",
        primary_instrument_external=True,
        run_mode="hardware",
        accuracy_check_age_s=3600.0,
        clock_uncertainty_s=0.005,
        physical_max_tool_speed_mps=0.25,
        required_points=None,
    )
    return replace(base, **changes)


def ev03_input(duration_s: float = 12.0, path: Path | None = None, rate_hz: float = 30.0,
               points: Sequence[str] = ("tool", "elbow"), noise_m: float = 0.0,
               drop: Callable[[str, int, float], bool] | None = None,
               strategies: Sequence[str] = STRATEGIES, sampled_points: Sequence[str] | None = None,
               **changes) -> ScoringInput:
    """A clean run (PASS under ev03_profile) unless changes say otherwise. Samples run from 0.5 s
    before the first command to 1.5 s after the last."""
    commands = command_rows(duration_s, strategies)
    path = path or square_loop()
    samples = sample_rows(path, -0.5, commands[-1].t_sent_s + 1.5, rate_hz,
                          sampled_points or points, noise_m, drop=drop)
    inp = ScoringInput(
        envelope=envelope(points=points),
        samples=samples,
        commands=commands,
        accuracy=accuracy_rows(),
        declared=declared(),
        profile=ev03_profile(),
    )
    return replace(inp, **changes)


EV10_WAYPOINTS = [
    {"id": "a", "position_m": [0.25, -0.05, 0.15]},
    {"id": "b", "position_m": [0.35, -0.05, 0.20]},
    {"id": "c", "position_m": [0.35, 0.05, 0.15]},
    {"id": "d", "position_m": [0.25, 0.05, 0.20]},
]


def ev10_input(waypoints: Sequence[dict] = EV10_WAYPOINTS, dwell_s: float = 3.0, cycles: int = 2,
               freeze: bool = False, miss_z_m: float = 0.0, speed: float = 0.1,
               **changes) -> ScoringInput:
    """The tool goes to each waypoint in turn at `speed` and waits there, while 50 Hz commands
    name the waypoint; freeze=True keeps it at its start instead (a gate that refuses all).
    miss_z_m: the tool stops that far below each declared waypoint."""
    ids = [w["id"] for w in waypoints]
    commands = waypoint_command_rows(ids, dwell_s, cycles)
    start: Xyz = CENTRE
    targets = {w["id"]: (w["position_m"][0], w["position_m"][1], w["position_m"][2] - miss_z_m)
               for w in waypoints}
    legs: list[tuple[float, Xyz, Xyz]] = []  # (visit start, from, to)
    here = start
    t = 0.0
    for _ in range(cycles):
        for wid in ids:
            legs.append((t, here, targets[wid]))
            here = targets[wid]
            t += dwell_s

    def path(t: float) -> Xyz:
        if freeze:
            return start
        current = start
        for t_start, a, b in legs:
            if t < t_start:
                break
            dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
            length = math.sqrt(dx * dx + dy * dy + dz * dz)
            f = 1.0 if length == 0 else min(1.0, (t - t_start) * speed / length)
            current = (a[0] + dx * f, a[1] + dy * f, a[2] + dz * f)
        return current

    samples = sample_rows(path, -0.5, commands[-1].t_sent_s + 1.5, 30.0, ("tool", "elbow"))
    inp = ScoringInput(
        envelope=envelope(points=("tool", "elbow")),
        samples=samples,
        commands=commands,
        accuracy=accuracy_rows(),
        declared=declared(),
        profile=ev10_profile(),
        waypoints=[dict(w) for w in waypoints],
    )
    return replace(inp, **changes)


# --------------------------------------------------------------------------- CSV round trip


def to_csv(header: Sequence[str], rows: Sequence[tuple]) -> bytes:
    """CSV bytes with floats written by repr, which reads back to the same double."""
    out = io.StringIO(newline="")
    writer = csv.writer(out, lineterminator="\n")
    writer.writerow(header)
    for row in rows:
        writer.writerow(["" if v is None else repr(v) if isinstance(v, float) else v for v in row])
    return out.getvalue().encode("utf-8")


def fk_state_rows(fk: Sequence[Sample]) -> list[StateRow]:
    """be-state-csv/1 rows (long format) carrying fk samples: per sample an encoder row, then
    the fk rows <point>.x, <point>.y and <point>.z (metres, envelope frame)."""
    rows = []
    for s in fk:
        rows.append(StateRow(s.t_s, "encoder", "elbow_flex", 0.0))
        rows.append(StateRow(s.t_s, "fk", f"{s.point}.x", s.x_m))
        rows.append(StateRow(s.t_s, "fk", f"{s.point}.y", s.y_m))
        rows.append(StateRow(s.t_s, "fk", f"{s.point}.z", s.z_m))
    return rows


# --------------------------------------------------------------------------- EV-02 and EV-05

#: The hardwired stop declared tighter than the top-level stop block (which is the worst case).
STOP_SOURCES = (
    {"source": "stop_message", "category": 1, "max_time_ms": 300, "max_distance_m": 0.05},
    {"source": "hardwired_stop", "category": 0, "max_time_ms": 250, "max_distance_m": 0.04},
)
STOP_START: Xyz = CENTRE
DIRECTIONS = {"x_plus": (1.0, 0.0, 0.0), "x_minus": (-1.0, 0.0, 0.0)}
EV02_TRIALS = (
    ("stop_message", "x_plus"), ("stop_message", "x_minus"), ("stop_message", "x_plus"),
    ("hardwired_stop", "x_minus"), ("hardwired_stop", "x_plus"), ("hardwired_stop", "x_minus"),
)
SPEED = 0.25  # the tool's speed limit in envelope_doc()


def ev02_profile(**changes) -> dict:
    profile = load_profile("ev02-illustrative-0.1")
    profile.update(changes)
    return profile


def ev05_profile(**changes) -> dict:
    profile = load_profile("ev05-illustrative-0.1")
    profile.update(changes)
    return profile


def _along(a: Xyz, u: Xyz, s: float) -> Xyz:
    return (a[0] + u[0] * s, a[1] + u[1] * s, a[2] + u[2] * s)


def _braking(v: float, run_s: float, brake_s: float, tau: float) -> float:
    """Distance after tau: v for run_s, then uniform braking to rest over brake_s."""
    if tau <= 0.0:
        return 0.0
    if tau <= run_s:
        return v * tau
    if tau <= run_s + brake_s:
        b = tau - run_s
        return v * run_s + v * b - 0.5 * (v / brake_s) * b * b
    return v * run_s + 0.5 * v * brake_s


def ev02_input(trials: Sequence[tuple[str, str]] = EV02_TRIALS, reaction_s: float = 0.02,
               brake_s: float = 0.15, slow: dict[int, float] | None = None,
               creep: dict[int, float] | None = None, segment_s: float = 4.0,
               rate_hz: float = 30.0, noise_m: float = 0.0, sources: Sequence[str] | None = None,
               drop: Callable[[str, int, float], bool] | None = None, **changes) -> ScoringInput:
    """An EV-02 run: per trial (every segment_s) a walk back to the start (25 return rows over
    0.5 s, not before the first trial), 15 drive rows at 50 Hz while the tool moves from the start
    at SPEED along the drive, and the stop row 1/50 s after the last; the tool runs on for
    reaction_s and brakes uniformly over brake_s (``slow`` maps a trial to a longer one), then
    rests (``creep`` maps a trial to a drift speed after the rest)."""
    rows: list[Command] = []
    legs = []  # (t_return or None, t_drive, t_stop, direction, brake_s, creep_mps)
    seq = 0
    for k, (source, drive) in enumerate(trials):
        t_seg = k * segment_s
        t_return = None
        if k:
            t_return = t_seg
            for i in range(25):
                t = t_seg + i / 50.0
                rows.append(Command(seq, t, t + 0.004, "return", "{}", "ok", None))
                seq += 1
        t_drive = t_seg + (0.5 if k else 0.0)
        for i in range(15):
            t = t_drive + i / 50.0
            rows.append(Command(seq, t, t + 0.004, f"drive:{drive}", "{}", "ok", None))
            seq += 1
        t_stop = t_drive + 15 / 50.0
        rows.append(Command(seq, t_stop, t_stop + 0.004, f"stop:{source}", "{}", "triggered", None))
        seq += 1
        legs.append((t_return, t_drive, t_stop, DIRECTIONS[drive],
                     (slow or {}).get(k, brake_s), (creep or {}).get(k, 0.0)))

    def leg_at(leg: tuple, t: float) -> Xyz:
        _, t_drive, t_stop, u, b, drift = leg
        run_s = t_stop - t_drive + reaction_s
        s = _braking(SPEED, run_s, b, t - t_drive)
        t_rest = t_drive + run_s + b
        if drift and t > t_rest:
            s += drift * (t - t_rest)
        return _along(STOP_START, u, s)

    def path(t: float) -> Xyz:
        k = -1
        for i, leg in enumerate(legs):
            if (leg[1] if leg[0] is None else leg[0]) <= t:
                k = i
        if k < 0:
            return STOP_START
        t_return, t_drive = legs[k][0], legs[k][1]
        if t_return is not None and t < t_drive:  # walking back to the start
            a = leg_at(legs[k - 1], t_return)
            f = (t - t_return) / (t_drive - t_return)
            return (a[0] + (STOP_START[0] - a[0]) * f, a[1] + (STOP_START[1] - a[1]) * f,
                    a[2] + (STOP_START[2] - a[2]) * f)
        return leg_at(legs[k], t)

    t_end = rows[-1].t_sent_s + 3.0
    samples = sample_rows(path, -0.5, t_end, rate_hz, ("tool",), noise_m, drop=drop)
    inp = ScoringInput(
        envelope=envelope(points=("tool", "elbow"), stop_sources=STOP_SOURCES),
        samples=samples,
        commands=rows,
        accuracy=accuracy_rows(),
        declared=declared(),
        profile=ev02_profile(),
        plan={"point": "tool",
              "sources": list(sources or dict.fromkeys(src for src, _ in trials))},
    )
    return replace(inp, **changes)


LOSS_A: Xyz = (0.30, 0.10, 0.175)
LOSS_B: Xyz = (0.30, -0.10, 0.175)


def _shuttle(s: float) -> Xyz:
    """Back and forth between LOSS_A and LOSS_B: the point s metres along the shuttle."""
    length = LOSS_A[1] - LOSS_B[1]
    s = s % (2.0 * length)
    if s <= length:
        return (LOSS_A[0], LOSS_A[1] - s, LOSS_A[2])
    return (LOSS_A[0], LOSS_B[1] + (s - length), LOSS_A[2])


def ev05_input(motion_s: Sequence[float] = (1.2, 1.3, 1.26), run_on_s: float = 0.3,
               brake_s: float = 0.1, late: dict[int, float] | None = None,
               resumes: Sequence[int] = (), silence_s: float = 4.0, restart_s: float = 2.0,
               pause_s: float = 3.0, resume: dict | None = None, resume_after_s: float = 0.0,
               rate_hz: float = 30.0, noise_m: float = 0.0, **changes) -> ScoringInput:
    """An EV-05 run: per trial, motion rows at 50 Hz for motion_s[k] while the tool shuttles at
    SPEED (the default lengths put every loss mid-leg, away from a turn), then silence_s without
    a command, then restart rows at 50 Hz for restart_s, then
    pause_s before the next trial. After the last motion row the tool runs on for run_on_s
    (``late`` maps a trial to a longer one) and brakes over brake_s; trials in ``resumes`` follow
    the restart commands resume_after_s after the first (shuttling on at SPEED while they
    last)."""
    rows: list[Command] = []
    legs = []  # (t0, t_loss, t_restart, t_restart_end, run_on, s0)
    seq, t, s_walk = 0, 0.0, 0.0
    for k, m in enumerate(motion_s):
        t0 = t
        n = round(m * 50)
        for i in range(n):
            ts = t0 + i / 50.0
            rows.append(Command(seq, ts, ts + 0.004, "motion", "{}", "ok", None))
            seq += 1
        t_loss = t0 + (n - 1) / 50.0
        t_restart = t_loss + silence_s
        r = round(restart_s * 50)
        for j in range(r):
            ts = t_restart + j / 50.0
            rows.append(Command(seq, ts, ts + 0.004, "restart", "{}", "ok", None))
            seq += 1
        run_on = (late or {}).get(k, run_on_s)
        legs.append((t0, t_loss, t_restart, t_restart + r / 50.0, run_on, s_walk, k in resumes))
        s_walk += _braking(SPEED, t_loss - t0 + run_on, brake_s, 1e9)
        if k in resumes:
            s_walk += SPEED * max(0.0, r / 50.0 - resume_after_s)
        t = t_restart + r / 50.0 + pause_s

    def path(tt: float) -> Xyz:
        here = _shuttle(0.0)
        for t0, t_loss, t_restart, t_restart_end, run_on, s0, follows in legs:
            if tt < t0:
                return here
            s = s0 + _braking(SPEED, t_loss - t0 + run_on, brake_s, tt - t0)
            if follows and tt > t_restart + resume_after_s:
                s += SPEED * (min(tt, t_restart_end) - (t_restart + resume_after_s))
            here = _shuttle(s)
        return here

    samples = sample_rows(path, -0.5, rows[-1].t_sent_s + 3.0, rate_hz, ("tool",), noise_m)
    inp = ScoringInput(
        envelope=envelope(points=("tool", "elbow")),
        samples=samples,
        commands=rows,
        accuracy=accuracy_rows(),
        declared=declared(),
        profile=ev05_profile(),
        plan={"point": "tool", "source": "heartbeat_loss",
              "resume": dict(resume or {"rule": "manual"})},
    )
    return replace(inp, **changes)
