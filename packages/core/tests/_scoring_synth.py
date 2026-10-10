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
                 motion_max_speed: float = 0.3, fixture: bool = True) -> dict:
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
    return {
        "envelope_version": "0.2",
        "machine": {"id": "synthetic-arm", "class": "arm"},
        "frame": "base_link",
        "points": pts,
        "workspace": {"zones": zones},
        "motion": {"max_speed_mps": motion_max_speed},
        "stop": {"category": 1, "max_time_ms": 300, "max_distance_m": 0.05},
        "heartbeat": {"model_timeout_ms": 500, "gate_timeout_ms": 200, "on_loss": "stop"},
    }


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
