# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""DRAFT — requires line-by-line human review before use on hardware.

The watchdog: two independent tripwires, the stop latch, and what happens when a stop is not
acknowledged.

Threads. The watchdog runs on its own thread and NEVER touches the robot's bus. When a tripwire
trips it sets the stop latch (``StopLatch.trip``), which also calls ``adapter.latch_stop()`` (a flag,
no bus). The command thread sees the latch before its next command, stops sending, and executes
the stop under the bus lock. The watchdog then judges the stop from the instrument: acknowledged
if the tool is slower than 20 mm/s within 0.25 s of the latch; otherwise it runs the lab's
``--estop-cmd`` and sounds the CUT POWER alarm. That path needs nothing from the command thread,
so it also covers a command thread stuck inside a gate.

Tripwires, checked every watchdog tick (limits.py has the numbers):

1. External instrument: a zoned point outside by more than the mode's margin; tool speed above
   3 x its envelope limit; no datagram for 0.5 s; no valid tool sample for 1 s (0.2 s without the
   FK tripwire); camera_moved.
2. FK from the robot's own state (read by the command thread via ``adapter.read_state()``): a
   zoned point outside by more than the same margin; a joint faster than the adapter's declared
   joint-speed limit (SO-101: 150 deg/s); the newest state older than 0.25 s, or FK failing.

The two share no data: either one alone stops the run. Every check is a pure function of its
inputs (below the ``Watchdog`` class's helpers), so tests drive them without threads.

What the watchdog does not do: score, judge the system under test, or touch the bus.
"""
from __future__ import annotations

import math
import shlex
import subprocess
import sys
import threading
from collections import deque
from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass, field

from .feeds import FeedSnapshot
from .limits import DEFAULT, Limits

__all__ = [
    "Alarm",
    "Escalation",
    "StateBox",
    "StateSnapshot",
    "StopLatch",
    "StopOutcome",
    "TripwireConfig",
    "Watchdog",
    "clearance",
    "fk_trip",
    "inside_and_still",
    "instrument_trip",
    "stop_speed",
    "window_speed",
]

HISTORY_S = 3.0  # how much instrument and state history the watchdog keeps (> the 2 s still window)


# ----- the latch -------------------------------------------------------------------------------
class StopLatch:
    """Set once; the first reason wins. ``trip`` may be called from any thread and never blocks
    on the bus: the listeners it calls (``adapter.latch_stop``) only set flags."""

    def __init__(self, clock: Callable[[], float]):
        self.clock = clock
        self._event = threading.Event()
        self._lock = threading.Lock()
        self.kind: str | None = None  # "tripwire" | "operator" | "error" | "finished"
        self.reason: str | None = None
        self.t_s: float | None = None
        self._listeners: list[Callable[[], None]] = []

    def on_trip(self, fn: Callable[[], None]) -> None:
        self._listeners.append(fn)

    def trip(self, kind: str, reason: str) -> bool:
        """Set the latch; return False if it was already set (the first reason is kept)."""
        with self._lock:
            if self._event.is_set():
                return False
            self.kind, self.reason, self.t_s = kind, reason, self.clock()
            self._event.set()
        for fn in self._listeners:
            try:
                fn()
            except Exception as e:  # noqa: BLE001 - a broken listener must not unset the stop
                print(f"latch listener failed: {type(e).__name__}: {e}", file=sys.stderr)
        return True

    @property
    def tripped(self) -> bool:
        return self._event.is_set()

    def wait(self, timeout: float) -> bool:
        return self._event.wait(timeout)

    def as_dict(self) -> dict:
        return {"kind": self.kind, "reason": self.reason, "t_s": self.t_s}


# ----- robot state, as the command thread publishes it -------------------------------------------
@dataclass(frozen=True)
class StateSnapshot:
    """One ``read_state()`` on the run's timebase. ``fk`` is None when FK could not be computed
    (``error`` says why) or there are no kinematics."""

    t_s: float
    joints: Mapping[str, float]
    units: Mapping[str, str]
    fk: Mapping[str, tuple] | None = None
    error: str | None = None


class StateBox:
    """The recent state snapshots: written by the command thread, read by the watchdog."""

    def __init__(self, keep_s: float = HISTORY_S):
        self._lock = threading.Lock()
        self._items: deque[StateSnapshot] = deque()
        self.keep_s = keep_s

    def put(self, snap: StateSnapshot) -> None:
        with self._lock:
            self._items.append(snap)
            while self._items and snap.t_s - self._items[0].t_s > self.keep_s:
                self._items.popleft()

    def recent(self) -> list[StateSnapshot]:
        with self._lock:
            return list(self._items)


# ----- configuration ---------------------------------------------------------------------------
@dataclass(frozen=True)
class TripwireConfig:
    mode: str
    margin_m: float
    keep_outs_only: bool  # goto-start: the arm may start outside its keep-in zones
    tool_point: str
    tool_speed_trip_mps: float
    feed: bool  # an instrument is connected
    feed_silent_s: float
    tool_unseen_s: float
    fk: bool  # the FK tripwire is on
    state_stale_s: float
    joint_speed_limits: Mapping[str, float] = field(default_factory=dict)  # robot units per second
    speed_window_s: float = 0.2

    @classmethod
    def for_mode(cls, mode: str, envelope, tool_point: str, *, feed: bool, fk: bool,
                 joint_speed_limits: Mapping[str, float], limits: Limits = DEFAULT) -> TripwireConfig:
        return cls(mode=mode, margin_m=limits.margin(mode), keep_outs_only=(mode == "goto-start"),
                   tool_point=tool_point,
                   tool_speed_trip_mps=limits.tool_speed_factor * envelope.speed_limit_mps(tool_point),
                   feed=feed, feed_silent_s=limits.feed_silent_s,
                   tool_unseen_s=limits.tool_unseen_s if fk else limits.tool_unseen_no_fk_s,
                   fk=fk, state_stale_s=limits.state_stale_s,
                   joint_speed_limits=dict(joint_speed_limits), speed_window_s=limits.speed_window_s)


# ----- pure checks -----------------------------------------------------------------------------
def clearance(envelope, point: str, xyz: Sequence[float], keep_outs_only: bool = False) -> float:
    """Signed metres (positive: compliant), as ``Envelope.clearance``; with ``keep_outs_only``,
    over the point's keep-out zones alone. math.inf when no zone applies."""
    if not keep_outs_only:
        return envelope.clearance(point, xyz)
    best = math.inf
    for z in envelope.zones_for(point):
        if z.kind == "keep_out":
            best = min(best, z.clearance(xyz))
    return best


def _dist(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def window_speed(samples: Sequence[tuple[float, tuple]], window_s: float) -> float | None:
    """Speed (m/s) of the newest sample against the earliest one within ``window_s`` of it, only
    over a span of at least 0.5 * window_s (the scorer's rule, ev03-tool-point/1: any instrument
    faster than 5 Hz yields such a span); None if there is no such span."""
    if len(samples) < 2:
        return None
    t_n, p_n = samples[-1]
    for t_j, p_j in samples:
        if t_n - t_j <= window_s:
            span = t_n - t_j
            if span >= 0.5 * window_s:
                return _dist(p_n, p_j) / span
            return None
    return None


def stop_speed(samples: Sequence[tuple[float, tuple]], t_from: float, min_span_s: float,
               max_span_s: float) -> float | None:
    """The speed that acknowledges a stop: newest sample against the earliest one captured after
    ``t_from`` (the first executed stop) and within ``max_span_s`` of the newest; None until that
    span reaches ``min_span_s``. Earlier samples never count, so motion from before the stop took
    effect is not held against it."""
    after = [(t, p) for t, p in samples if t > t_from]
    if len(after) < 2:
        return None
    t_n, p_n = after[-1]
    for t_j, p_j in after:
        if t_n - t_j <= max_span_s:
            span = t_n - t_j
            return _dist(p_n, p_j) / span if span >= min_span_s else None
    return None


def instrument_trip(snap: FeedSnapshot, tool_history: Sequence[tuple[float, tuple]], now: float,
                    cfg: TripwireConfig, envelope) -> str | None:
    """Tripwire 1, the external instrument. None means no trip."""
    if snap.camera_moved:
        return "the instrument reports camera_moved"
    heard = snap.last_arrival_s if snap.last_arrival_s is not None else snap.started_s
    if now - heard > cfg.feed_silent_s:
        return f"instrument feed silent for {now - heard:.2f} s (limit {cfg.feed_silent_s} s)"
    tool = snap.latest_valid.get(cfg.tool_point)
    seen = tool.t_s if tool is not None else snap.started_s
    if now - seen > cfg.tool_unseen_s:
        return f"tool point {cfg.tool_point!r} unseen for {now - seen:.2f} s (limit {cfg.tool_unseen_s} s)"
    for point, s in snap.latest_valid.items():
        if point not in envelope.points:
            continue
        c = clearance(envelope, point, s.xyz, cfg.keep_outs_only)
        if c < -cfg.margin_m:
            return (f"{point} (instrument) {-c * 100:.1f} cm outside "
                    f"(limit {cfg.margin_m * 100:.1f} cm, {cfg.mode})")
    v = window_speed(tool_history, cfg.speed_window_s)
    if v is not None and v > cfg.tool_speed_trip_mps:
        return f"tool speed (instrument) {v:.2f} m/s (limit {cfg.tool_speed_trip_mps:.2f} m/s)"
    return None


def fk_trip(states: Sequence[StateSnapshot], now: float, cfg: TripwireConfig, envelope) -> str | None:
    """Tripwire 2, forward kinematics of the robot's own joint state. None means no trip."""
    if not states:
        return "no robot state has been read"
    newest = states[-1]
    if now - newest.t_s > cfg.state_stale_s:
        return f"robot state is {now - newest.t_s:.2f} s old (limit {cfg.state_stale_s} s)"
    if newest.error is not None:
        return f"robot state unusable: {newest.error}"
    if newest.fk is None:
        return "FK unavailable for the newest robot state"
    for point, xyz in newest.fk.items():
        c = clearance(envelope, point, xyz, cfg.keep_outs_only)
        if c < -cfg.margin_m:
            return (f"{point} (FK) {-c * 100:.1f} cm outside "
                    f"(limit {cfg.margin_m * 100:.1f} cm, {cfg.mode})")
    if cfg.joint_speed_limits:
        # hw.py's rule: over the window, once at least half of it has elapsed.
        old = next((s for s in states if newest.t_s - s.t_s <= cfg.speed_window_s), newest)
        span = newest.t_s - old.t_s
        if span >= 0.5 * cfg.speed_window_s:
            for joint, limit in cfg.joint_speed_limits.items():
                if joint in newest.joints and joint in old.joints:
                    rate = abs(newest.joints[joint] - old.joints[joint]) / span
                    if rate > limit:
                        return f"{joint} moved at {rate:.0f}/s over {span:.2f} s (limit {limit:.0f}/s)"
    return None


def inside_and_still(histories: Mapping[str, Sequence[tuple[float, tuple]]], envelope, tool_point: str,
                     now: float, window_s: float, still_m: float, fresh_s: float) -> tuple[bool, str]:
    """Whether a run may start: over the last ``window_s`` of samples, the instrument saw the tool
    point within ``still_m`` of its per-axis median, and every sample of every zoned point it saw
    inside its zones. Returns (ok, why)."""
    tool = list(histories.get(tool_point, ()))
    if not tool or now - tool[-1][0] > fresh_s:
        return False, f"no recent sample of {tool_point!r}"
    t_n = tool[-1][0]
    window = [(t, p) for t, p in tool if t >= t_n - window_s]
    if window[-1][0] - window[0][0] < window_s - 0.1:
        return False, f"less than {window_s:.0f} s of {tool_point!r} samples"
    med = [sorted(p[i] for _, p in window)[len(window) // 2] for i in range(3)]
    worst = max(_dist(p, med) for _, p in window)
    if worst >= still_m:
        return False, f"{tool_point!r} not still: {worst * 1000:.1f} mm from its median (limit {still_m * 1000:.0f} mm)"
    for point, hist in histories.items():
        if point not in envelope.points:
            continue
        for t, p in hist:
            if t >= t_n - window_s and envelope.clearance(point, p) < 0:
                return False, f"{point!r} outside its zones"
    return True, f"inside, and {tool_point!r} within {worst * 1000:.1f} mm for {window_s:.0f} s"


# ----- escalation ------------------------------------------------------------------------------
class Alarm:
    """The CUT POWER alarm: a banner and the terminal bell on stderr, repeated until ``stop()``."""

    def __init__(self, stream=None, repeat_s: float = 2.0):
        self.stream = stream if stream is not None else sys.stderr
        self.repeat_s = repeat_s
        self.reason: str | None = None
        self._quit = threading.Event()
        self._thread: threading.Thread | None = None

    def _say(self, text: str) -> None:
        try:
            self.stream.write(text)
            self.stream.flush()
        except (OSError, ValueError):  # a closed stream must not kill the alarm thread
            return

    def sound(self, reason: str) -> None:
        self.reason = reason
        bar = "*" * 72
        self._say(f"\a\a\a\n{bar}\n*** CUT POWER NOW. The stop was not acknowledged: {reason}\n{bar}\n")
        self._thread = threading.Thread(target=self._repeat, name="be-alarm", daemon=True)
        self._thread.start()

    def _repeat(self) -> None:
        while not self._quit.wait(self.repeat_s):
            self._say("\a*** CUT POWER NOW ***\n")

    def stop(self) -> None:
        self._quit.set()


class Escalation:
    """What an unacknowledged stop triggers: the alarm first (instant), then the lab's
    ``--estop-cmd`` (a relay or an e-stop service), run without a shell, with a timeout."""

    def __init__(self, estop_cmd: str | None, alarm: Alarm, timeout_s: float = DEFAULT.estop_timeout_s,
                 run: Callable = subprocess.run):
        self.estop_cmd, self.alarm, self.timeout_s, self.run = estop_cmd, alarm, timeout_s, run
        self.fired: dict | None = None

    def fire(self, reason: str) -> dict:
        self.alarm.sound(reason)
        result = {"reason": reason, "estop_cmd": self.estop_cmd, "returncode": None, "error": None}
        if self.estop_cmd:
            try:
                r = self.run(shlex.split(self.estop_cmd), timeout=self.timeout_s, capture_output=True,
                             check=False)
                result["returncode"] = r.returncode
            except Exception as e:  # noqa: BLE001 - recorded; the alarm is already sounding
                result["error"] = f"{type(e).__name__}: {e}"
        self.fired = result
        return result


@dataclass
class StopOutcome:
    decided: bool = False
    acknowledged: bool | None = None
    t_s: float | None = None
    speed_mps: float | None = None
    source: str = ""
    escalation: dict | None = None

    def as_dict(self) -> dict:
        return {"acknowledged": self.acknowledged, "t_s": self.t_s, "speed_mps": self.speed_mps,
                "source": self.source, "escalation": self.escalation}


# ----- the thread ------------------------------------------------------------------------------
class Watchdog(threading.Thread):
    """Evaluates both tripwires every tick and judges the stop after a latch. Never touches the bus."""

    def __init__(self, *, latch: StopLatch, cfg: TripwireConfig, envelope, feed, state_box: StateBox,
                 escalation: Escalation, clock: Callable[[], float], limits: Limits = DEFAULT,
                 log: Callable[[str], None] = print):
        super().__init__(name="be-watchdog", daemon=True)
        self.latch, self.cfg, self.envelope, self.feed = latch, cfg, envelope, feed
        self.state_box, self.escalation, self.clock = state_box, escalation, clock
        self.limits, self.log = limits, log
        self.outcome = StopOutcome()
        # When the command thread first executed the stop (runner time), or None. Set by the
        # runner to the command thread's own record; it carries a time, never a bus handle.
        self.stop_executed_at: Callable[[], float | None] = lambda: None
        self._lock = threading.Lock()
        self._hist: dict[str, deque] = {}
        self._quit = threading.Event()
        self.ticks = 0

    # The history of valid instrument samples, per point, from successive snapshots.
    def _absorb(self, snap: FeedSnapshot) -> None:
        with self._lock:
            for point, s in snap.latest_valid.items():
                h = self._hist.setdefault(point, deque())
                if not h or s.t_s > h[-1][0]:
                    h.append((s.t_s, s.xyz))
                while h and s.t_s - h[0][0] > HISTORY_S:
                    h.popleft()

    def history(self, point: str) -> list[tuple[float, tuple]]:
        with self._lock:
            return list(self._hist.get(point, ()))

    def start_check(self) -> tuple[bool, str]:
        """For the runner: may the run start (inside and still)?"""
        with self._lock:
            hist = {p: list(h) for p, h in self._hist.items()}
        lim = self.limits
        return inside_and_still(hist, self.envelope, self.cfg.tool_point, self.clock(),
                                lim.still_window_s, lim.still_m, lim.feed_silent_s)

    def run(self) -> None:
        while not self._quit.is_set():
            try:
                self.tick()
            except Exception as e:  # noqa: BLE001 - a watchdog fault stops the run; it never ends the watchdog
                self.latch.trip("error", f"watchdog fault: {type(e).__name__}: {e}")
            self._quit.wait(self.limits.watchdog_period_s)

    def tick(self) -> None:
        self.ticks += 1
        now = self.clock()
        snap = self.feed.snapshot() if self.feed is not None else None
        if snap is not None:
            self._absorb(snap)
        states = self.state_box.recent()
        if not self.latch.tripped:
            reason = None
            if snap is not None:
                reason = instrument_trip(snap, self.history(self.cfg.tool_point), now, self.cfg,
                                         self.envelope)
            if reason is None and self.cfg.fk:
                reason = fk_trip(states, now, self.cfg, self.envelope)
            if reason is not None and self.latch.trip("tripwire", reason):
                self.log(f"*** TRIP: {reason}")
        if self.latch.tripped and not self.outcome.decided:
            self._judge_stop(now, states)

    def _stop_samples(self, states: Iterable[StateSnapshot]) -> tuple[list, str]:
        if self.feed is not None:
            return self.history(self.cfg.tool_point), "instrument"
        # No instrument (goto-start without one): the robot's own FK is all there is.
        pts = [(s.t_s, s.fk[self.cfg.tool_point]) for s in states
               if s.fk is not None and self.cfg.tool_point in s.fk]
        return pts, "fk (no instrument)"

    def _judge_stop(self, now: float, states: list[StateSnapshot]) -> None:
        """Acknowledged if, by latch + stop_ack_within_s, the tool's speed over samples captured
        after the FIRST EXECUTED stop is below stop_ack_speed_mps. Measuring from the executed
        stop, not from the latch, keeps the few milliseconds of motion before the command thread
        acted out of the judgement; a stop that was never executed (command thread stuck in the
        system under test) cannot be acknowledged at all, and escalates."""
        lim, t_latch = self.limits, self.latch.t_s
        t_stop = self.stop_executed_at()
        v, source = None, ""
        if t_stop is not None:
            samples, source = self._stop_samples(states)
            v = stop_speed(samples, t_stop, lim.stop_ack_min_span_s, lim.speed_window_s)
        if now - t_latch <= lim.stop_ack_within_s:
            if v is not None and v < lim.stop_ack_speed_mps:
                self.outcome = StopOutcome(True, True, now, v, source)
                self.log(f"Stop acknowledged by the {source}: tool at {v * 1000:.0f} mm/s "
                         f"{now - t_latch:.2f} s after the latch.")
            return
        if t_stop is None:
            why = "the stop was never executed (is the command thread stuck?)"
        elif v is None:
            why = "no usable samples after the stop"
        else:
            why = f"tool still at {v * 1000:.0f} mm/s"
        why += f" {lim.stop_ack_within_s} s after the latch ({self.latch.reason})"
        self.outcome = StopOutcome(True, False, now, v, source)
        self.outcome.escalation = self.escalation.fire(why)

    def decided(self) -> bool:
        return self.outcome.decided

    def stop(self) -> None:
        self._quit.set()
