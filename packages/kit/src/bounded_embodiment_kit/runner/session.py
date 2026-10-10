# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""DRAFT — requires line-by-line human review before use on hardware.

One session: the command thread, the raw-data files, and the order in which a run starts and ends.

Order (``run_session``), after the caller has opened the adapter and checked the interlocks:

1. start the instrument feed and wait until it has sent something (else: refuse, nothing moved);
2. start the command thread, which reads the robot's state every 20 ms; wait for the first state;
3. start the watchdog (both tripwires live from here on);
4. ask for the typed word (RUN / GO); anything else cancels (nothing sent, no stop needed);
5. if asked, wait until the instrument sees the arm inside and still (else cancel);
6. release the command thread: it paces the proposals by their ``t_s`` through ``adapter.send``;
7. the stream ends, a tripwire trips, Ctrl+C, or an error: the latch is set and the command thread
   executes the stop (same goals every time) until the watchdog has acknowledged or escalated;
8. the caller stops the watchdog (it has judged the stop), closes the adapter (typed OFF), then
   the feed.

Threads: the main thread (prompts, waits), ``be-command`` (the only thread that calls the adapter
while the run is live) and ``be-watchdog`` (never calls the adapter except ``latch_stop`` through
the latch). One timebase: ``clock()`` is ``time.monotonic()`` minus its value at run start.
"""
from __future__ import annotations

import csv
import math
import signal
import threading
import time
from collections.abc import Callable, Iterable
from dataclasses import dataclass, field
from pathlib import Path

from bounded_embodiment.generator import Proposal, encode_payload

from ..adapters.base import SendResult
from .interlocks import confirm
from .kin import ArmFK, FKError
from .limits import DEFAULT, Limits
from .watchdog import StateBox, StateSnapshot, StopLatch, Watchdog

__all__ = ["COMMANDS_HEADER", "STATE_HEADER", "CommandThread", "RawFiles", "SessionResult",
           "make_clock", "run_session"]

COMMANDS_HEADER = ("seq", "t_sent_s", "t_done_s", "strategy", "payload_json", "result", "error_kind")
STATE_HEADER = ("t_s", "source", "joint", "value")
FLUSH_EVERY_S = 1.0


def make_clock() -> Callable[[], float]:
    """The run's timebase: seconds since this call, from time.monotonic()."""
    t0 = time.monotonic()
    return lambda: time.monotonic() - t0


def _num(value: float) -> str | None:
    """A cell the strict raw-data readers accept: a finite decimal; None for anything else."""
    v = float(value)
    return repr(v) if math.isfinite(v) else None


class RawFiles:
    """raw/commands.csv (be-commands-csv/1) and raw/state.csv (be-state-csv/1). Written only by the
    command thread; flushed every second so a crash loses at most a second."""

    def __init__(self, raw_dir: Path):
        raw_dir.mkdir(parents=True, exist_ok=True)
        self._cmd_fh = (raw_dir / "commands.csv").open("w", newline="", encoding="utf-8")
        self._state_fh = (raw_dir / "state.csv").open("w", newline="", encoding="utf-8")
        self._cmd = csv.writer(self._cmd_fh, lineterminator="\n")
        self._state = csv.writer(self._state_fh, lineterminator="\n")
        self._cmd.writerow(COMMANDS_HEADER)
        self._state.writerow(STATE_HEADER)
        self._last_flush = time.monotonic()

    def command(self, p: Proposal, t_sent: float, t_done: float, r: SendResult) -> None:
        result = ("applied" if r.applied else "not_applied") + (f"/{r.decision}" if r.decision else "")
        self._cmd.writerow([p.seq, f"{t_sent:.6f}", f"{t_done:.6f}", p.strategy,
                            encode_payload(p.payload), result, r.error_kind or ""])
        self._maybe_flush()

    def state(self, s: StateSnapshot) -> None:
        t = f"{s.t_s:.6f}"
        for joint, value in s.joints.items():
            cell = _num(value)
            if cell is not None:
                self._state.writerow([t, "encoder", joint, cell])
        for point, xyz in (s.fk or {}).items():
            for axis, value in zip("xyz", xyz):
                cell = _num(value)
                if cell is not None:
                    self._state.writerow([t, "fk", f"{point}.{axis}", cell])
        self._maybe_flush()

    def _maybe_flush(self) -> None:
        if time.monotonic() - self._last_flush >= FLUSH_EVERY_S:
            self._cmd_fh.flush()
            self._state_fh.flush()
            self._last_flush = time.monotonic()

    def close(self) -> None:
        for fh in (self._cmd_fh, self._state_fh):
            fh.flush()
            fh.close()


class CommandThread(threading.Thread):
    """Paces proposals through ``adapter.send``; reads state between them; executes the stop.

    It checks the latch immediately before every send, so after a trip at most the one command
    already inside ``adapter.send`` completes, and the adapter itself refuses any later one."""

    def __init__(self, *, adapter, latch: StopLatch, state_box: StateBox, fk: ArmFK | None,
                 files: RawFiles | None, proposals: Iterable[Proposal], clock: Callable[[], float],
                 stop_decided: Callable[[], bool], limits: Limits = DEFAULT,
                 log: Callable[[str], None] = print):
        super().__init__(name="be-command", daemon=True)
        self.adapter, self.latch, self.state_box, self.fk = adapter, latch, state_box, fk
        self.files, self.proposals, self.clock = files, proposals, clock
        self.stop_decided, self.limits, self.log = stop_decided, limits, log
        self.go = threading.Event()  # main thread: start sending
        self.abort = threading.Event()  # main thread: leave without sending (nothing to stop)
        self.sent = 0
        self.applied = 0
        self.error_kinds: dict[str, int] = {}
        self.first_t: float | None = None
        self.last_t: float | None = None
        self.stream_finished = False
        self.stop_calls = 0
        self.first_stop_t: float | None = None  # when execute_stop() first returned (runner time)
        self.stop_errors: list[str] = []
        self._last_state_t = -math.inf

    # ----- state -------------------------------------------------------------------------------
    def read_state(self) -> None:
        """One read_state(), stamped at the middle of the call, with FK, to the box and the CSV."""
        t_a = self.clock()
        error = None
        try:
            st = self.adapter.read_state()
        except Exception as e:  # noqa: BLE001 - recorded; the FK tripwire trips on it
            st, error = None, f"read_state raised {type(e).__name__}: {e}"
        t_b = self.clock()
        self._last_state_t = t_b
        if st is None and error is None:
            if self.fk is None:
                return  # this adapter has no joint state, and nothing expects one
            error = "read_state returned None"
        joints = dict(st.joints) if st is not None else {}
        units = dict(st.units) if st is not None else {}
        fk = None
        if st is not None and self.fk is not None:
            try:
                fk = self.fk.points(joints, units)
            except FKError as e:
                error = str(e)
        snap = StateSnapshot((t_a + t_b) / 2, joints, units, fk, error)
        self.state_box.put(snap)
        if self.files is not None:
            self.files.state(snap)

    def _state_if_due(self) -> None:
        if self.clock() - self._last_state_t >= self.limits.state_period_s:
            self.read_state()

    # ----- the thread --------------------------------------------------------------------------
    def run(self) -> None:
        try:
            while not (self.go.is_set() or self.abort.is_set() or self.latch.tripped):
                self._state_if_due()
                time.sleep(0.002)
            if self.go.is_set() and not self.latch.tripped and not self.abort.is_set():
                self._send_all()
        except BaseException as e:  # noqa: BLE001 - any fault here stops the run
            self.latch.trip("error", f"command thread: {type(e).__name__}: {e}")
        finally:
            if self.latch.tripped:
                self._stop_phase()

    def _send_all(self) -> None:
        t_base = self.clock()
        for p in self.proposals:
            due = t_base + p.t_s
            while True:
                if self.latch.tripped:
                    return
                self._state_if_due()
                now = self.clock()
                if now >= due:
                    break
                time.sleep(min(due - now, 0.002))
            if self.latch.tripped:  # the last look before every send
                return
            t_sent = self.clock()
            try:
                r = self.adapter.send(p.payload)
            except Exception as e:  # noqa: BLE001 - the contract says send() never raises; fail closed
                r = SendResult(False, None, f"adapter_raised:{type(e).__name__}")
                self.latch.trip("error", f"adapter.send raised {type(e).__name__}: {e}")
            t_done = self.clock()
            self.sent += 1
            self.applied += int(r.applied)
            if r.error_kind:
                self.error_kinds[r.error_kind] = self.error_kinds.get(r.error_kind, 0) + 1
            self.first_t = t_sent if self.first_t is None else self.first_t
            self.last_t = t_sent
            if self.files is not None:
                self.files.command(p, t_sent, t_done, r)
        self.stream_finished = True
        self.latch.trip("finished", "the command stream ended")

    def _stop_phase(self) -> None:
        """Re-send the stop (the adapter keeps its goals from the first call) every state period,
        with a state read each time, until the watchdog has decided and stop_hold_s has passed,
        or stop_hold_max_s in any case."""
        lim = self.limits
        t_latch = self.latch.t_s if self.latch.t_s is not None else self.clock()
        while True:
            try:
                self.adapter.execute_stop()
                self.stop_calls += 1
                if self.first_stop_t is None:
                    self.first_stop_t = self.clock()
            except Exception as e:  # noqa: BLE001 - keep trying; the watchdog escalates if it does not stop
                if len(self.stop_errors) < 20:
                    self.stop_errors.append(f"{type(e).__name__}: {e}")
                if len(self.stop_errors) == 1:
                    self.log(f"*** The stop raised {type(e).__name__}: {e}. Retrying.")
            self.read_state()
            held = self.clock() - t_latch
            if held >= lim.stop_hold_max_s or (held >= lim.stop_hold_s and self.stop_decided()):
                return
            time.sleep(lim.state_period_s)


@dataclass
class SessionResult:
    """How a session ended. ``started`` is True once the command thread was released."""

    status: str  # "ended" | "cancelled" | "refused" | "stuck"
    why: str
    started: bool = False
    command_thread: CommandThread | None = None
    watchdog: Watchdog | None = None
    details: dict = field(default_factory=dict)


def _wait(predicate: Callable[[], bool], timeout_s: float, poll_s: float = 0.02) -> bool:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(poll_s)
    return predicate()


def run_session(*, adapter, latch: StopLatch, watchdog: Watchdog, command: CommandThread, feed,
                prompt: Callable[[str], str], confirm_word: str, confirm_text: str,
                wait_inside_and_still: bool, limits: Limits = DEFAULT,
                log: Callable[[str], None] = print) -> SessionResult:
    """Steps 1 to 7 of the module docstring. The caller opened the adapter and closes it.

    If the main thread fails here, the command thread is told to leave without sending (before
    step 6) or the latch is tripped so it stops the arm (after), and the error propagates."""
    try:
        return _run_session(adapter=adapter, latch=latch, watchdog=watchdog, command=command,
                            feed=feed, prompt=prompt, confirm_word=confirm_word,
                            confirm_text=confirm_text, wait_inside_and_still=wait_inside_and_still,
                            limits=limits, log=log)
    except BaseException as e:
        if command.go.is_set():
            latch.trip("error", f"runner main thread: {type(e).__name__}: {e}")
        else:
            command.abort.set()
        raise


def _run_session(*, adapter, latch, watchdog, command, feed, prompt, confirm_word, confirm_text,
                 wait_inside_and_still, limits, log) -> SessionResult:
    if feed is not None:
        feed.start()
        if not _wait(lambda: feed.snapshot().last_arrival_s is not None, limits.feed_start_timeout_s):
            return SessionResult("refused", f"the instrument sent nothing within "
                                            f"{limits.feed_start_timeout_s:.0f} s; start it first")
    command.start()
    if not _wait(lambda: bool(watchdog.state_box.recent()) or command.fk is None, 2.0):
        command.abort.set()
        command.join(2.0)
        return SessionResult("refused", "no robot state could be read")
    watchdog.start()

    if not confirm(prompt, confirm_word, confirm_text):
        command.abort.set()
        command.join(2.0)
        if latch.tripped:  # the watchdog tripped while the operator was reading
            return _finish(latch, watchdog, command, limits, log, started=False)
        return SessionResult("cancelled", "cancelled by the operator", command_thread=command,
                             watchdog=watchdog)

    previous = _install_sigint(latch, log)
    try:
        if wait_inside_and_still:
            log("Waiting for the instrument to see the arm inside and still "
                f"({limits.still_window_s:.0f} s, {limits.still_m * 1000:.0f} mm)...")
            ok, why = False, ""
            end = time.monotonic() + limits.start_timeout_s
            while time.monotonic() < end and not latch.tripped:
                ok, why = watchdog.start_check()
                if ok:
                    break
                time.sleep(0.1)
            if latch.tripped:
                return _finish(latch, watchdog, command, limits, log, started=False)
            if not ok:
                command.abort.set()
                command.join(2.0)
                return SessionResult("cancelled", f"not inside and still within "
                                                  f"{limits.start_timeout_s:.0f} s: {why}",
                                     command_thread=command, watchdog=watchdog)
            log(f"Starting: {why}.")
        command.go.set()
        return _finish(latch, watchdog, command, limits, log, started=True)
    finally:
        _restore_sigint(previous)


def _finish(latch, watchdog, command, limits, log, started: bool) -> SessionResult:
    """Wait for the command thread: the whole stream, then its stop phase. Once the latch is set
    it gets stop_hold_max_s plus stop_stuck_s; a thread still alive after that is stuck (inside the
    system under test, holding the bus)."""
    while command.is_alive():
        command.join(0.1)
        if (latch.tripped and latch.t_s is not None and command.is_alive()
                and latch.clock() - latch.t_s > limits.stop_hold_max_s + limits.stop_stuck_s):
            log("*** The command thread did not finish its stop (stuck inside the system under "
                "test?). CUT POWER if the arm is not still.")
            return SessionResult("stuck", latch.reason or "", started, command, watchdog)
    return SessionResult("ended", latch.reason or "", started, command, watchdog)


def _install_sigint(latch: StopLatch, log):
    """Ctrl+C trips the latch (the command thread then stops the arm). Main thread only."""
    if threading.current_thread() is not threading.main_thread():
        return None

    def handler(signum, frame):
        if latch.trip("operator", "Ctrl+C"):
            log("\nStopping (Ctrl+C).")
        else:
            log("\nAlready stopping. Cut power if the arm is moving.")

    return signal.signal(signal.SIGINT, handler)


def _restore_sigint(previous) -> None:
    if previous is not None and threading.current_thread() is threading.main_thread():
        signal.signal(signal.SIGINT, previous)
