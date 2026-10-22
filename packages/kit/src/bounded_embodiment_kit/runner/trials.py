# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""DRAFT — requires line-by-line human review before use on hardware.

EV-02 and EV-05 sessions (``be run ev02``, ``be run ev05``): core's ``be-stops/1`` and
``be-model-loss/1`` streams, sent one trial (one segment of the stream) at a time.

What is new against an EV-03 session (``session.py``), and nothing else is:

- ``TrialCommandThread`` paces each segment from the moment the segment begins. An EV-02 segment
  holds one ``stop:<source>`` proposal, which is never sent: at its time the thread arms the
  watchdog's trial watch (``Watchdog.watch_trial``, tripwire 3) and then issues the stop through
  that source: ``adapter.trigger_stop("stop_message")``, or for ``hardwired_stop`` and
  ``power_loss`` the lab's own command for that source (``--hardwired-stop-cmd``,
  ``--power-loss-cmd``), started without a shell, its input and output closed, in a session of
  its own, and never waited for on this thread. Its row's ``t_sent_s`` is the moment issuing
  began. The ``continue:<id>`` proposals after it are sent like any others (method §12.5, D1:
  the model keeps commanding the drive through the stop); a system that refuses them while
  stopped is expected to, and its refusals are recorded, not treated as errors. An EV-05
  segment's loss instant is its last motion command's send time; the watch is armed there.
- A helper thread waits up to ``estop_timeout_s`` for a source command. If it fails or times
  out, the latch trips and the stop row records why (core then leaves the trial unmeasured).
  Rows sent while the outcome is unknown are written after the stop row, in order.
- The watch stays armed until the next segment begins: through the commands that continue after
  the stop, the pause, the operator's NEXT, the reset and the start check. In EV-05 under a
  ``commands`` resume rule it ends when the rule lets the machine move (that long after the
  first restart command); under a ``manual`` rule it too lasts until the next segment.
- Between segments the thread waits, reading state, ``pause_s`` after the segment's last row;
  then the main thread asks for a typed NEXT. After NEXT, and a last look at the latch, the
  thread resets the source (``adapter.reset_stop(source)``) when the trial needs it, and begins
  the next segment only once the start check (inside and still) passes on samples taken after
  the reset returned: a full still window of them. In a velocity command space the stop point
  must also have room for the trial: at least the travel the trial may take (``travel_m``) to
  its nearest zone boundary. Anything else ends the run, through the latch.

Everything else is the EV-03 session's, unchanged: the latch, the stop phase and the stop rule
(``CommandThread._stop_phase``), tripwires 1 and 2, the acknowledgement and the escalation, the
post-roll and the teardown. The thread is still the only one that calls the adapter; the helper
threads only wait on the source commands.
"""
from __future__ import annotations

import shlex
import subprocess
import threading
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass

from bounded_embodiment.generator import Proposal

from ..adapters.base import ADAPTER_STOP_SOURCES, SendResult
from .interlocks import confirm
from .limits import DEFAULT, Limits
from .session import CommandThread, SessionResult, run_session

__all__ = [
    "EXTERNAL_SOURCES",
    "STOP_PREFIX",
    "TrialCommandThread",
    "TrialSegment",
    "run_trial_session",
    "spawn_source_command",
]

STOP_PREFIX = "stop:"
#: A source command normally ends at once: the helper's first, plain wait for it (seconds).
FIRST_WAIT_S = 0.5
#: The stop sources outside the system's software: the lab's bench issues them with a command of
#: its own for each (``--hardwired-stop-cmd``, ``--power-loss-cmd``), never with ``--estop-cmd``.
EXTERNAL_SOURCES = ("hardwired_stop", "power_loss")


@dataclass(frozen=True)
class TrialSegment:
    """One trial: its proposals (``t_s`` counted from the segment's start), what the watchdog
    expects after its instant, and what happens before the next one.

    kind        "stop" (EV-02: the instant is the stop row's) or "loss" (EV-05: the instant is
                the last motion row's send time)
    source      the stop source (EV-02) or the loss source (EV-05)
    allowed_s   the time the trial allows after its instant (the source's max_time_ms; in EV-05
                the model timeout plus that)
    pause_s     the least time from the segment's last row to the moment the operator is asked
                for NEXT
    reset       whether ``adapter.reset_stop(source)`` runs after NEXT, before the next segment
    operator_reset  whether the operator must reset something by hand before typing NEXT (EV-02's
                external sources; EV-05 under a manual resume rule the adapter cannot reset)
    point       the plan's stop point, which the watch follows (None: the tool point)
    release_after_s  EV-05 under a commands resume rule: the watch ends this long after the first
                restart command, when the rule lets the machine move; None: at the next segment
    travel_m    velocity command spaces: the most the trial may carry the stop point (its drive or
                motion at the speed limit, plus the stop), which must fit before its zone
                boundaries at the segment's start; None: not checked (position spaces)
    """

    proposals: tuple[Proposal, ...]
    label: str
    kind: str
    source: str
    allowed_s: float
    pause_s: float
    reset: bool
    operator_reset: bool = False
    point: str | None = None
    release_after_s: float | None = None
    travel_m: float | None = None

    @property
    def last_motion(self) -> int | None:
        idx = [i for i, p in enumerate(self.proposals) if p.strategy == "motion"]
        return idx[-1] if idx else None

    @property
    def first_restart(self) -> int | None:
        return next((i for i, p in enumerate(self.proposals) if p.strategy == "restart"), None)


def spawn_source_command(cmd: str) -> subprocess.Popen:
    """Start a stop source's command (review B1): split with ``shlex``, no shell, every stream
    closed (so a command that leaves a child behind cannot hold a pipe open), in a session of its
    own (so a Ctrl+C at the bench does not reach it)."""
    return subprocess.Popen(shlex.split(cmd), stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL, start_new_session=True)


class TrialCommandThread(CommandThread):
    """``CommandThread`` for EV-02 and EV-05: segments, stops through named sources, pauses."""

    def __init__(self, *, segments: Sequence[TrialSegment], watchdog, adapter_sources: Sequence[str],
                 external_cmds: Mapping[str, str | None] | None = None,
                 spawn: Callable[[str], subprocess.Popen] = spawn_source_command,
                 clearance: Callable[[str | None], tuple[float, str] | None] | None = None,
                 check_next: Callable[[TrialSegment], str | None] | None = None, **kw):
        proposals = [p for seg in segments for p in seg.proposals]
        super().__init__(proposals=proposals, **kw)
        self.segments = tuple(segments)
        self.watchdog = watchdog
        self.adapter_sources = frozenset(adapter_sources)
        self.external_cmds = {k: v for k, v in (external_cmds or {}).items() if v}
        self.spawn = spawn
        self.clearance = clearance  # the stop point's room to its nearest boundary, now
        self.check_next = check_next  # why the next segment may not begin from here, or None
        self.asking = threading.Event()   # set by this thread: ask the operator for NEXT
        self.next_go = threading.Event()  # set by the main thread: the operator typed NEXT
        self.next_trial = 0               # the index of the trial NEXT would begin
        self.trials: list[dict] = [{"trial": k, "label": s.label, "source": s.source,
                                    "instant_t_s": None, "issued": None, "reset": None,
                                    "start_check": None} for k, s in enumerate(self.segments)]
        self._pending: dict | None = None  # a stop row waiting for its source command's outcome
        self._held_rows: list[tuple] = []  # rows sent meanwhile, written after it
        self._release_at: float | None = None

    # ----- recording -------------------------------------------------------------------------------
    def _write(self, p: Proposal, t_sent: float, t_done: float, r: SendResult) -> None:
        if self.files is None:
            return
        if self._pending is not None:
            self._held_rows.append((p, t_sent, t_done, r))
        else:
            self.files.command(p, t_sent, t_done, r)

    def _record(self, p: Proposal, t_sent: float, t_done: float, r: SendResult) -> None:
        self.sent += 1
        self.applied += int(r.applied)
        if r.error_kind:
            self.error_kinds[r.error_kind] = self.error_kinds.get(r.error_kind, 0) + 1
        self.first_t = t_sent if self.first_t is None else self.first_t
        self.last_t = t_sent
        self._write(p, t_sent, t_done, r)

    def _settle_pending(self, final: bool = False) -> None:
        """Write the pending stop row once its command's outcome is known (or, with ``final``, as
        unfinished), then the rows held back meanwhile, in order."""
        pend = self._pending
        if pend is None or not (pend["done"].is_set() or final):
            return
        r = pend["result"] if pend["done"].is_set() else SendResult(False, None, "external_unfinished")
        t_done = pend["t_done"] if pend["done"].is_set() else self.clock()
        self.applied += int(r.applied)
        if r.error_kind:
            self.error_kinds[r.error_kind] = self.error_kinds.get(r.error_kind, 0) + 1
        self.trials[pend["k"]]["issued"] = r.decision if r.applied else r.error_kind
        self._pending = None
        if self.files is not None:
            self.files.command(pend["p"], pend["t_sent"], t_done, r)
            for row in self._held_rows:
                self.files.command(*row)
        self._held_rows = []

    def _send(self, p: Proposal) -> float:
        """As CommandThread._send_all sends each proposal; returns its send time. A system that
        refuses commands while its stop is in force says so in the result; that is recorded, and
        only an adapter that raises (breaking its contract) trips the latch."""
        t_sent = self.clock()
        try:
            r = self.adapter.send(p.payload)
        except Exception as e:  # noqa: BLE001 - the contract says send() never raises; fail closed
            r = SendResult(False, None, f"adapter_raised:{type(e).__name__}")
            self.latch.trip("error", f"adapter.send raised {type(e).__name__}: {e}")
        self._record(p, t_sent, self.clock(), r)
        return t_sent

    # ----- issuing a stop ----------------------------------------------------------------------------
    def _wait_for_command(self, pend: dict, proc: subprocess.Popen, label: str, source: str) -> None:
        """Helper thread: wait for a source command, up to ``estop_timeout_s`` on the run's clock;
        a failure or a timeout trips the latch. A command normally ends at once, so the first
        half second is one plain wait; after that the thread polls, sleeping on the run's clock."""
        deadline = pend["t_sent"] + self.limits.estop_timeout_s
        try:
            try:
                rc = proc.wait(timeout=min(FIRST_WAIT_S, self.limits.estop_timeout_s))
            except subprocess.TimeoutExpired:
                rc = proc.poll()
                while rc is None and self.clock() < deadline:
                    time.sleep(0.01)
                    rc = proc.poll()
            if rc is None:
                r = SendResult(False, None, "external_timeout")
            elif rc == 0:
                r = SendResult(True, "issued", None)
            else:
                r = SendResult(False, None, f"external_returncode:{rc}")
        except Exception as e:  # noqa: BLE001 - recorded; the latch trips below
            r = SendResult(False, None, f"external_failed:{type(e).__name__}")
        pend["result"], pend["t_done"] = r, self.clock()
        pend["done"].set()
        if not r.applied:
            self.latch.trip("error", f"{label}: the {source} command did not succeed ({r.error_kind})")

    def _issue_stop(self, k: int, p: Proposal) -> None:
        """At the stop row's time: arm the watch, then issue the stop through its source and
        record the row. Any failure trips the latch: the runner's own stop then follows."""
        seg = self.segments[k]
        source = p.strategy[len(STOP_PREFIX):]
        t_sent = self.clock()
        self.trials[k]["instant_t_s"] = t_sent
        self.watchdog.watch_trial(seg.label, t_sent, seg.allowed_s, seg.point)  # before issuing
        self.sent += 1
        self.first_t = t_sent if self.first_t is None else self.first_t
        self.last_t = t_sent
        if source in ADAPTER_STOP_SOURCES and source in self.adapter_sources:
            try:
                self.adapter.trigger_stop(source)
                r = SendResult(True, "triggered", None)
            except Exception as e:  # noqa: BLE001 - recorded; the latch trips below
                r = SendResult(False, None, f"trigger_raised:{type(e).__name__}")
            self._finish_issue(k, p, t_sent, r)
            return
        cmd = self.external_cmds.get(source) if source in EXTERNAL_SOURCES else None
        if cmd is None:  # run.py refuses such a plan; never issue nothing silently (fail closed)
            self._finish_issue(k, p, t_sent, SendResult(False, None, "no_way_to_issue"))
            return
        try:
            proc = self.spawn(cmd)
        except Exception as e:  # noqa: BLE001 - recorded; the latch trips below
            self._finish_issue(k, p, t_sent, SendResult(False, None, f"external_failed:{type(e).__name__}"))
            return
        self._settle_pending(final=True)  # (never more than one: a trial issues one stop)
        pend = {"p": p, "t_sent": t_sent, "k": k, "done": threading.Event(), "result": None,
                "t_done": None}
        self._pending = pend
        threading.Thread(target=self._wait_for_command, args=(pend, proc, seg.label, source),
                         name="be-stop-cmd", daemon=True).start()

    def _finish_issue(self, k: int, p: Proposal, t_sent: float, r: SendResult) -> None:
        self.applied += int(r.applied)
        if r.error_kind:
            self.error_kinds[r.error_kind] = self.error_kinds.get(r.error_kind, 0) + 1
        self.trials[k]["issued"] = r.decision if r.applied else r.error_kind
        self._write(p, t_sent, self.clock(), r)
        if not r.applied:
            seg = self.segments[k]
            self.latch.trip("error", f"{seg.label}: the stop through {seg.source} could not be issued "
                                     f"({r.error_kind})")

    # ----- the thread ------------------------------------------------------------------------------
    def run(self) -> None:
        try:
            super().run()
        finally:
            self._settle_pending(final=True)

    def _wait_until(self, t: float) -> bool:
        """Read state until the run's clock reaches t; False if the latch tripped meanwhile."""
        while True:
            self._settle_pending()
            if self._release_at is not None and self.clock() >= self._release_at:
                self._release_at = None
                self.watchdog.end_trial_watch()  # the resume rule lets the machine move now
            if self.latch.tripped or self.abort.is_set():
                return False
            self._state_if_due()
            now = self.clock()
            if now >= t:
                return True
            time.sleep(min(t - now, 0.002))

    def _send_segment(self, k: int) -> bool:
        seg = self.segments[k]
        t_base = self.clock()
        self.watchdog.end_trial_watch()  # the previous trial is over: its watch ends here
        self._release_at = None
        last_motion, first_restart = seg.last_motion, seg.first_restart
        for i, p in enumerate(seg.proposals):
            if not self._wait_until(t_base + p.t_s):
                return False
            if self.latch.tripped:  # the last look before every send
                return False
            if p.strategy.startswith(STOP_PREFIX):
                self._issue_stop(k, p)
                continue
            if i == first_restart and seg.release_after_s is not None:
                self._release_at = self.clock() + seg.release_after_s
            t_sent = self._send(p)
            if i == last_motion:  # EV-05: the loss instant is the last motion command's send time
                self.trials[k]["instant_t_s"] = t_sent
                self.trials[k]["issued"] = "silence"
                self.watchdog.watch_trial(seg.label, t_sent, seg.allowed_s, seg.point)
        return not self.latch.tripped

    def _pause(self, k: int) -> bool:
        """Wait out trial k's pause after its last row."""
        start = self.last_t if self.last_t is not None else self.clock()
        return self._wait_until(start + self.segments[k].pause_s)

    def _room(self, k: int) -> bool:
        """Velocity command spaces: the stop point must have room for trial k (its travel_m) to
        its nearest zone boundary, or the run ends here, before the trial (review S5)."""
        seg = self.segments[k]
        if seg.travel_m is None or self.clearance is None:
            return True
        room = self.clearance(seg.point)
        if room is None:
            self.latch.trip("error", f"before {seg.label}: no recent sample of the stop point to check its room")
            return False
        clear, where = room
        if clear < seg.travel_m:
            self.latch.trip("error", f"before {seg.label}: the stop point is {clear:.2f} m from {where}, "
                                     f"and the trial may carry it {seg.travel_m:.2f} m (drive or "
                                     "motion at the speed limit, plus the stop and the margin)")
            return False
        self.log(f"{seg.label}: {clear:.2f} m of room, {seg.travel_m:.2f} m needed.")
        return True

    def _between(self, k: int) -> bool:
        """Before trial k (k > 0): NEXT from the operator, the reset, the start check (on samples
        taken after the reset), and the room the trial needs. The previous trial's watch stays
        armed throughout."""
        prev = self.segments[k - 1]
        self.next_trial = k
        self.asking.set()
        while not self.next_go.is_set():
            self._settle_pending()
            if self.latch.tripped or self.abort.is_set():
                return False
            self._state_if_due()
            time.sleep(0.002)
        self.next_go.clear()
        if self.latch.tripped or self.abort.is_set():  # the last look before the reset
            return False
        if prev.reset:
            try:
                self.adapter.reset_stop(prev.source)
                self.trials[k - 1]["reset"] = "done"
            except Exception as e:  # noqa: BLE001 - recorded; the run stops
                self.trials[k - 1]["reset"] = f"failed: {type(e).__name__}: {e}"
                self.latch.trip("error", f"{prev.label}: resetting {prev.source} failed: "
                                         f"{type(e).__name__}: {e}")
                return False
        else:
            self.trials[k - 1]["reset"] = "not needed"
        since = self.clock()  # only what the instrument sees after the reset counts (review S1)
        end = since + self.limits.start_timeout_s
        ok, why = False, ""
        while self.clock() < end:
            self._settle_pending()
            if self.latch.tripped or self.abort.is_set():
                return False
            self._state_if_due()
            ok, why = self.watchdog.start_check(since=since)
            if ok:
                break
            time.sleep(0.1)
        self.trials[k]["start_check"] = why
        if not ok:
            self.latch.trip("error", f"before {self.segments[k].label}: not inside and still within "
                                     f"{self.limits.start_timeout_s:.0f} s of the reset: {why}")
            return False
        if not self._room(k):
            return False
        far = self.check_next(self.segments[k]) if self.check_next is not None else None
        if far:  # e.g. a stop that did not hold left the machine where the walk does not begin
            self.latch.trip("error", f"before {self.segments[k].label}: {far}")
            return False
        self.log(f"Starting {self.segments[k].label}: {why}.")
        return True

    def _send_all(self) -> None:
        for k in range(len(self.segments)):
            if k == 0 and not self._room(0):
                return
            if k > 0 and not (self._pause(k - 1) and self._between(k)):
                return
            if not self._send_segment(k):
                return
        if not self._pause(len(self.segments) - 1):
            return
        self.stream_finished = True
        self.latch.trip("finished", "the last trial ended")


def run_trial_session(*, adapter, latch, watchdog, command: TrialCommandThread, feed,
                      prompt: Callable[[str], str], limits: Limits = DEFAULT,
                      log: Callable[[str], None] = print,
                      ask: Callable[..., bool] | None = None) -> SessionResult:
    """``session.run_session`` for a trial stream: the same start (feed, state, watchdog, typed
    RUN, inside and still) and the same end, with the operator's NEXT between trials
    (``_between_trials``, on the main thread while the command thread runs). ``ask`` (hardware:
    ``interlocks.typed_word``) asks for NEXT at the terminal; without it ``prompt`` does."""
    return run_session(adapter=adapter, latch=latch, watchdog=watchdog, command=command, feed=feed,
                       prompt=prompt, confirm_word="RUN",
                       confirm_text="Hands clear of the machine; one hand on the hardware stop.",
                       wait_inside_and_still=True, limits=limits, log=log,
                       between=lambda: _between_trials(latch, command, prompt, limits, ask))


def _between_trials(latch, command: TrialCommandThread, prompt: Callable[[str], str],
                    limits: Limits, ask: Callable[..., bool] | None = None) -> None:
    """Answer the command thread's requests for NEXT until it ends. Returns early when the latch
    has been set for longer than the stop phase can take, so session._finish can report a stuck
    command thread. Ctrl+C is session.py's handler: it trips the latch, which ends a question
    asked through ``ask`` at once."""
    n = len(command.segments)
    while command.is_alive():
        if not command.asking.wait(0.1):
            if (latch.tripped and latch.t_s is not None
                    and latch.clock() - latch.t_s > limits.stop_hold_max_s + limits.stop_stuck_s):
                return
            continue
        command.asking.clear()
        if latch.tripped:
            return
        k = command.next_trial
        prev = command.segments[k - 1]
        if prev.operator_reset or (prev.kind == "stop" and prev.source not in ADAPTER_STOP_SOURCES):
            reset = f" Reset {prev.source} first; the machine must not move when you do."
        elif prev.reset:
            reset = f" After NEXT the kit resets {prev.source} through the adapter."
        else:
            reset = ""
        text = f"{prev.label} done ({k} of {n}).{reset} Hands clear of the machine; one hand on the hardware stop."
        if ask is not None:
            go = ask("NEXT", text, cancelled=lambda: latch.tripped)
        else:
            go = confirm(prompt, "NEXT", text)
        if go:
            command.next_go.set()
        elif not latch.tripped:
            latch.trip("operator", f"the operator ended the run before trial {k + 1} of {n}")
