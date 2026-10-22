# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""DRAFT — requires line-by-line human review before use on hardware.

EV-02 and EV-05 sessions (``be run ev02``, ``be run ev05``): core's ``be-stops/1`` and
``be-model-loss/1`` streams, sent one trial (one segment of the stream) at a time.

What is new against an EV-03 session (``session.py``), and nothing else is:

- ``TrialCommandThread`` paces each segment from the moment the segment begins. An EV-02 segment
  ends in a ``stop:<source>`` proposal, which is never sent: at its time the thread issues the
  stop through that source (``adapter.trigger_stop(source)`` for a source the adapter issues
  itself, the lab's ``--estop-cmd`` for ``hardwired_stop`` or ``power_loss``) and records the row,
  its ``t_sent_s`` being the moment it began to issue it. An EV-05 segment's restart proposals come
  after a silence in which the thread sends nothing at all.
- After each stop or loss instant the thread tells the watchdog what the trial allows
  (``Watchdog.watch_trial``): the tool at rest by then plus 0.5 s, and staying there until the
  commands resume. Otherwise the watchdog trips the latch, as any tripwire does.
- Between segments the thread waits, reading state, until the hold can have ended; then the main
  thread asks the operator for a typed NEXT. Only after NEXT does the thread reset the source
  (``adapter.reset_stop(source)``), and it begins the next segment only once the watchdog's start
  check (inside and still) passes. Anything but NEXT ends the run, through the latch.
- If a stop cannot be issued (the adapter raises, the external command fails or times out), the
  latch trips at once: the runner's own stop never waits for a trial.

Everything else is the EV-03 session's, unchanged: the latch, the stop phase and the stop rule
(``CommandThread._stop_phase``), tripwires 1 and 2, the acknowledgement and the escalation, the
post-roll and the teardown. The thread is still the only one that calls the adapter.
"""
from __future__ import annotations

import shlex
import subprocess
import threading
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass

from bounded_embodiment.generator import Proposal

from ..adapters.base import ADAPTER_STOP_SOURCES, SendResult
from .interlocks import confirm
from .limits import DEFAULT, Limits
from .session import CommandThread, SessionResult, run_session

__all__ = ["STOP_PREFIX", "TrialCommandThread", "TrialSegment", "run_trial_session"]

STOP_PREFIX = "stop:"


@dataclass(frozen=True)
class TrialSegment:
    """One trial: its proposals (``t_s`` counted from the segment's start), what the watchdog
    expects after its instant, and what happens before the next one.

    kind        "stop" (EV-02: the instant is the stop row's) or "loss" (EV-05: the instant is
                the last motion row's send time)
    source      the stop source (EV-02) or the loss source (EV-05)
    allowed_s   the time the trial allows after its instant (the source's max_time_ms; in EV-05
                the model timeout plus that)
    pause_s     the least time from the instant (EV-02) or from the segment's last row (EV-05) to
                the moment the operator is asked for NEXT
    reset       whether ``adapter.reset_stop(source)`` runs after NEXT, before the next segment
    operator_reset  whether the operator must reset something by hand before typing NEXT (EV-05
                under a manual resume rule the adapter cannot reset; EV-02's external sources
                always ask for it)
    """

    proposals: tuple[Proposal, ...]
    label: str
    kind: str
    source: str
    allowed_s: float
    pause_s: float
    reset: bool
    operator_reset: bool = False

    @property
    def last_motion(self) -> int | None:
        idx = [i for i, p in enumerate(self.proposals) if p.strategy == "motion"]
        return idx[-1] if idx else None

    @property
    def first_restart(self) -> int | None:
        return next((i for i, p in enumerate(self.proposals) if p.strategy == "restart"), None)


class TrialCommandThread(CommandThread):
    """``CommandThread`` for EV-02 and EV-05: segments, stops through named sources, pauses."""

    def __init__(self, *, segments: Sequence[TrialSegment], watchdog, adapter_sources: Sequence[str],
                 external_cmd: str | None, run_external: Callable = subprocess.run, **kw):
        proposals = [p for seg in segments for p in seg.proposals]
        super().__init__(proposals=proposals, **kw)
        self.segments = tuple(segments)
        self.watchdog = watchdog
        self.adapter_sources = frozenset(adapter_sources)
        self.external_cmd = external_cmd
        self.run_external = run_external
        self.asking = threading.Event()   # set by this thread: ask the operator for NEXT
        self.next_go = threading.Event()  # set by the main thread: the operator typed NEXT
        self.next_trial = 0               # the index of the trial NEXT would begin
        self.trials: list[dict] = [{"trial": k, "label": s.label, "source": s.source,
                                    "instant_t_s": None, "issued": None, "reset": None,
                                    "start_check": None} for k, s in enumerate(self.segments)]

    # ----- one proposal --------------------------------------------------------------------------
    def _record(self, p: Proposal, t_sent: float, t_done: float, r: SendResult) -> None:
        self.sent += 1
        self.applied += int(r.applied)
        if r.error_kind:
            self.error_kinds[r.error_kind] = self.error_kinds.get(r.error_kind, 0) + 1
        self.first_t = t_sent if self.first_t is None else self.first_t
        self.last_t = t_sent
        if self.files is not None:
            self.files.command(p, t_sent, t_done, r)

    def _send(self, p: Proposal) -> float:
        """As CommandThread._send_all sends each proposal; returns its send time."""
        t_sent = self.clock()
        try:
            r = self.adapter.send(p.payload)
        except Exception as e:  # noqa: BLE001 - the contract says send() never raises; fail closed
            r = SendResult(False, None, f"adapter_raised:{type(e).__name__}")
            self.latch.trip("error", f"adapter.send raised {type(e).__name__}: {e}")
        self._record(p, t_sent, self.clock(), r)
        return t_sent

    def _external(self, source: str) -> SendResult:
        """The lab's external stop command (--estop-cmd), run without a shell, with a timeout. It
        runs on this thread: a command slower than the state tripwire's 0.25 s stops the run."""
        try:
            done = self.run_external(shlex.split(self.external_cmd or ""), timeout=self.limits.estop_timeout_s,
                                     capture_output=True, check=False)
        except Exception as e:  # noqa: BLE001 - recorded; the latch trips below
            return SendResult(False, None, f"external_failed:{type(e).__name__}")
        if done.returncode != 0:
            return SendResult(False, None, f"external_returncode:{done.returncode}")
        return SendResult(True, "triggered", None)

    def _issue_stop(self, k: int, p: Proposal) -> None:
        """At the stop row's time: issue the stop through its source, record the row, and tell
        the watchdog. Any failure trips the latch: the runner's own stop then follows."""
        seg = self.segments[k]
        source = p.strategy[len(STOP_PREFIX):]
        t_sent = self.clock()
        if source in self.adapter_sources:
            try:
                self.adapter.trigger_stop(source)
                r = SendResult(True, "triggered", None)
            except Exception as e:  # noqa: BLE001 - recorded; the latch trips below
                r = SendResult(False, None, f"trigger_raised:{type(e).__name__}")
        elif self.external_cmd:
            r = self._external(source)
        else:  # run.py refuses such a plan; never issue nothing silently
            r = SendResult(False, None, "no_way_to_issue")
        self._record(p, t_sent, self.clock(), r)
        self.trials[k]["instant_t_s"] = t_sent
        self.trials[k]["issued"] = r.decision if r.applied else r.error_kind
        if not r.applied:
            self.latch.trip("error", f"{seg.label}: the stop through {source} could not be issued "
                                     f"({r.error_kind})")
            return
        self.watchdog.watch_trial(seg.label, t_sent, seg.allowed_s)

    # ----- the thread ------------------------------------------------------------------------------
    def _wait_until(self, t: float) -> bool:
        """Read state until the run's clock reaches t; False if the latch tripped meanwhile."""
        while True:
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
        self.watchdog.end_trial_watch()  # the commands resume
        last_motion, first_restart = seg.last_motion, seg.first_restart
        for i, p in enumerate(seg.proposals):
            if not self._wait_until(t_base + p.t_s):
                return False
            if self.latch.tripped:  # the last look before every send
                return False
            if p.strategy.startswith(STOP_PREFIX):
                self._issue_stop(k, p)
                continue
            if i == first_restart:
                self.watchdog.end_trial_watch()  # the commands resume after the silence
            t_sent = self._send(p)
            if i == last_motion:  # EV-05: the loss instant is the last motion command's send time
                self.trials[k]["instant_t_s"] = t_sent
                self.trials[k]["issued"] = "silence"
                self.watchdog.watch_trial(seg.label, t_sent, seg.allowed_s)
        return not self.latch.tripped

    def _pause(self, k: int) -> bool:
        """Wait out trial k's hold: pause_s after its instant (EV-02) or its last row (EV-05)."""
        seg = self.segments[k]
        start = self.trials[k]["instant_t_s"] if seg.kind == "stop" else self.last_t
        if start is None:
            start = self.clock()
        return self._wait_until(start + seg.pause_s)

    def _between(self, k: int) -> bool:
        """Before trial k (k > 0): NEXT from the operator, the reset, the start check."""
        prev = self.segments[k - 1]
        self.next_trial = k
        self.asking.set()
        while not self.next_go.is_set():
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
        end = self.clock() + self.limits.start_timeout_s
        ok, why = False, ""
        while self.clock() < end:
            if self.latch.tripped or self.abort.is_set():
                return False
            self._state_if_due()
            ok, why = self.watchdog.start_check()
            if ok:
                break
            time.sleep(0.1)
        self.trials[k]["start_check"] = why
        if not ok:
            self.latch.trip("error", f"before {self.segments[k].label}: not inside and still within "
                                     f"{self.limits.start_timeout_s:.0f} s: {why}")
            return False
        self.log(f"Starting {self.segments[k].label}: {why}.")
        return True

    def _send_all(self) -> None:
        for k in range(len(self.segments)):
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
                      log: Callable[[str], None] = print) -> SessionResult:
    """``session.run_session`` for a trial stream: the same start (feed, state, watchdog, typed
    RUN, inside and still) and the same end, with the operator's NEXT between trials
    (``_between_trials``, on the main thread while the command thread runs)."""
    return run_session(adapter=adapter, latch=latch, watchdog=watchdog, command=command, feed=feed,
                       prompt=prompt, confirm_word="RUN",
                       confirm_text="Hands clear of the machine; one hand on the hardware stop.",
                       wait_inside_and_still=True, limits=limits, log=log,
                       between=lambda: _between_trials(latch, command, prompt, limits))


def _between_trials(latch, command: TrialCommandThread, prompt: Callable[[str], str],
                    limits: Limits) -> None:
    """Answer the command thread's requests for NEXT until it ends. Returns early when the latch
    has been set for longer than the stop phase can take, so session._finish can report a stuck
    command thread. Ctrl+C is session.py's handler: it trips the latch."""
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
        if confirm(prompt, "NEXT", text):
            command.next_go.set()
        else:
            latch.trip("operator", f"the operator ended the run before trial {k + 1} of {n}")
