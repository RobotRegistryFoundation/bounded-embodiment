# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""A virtual clock that runs the kit's real-time runner faster than real time, and the same way
every time. Tests only.

The runner (``runner/session.py``, ``watchdog.py``, ``feeds.py``, ``run.py``) and the simulation
adapters (``sim_so101.py``, ``sim_rover.py``) take their time from the ``time`` module and wait with
``time.sleep`` and ``threading.Event.wait``; the kit has no timebase seam. ``virtual_time(clock,
modules, joins)`` stands in for one, for the duration of a ``with`` block:

- In the given modules, ``time.monotonic()`` and ``time.perf_counter()`` read the virtual clock,
  ``time.sleep(s)`` and ``Event.wait(timeout)`` return once it has advanced by ``s`` (an event wait
  also returns as soon as the event is set), and ``time.time()`` (the wall-clock stamps in
  declared.json and the shakedown ledger) stays real.
- The clock advances only when every thread that uses it is waiting on it, and then straight to the
  earliest wake-up; threads due at the same moment run one at a time, in the order they began to
  wait. So only one thread runs at a time, and computation takes no virtual time, except that
  every reading of the clock moves it on by at least ``READ_S`` (no two readings are equal). The
  command thread keeps to its schedule exactly, as in ev03-harness's lockstep simulation.
- A thread started inside the block runs alone until it first waits on the clock (``start()``
  returns only then), and ``join`` on the given thread classes waits on the clock. Without either,
  threads would run side by side in real time at moments that depend on the machine, and so would
  their readings of the clock; with both, the same case gives the same run, byte for byte.

A thread that blocked in some other real call (a lock held by a thread that waits on the clock)
would stop the clock for good; nothing in the runner or the simulators does, and the B6 matrix runs
every case under a real-time timeout in a process of its own.
"""
from __future__ import annotations

import itertools
import math
import threading
import time
from collections.abc import Iterable, Iterator
from contextlib import contextmanager
from dataclasses import dataclass, field

__all__ = ["JOIN_POLL_S", "READ_S", "VirtualClock", "VirtualEvent", "virtual_time"]

#: How often (real seconds) a waiting thread looks at the clock again by itself: the only way it
#: learns that a thread it was waiting behind has ended.
REAL_POLL_S = 0.05
#: Every reading of the clock moves it on by at least this much, so that no two readings are equal
#: (the runner stamps state rows to the microsecond, and core's readers refuse two rows of one
#: point at the same time). On hardware a bus read takes milliseconds.
READ_S = 2e-6
#: A patched ``join`` looks whether the thread has ended this often, in virtual seconds.
JOIN_POLL_S = 0.02
#: How long (real seconds) ``start()`` waits for a new thread's first wait before going on anyway.
SETTLE_TIMEOUT_S = 10.0


@dataclass(eq=False)
class _Wait:
    wake: float
    order: int
    event: VirtualEvent | None = None
    real: threading.Event = field(default_factory=threading.Event)


class VirtualClock:
    """The clock: ``monotonic``, ``perf_counter`` and ``sleep`` as the ``time`` module has them."""

    def __init__(self, start: float = 1000.0):
        self.start = float(start)
        self._now = self.start
        self._read = -math.inf  # the last reading handed out
        self._lock = threading.Lock()
        self._waiting: dict[threading.Thread, _Wait] = {}
        self._threads: set[threading.Thread] = {threading.main_thread()}
        self._real: dict[threading.Thread, threading.Event] = {}  # one reusable event per thread
        self._order = itertools.count()
        self.advances = 0

    # ----- the time module's API ----------------------------------------------------------------
    def monotonic(self) -> float:
        with self._lock:
            self._now = max(self._now, self._read + READ_S)
            self._read = self._now
            return self._now

    perf_counter = monotonic

    def sleep(self, seconds: float) -> None:
        self.wait(seconds)

    def now(self) -> float:
        """The clock's time, without counting as a reading."""
        with self._lock:
            return self._now

    # ----- waiting ------------------------------------------------------------------------------
    def wait(self, timeout: float | None, event: VirtualEvent | None = None) -> bool:
        """Wait ``timeout`` virtual seconds (None: until ``event`` is set). Returns whether the
        event is set (False for a plain sleep)."""
        me = threading.current_thread()
        with self._lock:
            if event is not None and event._flag:
                return True
            if timeout is None:
                wake = math.inf
            elif timeout > 0:
                # A positive wait always lets time move on, even when now + timeout rounds to now.
                wake = max(self._now + float(timeout), math.nextafter(self._now, math.inf))
            else:
                wake = self._now
            w = _Wait(wake, next(self._order), event, self._event_for(me))
            self._threads.add(me)
            self._waiting[me] = w
            if event is not None:
                event._waits.add(w)
            self._step()
        while not w.real.wait(REAL_POLL_S):
            with self._lock:
                self._step()
        with self._lock:
            self._waiting.pop(me, None)
            if event is not None:
                event._waits.discard(w)
                return event._flag
        return False

    def adopt(self, thread: threading.Thread) -> None:
        """Count a thread just started as running, and return once it waits on the clock (or has
        ended): until then the caller, which is running too, keeps time standing still, and the
        new thread runs alone."""
        with self._lock:
            self._threads.add(thread)
        deadline = time.monotonic() + SETTLE_TIMEOUT_S
        while thread.is_alive() and time.monotonic() < deadline:
            with self._lock:
                if thread in self._waiting:
                    return
            time.sleep(0.0002)

    def join(self, thread: threading.Thread, timeout: float | None) -> None:
        """``thread.join(timeout)`` with the timeout in virtual seconds, waiting on the clock."""
        end = None if timeout is None else self.now() + timeout
        while thread.is_alive() and (end is None or self.now() < end):
            self.wait(JOIN_POLL_S if end is None else min(JOIN_POLL_S, end - self.now()))

    def _event_for(self, thread: threading.Thread) -> threading.Event:
        """The thread's reusable wake-up event, cleared (lock held)."""
        real = self._real.get(thread)
        if real is None:
            real = self._real[thread] = threading.Event()
        real.clear()
        return real

    def _step(self) -> None:
        """Lock held. If no thread is running, let the first due one run, moving the clock on to
        its wake-up if that is in the future. Due first: a waiter whose event is set, then by
        wake-up time, then by when it began to wait."""
        waiting = self._waiting
        for t in tuple(self._threads):
            w = waiting.get(t)
            if w is None:
                if t.is_alive():
                    return  # a thread is running: time stands still
                self._threads.discard(t)
                self._real.pop(t, None)
            elif w.real.is_set():
                return  # woken and not yet waiting again: running
        best, best_key = None, None
        for w in waiting.values():
            if w.event is not None and w.event._flag:
                key = (-math.inf, w.order)
            elif w.wake == math.inf:
                continue  # waits for an event that is not set
            else:
                key = (w.wake, w.order)
            if best_key is None or key < best_key:
                best, best_key = w, key
        if best is None:
            return
        if best_key[0] > self._now:
            self._now = best_key[0]
            self.advances += 1
        best.real.set()


class VirtualEvent:
    """``threading.Event`` whose ``wait(timeout)`` counts virtual time."""

    def __init__(self, clock: VirtualClock):
        self._clock = clock
        self._flag = False
        self._waits: set[_Wait] = set()

    def is_set(self) -> bool:
        return self._flag

    isSet = is_set

    def set(self) -> None:
        with self._clock._lock:
            self._flag = True
            for w in self._waits:
                w.real.set()

    def clear(self) -> None:
        with self._clock._lock:
            self._flag = False

    def wait(self, timeout: float | None = None) -> bool:
        return self._clock.wait(timeout, self)


class _TimeModule:
    """Stands in for the ``time`` module inside a patched module."""

    def __init__(self, clock: VirtualClock):
        self.monotonic = clock.monotonic
        self.perf_counter = clock.perf_counter
        self.sleep = clock.sleep

    def __getattr__(self, name: str):  # time.time (wall clock), time.strftime, ...: the real ones
        return getattr(time, name)


class _ThreadingModule:
    """Stands in for the ``threading`` module inside a patched module: only ``Event`` differs."""

    def __init__(self, clock: VirtualClock):
        self._clock = clock

    def Event(self) -> VirtualEvent:
        return VirtualEvent(self._clock)

    def __getattr__(self, name: str):
        return getattr(threading, name)


@contextmanager
def virtual_time(clock: VirtualClock, modules: Iterable, joins: Iterable[type] = ()
                 ) -> Iterator[VirtualClock]:
    """Run the given modules on ``clock`` for the duration of the block: their ``time`` and
    ``threading`` globals (where they have them) are replaced, every thread started in the process
    runs alone until it first waits on the clock, and ``join`` on the given thread classes waits
    on the clock. Objects created inside the block keep virtual events after it. It patches
    process-wide state: one block at a time, in a process of its own."""
    saved = []
    try:
        for m in modules:
            for name, shim in (("time", _TimeModule(clock)), ("threading", _ThreadingModule(clock))):
                if hasattr(m, name):
                    saved.append((m, name, getattr(m, name)))
                    setattr(m, name, shim)
        real_start = threading.Thread.start

        def start(self, _real=real_start):
            # The starting thread is running (not waiting) until this returns, so the clock stands
            # still while the new thread starts up.
            _real(self)
            clock.adopt(self)

        saved.append((threading.Thread, "start", real_start))
        threading.Thread.start = start
        for cls in joins:

            def join(self, timeout=None):
                clock.join(self, timeout)

            saved.append((cls, "join", cls.__dict__.get("join")))
            cls.join = join
        yield clock
    finally:
        for owner, name, original in reversed(saved):
            if original is None:
                delattr(owner, name)
            else:
                setattr(owner, name, original)
