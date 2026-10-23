# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""A run that loses its terminal (round-2 review of #13, items 2 and 3): SIGHUP (an ssh session
that drops) and SIGTERM stop the run as Ctrl+C does, through the latch, so the stop phase runs and
the end record is written; the logger survives a terminal that is gone; and a typed word that
cannot be asked is logged with its reason. On the fakes of ``_runner_fakes``: no robot, port or
network."""
from __future__ import annotations

import dataclasses
import io
import os
import signal
import sys
import threading
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import _runner_fakes as fakes
from bounded_embodiment import envelope as envelope_mod
from bounded_embodiment.generator import Proposal
from bounded_embodiment_kit.runner import run as runmod
from bounded_embodiment_kit.runner import session
from bounded_embodiment_kit.runner.interlocks import InterlockError
from bounded_embodiment_kit.runner.limits import DEFAULT
from bounded_embodiment_kit.runner.session import make_clock

FAST = dataclasses.replace(DEFAULT, stop_hold_s=0.3, stop_hold_max_s=1.0, stop_stuck_s=0.5,
                           feed_start_timeout_s=2.0)


@pytest.mark.parametrize(("name", "why"), [("SIGHUP", "hangup (SIGHUP): the terminal was lost"),
                                            ("SIGTERM", "SIGTERM")])
def test_a_hangup_or_sigterm_stops_the_run_as_ctrl_c_does(name, why):
    """Item 3: the runner handled only SIGINT, so the SIGHUP of a dropped ssh session ended the
    process at once: no stop was executed, and declared.json's end stayed null."""
    sig = getattr(signal, name)
    ours = signal.signal(sig, lambda *a: None)  # a signal outside the run must not end pytest
    try:
        clock = make_clock()
        adapter = fakes.FakeAdapter(clock)
        feed = fakes.ScriptedFeed(clock)
        pump = fakes.FeedPump(feed, lambda t: (0.25, 0.0, 0.15))
        pump.start()
        logs = []
        live = runmod._Live(adapter=adapter, envelope=envelope_mod.load(fakes.envelope_path("so101")), fk=None,
                            fk_tripwire=False, feed=feed, mode="shakedown", tool_point="tool",
                            proposals=[Proposal(i, i / 50.0, "uniform", {"shoulder_pan.pos": 0.0})
                                       for i in range(500)],
                            files=None, clock=clock, estop_cmd=None, limits=FAST, log=logs.append)
        threading.Timer(0.6, lambda: os.kill(os.getpid(), sig)).start()
        try:
            result = live.run(lambda q: "RUN", "RUN", "test", wait_still=False)
        finally:
            pump.quit.set()
        live.teardown(result)
        assert (live.latch.kind, live.latch.reason) == ("operator", why)
        assert live.command.first_stop_t is not None  # the runner's own stop ran
        assert f"\nStopping ({why})." in logs
        assert signal.getsignal(sig) is not None and signal.getsignal(sig).__name__ == "<lambda>"  # restored
    finally:
        signal.signal(sig, ours)


def test_the_log_survives_a_terminal_that_is_gone(monkeypatch):
    """After a hangup, writing to the terminal raises EIO; the stop, the teardown and the end
    record must not depend on it."""

    class Gone(io.StringIO):
        def write(self, s):
            raise OSError(5, "Input/output error")

    monkeypatch.setattr(sys, "stderr", Gone())
    runmod._stderr("Stopping (hangup).")  # no exception


def test_a_typed_word_that_cannot_be_asked_is_logged_with_its_reason():
    """Item 2: _confirm_word swallowed typed_word's InterlockError as a bare cancel ("goto-start
    ended (None: None)"); the cancel is now logged with why."""

    def prompt(question):  # pragma: no cover - not asked: the terminal prompt fails first
        return ""

    def ask_word(word, text, **kw):
        raise InterlockError("cannot clear input typed ahead: no termios on this platform")

    prompt.ask_word = ask_word
    logs = []
    latch = session.StopLatch(lambda: 0.0)
    assert not session._confirm_word(prompt, latch, "GO", "test", logs.append)
    assert logs == ["Cannot ask for GO: cannot clear input typed ahead: no termios on this platform"]
