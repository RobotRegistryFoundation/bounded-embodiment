# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""The CUT POWER escalation (from the review of PR #12: N2 and N7, on main's own path): a slow
escalation command is still recorded, and the command runs in a session of its own with its
streams closed."""
from __future__ import annotations

import dataclasses
import os
import sys
import threading
import time
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import _runner_fakes as fakes
from bounded_embodiment import envelope as envelope_mod
from bounded_embodiment.generator import Proposal
from bounded_embodiment_kit.runner import run as runmod
from bounded_embodiment_kit.runner.limits import DEFAULT
from bounded_embodiment_kit.runner.session import make_clock
from bounded_embodiment_kit.runner.watchdog import Escalation

FAST = dataclasses.replace(DEFAULT, stop_hold_s=0.3, stop_hold_max_s=1.0, stop_stuck_s=0.5,
                           feed_start_timeout_s=2.0)


class _SlowRun:
    """subprocess.run that takes ``seconds`` and succeeds, recording its keyword arguments."""

    def __init__(self, seconds):
        self.seconds, self.kwargs = seconds, []

    def __call__(self, argv, **kw):
        self.kwargs.append(kw)
        time.sleep(self.seconds)
        return types.SimpleNamespace(returncode=0)


def test_a_slow_escalation_command_is_still_recorded():
    """Review N2: the escalation command took longer than the 2 s the teardown waited for the
    watchdog, so declared.json's end.stop.escalation was null although the command ran."""
    clock = make_clock()
    adapter = fakes.FakeAdapter(clock)
    feed = fakes.ScriptedFeed(clock)
    pump = fakes.FeedPump(feed, lambda t: (0.25, -0.15 + 0.1 * t, 0.15))  # keeps moving after the stop
    pump.start()
    live = runmod._Live(adapter=adapter, envelope=envelope_mod.load(fakes.envelope_path("so101")), fk=None,
                        fk_tripwire=False, feed=feed, mode="shakedown", tool_point="tool",
                        proposals=[Proposal(i, i / 50.0, "uniform", {"shoulder_pan.pos": 0.0})
                                   for i in range(500)],
                        files=None, clock=clock, estop_cmd="relay --off", limits=FAST, log=lambda m: None)
    live.escalation.run = _SlowRun(4.5)  # within the 5 s timeout, but past the 2 s teardown wait
    live.escalation.alarm = fakes.FakeAlarm()
    threading.Timer(0.6, lambda: live.latch.trip("operator", "test")).start()
    try:
        result = live.run(lambda q: "RUN", "RUN", "test", wait_still=False)
    finally:
        pump.quit.set()
    live.teardown(result)
    escalation = live.watchdog.outcome.escalation
    assert escalation is not None and escalation["returncode"] == 0 and escalation["error"] is None


def test_the_escalation_command_runs_in_a_session_of_its_own_with_its_streams_closed(tmp_path):
    """Review N7: in the operator's process group, a Ctrl+C at the bench also interrupted the
    escalation command; and with its output captured, a command that leaves a child behind held
    the watchdog until the timeout and was recorded as failed."""
    marker = tmp_path / "sid"
    alarm = fakes.FakeAlarm()
    write_sid = f"import os; open({str(marker)!r}, 'w').write(str(os.getsid(0)))"
    assert Escalation(f'{sys.executable} -c "{write_sid}"', alarm, timeout_s=2.0).fire("test")["returncode"] == 0
    assert int(marker.read_text()) != os.getsid(0)
    t0 = time.monotonic()
    left = Escalation("sh -c 'echo cut; sleep 30 &'", alarm, timeout_s=2.0).fire("test")
    assert time.monotonic() - t0 < 1.0
    assert left["returncode"] == 0 and left["error"] is None
