# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""The watchdog and the stop latch, with real threads: the watchdog never touches the bus, a trip
stops the command thread within one cycle, and an unacknowledged stop runs the e-stop command
and sounds the alarm."""
from __future__ import annotations

import argparse
import dataclasses
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import _runner_fakes as fakes
from _runner_fakes import (
    FakeAdapter,
    FakeAlarm,
    FakeRun,
    FeedPump,
    ScriptedFeed,
    envelope_path,
)
from bounded_embodiment import envelope as envelope_mod
from bounded_embodiment.generator import Proposal
from bounded_embodiment_kit import cli
from bounded_embodiment_kit.adapters import sim_so101
from bounded_embodiment_kit.runner import run as runmod
from bounded_embodiment_kit.runner.limits import DEFAULT
from bounded_embodiment_kit.runner.session import make_clock
from bounded_embodiment_kit.runner.watchdog import StopLatch

ENV = envelope_mod.load(envelope_path("so101"))
FAST = dataclasses.replace(DEFAULT, stop_hold_s=0.3, stop_hold_max_s=1.0, stop_stuck_s=0.5,
                           feed_start_timeout_s=2.0)


def test_the_latch_keeps_the_first_reason_and_calls_the_adapter_once():
    clock = make_clock()
    latch = StopLatch(clock)
    flags = []
    latch.on_trip(lambda: flags.append(threading.current_thread().name))
    assert latch.trip("tripwire", "first")
    assert not latch.trip("operator", "second")
    assert (latch.kind, latch.reason) == ("tripwire", "first") and len(flags) == 1


def _live(adapter, clock, *, feed=None, estop="relay --off 'arm 1'", proposals=None, limits=FAST):
    proposals = proposals if proposals is not None else [
        Proposal(i, i / 50.0, "uniform", {"shoulder_pan.pos": 0.0}) for i in range(500)]
    live = runmod._Live(adapter=adapter, envelope=ENV, fk=None, fk_tripwire=False, feed=feed,
                        mode="shakedown", tool_point="tool", proposals=proposals, files=None,
                        clock=clock, estop_cmd=estop, limits=limits, log=lambda m: None)
    live.escalation.run = FakeRun()
    live.escalation.alarm = FakeAlarm()
    return live


def _trip_after(live, seconds, kind="operator", reason="test"):
    threading.Timer(seconds, lambda: live.latch.trip(kind, reason)).start()


def _run(live):
    return live.run(lambda q: "RUN", "RUN", "test", wait_still=False)


def test_a_trip_stops_the_command_thread_within_one_cycle():
    clock = make_clock()
    adapter = FakeAdapter(clock)
    live = _live(adapter, clock)
    _trip_after(live, 0.5)
    result = _run(live)
    live.teardown(result)
    t_latch = live.latch.t_s
    after = [t for t, _ in adapter.sends if t > t_latch]
    assert len(after) <= 1  # at most the one already inside send()
    assert {name for _, name in adapter.sends} == {"be-command"}
    assert adapter.stops and adapter.stops[0][0] - t_latch < 0.02 + 0.03  # the next cycle (+ CI slack)
    assert {name for _, name in adapter.stops} == {"be-command"}
    assert result.status == "ended" and live.command.sent < 500


def test_an_unacknowledged_stop_runs_the_estop_command_and_sounds_the_alarm():
    clock = make_clock()
    adapter = FakeAdapter(clock)
    feed = ScriptedFeed(clock)
    pump = FeedPump(feed, lambda t: (0.25, -0.15 + 0.1 * t, 0.15))  # keeps moving after the stop
    pump.start()
    live = _live(adapter, clock, feed=feed)
    _trip_after(live, 0.6)
    try:
        result = _run(live)
    finally:
        pump.quit.set()
    live.teardown(result)
    outcome = live.watchdog.outcome
    assert outcome.decided and outcome.acknowledged is False
    assert outcome.t_s - live.latch.t_s >= DEFAULT.stop_ack_within_s
    assert live.escalation.run.calls == [["relay", "--off", "arm 1"]]  # shlex-split, no shell
    assert live.escalation.alarm.sounded and "100 mm/s" in live.escalation.alarm.sounded[0]
    assert runmod._exit_code(result, live) == runmod.EXIT_ESCALATED


def test_a_stop_the_instrument_sees_settle_is_acknowledged_without_escalation():
    clock = make_clock()
    adapter = FakeAdapter(clock)
    feed = ScriptedFeed(clock)
    live = _live(adapter, clock, feed=feed)
    frozen = {}

    def position(t):  # 0.1 m/s until the stop is executed, then still
        if adapter.stops:
            return frozen.setdefault("p", (0.25, -0.15 + 0.1 * t, 0.15))
        return (0.25, -0.15 + 0.1 * t, 0.15)

    pump = FeedPump(feed, position)
    pump.start()
    _trip_after(live, 0.6)
    try:
        result = _run(live)
    finally:
        pump.quit.set()
    live.teardown(result)
    outcome = live.watchdog.outcome
    assert outcome.acknowledged is True and outcome.source == "instrument"
    assert outcome.t_s - live.latch.t_s <= DEFAULT.stop_ack_within_s
    assert live.escalation.run.calls == [] and live.escalation.alarm.sounded == []


def test_a_stop_that_never_executes_escalates_and_the_session_reports_stuck():
    clock = make_clock()
    release = threading.Event()
    adapter = FakeAdapter(clock, block_send=release)  # the first send never returns: a stuck gate
    feed = ScriptedFeed(clock)
    pump = FeedPump(feed, lambda t: (0.25, 0.0, 0.15))
    pump.start()
    live = _live(adapter, clock, feed=feed)
    _trip_after(live, 0.3)
    try:
        result = _run(live)
        outcome = live.watchdog.outcome
        assert result.status == "stuck"
        assert outcome.acknowledged is False and "never executed" in live.escalation.alarm.sounded[0]
        assert live.escalation.run.calls  # the lab's e-stop ran
        closed = live.teardown(result)
        assert closed.startswith("not closed")  # close() skipped: the bus is held by the stuck send
    finally:
        release.set()
        pump.quit.set()


def test_a_watchdog_fault_trips_the_latch():
    clock = make_clock()

    class BrokenFeed(ScriptedFeed):
        def snapshot(self):
            raise RuntimeError("tracker crashed")

    adapter = FakeAdapter(clock)
    live = _live(adapter, clock, feed=BrokenFeed(clock))
    live.watchdog.start()
    assert live.latch.wait(1.0)
    live.watchdog.stop()
    assert live.latch.kind == "error" and "watchdog fault" in live.latch.reason
    assert adapter.latched.is_set()  # the latch told the adapter (a flag only)


def test_the_watchdog_thread_never_calls_the_bus(tmp_path, monkeypatch):
    """A whole simulated control run through the CLI path, ended by a tripwire; the simulated bus
    logs the thread of every call."""
    made = []
    real_from_args = sim_so101.from_args

    def recording_from_args(args, context):
        adapter = real_from_args(args, context)
        adapter.record_bus_calls = True
        made.append(adapter)
        return adapter

    monkeypatch.setattr(sim_so101, "from_args", recording_from_args)
    p = argparse.ArgumentParser(prog="be")
    cli.register(p.add_subparsers(dest="command"))
    args = p.parse_args([
        "run", "ev03", "--mode", "control", "--adapter", "sim-so101",
        "--envelope", str(envelope_path("so101")), "--command-space", str(fakes.SO101_SPACE),
        "--kit-config", str(fakes.ENVELOPES / "so101.kit.json"), "--urdf", str(fakes.URDF),
        "--instrument", "sim", "--out", str(tmp_path / "run"), "--target", "full_reach_forward",
        "--step-cap", "none", "--max-relative-target", "1.0", "--i-accept-the-risk",
        "--hardware-stop", "none (simulation)", "--ledger", str(tmp_path / "ledger.jsonl")])
    answers = iter(["RUN", ""])
    code = runmod.execute_run(args, prompt=lambda q: next(answers), log=lambda m: None)
    assert code in (runmod.EXIT_STOPPED, runmod.EXIT_ESCALATED)  # stopped by a tripwire
    calls = made[0].sim_robot.bus.calls
    threads = {c.thread for c in calls}
    assert "be-watchdog" not in threads
    assert "be-command" in threads and "MainThread" in threads
    stop_writes = [c for c in calls if c.thread == "be-command" and c.method == "sync_write"
                   and c.register == "Goal_Position" and isinstance(next(iter(c.values.values())), int)]
    assert stop_writes, "the stop ran on the command thread, in raw ticks"


def test_latch_stop_does_not_wait_for_the_bus():
    """latch_stop() is a flag: the lerobot adapter's never takes the bus lock, so the watchdog
    can latch even while the command thread holds the bus."""
    adapter = fakes.sim_adapter()
    adapter.open()
    held = adapter._bus_lock.acquire()  # as if the command thread were stuck holding the bus
    try:
        t0 = time.monotonic()
        adapter.latch_stop()
        assert time.monotonic() - t0 < 0.05
        assert adapter._latched.is_set()
    finally:
        if held:
            adapter._bus_lock.release()
    adapter.close()
