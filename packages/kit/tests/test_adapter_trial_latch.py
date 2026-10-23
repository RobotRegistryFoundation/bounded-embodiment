# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""The kit adapters' EV-02/EV-05 methods: trigger_stop and reset_stop refuse while the runner's
stop is latched, looking before taking the bus lock and again under it (the watchdog may trip the
latch in between), and a wrapper's resume() is told which stop to clear when it takes it."""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import _runner_fakes as fakes
import _trial_gates
from _trials_sim import ROVER_SPACE
from bounded_embodiment_kit.adapters.base import AdapterError, call_resume
from bounded_embodiment_kit.adapters.sim_rover import SimRoverAdapter


class _TripsUnderTheLock:
    """The bus lock of an adapter whose latch trips while a method waits for the bus."""

    def __init__(self, adapter):
        self.adapter, self.inner = adapter, adapter._bus_lock

    def __enter__(self):
        self.adapter._latched.set()
        return self.inner.__enter__()

    def __exit__(self, *exc):
        return self.inner.__exit__(*exc)


def _trip_under_the_lock(adapter, call) -> None:
    adapter._latched.clear()
    adapter._bus_lock = _TripsUnderTheLock(adapter)
    try:
        with pytest.raises(AdapterError, match="latched"):
            call()
    finally:
        adapter._bus_lock = adapter._bus_lock.inner
        adapter._latched.clear()


def test_the_kit_adapters_look_at_the_latch_again_under_the_bus_lock(tmp_path):
    """Review S7: the runner's look at the latch is check-then-act."""
    rover = SimRoverAdapter(command_space_path=ROVER_SPACE, realtime=False, estop_file=tmp_path / "e1")
    rover.open()
    _trip_under_the_lock(rover, lambda: rover.trigger_stop("stop_message"))
    assert rover.send({"linear": 0.5, "angular": 0.0}).applied  # its own stop was never issued
    _trip_under_the_lock(rover, lambda: rover.reset_stop("stop_message"))
    rover.close()

    arm = fakes.sim_adapter(wrapper="_trial_gates:stoppable", estop_file=tmp_path / "e2")
    arm.open()
    gate = arm._target
    _trip_under_the_lock(arm, lambda: arm.trigger_stop("stop_message"))
    _trip_under_the_lock(arm, lambda: arm.reset_stop("stop_message"))
    _trip_under_the_lock(arm, lambda: arm.reset_stop("power_loss"))
    assert "stop" not in gate.events and "resume" not in gate.events  # nothing reached the gate
    arm.close()


def test_resume_is_told_the_source_when_it_takes_one():
    """Review N5: resume(source) when the wrapper's resume takes it, else resume()."""
    seen = []

    class Plain:
        def resume(self):
            seen.append("plain")

    class Told:
        def resume(self, source):
            seen.append(source)

    call_resume(Plain(), "stop_message")
    call_resume(Told(), "heartbeat_loss")
    call_resume(_trial_gates.TrialGate(object(), {}), "stop_message")  # the test gate: plain
    assert seen == ["plain", "heartbeat_loss"]


def test_the_lerobot_adapter_tells_a_wrappers_resume_which_stop_to_clear(tmp_path):
    """Round-2 K6: lerobot's reset_stop goes through call_resume, so a wrapper whose resume has a
    source parameter is told which stop to clear (a reset_stop calling resume() without it failed
    no test)."""
    arm = fakes.sim_adapter(wrapper="_trial_gates:stoppable", estop_file=tmp_path / "e")
    arm.open()
    told = []
    arm._target.resume = lambda source=None: told.append(source)
    try:
        arm.reset_stop("stop_message")
    finally:
        arm.close()
    assert told == ["stop_message"]


def test_resume_is_told_the_source_by_name_only():
    """Round-2 K7: the source went to whatever the first positional parameter was, so a gate
    written as resume(self, clear_all_holds=False) got clear_all_holds="stop_message" (truthy, the
    opposite of N5's intent), and resume(self, *, source=None) was never told. It is now passed as
    source=, and only to a resume that has a parameter called source, or takes **kwargs."""
    seen = []

    class KeywordOnly:
        def resume(self, *, source=None):
            seen.append(("keyword-only", source))

    class AnyKeyword:
        def resume(self, **kw):
            seen.append(("**kw", kw))

    class OtherFirst:
        def resume(self, clear_all_holds=False):
            seen.append(("other", clear_all_holds))

    class Positional:
        def resume(self, *args):
            seen.append(("*args", args))

    for gate in (KeywordOnly(), AnyKeyword(), OtherFirst(), Positional()):
        call_resume(gate, "stop_message")
    assert seen == [("keyword-only", "stop_message"), ("**kw", {"source": "stop_message"}),
                    ("other", False), ("*args", ())]
