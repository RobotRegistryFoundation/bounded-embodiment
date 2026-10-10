# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""The stop ratchet (plan lesson 3): repeated stops must not walk a sagging arm down.

Simulated SO-101 with the gravity-sag model fitted to bob, driven through the real lerobot adapter
code (sim-so101). No real-time thread: the test steps the servos itself, so it is deterministic."""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from _runner_fakes import (
    BOB_CAL,
    URDF,
    dist,
    envelope_path,
    kit,
    sim_adapter,
)
from bounded_embodiment import envelope as envelope_mod
from bounded_embodiment_kit.runner.kin import ArmFK

CASES = [("so101", None), ("bob", BOB_CAL)]


def _tool(adapter, fk) -> tuple:
    joints = adapter.true_joints()
    units = {j: ("percent" if j == "gripper" else "deg") for j in joints}
    return fk.points(joints, units)["tool"]


def _opened(name, cal, **kw):
    adapter = sim_adapter(name, calibration=cal, record=True, **kw)
    adapter.open()
    adapter.step(1.0)  # settle after connect (the sag shows up once torque is on)
    fk = ArmFK.build(URDF, kit(name).get("joint_map"), envelope_mod.load(envelope_path(name)))
    return adapter, fk


@pytest.mark.parametrize(("name", "cal"), CASES)
def test_fifty_stops_move_the_tool_less_than_2_mm(name, cal):
    adapter, fk = _opened(name, cal)
    start = prev = _tool(adapter, fk)
    path = 0.0
    adapter.latch_stop()
    for _ in range(50):
        adapter.execute_stop()
        adapter.step(0.02)  # the command thread re-sends the stop every 20 ms
        now = _tool(adapter, fk)
        path += dist(prev, now)
        prev = now
    adapter.step(1.0)  # and nothing more happens afterwards
    end = _tool(adapter, fk)
    path += dist(prev, end)
    assert dist(start, end) < 0.002
    assert path < 0.002
    adapter.close()


@pytest.mark.parametrize(("name", "cal"), CASES)
def test_every_stop_resends_the_goals_chosen_at_the_first(name, cal):
    adapter, _ = _opened(name, cal)
    bus = adapter.sim_robot.bus
    adapter.latch_stop()
    n = len(bus.calls)
    for _ in range(10):
        adapter.execute_stop()
        adapter.step(0.02)
    writes = [c.values for c in bus.calls[n:] if c.method == "sync_write" and c.register == "Goal_Position"]
    reads = [c for c in bus.calls[n:] if c.method == "sync_read" and c.register == "Present_Position"]
    assert len(writes) == 10
    assert all(w == writes[0] for w in writes)  # the same goals every time
    assert len(reads) == 1  # the present position is read once per latch, at the first stop
    adapter.close()


@pytest.mark.parametrize(("name", "cal"), CASES)
def test_reanchoring_every_stop_is_the_ratchet_this_test_catches(name, cal):
    """The same 50 stops, re-anchored to the present position each time (hw.py's hold(), and the
    gateway's estop before so-arm101-actuator PR #7): the model must show the ratchet, or the test
    above would prove nothing."""
    adapter, fk = _opened(name, cal)
    bus = adapter.sim_robot.bus
    start = _tool(adapter, fk)
    for _ in range(50):
        present = bus.sync_read("Present_Position", normalize=False)
        bus.sync_write("Goal_Position", present, normalize=False)
        adapter.step(0.02)
    adapter.step(1.0)
    assert dist(start, _tool(adapter, fk)) > 0.01
    adapter.close()


def test_a_stop_of_a_moving_arm_settles_inside_the_acknowledgement_window():
    """Stopping mid-move: the tool must be slower than 20 mm/s within 0.25 s of the stop (the
    watchdog's acknowledgement rule) in the model, or every stop on hardware would escalate."""
    adapter, fk = _opened("bob", BOB_CAL)
    pose = adapter.true_joints()
    for i in range(10):  # a gate moving the arm in 3 deg steps at 50 Hz
        adapter.send({"shoulder_lift.pos": pose["shoulder_lift"] + 3 * (i + 1),
                      "elbow_flex.pos": pose["elbow_flex"] - 3 * (i + 1)})
        adapter.step(0.02)
    adapter.latch_stop()
    adapter.execute_stop()
    adapter.step(0.15)
    a = _tool(adapter, fk)
    adapter.step(0.1)
    b = _tool(adapter, fk)
    assert dist(a, b) / 0.1 < 0.020
    adapter.close()
