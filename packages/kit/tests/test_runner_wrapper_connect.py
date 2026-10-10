# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""Safe connect and the lab's wrapper: goals are synced to present before any torque, and the
wrapper never sees connect / configure / disconnect.

The bus is the simulator's stand-in for LeRobot's Feetech bus, at bob's power-up state: torque off
and every goal register reading 0. Its call log records each register write in order."""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import _runner_fakes as fakes
from _runner_fakes import sim_adapter
from bounded_embodiment_kit.adapters import AdapterContext, AdapterError
from bounded_embodiment_kit.adapters.lerobot import build_follower

WRAPPER = "_runner_fakes:make_wrapper"


def _goal_writes(calls):
    return [(i, c) for i, c in enumerate(calls)
            if c.method in ("sync_write", "write") and c.register == "Goal_Position"]


def _torque_on(calls):
    return [i for i, c in enumerate(calls) if c.method in ("sync_write", "write")
            and c.register == "Torque_Enable" and any(v == 1 for v in c.values.values())]


def test_power_up_goals_read_zero_and_torque_is_off():
    adapter = sim_adapter(record=True)
    robot = adapter._make_robot()  # built, not connected: bob at power-up
    servos = adapter.servos
    assert all(g == 0 for g in servos.goal.values())
    assert all(t == 0 for t in servos.torque.values())
    assert robot.calls == []


def test_goals_are_synced_to_present_before_any_torque_and_the_wrapper_is_never_connected():
    fakes.WRAPPERS.clear()
    context = AdapterContext(command_space_path=fakes.SO101_SPACE,
                             envelope_path=fakes.envelope_path("so101"), mode="shakedown",
                             speed_factor=0.5)
    adapter = sim_adapter(record=True, wrapper=WRAPPER, wrapper_context=context.wrapper_context())
    space = adapter.open()
    assert space.kind == "joint_position"
    robot, bus, servos = adapter.sim_robot, adapter.sim_robot.bus, adapter.servos

    calls = bus.calls
    goals = _goal_writes(calls)
    torque = _torque_on(calls)
    assert goals and torque
    first_goal_i, first_goal = goals[0]
    # The first goal write is the sync, in raw ticks, to where every joint was at power-up.
    assert first_goal.method == "sync_write"
    reads = [c for c in calls[:first_goal_i] if c.method == "sync_read" and c.register == "Present_Position"]
    assert reads, "the present positions are read before the goals are written"
    # (No real-time thread: nothing has moved since power-up, so "present" is still present.)
    assert first_goal.values == {m: round(servos.present[m]) for m in servos.present}
    # No torque before the sync; the goals were read back before torque too.
    assert min(torque) > first_goal_i
    readback = [i for i, c in enumerate(calls) if c.method == "sync_read"
                and c.register == "Goal_Position" and i > first_goal_i]
    assert readback and readback[0] < min(torque)
    # LeRobot's own configure() ran on the INNER robot (it enables torque); connect() never did.
    assert robot.calls.count("configure") == 1 and "connect" not in robot.calls

    # The lab's wrapper got the connected robot and plain-data context, and only send_action.
    assert len(fakes.WRAPPERS) == 1
    wrapper = fakes.WRAPPERS[0]
    assert wrapper._robot is robot and robot.is_connected
    assert wrapper.context["mode"] == "shakedown" and wrapper.context["speed_factor"] == 0.5
    assert all(not callable(v) for v in wrapper.context.values())  # plain data only
    result = adapter.send({"shoulder_pan.pos": 1.0})
    assert result.applied and result.decision == "allow"
    adapter.latch_stop()
    adapter.execute_stop()
    adapter.close()
    for name in ("connect", "configure", "disconnect"):
        assert name not in wrapper.touched, f"the adapter touched the wrapper's {name}"
    # Closing never used LeRobot's robot.disconnect() (it would drop torque): only the bus, with
    # disable_torque=False, after the operator did not type OFF.
    assert "disconnect" not in robot.calls
    last = bus.calls[-1]
    assert last.method == "disconnect" and last.values is False
    assert all(t == 1 for t in servos.torque.values())


def test_the_hazard_the_sync_prevents():
    """LeRobot's own connect() on the same power-up state: torque comes on toward goal 0, and the
    joints run to their firmware limits."""
    adapter = sim_adapter(record=True)
    robot = adapter._make_robot()
    before = dict(adapter.servos.present)
    robot.connect()
    adapter.step(0.5)
    moved = max(abs(adapter.servos.present[m] - before[m]) for m in before)
    assert moved > 300  # ticks: tens of degrees


def test_a_lost_goal_write_stops_before_torque():
    adapter = sim_adapter(record=True)
    robot = adapter._make_robot()
    real_apply = robot.bus._apply

    def lossy(register, motor, raw):  # the sync_write packet is lost on the bus
        if register == "Goal_Position":
            return
        real_apply(register, motor, raw)

    robot.bus._apply = lossy
    adapter._make_robot = lambda: robot
    with pytest.raises(AdapterError, match="did not take"):
        adapter.open()
    assert _torque_on(robot.bus.calls) == []
    assert not robot.bus.is_connected  # the port was released, torque untouched


def test_an_arm_already_holding_keeps_its_goals():
    adapter = sim_adapter(record=True, power_up_goals_zero=False)  # torque on, goals near present
    adapter.open()
    first_torque_off = next(i for i, c in enumerate(adapter.sim_robot.bus.calls)
                            if c.register == "Torque_Enable")
    assert all(i > first_torque_off for i, _ in _goal_writes(adapter.sim_robot.bus.calls))
    adapter.close()


def test_a_failing_wrapper_factory_releases_the_port_with_torque_on():
    adapter = sim_adapter(record=True, wrapper="_runner_fakes:failing_factory")
    with pytest.raises(RuntimeError, match="failed to build"):
        adapter.open()
    bus = adapter.sim_robot.bus
    assert not bus.is_connected
    assert bus.calls[-1].method == "disconnect" and bus.calls[-1].values is False
    assert all(t == 1 for t in adapter.servos.torque.values())


def test_send_never_raises_and_the_latch_refuses_commands():
    adapter = sim_adapter()
    adapter.open()
    for bad in ({"shoulder_pan.pos": float("nan")}, {"shoulder_pan.pos": "1.0"}, {}, {"x.pos": 1.0},
                {"shoulder_pan.pos": True}, {"shoulder_pan.pos": None}):
        r = adapter.send(bad)
        assert not r.applied and r.error_kind.startswith("exception:")
    assert adapter.send({"shoulder_pan.pos": 2.0}).applied
    adapter.latch_stop()
    r = adapter.send({"shoulder_pan.pos": 3.0})
    assert (r.applied, r.error_kind) == (False, "stop_latched")
    adapter.close()


def test_other_lerobot_robots_are_refused_in_v01():
    with pytest.raises(AdapterError, match="refused in v0.1"):
        build_follower("koch_follower", "/dev/null", "x")


def test_the_lerobot_adapter_never_imports_lerobot_at_import_time():
    assert "lerobot" not in sys.modules
