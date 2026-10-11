# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""The control configurations (``passthrough``: ungated, LeRobot's clamp) and the gate that refuses
everything (``freeze``), on the kit's simulated SO-101 in lockstep."""
from __future__ import annotations

import math
import random

import _support as S
import numpy as np
import pytest
from reference_gates import freeze, passthrough

FULL_REACH = {"shoulder_pan.pos": 0.0, "shoulder_lift.pos": 50.0, "elbow_flex.pos": -25.0,
              "wrist_flex.pos": -30.0, "wrist_roll.pos": 0.0, "gripper.pos": 30.0}


def test_the_freeze_gate_never_commands_the_arm():
    arm = S.Arm(freeze.factory, servos="sag")
    arm.adapter.sim_robot.bus.record = True
    before = arm.servos.degrees()
    for _ in range(100):
        arm.send(FULL_REACH)
        arm.run(0.02)
    assert arm.gate.decisions == {"reject_hold": 100}
    assert not [c for c in arm.robot.bus.calls if c.is_write]
    after = arm.servos.degrees()
    assert all(abs(after[m] - before[m]) < 0.05 for m in after)


def test_ungated_is_the_robot_itself_and_leaves_the_box():
    arm = S.Arm(passthrough.ungated)
    assert arm.gate is arm.robot
    for _ in range(100):
        arm.send(FULL_REACH)
        arm.run(0.02)
    assert arm.clearance() < -0.05


def test_the_lerobot_clamp_is_sized_for_the_tool_speed_limit():
    arm = S.Arm(passthrough.lerobot_clamp)
    cap = passthrough.clamp_for_envelope(0.25)
    assert cap == pytest.approx(0.2182, abs=1e-4)
    assert arm.robot.config.max_relative_target == pytest.approx(cap)
    half = S.Arm(passthrough.lerobot_clamp, context=S.so101_context(speed_factor=0.5, mode="shakedown"))
    assert half.robot.config.max_relative_target == pytest.approx(cap / 2)
    tighter = S.Arm()
    tighter.robot.config.max_relative_target = 0.1
    passthrough.lerobot_clamp(tighter.robot, S.so101_context())
    assert tighter.robot.config.max_relative_target == 0.1


def test_the_lerobot_clamp_limits_every_step_and_nothing_else():
    """LeRobot's own clamp, relative to the present position: every command moves each goal at
    most 0.218 deg, and nothing keeps the arm in its box (ideal servos: out in 30 s)."""
    arm = S.Arm(passthrough.lerobot_clamp)
    cap = arm.robot.config.max_relative_target
    tick = 360 / 4095
    out_at = None
    for k in range(1500):
        present = arm.servos.degrees()
        arm.send(FULL_REACH)
        goal = arm.robot.bus.sync_read("Goal_Position")
        # LeRobot clamps against the present position it reads (whole ticks) and truncates the
        # goal to whole ticks: at most two ticks of rounding on top of the cap.
        assert all(abs(goal[m] - present[m]) <= cap + 2 * tick for m in S.ARM_JOINTS)
        arm.run(0.02)
        if out_at is None and arm.clearance() < -0.01:
            out_at = k * 0.02
    assert out_at is not None and out_at < 30.0


def test_the_lever_arm_bound_bounds_the_kits_so101_model():
    """``SO101_LEVER_ARM_BOUND_M_PER_RAD`` (the harness's 1.313 m/rad) is at least the sum over the
    arm joints of |d tool / d q| at 2000 random poses inside the URDF's joint limits."""
    model = S.URDFModel.from_file(S.URDF)
    limits = model.limits()
    rng = random.Random(1)
    h = 1e-4
    poses = [{j: rng.uniform(*limits[j]) for j in S.ARM_JOINTS} for _ in range(2000)]
    configs = []
    for q in poses:
        configs.append(q)
        configs.extend(dict(q, **{j: q[j] + h}) for j in S.ARM_JOINTS)
    T = model.fk_batch({j: [c[j] for c in configs] for j in S.ARM_JOINTS})["gripper_frame_link"]
    p = T[:, :3, 3].reshape(len(poses), 1 + len(S.ARM_JOINTS), 3)
    lever = (np.linalg.norm(p[:, 1:] - p[:, :1], axis=2) / h).sum(axis=1)
    assert lever.max() <= passthrough.SO101_LEVER_ARM_BOUND_M_PER_RAD
    assert lever.max() > 0.9 * passthrough.SO101_LEVER_ARM_BOUND_M_PER_RAD
    assert math.isfinite(float(lever.max()))
