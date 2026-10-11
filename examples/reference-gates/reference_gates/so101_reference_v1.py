# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""The first reference gate, kept to reproduce its failure: A KNOWN-BAD CONFIGURATION, NOT A GATE.

Ported from ev03-harness ``ev03/arm/legacy/gate_v1.py`` (as run on the real arm on 2026-10-08).
It planned every command from the MEASURED pose and held by re-sending the measured pose. Under
gravity a position servo settles past its goal, so each re-anchor lowered the goal by the sag and
the arm crept out of the envelope while the gate reported holds and clamps: out of the box in under
6 s on bob, after passing in simulation with ideal servos. ``so101_reference.py`` is the fix.

Differences from ``ReferenceArmGate``, all of them v1's: it plans from the encoder reading, not
from its own last goals; it holds by re-sending that reading; it checks the tool point only (no
links, no predicted offsets, no lead limits); and it sends degrees without the quarter-tick
allowance, so LeRobot's ``int()`` can floor a goal a tick below the intended one.

``--wrapper reference_gates.so101_reference_v1:factory``.
"""
from __future__ import annotations

import time
from collections.abc import Mapping

import numpy as np
from bounded_embodiment import envelope as envelope_mod

from ._context import load_kit_config
from .so101_reference import (
    ARM_JOINTS,
    DEFAULT_SPEED_FACTOR,
    DEFAULT_TOLERANCE_M,
    FRACTIONS,
    ArmKinematics,
    ReferenceArmGate,
)

__all__ = ["ReferenceArmGateV1", "factory"]


class ReferenceArmGateV1(ReferenceArmGate):
    """ev03-harness ``ReferenceArmGateV1``: the same clamps, budget and checked line as
    ``ReferenceArmGate``, all of it planned from the measured pose (module docstring)."""

    name = "reference gate v1 (plans from the measured pose)"

    def _send(self, joints):
        return self.robot.send_action({f"{m}.pos": v for m, v in joints.items()})

    def propose(self, action):
        now = self.clock()
        dt = 0.02 if self.last_t is None else min(max(now - self.last_t, 0.0), 0.04)
        self.last_t = now
        present = self._present()
        self.last_present = present
        target = self._valid(action)
        if target is None:
            return self._decide("invalid_hold", present)
        goal = dict(present)
        for j, v in target.items():
            if j == "gripper":
                goal[j] = min(100.0, max(0.0, v))
                continue
            step = self.max_step
            if j in self.joint_speed_dps:
                step = min(step, self.joint_speed_dps[j] * dt)
            _, _, lo, hi = self.ticks[j]
            v = min(hi, max(lo, v))
            goal[j] = present[j] + max(-step, min(step, v - present[j]))
        capped = any(abs(goal[j] - v) > 1e-9 for j, v in target.items())
        steps = [self._executed(present, {j: present[j] + u * (goal[j] - present[j]) for j in ARM_JOINTS})
                 for u in FRACTIONS]
        P = self.kin.link_points([present] + steps)[self.tool]
        clear = [self.env.clearance(self.tool, p) for p in P.tolist()]
        disp = np.linalg.norm(P[1:] - P[0], axis=1)
        rate = self.max_tool_speed_mps * self.speed_factor
        self.budget = min(rate * 0.02, self.budget + rate * dt)
        k = 0
        while k < len(FRACTIONS):
            c = clear[1 + k]
            if not ((c >= self.margin or c >= clear[0] - 1e-9) and disp[k] <= self.budget):
                break
            k += 1
        if k == 0:
            return self._decide("reject_hold", present)
        self.budget = max(0.0, self.budget - float(disp[k - 1]))
        q = dict(steps[k - 1])
        q["gripper"] = goal["gripper"]
        exact = k == len(FRACTIONS) and not capped and all(
            abs(q[j] - goal[j]) <= 360 / self.ticks[j][1] + 1e-9 for j in ARM_JOINTS)
        if exact:
            return self._decide("allow", q)
        wanted_motion = any(abs(goal[j] - present[j]) > 1e-9 for j in ARM_JOINTS)
        moved = any(abs(q[j] - present[j]) > 1e-9 for j in ARM_JOINTS)
        return self._decide("clamp" if moved or not wanted_motion else "reject_hold", q)

    send_action = propose


def factory(robot, context: Mapping) -> ReferenceArmGateV1:
    """``--wrapper reference_gates.so101_reference_v1:factory`` (see ``so101_reference.factory``)."""
    if not context.get("envelope_path") or not context.get("urdf_path"):
        raise ValueError("the v1 gate needs the run's envelope and URDF (--envelope, --urdf)")
    envelope = envelope_mod.load(context["envelope_path"])
    kit = load_kit_config(context)
    kin = ArmKinematics.from_files(context["urdf_path"], kit.get("joint_map"), envelope)
    speed_factor = context.get("speed_factor")
    return ReferenceArmGateV1(robot, envelope, kin, time.monotonic,
                              tolerance_m=float(kit.get("tolerance_m", DEFAULT_TOLERANCE_M)),
                              speed_factor=DEFAULT_SPEED_FACTOR if speed_factor is None else float(speed_factor))

