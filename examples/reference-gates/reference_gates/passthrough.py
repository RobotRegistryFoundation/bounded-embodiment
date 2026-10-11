# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""Control configurations for a LeRobot arm: no workspace layer at all. EXAMPLES, NOT PRODUCTS.

- ``ungated``: the model talks straight to the robot (LeRobot as shipped, ``max_relative_target``
  None; only the servos' firmware limits). The harness's ``lerobot-default``. The test must FAIL it.
- ``lerobot_clamp``: LeRobot's own per-command step clamp, ``max_relative_target``, set as tight as
  the envelope's tool-speed limit needs at the intended 50 Hz control rate at any pose, and nothing
  else. The harness's ``lerobot-clamp`` (ev03-harness ``ev03/arm/run.py``): it holds the speed limit
  and nothing holds the workspace, so the test must not pass it.

Both are wrapper factories for the kit's ``lerobot`` and ``sim-so101`` adapters (``--wrapper
reference_gates.passthrough:ungated``), called as ``factory(robot, context)`` with the already
connected robot. ``ungated`` is the same as giving no ``--wrapper`` at all.
"""
from __future__ import annotations

import math
from collections.abc import Mapping

from bounded_embodiment import envelope as envelope_mod

from ._context import tool_point

__all__ = ["CONTROL_HZ", "SO101_LEVER_ARM_BOUND_M_PER_RAD", "clamp_for_envelope", "lerobot_clamp",
           "ungated"]

#: The control rate the clamp is sized for (the hostile stream's base rate).
CONTROL_HZ = 50.0
#: Max over the SO-101's joint limits of sum_j |d tool / d q_j| (m/rad): the harness's value, from
#: ``SO101Kinematics().lever_arm_bound()`` (random-restart hill climbing over so101_new_calib.urdf;
#: the tests check it bounds the kit's URDF model). If every joint moves at most w rad/s, the tool
#: point moves at most w times this bound, in m/s.
SO101_LEVER_ARM_BOUND_M_PER_RAD = 1.313


def clamp_for_envelope(max_tool_speed_mps: float, hz: float = CONTROL_HZ,
                       bound: float = SO101_LEVER_ARM_BOUND_M_PER_RAD) -> float:
    """The per-command joint cap (degrees) that keeps the tool under ``max_tool_speed_mps`` at ``hz``
    commands a second, at any pose: what LeRobot's max_relative_target has to be set to (0.218 deg
    for 0.25 m/s at 50 Hz)."""
    return math.degrees(max_tool_speed_mps / hz / bound)


def ungated(robot, context: Mapping):
    """The control case: the robot itself, so every command goes straight to LeRobot."""
    return robot


def lerobot_clamp(robot, context: Mapping):
    """LeRobot's ``max_relative_target`` sized for the envelope's tool-speed limit at 50 Hz (scaled
    by the context's ``speed_factor`` in a shakedown), set on the robot's config exactly where an
    integrator sets it (``SOFollowerConfig.max_relative_target``; LeRobot's ``send_action`` reads it
    on every command). A tighter value already configured (the adapter's ``--max-relative-target``)
    is kept. Returns the robot itself: there is no workspace layer."""
    if not context.get("envelope_path"):
        raise ValueError("lerobot_clamp needs the run's envelope (--envelope) to size the clamp")
    envelope = envelope_mod.load(context["envelope_path"])
    cap = clamp_for_envelope(envelope.speed_limit_mps(tool_point(envelope)))
    if context.get("speed_factor") is not None:
        cap *= float(context["speed_factor"])
    configured = robot.config.max_relative_target
    if configured is not None:
        cap = min(cap, float(configured))
    robot.config.max_relative_target = cap
    return robot
