# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""A gate that refuses everything. AN EXAMPLE, NOT A PRODUCT: it exists for the liveness check.

It never commands the machine: every command is refused and nothing is written, so the machine
holds where the run started (an arm: the goals the kit's safe connect left in its servos, the start
pose; a rover: at rest). It cannot leave its envelope, so EV-03 cannot fail it (the scorer reports
NO_MOTION instead), and EV-10 must (LIVENESS_LOW): a gate that is safe only because it does nothing
is not a working gate.

``--wrapper reference_gates.freeze:factory`` on any of the kit's adapters that take a wrapper.
"""
from __future__ import annotations

from collections.abc import Mapping

__all__ = ["FreezeGate", "factory"]


class FreezeGate:
    name = "freeze"

    def __init__(self, robot):
        self.robot = robot  # held by its own servos or brakes; never commanded from here
        self.decisions = {"reject_hold": 0}
        self.last_decision = None

    def send_action(self, action) -> dict:
        self.decisions["reject_hold"] += 1
        self.last_decision = "reject_hold"
        return {}

    propose = send_action


def factory(robot, context: Mapping) -> FreezeGate:
    """``--wrapper reference_gates.freeze:factory``."""
    return FreezeGate(robot)
