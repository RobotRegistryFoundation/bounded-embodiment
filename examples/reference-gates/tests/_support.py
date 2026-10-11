# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""The kit's simulated SO-101 and rover, driven in lockstep (no threads, no runner, simulated
time), for the gates' own tests."""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
GATES = HERE.parent
REPO = HERE.parents[2]
if str(GATES) not in sys.path:
    sys.path.insert(0, str(GATES))

from bounded_embodiment import envelope as envelope_mod
from bounded_embodiment_kit.adapters.sim_rover import SimRover
from bounded_embodiment_kit.adapters.sim_so101 import SimSO101Adapter
from bounded_embodiment_kit.kinematics import JointMap, URDFModel

EXAMPLES = REPO / "examples"
SO101_ENVELOPE = EXAMPLES / "envelopes" / "so101.envelope.json"
SO101_KIT = EXAMPLES / "envelopes" / "so101.kit.json"
SO101_SPACE = EXAMPLES / "command-spaces" / "so101.command-space.json"
ROVER_ENVELOPE = EXAMPLES / "envelopes" / "rover.envelope.json"
ROVER_SPACE = EXAMPLES / "command-spaces" / "rover.command-space.json"
URDF = EXAMPLES / "robots" / "so101" / "so101_new_calib.urdf"
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
SERVOS = {"ideal": {"sag": 0.0, "tau_s": 0.0}, "sag": {"sag": 1.0, "tau_s": 0.1}}


def kit_config() -> dict:
    return json.loads(SO101_KIT.read_text(encoding="utf-8"))


def so101_context(**overrides) -> dict:
    """What the kit hands a wrapper factory for the SO-101 example (``wrapper_context()``)."""
    return {"mode": "full", "speed_factor": None, "envelope_path": str(SO101_ENVELOPE),
            "command_space_path": str(SO101_SPACE), "kit_config_path": str(SO101_KIT),
            "urdf_path": str(URDF), **overrides}


def rover_context(**overrides) -> dict:
    return {"mode": "full", "speed_factor": None, "envelope_path": str(ROVER_ENVELOPE),
            "command_space_path": str(ROVER_SPACE), "kit_config_path": None, "urdf_path": None,
            **overrides}


class Clock:
    """A gate's clock in lockstep: simulated seconds."""

    def __init__(self):
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


class Arm:
    """The kit's simulated SO-101 at the example's start pose, connected by the kit's own safe
    connect, with a gate built by ``factory`` around it on a lockstep clock.

    ``powered=True`` starts it as ev03-harness's simulator did (torque on, every goal register at
    the start pose, the joints settled past them by the sag), so the kit's safe connect finds it
    holding and leaves the goals alone; ``False`` is bob at power-up (torque off, goals 0), which
    the safe connect syncs to the present (sagged) position."""

    def __init__(self, factory=None, *, servos: str = "ideal", context=None, start=None,
                 powered: bool = True):
        kit = kit_config()
        self.adapter = SimSO101Adapter(command_space_path=SO101_SPACE, urdf_path=URDF,
                                       joint_map=JointMap.from_dict(kit.get("joint_map")),
                                       start_pose=start or kit["start_pose_deg"],
                                       power_up_goals_zero=not powered, realtime=False,
                                       prompt=lambda q: "", log=lambda m: None, **SERVOS[servos])
        self.adapter.open()
        self.robot = self.adapter.sim_robot
        self.servos = self.adapter.servos
        self.envelope = envelope_mod.load(SO101_ENVELOPE)
        self.model = URDFModel.from_file(URDF)
        self.clock = Clock()
        self.gate = None
        if factory is not None:
            self.gate = factory(self.robot, context or so101_context())
            if hasattr(self.gate, "clock"):
                self.gate.clock = self.clock

    def joints(self) -> dict[str, float]:
        """The true joints (LeRobot degrees; gripper 0..100)."""
        return self.adapter.true_joints()

    def point(self, pid: str = "tool") -> tuple[float, float, float]:
        """An envelope point of the true arm (the example's joint map is one to one)."""
        link = self.envelope.points[pid].link
        q = {j: math.radians(v) for j, v in self.joints().items() if j in ARM_JOINTS}
        return tuple(float(v) for v in self.model.point(link, None, q))

    def clearance(self) -> float:
        """The smallest clearance of any zoned point of the true arm (negative: outside)."""
        return min(self.envelope.clearance(p, self.point(p)) for p in ("tool", "elbow", "wrist"))

    def send(self, action: dict):
        return (self.gate or self.robot).send_action(dict(action))

    def run(self, seconds: float, dt: float = 0.001) -> None:
        for _ in range(round(seconds / dt)):
            self.servos.step(dt)
            self.clock.t += dt


class Rover:
    """The kit's simulated rover at (1, 1) heading +x (the adapter's default start), with a gate
    built by ``factory``."""

    def __init__(self, factory=None, context=None, start=(1.0, 1.0, 0.0)):
        self.rover = SimRover(start=start)
        self.envelope = envelope_mod.load(ROVER_ENVELOPE)
        self.gate = factory(self.rover, context or rover_context()) if factory else None

    def send(self, cmd):
        return (self.gate or self.rover).send_action(cmd)

    def clearance(self) -> float:
        x, y, _ = self.rover.pose()
        return self.envelope.clearance("base", (x, y, 0.0))

    def run(self, seconds: float, dt: float = 0.005) -> None:
        for _ in range(round(seconds / dt)):
            self.rover.tick(dt)
