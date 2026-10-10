# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""DRAFT — requires line-by-line human review before use on hardware.

Forward kinematics for the FK tripwire and the state CSV: the robot's reported joints, through the
lab's declared joint map and the URDF, to the envelope's points.

It uses ``bounded_embodiment_kit.kinematics`` (``URDFModel``, ``JointMap``, ``envelope_points``)
and adds one safety rule: every actuated URDF joint on the way to an envelope point must be in
the robot's state, in degrees or radians. ``URDFModel.fk`` treats a missing joint as 0, which would
put a point somewhere plausible and wrong; here a missing joint is an error instead.
"""
from __future__ import annotations

import math
from collections.abc import Mapping
from pathlib import Path

from bounded_embodiment_kit.kinematics import JointMap, URDFModel, envelope_points

__all__ = ["ArmFK", "FKError"]


class FKError(ValueError):
    """FK cannot be computed from this state (a joint is missing or in the wrong unit)."""


class ArmFK:
    """Envelope points from joint state. Pure and thread-safe after construction."""

    def __init__(self, model: URDFModel, joint_map: JointMap, points: Mapping[str, tuple]):
        if not points:
            raise FKError("the envelope has no point with a link, so there is nothing to compute")
        self.model, self.joint_map = model, joint_map
        self.named = dict(points)
        needed = set()
        for pid, (link, _) in self.named.items():
            try:
                chain = model.chain(link)
            except KeyError:
                raise FKError(f"point {pid!r}: link {link!r} is not in the URDF") from None
            needed.update(j.name for j in chain if j.actuated)
        self.required = sorted(needed)

    @classmethod
    def build(cls, urdf_path: str | Path, joint_map: Mapping | None, envelope) -> ArmFK:
        """From the URDF file, the kit config's joint_map and a loaded envelope."""
        model = URDFModel.from_file(urdf_path)
        if envelope.frame != model.root:
            # FK gives points in the URDF's root frame; v0.1 has no transform between frames.
            raise FKError(f"the envelope's frame is {envelope.frame!r} but the URDF's root link is "
                          f"{model.root!r}; FK points would be in the wrong frame")
        return cls(model, JointMap.from_dict(joint_map), envelope_points(envelope.normalized))

    def urdf_values(self, joints: Mapping[str, float], units: Mapping[str, str]) -> dict[str, float]:
        out = {}
        for name in self.required:
            if name not in joints:
                raise FKError(f"the robot state has no {name!r}, which the URDF needs")
            value, unit = joints[name], units.get(name)
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise FKError(f"{name!r} = {value!r} is not a finite number")
            if unit == "deg":
                out[name] = self.joint_map.to_urdf({name: value})[name]
            elif unit == "rad":  # the joint map is written in degrees
                out[name] = self.joint_map.to_urdf({name: math.degrees(value)})[name]
            else:
                raise FKError(f"{name!r} is in {unit!r}; FK takes deg or rad joints")
        return out

    def points(self, joints: Mapping[str, float], units: Mapping[str, str]) -> dict[str, tuple]:
        """``{point id: (x, y, z)}`` in metres in the envelope frame (the URDF root frame)."""
        q = self.urdf_values(joints, units)
        p = self.model.points(self.named, q)
        return {pid: (float(v[0]), float(v[1]), float(v[2])) for pid, v in p.items()}
