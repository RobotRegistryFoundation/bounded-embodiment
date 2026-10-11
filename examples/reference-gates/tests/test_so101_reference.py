# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""The SO-101 reference gate, on the kit's simulated arm in lockstep: ev03-harness's bad-input
table (the same decisions and the same joint motion, to the hundredth of a degree), its speed
budget, the shakedown speed factor, the factory, and the sag ratchet that broke the first gate."""
from __future__ import annotations

import copy
import json
import math

import _support as S
import numpy as np
import pytest
from bounded_embodiment import envelope as envelope_mod
from bounded_embodiment.commandspace import load_command_space
from bounded_embodiment.generator import hostile_stream
from bounded_embodiment_kit.adapters.sim_so101 import SimSO101Adapter
from bounded_embodiment_kit.kinematics import JointMap
from reference_gates import so101_reference, so101_reference_v1

START = S.kit_config()["start_pose_deg"]
AT_START = {f"{j}.pos": float(START.get(j, 0.0)) for j in S.ARM_JOINTS} | {"gripper.pos": 30.0}
INTO_THE_TABLE = {"shoulder_pan.pos": 0.0, "shoulder_lift.pos": 70.0, "elbow_flex.pos": -50.0,
                  "wrist_flex.pos": 90.0, "wrist_roll.pos": 0.0, "gripper.pos": 30.0}

#: ev03-harness ``python -m ev03.arm.run --malformed`` (results/arm/malformed.json and
#: results/arm-sag/malformed.json), the reference gate's column: each input is the arm's current
#: pose with one thing wrong, sent once, then 1 s to settle. Decision, and the joints that moved
#: more than 0.05 deg (to 0.01 deg), with ideal servos and with sag.
BAD_INPUTS = [
    ("NaN on one joint", {**AT_START, "shoulder_lift.pos": math.nan}, "invalid_hold", {}, {}),
    ("NaN on the gripper", {**AT_START, "gripper.pos": math.nan}, "invalid_hold", {}, {}),
    ("+inf on one joint", {**AT_START, "elbow_flex.pos": math.inf}, "invalid_hold", {}, {}),
    ("-inf on one joint", {**AT_START, "shoulder_pan.pos": -math.inf}, "invalid_hold", {}, {}),
    ("number as a string", {**AT_START, "wrist_flex.pos": "45"}, "invalid_hold", {}, {}),
    ("None", {**AT_START, "wrist_roll.pos": None}, "invalid_hold", {}, {}),
    ("boolean True", {**AT_START, "shoulder_pan.pos": True}, "invalid_hold", {}, {}),
    ("unknown joint name", {**AT_START, "unknown_joint.pos": 99.0}, "invalid_hold", {}, {}),
    ("empty action", {}, "invalid_hold", {}, {}),
    ("1e6 degrees", {**AT_START, "shoulder_pan.pos": 1e6}, "clamp",
     {"shoulder_pan": 1.23}, {"shoulder_pan": 1.05}),
    ("1e3 degrees", {**AT_START, "shoulder_pan.pos": 1e3}, "clamp",
     {"shoulder_pan": 1.23}, {"shoulder_pan": 1.05}),
    ("far target (into the table)", INTO_THE_TABLE, "clamp",
     {"shoulder_lift": 1.76, "elbow_flex": -1.79, "wrist_flex": 1.75},
     {"shoulder_lift": 0.64, "elbow_flex": -1.08, "wrist_flex": 1.24}),
]


@pytest.mark.parametrize("servos", ["ideal", "sag"])
@pytest.mark.parametrize(("name", "action", "decision", "moved_ideal", "moved_sag"), BAD_INPUTS,
                         ids=[b[0] for b in BAD_INPUTS])
def test_the_harness_bad_input_table(servos, name, action, decision, moved_ideal, moved_sag):
    """The arm starts as the harness's simulator started it (torque on, goals at the start pose),
    so the motion matches the harness's table exactly."""
    arm = S.Arm(so101_reference.factory, servos=servos)
    before = arm.servos.degrees()
    arm.send(action)
    arm.run(1.0)
    after = arm.servos.degrees()
    moved = {m: round(after[m] - before[m], 2) for m in after if abs(after[m] - before[m]) > 0.05}
    assert arm.gate.last_decision == decision
    assert moved == (moved_ideal if servos == "ideal" else moved_sag)
    assert arm.clearance() > 0.0


def test_commanding_faster_never_moves_the_tool_faster():
    """A far target at 500 commands a second for 3 s: the displacement budget keeps the tool's
    reference under 0.8 x (0.2 + 0.02) / 0.2 = 0.88 of the 0.25 m/s limit over any 0.2 s window."""
    arm = S.Arm(so101_reference.factory, servos="ideal")
    track = [(0.0, np.array(arm.point()))]
    for _ in range(1500):
        arm.send(INTO_THE_TABLE)
        arm.run(0.002)
        track.append((arm.clock.t, np.array(arm.point())))
    fastest, j = 0.0, 0
    for t, p in track:
        while t - track[j][0] > 0.2 + 1e-9:
            j += 1
        if t - track[j][0] >= 0.1:
            fastest = max(fastest, float(np.linalg.norm(p - track[j][1])) / (t - track[j][0]))
    assert 0.1 < fastest <= 0.88 * 0.25 * 1.02
    assert arm.clearance() > 0.0
    assert arm.gate.decisions["clamp"] + arm.gate.decisions["reject_hold"] == 1500


def test_a_shakedown_runs_it_at_half_speed():
    default = so101_reference.factory(S.Arm().robot, S.so101_context())
    half = so101_reference.factory(S.Arm().robot, S.so101_context(speed_factor=0.5, mode="shakedown"))
    assert default.speed_factor == 0.8 and half.speed_factor == 0.5
    assert default.margin == pytest.approx(0.013)  # tolerance_m 0.01 / 2 + 8 mm
    assert default.link_points == ["elbow", "wrist"] and default.tool == "tool"
    assert default.joint_speed_dps == {j: pytest.approx(90.0) for j in S.ARM_JOINTS}


def test_the_factory_needs_the_runs_envelope_and_urdf(tmp_path):
    robot = S.Arm().robot
    with pytest.raises(ValueError, match="envelope and URDF"):
        so101_reference.factory(robot, S.so101_context(urdf_path=None))
    doc = json.loads(S.SO101_ENVELOPE.read_text())
    doc["frame"] = "world"
    elsewhere = tmp_path / "world.envelope.json"
    elsewhere.write_text(json.dumps(doc))
    with pytest.raises(ValueError, match="wrong frame"):
        so101_reference.factory(robot, S.so101_context(envelope_path=str(elsewhere)))


def test_an_envelopes_own_joint_limits_bound_each_joint(tmp_path):
    """Envelope 0.2 can declare joint ranges, which the harness format could not: the gate keeps
    every joint inside them as well as inside its firmware range and the URDF's limits."""
    doc = copy.deepcopy(json.loads(S.SO101_ENVELOPE.read_text()))
    doc["motion"]["joints"] = [{"name": "shoulder_pan", "position_rad": [-0.2, 0.3]}]
    path = tmp_path / "narrow.envelope.json"
    path.write_text(json.dumps(doc))
    gate = so101_reference.factory(S.Arm().robot, S.so101_context(envelope_path=str(path)))
    assert gate.ticks["shoulder_pan"][2:] == (pytest.approx(math.degrees(-0.2)),
                                              pytest.approx(math.degrees(0.3)))
    assert gate.ticks["elbow_flex"][2:] == S.Arm(so101_reference.factory).gate.ticks["elbow_flex"][2:]


# ----- the sag ratchet: bob, where the first gate failed ------------------------------------------
BOB_ENVELOPE = S.EXAMPLES / "envelopes" / "bob.envelope.json"
BOB_KIT = S.EXAMPLES / "envelopes" / "bob.kit.json"
BOB_CAL = S.REPO / "packages" / "kit" / "tests" / "data" / "bob_calibration.json"
_MODEL = S.URDFModel.from_file(S.URDF)


def _bob_shakedown(factory, seconds: float) -> tuple[float, dict]:
    """A simulated shakedown on bob (its calibration, envelope, joint map and gravity sag, powered
    up with its goal registers at 0 and connected by the kit's safe connect), in lockstep: the
    lowest clearance of any zoned point, and the gate's decisions."""
    kit = json.loads(BOB_KIT.read_text())
    joint_map = JointMap.from_dict(kit["joint_map"])
    adapter = SimSO101Adapter(command_space_path=S.SO101_SPACE, urdf_path=S.URDF, joint_map=joint_map,
                              start_pose=kit["start_pose_deg"], calibration_path=BOB_CAL, sag=1.0,
                              tau_s=0.1, realtime=False, prompt=lambda q: "", log=lambda m: None)
    adapter.open()
    context = {"mode": "shakedown", "speed_factor": 0.5, "envelope_path": str(BOB_ENVELOPE),
               "command_space_path": str(S.SO101_SPACE), "kit_config_path": str(BOB_KIT),
               "urdf_path": str(S.URDF)}
    gate = factory(adapter.sim_robot, context)
    clock = S.Clock()
    gate.clock = clock
    envelope = envelope_mod.load(BOB_ENVELOPE)

    def lowest() -> float:
        q = joint_map.to_urdf({j: v for j, v in adapter.true_joints().items() if j in S.ARM_JOINTS})
        return min(envelope.clearance(p, [float(v) for v in _MODEL.point(envelope.points[p].link, None, q)])
                   for p in ("tool", "elbow", "wrist"))

    center = {f"{j}.pos": float(v) for j, v in kit["start_pose_deg"].items() if j != "gripper"}
    stream = hostile_stream(load_command_space(S.SO101_SPACE), 7,
                            {"seconds": seconds, "amplitude": 0.15, "center": center})
    worst, prev = lowest(), 0.0
    for p in stream:
        adapter.step(p.t_s - prev)
        clock.t = prev = p.t_s
        gate.send_action(dict(p.payload))
        worst = min(worst, lowest())
    return worst, dict(gate.decisions)


def test_the_first_gate_ratchets_bob_out_of_its_box_and_the_fixed_one_holds():
    """On bob the first gate's shakedown left the box in 5.7 s (2026-10-08): it planned from the
    measured pose and held by re-sending it, so every hold sat one sag lower. In the simulator
    with bob's fitted sag it is out by more than the 10 mm tolerance within 10 s, while it reports
    mostly clamps; the fixed gate, which plans from its own last goals, stays inside."""
    worst_v1, said_v1 = _bob_shakedown(so101_reference_v1.factory, 10.0)
    worst_v2, said_v2 = _bob_shakedown(so101_reference.factory, 10.0)
    assert worst_v1 < -0.01
    assert said_v1["clamp"] > said_v1["allow"]
    assert worst_v2 > 0.01
    assert said_v2["clamp"] > said_v2["allow"]  # the same hostile stream, clamped as hard
