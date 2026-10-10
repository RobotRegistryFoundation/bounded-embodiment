"""URDF kinematics: parsing, FK against an independent implementation, batch FK, points, joint maps."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest

from bounded_embodiment_kit.kinematics import JointMap, URDFError, URDFModel, envelope_points

REPO = Path(__file__).resolve().parents[3]
SO101 = REPO / "examples" / "robots" / "so101" / "so101_new_calib.urdf"
GOLDEN = json.loads((Path(__file__).parent / "data" / "so101_fk_golden.json").read_text())
ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
POINTS = {"tool": ("gripper_frame_link", None), "elbow": ("lower_arm_link", None),
          "wrist": ("wrist_link", None)}


@pytest.fixture(scope="module")
def so101() -> URDFModel:
    return URDFModel.from_file(SO101)


def test_parse_so101(so101):
    assert so101.root == "base_link"
    assert so101.actuated == ARM + ["gripper"]
    lim = so101.limits()
    assert lim["elbow_flex"] == pytest.approx((-1.69, 1.69))
    assert so101.chain("gripper_frame_link")[-1].name == "gripper_frame_joint"
    meshes = {v.path.name for vs in so101.visuals.values() for v in vs}
    assert "base_so101_v2.stl" in meshes and "moving_jaw_so101_v1.stl" in meshes


@pytest.mark.parametrize("case", GOLDEN["cases"], ids=lambda c: str(c["q_deg"]["shoulder_pan"]))
def test_fk_matches_independent_implementation(so101, case):
    q = {j: math.radians(v) for j, v in case["q_deg"].items()}
    got = so101.points(POINTS, q)
    for pid in POINTS:
        assert np.allclose(got[pid], case[pid], atol=1e-12), pid


def test_batch_equals_single(so101):
    rng = np.random.default_rng(0)
    Q = {j: rng.uniform(-1.5, 1.5, size=7) for j in ARM + ["gripper"]}
    batch = so101.fk_batch(Q)
    for i in range(7):
        single = so101.fk({j: float(Q[j][i]) for j in Q})
        for link in so101.links:
            assert np.allclose(batch[link][i], single[link], atol=1e-12), link


def test_point_offset_is_in_link_frame(so101):
    q = {"shoulder_pan": 0.3, "elbow_flex": -0.4}
    T = so101.fk(q)["gripper_frame_link"]
    p = so101.point("gripper_frame_link", [0.0, 0.0, 0.01], q)
    assert np.allclose(p, T[:3, 3] + 0.01 * T[:3, 2])


def test_envelope_points_from_bob():
    doc = json.loads((REPO / "examples" / "envelopes" / "bob.envelope.json").read_text())
    assert envelope_points(doc) == {"tool": ("gripper_frame_link", None),
                                    "elbow": ("lower_arm_link", None), "wrist": ("wrist_link", None)}


def test_joint_map_round_trip():
    jm = JointMap.from_dict({"elbow_flex": {"sign": -1, "offset_deg": -90}})
    urdf = jm.to_urdf({"elbow_flex": 10.0, "wrist_roll": 5.0})
    assert urdf["elbow_flex"] == pytest.approx(math.radians(-100.0))
    assert urdf["wrist_roll"] == pytest.approx(math.radians(5.0))
    back = jm.from_urdf(urdf)
    assert back == pytest.approx({"elbow_flex": 10.0, "wrist_roll": 5.0})
    with pytest.raises(ValueError):
        JointMap.from_dict({"x": {"sign": 2}})


def test_rejects_cycles_and_mimic():
    two_roots = """<robot name="r"><link name="a"/><link name="b"/></robot>"""
    with pytest.raises(URDFError):
        URDFModel.from_string(two_roots)
    mimic = """<robot name="r"><link name="a"/><link name="b"/>
      <joint name="j" type="revolute"><parent link="a"/><child link="b"/><axis xyz="0 0 1"/>
      <mimic joint="k"/><limit lower="0" upper="1"/></joint></robot>"""
    with pytest.raises(URDFError):
        URDFModel.from_string(mimic)


def test_prismatic_joint():
    xml = """<robot name="r"><link name="a"/><link name="b"/>
      <joint name="slide" type="prismatic"><parent link="a"/><child link="b"/>
      <origin xyz="1 0 0"/><axis xyz="0 0 2"/><limit lower="0" upper="1"/></joint></robot>"""
    m = URDFModel.from_string(xml)
    assert np.allclose(m.point("b", None, {"slide": 0.25}), [1.0, 0.0, 0.25])
