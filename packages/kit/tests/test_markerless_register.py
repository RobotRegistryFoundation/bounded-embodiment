"""Camera -> base_link registration (`be fit register`): the board frame from tape measurements, the base
offset from the URDF, and the whole chain on a synthetic recording."""

from __future__ import annotations

import math

import numpy as np
import pytest
from _markerless_support import URDF, need_fit_stack, write_tiny_session
from bounded_embodiment_kit.instruments.markerless import synth
from bounded_embodiment_kit.instruments.markerless.register import (
    RegistrationError,
    board_frame_in_F,
    board_scale,
    register_session,
)
from bounded_embodiment_kit.instruments.session import parse_measurements
from bounded_embodiment_kit.kinematics import URDFModel

SIZE = (0.15, 0.21)
UNIT = {"x": 1.0, "y": 1.0}


def measured(T_F_board=None, round_mm=None) -> dict:
    T = synth.default_board_T_F() if T_F_board is None else T_F_board
    return parse_measurements(synth.measurements_text(T, 0.005, SIZE, round_mm=round_mm))


def rot_deg(Ra, Rb) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(Ra.T @ Rb) - 1) / 2))))


def test_board_frame_from_exact_corners_is_the_true_one():
    warnings = []
    T, checks = board_frame_in_F(measured(), SIZE, UNIT, warnings)
    assert np.allclose(T, synth.default_board_T_F(), rtol=0, atol=1e-6)
    assert np.allclose(checks["z_axis_in_F"], [0, 0, -1])
    assert checks["O_to_X_mm"] == pytest.approx(150.0) and checks["O_to_Y_mm"] == pytest.approx(210.0)
    assert checks["angle_XOY_deg"] == pytest.approx(90.0)
    assert checks["board_yaw_from_OX_deg"] == pytest.approx(-3.0, abs=1e-4)
    assert warnings == []


def test_board_frame_refuses_the_wrong_handedness():
    m = measured()
    swapped = {**m, "corner_X_x_mm": m["corner_Y_x_mm"], "corner_X_y_mm": m["corner_Y_y_mm"],
               "corner_Y_x_mm": m["corner_X_x_mm"], "corner_Y_y_mm": m["corner_X_y_mm"]}
    with pytest.raises(RegistrationError, match="clockwise"):
        board_frame_in_F(swapped, SIZE, UNIT, [])
    flipped = {**m, **{k: -m[k] for k in ("corner_O_y_mm", "corner_X_y_mm", "corner_Y_y_mm")}}
    with pytest.raises(RegistrationError, match=r"not \(0, 0, -1\)"):         # +y measured to the right
        board_frame_in_F(flipped, SIZE, UNIT, [])


def test_board_frame_refuses_face_down_and_missing_numbers():
    with pytest.raises(RegistrationError, match="face up"):
        board_frame_in_F({**measured(), "board_face": "down"}, SIZE, UNIT, [])
    m = measured()
    del m["corner_Y_y_mm"]
    with pytest.raises(RegistrationError, match="corner_Y_y_mm"):
        board_frame_in_F(m, SIZE, UNIT, [])


def test_board_frame_warns_when_the_corners_disagree():
    m = measured()
    warnings = []
    board_frame_in_F({**m, "corner_X_x_mm": m["corner_X_x_mm"] + 5.0}, SIZE, UNIT, warnings)
    assert any("O-X" in w for w in warnings)
    warnings = []
    skew = {**m, "corner_Y_x_mm": m["corner_Y_x_mm"] + 210.0 * math.radians(2.0)}
    _, checks = board_frame_in_F(skew, SIZE, UNIT, warnings)
    assert any("angle XOY" in w for w in warnings)
    assert abs(checks["yaw_difference_deg"]) > 0.3                            # the both-edges fit differs


def test_board_scale_from_the_printed_bars():
    w = []
    assert board_scale({"printed_100mm_bar_horizontal_mm": 100.0, "printed_100mm_bar_vertical_mm": 100.0}, w)["x"] == 1.0
    assert w == []
    s = board_scale({"printed_100mm_bar_horizontal_mm": 100.8, "printed_100mm_bar_vertical_mm": 99.6}, w)
    assert s["x"] == pytest.approx(1.008) and s["y"] == pytest.approx(0.996) and len(w) == 1
    with pytest.raises(RegistrationError, match="reprint"):
        board_scale({"printed_100mm_bar_horizontal_mm": 104.0, "printed_100mm_bar_vertical_mm": 100.0}, [])
    w = []
    assert board_scale({}, w)["x"] == 1.0 and "not given" in w[0]


def test_base_offset_is_computed_from_the_urdf():
    need_fit_stack()
    from bounded_embodiment_kit.instruments.markerless import mesh
    from bounded_embodiment_kit.instruments.markerless.model import base_offset

    u = URDFModel.from_file(URDF)
    off = base_offset(u)
    pan = u.joints["shoulder_pan"].origin
    assert off.joint == "shoulder_pan"
    assert off.pan_x == pytest.approx(pan[0, 3]) and off.pan_y == pytest.approx(pan[1, 3])
    assert off.z_min == pytest.approx(mesh.link_triangles(u)["base_link"][..., 2].min())
    assert off.pan_x == pytest.approx(0.0388, abs=1e-3) and abs(off.pan_y) < 1e-6
    assert off.z_min == pytest.approx(-0.0024, abs=1e-3)
    assert "38.8 mm behind the axis" in off.explanation
    assert np.allclose(off.T_root_F()[:3, 3], [off.pan_x, off.pan_y, off.z_min])


def test_base_offset_needs_a_vertical_first_axis(tmp_path):
    from bounded_embodiment_kit.instruments.markerless.model import base_offset

    xml = """<robot name="r"><link name="b"/><link name="a"/>
      <joint name="j" type="revolute"><parent link="b"/><child link="a"/><origin xyz="0.03 0 0.05"/>
      <axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>"""
    with pytest.raises(ValueError, match="not vertical"):
        base_offset(URDFModel.from_string(xml, base_dir=tmp_path))


def test_register_needs_a_board_placed_event(tmp_path):
    pytest.importorskip("cv2")
    root = write_tiny_session(tmp_path / "s", events=[(101.0, "background_start")])
    with pytest.raises(RegistrationError, match="board_placed"):
        register_session(root, URDF, say=lambda s: None)


# ----- the whole chain on a synthetic recording -----------------------------------------------------------
@pytest.mark.slow
def test_registration_recovers_the_camera(full_run, capsys):
    from bounded_embodiment_kit.instruments.markerless.fit import load_registration

    doc, truth = full_run.reg, full_run.truth
    T = np.array(doc["T_bl_cam"])
    err_mm = float(np.linalg.norm(T[:3, 3] - truth["T_bl_cam"][:3, 3])) * 1000
    err_deg = rot_deg(T[:3, :3], truth["T_bl_cam"][:3, :3])
    with capsys.disabled():
        print(f"\nregistration: {err_mm:.2f} mm, {err_deg:.3f} deg; PnP {doc['checks']['pnp']['reprojection_rms_px']:.2f} px")
    assert err_mm <= 1.0 and err_deg <= 0.1
    steps = doc["steps"]
    chain = np.array(steps["T_bl_F"]) @ np.array(steps["T_F_board"]) @ np.linalg.inv(np.array(steps["T_cam_board"]))
    assert np.allclose(chain, T)
    assert "38.8 mm behind" in steps["base_offset"]["explanation"]
    m = doc["checks"]["measurements"]
    assert np.allclose(m["z_axis_in_F"], [0, 0, -1])
    assert m["O_to_X_mm"] == pytest.approx(150.0, abs=0.01) and m["angle_XOY_deg"] == pytest.approx(90.0, abs=0.01)
    d = doc["checks"]["depth"]
    assert d["status"] == "ok"
    assert abs(d["signed_offset_from_pnp_plane_mm"]["mean"]) < 1.5 and d["angle_between_planes_deg"] < 1.0
    assert doc["warnings"] == []
    assert np.allclose(load_registration(full_run.root / "fit" / "registration.json")["T_bl_cam"], T)


@pytest.mark.slow
def test_registration_with_corners_taped_to_the_nearest_mm(full_run, tmp_path, capsys):
    truth = full_run.truth
    meas = measured(round_mm=1.0)
    doc = register_session(full_run.root, URDF, measurements=meas, out=tmp_path, say=lambda s: None)
    T = np.array(doc["T_bl_cam"])
    err_mm = float(np.linalg.norm(T[:3, 3] - truth["T_bl_cam"][:3, 3])) * 1000
    err_deg = rot_deg(T[:3, :3], truth["T_bl_cam"][:3, :3])
    with capsys.disabled():
        print(f"\nregistration from corners rounded to 1 mm: {err_mm:.2f} mm, {err_deg:.3f} deg")
    assert err_mm <= 1.0 and err_deg <= 0.1
