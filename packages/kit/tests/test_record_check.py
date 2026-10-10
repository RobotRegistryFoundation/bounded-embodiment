# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""`be record check` on synthetic sessions with known geometry."""
from __future__ import annotations

import json
import math

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
from bounded_embodiment_kit.instruments import record  # noqa: E402

PPM = 4.0                                                  # board render: px per mm
K_OAK = np.array([[1000.0, 0, 640.0], [0, 1000.0, 360.0], [0, 0, 1]])


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


@pytest.fixture(scope="module")
def board_img():
    b = record.charuco_board(cv2)
    return b.generateImage((int(150 * PPM), int(210 * PPM)), marginSize=0, borderBits=1)


def render(board_img, K, R, t, size, bg=170):
    """The board seen by a pinhole camera: board point X (m) -> K (R X + t)."""
    s = 1.0 / (PPM * 1000.0)
    S = np.array([[s, 0, 0.5 * s], [0, s, 0.5 * s], [0, 0, 1.0]])         # board pixel centre -> metres
    H = K @ np.column_stack([R[:, 0], R[:, 1], t]) @ S
    img = cv2.warpPerspective(board_img, H, size, flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                              borderValue=bg)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def plane_depth_mm(K, R, t, size):
    """Z (mm) of the board's plane at every pixel of an image of `size` with intrinsics K."""
    w, h = size
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    rays = np.linalg.inv(K) @ np.stack([u.ravel(), v.ravel(), np.ones(u.size)])
    n = R[:, 2]
    z = (n @ t) / (n @ rays)
    return np.clip(z * 1000.0, 0, 65535).round().astype(np.uint16).reshape(h, w)


def jpeg(img, q=95):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), q])
    assert ok
    return buf.tobytes()


def write_session(root, *, color=(), depth=(), wrist=(), events=(), meta_extra=None, status="finished",
                  clock=True):
    """A session directory in the recorder's own format, written with the recorder's own writers."""
    root.mkdir(parents=True, exist_ok=True)
    stats = {n: record.StreamStats(n, 30) for n in ("color", "depth", "wrist")}
    writers = []
    if color:
        (root / "oakd").mkdir(exist_ok=True)
        w = record.MjpegWriter(root / record.PATHS["color_stream"], root / record.PATHS["color_index"],
                               record.COLOR_COLUMNS, stats["color"], max_queue=100000)
        w.start()
        for i, (t, data) in enumerate(color):
            w.put(((i, t), data, (None, None, t + 0.02, 0)))
        writers.append(w)
    if depth:
        w = record.DepthChunkWriter(root / record.PATHS["depth_dir"], root / record.PATHS["depth_index"],
                                    stats["depth"], chunk_frames=8, max_queue=100000)
        w.start()
        for i, (t, d) in enumerate(depth):
            w.put(((i, t), d, (None, None, t + 0.02, 0)))
        writers.append(w)
    if wrist:
        (root / "wrist").mkdir(exist_ok=True)
        w = record.MjpegWriter(root / record.PATHS["wrist_stream"], root / record.PATHS["wrist_index"],
                               record.WRIST_COLUMNS, stats["wrist"], max_queue=100000)
        w.start()
        for t, data in wrist:
            w.put(((t,), data, (t - 0.01, 1, 0)))
        writers.append(w)
    for w in writers:
        w.finish()
        assert w.error is None
    with open(root / "events.csv", "w") as f:
        f.write("n,t_mono_s,t_wall_s,label,source\n")
        for n, (t, label) in enumerate(events):
            f.write(f"{n},{t:.6f},{t + 1e9:.6f},{label},keyboard\n")
    with open(root / "clock.csv", "w") as f:
        f.write("kind,t_wall_s,t_mono_s,uncertainty_s\n")
        if clock:
            f.write(f"start,{1e9:.6f},0.000000,0.000001\nstop,{1e9 + 30:.6f},30.000000,0.000001\n")
    meta = {"format": record.FORMAT, "status": status, "t0_mono_s": 0.0,
            "settings": {"color_fps": 30, "depth_fps": 30, "wrist_fps": 30},
            "oakd": {"intrinsics": {"rgb_1280x720": K_OAK.tolist(), "rgb_distortion": [0.0] * 14}} if color else None,
            "wrist": {"device": "/dev/video0"} if wrist else None, "board": record.BOARD_SPEC}
    meta.update(meta_extra or {})
    (root / "meta.json").write_text(json.dumps(meta))
    return root


def board_pose(distance=0.6, tilt_deg=40.0, yaw_deg=10.0):
    R = _ry(math.radians(yaw_deg)) @ _rx(math.radians(tilt_deg))
    centre = np.array([0.075, 0.105, 0.0])
    t = np.array([0.0, 0.0, distance]) - R @ centre
    return R, t


def test_board_pose_and_depth_agree_with_the_truth(tmp_path, board_img):
    R, t = board_pose()
    color_img = render(board_img, K_OAK, R, t, (1280, 720))
    K_d = K_OAK.copy()
    K_d[:2] *= 0.5
    depth = plane_depth_mm(K_d, R, t, (640, 360))
    times = np.arange(0.0, 12.0, 0.5)
    s = write_session(tmp_path / "s", color=[(float(x), jpeg(color_img)) for x in times],
                      depth=[(float(x) + 0.004, depth) for x in times], events=[(0.2, "board_placed")])
    rep, problems = record.check_session(s, cv2=cv2, quick=True, say=lambda *_: None)
    bp = rep["board_placed"]
    assert bp["status"] == "ok" and bp["corners"] == 24
    assert bp["camera_to_board_m"] == pytest.approx(float(np.linalg.norm(t)), abs=0.002)
    assert bp["board_origin_in_camera_m"] == pytest.approx(t.tolist(), abs=0.002)
    assert bp["board_tilt_to_camera_axis_deg"] == pytest.approx(
        math.degrees(math.acos(abs((_ry(math.radians(10)) @ _rx(math.radians(40)))[2, 2]))), abs=1.0)
    assert bp["reprojection_rms_px"] < 0.5
    dm = bp["depth_minus_board_mm"]
    assert dm["n"] == 24 and abs(dm["median"]) < 2.0 and dm["p90_abs"] < 4.0
    # live-style output is not used for a finished session
    assert (s / "check" / "check.json").exists() and not (s / "check" / "check-live.json").exists()


def test_depth_scale_error_shows_up(tmp_path, board_img):
    """A 3 % depth scale error must be visible in depth_minus_board (that is the point of the check)."""
    R, t = board_pose()
    color_img = render(board_img, K_OAK, R, t, (1280, 720))
    K_d = K_OAK.copy()
    K_d[:2] *= 0.5
    depth = (plane_depth_mm(K_d, R, t, (640, 360)).astype(float) * 1.03).astype(np.uint16)
    times = np.arange(0.0, 10.0, 1.0)
    s = write_session(tmp_path / "s", color=[(float(x), jpeg(color_img)) for x in times],
                      depth=[(float(x), depth) for x in times], events=[(0.0, "board_placed")])
    rep, _ = record.check_session(s, cv2=cv2, quick=True, say=lambda *_: None)
    assert rep["board_placed"]["depth_minus_board_mm"]["relative_median"] == pytest.approx(0.03, abs=0.005)


def test_missing_board_is_a_problem(tmp_path):
    blank = jpeg(np.full((720, 1280, 3), 128, np.uint8))
    s = write_session(tmp_path / "s", color=[(float(x), blank) for x in np.arange(0, 10, 0.5)],
                      events=[(0.0, "board_placed")])
    rep, problems = record.check_session(s, cv2=cv2, quick=True, say=lambda *_: None)
    assert rep["board_placed"]["status"] == "BOARD NOT FOUND"
    assert any("board was not found" in p for p in problems)


def test_wrist_calibration_recovers_the_focal_length(tmp_path, board_img):
    Kw = np.array([[520.0, 0, 318.0], [0, 515.0, 243.0], [0, 0, 1]])
    frames, t = [], 10.0
    for k in range(18):
        ax, ay = math.radians(25 * math.sin(k * 1.3)), math.radians(28 * math.cos(k * 0.9))
        R = _ry(ay) @ _rx(ax)
        dist = 0.32 + 0.12 * ((k * 7) % 5) / 4
        off = np.array([0.02 * math.sin(k), 0.015 * math.cos(2 * k), 0.0])
        tt = np.array([0.0, 0.0, dist]) + off - R @ np.array([0.075, 0.105, 0.0])
        img = render(board_img, Kw, R, tt, (640, 480), bg=120)
        for _ in range(3):                                   # ~3 frames per pose, as a held board gives
            frames.append((t, jpeg(img, 92)))
            t += 1 / 30
        t += 1.0
    s = write_session(tmp_path / "s", wrist=frames,
                      events=[(9.5, "wrist_calib_start"), (t + 0.5, "wrist_calib_end")])
    lines = []
    rep, problems = record.check_session(s, cv2=cv2, say=lines.append)
    wc = rep["wrist_calibration"]
    assert wc["status"] == "ok" and wc["views_used"] >= 12, wc
    assert wc["fx"] == pytest.approx(520.0, rel=0.02) and wc["fy"] == pytest.approx(515.0, rel=0.02)
    assert wc["rms_px"] < 1.0
    assert any("wrist calibration views" in line for line in lines)
    assert (s / "check" / "contact_sheet.png").exists()


def test_oak_board_views_compare_both_intrinsics_candidates(tmp_path, board_img):
    frames, t = [], 5.0
    for k in range(10):
        R = _ry(math.radians(30 * math.sin(k))) @ _rx(math.radians(25 * math.cos(1.7 * k)))
        tt = np.array([0.03 * math.sin(2 * k), 0.02 * math.cos(k), 0.55 + 0.05 * (k % 3)]) - \
            R @ np.array([0.075, 0.105, 0.0])
        frames.append((t, jpeg(render(board_img, K_OAK, R, tt, (1280, 720)))))
        t += 2.0
    crop = (K_OAK * np.array([[1.055], [1.055], [1]])).tolist()
    s = write_session(tmp_path / "s", color=frames, events=[(4.5, "oakd_views_start"), (t, "oakd_views_end")],
                      meta_extra={"oakd": {"intrinsics": {"rgb_1280x720": K_OAK.tolist(),
                                                          "rgb_1280x720_crop_aware": crop,
                                                          "rgb_distortion": [0.0] * 14}}})
    rep, _ = record.check_session(s, cv2=cv2, say=lambda *_: None)
    oc = rep["oakd_views_calibration"]
    assert oc["status"] == "ok"
    assert oc["fx_ratio_to_rgb_1280x720"] == pytest.approx(1.0, abs=0.02)
    assert oc["fx_ratio_to_rgb_1280x720_crop_aware"] == pytest.approx(1 / 1.055, abs=0.02)


def _stream_rows(tmp, times, host_offset=0.02, path="oakd/color"):
    blob = jpeg(np.full((72, 128, 3), 90, np.uint8))
    (tmp / "oakd").mkdir(parents=True, exist_ok=True)
    (tmp / f"{path}.mjpeg").write_bytes(blob * len(times))
    with open(tmp / f"{path}_index.csv", "w") as f:
        f.write(",".join(record.COLOR_COLUMNS) + "\n")
        for i, t in enumerate(times):
            f.write(f"{i},{i},{t:.6f},{i * len(blob)},{len(blob)},,,{t + host_offset:.6f},0\n")


def test_gaps_and_backwards_timestamps_are_problems(tmp_path):
    times = list(np.arange(0, 1.0, 1 / 30)) + [2.6, 2.59] + list(2.62 + np.arange(0, 1.0, 1 / 30))
    write_session(tmp_path, meta_extra={"oakd": {"intrinsics": {}}})
    _stream_rows(tmp_path, times)
    rep, problems = record.check_session(tmp_path, cv2=cv2, quick=True, say=lambda *_: None)
    s = rep["streams"]["color"]
    assert s["not_increasing"] == 1 and s["max_gap_s"] == pytest.approx(1.6, abs=0.05)
    assert any("not increasing" in p for p in problems) and any("gap" in p for p in problems)
    assert any("depth: no frames" in p for p in problems)


def test_negative_latency_means_the_clocks_disagree(tmp_path):
    write_session(tmp_path, meta_extra={"oakd": {"intrinsics": {}}})
    _stream_rows(tmp_path, list(np.arange(0, 2.0, 1 / 30)), host_offset=-0.050)
    rep, problems = record.check_session(tmp_path, cv2=cv2, quick=True, say=lambda *_: None)
    assert any("not on this host's monotonic clock" in p for p in problems)


def test_live_session_with_a_half_written_line(tmp_path):
    write_session(tmp_path, status="recording", meta_extra={"oakd": {"intrinsics": {}}})
    _stream_rows(tmp_path, list(np.arange(0, 2.0, 1 / 30)))
    with open(tmp_path / "oakd/color_index.csv", "a") as f:
        f.write("60,60,2.0000")                                  # no newline: still being written
    rep, _ = record.check_session(tmp_path, cv2=cv2, quick=True, say=lambda *_: None)
    assert rep["live"] and rep["streams"]["color"]["frames"] == 60
    assert (tmp_path / "check" / "check-live.json").exists()


def test_not_a_session(tmp_path):
    with pytest.raises(record.RecordError, match="not a recording"):
        record.check_session(tmp_path, cv2=cv2, say=lambda *_: None)


def test_a_redone_board_step_uses_the_last_board_placed(tmp_path, board_img):
    R, t = board_pose(distance=0.55)
    good = jpeg(render(board_img, K_OAK, R, t, (1280, 720)))
    blank = jpeg(np.full((720, 1280, 3), 128, np.uint8))
    color = [(float(x), blank) for x in np.arange(0, 12, 0.5)] + [(float(x), good) for x in np.arange(12, 24, 0.5)]
    s = write_session(tmp_path / "s", color=color, events=[(0.0, "board_placed"), (12.5, "board_placed")])
    rep, problems = record.check_session(s, cv2=cv2, quick=True, say=lambda *_: None)
    assert rep["board_placed"]["status"] == "ok" and rep["board_placed"]["event_t_mono_s"] == 12.5
    assert not [p for p in problems if "board" in p]
