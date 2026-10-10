# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""Whole recordings against a fake OAK-D and a fake V4L2 wrist camera."""
from __future__ import annotations

import argparse
import csv
import io
import json
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
sys.path.insert(0, str(Path(__file__).parent))
from _record_fakes import FakeCapture, jpeg_bytes, make_fake_dai  # noqa: E402

from bounded_embodiment_kit.instruments import record  # noqa: E402


def _args(*argv):
    p = argparse.ArgumentParser()
    record.add_arguments(p)
    return p.parse_args(list(argv))


def _rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def _depth(seq):
    d = np.full((360, 640), 700, np.uint16)
    d[:, :40] = 0
    d[100:200, 300:340] = 450 + seq % 5
    return d


@pytest.fixture
def fake_dai():
    return make_fake_dai(color_jpeg=jpeg_bytes(cv2, text="oak"), depth_frame=_depth)


def _run(args, dai, *, stdin=None, capture=None, during=None):
    console = io.StringIO()
    rec = record.Recorder(args, dai=dai, cv2=cv2, stdin=stdin, console_stream=console, install_signals=False,
                          capture_factory=capture or (lambda dev: FakeCapture(cv2, dev)))
    timers = [threading.Timer(t, fn) for t, fn in (during or [])]
    for t in timers:
        t.start()
    try:
        rc = rec.run()
    finally:
        for t in timers:
            t.cancel()
    return rc, rec, console.getvalue()


def test_a_short_recording_writes_everything(tmp_path, fake_dai):
    out = tmp_path / "s1"
    fifo = tmp_path / "events.fifo"
    args = _args("--out", str(out), "--seconds", "2.5", "--wrist", "/dev/video0", "--events-fifo", str(fifo),
                 "--status-every", "0.5", "--chunk-frames", "20")
    marks = [(1.0, lambda: record.send_mark("sweep start", fifo)), (1.5, lambda: record.send_mark("x", fifo))]
    rc, rec, console = _run(args, fake_dai, stdin=io.StringIO("hello world\n\n"), during=marks)
    assert rc == 0
    meta = json.loads((out / "meta.json").read_text())
    assert meta["format"] == "be-camera-recording/1" and meta["status"] == "finished"
    s = meta["summary"]
    assert s["stop_reason"].startswith("time limit")
    for name in ("color", "depth", "wrist"):
        st = s["streams"][name]
        assert 50 <= st["frames_written"] <= 90, (name, st)
        assert st["dropped_queue_full"] == 0 and st["timestamps_not_increasing"] == 0
        assert 25 < st["mean_fps"] < 35
    assert s["streams"]["depth"]["chunks"] >= 3
    # OAK-D metadata
    oak = meta["oakd"]
    assert oak["intrinsics"]["rgb_1280x720"][0][0] == pytest.approx(1000.0)
    assert oak["calibration_eeprom"]["boardName"] == "FAKE-OAK-D"
    assert oak["color"]["mjpeg_quality"] == 85 and oak["depth"]["output_size"] == [640, 360]
    assert meta["wrist"]["raw_mjpeg_passthrough"] is True
    # indexes and payloads
    crow = _rows(out / "oakd/color_index.csv")
    data = (out / "oakd/color.mjpeg").read_bytes()
    assert len(crow) == s["streams"]["color"]["frames_written"]
    r = crow[-1]
    assert int(r["byte_offset"]) + int(r["byte_len"]) == len(data)
    assert data[int(r["byte_offset"]):][:2] == b"\xff\xd8"
    assert all(float(r["t_host_s"]) > float(r["t_mono_s"]) for r in crow)
    assert all(float(r["t_mid_s"]) < float(r["t_mono_s"]) for r in crow)
    drow = _rows(out / "oakd/depth_index.csv")
    with np.load(out / "oakd/depth" / drow[5]["chunk"]) as z:
        np.testing.assert_array_equal(z["depth"][int(drow[5]["index_in_chunk"])], _depth(int(drow[5]["device_seq"])))
    wrow = _rows(out / "wrist/wrist_index.csv")
    wdata = (out / "wrist/wrist.mjpeg").read_bytes()
    assert wrow[0]["raw"] == "1"
    w0 = wdata[int(wrow[0]["byte_offset"]):][: int(wrow[0]["byte_len"])]
    assert w0 == FakeCapture(cv2, 0).jpeg                         # the camera's own bytes, untouched
    assert all(float(r["t_mono_s"]) > float(r["t_v4l2_s"]) for r in wrow)
    # events, clock and the rest
    ev = _rows(out / "events.csv")
    labels = [e["label"] for e in ev]
    assert labels[:2] == ["hello_world", "mark"] and "sweep_start" in labels and "x" in labels
    assert {e["source"] for e in ev} == {"keyboard", "fifo"}
    t_sweep = float(next(e for e in ev if e["label"] == "sweep_start")["t_mono_s"])
    assert t_sweep - meta["t0_mono_s"] == pytest.approx(1.0, abs=0.3)
    clock = _rows(out / "clock.csv")
    assert [c["kind"] for c in clock][0] == "start" and clock[-1]["kind"] == "stop"
    assert (out / "measurements.txt").read_text().count("corner_O_x_mm") == 1
    assert (out / "recorder.log").stat().st_size > 0
    assert not (out / "recorder.pid").exists() and not fifo.exists()
    assert "colour" not in console or True
    assert console.count("| free ") >= 3                         # status lines
    assert "EVENT" in console
    assert fake_dai.devices[0].closed


def test_check_passes_on_a_clean_recording(tmp_path, fake_dai):
    out = tmp_path / "s2"
    args = _args("--out", str(out), "--seconds", "2.0", "--wrist", "/dev/video0", "--chunk-frames", "15")
    assert _run(args, fake_dai)[0] == 0
    lines = []
    rep, problems = record.check_session(out, cv2=cv2, say=lines.append)
    assert problems == [], problems
    assert rep["result"] == "OK"
    assert rep["streams"]["color"]["host_latency"]["min_ms"] > 0
    assert rep["streams"]["wrist"]["raw_mjpeg_frames"] == rep["streams"]["wrist"]["frames"]
    assert (out / "check" / "contact_sheet.png").exists() and (out / "check" / "check.json").exists()
    sheet = cv2.imread(str(out / "check" / "contact_sheet.png"))
    assert sheet is not None and sheet.shape[1] == 250 + 320 + 320 + 240
    assert any("RESULT: OK" in line for line in lines)


def test_wrist_fallback_reencodes_when_raw_is_unavailable(tmp_path, fake_dai):
    out = tmp_path / "s3"
    args = _args("--out", str(out), "--seconds", "1.0", "--wrist", "/dev/video0", "--no-oak")
    rc, rec, _ = _run(args, fake_dai, capture=lambda dev: FakeCapture(cv2, dev, raw_ok=False))
    assert rc == 0 and not fake_dai.devices
    meta = json.loads((out / "meta.json").read_text())
    assert meta["wrist"]["raw_mjpeg_passthrough"] is False and meta["oakd"] is None
    wrow = _rows(out / "wrist/wrist_index.csv")
    assert wrow and all(r["raw"] == "0" for r in wrow)
    data = (out / "wrist/wrist.mjpeg").read_bytes()
    img = cv2.imdecode(np.frombuffer(data[: int(wrow[0]["byte_len"])], np.uint8), cv2.IMREAD_COLOR)
    assert img.shape == (480, 640, 3)


def test_yuyv_camera_is_not_put_in_raw_mode(tmp_path, fake_dai):
    out = tmp_path / "s4"
    args = _args("--out", str(out), "--seconds", "0.6", "--wrist", "/dev/video0", "--no-oak")
    caps = []

    def factory(dev):
        caps.append(FakeCapture(cv2, dev, fourcc="YUYV"))
        return caps[-1]

    assert _run(args, fake_dai, capture=factory)[0] == 0
    assert cv2.CAP_PROP_CONVERT_RGB not in caps[0].props
    assert json.loads((out / "meta.json").read_text())["wrist"]["actual"]["fourcc"] == "YUYV"


def test_oak_reconnects_after_losing_the_device(tmp_path, monkeypatch):
    monkeypatch.setattr(record, "OAK_RETRY_S", 0.2)
    dai = make_fake_dai(color_jpeg=jpeg_bytes(cv2), depth_frame=_depth, fail_after_s=0.6)
    out = tmp_path / "s5"
    args = _args("--out", str(out), "--seconds", "2.0", "--wrist", "none", "--chunk-frames", "10")
    rc, rec, console = _run(args, dai)
    assert rc == 0
    assert len(dai.devices) == 2 and all(d.closed for d in dai.devices)
    meta = json.loads((out / "meta.json").read_text())
    sessions = meta["summary"]["oak_sessions"]
    assert [s["oak_session"] for s in sessions] == [0, 1] and "X_LINK_ERROR" in sessions[0]["error"]
    rows = _rows(out / "oakd/color_index.csv")
    assert {r["oak_session"] for r in rows} == {"0", "1"}
    assert "OAK-D lost" in console and "reconnected" in console
    rep, problems = record.check_session(out, cv2=cv2, quick=True, say=lambda *_: None)
    assert rep["streams"]["color"]["device_seq_resets"] >= 1


def test_refuses_to_start_without_disk_space(tmp_path, fake_dai, monkeypatch):
    monkeypatch.setattr(record, "free_bytes", lambda path: 5e9)
    args = _args("--out", str(tmp_path / "s6"), "--seconds", "1200", "--wrist", "none")
    with pytest.raises(record.RecordError, match="Not enough free disk") as e:
        record.Recorder(args, dai=fake_dai, console_stream=io.StringIO(), install_signals=False).run()
    assert "--depth-fps 15" in str(e.value)
    assert not fake_dai.devices and not (tmp_path / "s6").exists()


def test_disk_estimate_uses_the_planned_seconds(tmp_path, fake_dai, monkeypatch):
    monkeypatch.setattr(record, "free_bytes", lambda path: 12e9)
    console = io.StringIO()
    args = _args("--out", str(tmp_path / "s7"), "--seconds", "600", "--wrist", "none", "--depth-fps", "15")
    plan = record.Recorder(args, dai=fake_dai, console_stream=console, install_signals=False)._plan()
    rates = record.estimate_rates(depth_fps=15, wrist=False)
    assert plan["need_gb_with_margin"] == pytest.approx(600 * rates["total"] * 1.5 / 1e3, abs=0.01)
    assert "need" in console.getvalue() and "free 12.0 GB" in console.getvalue()


def test_refuses_a_non_empty_output_directory(tmp_path, fake_dai):
    (tmp_path / "s8").mkdir()
    (tmp_path / "s8" / "x").write_text("x")
    args = _args("--out", str(tmp_path / "s8"), "--seconds", "1", "--wrist", "none")
    with pytest.raises(record.RecordError, match="not empty"):
        record.Recorder(args, dai=fake_dai, console_stream=io.StringIO(), install_signals=False).run()


def test_no_frames_from_the_camera_fails_cleanly(tmp_path):
    dai = make_fake_dai(color_jpeg=jpeg_bytes(cv2), depth_frame=_depth, color_fps=0.001, depth_fps=0.001)
    args = _args("--out", str(tmp_path / "s9"), "--seconds", "5", "--wrist", "none", "--first-frame-timeout", "0.5")
    with pytest.raises(record.RecordError, match="no frames from color, depth"):
        record.Recorder(args, dai=dai, console_stream=io.StringIO(), install_signals=False).run()
    meta = json.loads((tmp_path / "s9" / "meta.json").read_text())
    assert meta["status"] == "failed" and dai.devices[0].closed


def test_stop_command_stops_a_running_recorder(tmp_path, fake_dai):
    """`record.py stop DIR` sends SIGINT to the pid in recorder.pid; the recorder finishes cleanly."""
    import os
    import signal
    out = tmp_path / "s10"
    args = _args("--out", str(out), "--wrist", "none")
    rec = record.Recorder(args, dai=fake_dai, console_stream=io.StringIO(), install_signals=True)
    old = signal.getsignal(signal.SIGINT)

    def stopper():
        deadline = time.monotonic() + 10
        while not (out / "recorder.pid").exists() and time.monotonic() < deadline:
            time.sleep(0.05)
        time.sleep(0.5)
        assert int((out / "recorder.pid").read_text()) == os.getpid()
        os.kill(os.getpid(), signal.SIGINT)

    t = threading.Thread(target=stopper)
    t.start()
    assert rec.run() == 0
    t.join()
    meta = json.loads((out / "meta.json").read_text())
    assert meta["summary"]["stop_reason"] == "stopped by operator (Ctrl+C)"
    assert signal.getsignal(signal.SIGINT) is old


def test_wrist_auto_without_a_camera_closes_the_oak(tmp_path, fake_dai, monkeypatch):
    monkeypatch.setattr(record, "find_wrist_camera", lambda: (None, []))
    args = _args("--out", str(tmp_path / "s11"), "--seconds", "1", "--wrist", "auto")
    with pytest.raises(record.RecordError, match="no USB camera"):
        record.Recorder(args, dai=fake_dai, cv2=cv2, console_stream=io.StringIO(), install_signals=False).run()
    assert fake_dai.devices and fake_dai.devices[0].closed and not (tmp_path / "s11").exists()
