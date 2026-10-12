# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""The OAK-D side: DepthAI version gate, the v2 pipeline, calibration and intrinsics."""
from __future__ import annotations

import sys
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from _record_fakes import FakeCalibration, make_fake_dai  # noqa: E402

from bounded_embodiment_kit.instruments import record  # noqa: E402


def _settings(**kw):
    base = dict(color_fps=30.0, depth_fps=30.0, mjpeg_quality=85, stereo_preset="HIGH_DENSITY", median="preset",
                rgb_exposure_us=None, rgb_iso=400, rgb_focus=None, mono_resolution=None, oak_device=None)
    base.update(kw)
    return types.SimpleNamespace(**base)


@pytest.mark.parametrize("version", ["3.0.0", "3.2.1", "3.10.0"])
def test_depthai_v3_is_refused_with_a_clear_message(monkeypatch, version):
    monkeypatch.setitem(sys.modules, "depthai", types.SimpleNamespace(__version__=version))
    with pytest.raises(record.RecordError) as e:
        record.import_depthai()
    msg = str(e.value)
    assert "DepthAI v3" in msg and "depthai>=2.24,<3" in msg


def test_the_oakd_extra_installs_the_depthai_the_recorder_accepts():
    """pip install "bounded-embodiment-kit[oakd]" must not bring in DepthAI v3, which the recorder refuses."""
    from importlib.metadata import requires

    lines = [r for r in requires("bounded-embodiment-kit") or [] if r.startswith("depthai") and "oakd" in r]
    assert lines, "the oakd extra no longer lists depthai"
    spec = lines[0].split(";")[0].replace(" ", "")
    assert ">=2.24" in spec and "<3" in spec, lines[0]


def test_old_depthai_v2_is_refused(monkeypatch):
    monkeypatch.setitem(sys.modules, "depthai", types.SimpleNamespace(__version__="2.21.2.0"))
    with pytest.raises(record.RecordError, match="older than 2.24"):
        record.import_depthai()


def test_missing_depthai_says_how_to_install(monkeypatch):
    monkeypatch.setitem(sys.modules, "depthai", None)
    with pytest.raises(record.RecordError, match="pip install"):
        record.import_depthai()


def test_v2_is_accepted(monkeypatch):
    fake = make_fake_dai("2.24.0.0")
    monkeypatch.setitem(sys.modules, "depthai", fake)
    assert record.import_depthai() is fake


def test_cli_exits_2_on_depthai_v3(monkeypatch, tmp_path, capsys):
    monkeypatch.setitem(sys.modules, "depthai", types.SimpleNamespace(__version__="3.2.1"))
    rc = record.main(["--out", str(tmp_path / "s"), "--wrist", "none", "--seconds", "5", "--no-stdin-events"])
    assert rc == 2
    assert "DepthAI v3" in capsys.readouterr().err
    assert not (tmp_path / "s").exists()


def test_pipeline_is_what_the_session_needs():
    dai = make_fake_dai()
    p, nodes = record.build_oak_pipeline(dai, _settings(depth_fps=15.0), lens_position=135,
                                         mono_resolution="THE_400_P")
    cam, enc, st = nodes["color"], nodes["encoder"], nodes["stereo"]
    S = dai.CameraBoardSocket
    assert cam.called("setBoardSocket") == [(S.CAM_A,)]
    assert cam.called("setResolution") == [(dai.ColorCameraProperties.SensorResolution.THE_1080_P,)]
    assert cam.called("setIspScale") == [(2, 3)] and cam.called("setVideoSize") == [(1280, 720)]
    assert cam.called("setFps") == [(30.0,)]
    assert ("setManualFocus", (135,)) in cam.initialControl.calls
    assert enc.called("setDefaultProfilePreset") == [(30.0, dai.VideoEncoderProperties.Profile.MJPEG)]
    assert enc.called("setQuality") == [(85,)]
    assert enc.input in cam.video.links
    for mono, sock in ((nodes["left"], S.CAM_B), (nodes["right"], S.CAM_C)):
        assert mono.called("setBoardSocket") == [(sock,)]
        assert mono.called("setResolution") == [(dai.MonoCameraProperties.SensorResolution.THE_400_P,)]
        assert mono.called("setFps") == [(15.0,)]
    names = [n for n, _ in st.calls]
    assert names.index("setDefaultProfilePreset") < names.index("setLeftRightCheck")   # preset first
    assert st.called("setDefaultProfilePreset") == [(dai.node.StereoDepth.PresetMode.HIGH_DENSITY,)]
    assert st.called("setLeftRightCheck") == [(True,)] and st.called("setSubpixel") == [(True,)]
    assert st.called("setExtendedDisparity") == [(False,)]
    assert st.called("setDepthAlign") == [(S.CAM_A,)] and st.called("setOutputSize") == [(640, 360)]
    assert st.called("setMedianFilter") == []                                            # the preset's
    assert st.left in nodes["left"].out.links and st.right in nodes["right"].out.links
    xouts = p.by_kind("XLinkOut")
    streams = {x.called("setStreamName")[0][0]: x for x in xouts}
    assert set(streams) == {"color", "depth"}
    assert streams["color"].input in enc.bitstream.links
    assert streams["depth"].input in st.depth.links


def test_pipeline_options():
    dai = make_fake_dai()
    _, nodes = record.build_oak_pipeline(dai, _settings(median="off", stereo_preset="HIGH_ACCURACY",
                                                        rgb_exposure_us=4000, rgb_iso=800), lens_position=None)
    st, cam = nodes["stereo"], nodes["color"]
    assert st.called("setMedianFilter") == [(dai.MedianFilter.MEDIAN_OFF,)]
    assert st.called("setDefaultProfilePreset") == [(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY,)]
    assert ("setManualExposure", (4000, 800)) in cam.initialControl.calls
    assert not [c for c in cam.initialControl.calls if c[0] == "setManualFocus"]


def test_open_oak_reads_calibration_and_starts():
    dai = make_fake_dai(color_jpeg=b"\xff\xd8x\xff\xd9", depth_frame=None)
    h = record.open_oak(dai, _settings())
    dev = dai.devices[0]
    assert dev.pipeline is not None
    info = h.info
    assert info["usb_speed"] == "SUPER" and info["mxid"] == "18443010FAKE0000"
    assert info["sensors"] == {"CAM_A": "IMX378", "CAM_B": "OV9282", "CAM_C": "OV9282"}
    assert info["color"]["lens_position"] == 135 and "calibration" in info["color"]["focus"]
    assert info["depth"]["mono_resolution"] == "THE_400_P"
    intr = info["calibration"]["intrinsics"]
    assert intr["rgb_1280x720"][0][0] == pytest.approx(1000.0) and intr["rgb_640x360"][0][0] == pytest.approx(500.0)
    assert intr["rgb_calibrated_at"]["width"] == 1920 and "rgb_1280x720_crop_aware" not in intr
    assert info["calibration"]["eeprom"]["boardName"] == "FAKE-OAK-D"
    h.close()
    assert dev.closed


def test_12mp_calibration_adds_the_crop_aware_candidate():
    dai = make_fake_dai(calibration=FakeCalibration(calibrated_at=(4056, 3040), f=3000.0))
    cal = record.calibration_info(dai, dai.cfg.calibration)
    k = cal["intrinsics"]["rgb_1280x720_crop_aware"]
    assert k[0][0] == pytest.approx(1000.0)                    # 3000 / 3: 3840 px of sensor -> 1280
    assert k[0][2] == pytest.approx((2028 - 108) / 3) and k[1][2] == pytest.approx((1520 - 440) / 3)
    assert cal["intrinsics"]["rgb_1280x720"][0][0] == pytest.approx(3000 * 1280 / 4056)   # what DepthAI says


def test_ov7251_mono_uses_480p():
    dai = make_fake_dai()
    S = dai.CameraBoardSocket
    dai.Device.getCameraSensorNames = lambda self: {S.CAM_A: "IMX214", S.CAM_B: "OV7251", S.CAM_C: "OV7251"}
    h = record.open_oak(dai, _settings())
    assert h.info["depth"]["mono_resolution"] == "THE_480_P"
    h.close()


def test_real_depthai_v2_accepts_the_pipeline():
    """With the real depthai 2.x installed (no device needed), every call the recorder makes exists."""
    dai = pytest.importorskip("depthai")
    if record._version_tuple(dai.__version__)[0] >= 3:
        pytest.skip("depthai v3 installed")
    p, nodes = record.build_oak_pipeline(dai, _settings(), lens_position=135)
    assert len(p.getAllNodes()) == 7
    cfg = record.stereo_config(nodes["stereo"])
    assert cfg["left_right_check"] and cfg["subpixel"] and not cfg["extended_disparity"]
    assert not cfg["temporal_filter"] and not cfg["spatial_filter"] and cfg["decimation"] == 1
    assert nodes["color"].getIspSize() == (1280, 720) and nodes["color"].getVideoSize() == (1280, 720)
    p.serializeToJson()
