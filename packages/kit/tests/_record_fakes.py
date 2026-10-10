# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""Stand-ins for the DepthAI v2 module and an OpenCV V4L2 capture, for the `be record` tests.

FakeDai mimics the parts of depthai 2.x the recorder uses: node classes and enums, a Pipeline whose
nodes record every call, a Device that serves colour (JPEG bytes) and depth (uint16) messages in real
time at the configured rate with host-monotonic timestamps, and a calibration handler.
"""
from __future__ import annotations

import datetime
import threading
import time
import types

import numpy as np


class FakeEnum:
    def __init__(self, group, name):
        self.group, self.name = group, name

    def __repr__(self):
        return f"{self.group}.{self.name}"


def _enums(group, *names):
    return types.SimpleNamespace(**{n: FakeEnum(group, n) for n in names})


class _Port:
    def __init__(self, node, name):
        self.node, self.name, self.links = node, name, []

    def link(self, other):
        self.links.append(other)


class _Control:
    def __init__(self):
        self.calls = []

    def __getattr__(self, name):
        if name.startswith("set"):
            return lambda *a: self.calls.append((name, a)) or self
        raise AttributeError(name)


class _StereoConfig:
    def getConfidenceThreshold(self):
        return 245

    def getMedianFilter(self):
        return FakeEnum("MedianFilter", "KERNEL_5x5")

    def getLeftRightCheckThreshold(self):
        return 10


class FakeNode:
    PORTS = ("video", "isp", "out", "depth", "bitstream", "input", "left", "right", "disparity")

    def __init__(self, kind):
        self.kind = kind
        self.calls = []
        for p in self.PORTS:
            setattr(self, p, _Port(self, p))
        self.initialControl = _Control()
        self.initialConfig = _StereoConfig()

    def __getattr__(self, name):
        if name.startswith("set"):
            def rec(*a, **k):
                self.calls.append((name, a))
            return rec
        raise AttributeError(name)

    def called(self, name):
        return [a for n, a in self.calls if n == name]


def _node_class(name, **attrs):
    return type(name, (), attrs)


class FakeCalibration:
    """A 1920x1080 RGB calibration: fx = fy = 1500, principal point at the centre."""

    def __init__(self, calibrated_at=(1920, 1080), f=1500.0, lens_position=135):
        self.w, self.h, self.f, self.lens = calibrated_at[0], calibrated_at[1], f, lens_position

    def eepromToJson(self):
        return {"boardName": "FAKE-OAK-D", "cameraData": [[0, {"width": self.w, "height": self.h}]]}

    def getDefaultIntrinsics(self, socket):
        return [[self.f, 0.0, self.w / 2], [0.0, self.f, self.h / 2], [0.0, 0.0, 1.0]], self.w, self.h

    def getCameraIntrinsics(self, socket, w=-1, h=-1, *rest):
        if getattr(socket, "name", "") != "CAM_A":
            return [[440.0, 0.0, w / 2], [0.0, 440.0, h / 2], [0.0, 0.0, 1.0]]
        s = w / self.w
        return [[self.f * s, 0.0, self.w / 2 * s], [0.0, self.f * s, self.h / 2 * s], [0.0, 0.0, 1.0]]

    def getDistortionCoefficients(self, socket):
        return [0.0] * 14

    def getLensPosition(self, socket):
        return self.lens

    def getFov(self, socket):
        return 68.8

    def getBaselineDistance(self):
        return 7.5


class FakeMsg:
    def __init__(self, seq, t, data=None, frame=None, exposure_s=0.008):
        self.seq, self.t, self.data, self.frame, self.exposure = seq, t, data, frame, exposure_s

    def getSequenceNum(self):
        return self.seq

    def getTimestamp(self, offset=None):
        t = self.t - (self.exposure / 2 if offset is not None and offset.name == "MIDDLE" else 0.0)
        return datetime.timedelta(seconds=t)

    def getTimestampDevice(self):
        return datetime.timedelta(seconds=self.t - 1234.5)

    def getData(self):
        return np.frombuffer(self.data, dtype=np.uint8)

    def getFrame(self, copy=False):
        return self.frame.copy() if copy else self.frame


class FakeQueue:
    def __init__(self, dev, name, fps, make):
        self.dev, self.name, self.fps, self.make = dev, name, fps, make
        self.next_t = time.monotonic() + max(0.05, 1.0 / fps)
        self.seq = 0

    def tryGetAll(self):
        if self.dev.closed:
            raise RuntimeError("Communication exception - queue closed")
        if self.dev.fail_at is not None and time.monotonic() >= self.dev.fail_at:
            raise RuntimeError("X_LINK_ERROR (fake)")
        now, out = time.monotonic(), []
        while self.next_t <= now - 0.010:          # 10 ms of "latency" between exposure and arrival
            if self.seq not in self.dev.cfg.skip_seq.get(self.name, ()):
                out.append(self.make(self.seq, self.next_t))
            self.seq += 1
            self.next_t += 1.0 / self.fps
        return out


class FakeDevice:
    def __init__(self, dai, *args):
        self.dai, self.args = dai, args
        self.cfg = dai.cfg
        self.closed = False
        self.pipeline = None
        n = len(dai.devices)
        self.fail_at = (time.monotonic() + self.cfg.fail_after_s) if (self.cfg.fail_after_s and n == 0) else None
        dai.devices.append(self)

    def readCalibration(self):
        return self.cfg.calibration

    def startPipeline(self, pipeline):
        self.pipeline = pipeline
        return True

    def isPipelineRunning(self):
        return self.pipeline is not None

    def getOutputQueue(self, name, maxSize=8, blocking=True):
        cfg = self.cfg
        if name == "color":
            return FakeQueue(self, name, cfg.color_fps, lambda s, t: FakeMsg(s, t, data=cfg.color_jpeg(s)))
        if name == "depth":
            return FakeQueue(self, name, cfg.depth_fps, lambda s, t: FakeMsg(s, t, frame=cfg.depth_frame(s)))
        raise RuntimeError(f"no stream {name}")

    def getMxId(self):
        return "18443010FAKE0000"

    def getDeviceName(self):
        return "OAK-D"

    def getProductName(self):
        return "OAK-D (fake)"

    def getUsbSpeed(self):
        return self.dai.UsbSpeed.SUPER

    def getCameraSensorNames(self):
        S = self.dai.CameraBoardSocket
        return {S.CAM_A: "IMX378", S.CAM_B: "OV9282", S.CAM_C: "OV9282"}

    def isClosed(self):
        return self.closed

    def close(self):
        self.closed = True


class FakeDaiConfig:
    def __init__(self, *, color_jpeg=None, depth_frame=None, color_fps=30.0, depth_fps=30.0, skip_seq=None,
                 fail_after_s=None, calibration=None):
        self._jpeg = color_jpeg
        self._depth = depth_frame
        self.color_fps, self.depth_fps = color_fps, depth_fps
        self.skip_seq = skip_seq or {}
        self.fail_after_s = fail_after_s
        self.calibration = calibration or FakeCalibration()

    def color_jpeg(self, seq):
        j = self._jpeg
        return j(seq) if callable(j) else j

    def depth_frame(self, seq):
        d = self._depth
        return d(seq) if callable(d) else d


def make_fake_dai(version="2.33.0.0", **cfg):
    """A module-like namespace standing in for `depthai`."""
    dai = types.SimpleNamespace()
    dai.__version__ = version
    dai.cfg = FakeDaiConfig(**cfg)
    dai.devices = []
    dai.pipelines = []
    dai.CameraBoardSocket = _enums("CameraBoardSocket", "CAM_A", "CAM_B", "CAM_C", "RGB", "LEFT", "RIGHT")
    dai.ColorCameraProperties = types.SimpleNamespace(
        SensorResolution=_enums("SensorResolution", "THE_1080_P", "THE_4_K", "THE_12_MP", "THE_720_P"))
    dai.MonoCameraProperties = types.SimpleNamespace(
        SensorResolution=_enums("SensorResolution", "THE_400_P", "THE_480_P", "THE_720_P", "THE_800_P"))
    dai.VideoEncoderProperties = types.SimpleNamespace(Profile=_enums("Profile", "MJPEG", "H264_MAIN", "H265_MAIN"))
    dai.MedianFilter = _enums("MedianFilter", "MEDIAN_OFF", "KERNEL_3x3", "KERNEL_5x5", "KERNEL_7x7")
    dai.UsbSpeed = _enums("UsbSpeed", "HIGH", "SUPER", "SUPER_PLUS", "UNKNOWN")
    dai.CameraExposureOffset = _enums("CameraExposureOffset", "START", "MIDDLE", "END")
    dai.node = types.SimpleNamespace(
        ColorCamera=_node_class("ColorCamera"), MonoCamera=_node_class("MonoCamera"),
        StereoDepth=_node_class("StereoDepth", PresetMode=_enums("PresetMode", "HIGH_DENSITY", "HIGH_ACCURACY")),
        VideoEncoder=_node_class("VideoEncoder"), XLinkOut=_node_class("XLinkOut"))

    class Pipeline:
        def __init__(self):
            self.nodes = []
            dai.pipelines.append(self)

        def create(self, cls):
            n = FakeNode(cls.__name__)
            self.nodes.append(n)
            return n

        def by_kind(self, kind):
            return [n for n in self.nodes if n.kind == kind]

    dai.Pipeline = Pipeline

    class Device(FakeDevice):
        def __init__(self, *args):
            super().__init__(dai, *args)

        @staticmethod
        def getAllConnectedDevices():
            return []

    dai.Device = Device
    dai.DeviceInfo = lambda s: ("DeviceInfo", s)
    return dai


def jpeg_bytes(cv2, w=1280, h=720, text="frame", value=90):
    img = np.full((h, w, 3), value, np.uint8)
    cv2.putText(img, text, (40, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (255, 255, 255), 3)
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
    assert ok
    return buf.tobytes()


class FakeCapture:
    """cv2.VideoCapture over V4L2, as far as WristSource uses it.

    raw_ok: whether CAP_PROP_CONVERT_RGB = 0 yields the camera's MJPEG bytes (as OpenCV's V4L2 backend does).
    """

    def __init__(self, cv2, device, *, jpeg=None, raw_ok=True, fourcc="MJPG", fps=30.0, size=(640, 480),
                 opened=True, fail_after_s=None):
        self.cv2, self.device = cv2, device
        self.jpeg = jpeg or jpeg_bytes(cv2, *size, text="wrist")
        self.bgr = cv2.imdecode(np.frombuffer(self.jpeg, np.uint8), cv2.IMREAD_COLOR)
        self.raw_ok, self.fourcc, self.fps, self.size, self.opened = raw_ok, fourcc, fps, size, opened
        self.convert = True
        self.props = {}
        self.next_t = time.monotonic()
        self.last_t = None
        self.released = False
        self.fail_at = time.monotonic() + fail_after_s if fail_after_s else None
        self._lock = threading.Lock()

    def isOpened(self):
        return self.opened

    def set(self, prop, value):
        cv2 = self.cv2
        self.props[prop] = value
        if prop == cv2.CAP_PROP_CONVERT_RGB:
            if not value and not self.raw_ok:
                return False
            self.convert = bool(value)
            return True
        if prop == cv2.CAP_PROP_AUTOFOCUS:
            return False                     # a fixed-focus camera
        return True

    def get(self, prop):
        cv2 = self.cv2
        if prop == cv2.CAP_PROP_FOURCC:
            return float(cv2.VideoWriter_fourcc(*self.fourcc))
        if prop == cv2.CAP_PROP_FRAME_WIDTH:
            return float(self.size[0])
        if prop == cv2.CAP_PROP_FRAME_HEIGHT:
            return float(self.size[1])
        if prop == cv2.CAP_PROP_FPS:
            return float(self.fps)
        if prop == cv2.CAP_PROP_POS_MSEC:
            return 0.0 if self.last_t is None else (self.last_t - 0.004) * 1000.0
        if prop == cv2.CAP_PROP_CONVERT_RGB:
            return float(self.convert)
        return -1.0

    def grab(self):
        if self.released:
            return False
        if self.fail_at is not None and time.monotonic() >= self.fail_at:
            return False
        now = time.monotonic()
        if self.next_t > now:
            time.sleep(self.next_t - now)
        self.last_t = self.next_t
        self.next_t += 1.0 / self.fps
        return True

    def retrieve(self):
        if not self.convert and self.raw_ok and self.fourcc == "MJPG":
            return True, np.frombuffer(self.jpeg, np.uint8).reshape(1, -1).copy()
        return True, self.bgr.copy()

    def read(self):
        if not self.grab():
            return False, None
        return self.retrieve()

    def release(self):
        self.released = True
