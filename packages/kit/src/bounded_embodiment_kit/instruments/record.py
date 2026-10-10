# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""Record the OAK-D and the gripper camera for the markerless arm instrument (`be record`).

What it records. Nothing comes from the robot: no encoder readings and no commands.

* OAK-D, through the DepthAI v2 API:
  - colour from CAM_A, 1080p scaled by the ISP to 1280x720 and MJPEG-encoded on the device. It goes to
    one ``oakd/color.mjpeg`` stream (concatenated JPEGs), indexed by ``oakd/color_index.csv``.
  - stereo depth from CAM_B and CAM_C at 400p, with left-right check and subpixel on, aligned to CAM_A
    at 640x360. Values are uint16 millimetres (0 = no depth), in compressed ``oakd/depth/*.npz``
    chunks indexed by ``oakd/depth_index.csv``.
  - the device calibration (``eepromToJson``) and the intrinsics for 1280x720 and 640x360, in meta.json.
* The wrist camera (UVC, V4L2): the camera's own MJPEG bytes, untouched when OpenCV can pass them
  through and re-encoded as JPEG quality 90 otherwise, in ``wrist/wrist.mjpeg`` indexed by
  ``wrist/wrist_index.csv``.
* ``events.csv``: labelled events, typed at the keyboard (label, then Enter) or written to --events-fifo.
* ``clock.csv``: pairs of (time.time(), time.monotonic()) at the start, every 5 s, and at the stop.

Timestamps. Every ``t_mono_s`` is on the host's CLOCK_MONOTONIC, the clock Python's time.monotonic()
reads.
* DepthAI v2 syncs ImgFrame.getTimestamp() to the host's steady clock, which on Linux is CLOCK_MONOTONIC.
* For the wrist camera, t_mono_s is time.monotonic() when grab() returned. ``t_v4l2_s`` is the driver's
  own buffer timestamp.
* clock.csv maps wall-clock logs onto this timebase. The arm harness logs time.time().

  be record --out DIR [--seconds S] [--wrist auto|none|DEVICE] [--depth-fps 15] [--events-fifo PATH]
  be record check DIR [--quick]       fps, gaps, drops, timestamps and board views; writes a contact sheet
  be record board --out board.png --pdf board.pdf
  be record devices                   what is plugged in, and which process holds it
  be record mark LABEL --fifo PATH    log an event from another terminal
  be record stop DIR                  stop a recorder whose terminal was lost

It also runs standalone: ``python -m bounded_embodiment_kit.instruments.record ...``, or as a single
copied file, ``python record.py ...``. It needs only numpy, opencv-python-headless (for the wrist
camera, check and board) and, for the OAK-D, ``depthai>=2.24,<3``.
"""
from __future__ import annotations

import argparse
import csv
import datetime as _dt
import errno
import glob
import io
import json
import math
import os
import platform
import queue
import re
import select
import shutil
import signal
import stat
import sys
import threading
import time
import warnings
import zipfile
from pathlib import Path

import numpy as np

RECORDER_VERSION = "0.1.0"
FORMAT = "be-camera-recording/1"

COLOR_W, COLOR_H = 1280, 720
DEPTH_W, DEPTH_H = 640, 360

COLOR_COLUMNS = ("frame_idx", "device_seq", "t_mono_s", "byte_offset", "byte_len",
                 "t_mid_s", "t_dev_s", "t_host_s", "oak_session")
DEPTH_COLUMNS = ("frame_idx", "device_seq", "t_mono_s", "chunk", "index_in_chunk",
                 "t_mid_s", "t_dev_s", "t_host_s", "valid_frac", "oak_session")
WRIST_COLUMNS = ("frame_idx", "t_mono_s", "byte_offset", "byte_len", "t_v4l2_s", "raw", "wrist_session")
EVENT_COLUMNS = ("n", "t_mono_s", "t_wall_s", "label", "source")
CLOCK_COLUMNS = ("kind", "t_wall_s", "t_mono_s", "uncertainty_s")

PATHS = {
    "meta": "meta.json",
    "events": "events.csv",
    "clock": "clock.csv",
    "log": "recorder.log",
    "pid": "recorder.pid",
    "measurements": "measurements.txt",
    "color_stream": "oakd/color.mjpeg",
    "color_index": "oakd/color_index.csv",
    "depth_dir": "oakd/depth",
    "depth_index": "oakd/depth_index.csv",
    "wrist_stream": "wrist/wrist.mjpeg",
    "wrist_index": "wrist/wrist_index.csv",
    "check_dir": "check",
}

# The board `be record board` prints and `be record check` looks for.
BOARD_SPEC = {
    "type": "charuco",
    "dictionary": "DICT_5X5_100",
    "squares_x": 5,
    "squares_y": 7,
    "square_length_m": 0.030,
    "marker_length_m": 0.022,
    "legacy_pattern": False,
}

# Disk-use estimates, deliberately on the high side (the live status line shows the real rate).
DEPTH_RATIO_EST = 0.35     # deflate level 1 on subpixel depth; synthetic scenes gave 0.24
WRIST_BPP_EST = 2.0        # UVC cameras' own MJPEG at 640x480: about 40-80 KB a frame
DISK_FACTOR = 1.5
DEFAULT_PLAN_SECONDS = 1200.0

STEREO_PRESETS = ("HIGH_DENSITY", "HIGH_ACCURACY")
OAK_RETRY_S = 3.0          # wait between OAK-D reconnect attempts
OAK_STALL_S = 5.0          # no OAK-D frames for this long counts as a lost device
WRIST_RETRY_S = 2.0        # wait between wrist-camera reopen attempts
MEDIAN = {"off": "MEDIAN_OFF", "3": "KERNEL_3x3", "5": "KERNEL_5x5", "7": "KERNEL_7x7"}


class RecordError(RuntimeError):
    """A problem the operator can fix; the CLI prints it without a traceback."""


# ----- small helpers -------------------------------------------------------------------------------
def _version_tuple(text):
    nums = []
    for part in str(text).split("."):
        m = re.match(r"\d+", part)
        if not m:
            break
        nums.append(int(m.group()))
    while len(nums) < 2:
        nums.append(0)
    return tuple(nums)


def import_depthai():
    """Import DepthAI and insist on v2 (>= 2.24, < 3)."""
    try:
        import depthai as dai  # noqa: PLC0415
    except ImportError as e:
        raise RecordError("DepthAI is not installed in this Python. The recorder needs DepthAI v2:\n"
                          "  pip install 'depthai>=2.24,<3'") from e
    ver = str(getattr(dai, "__version__", "0"))
    major, minor = _version_tuple(ver)[:2]
    if major >= 3:
        raise RecordError(
            f"depthai {ver} is DepthAI v3. This recorder is written for the DepthAI v2 API (ColorCamera, "
            "MonoCamera and XLinkOut nodes; dai.Device + startPipeline), which v3 removed. Use a venv of its "
            "own with v2:\n  python3 -m venv ~/venvs/be-rec && ~/venvs/be-rec/bin/pip install 'depthai>=2.24,<3'"
            "\n(A v3 port comes later.)")
    if (major, minor) < (2, 24):
        raise RecordError(f"depthai {ver} is older than 2.24: pip install -U 'depthai>=2.24,<3'")
    return dai


def import_cv2():
    try:
        import cv2  # noqa: PLC0415
    except ImportError as e:
        raise RecordError("OpenCV is not installed: pip install 'opencv-python-headless>=4.9,<4.14'") from e
    return cv2


def _name(x):
    n = getattr(x, "name", None)
    return n if isinstance(n, str) else str(x).split(".")[-1]


def _jsonable(o):
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.floating):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, Path):
        return str(o)
    if isinstance(o, _dt.timedelta):
        return o.total_seconds()
    return _name(o)


def _clean(o):
    """Replace NaN and infinities by None, so the JSON is strict."""
    if isinstance(o, float):
        return o if math.isfinite(o) else None
    if isinstance(o, dict):
        return {str(k): _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    return o


def write_json_atomic(path, obj):
    path = Path(path)
    tmp = path.with_name(path.name + ".part")
    tmp.write_text(json.dumps(_clean(json.loads(json.dumps(obj, default=_jsonable))), indent=2) + "\n",
                   encoding="utf-8")
    os.replace(tmp, path)


def _fmt(v):
    if v is None:
        return ""
    if isinstance(v, (bool, np.bool_)):
        return "1" if v else "0"
    if isinstance(v, (float, np.floating)):
        return "" if not math.isfinite(float(v)) else f"{float(v):.6f}"
    return str(v)


def _csv_line(values):
    return ",".join(_fmt(v) for v in values) + "\n"


def clock_pair():
    """(t_wall_s, t_mono_s, uncertainty_s): the tightest of five back-to-back readings."""
    best = None
    for _ in range(5):
        m0 = time.monotonic()
        w = time.time()
        m1 = time.monotonic()
        if best is None or m1 - m0 < best[2] * 2:
            best = (w, (m0 + m1) / 2, (m1 - m0) / 2)
    return best


def _iso(t=None):
    return _dt.datetime.fromtimestamp(time.time() if t is None else t, _dt.timezone.utc).astimezone().isoformat(
        timespec="seconds")


def free_bytes(path):
    p = Path(path).expanduser().absolute()
    while not p.exists():
        if p.parent == p:
            break
        p = p.parent
    return shutil.disk_usage(p).free


def clean_label(text):
    label = re.sub(r"\s+", "_", str(text).strip())
    label = re.sub(r"[^A-Za-z0-9_.:+-]", "", label)[:64]
    return label or "mark"


def parse_size(text):
    m = re.fullmatch(r"\s*(\d+)\s*[xX]\s*(\d+)\s*", str(text))
    if not m:
        raise argparse.ArgumentTypeError(f"expected WIDTHxHEIGHT, got {text!r}")
    return int(m.group(1)), int(m.group(2))


def _fourcc_str(v):
    v = int(v)
    return "".join(chr((v >> (8 * i)) & 0xFF) for i in range(4)).strip("\x00")


def _gb(nbytes):
    return nbytes / 1e9


# ----- disk estimate ------------------------------------------------------------------------------------
def _mjpeg_bpp(quality):
    for q, bpp in ((70, 0.8), (80, 1.0), (85, 1.2), (90, 1.5), (95, 2.2)):
        if quality <= q:
            return bpp
    return 3.5


def estimate_rates(*, oak=True, color_fps=30.0, mjpeg_quality=85, depth_fps=30.0, wrist=True,
                   wrist_size=(640, 480), wrist_fps=30.0):
    """Estimated data rates in MB/s (1 MB = 1e6 bytes), on the high side."""
    color = COLOR_W * COLOR_H * _mjpeg_bpp(mjpeg_quality) / 8 * color_fps / 1e6 if oak else 0.0
    depth = DEPTH_W * DEPTH_H * 2 * DEPTH_RATIO_EST * depth_fps / 1e6 if oak else 0.0
    wr = wrist_size[0] * wrist_size[1] * WRIST_BPP_EST / 8 * wrist_fps / 1e6 if wrist else 0.0
    return {"color": round(color, 3), "depth": round(depth, 3), "wrist": round(wr, 3),
            "total": round(color + depth + wr, 3)}


def disk_need(rates, seconds, factor=DISK_FACTOR):
    """Bytes needed: seconds x estimated rate x factor."""
    return seconds * rates["total"] * 1e6 * factor


# ----- who holds a device (Linux /proc scan, like the harness's serial-port check) ----------------
def holders(path):
    """[(pid, cmdline)] of processes with `path` open; also how many processes could not be inspected."""
    real = os.path.realpath(path)
    found, hidden = [], 0
    me = os.getpid()
    for pid in filter(str.isdigit, os.listdir("/proc")) if os.path.isdir("/proc") else []:
        if int(pid) == me:
            continue
        try:
            fds = os.listdir(f"/proc/{pid}/fd")
        except OSError:
            hidden += 1
            continue
        for fd in fds:
            try:
                if os.path.realpath(f"/proc/{pid}/fd/{fd}") == real:
                    cmd = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode(
                        "utf-8", "replace").strip()[:160]
                    found.append((int(pid), cmd))
                    break
            except OSError:
                continue
    return found, hidden


def list_v4l2():
    """Video4Linux capture nodes with their names and /dev/v4l/by-id links."""
    by_id = {}
    for link in sorted(glob.glob("/dev/v4l/by-id/*")):
        by_id.setdefault(os.path.realpath(link), []).append(link)
    out = []
    for sysdir in sorted(glob.glob("/sys/class/video4linux/video*"), key=lambda s: int(re.sub(r"\D", "", s) or 0)):
        dev = "/dev/" + os.path.basename(sysdir)
        try:
            name = Path(sysdir, "name").read_text().strip()
        except OSError:
            name = "?"
        try:
            index = int(Path(sysdir, "index").read_text().strip())
        except (OSError, ValueError):
            index = None
        out.append({"device": dev, "name": name, "index": index, "by_id": by_id.get(os.path.realpath(dev), []),
                    "usb": bool(by_id.get(os.path.realpath(dev)))})
    return out


def find_wrist_camera():
    """The first USB camera's capture node (/dev/v4l/by-id/*-video-index0), skipping Luxonis devices."""
    links = sorted(glob.glob("/dev/v4l/by-id/*-video-index0"))
    good = [p for p in links if not re.search(r"luxonis|movidius|oak", p, re.I)]
    return good[0] if good else None, good


def list_oak_usb():
    """Movidius (vendor 03e7) USB devices from sysfs, with link speed and the processes holding them."""
    out = []
    for d in sorted(glob.glob("/sys/bus/usb/devices/*")):
        try:
            if Path(d, "idVendor").read_text().strip() != "03e7":
                continue
            pid = Path(d, "idProduct").read_text().strip()
            bus = int(Path(d, "busnum").read_text())
            num = int(Path(d, "devnum").read_text())
            speed = Path(d, "speed").read_text().strip()
        except (OSError, ValueError):
            continue
        node = f"/dev/bus/usb/{bus:03d}/{num:03d}"
        held, _ = holders(node) if os.path.exists(node) else ([], 0)
        out.append({"sysfs": d, "product_id": pid, "node": node, "speed_mbps": speed,
                    "state": {"2485": "not booted (USB2 bootloader)", "f63b": "booted (in use or idle)",
                              "f63c": "bootloader"}.get(pid, pid), "held_by": held})
    return out


# ----- per-stream counters ------------------------------------------------------------------------
class StreamStats:
    """Counters for one stream. Each field has one writer thread; the status thread only reads."""

    def __init__(self, name, nominal_fps):
        self.name, self.nominal_fps = name, nominal_fps
        self.received = self.dropped = self.written = self.bytes = 0
        self.seq_gaps = self.seq_resets = self.not_increasing = self.encode_errors = 0
        self.first_t = self.last_t = self.last_seq = None
        self._last_count, self._last_when = 0, time.monotonic()

    def on_frame(self, t, seq=None):
        self.received += 1
        if self.first_t is None:
            self.first_t = t
        if self.last_t is not None and t <= self.last_t:
            self.not_increasing += 1
        self.last_t = t
        if seq is not None:
            if self.last_seq is not None:
                d = seq - self.last_seq
                if d > 1:
                    self.seq_gaps += d - 1
                elif d <= 0:
                    self.seq_resets += 1
            self.last_seq = seq

    def window_fps(self):
        now, n = time.monotonic(), self.received
        dt = now - self._last_when
        fps = (n - self._last_count) / dt if dt > 0 else 0.0
        self._last_count, self._last_when = n, now
        return fps

    def as_dict(self, duration_s):
        span = (self.last_t - self.first_t) if (self.first_t is not None and self.last_t is not None) else None
        return {"frames_received": self.received, "frames_written": self.written,
                "dropped_queue_full": self.dropped, "device_seq_missing": self.seq_gaps,
                "device_seq_resets": self.seq_resets, "timestamps_not_increasing": self.not_increasing,
                "encode_errors": self.encode_errors, "bytes": self.bytes,
                "mean_fps": round((self.received - 1) / span, 3) if span else None,
                "mb_per_s": round(self.bytes / 1e6 / duration_s, 3) if duration_s else None,
                "first_t_mono_s": self.first_t, "last_t_mono_s": self.last_t}


# ----- writers ------------------------------------------------------------------------------------
class _Stop:
    pass


_STOP = _Stop()


class _QueueWriter(threading.Thread):
    def __init__(self, name, stats, max_queue):
        super().__init__(name=f"write-{name}", daemon=True)
        self.stats = stats
        self.q = queue.Queue(maxsize=max_queue)
        self.error = None
        self.frames = 0

    def put(self, item):
        """Never blocks: a full queue drops the frame and counts it."""
        try:
            self.q.put_nowait(item)
            return True
        except queue.Full:
            self.stats.dropped += 1
            return False

    def finish(self, timeout=120.0):
        if self.ident is None:              # never started
            return
        try:
            self.q.put(_STOP, timeout=timeout)
        except queue.Full:
            self.error = self.error or "writer did not drain its queue at stop"
            return
        self.join(timeout)

    def run(self):
        try:
            self._run()
        except BaseException as e:  # noqa: BLE001 - report, then keep draining so producers never stall
            self.error = f"{type(e).__name__}: {e}"
            while True:
                try:
                    if self.q.get(timeout=0.5) is _STOP:
                        return
                except queue.Empty:
                    continue


class MjpegWriter(_QueueWriter):
    """Appends JPEG frames to one stream file and indexes them.

    Items are (before, payload, after): the index row is frame_idx, *before, byte_offset, byte_len, *after.
    payload is bytes (or a 1-D uint8 array) of a JPEG, or an HxWx3 BGR array to encode here.
    The index is written only after the bytes it points to, so a reader never sees a row whose frame
    is not in the stream yet (`check` can run on a live session).
    """

    def __init__(self, stream_path, index_path, columns, stats, *, max_queue=120, flush_s=0.5,
                 jpeg_quality=90):
        super().__init__(stats.name, stats, max_queue)
        self.stream_path, self.index_path, self.columns = Path(stream_path), Path(index_path), columns
        self.flush_s, self.jpeg_quality = flush_s, jpeg_quality

    def _payload_bytes(self, payload):
        if isinstance(payload, (bytes, bytearray, memoryview)):
            return payload
        arr = np.asarray(payload)
        if arr.ndim == 3:
            cv2 = import_cv2()
            ok, buf = cv2.imencode(".jpg", arr, [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)])
            if not ok:
                self.stats.encode_errors += 1
                return None
            return buf.tobytes()
        return np.ascontiguousarray(arr, dtype=np.uint8).tobytes()

    def _run(self):
        new_index = not self.index_path.exists() or self.index_path.stat().st_size == 0
        with open(self.stream_path, "ab") as fs, open(self.index_path, "a", newline="", encoding="utf-8") as fi:
            if new_index:
                fi.write(",".join(self.columns) + "\n")
                fi.flush()
            offset, pending, last_flush = fs.tell(), [], time.monotonic()
            while True:
                try:
                    item = self.q.get(timeout=self.flush_s)
                except queue.Empty:
                    item = None
                if item is _STOP:
                    break
                if item is not None:
                    before, payload, after = item
                    data = self._payload_bytes(payload)
                    if data is not None:
                        n = len(data)
                        fs.write(data)
                        pending.append(_csv_line((self.frames, *before, offset, n, *after)))
                        offset += n
                        self.frames += 1
                        self.stats.written += 1
                        self.stats.bytes += n
                now = time.monotonic()
                if pending and (item is None or now - last_flush >= self.flush_s):
                    fs.flush()
                    fi.write("".join(pending))
                    fi.flush()
                    pending.clear()
                    last_flush = now
            fs.flush()
            fi.write("".join(pending))
            fi.flush()
            os.fsync(fs.fileno())
            os.fsync(fi.fileno())


def write_npz(path, compresslevel=1, **arrays):
    """np.savez_compressed's format, with a chosen deflate level, written atomically.

    Level 1 rather than savez_compressed's 6: on subpixel depth, level 6 compressed at about 13 MB/s
    on a fast x86 core, slower than the raw depth stream (640x360x2 bytes at 30 fps = 13.8 MB/s),
    while level 1 ran at about 85 MB/s for files 14 % larger.
    """
    path = Path(path)
    tmp = path.with_name(path.name + ".part")
    with open(tmp, "wb") as fh:
        with zipfile.ZipFile(fh, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=compresslevel,
                             allowZip64=True) as zf:
            for name, arr in arrays.items():
                with zf.open(name + ".npy", "w", force_zip64=True) as f:
                    np.lib.format.write_array(f, np.asanyarray(arr), allow_pickle=False)
    os.replace(tmp, path)
    return path.stat().st_size


class DepthChunkWriter(_QueueWriter):
    """Collects depth frames into chunks of `chunk_frames` and writes each as an .npz.

    Each chunk holds `depth` (n, H, W) uint16 mm, `frame_idx` (n,), `t_mono_s` (n,) and `device_seq`
    (n,). A chunk's index rows are written after its file is in place.
    """

    def __init__(self, depth_dir, index_path, stats, *, chunk_frames=150, compresslevel=1, max_queue=None):
        super().__init__(stats.name, stats, max_queue or chunk_frames + 90)
        self.depth_dir, self.index_path = Path(depth_dir), Path(index_path)
        self.chunk_frames, self.compresslevel = int(chunk_frames), int(compresslevel)
        self.chunks = 0
        self.shapes = []

    def _write_chunk(self, fi, buf, n, rows, idx, ts, seqs):
        name = f"depth_{self.chunks:06d}.npz"
        size = write_npz(self.depth_dir / name, self.compresslevel, depth=buf[:n],
                         frame_idx=np.asarray(idx, np.int64), t_mono_s=np.asarray(ts, np.float64),
                         device_seq=np.asarray(seqs, np.int64))
        fi.write("".join(r.replace("@CHUNK@", name) for r in rows))
        fi.flush()
        self.chunks += 1
        self.stats.written += n
        self.stats.bytes += size

    def _run(self):
        self.depth_dir.mkdir(parents=True, exist_ok=True)
        new_index = not self.index_path.exists() or self.index_path.stat().st_size == 0
        with open(self.index_path, "a", newline="", encoding="utf-8") as fi:
            if new_index:
                fi.write(",".join(DEPTH_COLUMNS) + "\n")
                fi.flush()
            buf, n, rows, idx, ts, seqs = None, 0, [], [], [], []
            while True:
                item = self.q.get()
                if item is _STOP:
                    break
                (seq, t), arr, (t_mid, t_dev, t_host, sess) = item
                arr = np.asarray(arr)
                if buf is not None and arr.shape != buf.shape[1:]:
                    if n:
                        self._write_chunk(fi, buf, n, rows, idx, ts, seqs)
                    buf, n, rows, idx, ts, seqs = None, 0, [], [], [], []
                if buf is None:
                    buf = np.empty((self.chunk_frames, *arr.shape), dtype=np.uint16)
                    self.shapes.append(list(arr.shape))
                buf[n] = arr
                valid = float(np.count_nonzero(arr)) / arr.size if arr.size else 0.0
                rows.append(_csv_line((self.frames, seq, t, "@CHUNK@", n, t_mid, t_dev, t_host,
                                       f"{valid:.4f}", sess)))
                idx.append(self.frames)
                ts.append(t)
                seqs.append(-1 if seq is None else seq)
                n += 1
                self.frames += 1
                if n == self.chunk_frames:
                    self._write_chunk(fi, buf, n, rows, idx, ts, seqs)
                    n, rows, idx, ts, seqs = 0, [], [], [], []
            if n:
                self._write_chunk(fi, buf, n, rows, idx, ts, seqs)
            os.fsync(fi.fileno())


# ----- OAK-D (DepthAI v2) ---------------------------------------------------------------------------
def build_oak_pipeline(dai, s, lens_position=None, mono_resolution="THE_400_P"):
    """The DepthAI v2 pipeline: colour MJPEG on the device plus depth aligned to CAM_A.

    `s` needs color_fps, depth_fps, mjpeg_quality, stereo_preset, median, rgb_exposure_us and rgb_iso.
    Returns (pipeline, nodes).
    """
    p = dai.Pipeline()
    cam = p.create(dai.node.ColorCamera)
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setIspScale(2, 3)                       # 1920x1080 -> 1280x720, same field of view
    cam.setVideoSize(COLOR_W, COLOR_H)
    cam.setFps(float(s.color_fps))
    if lens_position is not None:
        cam.initialControl.setManualFocus(int(lens_position))
    if getattr(s, "rgb_exposure_us", None):
        cam.initialControl.setManualExposure(int(s.rgb_exposure_us), int(getattr(s, "rgb_iso", 400) or 400))

    enc = p.create(dai.node.VideoEncoder)
    enc.setDefaultProfilePreset(float(s.color_fps), dai.VideoEncoderProperties.Profile.MJPEG)
    enc.setQuality(int(s.mjpeg_quality))
    cam.video.link(enc.input)
    x_color = p.create(dai.node.XLinkOut)
    x_color.setStreamName("color")
    enc.bitstream.link(x_color.input)

    res = getattr(dai.MonoCameraProperties.SensorResolution, mono_resolution)
    left = p.create(dai.node.MonoCamera)
    left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    left.setResolution(res)
    left.setFps(float(s.depth_fps))
    right = p.create(dai.node.MonoCamera)
    right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    right.setResolution(res)
    right.setFps(float(s.depth_fps))

    stereo = p.create(dai.node.StereoDepth)
    # depthai 2.30+ deprecates HIGH_DENSITY/HIGH_ACCURACY in favour of DEFAULT. DEFAULT is not used here: it turns
    # on temporal and spatial filtering and decimation, which smear and thin a moving arm.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", DeprecationWarning)
        try:
            preset = getattr(dai.node.StereoDepth.PresetMode, s.stereo_preset)
        except AttributeError as e:
            raise RecordError(f"this depthai has no stereo preset {s.stereo_preset}") from e
        stereo.setDefaultProfilePreset(preset)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)
    stereo.setExtendedDisparity(False)
    if getattr(s, "median", "preset") not in (None, "preset"):
        stereo.setMedianFilter(getattr(dai.MedianFilter, MEDIAN[str(s.median)]))
    _no_postfilters(stereo)
    stereo.setRectifyEdgeFillColor(0)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(DEPTH_W, DEPTH_H)
    left.out.link(stereo.left)
    right.out.link(stereo.right)
    x_depth = p.create(dai.node.XLinkOut)
    x_depth.setStreamName("depth")
    stereo.depth.link(x_depth.input)
    return p, {"color": cam, "encoder": enc, "left": left, "right": right, "stereo": stereo}


def _no_postfilters(stereo):
    """Temporal, spatial and speckle filters and decimation off: a temporal filter smears a moving arm."""
    try:
        raw = stereo.initialConfig.get()
        pp = raw.postProcessing
        pp.temporalFilter.enable = False
        pp.spatialFilter.enable = False
        pp.speckleFilter.enable = False
        pp.decimationFilter.decimationFactor = 1
        stereo.initialConfig.set(raw)
    except Exception:  # noqa: BLE001 - older or fake depthai without these fields
        pass


def stereo_config(stereo):
    """What the stereo node will actually do, for meta.json."""
    out = {}
    c = getattr(stereo, "initialConfig", None)
    for key, fn in (("confidence_threshold", "getConfidenceThreshold"), ("median_filter", "getMedianFilter"),
                    ("left_right_check_threshold", "getLeftRightCheckThreshold"),
                    ("bilateral_sigma", "getBilateralFilterSigma"), ("max_disparity", "getMaxDisparity"),
                    ("depth_unit", "getDepthUnit")):
        try:
            out[key] = _jsonable(getattr(c, fn)())
        except Exception:  # noqa: BLE001
            pass
    try:
        raw = c.get()
        ac, pp = raw.algorithmControl, raw.postProcessing
        out.update(left_right_check=bool(ac.enableLeftRightCheck), subpixel=bool(ac.enableSubpixel),
                   subpixel_fractional_bits=int(ac.subpixelFractionalBits),
                   extended_disparity=bool(ac.enableExtended),
                   temporal_filter=bool(pp.temporalFilter.enable), spatial_filter=bool(pp.spatialFilter.enable),
                   speckle_filter=bool(pp.speckleFilter.enable),
                   decimation=int(pp.decimationFilter.decimationFactor))
    except Exception:  # noqa: BLE001
        pass
    return out


def _call(obj, name, *args, default=None):
    try:
        return getattr(obj, name)(*args)
    except Exception:  # noqa: BLE001
        return default


def calibration_info(dai, calib, mono_size=(640, 400)):
    """Calibration and intrinsics for meta.json. Every value is as DepthAI reports it."""
    A, B, C = dai.CameraBoardSocket.CAM_A, dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C
    eeprom = _call(calib, "eepromToJson")
    if isinstance(eeprom, str):
        try:
            eeprom = json.loads(eeprom)
        except ValueError:
            pass
    info = {
        "rgb_1280x720": _call(calib, "getCameraIntrinsics", A, COLOR_W, COLOR_H),
        "rgb_640x360": _call(calib, "getCameraIntrinsics", A, DEPTH_W, DEPTH_H),
        "rgb_distortion": _call(calib, "getDistortionCoefficients", A),
        "rgb_lens_position": _call(calib, "getLensPosition", A),
        "rgb_fov_deg": _call(calib, "getFov", A),
        "left_mono": _call(calib, "getCameraIntrinsics", B, *mono_size),
        "right_mono": _call(calib, "getCameraIntrinsics", C, *mono_size),
        "mono_size": list(mono_size),
        "baseline_cm": _call(calib, "getBaselineDistance"),
    }
    default = _call(calib, "getDefaultIntrinsics", A)
    if default:
        K, w, h = default
        info["rgb_calibrated_at"] = {"K": K, "width": int(w), "height": int(h)}
        if (int(w), int(h)) == (4056, 3040):
            # IMX378's 1080P mode sees the centred 3840x2160 of the 4056x3040 array (a 4K crop, binned).
            # getCameraIntrinsics(..., 1280, 720) assumes a full-width crop instead, about 5 % off in
            # focal length. Unverified on bob: `be record check` compares both against the board views.
            def crop(scale):
                return [[K[0][0] * scale, 0.0, (K[0][2] - 108.0) * scale],
                        [0.0, K[1][1] * scale, (K[1][2] - 440.0) * scale], [0.0, 0.0, 1.0]]
            info["rgb_1280x720_crop_aware"] = crop(COLOR_W / 3840.0)
            info["rgb_640x360_crop_aware"] = crop(DEPTH_W / 3840.0)
    info["notes"] = ("K matrices are 3x3, pixels. rgb_640x360 applies to the aligned depth. Distortion is "
                     "DepthAI's 14-coefficient OpenCV model for CAM_A. If rgb_1280x720_crop_aware is present, "
                     "the calibration was stored at 4056x3040 and the two candidates differ; check decides.")
    return {"intrinsics": info, "eeprom": eeprom}


class OakHandle:
    def __init__(self, device, q_color, q_depth, info):
        self.device, self.q_color, self.q_depth, self.info = device, q_color, q_depth, info

    def close(self, timeout=8.0):
        dev = self.device
        if dev is None:
            return
        self.device = None
        t = threading.Thread(target=lambda: _call(dev, "close"), daemon=True)
        t.start()
        t.join(timeout)


def _open_device(dai, which=None):
    if not which:
        return dai.Device()
    try:
        return dai.Device(which)
    except Exception:  # noqa: BLE001 - older v2 wants a DeviceInfo
        return dai.Device(dai.DeviceInfo(which))


def open_oak(dai, s):
    """Boot the OAK-D, read its calibration, build and start the pipeline. Returns an OakHandle."""
    try:
        dev = _open_device(dai, getattr(s, "oak_device", None))
    except Exception as e:  # noqa: BLE001
        raise RecordError(f"cannot open the OAK-D: {e}\n  Is it plugged in (blue USB3 port), powered, and not "
                          "held by another program? Run `record.py devices` to see.") from e
    try:
        info = {"depthai_version": getattr(dai, "__version__", "?"),
                "mxid": _call(dev, "getMxId"), "device_name": _call(dev, "getDeviceName"),
                "product_name": _call(dev, "getProductName"),
                "usb_speed": _name(_call(dev, "getUsbSpeed", default="UNKNOWN"))}
        sensors = _call(dev, "getCameraSensorNames", default={}) or {}
        info["sensors"] = {_name(k): str(v) for k, v in dict(sensors).items()}
        mono = getattr(s, "mono_resolution", None)
        if not mono:
            mono = "THE_480_P" if info["sensors"].get("CAM_B", "").upper().startswith("OV7251") else "THE_400_P"
        mono_size = (640, 480) if mono == "THE_480_P" else (640, 400)
        try:
            calib = dev.readCalibration()
            cal = calibration_info(dai, calib, mono_size)
        except Exception as e:  # noqa: BLE001
            calib, cal = None, {"intrinsics": {}, "eeprom": None, "error": f"{type(e).__name__}: {e}"}
        lens = getattr(s, "rgb_focus", None)
        cal_lens = cal["intrinsics"].get("rgb_lens_position")
        focus = "manual (--rgb-focus)"
        if lens is None and cal_lens:
            lens, focus = int(cal_lens), "manual, at the calibration's lens position"
        elif lens is None:
            focus = "camera default (no calibration lens position found)"
        pipeline, nodes = build_oak_pipeline(dai, s, lens, mono)
        info.update(
            color={"socket": "CAM_A", "sensor_resolution": "THE_1080_P", "isp_scale": [2, 3],
                   "size": [COLOR_W, COLOR_H], "fps": float(s.color_fps), "encoding": "MJPEG (on device)",
                   "mjpeg_quality": int(s.mjpeg_quality), "lens_position": lens, "focus": focus,
                   "exposure": (f"manual {s.rgb_exposure_us} us ISO {getattr(s, 'rgb_iso', 400)}"
                                if getattr(s, "rgb_exposure_us", None) else "auto")},
            depth={"left": "CAM_B", "right": "CAM_C", "mono_resolution": mono, "mono_size": list(mono_size),
                   "fps": float(s.depth_fps), "preset": s.stereo_preset, "align_to": "CAM_A",
                   "output_size": [DEPTH_W, DEPTH_H], "units": "mm", "dtype": "uint16", "invalid_value": 0,
                   "stereo": stereo_config(nodes["stereo"])},
        )
        try:
            started = dev.startPipeline(pipeline)
            if started is False and _call(dev, "isPipelineRunning") is False:
                raise RuntimeError("startPipeline returned False")
        except Exception as e:  # noqa: BLE001 - fall back to booting the device with the pipeline
            info["start_fallback"] = f"startPipeline on the open device failed ({e}); reopened with the pipeline"
            _call(dev, "close")
            dev = None
            deadline = time.monotonic() + 15.0
            while dev is None:
                try:
                    dev = dai.Device(pipeline)
                except Exception:  # noqa: BLE001
                    if time.monotonic() > deadline:
                        raise
                    time.sleep(1.0)
        q_color = dev.getOutputQueue("color", maxSize=30, blocking=False)
        q_depth = dev.getOutputQueue("depth", maxSize=30, blocking=False)
    except BaseException:
        if dev is not None:
            _call(dev, "close")
        raise
    info["calibration"] = cal
    return OakHandle(dev, q_color, q_depth, info)


def _td(fn, *args):
    try:
        return fn(*args).total_seconds()
    except Exception:  # noqa: BLE001
        return None


class OakSource:
    """Reads both OAK-D queues on one thread and hands frames to the writers; reconnects on failure."""

    def __init__(self, dai, settings, color_writer, depth_writer, cstats, dstats, console, halt):
        self.dai, self.s = dai, settings
        self.cw, self.dw, self.cstats, self.dstats = color_writer, depth_writer, cstats, dstats
        self.console, self.halt = console, halt
        self.stop = threading.Event()
        self.handle = None
        self.session = 0
        self.sessions = []
        self.state = "starting"
        self.thread = None
        self._mid = getattr(getattr(dai, "CameraExposureOffset", None), "MIDDLE", None)

    def open(self, session=0):
        self.handle = open_oak(self.dai, self.s)
        self.session = session
        self.sessions.append({"oak_session": session, "opened_t_mono_s": time.monotonic(), "error": None})
        self.state = "ok"
        return self.handle.info

    def start(self):
        self.thread = threading.Thread(target=self._run, name="oak", daemon=True)
        self.thread.start()

    def _mid_ts(self, m):
        if self._mid is None:
            return None
        try:
            return m.getTimestamp(self._mid).total_seconds()
        except Exception:  # noqa: BLE001
            return None

    def _on_color(self, m):
        t_host = time.monotonic()
        seq, t = int(m.getSequenceNum()), m.getTimestamp().total_seconds()
        data = m.getData()
        payload = data.tobytes() if hasattr(data, "tobytes") else bytes(data)
        self.cstats.on_frame(t, seq)
        self.cw.put(((seq, t), payload, (self._mid_ts(m), _td(m.getTimestampDevice), t_host, self.session)))

    def _on_depth(self, m):
        t_host = time.monotonic()
        seq, t = int(m.getSequenceNum()), m.getTimestamp().total_seconds()
        arr = np.array(m.getFrame(), dtype=np.uint16, copy=True)
        self.dstats.on_frame(t, seq)
        self.dw.put(((seq, t), arr, (self._mid_ts(m), _td(m.getTimestampDevice), t_host, self.session)))

    def _read(self, h):
        last_frame = last_check = time.monotonic()
        while not self.stop.is_set():
            got = 0
            for m in h.q_color.tryGetAll() or ():
                self._on_color(m)
                got += 1
            for m in h.q_depth.tryGetAll() or ():
                self._on_depth(m)
                got += 1
            now = time.monotonic()
            if got:
                last_frame = now
            elif now - last_frame > OAK_STALL_S:
                raise RuntimeError(f"no frames from the OAK-D for {OAK_STALL_S:g} s")
            else:
                time.sleep(0.002)
            if now - last_check > 0.5:
                last_check = now
                if _call(h.device, "isClosed", default=False):
                    raise RuntimeError("the OAK-D connection closed")

    def _run(self):
        while not self.stop.is_set():
            try:
                self._read(self.handle)
                break
            except Exception as e:  # noqa: BLE001
                msg = f"{type(e).__name__}: {e}"
                self.sessions[-1]["error"] = msg
                self.state = "RECONNECTING"
                self.console.say(f"!!! OAK-D lost ({msg}); reconnecting every {OAK_RETRY_S:g} s")
                self.handle.close()
                while not self.stop.wait(OAK_RETRY_S):
                    try:
                        self.open(self.session + 1)
                        self.cstats.last_seq = self.dstats.last_seq = None
                        self.console.say(f"!!! OAK-D reconnected (oak_session {self.session})")
                        break
                    except Exception as e2:  # noqa: BLE001
                        self.console.say(f"!!! OAK-D reconnect failed: {e2}")
                else:
                    break
        if self.handle is not None:
            self.handle.close()
        self.state = "stopped"

    def finish(self, timeout=10.0):
        self.stop.set()
        if self.thread is not None:
            self.thread.join(timeout)
        elif self.handle is not None:       # opened, never started
            self.handle.close()


# ----- wrist camera (V4L2 through OpenCV) ---------------------------------------------------------
class WristSource:
    """Grabs the wrist camera on its own thread. Keeps the camera's MJPEG bytes when OpenCV can pass them
    through (CAP_PROP_CONVERT_RGB = 0 on the V4L2 backend gives the raw buffer), else hands BGR frames
    to the writer to encode."""

    def __init__(self, cv2, device, writer, stats, console, *, size=(640, 480), fps=30.0, fourcc="MJPG",
                 autofocus="off", capture_factory=None):
        self.cv2, self.device, self.writer, self.stats, self.console = cv2, device, writer, stats, console
        self.size, self.fps, self.fourcc, self.autofocus = size, float(fps), fourcc, autofocus
        self.factory = capture_factory or (lambda dev: cv2.VideoCapture(dev, cv2.CAP_V4L2))
        self.cap = None
        self.raw = False
        self.session = 0
        self.sessions = []
        self.state = "starting"
        self.stop = threading.Event()
        self.thread = None
        self.info = {}

    def _target(self):
        d = str(self.device)
        return int(d) if d.isdigit() else d

    def open(self, session=0):
        cv2 = self.cv2
        cap = self.factory(self._target())
        if cap is None or not cap.isOpened():
            raise RecordError(f"cannot open the wrist camera {self.device} (busy, unplugged, or not a camera?). "
                              "Run `record.py devices`.")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        af = None
        if self.autofocus in ("off", "on"):
            af = bool(cap.set(cv2.CAP_PROP_AUTOFOCUS, 1 if self.autofocus == "on" else 0))
        actual = {"fourcc": _fourcc_str(cap.get(cv2.CAP_PROP_FOURCC)),
                  "width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                  "fps": float(cap.get(cv2.CAP_PROP_FPS))}
        raw = False
        if actual["fourcc"] == "MJPG" and cap.set(cv2.CAP_PROP_CONVERT_RGB, 0):
            frame = None
            for _ in range(10):
                ok, frame = cap.read()
                if ok and frame is not None:
                    break
            buf = np.asarray(frame).reshape(-1) if frame is not None else None
            if buf is not None and frame.ndim <= 2 and min(frame.shape) == 1 and buf.dtype == np.uint8 \
                    and buf.size > 4 and buf[0] == 0xFF and buf[1] == 0xD8:
                raw = True
            else:
                cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
        if not raw:
            ok, frame = False, None
            for _ in range(10):
                ok, frame = cap.read()
                if ok and frame is not None:
                    break
            if not ok or frame is None:
                cap.release()
                raise RecordError(f"the wrist camera {self.device} opened but gave no frames")
        self.cap, self.raw = cap, raw
        self.info = {"device": str(self.device), "resolved": os.path.realpath(str(self.device))
                     if not str(self.device).isdigit() else str(self.device),
                     "requested": {"fourcc": self.fourcc, "width": self.size[0], "height": self.size[1],
                                   "fps": self.fps},
                     "actual": actual, "raw_mjpeg_passthrough": raw,
                     "autofocus_request": self.autofocus, "autofocus_set_ok": af,
                     "focus": _call(cap, "get", cv2.CAP_PROP_FOCUS),
                     "stored": "camera's own MJPEG bytes" if raw else "decoded by OpenCV, re-encoded as JPEG q90"}
        self.session = session
        self.sessions.append({"wrist_session": session, "opened_t_mono_s": time.monotonic(), "error": None})
        self.state = "ok"
        return self.info

    def start(self):
        self.thread = threading.Thread(target=self._run, name="wrist", daemon=True)
        self.thread.start()

    def _grab_loop(self):
        cv2, cap, fails = self.cv2, self.cap, 0
        while not self.stop.is_set():
            ok = cap.grab()
            t = time.monotonic()
            if not ok:
                fails += 1
                if fails >= 30:
                    raise RuntimeError("the wrist camera stopped delivering frames")
                time.sleep(0.01)
                continue
            ok, frame = cap.retrieve()
            if not ok or frame is None or (self.raw and np.asarray(frame).size < 64):
                fails += 1                  # empty or truncated buffer from the driver: skip it
                continue
            fails = 0
            ms = cap.get(cv2.CAP_PROP_POS_MSEC)
            t_v4l2 = ms / 1000.0 if ms and ms > 0 else None
            if self.raw:
                payload = np.asarray(frame).reshape(-1).tobytes()
            else:
                payload = frame
            self.stats.on_frame(t)
            self.writer.put(((t,), payload, (t_v4l2, 1 if self.raw else 0, self.session)))

    def _run(self):
        while not self.stop.is_set():
            try:
                self._grab_loop()
                break
            except Exception as e:  # noqa: BLE001
                msg = f"{type(e).__name__}: {e}"
                self.sessions[-1]["error"] = msg
                self.state = "RECONNECTING"
                self.console.say(f"!!! wrist camera lost ({msg}); reopening every {WRIST_RETRY_S:g} s")
                _call(self.cap, "release")
                while not self.stop.wait(WRIST_RETRY_S):
                    try:
                        self.open(self.session + 1)
                        self.console.say(f"!!! wrist camera back (wrist_session {self.session})")
                        break
                    except Exception as e2:  # noqa: BLE001
                        self.console.say(f"!!! wrist camera reopen failed: {e2}")
                else:
                    break
        _call(self.cap, "release")
        self.state = "stopped"

    def finish(self, timeout=5.0):
        self.stop.set()
        if self.thread is not None:
            self.thread.join(timeout)
        else:                               # opened, never started
            _call(self.cap, "release")


# ----- console, events, clock -----------------------------------------------------------------------
class Console:
    """Prints to the terminal and appends to recorder.log. Printing never raises: if the SSH session
    drops, recording continues."""

    def __init__(self, stream=None):
        self.stream = stream if stream is not None else sys.stdout
        self.log = None
        self.t0 = time.monotonic()
        self._lock = threading.Lock()

    def attach(self, path):
        self.log = open(path, "a", encoding="utf-8", buffering=1)  # noqa: SIM115

    def say(self, msg, *, log=True):
        with self._lock:
            try:
                self.stream.write(msg + "\n")
                self.stream.flush()
            except (OSError, ValueError):
                pass
            if log and self.log is not None:
                try:
                    self.log.write(f"{_iso()} +{time.monotonic() - self.t0:9.3f}s {msg}\n")
                except (OSError, ValueError):
                    pass

    def close(self):
        if self.log is not None:
            try:
                self.log.close()
            except OSError:
                pass
            self.log = None


class EventLog:
    def __init__(self, path, console, t0):
        self.console, self.t0 = console, t0
        self.n = 0
        self._lock = threading.Lock()
        new = not Path(path).exists()
        self.f = open(path, "a", newline="", encoding="utf-8")  # noqa: SIM115
        if new:
            self.f.write(",".join(EVENT_COLUMNS) + "\n")
            self.f.flush()

    def mark(self, label, source, t_mono=None):
        t = time.monotonic() if t_mono is None else t_mono
        wall = time.time() - (time.monotonic() - t)
        label = clean_label(label)
        with self._lock:
            n = self.n
            self.n += 1
            self.f.write(_csv_line((n, t, wall, label, source)))
            self.f.flush()
        self.console.say(f">>> EVENT {n}: {label}   (+{t - self.t0:.1f} s, from {source})")
        return label

    def close(self):
        with self._lock:
            self.f.close()


def _stdin_events(stream, events, halt):
    while not halt.is_set():
        try:
            line = stream.readline()
        except (OSError, ValueError):
            return
        if line == "":
            return
        t = time.monotonic()
        events.mark(line, "keyboard", t)


class FifoEvents:
    """Reads event labels, one per line, from a named pipe (`record.py mark LABEL --fifo PATH`)."""

    def __init__(self, path, events, halt):
        self.path, self.events, self.halt = Path(path).expanduser(), events, halt
        self.fd, self.created, self.thread = None, False, None

    def open(self):
        if self.path.exists():
            if not stat.S_ISFIFO(self.path.stat().st_mode):
                raise RecordError(f"{self.path} exists and is not a FIFO")
        else:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            os.mkfifo(self.path, 0o600)
            self.created = True
        # O_RDWR: we hold a write end too, so the pipe never reads as EOF between writers (Linux).
        self.fd = os.open(self.path, os.O_RDWR | os.O_NONBLOCK)
        self.thread = threading.Thread(target=self._run, name="fifo-events", daemon=True)
        self.thread.start()

    def _run(self):
        buf = b""
        while not self.halt.is_set() and self.fd is not None:
            try:
                r, _, _ = select.select([self.fd], [], [], 0.2)
            except (OSError, ValueError):
                return
            if not r:
                continue
            t = time.monotonic()
            try:
                buf += os.read(self.fd, 4096)
            except BlockingIOError:
                continue
            except OSError:
                return
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if line.strip():
                    self.events.mark(line.decode("utf-8", "replace"), "fifo", t)

    def close(self):
        if self.thread is not None:
            self.thread.join(1.0)
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None
        if self.created:
            try:
                self.path.unlink()
            except OSError:
                pass


class ClockLog:
    def __init__(self, path):
        new = not Path(path).exists()
        self.f = open(path, "a", newline="", encoding="utf-8")  # noqa: SIM115
        if new:
            self.f.write(",".join(CLOCK_COLUMNS) + "\n")
        self.n = 0

    def pair(self, kind):
        w, m, u = clock_pair()
        self.f.write(f"{kind},{w:.6f},{m:.6f},{u:.9f}\n")
        self.f.flush()
        self.n += 1
        return w, m, u

    def close(self):
        self.f.close()


MEASUREMENTS_TEMPLATE = """\
# Measurements for this recording. Millimetres, measured with a ruler or tape measure.
# Robot base frame: origin on the table directly under the shoulder_pan axis (as bob_envelope.json);
# +x forward, +y to the robot's left, +z up.
# The printed board labels corner O (its origin), X (end of the +x edge) and Y (end of the +y edge).

printed_100mm_bar_horizontal_mm =        # the horizontal scale bar, measured
printed_100mm_bar_vertical_mm =          # the vertical scale bar, measured
board_width_mm =                         # outer edge to outer edge along the 5-square side (nominal 150.0)
board_height_mm =                        # same along the 7-square side (nominal 210.0)

corner_O_x_mm =                          # how far forward (+x) of the base centre corner O is
corner_O_y_mm =                          # how far to the left (+y); negative if to the right
corner_X_x_mm =                          # the same for corner X (end of the +x arrow edge, 150 mm from O)
corner_X_y_mm =
corner_Y_x_mm =                          # the same for corner Y (end of the +y arrow edge, 210 mm from O)
corner_Y_y_mm =
# self-check: from these numbers O-X should come out at 150 mm and O-Y at 210 mm (within ~3 mm)
board_surface_above_table_mm =           # backing + paper thickness (0 if taped straight to the table)
board_face =                             # up (printed side up) -- it should be up

oakd_distance_to_base_centre_mm =        # rough, tape measure, to the front of the OAK-D
oakd_height_above_table_mm =             # rough
oakd_direction =                         # e.g. "front-left of the arm, looking back at it, tilted down ~25 deg"
wrist_camera =                           # model if known; where it sits on the gripper; which way it looks
lighting =                               # e.g. ceiling LEDs + desk lamp, blinds closed
notes =
"""


# ----- the recorder -------------------------------------------------------------------------------
class Recorder:
    def __init__(self, args, *, dai=None, cv2=None, capture_factory=None, stdin=None, console_stream=None,
                 install_signals=True):
        self.a = args
        self.dai, self.cv2, self.capture_factory = dai, cv2, capture_factory
        self.stdin = stdin
        self.console = Console(console_stream)
        self.install_signals = install_signals
        self.halt = threading.Event()
        self.stop_reason = None
        self._stopping = False
        self._sigints = 0
        self._old_handlers = {}
        self.oak = self.wrist = None
        self.writers = {}
        self.stats = {}
        self.fifo = None
        self.events = self.clock = None
        self.out = None
        self.meta = {}

    # signals ----------------------------------------------------------------------------------
    def _request_stop(self, reason):
        if self.stop_reason is None:
            self.stop_reason = reason
        self.halt.set()

    def _on_sigint(self, signum, frame):  # noqa: ARG002
        if self._stopping:
            self._sigints += 1
            if self._sigints >= 2:
                os._exit(130)
            self.console.say("Still closing the files; Ctrl+C once more quits without finishing them.")
            return
        self._request_stop("stopped by operator (Ctrl+C)")

    def _on_sigterm(self, signum, frame):  # noqa: ARG002
        self._request_stop("stopped by SIGTERM")

    def _on_sighup(self, signum, frame):  # noqa: ARG002
        self.console.say("Terminal hung up; still recording. Stop it with: record.py stop " + str(self.out))

    def _signals(self, install):
        if not self.install_signals or threading.current_thread() is not threading.main_thread():
            return
        if install:
            for sig, fn in ((signal.SIGINT, self._on_sigint), (signal.SIGTERM, self._on_sigterm),
                            (getattr(signal, "SIGHUP", None), self._on_sighup)):
                if sig is not None:
                    self._old_handlers[sig] = signal.signal(sig, fn)
        else:
            for sig, old in self._old_handlers.items():
                signal.signal(sig, old)
            self._old_handlers.clear()

    # setup ------------------------------------------------------------------------------------
    def _check_out(self):
        out = Path(self.a.out).expanduser()
        if out.exists() and (not out.is_dir() or any(out.iterdir())):
            raise RecordError(f"{out} already exists and is not empty; choose a new --out")
        return out

    def _plan(self):
        a = self.a
        rates = estimate_rates(oak=not a.no_oak, color_fps=a.color_fps, mjpeg_quality=a.mjpeg_quality,
                               depth_fps=a.depth_fps, wrist=a.wrist != "none", wrist_size=a.wrist_size,
                               wrist_fps=a.wrist_fps)
        seconds = float(a.seconds) if a.seconds else DEFAULT_PLAN_SECONDS
        need = disk_need(rates, seconds)
        free = free_bytes(a.out)
        plan = {"mb_per_s": rates, "seconds": seconds, "seconds_from": "--seconds" if a.seconds else "default plan",
                "estimated_gb": round(seconds * rates["total"] / 1e3, 2), "need_gb_with_margin": round(_gb(need), 2),
                "margin_factor": DISK_FACTOR, "free_gb": round(_gb(free), 2)}
        line = (f"Estimated data rate: colour {rates['color']:.1f} + depth {rates['depth']:.1f} + wrist "
                f"{rates['wrist']:.1f} = {rates['total']:.1f} MB/s ({rates['total'] * 60 / 1e3:.2f} GB/min). "
                f"For {seconds:.0f} s ({plan['seconds_from']}): {plan['estimated_gb']:.1f} GB; need "
                f"{plan['need_gb_with_margin']:.1f} GB free (x{DISK_FACTOR}); free {plan['free_gb']:.1f} GB.")
        self.console.say(line, log=False)
        if free < need:
            raise RecordError(
                "Not enough free disk for this recording.\n  " + line + "\n  Options: --depth-fps 15 (halves "
                "the depth data), a shorter --seconds, --mjpeg-quality 80, or --out on a bigger disk.")
        return plan

    def _open_sources(self):
        a = self.a
        if not a.no_oak:
            dai = self.dai or import_depthai()
            self.dai = dai
            self.stats["color"] = StreamStats("color", a.color_fps)
            self.stats["depth"] = StreamStats("depth", a.depth_fps)
            self.oak = OakSource(dai, a, None, None, self.stats["color"], self.stats["depth"], self.console,
                                 self.halt)
            info = self.oak.open()
            self.console.say(
                f"OAK-D {info.get('device_name') or ''} {info.get('mxid')}: USB {info.get('usb_speed')}, "
                f"depthai {info.get('depthai_version')}, sensors {info.get('sensors')}, colour focus: "
                f"{info['color']['focus']} ({info['color']['lens_position']}), depth {info['depth']['mono_resolution']}"
                f" {info['depth']['preset']}")
            if info.get("usb_speed") not in ("SUPER", "SUPER_PLUS"):
                self.console.say(f"!!! the OAK-D is on USB {info.get('usb_speed')}, not USB3: use a blue port "
                                 "and the USB3 cable if frames drop")
        if a.wrist != "none":
            try:
                self._open_wrist()
            except BaseException:
                if self.oak is not None and self.oak.handle is not None:
                    self.oak.handle.close()
                raise
        if self.oak is None and self.wrist is None:
            raise RecordError("nothing to record: --no-oak and --wrist none")

    def _open_wrist(self):
        a = self.a
        cv2 = self.cv2 or import_cv2()
        self.cv2 = cv2
        device = a.wrist
        if device == "auto":
            device, found = find_wrist_camera()
            if device is None:
                raise RecordError("no USB camera found under /dev/v4l/by-id for the wrist camera. Plug it "
                                  "in, pass --wrist /dev/videoN, or --wrist none to record without it.")
            if len(found) > 1:
                self.console.say(f"!!! several USB cameras: {found}; using {device} (pass --wrist to choose)")
        self.stats["wrist"] = StreamStats("wrist", a.wrist_fps)
        self.wrist = WristSource(cv2, device, None, self.stats["wrist"], self.console, size=a.wrist_size,
                                 fps=a.wrist_fps, fourcc=a.wrist_fourcc, autofocus=a.wrist_autofocus,
                                 capture_factory=self.capture_factory)
        info = self.wrist.open()
        act = info["actual"]
        self.console.say(f"Wrist camera {device}: {act['fourcc']} {act['width']}x{act['height']} @ "
                         f"{act['fps']:g} fps, {info['stored']}")

    def _make_writers(self):
        out, a = self.out, self.a
        if self.oak is not None:
            (out / "oakd").mkdir(parents=True, exist_ok=True)
            self.writers["color"] = MjpegWriter(out / PATHS["color_stream"], out / PATHS["color_index"],
                                                COLOR_COLUMNS, self.stats["color"], max_queue=120)
            self.writers["depth"] = DepthChunkWriter(out / PATHS["depth_dir"], out / PATHS["depth_index"],
                                                     self.stats["depth"], chunk_frames=a.chunk_frames,
                                                     compresslevel=a.depth_compress_level)
            self.oak.cw, self.oak.dw = self.writers["color"], self.writers["depth"]
        if self.wrist is not None:
            (out / "wrist").mkdir(parents=True, exist_ok=True)
            self.writers["wrist"] = MjpegWriter(out / PATHS["wrist_stream"], out / PATHS["wrist_index"],
                                                WRIST_COLUMNS, self.stats["wrist"], max_queue=120,
                                                jpeg_quality=a.wrist_jpeg_quality)
            self.wrist.writer = self.writers["wrist"]
        for w in self.writers.values():
            w.start()

    def _initial_meta(self, plan):
        a = self.a
        settings = {k: (list(v) if isinstance(v, tuple) else v) for k, v in vars(a).items()
                    if k not in ("func", "record_command") and not callable(v)}
        files = {"events": {"path": PATHS["events"], "columns": list(EVENT_COLUMNS)},
                 "clock": {"path": PATHS["clock"], "columns": list(CLOCK_COLUMNS)},
                 "log": PATHS["log"], "measurements": PATHS["measurements"]}
        if self.oak is not None:
            files["color"] = {"stream": PATHS["color_stream"], "index": PATHS["color_index"],
                              "columns": list(COLOR_COLUMNS),
                              "about": "concatenated JPEGs; a frame is byte_len bytes at byte_offset"}
            files["depth"] = {"dir": PATHS["depth_dir"], "index": PATHS["depth_index"], "columns": list(DEPTH_COLUMNS),
                              "about": "chunk is a file in dir; np.load(chunk)['depth'][index_in_chunk] is the "
                                       "frame, uint16 mm, 0 = no depth; chunks also hold frame_idx, t_mono_s and "
                                       "device_seq arrays"}
        if self.wrist is not None:
            files["wrist"] = {"stream": PATHS["wrist_stream"], "index": PATHS["wrist_index"],
                              "columns": list(WRIST_COLUMNS),
                              "about": "concatenated JPEGs; raw=1 when they are the camera's own MJPEG bytes"}
        oak = None
        if self.oak is not None:
            info = dict(self.oak.handle.info)
            cal = info.pop("calibration", {})
            oak = {**info, "intrinsics": cal.get("intrinsics"), "calibration_eeprom": cal.get("eeprom"),
                   "calibration_error": cal.get("error"),
                   "timestamps": {
                       "t_mono_s": "ImgFrame.getTimestamp(): device time synced to the host steady clock "
                                   "(CLOCK_MONOTONIC on Linux = Python time.monotonic())",
                       "t_mid_s": "getTimestamp(CameraExposureOffset.MIDDLE), when available",
                       "t_dev_s": "getTimestampDevice(): the device's own clock",
                       "t_host_s": "time.monotonic() when this program dequeued the frame"}}
        wrist = None
        if self.wrist is not None:
            wrist = {**self.wrist.info, "timestamps": {
                "t_mono_s": "time.monotonic() when VideoCapture.grab() returned",
                "t_v4l2_s": "CAP_PROP_POS_MSEC / 1000: the V4L2 buffer timestamp (CLOCK_MONOTONIC for uvcvideo)"}}
        mono = time.get_clock_info("monotonic")
        return {
            "format": FORMAT,
            "status": "recording",
            "recorder": {"name": "bounded_embodiment_kit.instruments.record", "version": RECORDER_VERSION,
                         "argv": sys.argv, "python": platform.python_version(), "platform": platform.platform(),
                         "machine": platform.machine(), "pid": os.getpid(), "numpy": np.__version__,
                         "opencv": getattr(self.cv2, "__version__", None)},
            "started": _iso(), "t0_mono_s": self.t0, "t0_wall_s": self.t0_wall,
            "clock": {"monotonic_implementation": mono.implementation, "monotonic_resolution_s": mono.resolution,
                      "about": "all t_*_s columns except t_wall_s and t_dev_s are CLOCK_MONOTONIC seconds; "
                               "clock.csv pairs map time.time() onto them"},
            "settings": settings,
            "disk_plan": plan,
            "oakd": oak,
            "wrist": wrist,
            "files": files,
            "board": BOARD_SPEC,
        }

    def _status(self):
        parts = [f"+{time.monotonic() - self.t0:7.1f}s"]
        for name, st in self.stats.items():
            fps = st.window_fps()
            w = self.writers.get(name)
            q = f" q{w.q.qsize()}" if w is not None and w.q.qsize() > w.q.maxsize // 4 else ""
            extra = f" miss {st.seq_gaps}" if name in ("color", "depth") else ""
            parts.append(f"{name} {fps:4.1f}fps drop {st.dropped}{extra}{q}")
        total = sum(st.bytes for st in self.stats.values())
        el = max(time.monotonic() - self.t0, 1e-6)
        rate = total / 1e6 / el
        free = free_bytes(self.out)
        mins = (free - self.a.min_free_gb * 1e9) / (rate * 1e6) / 60 if rate > 0.01 else float("inf")
        parts.append(f"{total / 1e6:,.0f} MB ({rate:.1f} MB/s)")
        parts.append(f"free {_gb(free):.1f} GB (~{mins:.0f} min)" if math.isfinite(mins) else
                     f"free {_gb(free):.1f} GB")
        parts.append(f"ev {self.events.n}")
        for src, label in ((self.oak, "OAK-D"), (self.wrist, "WRIST")):
            if src is not None and src.state not in ("ok", "starting"):
                parts.append(f"{label} {src.state}")
        return " | ".join(parts)

    def _wait_first_frames(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not self.halt.is_set():
            missing = [n for n, st in self.stats.items() if st.received == 0]
            if not missing:
                return
            time.sleep(0.05)
        missing = [n for n, st in self.stats.items() if st.received == 0]
        if missing and not self.halt.is_set():
            raise RecordError(f"no frames from {', '.join(missing)} within {timeout:.0f} s of starting")

    def run(self):
        a = self.a
        self.out = self._check_out()
        plan = self._plan()
        self._open_sources()
        failed = None
        try:
            self.out.mkdir(parents=True, exist_ok=True)
            self.console.attach(self.out / PATHS["log"])
            self.t0_wall, self.t0 = time.time(), time.monotonic()
            self.console.t0 = self.t0
            self.clock = ClockLog(self.out / PATHS["clock"])
            self.clock.pair("start")
            self.events = EventLog(self.out / PATHS["events"], self.console, self.t0)
            if not (self.out / PATHS["measurements"]).exists():
                (self.out / PATHS["measurements"]).write_text(MEASUREMENTS_TEMPLATE, encoding="utf-8")
            self._make_writers()
            self.meta = self._initial_meta(plan)
            write_json_atomic(self.out / PATHS["meta"], self.meta)
            (self.out / PATHS["pid"]).write_text(f"{os.getpid()}\n")
            self._signals(True)
            if self.oak is not None:
                self.oak.start()
            if self.wrist is not None:
                self.wrist.start()
            if a.events_fifo:
                self.fifo = FifoEvents(a.events_fifo, self.events, self.halt)
                self.fifo.open()
            if self.stdin is not None:
                threading.Thread(target=_stdin_events, args=(self.stdin, self.events, self.halt),
                                 name="stdin-events", daemon=True).start()
            self._wait_first_frames(a.first_frame_timeout)
            self.console.say(f"Recording to {self.out}")
            how = []
            if self.stdin is not None:
                how.append("type a label and Enter (Enter alone logs 'mark')")
            if self.fifo is not None:
                how.append(f"`record.py mark LABEL --fifo {a.events_fifo}`")
            self.console.say("Events: " + (" or ".join(how) if how else "none") + ". Stop: Ctrl+C" +
                             (f" or after {a.seconds:g} s." if a.seconds else "."))
            self._loop()
        except RecordError as e:
            failed = str(e)
            self._request_stop(f"failed: {e}")
        except Exception as e:  # noqa: BLE001
            failed = f"{type(e).__name__}: {e}"
            self._request_stop(f"failed: {failed}")
        finally:
            self._stopping = True
            self._finish()
            self._signals(False)
        if failed:
            raise RecordError(failed)
        return 0

    def _loop(self):
        a = self.a
        next_status = self.t0 + a.status_every
        next_clock = self.t0 + 5.0
        next_disk = self.t0 + 5.0
        while not self.halt.wait(0.1):
            now = time.monotonic()
            if a.seconds and now - self.t0 >= a.seconds:
                self._request_stop(f"time limit reached ({a.seconds:g} s)")
                break
            for name, w in self.writers.items():
                if w.error:
                    self._request_stop(f"writing {name} failed: {w.error}")
            if now >= next_disk:
                next_disk = now + 5.0
                if free_bytes(self.out) < a.min_free_gb * 1e9:
                    self._request_stop(f"less than {a.min_free_gb:g} GB of disk left")
                    break
            if now >= next_clock:
                self.clock.pair("periodic")
                next_clock += 5.0
            if now >= next_status:
                self.console.say(self._status())
                next_status = max(next_status + a.status_every, now + 0.5)

    def _finish(self):
        reason = self.stop_reason or "stopped"
        self.console.say(f"Stopping: {reason}. Closing files...")
        for src in (self.oak, self.wrist):
            if src is not None:
                src.finish()
        for w in self.writers.values():
            w.finish()
        t_end = time.monotonic()
        if self.fifo is not None:
            self.fifo.close()
        if self.clock is not None:
            self.clock.pair("stop")
            self.clock.close()
        duration = t_end - getattr(self, "t0", t_end)
        streams = {n: st.as_dict(duration) for n, st in self.stats.items()}
        if "depth" in self.writers:
            streams["depth"]["chunks"] = self.writers["depth"].chunks
            streams["depth"]["frame_shapes"] = self.writers["depth"].shapes
        errors = {n: w.error for n, w in self.writers.items() if w.error}
        total = sum(st.bytes for st in self.stats.values())
        summary = {"stop_reason": reason, "stopped": _iso(), "t_stop_mono_s": t_end, "duration_s": round(duration, 3),
                   "streams": streams, "events": self.events.n if self.events else 0,
                   "data_gb": round(_gb(total), 3), "mb_per_s": round(total / 1e6 / duration, 3) if duration else None,
                   "disk_free_gb_at_stop": round(_gb(free_bytes(self.out)), 2) if self.out else None,
                   "oak_sessions": self.oak.sessions if self.oak else None,
                   "wrist_sessions": self.wrist.sessions if self.wrist else None, "write_errors": errors}
        if self.meta:
            self.meta["status"] = "failed" if reason.startswith("failed") else "finished"
            self.meta["summary"] = summary
            write_json_atomic(self.out / PATHS["meta"], self.meta)
        if self.events is not None:
            self.events.close()
        try:
            (self.out / PATHS["pid"]).unlink()
        except (OSError, TypeError):
            pass
        lines = [f"Recorded {duration:.1f} s to {self.out}: {_gb(total):.2f} GB, "
                 f"{(total / 1e6 / duration) if duration else 0:.1f} MB/s."]
        for n, s in streams.items():
            lines.append(f"  {n:6s} {s['frames_written']:7d} frames written, {s['mean_fps'] or 0:.2f} fps, "
                         f"dropped {s['dropped_queue_full']}, device frames missing {s['device_seq_missing']}, "
                         f"{s['bytes'] / 1e9:.2f} GB")
        if errors:
            lines.append(f"  WRITE ERRORS: {errors}")
        lines.append(f"  events: {summary['events']}. Next: record.py check {self.out}")
        for line in lines:
            self.console.say(line)
        self.console.close()


# ----- check --------------------------------------------------------------------------------------
def _read_rows(path):
    """CSV rows as dicts, ignoring a partly written last line (live sessions)."""
    path = Path(path)
    if not path.exists():
        return []
    text = path.read_text(encoding="utf-8", errors="replace")
    if not text.endswith("\n"):
        text = text[: text.rfind("\n") + 1]
    rows = list(csv.DictReader(io.StringIO(text)))
    if rows:
        keys = list(rows[0].keys())
        rows = [r for r in rows if None not in r.values() and len(r) == len(keys)]
    return rows


def _num(rows, key, dtype=float):
    out = np.full(len(rows), np.nan)
    for i, r in enumerate(rows):
        v = r.get(key, "")
        if v not in ("", None):
            try:
                out[i] = float(v)
            except ValueError:
                pass
    return out.astype(np.int64) if dtype is int and np.isfinite(out).all() else out


def _pct(x, q):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(np.percentile(x, q)) if x.size else None


def stream_stats(t, seq=None, nominal_fps=None):
    t = np.asarray(t, float)
    t = t[np.isfinite(t)]
    out = {"frames": int(t.size)}
    if t.size < 2:
        return out
    dt = np.diff(t)
    med = float(np.median(dt))
    thr = 1.5 * max(med, 1.0 / float(nominal_fps) if nominal_fps else 0.0)
    gaps = dt > thr
    out.update(duration_s=float(t[-1] - t[0]), mean_fps=float((t.size - 1) / (t[-1] - t[0])) if t[-1] > t[0] else None,
               median_dt_ms=med * 1e3, gap_threshold_ms=thr * 1e3, gaps=int(gaps.sum()),
               gap_time_s=float((dt[gaps] - med).sum()) if gaps.any() else 0.0, max_gap_s=float(dt.max()),
               not_increasing=int((dt <= 0).sum()))
    if seq is not None:
        s = np.asarray(seq, float)
        s = s[np.isfinite(s)]
        if s.size > 1:
            ds = np.diff(s)
            out["device_seq_missing"] = int((ds[ds > 1] - 1).sum())
            out["device_seq_resets"] = int((ds <= 0).sum())
    return out


def _lat_stats(x_s):
    x = np.asarray(x_s, float) * 1e3
    x = x[np.isfinite(x)]
    if not x.size:
        return None
    return {"min_ms": float(x.min()), "p50_ms": float(np.median(x)), "p95_ms": float(np.percentile(x, 95)),
            "max_ms": float(x.max())}


class _Frames:
    """Random access to a session's frames."""

    def __init__(self, session, cv2):
        self.s, self.cv2 = Path(session), cv2
        self._fh = {}
        self._chunk = (None, None)

    def jpeg(self, stream, offset, length):
        fh = self._fh.get(stream)
        if fh is None:
            fh = self._fh[stream] = open(self.s / stream, "rb")  # noqa: SIM115
        fh.seek(int(offset))
        data = fh.read(int(length))
        if len(data) != int(length):
            return None
        return self.cv2.imdecode(np.frombuffer(data, np.uint8), self.cv2.IMREAD_COLOR)

    def depth(self, chunk, index):
        name, arr = self._chunk
        if name != chunk:
            try:
                with np.load(self.s / PATHS["depth_dir"] / chunk) as z:
                    arr = z["depth"]
            except (OSError, KeyError, ValueError, zipfile.BadZipFile):
                arr = None
            self._chunk = (chunk, arr)
        if arr is None or int(index) >= len(arr):
            return None
        return arr[int(index)]

    def close(self):
        for fh in self._fh.values():
            fh.close()


def charuco_board(cv2, spec=None):
    spec = spec or BOARD_SPEC
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec["dictionary"]))
    b = cv2.aruco.CharucoBoard((int(spec["squares_x"]), int(spec["squares_y"])), float(spec["square_length_m"]),
                               float(spec["marker_length_m"]), d)
    if hasattr(b, "setLegacyPattern"):
        b.setLegacyPattern(bool(spec.get("legacy_pattern", False)))
    return b


def detect_charuco(cv2, board, image, detector=None):
    """(object_points (N,3) m, image_points (N,2) px, ids) of the ChArUco corners found, or None."""
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    det = detector or cv2.aruco.CharucoDetector(board)
    cc, ci, _mc, _mi = det.detectBoard(gray)
    if ci is None or len(ci) < 4:
        return None
    obj, img = board.matchImagePoints(cc, ci)
    return np.asarray(obj, np.float64).reshape(-1, 3), np.asarray(img, np.float64).reshape(-1, 2), \
        np.asarray(ci).reshape(-1)


def _pnp(cv2, obj, img, K, dist):
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        return None
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    err = np.linalg.norm(proj.reshape(-1, 2) - img, axis=1)
    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.reshape(3), float(np.sqrt((err ** 2).mean()))


def _nearest(t_sorted, t):
    i = int(np.searchsorted(t_sorted, t))
    if i <= 0:
        return 0
    if i >= len(t_sorted):
        return len(t_sorted) - 1
    return i if abs(t_sorted[i] - t) < abs(t_sorted[i - 1] - t) else i - 1


def _event_time(events, *names, after=None):
    """Time of the last event with one of these labels (a redone step replaces the first try)."""
    times = [float(e["t_mono_s"]) for e in events if e["label"] in names]
    if after is not None:
        times = [t for t in times if t > after]
    return times[-1] if times else None


def _board_check(cv2, frames, meta, color, depth, events, spec):
    t_ev = _event_time(events, "board_placed")
    if t_ev is None:
        return {"status": "no board_placed event"}
    intr = ((meta.get("oakd") or {}).get("intrinsics") or {})
    K = np.array(intr.get("rgb_1280x720") or [[0, 0, 0]] * 3, float)
    if K.shape != (3, 3) or K[0, 0] <= 0:
        return {"status": "no OAK-D intrinsics in meta.json"}
    dist = np.array(intr.get("rgb_distortion") or [], float)
    dist = dist if dist.size in (4, 5, 8, 12, 14) else np.zeros(5)
    tc = color["t"]
    sel = np.where((tc >= t_ev + 1.0) & (tc <= t_ev + 9.0))[0]
    if not sel.size:
        return {"status": "no colour frames 1-9 s after board_placed (yet)"}
    board = charuco_board(cv2, spec)
    detector = cv2.aruco.CharucoDetector(board)
    picks = sel[np.linspace(0, sel.size - 1, min(8, sel.size)).astype(int)]
    results, best = [], None
    for i in picks:
        img = frames.jpeg(PATHS["color_stream"], color["off"][i], color["len"][i])
        if img is None:
            results.append({"frame_idx": int(i), "corners": 0, "error": "decode"})
            continue
        r = detect_charuco(cv2, board, img, detector)
        n = 0 if r is None else len(r[0])
        row = {"frame_idx": int(i), "corners": n}
        if r is not None and n >= 6:
            p = _pnp(cv2, r[0], r[1], K, dist)
            if p is not None:
                row.update(distance_m=float(np.linalg.norm(p[1])), reprojection_rms_px=p[2])
                if best is None or n > best[0]:
                    best = (n, i, r, p)
        results.append(row)
    total = (board.getChessboardCorners().shape[0])
    out = {"status": "ok" if best is not None else "BOARD NOT FOUND", "event_t_mono_s": t_ev,
           "corners_possible": int(total), "frames": results}
    if best is None:
        return out
    n, i, (obj, img, _ids), (R, tvec, rms) = best
    Xc = (R @ obj.T).T + tvec
    tilt = float(np.degrees(np.arccos(abs(R[2, 2]))))
    out.update(best_frame_idx=int(i), corners=int(n), camera_to_board_m=float(np.linalg.norm(tvec)),
               board_tilt_to_camera_axis_deg=tilt, reprojection_rms_px=rms,
               board_origin_in_camera_m=[float(v) for v in tvec])
    # Depth against the board's own pose: tests the depth scale and the RGB/depth alignment together.
    if depth["t"].size:
        j = _nearest(depth["t"], float(color["t"][i]))
        d = frames.depth(depth["chunk"][j], depth["k"][j])
        if d is not None:
            sx, sy = d.shape[1] / COLOR_W, d.shape[0] / COLOR_H
            diffs = []
            for (u, v), z in zip(img, Xc[:, 2], strict=True):
                uu, vv = int(round(u * sx)), int(round(v * sy))
                patch = d[max(vv - 1, 0):vv + 2, max(uu - 1, 0):uu + 2].astype(float)
                patch = patch[patch > 0]
                if patch.size:
                    diffs.append(float(np.median(patch)) - z * 1000.0)
            if diffs:
                diffs = np.array(diffs)
                out["depth_minus_board_mm"] = {"n": int(diffs.size), "median": float(np.median(diffs)),
                                               "p90_abs": float(np.percentile(np.abs(diffs), 90)),
                                               "relative_median": float(np.median(diffs) / (np.median(Xc[:, 2]) * 1e3)),
                                               "depth_dt_ms": float((depth["t"][j] - color["t"][i]) * 1e3)}
    Kc = intr.get("rgb_1280x720_crop_aware")
    if Kc:
        p2 = _pnp(cv2, obj, img, np.array(Kc, float), dist)
        if p2 is not None:
            out["crop_aware_K"] = {"camera_to_board_m": float(np.linalg.norm(p2[1])), "reprojection_rms_px": p2[2]}
    return out


def _calibration_check(cv2, frames, stream, rows_off, rows_len, t, t_from, t_to, spec, image_size,
                       max_candidates=150, max_views=40, min_corners=8):
    sel = np.where((t >= t_from) & (t <= t_to))[0]
    if not sel.size:
        return {"status": "no frames in the window"}
    board = charuco_board(cv2, spec)
    detector = cv2.aruco.CharucoDetector(board)
    cand = sel[np.linspace(0, sel.size - 1, min(max_candidates, sel.size)).astype(int)]
    views = []
    for i in cand:
        img = frames.jpeg(stream, rows_off[i], rows_len[i])
        if img is None:
            continue
        image_size = (img.shape[1], img.shape[0])
        r = detect_charuco(cv2, board, img, detector)
        if r is not None and len(r[0]) >= min_corners:
            views.append((int(i), r))
    out = {"window_s": [float(t_from), float(t_to)], "frames_tried": int(cand.size),
           "frames_with_board": len(views)}
    if len(views) < 6:
        out["status"] = "too few views with the board (need >= 6; aim for 15 different angles)"
        return out
    pick = [views[k] for k in np.linspace(0, len(views) - 1, min(max_views, len(views))).astype(int)]
    obj = [v[1][0].astype(np.float32) for v in pick]
    img = [v[1][1].astype(np.float32) for v in pick]
    rms, K, dist, _r, _t = cv2.calibrateCamera(obj, img, image_size, None, None)
    out.update(status="ok", views_used=len(pick), rms_px=float(rms), image_size=list(image_size),
               fx=float(K[0, 0]), fy=float(K[1, 1]), cx=float(K[0, 2]), cy=float(K[1, 2]),
               dist=[float(x) for x in np.ravel(dist)[:5]])
    return out


def _load_events(session):
    rows = _read_rows(Path(session) / PATHS["events"])
    return [{"n": r.get("n"), "t_mono_s": float(r["t_mono_s"]), "label": r.get("label", ""),
             "source": r.get("source", "")} for r in rows if r.get("t_mono_s")]


def _depth_colormap(cv2, d, lo, hi, size):
    d = d.astype(np.float32)
    x = np.clip((d - lo) / max(hi - lo, 1.0), 0, 1)
    img = cv2.applyColorMap((255 - x * 255).astype(np.uint8), getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET))
    img[d <= 0] = 0
    return cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)


def _tile(cv2, img, size, text=None):
    if img is None:
        out = np.full((size[1], size[0], 3), 40, np.uint8)
        cv2.putText(out, text or "no frame", (10, size[1] // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        return out
    return cv2.resize(img, size, interpolation=cv2.INTER_AREA)


def contact_sheet(cv2, frames, session, meta, color, depth, wrist, events, out_path, every=None, max_rows=40):
    """One row per sampled colour frame (and one per event): colour, depth colormap, wrist frame."""
    t_all = [x["t"] for x in (color, depth, wrist) if x["t"].size]
    if not t_all:
        return None
    base = color["t"] if color["t"].size else (wrist["t"] if wrist["t"].size else depth["t"])
    t0 = float(meta.get("t0_mono_s") or min(float(x[0]) for x in t_all))
    n_periodic = max(1, max_rows - min(len(events), max_rows // 2))
    step = every or max(1, int(math.ceil(base.size / n_periodic)))
    times = [(float(base[i]), None) for i in range(0, base.size, step)]
    times += [(float(e["t_mono_s"]) + 0.5, e["label"]) for e in events]
    times.sort(key=lambda x: x[0])
    times = times[:max_rows] if len(times) <= max_rows else \
        [times[i] for i in np.linspace(0, len(times) - 1, max_rows).astype(int)]
    # depth colour range from the sampled frames
    samples = []
    for t, _ in times[:: max(1, len(times) // 8)]:
        if depth["t"].size:
            j = _nearest(depth["t"], t)
            d = frames.depth(depth["chunk"][j], depth["k"][j])
            if d is not None and (d > 0).any():
                samples.append(d[d > 0])
    lo, hi = (np.percentile(np.concatenate(samples), [2, 98]) if samples else (300.0, 1500.0))
    CW, CH, WW, LW = 320, 180, 240, 250
    rows_img = []
    ev_t = np.array([float(e["t_mono_s"]) for e in events]) if events else np.array([])
    for t, label in times:
        cimg = dimg = wimg = None
        dc = dd = dw = None
        if color["t"].size:
            i = _nearest(color["t"], t)
            dc = color["t"][i] - t
            cimg = frames.jpeg(PATHS["color_stream"], color["off"][i], color["len"][i])
        if depth["t"].size:
            j = _nearest(depth["t"], t)
            dd = depth["t"][j] - t
            d = frames.depth(depth["chunk"][j], depth["k"][j])
            dimg = _depth_colormap(cv2, d, lo, hi, (CW, CH)) if d is not None else None
        if wrist["t"].size:
            k = _nearest(wrist["t"], t)
            dw = wrist["t"][k] - t
            wimg = frames.jpeg(PATHS["wrist_stream"], wrist["off"][k], wrist["len"][k])
        panel = np.full((CH, LW, 3), 255, np.uint8)
        prev = [events[q]["label"] for q in np.where(ev_t <= t)[0]] if ev_t.size else []
        lines = [f"t = +{t - t0:.1f} s"]
        lines += [f"colour dt {dc * 1e3:+.0f} ms" if dc is not None else "colour: none",
                  f"depth  dt {dd * 1e3:+.0f} ms" if dd is not None else "depth: none",
                  f"wrist  dt {dw * 1e3:+.0f} ms" if dw is not None else "wrist: none"]
        lines.append(f"EVENT {label}" if label else (f"after: {prev[-1]}" if prev else ""))
        for li, text in enumerate(lines):
            cv2.putText(panel, text, (8, 28 + 30 * li), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 0, 200) if (label and li == 4) else (0, 0, 0), 1, cv2.LINE_AA)
        row = np.hstack([panel, _tile(cv2, cimg, (CW, CH)), dimg if dimg is not None else _tile(cv2, None, (CW, CH)),
                         _tile(cv2, wimg, (WW, CH))])
        rows_img.append(np.vstack([row, np.full((4, row.shape[1], 3), 255, np.uint8)]))
    header = np.full((40, rows_img[0].shape[1], 3), 255, np.uint8)
    cv2.putText(header, f"{Path(session).name}   colour | depth {lo:.0f}-{hi:.0f} mm (near=red) | wrist", (8, 27),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
    sheet = np.vstack([header, *rows_img])
    cv2.imwrite(str(out_path), sheet)
    return {"path": str(out_path), "rows": len(rows_img), "depth_range_mm": [float(lo), float(hi)]}


def _streams(session):
    s = Path(session)
    c = _read_rows(s / PATHS["color_index"])
    d = _read_rows(s / PATHS["depth_index"])
    w = _read_rows(s / PATHS["wrist_index"])
    color = {"rows": c, "t": _num(c, "t_mono_s"), "seq": _num(c, "device_seq"), "off": _num(c, "byte_offset"),
             "len": _num(c, "byte_len"), "host": _num(c, "t_host_s"), "sess": _num(c, "oak_session")}
    depth = {"rows": d, "t": _num(d, "t_mono_s"), "seq": _num(d, "device_seq"), "k": _num(d, "index_in_chunk"),
             "chunk": [r.get("chunk", "") for r in d], "host": _num(d, "t_host_s"), "valid": _num(d, "valid_frac"),
             "sess": _num(d, "oak_session")}
    wrist = {"rows": w, "t": _num(w, "t_mono_s"), "off": _num(w, "byte_offset"), "len": _num(w, "byte_len"),
             "v4l2": _num(w, "t_v4l2_s"), "raw": _num(w, "raw")}
    return color, depth, wrist


def check_session(session, *, every=None, max_rows=40, quick=False, out_dir=None, board_spec=None,
                  cv2=None, say=print):
    """Summarise a (finished or live) recording. Returns (report dict, problems list)."""
    session = Path(session).expanduser()
    meta_path = session / PATHS["meta"]
    if not meta_path.exists():
        raise RecordError(f"{session} has no meta.json: not a recording")
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    settings = meta.get("settings") or {}
    color, depth, wrist = _streams(session)
    events = _load_events(session)
    problems, warns = [], []
    rep = {"session": str(session), "format": meta.get("format"), "status": meta.get("status"),
           "live": meta.get("status") == "recording", "streams": {}, "events": events}
    expect = {"color": meta.get("oakd") is not None, "depth": meta.get("oakd") is not None,
              "wrist": meta.get("wrist") is not None}
    nominal = {"color": settings.get("color_fps", 30), "depth": settings.get("depth_fps", 30),
               "wrist": settings.get("wrist_fps", 30)}
    for name, st in (("color", color), ("depth", depth), ("wrist", wrist)):
        s = stream_stats(st["t"], st.get("seq"), nominal[name])
        if name in ("color", "depth"):
            s["host_latency"] = _lat_stats(st["host"] - st["t"])
            lat = s["host_latency"]
            if lat and lat["min_ms"] < -5.0:
                problems.append(f"{name}: a frame was dequeued {-lat['min_ms']:.0f} ms before its timestamp, so the "
                                "OAK-D timestamps are not on this host's monotonic clock")
        if name == "wrist" and st["t"].size:
            s["t_mono_minus_t_v4l2"] = _lat_stats(st["t"] - st["v4l2"])
            s["raw_mjpeg_frames"] = int(np.nansum(st["raw"]))
        if name == "depth" and st["t"].size:
            s["valid_fraction_median"] = _pct(st["valid"], 50)
        rep["streams"][name] = s
        if expect[name]:
            if s["frames"] == 0:
                problems.append(f"{name}: no frames")
                continue
            if s.get("not_increasing"):
                problems.append(f"{name}: {s['not_increasing']} timestamps not increasing")
            if s.get("mean_fps") and s["mean_fps"] < 0.8 * float(nominal[name]):
                problems.append(f"{name}: {s['mean_fps']:.1f} fps, under 80 % of {nominal[name]}")
            if s.get("max_gap_s", 0) > 1.0:
                problems.append(f"{name}: a {s['max_gap_s']:.2f} s gap")
            if s.get("device_seq_missing"):
                warns.append(f"{name}: {s['device_seq_missing']} frames missing by device sequence number")
    # files
    for name, st, stream in (("color", color, PATHS["color_stream"]), ("wrist", wrist, PATHS["wrist_stream"])):
        if st["t"].size:
            end = float(st["off"][-1] + st["len"][-1])
            size = (session / stream).stat().st_size if (session / stream).exists() else 0
            if size < end:
                problems.append(f"{name}: {stream} is {size} bytes but the index points to {end:.0f}")
    if depth["t"].size:
        missing = sorted({c for c in depth["chunk"] if not (session / PATHS["depth_dir"] / c).exists()})
        if missing:
            problems.append(f"depth: {len(missing)} chunk files missing, e.g. {missing[0]}")
    if color["t"].size and depth["t"].size:
        tc = np.sort(color["t"][np.isfinite(color["t"])])
        dts = [abs(tc[_nearest(tc, t)] - t) for t in depth["t"][:: max(1, depth["t"].size // 2000)]
               if np.isfinite(t)]
        rep["color_depth_nearest_ms"] = {"median": float(np.median(dts) * 1e3),
                                         "p95": float(np.percentile(dts, 95) * 1e3)}
    clock = _read_rows(session / PATHS["clock"])
    if clock:
        off = np.array([float(r["t_wall_s"]) - float(r["t_mono_s"]) for r in clock])
        rep["clock"] = {"pairs": len(clock), "wall_minus_mono_s": float(off[0]),
                        "drift_ms": float((off.max() - off.min()) * 1e3),
                        "max_uncertainty_us": float(max(float(r["uncertainty_s"]) for r in clock) * 1e6)}
    else:
        problems.append("clock.csv is empty")
    out_dir = Path(out_dir) if out_dir else session / PATHS["check_dir"]
    out_dir.mkdir(parents=True, exist_ok=True)
    try:
        cv2 = cv2 or import_cv2()
    except RecordError as e:
        cv2 = None
        warns.append(f"{e}; skipped the board checks and the contact sheet")
    if cv2 is not None:
        frames = _Frames(session, cv2)
        try:
            spec = board_spec or meta.get("board") or BOARD_SPEC
            if color["t"].size:
                rep["board_placed"] = _board_check(cv2, frames, meta, color, depth, events, spec)
                bp = rep["board_placed"]
                if bp.get("status") == "BOARD NOT FOUND":
                    problems.append("the board was not found in the OAK-D colour frames after board_placed")
            if not quick:
                t_from = _event_time(events, "wrist_calib_start")
                t_to = _event_time(events, "wrist_calib_end", after=t_from) if t_from is not None else None
                if wrist["t"].size and t_from is not None:
                    rep["wrist_calibration"] = _calibration_check(
                        cv2, frames, PATHS["wrist_stream"], wrist["off"], wrist["len"], wrist["t"], t_from,
                        t_to if t_to is not None else t_from + 90.0, spec, (640, 480))
                t_from = _event_time(events, "oakd_views_start")
                t_to = _event_time(events, "oakd_views_end", after=t_from) if t_from is not None else None
                if color["t"].size and t_from is not None:
                    oc = _calibration_check(cv2, frames, PATHS["color_stream"], color["off"], color["len"], color["t"],
                                            t_from, t_to if t_to is not None else t_from + 60.0, spec,
                                            (COLOR_W, COLOR_H), max_candidates=80, max_views=30)
                    intr = ((meta.get("oakd") or {}).get("intrinsics") or {})
                    for key in ("rgb_1280x720", "rgb_1280x720_crop_aware"):
                        if oc.get("status") == "ok" and intr.get(key):
                            oc[f"fx_ratio_to_{key}"] = oc["fx"] / float(intr[key][0][0])
                    rep["oakd_views_calibration"] = oc
                rep["contact_sheet"] = contact_sheet(cv2, frames, session, meta, color, depth, wrist, events,
                                                     out_dir / "contact_sheet.png", every=every, max_rows=max_rows)
        finally:
            frames.close()
    rep["problems"], rep["warnings"] = problems, warns
    rep["result"] = "PROBLEMS" if problems else "OK"
    write_json_atomic(out_dir / ("check-live.json" if rep["live"] else "check.json"), rep)
    _print_check(rep, meta, say)
    return rep, problems


def _print_check(rep, meta, say):
    t0 = meta.get("t0_mono_s") or 0.0
    say(f"Session {rep['session']} ({'LIVE, still recording' if rep['live'] else rep['status']})")
    for name, s in rep["streams"].items():
        if not s.get("frames"):
            say(f"  {name:6s} no frames")
            continue
        extra = ""
        if s.get("device_seq_missing") is not None:
            extra += f", device frames missing {s['device_seq_missing']}"
        if s.get("host_latency"):
            extra += f", latency p50 {s['host_latency']['p50_ms']:.0f} ms (min {s['host_latency']['min_ms']:.0f})"
        if s.get("valid_fraction_median") is not None:
            extra += f", valid depth {s['valid_fraction_median'] * 100:.0f} %"
        say(f"  {name:6s} {s['frames']:6d} frames over {s.get('duration_s', 0):7.1f} s = "
            f"{s.get('mean_fps') or 0:5.2f} fps, gaps {s.get('gaps', 0)} "
            f"(max {s.get('max_gap_s', 0) * 1e3:.0f} ms), non-increasing {s.get('not_increasing', 0)}{extra}")
    if rep.get("color_depth_nearest_ms"):
        cd = rep["color_depth_nearest_ms"]
        say(f"  colour/depth nearest-frame offset: median {cd['median']:.1f} ms, p95 {cd['p95']:.1f} ms")
    if rep.get("clock"):
        c = rep["clock"]
        say(f"  clock: {c['pairs']} pairs; wall - monotonic drifted {c['drift_ms']:.2f} ms")
    if rep["events"]:
        say("  events: " + ", ".join(f"+{e['t_mono_s'] - t0:.1f}s {e['label']}" for e in rep["events"]))
    bp = rep.get("board_placed")
    if bp:
        if bp.get("corners"):
            msg = (f"  board @ board_placed: {bp['corners']}/{bp['corners_possible']} corners, camera->board "
                   f"{bp['camera_to_board_m']:.3f} m, tilt {bp['board_tilt_to_camera_axis_deg']:.0f} deg, reprojection "
                   f"{bp['reprojection_rms_px']:.2f} px")
            dm = bp.get("depth_minus_board_mm")
            if dm:
                msg += f"; depth - board {dm['median']:+.1f} mm median (|p90| {dm['p90_abs']:.1f} mm, n {dm['n']})"
            say(msg)
        else:
            say(f"  board @ board_placed: {bp.get('status')} {[f.get('corners') for f in bp.get('frames', [])]}")
    for key, label in (("wrist_calibration", "wrist calibration views"),
                       ("oakd_views_calibration", "OAK-D board views")):
        c = rep.get(key)
        if c:
            if c.get("status") == "ok":
                extra = "".join(f", fx/{k[12:]} {v:.3f}" for k, v in c.items() if k.startswith("fx_ratio_to_"))
                say(f"  {label}: {c['views_used']} views of {c['frames_with_board']} with the board, RMS "
                    f"{c['rms_px']:.2f} px, fx {c['fx']:.1f} fy {c['fy']:.1f} cx {c['cx']:.1f} cy {c['cy']:.1f}{extra}")
            else:
                say(f"  {label}: {c.get('status')} ({c.get('frames_with_board', 0)} frames with the board)")
    if rep.get("contact_sheet"):
        say(f"  contact sheet: {rep['contact_sheet']['path']} ({rep['contact_sheet']['rows']} rows)")
    for w in rep["warnings"]:
        say(f"  note: {w}")
    say("RESULT: " + rep["result"] + ("" if not rep["problems"] else "\n  - " + "\n  - ".join(rep["problems"])))


# ----- the printable board ------------------------------------------------------------------------
LETTER_MM = (215.9, 279.4)


def make_board(out_png, pdf_path=None, json_path=None, *, px_per_mm=10, spec=None, cv2=None):
    """A ChArUco board for US Letter at 100 %, with 100 mm scale bars and labelled corners O and X.

    At 10 px/mm the page is 2159 x 2794 px, every square exactly 300 px and every marker 220 px, and the
    PDF page is exactly 612 x 792 pt.
    """
    cv2 = cv2 or import_cv2()
    spec = dict(spec or BOARD_SPEC)
    ppm = int(px_per_mm)
    sx, sy = int(spec["squares_x"]), int(spec["squares_y"])
    sq_mm, mk_mm = spec["square_length_m"] * 1000, spec["marker_length_m"] * 1000
    sq_px = sq_mm * ppm
    if abs(sq_px - round(sq_px)) > 1e-6 or abs(mk_mm * ppm - round(mk_mm * ppm)) > 1e-6:
        raise RecordError("square and marker sizes must be whole pixels at this px_per_mm")
    sq_px = int(round(sq_px))
    W, H = int(round(LETTER_MM[0] * ppm)), int(round(LETTER_MM[1] * ppm))
    page = np.full((H, W), 255, np.uint8)
    board = charuco_board(cv2, spec)
    bw, bh = sx * sq_px, sy * sq_px
    img = board.generateImage((bw, bh), marginSize=0, borderBits=1)
    x0, y0 = 33 * ppm, 25 * ppm                         # board's top-left (corner O), in px
    page[y0:y0 + bh, x0:x0 + bw] = img
    black, font = 0, cv2.FONT_HERSHEY_SIMPLEX

    def text(s, x_mm, y_mm, h_mm=3.2, thick=None):
        scale = h_mm * ppm / 22.0
        cv2.putText(page, s, (int(x_mm * ppm), int(y_mm * ppm)), font, scale, black,
                    thick or max(1, int(round(scale * 1.6))), cv2.LINE_8)

    def px(mm):
        return int(round(mm * ppm))

    bx0, by0 = x0 / ppm, y0 / ppm
    bx1, by1 = bx0 + sx * sq_mm, by0 + sy * sq_mm
    text(f"ChArUco {sx}x{sy}  {spec['dictionary']}  squares {sq_mm:g} mm  markers {mk_mm:g} mm", 12, 11, 3.4)
    text("bounded-embodiment board v1: origin O, axes as arrowed, z into the table (lay it face up)", 12, 16.5, 2.6)
    # axes from corner O, drawn outside the board
    cv2.arrowedLine(page, (px(bx0 + 8), px(by0 - 4)), (px(bx0 + 48), px(by0 - 4)), black, 4, cv2.LINE_8,
                    tipLength=0.15)
    text("+x", bx0 + 50, by0 - 2.5, 3.4)
    cv2.arrowedLine(page, (px(bx0 - 4), px(by0 + 8)), (px(bx0 - 4), px(by0 + 48)), black, 4, cv2.LINE_8,
                    tipLength=0.15)
    text("+y", bx0 - 12, by0 + 55, 3.4)
    text("O", bx0 - 8.5, by0 - 1.5, 5.0)
    text("X", bx1 + 2.5, by0 - 1.5, 5.0)
    text("Y", bx0 - 8.5, by1 + 2.5, 5.0)
    # ticks on the extensions of the board's edges, pointing at the three corners to measure
    for cx, cy, side in ((bx0, by0, -1), (bx1, by0, +1)):
        cv2.line(page, (px(cx + side * 2), px(cy)), (px(cx + side * 7), px(cy)), black, 2)
        cv2.line(page, (px(cx), px(cy - 2)), (px(cx), px(cy - 7)), black, 2)
    cv2.line(page, (px(bx0 - 2), px(by1)), (px(bx0 - 7), px(by1)), black, 2)
    text("Measure corners O, X and Y: the outer corners of the black corner squares.", 40, by1 + 6, 2.8)
    # horizontal 100 mm bar
    yb = by1 + 10
    cv2.rectangle(page, (int(bx0 * ppm), int(yb * ppm)), (int((bx0 + 100) * ppm) - 1, int((yb + 3) * ppm) - 1),
                  black, -1)
    for k in range(11):
        xx = int((bx0 + 10 * k) * ppm)
        cv2.line(page, (min(xx, int((bx0 + 100) * ppm) - 1), int((yb + 3) * ppm)),
                 (min(xx, int((bx0 + 100) * ppm) - 1), int((yb + 6) * ppm)), black, 2)
    text("0", bx0 - 1, yb + 10, 2.6)
    text("100 mm", bx0 + 92, yb + 10, 2.6)
    # vertical 100 mm bar, right of the board
    xv = bx1 + 12
    cv2.rectangle(page, (int(xv * ppm), int(by0 * ppm)), (int((xv + 3) * ppm) - 1, int((by0 + 100) * ppm) - 1),
                  black, -1)
    for k in range(11):
        yy = min(int((by0 + 10 * k) * ppm), int((by0 + 100) * ppm) - 1)
        cv2.line(page, (int((xv + 3) * ppm), yy), (int((xv + 6) * ppm), yy), black, 2)
    text("100", xv + 7, by0 + 101, 2.6)
    text("mm", xv + 7, by0 + 105, 2.6)
    text("print at 100%; check the bar with a ruler", 12, yb + 18, 3.6)
    text("(both bars: 100 mm each). Mount flat on something stiff; do not fold.", 12, yb + 23.5, 2.6)

    out_png = Path(out_png)
    out_png.parent.mkdir(parents=True, exist_ok=True)
    pdf_note = None
    try:
        from PIL import Image  # noqa: PLC0415
        pil = Image.fromarray(np.where(page >= 128, 255, 0).astype(np.uint8)).convert("1", dither=0)
        dpi = ppm * 25.4
        pil.save(out_png, dpi=(dpi, dpi))
        if pdf_path:
            Path(pdf_path).parent.mkdir(parents=True, exist_ok=True)
            pil.save(pdf_path, "PDF", resolution=dpi)
            pdf_note = str(pdf_path)
    except ImportError:
        cv2.imwrite(str(out_png), page)
        if pdf_path:
            pdf_note = None
    corners = board.getChessboardCorners().reshape(-1, 3)
    desc = {
        **spec,
        "opencv_version": cv2.__version__,
        "marker_ids": [int(i) for i in np.ravel(board.getIds())],
        "chessboard_corners": int(corners.shape[0]),
        "origin": "corner O: the outer corner of the black square at the board's top-left as printed",
        "axes": {"x": "from O along the 5-square edge, towards corner X (+x arrow)",
                 "y": "from O along the 7-square edge (+y arrow)",
                 "z": "into the board: down into the table when it lies face up"},
        "corner_X_m": [float(sx * spec["square_length_m"]), 0.0, 0.0],
        "corner_Y_m": [0.0, float(sy * spec["square_length_m"]), 0.0],
        "size_m": [float(sx * spec["square_length_m"]), float(sy * spec["square_length_m"])],
        "chessboard_corners_m": corners.tolist(),
        "page": {"size": "US Letter", "mm": list(LETTER_MM), "px_per_mm": ppm, "dpi": ppm * 25.4,
                 "corner_O_on_page_mm": [bx0, by0], "scale_bars_mm": 100},
        "print": "print at 100% (Actual size, not Fit); check both 100 mm bars with a ruler; write the measured "
                 "lengths in measurements.txt",
        "files": {"png": str(out_png), "pdf": pdf_note},
    }
    if pdf_path and pdf_note is None:
        desc["files"]["pdf_error"] = "Pillow is not installed, so no PDF; print the PNG at 254 dpi (100 %)"
    if json_path:
        Path(json_path).write_text(json.dumps(desc, indent=2) + "\n", encoding="utf-8")
    return desc


# ----- devices, mark, stop ------------------------------------------------------------------------
def show_devices(say=print, out=None):
    say("OAK-D (DepthAI):")
    try:
        dai = import_depthai()
        fn = getattr(dai.Device, "getAllConnectedDevices", None) or dai.Device.getAllAvailableDevices
        devs = fn()
        if not devs:
            say("  none found by DepthAI")
        for d in devs:
            say(f"  {_call(d, 'getMxId') or getattr(d, 'mxid', '?')}  name {getattr(d, 'name', '?')}  "
                f"state {_name(getattr(d, 'state', '?'))}  depthai {dai.__version__}")
    except RecordError as e:
        say(f"  {e}".replace("\n", "\n  "))
    for u in list_oak_usb():
        speed = {"5000": "USB3 (5 Gbps)", "10000": "USB3 (10 Gbps)", "480": "USB2 (480 Mbps)"}.get(
            u["speed_mbps"], u["speed_mbps"] + " Mbps")
        held = "; held by " + ", ".join(f"pid {p} {c}" for p, c in u["held_by"]) if u["held_by"] else ""
        say(f"  USB {u['node']}: 03e7:{u['product_id']} {u['state']}, {speed}{held}")
    say("Cameras (V4L2):")
    cams = list_v4l2()
    usb = [c for c in cams if c["usb"]]
    for c in usb:
        held, hidden = holders(c["device"])
        hs = "; held by " + ", ".join(f"pid {p} {cmd}" for p, cmd in held) if held else ""
        say(f"  {c['device']}  {c['name']}  index {c['index']}  {' '.join(c['by_id'])}{hs}")
    others = [c for c in cams if not c["usb"]]
    if others:
        say(f"  ({len(others)} non-USB video nodes, e.g. the Pi's ISP and codecs, not listed)")
    pick, found = find_wrist_camera()
    say(f"  --wrist auto would use: {pick}" + (f" (of {len(found)})" if len(found) > 1 else ""))
    target = Path(out).expanduser() if out else Path.home()
    say(f"Disk: {_gb(free_bytes(target)):.1f} GB free at {target}")
    return 0


def send_mark(label, fifo):
    path = Path(fifo).expanduser()
    if not path.exists():
        raise RecordError(f"no FIFO at {path}: is the recorder running with --events-fifo {fifo}?")
    if not stat.S_ISFIFO(path.stat().st_mode):
        raise RecordError(f"{path} is not a FIFO")
    try:
        fd = os.open(path, os.O_WRONLY | os.O_NONBLOCK)
    except OSError as e:
        if e.errno == errno.ENXIO:
            raise RecordError("nothing is reading the FIFO: the recorder is not running") from e
        raise
    try:
        os.write(fd, (clean_label(label) + "\n").encode())
    finally:
        os.close(fd)
    return clean_label(label)


def stop_recorder(session, timeout=120.0, say=print):
    pid_path = Path(session).expanduser() / PATHS["pid"]
    if not pid_path.exists():
        raise RecordError(f"no {PATHS['pid']} in {session}: the recorder is not running there")
    pid = int(pid_path.read_text().strip())
    try:
        os.kill(pid, signal.SIGINT)
    except ProcessLookupError as e:
        raise RecordError(f"process {pid} is gone; the recording ended without finishing its files "
                          "(meta.json has no summary). The data written so far is usable.") from e
    say(f"Sent Ctrl+C to recorder pid {pid}; waiting for it to close its files...")
    deadline = time.monotonic() + timeout
    while pid_path.exists() and time.monotonic() < deadline:
        time.sleep(0.25)
    if pid_path.exists():
        raise RecordError(f"the recorder (pid {pid}) is still running after {timeout:.0f} s")
    say("Stopped.")
    return 0


# ----- CLI ----------------------------------------------------------------------------------------
def add_arguments(p):
    """Recording options on `p`, and the check/board/devices/mark/stop subcommands."""
    p.add_argument("--out", default=None, help="session directory to create (default ./be-rec-<date>-<time>)")
    p.add_argument("--seconds", type=float, default=None,
                   help=f"stop by itself after this many seconds; also the length the disk check plans for "
                        f"(default: run until Ctrl+C, and plan for {DEFAULT_PLAN_SECONDS:.0f} s)")
    p.add_argument("--wrist", default="auto", help="wrist camera: auto (first USB camera), none, /dev/videoN, "
                                                   "a /dev/v4l/by-id path, or an index")
    p.add_argument("--wrist-size", type=parse_size, default=(640, 480), metavar="WxH")
    p.add_argument("--wrist-fps", type=float, default=30.0)
    p.add_argument("--wrist-fourcc", default="MJPG")
    p.add_argument("--wrist-autofocus", choices=("off", "on", "leave"), default="off",
                   help="off keeps the focus (and so the intrinsics) fixed (default)")
    p.add_argument("--wrist-jpeg-quality", type=int, default=90,
                   help="only when the camera's MJPEG cannot be passed through")
    p.add_argument("--no-oak", action="store_true", help="record without the OAK-D")
    p.add_argument("--oak-device", default=None, help="MxId or name of the OAK-D (default: the first one)")
    p.add_argument("--color-fps", type=float, default=30.0)
    p.add_argument("--depth-fps", type=float, default=30.0, help="30 (default) or 15 to halve the depth data")
    p.add_argument("--mjpeg-quality", type=int, default=85, help="OAK-D colour MJPEG quality, 1-100 (default 85)")
    p.add_argument("--stereo-preset", choices=STEREO_PRESETS, default="HIGH_DENSITY",
                   help="HIGH_DENSITY (default) or HIGH_ACCURACY; either way no temporal/spatial filtering")
    p.add_argument("--median", choices=("preset", "off", "3", "5", "7"), default="preset",
                   help="stereo median filter (default: the preset's, 5x5)")
    p.add_argument("--mono-resolution", choices=("THE_400_P", "THE_480_P", "THE_720_P", "THE_800_P"), default=None,
                   help="default THE_400_P (THE_480_P for OV7251 sensors)")
    p.add_argument("--rgb-focus", type=int, default=None,
                   help="colour lens position 0-255 (default: the calibration's own, so the intrinsics hold)")
    p.add_argument("--rgb-exposure-us", type=int, default=None, help="manual colour exposure (default auto)")
    p.add_argument("--rgb-iso", type=int, default=400)
    p.add_argument("--chunk-frames", type=int, default=150, help="depth frames per .npz chunk")
    p.add_argument("--depth-compress-level", type=int, default=1, choices=range(0, 10), metavar="0-9")
    p.add_argument("--events-fifo", default=None, help="also read event labels from this named pipe")
    p.add_argument("--no-stdin-events", action="store_true", help="do not read event labels from the keyboard")
    p.add_argument("--min-free-gb", type=float, default=2.0, help="stop if free disk falls below this")
    p.add_argument("--status-every", type=float, default=2.0)
    p.add_argument("--first-frame-timeout", type=float, default=15.0)
    sub = p.add_subparsers(dest="record_command", metavar="{check,board,devices,mark,stop}")
    c = sub.add_parser("check", help="summarise a recording; write check/contact_sheet.png")
    c.add_argument("session")
    c.add_argument("--every", type=int, default=None, help="contact sheet: every Nth colour frame")
    c.add_argument("--max-rows", type=int, default=40)
    c.add_argument("--quick", action="store_true", help="stats and the board check only (for a live session)")
    c.add_argument("--board-json", default=None, help="board.json, if not the default board")
    b = sub.add_parser("board", help="write the printable ChArUco board")
    b.add_argument("--out", dest="board_out", default="board.png")
    b.add_argument("--pdf", dest="board_pdf", default=None)
    b.add_argument("--json", dest="board_json_out", default=None, help="default: next to --out, board.json")
    d = sub.add_parser("devices", help="list the OAK-D and cameras, and who holds them")
    d.add_argument("--disk", dest="devices_disk", default=None, help="also report free space here")
    m = sub.add_parser("mark", help="log an event in a running recorder through its --events-fifo")
    m.add_argument("label")
    m.add_argument("--fifo", dest="mark_fifo", default=os.environ.get("BE_RECORD_FIFO"))
    s = sub.add_parser("stop", help="stop the recorder writing to SESSION (as Ctrl+C would)")
    s.add_argument("stop_session", metavar="session")
    return p


def run(args):
    """Dispatch a parsed `be record` command line. Returns the exit code."""
    cmd = getattr(args, "record_command", None)
    try:
        if cmd == "check":
            spec = json.loads(Path(args.board_json).read_text()) if args.board_json else None
            _, problems = check_session(args.session, every=args.every, max_rows=args.max_rows, quick=args.quick,
                                        board_spec=spec)
            return 1 if problems else 0
        if cmd == "board":
            jp = args.board_json_out or str(Path(args.board_out).with_name("board.json"))
            desc = make_board(args.board_out, args.board_pdf, jp)
            print(f"Wrote {desc['files']['png']}" + (f", {desc['files']['pdf']}" if desc["files"]["pdf"] else "") +
                  f" and {jp}.")
            if args.board_pdf and not desc["files"]["pdf"]:
                print("No PDF: Pillow is not installed (pip install pillow). Print the PNG at 254 dpi / 100 %.")
            print("Print at 100% (Actual size, not Fit to page); check both 100 mm bars with a ruler.")
            return 0
        if cmd == "devices":
            return show_devices(out=args.devices_disk)
        if cmd == "mark":
            if not args.mark_fifo:
                raise RecordError("give --fifo PATH (the recorder's --events-fifo) or set BE_RECORD_FIFO")
            print(f"marked {send_mark(args.label, args.mark_fifo)}")
            return 0
        if cmd == "stop":
            return stop_recorder(args.stop_session)
        if args.out is None:
            args.out = time.strftime("be-rec-%Y%m%d-%H%M%S")
        stdin = None if args.no_stdin_events else sys.stdin
        return Recorder(args, stdin=stdin).run()
    except RecordError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr)
        return 130


def register(subparsers):
    """Adds `record` to the `be` command line (entry-point group bounded_embodiment.cli)."""
    p = subparsers.add_parser("record", help="record the OAK-D and wrist camera for the markerless instrument",
                              description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_arguments(p)
    p.set_defaults(func=run)
    return p


def main(argv=None):
    p = argparse.ArgumentParser(prog="record.py", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    add_arguments(p)
    return run(p.parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
