"""Helpers for the markerless instrument's tests (fixtures are in conftest.py)."""

from __future__ import annotations

import builtins
import contextlib
import csv
import io
import json
import os
import pathlib
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[3]
URDF = REPO / "examples" / "robots" / "so101" / "so101_new_calib.urdf"
ENVELOPE = REPO / "examples" / "envelopes" / "bob.envelope.json"
FETCH_HINT = ("fetch them with: python tools/fetch_robot_assets.py examples/robots/so101 --from-dir "
              "<SO-ARM100 checkout>/Simulation/SO101")
POINTS = ("tool", "elbow", "wrist")
NAMED = {"tool": ("gripper_frame_link", None), "elbow": ("lower_arm_link", None), "wrist": ("wrist_link", None)}
MOTION = {"nominal_s": 3.0, "still_s": 0.5, "seed": 2}     # the short motion recordings (90 frames of motion)
HAND = ((0.08, 0.09, 0.09), 0.04)    # a 40 mm sphere beside the shoulder, ~9 mm from the arm at its closest


def need_fit_stack() -> None:
    """Skip unless OpenCV, SciPy and the SO-101 meshes are all here (the meshes are not in git)."""
    pytest.importorskip("cv2")
    pytest.importorskip("scipy")
    from bounded_embodiment_kit.instruments.markerless.mesh import missing_meshes
    from bounded_embodiment_kit.kinematics import URDFModel

    missing = missing_meshes(URDFModel.from_file(URDF))
    if missing:
        pytest.skip(f"the SO-101 meshes are not on disk ({len(missing)} missing, e.g. {missing[0].name}); {FETCH_HINT}")


# ----- every file a block of code opens --------------------------------------------------------------------
class OpenLog:
    def __init__(self):
        self.paths: list[Path] = []

    def add(self, file) -> None:
        if isinstance(file, (str, bytes, os.PathLike)):
            self.paths.append(Path(os.fsdecode(file)).resolve())

    def under(self, root: Path) -> set[Path]:
        root = Path(root).resolve()
        return {p for p in self.paths if p == root or root in p.parents}


_ACTIVE: list[OpenLog] = []
_HOOKED: list[bool] = []


def _audit(event, args):
    if _ACTIVE and event == "open" and args:
        _ACTIVE[-1].add(args[0])


@contextlib.contextmanager
def recording_opens():
    """Log every file opened inside the block: builtins.open, io.open and pathlib.Path.open are patched, and
    an audit hook catches the rest (os.open, and C code that opens through io). Audit hooks cannot be
    removed, so the hook stays installed and idle outside such a block."""
    log = OpenLog()
    if not _HOOKED:
        sys.addaudithook(_audit)
        _HOOKED.append(True)
    real_open, real_path_open = builtins.open, pathlib.Path.open

    def logged_open(file, *a, **k):
        log.add(file)
        return real_open(file, *a, **k)

    def logged_path_open(self, *a, **k):
        log.add(self)
        return real_path_open(self, *a, **k)

    with pytest.MonkeyPatch.context() as mp:
        mp.setattr(builtins, "open", logged_open)
        mp.setattr(io, "open", logged_open)
        mp.setattr(pathlib.Path, "open", logged_path_open)
        _ACTIVE.append(log)
        try:
            yield log
        finally:
            _ACTIVE.pop()


def plant_robot_state(session: Path) -> list[Path]:
    """Files the fitter must never read, where a careless one would find them: harness logs with encoder and
    commanded values, and probe results, inside the session directory."""
    planted = []
    hw = session / "results" / "arm-hw"
    hw.mkdir(parents=True, exist_ok=True)
    state = {"joints_deg": {"shoulder_pan": 12.5, "shoulder_lift": -40.0, "elbow_flex": 25.0},
             "note": "encoder and commanded values: the markerless instrument must never open this"}
    for name in ("goto-start-20261010-101500.json", "nominal-20261010-101600.json", "probe-hold1-20261010.json"):
        (hw / name).write_text(json.dumps(state), encoding="utf-8")
        planted.append(hw / name)
    (hw / "nominal-20261010-101600.csv").write_text("wall_time,shoulder_pan\n1.0,12.5\n", encoding="utf-8")
    planted.append(hw / "nominal-20261010-101600.csv")
    for name in ("probe.json", "probe-hold1.json", "probe_hold2.json"):
        (session / name).write_text(json.dumps(state), encoding="utf-8")
        planted.append(session / name)
    return [p.resolve() for p in planted]


# ----- a small recording written with the recorder's own writers --------------------------------------------
def write_tiny_session(root: Path, *, color=(), depth=(), events=(), intrinsics=None, measurements=None,
                       board=None) -> Path:
    """``color``/``depth``: (device_seq, t_mono_s, oak_session) per frame. Depth frame k is filled with k + 1
    (mm) so a test can tell which frame it read. Colour payloads are a few bytes (not decodable)."""
    from bounded_embodiment_kit.instruments import record

    root = Path(root)
    (root / "oakd").mkdir(parents=True, exist_ok=True)
    stats = {n: record.StreamStats(n, 30) for n in ("color", "depth")}
    cw = record.MjpegWriter(root / record.PATHS["color_stream"], root / record.PATHS["color_index"],
                            record.COLOR_COLUMNS, stats["color"], max_queue=10_000)
    dw = record.DepthChunkWriter(root / record.PATHS["depth_dir"], root / record.PATHS["depth_index"],
                                 stats["depth"], chunk_frames=4, max_queue=10_000)
    cw.start()
    dw.start()
    try:
        for seq, t, sess in color:
            cw.put(((seq, float(t)), b"\xff\xd8 not a real jpeg \xff\xd9", (None, None, float(t) + 0.01, sess)))
        for k, (seq, t, sess) in enumerate(depth):
            dw.put(((seq, float(t)), np.full((4, 6), k + 1, np.uint16), (None, None, float(t) + 0.01, sess)))
    finally:
        cw.finish()
        dw.finish()
    with open(root / record.PATHS["events"], "w", encoding="utf-8") as f:
        f.write(",".join(record.EVENT_COLUMNS) + "\n")
        for k, (t, label) in enumerate(events):
            f.write(f"{k},{t:.6f},{t + 1.7e9:.6f},{label},keyboard\n")
    meta = {"format": record.FORMAT, "status": "finished", "t0_mono_s": 100.0,
            "oakd": {"intrinsics": intrinsics if intrinsics is not None else {}}}
    if board is not None:
        meta["board"] = board
    record.write_json_atomic(root / record.PATHS["meta"], meta)
    if measurements is not None:
        (root / record.PATHS["measurements"]).write_text(measurements, encoding="utf-8")
    return root


# ----- samples.csv ---------------------------------------------------------------------------------------
def read_samples_csv(path: Path) -> list[dict]:
    with open(path, newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def shifts_mm(rows_a: list[dict], rows_b: list[dict]) -> dict[str, np.ndarray]:
    """Per point, the distance (mm) between two runs' estimates, over the rows valid in both."""
    out: dict[str, list[float]] = {}
    assert len(rows_a) == len(rows_b)
    for a, b in zip(rows_a, rows_b, strict=True):
        assert (a["t_s"], a["point"]) == (b["t_s"], b["point"])
        if a["valid"] == "1" and b["valid"] == "1":
            d = np.linalg.norm([float(a[k]) - float(b[k]) for k in ("x_m", "y_m", "z_m")]) * 1000
            out.setdefault(a["point"], []).append(float(d))
    return {k: np.array(v) for k, v in out.items()}
