# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""Read a recording made by `be record` (format ``be-camera-recording/1``).

The recorder (:mod:`bounded_embodiment_kit.instruments.record`) runs as a single copied file on the Pi,
so it cannot import this module; this module imports the recorder's own readers instead (``_streams``,
``_Frames``, ``_load_events``, ``_read_rows``) so that the two never disagree about the files.

What a session holds (see record.py for the writers)::

    meta.json                      settings, OAK-D intrinsics (1280x720 and 640x360) and distortion
    oakd/color.mjpeg + index       colour 1280x720 JPEGs
    oakd/depth/*.npz + index       depth 640x360 uint16 mm aligned to colour, 0 = no depth
    wrist/wrist.mjpeg + index      the wrist camera
    events.csv, clock.csv          labels typed during the session; (time.time, time.monotonic) pairs
    measurements.txt               tape-measure numbers, ``key = value  # comment``, millimetres

Every ``t_mono_s`` is the recording host's CLOCK_MONOTONIC. Nothing here reads robot state.
"""
from __future__ import annotations

import json
import re
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from . import record

__all__ = ["DepthFrame", "Session", "SessionError", "parse_measurements"]


class SessionError(ValueError):
    """The directory is not a usable recording, or a requested part of it is missing."""


def parse_measurements(text: str) -> dict[str, float | str | None]:
    """``key = value`` lines; ``#`` starts a comment; numbers become floats, blanks None, the rest text."""
    out: dict[str, float | str | None] = {}
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or "=" not in line:
            continue
        key, value = (s.strip() for s in line.split("=", 1))
        if not key:
            continue
        value = value.strip().strip('"').strip("'").strip()
        if value == "":
            out[key] = None
            continue
        try:
            out[key] = float(value)
        except ValueError:
            out[key] = value
    return out


@dataclass(frozen=True)
class DepthFrame:
    index: int                 # row in oakd/depth_index.csv
    t_mono_s: float
    device_seq: int
    depth_mm: np.ndarray       # (360, 640) uint16, 0 = no depth
    color_index: int | None    # the paired colour frame (row in oakd/color_index.csv), if any


class Session:
    """Random and sequential access to one recording. Reads files lazily; ``close()`` when done."""

    def __init__(self, path: str | Path):
        self.path = Path(path).expanduser()
        meta_path = self.path / record.PATHS["meta"]
        if not meta_path.is_file():
            raise SessionError(f"{self.path} has no meta.json: not a recording")
        self.meta = json.loads(meta_path.read_text(encoding="utf-8"))
        self._streams = None
        self._events = None
        self._frames = None
        self._cv2 = None
        self._pairing = None

    # ----- indices ------------------------------------------------------------------------------
    def _load_streams(self):
        if self._streams is None:
            self._streams = record._streams(self.path)
        return self._streams

    @property
    def color(self) -> dict:
        return self._load_streams()[0]

    @property
    def depth(self) -> dict:
        return self._load_streams()[1]

    @property
    def wrist(self) -> dict:
        return self._load_streams()[2]

    @property
    def events(self) -> list[dict]:
        if self._events is None:
            self._events = record._load_events(self.path)
        return self._events

    @property
    def t0(self) -> float:
        """The recording's start on its monotonic clock (meta.json ``t0_mono_s``; else the first frame)."""
        t0 = self.meta.get("t0_mono_s")
        if t0 is not None:
            return float(t0)
        ts = [x["t"][0] for x in (self.color, self.depth) if x["t"].size]
        return float(min(ts)) if ts else 0.0

    def clock(self) -> list[dict]:
        return record._read_rows(self.path / record.PATHS["clock"])

    def measurements(self) -> dict[str, float | str | None]:
        p = self.path / record.PATHS["measurements"]
        if not p.is_file():
            return {}
        return parse_measurements(p.read_text(encoding="utf-8"))

    def board_spec(self) -> dict:
        """``<session>/board.json`` if present, else the spec the recorder wrote in meta.json, else the
        recorder's default board."""
        p = self.path / "board.json"
        if p.is_file():
            spec = json.loads(p.read_text(encoding="utf-8"))
            return {**record.BOARD_SPEC, **{k: spec[k] for k in record.BOARD_SPEC if k in spec},
                    "source": str(p)}
        if isinstance(self.meta.get("board"), dict):
            return {**record.BOARD_SPEC, **self.meta["board"], "source": "meta.json"}
        return {**record.BOARD_SPEC, "source": "record.BOARD_SPEC"}

    # ----- intrinsics -----------------------------------------------------------------------------
    def intrinsics(self, size: str = "640x360") -> tuple[np.ndarray, np.ndarray]:
        """(K 3x3, distortion) for ``"1280x720"`` (colour) or ``"640x360"`` (colour scaled; the depth is
        aligned to it). Distortion is DepthAI's CAM_A coefficients (up to 14), zeros if absent."""
        intr = ((self.meta.get("oakd") or {}).get("intrinsics") or {})
        key = {"1280x720": "rgb_1280x720", "640x360": "rgb_640x360"}[size]
        K = intr.get(key)
        if K is None and size == "640x360" and intr.get("rgb_1280x720") is not None:
            K = (np.asarray(intr["rgb_1280x720"], float) * np.array([[0.5], [0.5], [1.0]])).tolist()
        if K is None:
            raise SessionError(f"meta.json has no OAK-D intrinsics for {size}")
        K = np.asarray(K, dtype=np.float64)
        if K.shape != (3, 3) or K[0, 0] <= 0 or K[1, 1] <= 0:
            raise SessionError(f"meta.json intrinsics {key} are not a usable 3x3 matrix")
        dist = np.asarray(intr.get("rgb_distortion") or [], dtype=np.float64).ravel()
        if dist.size not in (4, 5, 8, 12, 14) or not np.isfinite(dist).all():
            dist = np.zeros(5)
        return K, dist

    # ----- events and time ------------------------------------------------------------------------
    def event_time(self, label: str, *, after: float | None = None, first: bool = False) -> float | None:
        """Time of the last (or first) event with this label, optionally only those after ``after``."""
        ts = [float(e["t_mono_s"]) for e in self.events if e["label"] == label]
        if after is not None:
            ts = [t for t in ts if t > after]
        if not ts:
            return None
        return ts[0] if first else ts[-1]

    def next_event_after(self, t: float) -> tuple[float, str] | None:
        later = sorted((float(e["t_mono_s"]), e["label"]) for e in self.events if float(e["t_mono_s"]) > t)
        return later[0] if later else None

    def resolve_time(self, spec: str | float | None) -> float | None:
        """A time on the session clock from ``LABEL``, ``LABEL+S``/``LABEL-S`` (the last such event, offset
        by S seconds) or a number of seconds after the recording's start."""
        if spec is None or spec == "":
            return None
        if isinstance(spec, (int, float)):
            return self.t0 + float(spec)
        text = str(spec).strip()
        try:
            return self.t0 + float(text)
        except ValueError:
            pass
        exact = self.event_time(text)
        if exact is not None:
            return exact
        m = re.fullmatch(r"(.+?)([+-]\d+(?:\.\d*)?)?", text)
        label, off = m.group(1), float(m.group(2) or 0.0)
        t = self.event_time(label)
        if t is None:
            known = sorted({e["label"] for e in self.events})
            raise SessionError(f"no event {label!r} in events.csv (events: {', '.join(known) or 'none'})")
        return t + off

    # ----- frames ----------------------------------------------------------------------------------
    def _cv(self):
        if self._cv2 is None:
            self._cv2 = record.import_cv2()
        return self._cv2

    def _frame_reader(self):
        if self._frames is None:
            self._frames = record._Frames(self.path, self._cv2)
        return self._frames

    def depth_frame(self, i: int) -> np.ndarray:
        d = self.depth
        arr = self._frame_reader().depth(d["chunk"][i], d["k"][i])
        if arr is None:
            raise SessionError(f"depth frame {i} ({d['chunk'][i]}[{int(d['k'][i])}]) cannot be read")
        return arr

    def color_frame(self, i: int) -> np.ndarray | None:
        """Colour frame ``i`` as a BGR array (needs OpenCV), or None if it cannot be decoded."""
        cv2 = self._cv()
        fr = self._frame_reader()
        fr.cv2 = cv2
        c = self.color
        return fr.jpeg(record.PATHS["color_stream"], c["off"][i], c["len"][i])

    def color_indices(self, t_from: float, t_to: float) -> np.ndarray:
        t = self.color["t"]
        return np.where((t >= t_from) & (t <= t_to))[0]

    def depth_indices(self, t_from: float | None = None, t_to: float | None = None, every: int = 1
                      ) -> np.ndarray:
        t = self.depth["t"]
        sel = np.isfinite(t)
        if t_from is not None:
            sel &= t >= t_from
        if t_to is not None:
            sel &= t <= t_to
        idx = np.where(sel)[0]
        return idx[:: max(1, int(every))]

    def _pairs(self):
        """Depth row -> colour row: same (oak_session, device_seq) when the timestamps agree to half a
        colour frame, else the colour frame nearest in t_mono_s."""
        if self._pairing is None:
            c, d = self.color, self.depth
            out = np.full(d["t"].size, -1, dtype=np.int64)
            how = np.zeros(d["t"].size, dtype=np.int8)        # 1 by sequence, 2 by time
            if c["t"].size and d["t"].size:
                ct = c["t"]
                dtc = np.diff(ct[np.isfinite(ct)])
                tol = 0.5 * float(np.median(dtc)) if dtc.size else 0.017
                csess = np.nan_to_num(c["sess"], nan=0).astype(np.int64)
                dsess = np.nan_to_num(d["sess"], nan=0).astype(np.int64)
                lookup = {(int(s), int(q)): i for i, (s, q) in enumerate(zip(csess, c["seq"], strict=True))
                          if np.isfinite(q)}
                order = np.argsort(ct, kind="stable")
                cts = ct[order]
                for j in range(d["t"].size):
                    q = d["seq"][j]
                    i = lookup.get((int(dsess[j]), int(q))) if np.isfinite(q) else None
                    if i is not None and abs(ct[i] - d["t"][j]) <= tol:
                        out[j], how[j] = i, 1
                    else:
                        k = record._nearest(cts, float(d["t"][j]))
                        out[j], how[j] = int(order[k]), 2
            self._pairing = (out, how)
        return self._pairing

    def color_for_depth(self, i: int) -> int | None:
        out, _ = self._pairs()
        j = int(out[i]) if i < out.size else -1
        return None if j < 0 else j

    def pairing_summary(self) -> dict:
        out, how = self._pairs()
        return {"by_device_seq": int((how == 1).sum()), "by_nearest_time": int((how == 2).sum()),
                "unpaired": int((out < 0).sum())}

    def iter_depth(self, t_from: float | None = None, t_to: float | None = None, every: int = 1
                   ) -> Iterator[DepthFrame]:
        d = self.depth
        for i in self.depth_indices(t_from, t_to, every):
            arr = self.depth_frame(int(i))
            seq = d["seq"][i]
            yield DepthFrame(int(i), float(d["t"][i]), int(seq) if np.isfinite(seq) else -1, arr,
                             self.color_for_depth(int(i)))

    def close(self):
        if self._frames is not None:
            self._frames.close()
            self._frames = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
