# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""DRAFT — requires line-by-line human review before use on hardware.

Instrument feeds: the watchdog's live view of the external instrument.

An ``InstrumentFeed`` gives, at any moment, the newest sample of each envelope point (and the
newest VALID one), when the last datagram arrived, and whether the instrument has reported that
its camera moved. It is the watchdog's input, not the scored data: the instrument records its own
full-rate samples (``raw/samples.csv``) separately. The simulated feed writes that file itself,
because in a simulation it is the instrument.

UDP/JSON format (``--instrument udp:PORT`` or ``udp:HOST:PORT``): one UTF-8 JSON object per
datagram, one sample per datagram::

    {"t": 12.345678, "point": "tool", "xyz": [0.281, -0.012, 0.334], "valid": 1}

- ``t``: capture time in seconds on the instrument's own clock (any epoch; monotonic preferred).
- ``point``: an envelope point id. Points the envelope does not declare are ignored.
- ``xyz``: metres, in the envelope frame. Required when ``valid`` is 1.
- ``valid``: 1, or 0 when the instrument looked and could not measure the point (``xyz`` may then
  be omitted or null).
- ``camera_moved``: optional; 1 means the instrument's reference has moved since it locked. It is
  sticky for the rest of the run and may come in a datagram of its own: ``{"t": ..., "camera_moved": 1}``.

Clock. The runner maps instrument time onto its own timebase with ``offset = min(arrival - t)``
over every datagram so far (the plan's clock method). A sample's runner time is therefore its
capture time plus the smallest delivery latency seen, so samples read slightly late by that
minimum latency.
"""
from __future__ import annotations

import csv
import json
import math
import random
import socket
import threading
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol, runtime_checkable

__all__ = [
    "FeedSample",
    "FeedSnapshot",
    "InstrumentFeed",
    "SimFeed",
    "UDPFeed",
    "parse_datagram",
    "parse_instrument",
]


@dataclass(frozen=True)
class FeedSample:
    """One instrument sample on the run's timebase. ``xyz`` is None when ``valid`` is False."""

    t_s: float
    point: str
    xyz: tuple[float, float, float] | None
    valid: bool
    arrival_s: float


@dataclass(frozen=True)
class FeedSnapshot:
    latest: Mapping[str, FeedSample]  # newest sample per point, valid or not
    latest_valid: Mapping[str, FeedSample]  # newest valid sample per point
    last_arrival_s: float | None  # runner time of the last datagram of any kind; None: none yet
    camera_moved: bool
    started_s: float  # runner time the feed started (silence counts from here until a datagram)


@runtime_checkable
class InstrumentFeed(Protocol):
    def start(self) -> None: ...
    def snapshot(self) -> FeedSnapshot: ...
    def close(self) -> None: ...


def _finite(value: object, what: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{what} must be a finite number, not {value!r}")
    return float(value)


def _reject_constant(name: str):
    raise ValueError(f"{name} is not JSON")


def parse_datagram(data: bytes) -> dict:
    """One datagram -> {"t", "point" | None, "xyz" | None, "valid", "camera_moved"}; ValueError if
    it does not follow the format."""
    msg = json.loads(data.decode("utf-8"), parse_constant=_reject_constant)
    if not isinstance(msg, dict):  # ValueError, like every other bad datagram: one except clause
        raise ValueError("a datagram is a JSON object")  # noqa: TRY004
    t = _finite(msg.get("t"), "t")
    moved = msg.get("camera_moved", 0)
    if moved not in (0, 1):  # True/False compare equal to 1/0 and are accepted
        raise ValueError(f"camera_moved must be 0 or 1, not {moved!r}")
    point = msg.get("point")
    if point is None:
        if not moved:
            raise ValueError("a datagram without a point must carry camera_moved: 1")
        return {"t": t, "point": None, "xyz": None, "valid": False, "camera_moved": True}
    if not isinstance(point, str) or not point:
        raise ValueError(f"point must be a non-empty string, not {point!r}")
    valid = msg.get("valid")
    if valid not in (0, 1):
        raise ValueError(f"valid must be 0 or 1, not {valid!r}")
    xyz = None
    if valid:
        raw = msg.get("xyz")
        if not isinstance(raw, list) or len(raw) != 3:
            raise ValueError(f"a valid sample needs xyz: [x, y, z], not {raw!r}")
        xyz = tuple(_finite(v, "xyz") for v in raw)
    return {"t": t, "point": point, "xyz": xyz, "valid": bool(valid), "camera_moved": bool(moved)}


class _Store:
    """The newest samples, behind one lock (written by the feed's thread, read by the watchdog)."""

    def __init__(self, started_s: float):
        self.lock = threading.Lock()
        self.latest: dict[str, FeedSample] = {}
        self.latest_valid: dict[str, FeedSample] = {}
        self.last_arrival: float | None = None
        self.camera_moved = False
        self.started_s = started_s

    def put(self, sample: FeedSample | None, arrival: float, moved: bool) -> None:
        with self.lock:
            self.last_arrival = arrival
            self.camera_moved = self.camera_moved or moved
            if sample is not None:
                self.latest[sample.point] = sample
                if sample.valid:
                    self.latest_valid[sample.point] = sample

    def snapshot(self) -> FeedSnapshot:
        with self.lock:
            return FeedSnapshot(dict(self.latest), dict(self.latest_valid), self.last_arrival,
                                self.camera_moved, self.started_s)


class UDPFeed:
    """The UDP/JSON instrument feed (format in the module docstring)."""

    def __init__(self, port: int, clock: Callable[[], float], host: str = "0.0.0.0",
                 points: set[str] | None = None):
        self.host, self.port, self.clock = host, port, clock
        self.points = points  # envelope point ids; others are ignored
        self._store = _Store(clock())
        self._offset: float | None = None
        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._quit = threading.Event()
        self.received = 0
        self.rejected = 0

    def start(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((self.host, self.port))
        self._sock.settimeout(0.05)
        self._store = _Store(self.clock())
        self._thread = threading.Thread(target=self._run, name="be-feed-udp", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._quit.is_set():
            try:
                data, _ = self._sock.recvfrom(65535)
            except TimeoutError:
                continue
            except OSError:
                if self._quit.is_set():
                    return
                raise
            self.handle(data, self.clock())

    def handle(self, data: bytes, arrival: float) -> None:
        """One datagram that arrived at runner time ``arrival`` (public for tests)."""
        try:
            msg = parse_datagram(data)
        except (ValueError, UnicodeDecodeError):
            self.rejected += 1
            return
        self.received += 1
        # offset = min(arrival - t): the smallest delivery delay seen maps instrument time to ours.
        candidate = arrival - msg["t"]
        self._offset = candidate if self._offset is None else min(self._offset, candidate)
        sample = None
        if msg["point"] is not None and (self.points is None or msg["point"] in self.points):
            sample = FeedSample(msg["t"] + self._offset, msg["point"], msg["xyz"], msg["valid"], arrival)
        self._store.put(sample, arrival, msg["camera_moved"])

    def snapshot(self) -> FeedSnapshot:
        return self._store.snapshot()

    @property
    def bound_port(self) -> int | None:
        """The port actually bound (useful with port 0)."""
        return self._sock.getsockname()[1] if self._sock is not None else None

    def close(self) -> None:
        self._quit.set()
        if self._sock is not None:
            self._sock.close()
        if self._thread is not None:
            self._thread.join(timeout=1.0)


class SimFeed:
    """A simulated instrument: samples of the true point positions with noise, a fixed bias,
    dropouts and lag, at a fixed rate. ``truth()`` returns ``{point: (x, y, z)}``.

    sigma_m  per-axis Gaussian noise of every sample.
    bias_m   a fixed offset of that length along (1, 1, 1): a calibration error.
    dropout  the chance that a sample is invalid.
    lag_s    a sample captured at t becomes visible at t + lag_s.
    """

    def __init__(self, truth: Callable[[], Mapping[str, tuple]], clock: Callable[[], float], *,
                 fps: float = 30.0, sigma_m: float = 0.001, bias_m: float = 0.003,
                 dropout: float = 0.02, lag_s: float = 0.05, seed: int = 0,
                 samples_csv: str | Path | None = None):
        self.truth, self.clock = truth, clock
        self.fps, self.sigma, self.dropout, self.lag = fps, sigma_m, dropout, lag_s
        b = bias_m / math.sqrt(3.0)
        self.bias = (b, b, b)
        self.rng = random.Random(seed)
        self.samples_csv = Path(samples_csv) if samples_csv else None
        self._store = _Store(clock())
        self._pending: list[FeedSample] = []
        self._quit = threading.Event()
        self._thread: threading.Thread | None = None
        self._moved = False

    def params(self) -> dict:
        return {"fps": self.fps, "sigma_m": self.sigma, "bias_m": list(self.bias),
                "dropout": self.dropout, "lag_s": self.lag}

    def start(self) -> None:
        self._store = _Store(self.clock())
        self._thread = threading.Thread(target=self._run, name="be-feed-sim", daemon=True)
        self._thread.start()

    def set_camera_moved(self) -> None:
        """Test hook: the simulated instrument reports that its camera moved."""
        self._moved = True

    def _capture(self, t: float) -> list[FeedSample]:
        out = []
        for point, xyz in self.truth().items():
            if self.rng.random() < self.dropout:
                out.append(FeedSample(t, point, None, False, t + self.lag))
                continue
            noisy = tuple(float(xyz[i]) + self.bias[i] + self.rng.gauss(0.0, self.sigma) for i in range(3))
            out.append(FeedSample(t, point, noisy, True, t + self.lag))
        return out

    def _run(self) -> None:
        writer, fh = None, None
        if self.samples_csv is not None:
            fh = self.samples_csv.open("w", newline="", encoding="utf-8")
            writer = csv.writer(fh, lineterminator="\n")
            writer.writerow(["t_s", "point", "x_m", "y_m", "z_m", "valid", "residual_m"])
        try:
            period = 1.0 / self.fps
            next_t = self.clock()
            while not self._quit.is_set():
                now = self.clock()
                if now >= next_t:
                    batch = self._capture(now)
                    self._pending.extend(batch)
                    if writer is not None:
                        for s in batch:
                            cells = ([repr(v) for v in s.xyz] if s.valid else ["", "", ""])
                            writer.writerow([f"{s.t_s:.6f}", s.point, *cells, int(s.valid), ""])
                    next_t += period
                due = [s for s in self._pending if s.arrival_s <= now]
                if due:
                    self._pending = [s for s in self._pending if s.arrival_s > now]
                    for s in due:
                        self._store.put(s, s.arrival_s, self._moved)
                elif self._moved:
                    self._store.put(None, now, True)
                self._quit.wait(0.002)
        finally:
            if fh is not None:
                fh.close()

    def snapshot(self) -> FeedSnapshot:
        return self._store.snapshot()

    def close(self) -> None:
        self._quit.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)


def parse_instrument(text: str) -> tuple[str, str, int]:
    """``udp:PORT``, ``udp:HOST:PORT``, ``sim`` or ``none`` -> (kind, host, port)."""
    if text in ("sim", "none"):
        return text, "", 0
    parts = text.split(":")
    try:
        if parts[0] == "udp" and len(parts) == 2:
            return "udp", "0.0.0.0", int(parts[1])
        if parts[0] == "udp" and len(parts) == 3:
            return "udp", parts[1], int(parts[2])
    except ValueError:
        pass
    raise ValueError(f"--instrument {text!r}: expected udp:PORT, udp:HOST:PORT, sim or none")
