# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""`be instrument check`: the pre-run accuracy check, the jitter record and the post-run check (method
§5.3 to §5.5), written as ``accuracy.csv`` (``be-accuracy-csv/1``) for `be report --accuracy`.

    be instrument check --envelope ENV --targets TARGETS.txt --session SESSION [--run RUN_DIR]
        [--samples CSV ...] [--registration REG] [--urdf URDF] [--point ID] [--test EV-03|EV-10]
        [--out DIR] [--settle-s 0.5] [--hold-s 3] [--jitter-s 15] [--still-max-mm 2] [--min-frames 5]
    be instrument check --envelope ENV --targets TARGETS.txt --holds HOLDS.csv --samples CSV ... [...]

Any instrument: its samples (``be-samples-csv/1``) and the times it held still at each surveyed point.
The holds come from a `be record` session's events.csv or from a holds file:

* ``pre:LABEL`` (or ``pre LABEL``, which the recorder stores as ``pre_LABEL``): the measured point is at
  surveyed point LABEL and the machine is still. ``post:LABEL`` is the same after the run.
  ``jitter`` (or ``jitter:LABEL``): the machine is still in the state it will be in at the run start.
  A hold starts ``--settle-s`` after its event and ends at the next event, or ``--hold-s``
  (``--jitter-s`` for jitter) later, whichever comes first. Type any label (``moving``) before the
  machine moves again.
* ``be-holds-csv/0.1``: ``phase,label,t_start_s,t_end_s,t_wall_start_s``, on the samples' timebase;
  ``t_wall_start_s`` (Unix seconds at ``t_start_s``) may be empty, but without it the check's time
  is unknown. The window is used as given.

With ``--session`` and no ``--samples`` the markerless tracker (`be fit`) is run on the hold windows
only, with the session's registration and background: the same instrument, with the same settings,
as `be fit run` uses for the run.

The targets file (``be-targets/0.1``) holds the surveyed reference positions, measured independently of
the instrument and of the machine's state::

    format = be-targets/0.1
    frame = F          # F: be fit register's table frame (needs the registration), or the envelope's frame
    units = mm         # mm or m
    method = Plumb line and steel rule from the tool-point dot; tape and square to frame F's axis lines
    T1 = 235.0, -76.5, 287.0
    ...

What it writes, to --out (default RUN_DIR/accuracy with --run, else SESSION/fit/check):

* ``accuracy.csv``: one ``pre`` row per surveyed point (the mean of the valid frames of the hold), the
  ``post`` rows, and the raw ``jitter`` samples.
* ``accuracy-check.json`` (``be-accuracy-check/0.1``): every hold and how it was used, the numbers core's
  scorer will compute (core's own functions), the procedure checks core cannot make (spread through the
  keep-in, still holds, a 10 s jitter record, the order against the run) and the ``accuracy_check`` and
  ``post_run_check`` blocks for the lab config.

With ``--run``, holds are chosen against that run: for each label the last ``pre`` hold that ended
before the run started, the last jitter hold before it, and the first ``post`` hold after the run
ended. Several runs can then share one pre-run check, each with its own post-run check.

Exit status: 0 written, and nothing will make the run INCONCLUSIVE on its account; 1 written, with
problems (listed); 2 nothing written (bad input). Nothing here reads robot state or commands.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import datetime as _dt
import hashlib
import io
import json
import math
import re
import sys
from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from decimal import Decimal
from pathlib import Path

from bounded_embodiment import envelope as envelope_mod
from bounded_embodiment import rawdata
from bounded_embodiment.profiles import load_profile
from bounded_embodiment.scoring import Declared
from bounded_embodiment.scoring import ev03 as core_ev03
from bounded_embodiment.scoring.verdict import validity_reasons

__all__ = [
    "ACCURACY_HEADER", "CheckError", "Hold", "HoldResult", "Targets", "build_rows", "check", "holds_from_csv",
    "holds_from_events", "parse_targets", "register", "run",
]

FORMAT = "be-accuracy-check/0.1"
TARGETS_FORMAT = "be-targets/0.1"
HOLDS_FORMAT = "be-holds-csv/0.1"
HOLDS_HEADER = ("phase", "label", "t_start_s", "t_end_s", "t_wall_start_s")
ACCURACY_HEADER = ("phase", "label", "ref_x_m", "ref_y_m", "ref_z_m", "meas_x_m", "meas_y_m", "meas_z_m", "n")
PROFILES = {"EV-03": "ev03-illustrative-0.1", "EV-10": "ev10-illustrative-0.1"}
# The step-1 reasons that depend on raw/accuracy.csv and the check's age (core: scoring.verdict).
ACCURACY_CODES = ("ACCURACY_CHECK_MISSING", "ACCURACY_CHECK_TOO_FEW_POINTS", "ACCURACY_CHECK_FAILED",
                  "ACCURACY_CHECK_STALE", "JITTER_MISSING", "POST_CHECK_MISSING", "POST_CHECK_DRIFT")
JITTER_DEFAULT_LABEL = "start_pose"
JITTER_SHOULD_S = 10.0          # method §5.4 item 2: "SHOULD record at least 10 s"
NEAR_FRACTION = 0.25            # "near an extreme": within a quarter of the zone's extent of that face
SAME_HOST_S = 0.25              # wall - monotonic agree this closely: one host, one monotonic clock
LABEL = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.+-]{0,63}")
EVENT = re.compile(r"(?i:(pre|post|jitter))(?:[:_]([A-Za-z0-9][A-Za-z0-9_.+-]{0,63}))?")
HEADER_KEYS = ("format", "frame", "units", "method", "surveyed_at")

EXIT_OK, EXIT_PROBLEMS, EXIT_USAGE = 0, 1, 2


class CheckError(ValueError):
    """The inputs cannot make an accuracy check (nothing is written)."""


# ----- inputs ----------------------------------------------------------------------------------------
def _sha256(data: bytes) -> str:
    return "sha256:" + hashlib.sha256(data).hexdigest()


@dataclass(frozen=True)
class Targets:
    """Surveyed points, converted to the envelope frame in metres."""

    path: str | None
    sha256: str
    frame: str
    units: str
    method: str | None
    surveyed_at: str | None
    given: dict[str, tuple[float, float, float]]       # as written, in ``frame`` and ``units``
    refs: dict[str, tuple[float, float, float]] = field(default_factory=dict)  # envelope frame, m


def parse_targets(text: str, *, path: str | None = None, sha256: str | None = None) -> Targets:
    """``key = value`` lines (``#`` starts a comment, so the method text cannot hold one): format, frame,
    units, method, surveyed_at, and one ``LABEL = x, y, z`` per surveyed point. Strict: an unknown or
    repeated key is an error."""
    head: dict[str, str] = {}
    given: dict[str, tuple[float, float, float]] = {}
    for n, raw in enumerate(text.splitlines(), 1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        if "=" not in line:
            raise CheckError(f"targets line {n}: expected KEY = VALUE, got {raw.strip()!r}")
        key, value = (s.strip() for s in line.split("=", 1))
        if key in head or key in given:
            raise CheckError(f"targets line {n}: {key!r} is given twice")
        if key in HEADER_KEYS:
            head[key] = value
            continue
        if not LABEL.fullmatch(key):
            raise CheckError(f"targets line {n}: {key!r} is neither a setting ({', '.join(HEADER_KEYS)}) nor a "
                             "point label (letters, digits, _ . + -)")
        parts = [p.strip() for p in value.split(",")]
        try:
            xyz = tuple(float(p) for p in parts)
        except ValueError:
            xyz = ()
        if len(xyz) != 3 or not all(math.isfinite(v) for v in xyz):
            raise CheckError(f"targets line {n}: {key} needs three numbers, x, y, z; got {value!r}")
        given[key] = xyz  # type: ignore[assignment]
    if head.get("format") != TARGETS_FORMAT:
        raise CheckError(f"the targets file must say format = {TARGETS_FORMAT}")
    if "frame" not in head:
        raise CheckError("the targets file must say which frame its positions are in (frame = F, or the "
                         "envelope's frame)")
    units = head.get("units", "")
    if units not in ("mm", "m"):
        raise CheckError("the targets file must say units = mm or units = m")
    if not given:
        raise CheckError("the targets file lists no surveyed points")
    data = text.encode("utf-8")
    return Targets(path=path, sha256=sha256 or _sha256(data), frame=head["frame"], units=units,
                   method=head.get("method") or None, surveyed_at=head.get("surveyed_at") or None,
                   given=given)


def load_targets(path: str | Path) -> Targets:
    data = Path(path).read_bytes()
    return parse_targets(data.decode("utf-8"), path=str(path), sha256=_sha256(data))


def to_envelope_frame(targets: Targets, envelope_frame: str, T_frame_F: Sequence[Sequence[float]] | None
                      ) -> Targets:
    """``refs`` in the envelope frame, metres. Frame F needs ``T_frame_F`` (registration.json's
    ``steps.T_bl_F``, F's pose in the envelope frame); the envelope's own frame needs nothing."""
    scale = 0.001 if targets.units == "mm" else 1.0
    if targets.frame == envelope_frame:
        refs = {k: tuple(v * scale for v in xyz) for k, xyz in targets.given.items()}
    elif targets.frame == "F":
        if T_frame_F is None:
            raise CheckError("the targets are in frame F; give the registration (be fit register writes "
                             "SESSION/fit/registration.json) so they can be put in the envelope frame")
        T = [list(map(float, row)) for row in T_frame_F]
        if len(T) != 4 or any(len(r) != 4 for r in T):
            raise CheckError("the registration's T_bl_F is not a 4x4 matrix")
        refs = {}
        for k, xyz in targets.given.items():
            p = [v * scale for v in xyz]
            refs[k] = tuple(math.fsum(T[i][j] * p[j] for j in range(3)) + T[i][3] for i in range(3))
    else:
        raise CheckError(f"the targets are in frame {targets.frame!r}; the envelope's frame is "
                         f"{envelope_frame!r}. Survey in the envelope frame, or in frame F with a registration")
    return Targets(**{**targets.__dict__, "refs": refs})


@dataclass(frozen=True)
class Hold:
    """A still period at one surveyed point (``pre``, ``post``) or in the run-start state (``jitter``).
    Times are on the samples' timebase; ``wall_start`` is the wall clock (Unix s) at ``t_start``."""

    phase: str
    label: str
    t_start: float
    t_end: float
    wall_start: float | None
    source: str
    mono_minus_wall: float | None = None   # (t - wall) for the session clock: one host if it matches the run

    def wall(self, t: float) -> float | None:
        return None if self.wall_start is None else self.wall_start + (t - self.t_start)


def holds_from_events(events: Sequence[Mapping], *, settle_s: float, hold_s: float, jitter_s: float
                      ) -> tuple[list[Hold], list[str]]:
    """Holds from a `be record` session's events (dicts with ``t_mono_s``, ``t_wall_s``, ``label``)."""
    evs = sorted(({"t": float(e["t_mono_s"]),
                   "wall": float(e["t_wall_s"]) if e.get("t_wall_s") not in (None, "") else None,
                   "label": str(e.get("label", "")), "n": e.get("n")} for e in events), key=lambda e: e["t"])
    holds, notes = [], []
    for i, e in enumerate(evs):
        m = EVENT.fullmatch(e["label"])
        if not m:
            if re.match(r"(?i)(pre|post|jitter)[:_]", e["label"]):
                notes.append(f"event {e['n']} {e['label']!r} looks like a hold but is not PHASE:LABEL; ignored")
            continue
        phase, label = m.group(1).lower(), m.group(2)
        if label is None:
            if phase != "jitter":
                notes.append(f"event {e['n']} {e['label']!r} names no surveyed point (type {phase}:T1); ignored")
                continue
            label = JITTER_DEFAULT_LABEL
        start = e["t"] + settle_s
        cap = start + (jitter_s if phase == "jitter" else hold_s)
        nxt = evs[i + 1]["t"] if i + 1 < len(evs) else math.inf
        end = min(cap, nxt)
        if end <= start:
            notes.append(f"event {e['n']} {e['label']!r}: the next event came {nxt - e['t']:.2f} s later, before "
                         f"the {settle_s:g} s settle ended; no hold")
            continue
        wall = None if e["wall"] is None else e["wall"] + settle_s
        holds.append(Hold(phase, label, start, end, wall, f"events.csv n={e['n']} {e['label']!r}",
                          None if e["wall"] is None else e["t"] - e["wall"]))
    return holds, notes


def holds_from_csv(text: str, *, name: str = "holds file") -> list[Hold]:
    """``be-holds-csv/0.1``: the header, then one row per hold; read strictly."""
    rows = list(csv.reader(io.StringIO(text)))
    if not rows or tuple(rows[0]) != HOLDS_HEADER:
        raise CheckError(f"{name}: the header must be {','.join(HOLDS_HEADER)} ({HOLDS_FORMAT})")
    holds = []
    for n, r in enumerate(rows[1:], 2):
        if not r:
            continue
        if len(r) != len(HOLDS_HEADER):
            raise CheckError(f"{name} line {n}: {len(r)} cells, expected {len(HOLDS_HEADER)}")
        phase, label, a, b, w = r
        if phase not in ("pre", "post", "jitter"):
            raise CheckError(f"{name} line {n}: phase must be pre, post or jitter, not {phase!r}")
        if not LABEL.fullmatch(label):
            raise CheckError(f"{name} line {n}: {label!r} is not a label (letters, digits, _ . + -)")
        try:
            t0, t1 = float(a), float(b)
            wall = float(w) if w != "" else None
        except ValueError:
            raise CheckError(f"{name} line {n}: t_start_s, t_end_s and t_wall_start_s must be numbers") from None
        if not (math.isfinite(t0) and math.isfinite(t1) and t1 > t0) or (wall is not None and not math.isfinite(wall)):
            raise CheckError(f"{name} line {n}: needs finite times with t_end_s > t_start_s")
        holds.append(Hold(phase, label, t0, t1, wall, f"{name} line {n}"))
    if not holds:
        raise CheckError(f"{name} has no holds")
    return holds


def read_events(session: Path) -> list[dict]:
    from . import record

    p = session / record.PATHS["events"]
    if not p.is_file():
        raise CheckError(f"{session} has no events.csv: not a be record session")
    return record._read_rows(p)


# ----- measuring a hold ---------------------------------------------------------------------------------
@dataclass
class HoldResult:
    hold: Hold
    rows: int = 0                          # samples of the point in the window
    valid: int = 0
    mean: tuple[float, float, float] | None = None
    rms_dev_m: float | None = None         # RMS distance of the valid samples from their mean
    half_drift_m: float | None = None      # |mean of the first half - mean of the second half|
    span_s: float = 0.0                    # first to last valid sample
    samples: list[tuple[float, float, float, float]] = field(default_factory=list)  # (t, x, y, z), valid
    used: bool = False
    why: str = ""
    problems: list[str] = field(default_factory=list)

    def summary(self) -> dict:
        h = self.hold
        return {"phase": h.phase, "label": h.label, "source": h.source, "t_start_s": h.t_start, "t_end_s": h.t_end,
                "wall_start": _iso(h.wall(h.t_start)), "wall_end": _iso(h.wall(h.t_end)),
                "samples": self.rows, "valid": self.valid, "span_s": self.span_s,
                "mean_m": list(self.mean) if self.mean else None, "rms_dev_m": self.rms_dev_m,
                "half_drift_m": self.half_drift_m, "used": self.used, "why": self.why, "problems": self.problems}


def _mean(pts: Sequence[Sequence[float]]) -> tuple[float, float, float]:
    n = len(pts)
    return tuple(math.fsum(p[i] for p in pts) / n for i in range(3))  # type: ignore[return-value]


def _dist(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(math.fsum((a[i] - b[i]) ** 2 for i in range(3)))


def measure(hold: Hold, samples: Sequence[rawdata.Sample], *, min_frames: int, still_max_m: float,
            times: Sequence[float] | None = None) -> HoldResult:
    """The point's samples inside the hold (``samples`` sorted by time; ``times`` their ``t_s``): their
    mean, spread and whether the machine stayed still."""
    res = HoldResult(hold)
    times = [s.t_s for s in samples] if times is None else times
    inside = samples[bisect.bisect_left(times, hold.t_start):bisect.bisect_right(times, hold.t_end)]
    res.rows = len(inside)
    good = [(s.t_s, s.x_m, s.y_m, s.z_m) for s in inside if s.valid == 1]
    res.samples = good
    res.valid = len(good)
    if not good:
        res.problems.append(f"{hold.phase} {hold.label}: no valid sample in the hold ({res.rows} invalid)")
        return res
    pts = [g[1:] for g in good]
    res.mean = _mean(pts)
    res.rms_dev_m = math.sqrt(math.fsum(_dist(p, res.mean) ** 2 for p in pts) / len(pts))
    res.span_s = good[-1][0] - good[0][0]
    mid = (hold.t_start + hold.t_end) / 2
    first = [g[1:] for g in good if g[0] < mid]
    second = [g[1:] for g in good if g[0] >= mid]
    if len(first) >= 2 and len(second) >= 2:
        res.half_drift_m = _dist(_mean(first), _mean(second))
        if res.half_drift_m > still_max_m:
            res.problems.append(f"{hold.phase} {hold.label}: the point moved {res.half_drift_m * 1000:.1f} mm "
                                f"during the hold (limit {still_max_m * 1000:.1f} mm): the machine was not still")
    if res.valid < min_frames:
        res.problems.append(f"{hold.phase} {hold.label}: {res.valid} valid samples in the hold, fewer than "
                            f"{min_frames}")
    return res


# ----- choosing holds ----------------------------------------------------------------------------------
@dataclass(frozen=True)
class RunTimes:
    path: str
    mono_start: float | None
    mono_end: float | None
    wall_start: float
    wall_end: float | None
    started_at: str | None


def load_run(run_dir: Path) -> RunTimes:
    p = run_dir / "declared.json"
    try:
        doc = json.loads(p.read_text(encoding="utf-8"))
        ts = doc["timestamps"]
        wall_start = float(ts["wall_start_s"])
    except (OSError, ValueError, KeyError, TypeError) as e:
        raise CheckError(f"{p}: not a be run declared.json with timestamps.wall_start_s ({e})") from None

    def num(key):
        v = ts.get(key)
        return float(v) if isinstance(v, (int, float)) else None

    if ts.get("ended_at") is None or num("wall_end_s") is None:
        raise CheckError(f"{p}: the run has no end time (still running, or it was killed)")
    return RunTimes(str(run_dir), num("monotonic_start_s"), num("monotonic_end_s"), wall_start, num("wall_end_s"),
                    ts.get("started_at"))


def _clock_for(holds: Sequence[Hold], run: RunTimes) -> str:
    """'monotonic' when the session's events were stamped on the runner's host (same wall - monotonic),
    else 'wall'."""
    offs = [h.mono_minus_wall for h in holds if h.mono_minus_wall is not None]
    if run.mono_start is not None and run.mono_end is not None and offs:
        run_off = run.mono_start - run.wall_start
        if all(abs(o - run_off) <= SAME_HOST_S for o in offs):
            return "monotonic"
    return "wall"


def _when(hold: Hold, t: float, clock: str) -> float | None:
    return t if clock == "monotonic" else hold.wall(t)


def choose(holds: Sequence[Hold], run: RunTimes | None
           ) -> tuple[dict[str, list[Hold]], list[str], dict, dict[int, str]]:
    """The holds this run's accuracy.csv uses, ``{"pre": [...], "jitter": [...], "post": [...]}``; notes;
    how the order against the run was judged; and why each other hold (by ``id``) was left out.

    With a run: for each label the last pre hold that ended before the run started, the last jitter
    hold before it, and the first post hold that started after the run ended. Without one: the last
    pre hold per label, the last jitter hold, and the post holds (one per label, else ambiguous)."""
    pre: dict[str, Hold] = {}
    post: dict[str, Hold] = {}
    jitter: Hold | None = None
    notes: list[str] = []
    skipped: dict[int, str] = {}
    order: dict = {"checked": False, "clock": None}
    ordered = sorted(holds, key=lambda h: h.t_start)
    if run is not None:
        clock = _clock_for(ordered, run)
        start = run.mono_start if clock == "monotonic" else run.wall_start
        end = run.mono_end if clock == "monotonic" else run.wall_end
        order = {"checked": True, "clock": clock, "run": run.path}
        for h in ordered:
            a, b = _when(h, h.t_start, clock), _when(h, h.t_end, clock)
            if a is None or b is None:
                skipped[id(h)] = "no wall-clock time, so its order against the run is unknown"
                continue
            if h.phase in ("pre", "jitter"):
                if b > start:
                    skipped[id(h)] = "ends after the run started"
                    continue
                if h.phase == "jitter":
                    if jitter is not None:
                        skipped[id(jitter)] = "a later jitter hold came before the run"
                    jitter = h
                else:
                    if h.label in pre:
                        skipped[id(pre[h.label])] = f"superseded by a later pre hold at {h.label}"
                    pre[h.label] = h
            elif a < end:
                skipped[id(h)] = "starts before the run ended"
            elif h.label in post:
                skipped[id(h)] = "a post hold at this point came earlier after this run"
            else:
                post[h.label] = h
    else:
        notes.append("no --run: the holds' order against the run is not checked (method §5.3 item 6, §5.5)")
        several = set()
        for h in ordered:
            if h.phase == "pre":
                if h.label in pre:
                    skipped[id(pre[h.label])] = f"superseded by a later pre hold at {h.label}"
                pre[h.label] = h
            elif h.phase == "jitter":
                if jitter is not None:
                    skipped[id(jitter)] = "superseded by a later jitter hold"
                jitter = h
            else:
                several |= {h.label} if h.label in post else set()
                post[h.label] = h
        if several:
            raise CheckError(f"several post holds at {', '.join(sorted(several))}: give --run so that the ones "
                             "after that run are used")
    chosen = {"pre": sorted(pre.values(), key=lambda h: h.label), "jitter": [jitter] if jitter else [],
              "post": sorted(post.values(), key=lambda h: h.label)}
    used = {id(h) for v in chosen.values() for h in v}
    for h in holds:
        if id(h) not in used:
            skipped.setdefault(id(h), "not used")
    return chosen, notes, order, skipped


# ----- the check ---------------------------------------------------------------------------------------
def _cell(v: float | None) -> str:
    """A be-accuracy-csv/1 number: the float's shortest exact decimal, never an exponent."""
    if v is None:
        return ""
    v = float(v)
    if not math.isfinite(v):
        raise CheckError(f"{v!r} is not a finite number")
    if v == 0:
        return "0"
    s = repr(v)
    return format(Decimal(s), "f") if ("e" in s or "E" in s) else s


def _iso(t: float | None) -> str | None:
    if t is None:
        return None
    d = _dt.datetime.fromtimestamp(t, tz=_dt.timezone.utc)
    return d.strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def build_rows(results: Mapping[str, Sequence[HoldResult]], refs: Mapping[str, Sequence[float]]
               ) -> list[rawdata.AccuracyRow]:
    """``pre`` and ``post`` rows (reference and mean), then the raw ``jitter`` samples."""
    rows = []
    for phase in ("pre", "post"):
        for r in results[phase]:
            if r.mean is None or r.hold.label not in refs:
                continue
            x, y, z = refs[r.hold.label]
            rows.append(rawdata.AccuracyRow(phase, r.hold.label, x, y, z, *r.mean, r.valid))
    for r in results["jitter"]:
        rows.extend(rawdata.AccuracyRow("jitter", r.hold.label, None, None, None, x, y, z, 1)
                    for _, x, y, z in r.samples)
    return rows


def accuracy_csv(rows: Iterable[rawdata.AccuracyRow]) -> bytes:
    out = io.StringIO()
    out.write(",".join(ACCURACY_HEADER) + "\n")
    for r in rows:
        out.write(",".join([r.phase, r.label, _cell(r.ref_x_m), _cell(r.ref_y_m), _cell(r.ref_z_m),
                            _cell(r.meas_x_m), _cell(r.meas_y_m), _cell(r.meas_z_m), str(r.n)]) + "\n")
    data = out.getvalue().encode("utf-8")
    rawdata.read_accuracy(data)   # core's strict reader must accept what is written
    return data


def spread(envelope, point: str, refs: Mapping[str, Sequence[float]], tolerance_m: float,
           fraction: float = NEAR_FRACTION) -> dict:
    """Method §5.3 item 2: surveyed points near both extremes of each axis of the point's keep-in
    region (the intersection of the bounding boxes of its keep-in zones), counting only points inside
    the allowed region or within ``tolerance_m`` of it."""
    zones = [z for z in envelope.zones_for(point) if z.kind == "keep_in"]
    out: dict = {"zones": [z.id for z in zones], "fraction": fraction, "axes": {}, "outside": {}, "ok": True}
    inside = {}
    for label, xyz in refs.items():
        c = envelope.clearance(point, xyz)
        if c < -tolerance_m:
            out["outside"][label] = -c
        else:
            inside[label] = xyz
    if not zones:
        out["ok"] = None
        out["why"] = f"no keep-in zone applies to {point}, so there is no extent to spread through"
        return out
    lo, hi = [-math.inf] * 3, [math.inf] * 3
    for z in zones:
        if z.box_min_m is not None:
            zlo, zhi = list(z.box_min_m), list(z.box_max_m)
        else:
            xs = [v[0] for v in z.polygon_m]
            ys = [v[1] for v in z.polygon_m]
            zlo = [min(xs), min(ys), z.z_m[0] if z.z_m else -math.inf]
            zhi = [max(xs), max(ys), z.z_m[1] if z.z_m else math.inf]
        lo = [max(a, b) for a, b in zip(lo, zlo, strict=True)]
        hi = [min(a, b) for a, b in zip(hi, zhi, strict=True)]
    for i, axis in enumerate("xyz"):
        if not (math.isfinite(lo[i]) and math.isfinite(hi[i])) or hi[i] <= lo[i]:
            out["axes"][axis] = {"checked": False}
            continue
        width = hi[i] - lo[i]
        low = sorted(k for k, p in inside.items() if p[i] <= lo[i] + fraction * width)
        high = sorted(k for k, p in inside.items() if p[i] >= hi[i] - fraction * width)
        out["axes"][axis] = {"checked": True, "min_m": lo[i], "max_m": hi[i], "near_min": low, "near_max": high}
        if not low or not high:
            out["ok"] = False
    return out


def check(*, envelope, targets: Targets, holds: Sequence[Hold], samples: Sequence[rawdata.Sample], point: str,
          test: str = "EV-03", run: RunTimes | None = None, min_frames: int = 5, still_max_m: float = 0.002,
          event_notes: Sequence[str] = ()) -> tuple[bytes, dict]:
    """The accuracy file and the summary. Raises CheckError for unusable input."""
    if point not in envelope.points:
        raise CheckError(f"--point {point!r} is not in the envelope ({', '.join(sorted(envelope.points))})")
    profile = load_profile(PROFILES[test])
    tol = profile["tolerance_m"]
    ps = sorted((s for s in samples if s.point == point), key=lambda s: s.t_s)
    if not ps:
        raise CheckError(f"the samples have no rows for point {point!r}")
    times = [s.t_s for s in ps]
    chosen, notes, order, skipped = choose(holds, run)
    notes = list(event_notes) + notes
    problems: list[str] = []
    results: dict[str, list[HoldResult]] = {"pre": [], "post": [], "jitter": []}
    for phase, hs in chosen.items():
        for h in hs:
            r = measure(h, ps, min_frames=min_frames, still_max_m=still_max_m, times=times)
            r.used = True
            if phase != "jitter" and h.label not in targets.refs:
                r.used = False
                r.why = "no surveyed position for this label in the targets file"
                problems.append(f"{phase} {h.label}: the targets file has no surveyed position for {h.label}")
            problems.extend(r.problems)
            results[phase].append(r)
    rows = build_rows(results, targets.refs)
    data = accuracy_csv(rows)
    pre_rows = [r for r in results["pre"] if r.used and r.mean is not None]
    post_rows = [r for r in results["post"] if r.used and r.mean is not None]

    # The check's time is when its last surveyed point was measured: "not after the run start" is then
    # judged on the whole check, not on its first point.
    ends = [r.hold.wall(r.hold.t_end) for r in pre_rows]
    at = max(ends) if ends and None not in ends else None
    age = run.wall_start - at if run is not None and at is not None else None
    acc = core_ev03.accuracy(rows, age if age is not None else 0.0, profile)
    acc["robot_state_match"] = None
    declared = Declared("markerless_model_tracker", True, "hardware", acc["age_s"], 0.0)
    reasons = [r for r in validity_reasons(declared, acc) if r["code"] in ACCURACY_CODES]
    if run is None or at is None:
        acc["age_s"] = None   # unknown here: be report computes it from the declared times
    problems.extend(f"{r['code']}: {r['detail']}" for r in reasons)
    if age is not None and age < 0:
        problems.append("the pre-run check ended after the run started (method §5.3 item 6)")

    refs_used = {r.hold.label: targets.refs[r.hold.label] for r in pre_rows}
    sp = spread(envelope, point, refs_used, tol)
    if sp["ok"] is False:
        missing = [f"{a} {side}" for a, v in sp["axes"].items() if v.get("checked")
                   for side, key in (("min", "near_min"), ("max", "near_max")) if not v[key]]
        problems.append("the surveyed points do not reach both extremes of each axis of the keep-in (method §5.3 "
                        f"item 2): none near {', '.join(missing)}")
    elif sp["ok"] is None:
        notes.append(sp["why"])
    for label, d in sorted(sp["outside"].items()):
        notes.append(f"surveyed point {label} is {d * 1000:.0f} mm outside {point}'s allowed region; it does not "
                     "count toward the spread")
    for label in sorted(set(targets.refs) - {r.hold.label for r in results["pre"]}):
        notes.append(f"surveyed point {label} has no pre hold")
    jit = [r for r in results["jitter"] if r.valid >= 2]
    if jit and max(r.span_s for r in jit) < JITTER_SHOULD_S - 0.1:
        notes.append(f"the jitter record spans {max(r.span_s for r in jit):.1f} s; the method asks for at least "
                     f"{JITTER_SHOULD_S:g} s (§5.4 item 2, SHOULD)")
    if targets.method is None:
        problems.append("the targets file has no method line: say how the points were surveyed (it becomes "
                        "accuracy_check.method, which the method requires)")

    hold_summaries = []
    for h in holds:
        mine = next((r for v in results.values() for r in v if r.hold is h), None)
        hold_summaries.append(mine.summary() if mine else
                              {**HoldResult(h).summary(), "why": skipped.get(id(h), "not used")})
    post_ends = [r.hold.wall(r.hold.t_end) for r in post_rows]
    lab_config = {
        "accuracy_check": {"at": _iso(at), "points": len(pre_rows), "method": targets.method},
        "post_run_check": ({"at": _iso(max(post_ends)),
                            "method": "the same procedure after the run, without moving, re-registering or "
                                      "recalibrating the instrument; surveyed point(s) "
                                      + ", ".join(r.hold.label for r in post_rows)}
                           if post_ends and None not in post_ends else None),
    }
    summary = {
        "format": FORMAT,
        "point": point,
        "test": test,
        "profile": PROFILES[test],
        "targets": {"path": targets.path, "sha256": targets.sha256, "frame": targets.frame, "units": targets.units,
                    "method": targets.method, "surveyed_at": targets.surveyed_at,
                    "points": {k: {"given": list(targets.given[k]), "ref_m": list(v)}
                               for k, v in targets.refs.items()}},
        "run": None if run is None else {"path": run.path, "started_at": run.started_at,
                                         "wall_start_s": run.wall_start, "wall_end_s": run.wall_end},
        "order": order,
        "holds": hold_summaries,
        "accuracy": acc,
        "spread": sp,
        "lab_config": lab_config,
        "accuracy_csv_sha256": _sha256(data),
        "problems": problems,
        "notes": notes,
    }
    return data, summary


# ----- markerless: fit the hold windows ------------------------------------------------------------------
def fit_holds(session: Path, holds: Sequence[Hold], *, envelope: str, urdf: str | None, registration: Path,
              background: Path, out: Path, say: Callable[[str], None] = print) -> list[rawdata.Sample]:
    """Run `be fit` on each hold window only (with the session's registration and background) and return
    the samples. Each window's outputs go to ``out/<phase>_<label>_<k>/``."""
    from .markerless.cli import _default_urdf
    from .markerless.fit import FitError, fit_session
    from .markerless.model import ArmModel
    from .session import Session

    urdf_path = Path(urdf) if urdf else _default_urdf(session)
    model = ArmModel(urdf_path)
    sess = Session(session)
    t0 = sess.t0
    sess.close()
    samples: list[rawdata.Sample] = []
    for k, h in enumerate(holds):
        d = out / f"{h.phase}_{h.label}_{k}"
        try:
            meta = fit_session(session, urdf_path, envelope, registration, background=background, out=d,
                               t_from=h.t_start - t0, t_to=h.t_end - t0, model=model, say=lambda s: None)
        except FitError as e:
            say(f"  {h.phase} {h.label}: no fit ({e})")
            continue
        c = meta["counts"]
        say(f"  fitted {h.phase} {h.label}: {c['frames']} frames, {c['valid_frames']} valid")
        samples.extend(rawdata.read_samples((d / "samples.csv").read_bytes()))
    return samples


# ----- command line -------------------------------------------------------------------------------------
def register(subparsers) -> argparse.ArgumentParser:
    """Adds `instrument` to the `be` command line (entry-point group bounded_embodiment.cli)."""
    p = subparsers.add_parser("instrument", help="the instrument's accuracy check, jitter record and post-run check",
                              description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="instrument_command", metavar="{check,plan}")
    c = sub.add_parser("check", help="write accuracy.csv from the samples, the surveyed targets and the holds",
                       description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    c.add_argument("--envelope", required=True, help="the envelope JSON the run uses")
    c.add_argument("--targets", required=True, help=f"surveyed points ({TARGETS_FORMAT})")
    src = c.add_mutually_exclusive_group(required=True)
    src.add_argument("--session", help="a be record session: holds from its events.csv")
    src.add_argument("--holds", help=f"holds file ({HOLDS_FORMAT}), on the samples' timebase")
    c.add_argument("--samples", action="append", default=None, metavar="CSV",
                   help="the instrument's samples (be-samples-csv/1); default with --session: fit the holds")
    c.add_argument("--run", help="the run directory (declared.json): choose and order the holds against it")
    c.add_argument("--point", default=None, help="the measured point (default: tool, else the envelope's first)")
    c.add_argument("--test", choices=sorted(PROFILES), default="EV-03", help="whose profile limits to apply")
    c.add_argument("--registration", default=None, help="default SESSION/fit/registration.json")
    c.add_argument("--background", default=None, help="default SESSION/fit/background.npz")
    c.add_argument("--urdf", default=None, help="for fitting (default: the URDF the registration used)")
    c.add_argument("--out", default=None,
                   help="output directory (default RUN_DIR/accuracy with --run, else SESSION/fit/check)")
    c.add_argument("--settle-s", type=float, default=0.5, help="a hold starts this long after its event (default 0.5)")
    c.add_argument("--hold-s", type=float, default=3.0, help="longest pre or post hold, seconds (default 3)")
    c.add_argument("--jitter-s", type=float, default=15.0, help="longest jitter hold, seconds (default 15)")
    c.add_argument("--still-max-mm", type=float, default=2.0,
                   help="a hold whose two halves' means differ by more than this moved (default 2)")
    c.add_argument("--min-frames", type=int, default=5, help="fewest valid samples in a hold (default 5)")
    from .plan import add_parser as add_plan_parser

    add_plan_parser(sub)
    p.set_defaults(func=run, _instrument_parser=p)
    return p


def _say_table(say, summary: dict) -> None:
    acc = summary["accuracy"]
    errs = {e["label"]: e["error_m"] for e in acc["pre"]["errors"]}
    for h in summary["holds"]:
        if not h["used"]:
            say(f"  {h['phase']:6s} {h['label']:10s} not used: {h['why']}")
            continue
        m = h["mean_m"]
        mean = "no valid samples" if m is None else f"({m[0]:.4f}, {m[1]:.4f}, {m[2]:.4f}) m"
        extra = ""
        if h["phase"] == "pre" and h["label"] in errs:
            extra = f"  error {errs[h['label']] * 1000:5.1f} mm"
        drift = "" if h["half_drift_m"] is None else f"  still {h['half_drift_m'] * 1000:.1f} mm"
        say(f"  {h['phase']:6s} {h['label']:10s} {mean}  n {h['valid']}/{h['samples']}{extra}{drift}")
    lim, pre = acc["limits"], acc["pre"]
    if pre["points"]:
        say(f"pre-run check: {pre['points']} points, error p95 {pre['p95_error_m'] * 1000:.1f} mm (limit "
            f"{lim['p95_max_m'] * 1000:.1f}), max {pre['max_error_m'] * 1000:.1f} mm (limit "
            f"{lim['max_max_m'] * 1000:.1f})")
    sig = acc["jitter"]["sigma_m"]
    for j in acc["jitter"]["labels"]:
        say(f"jitter {j['label']}: {j['samples']} samples, sigma "
            + ("n/a" if j["sigma_m"] is None else f"{j['sigma_m'] * 1000:.2f} mm"))
    if sig is None:
        say("jitter: none")
    post = acc["post"]
    if post["compared"]:
        say(f"post-run check: {post['compared']} point(s), largest drift {post['max_drift_m'] * 1000:.1f} mm "
            f"(limit {lim['drift_max_m'] * 1000:.1f})")
    sp = summary["spread"]
    if sp["ok"] is not None:
        for axis, v in sp["axes"].items():
            if v.get("checked"):
                say(f"spread {axis}: near {v['min_m']:.3f} m: {', '.join(v['near_min']) or 'none'}; near "
                    f"{v['max_m']:.3f} m: {', '.join(v['near_max']) or 'none'}")
    if acc["age_s"] is not None:
        say(f"the pre-run check ended {acc['age_s']:.0f} s before the run started (limit {lim['max_age_s']:.0f} s)")
    lc = summary["lab_config"]
    say("for the lab config: " + json.dumps({k: v for k, v in lc.items() if v is not None}))


def run(args, *, say: Callable[[str], None] = print) -> int:
    cmd = getattr(args, "instrument_command", None)
    if cmd == "plan":
        from .plan import run_plan

        return run_plan(args, say=say)
    if cmd != "check":
        args._instrument_parser.print_help()
        return EXIT_USAGE
    try:
        return _run_check(args, say)
    except (CheckError, envelope_mod.EnvelopeError, rawdata.RawDataError, OSError, KeyError) as e:
        print(f"be instrument check: {e}", file=sys.stderr)
        return EXIT_USAGE


def _run_check(args, say) -> int:
    envelope = envelope_mod.load(args.envelope)
    point = args.point or ("tool" if "tool" in envelope.points else next(iter(envelope.points)))
    targets = load_targets(args.targets)
    session = Path(args.session) if args.session else None
    reg_path = Path(args.registration) if args.registration else (session / "fit" / "registration.json"
                                                                     if session else None)
    T_frame_F = None
    reg_doc = None
    if reg_path is not None and reg_path.is_file():
        reg_doc = json.loads(reg_path.read_text(encoding="utf-8"))
        T_frame_F = (reg_doc.get("steps") or {}).get("T_bl_F")
    elif args.registration:
        raise CheckError(f"no registration at {reg_path}")
    targets = to_envelope_frame(targets, envelope.frame, T_frame_F)
    run_times = load_run(Path(args.run)) if args.run else None
    if args.out:
        out = Path(args.out)
    elif args.run:
        out = Path(args.run) / "accuracy"
    elif session is not None:
        out = session / "fit" / "check"
    else:
        raise CheckError("give --out (no --run or --session to put the files beside)")
    notes: list[str] = []
    if session is not None:
        holds, notes = holds_from_events(read_events(session), settle_s=args.settle_s, hold_s=args.hold_s,
                                         jitter_s=args.jitter_s)
        if not holds:
            raise CheckError(f"{session}/events.csv has no pre:, post: or jitter events")
    else:
        holds = holds_from_csv(Path(args.holds).read_text(encoding="utf-8"), name=str(args.holds))
    if args.samples:
        samples = []
        for path in args.samples:
            samples.extend(rawdata.read_samples(Path(path).read_bytes()))
        sample_inputs = [{"path": p, "sha256": _sha256(Path(p).read_bytes())} for p in args.samples]
    else:
        if session is None:
            raise CheckError("--holds needs --samples")
        bg = Path(args.background) if args.background else session / "fit" / "background.npz"
        if reg_path is None or not reg_path.is_file():
            raise CheckError(f"no registration at {reg_path}: run `be fit register {session}` first")
        if not bg.is_file():
            raise CheckError(f"no background at {bg}: run `be fit background {session}` first (the check must use "
                             "the instrument exactly as the run does)")
        chosen, _, _, _ = choose(holds, run_times)
        wanted = [h for v in chosen.values() for h in v]
        say(f"be instrument check: fitting {len(wanted)} hold window(s) with be fit")
        samples = fit_holds(session, wanted, envelope=args.envelope, urdf=args.urdf, registration=reg_path,
                            background=bg, out=out / "fit", say=say)
        sample_inputs = [{"fitted": str(out / "fit"), "registration_sha256": _sha256(reg_path.read_bytes()),
                          "background_sha256": _sha256(bg.read_bytes())}]
    data, summary = check(envelope=envelope, targets=targets, holds=holds, samples=samples, point=point,
                          test=args.test, run=run_times, min_frames=args.min_frames,
                          still_max_m=args.still_max_mm / 1000.0, event_notes=notes)
    summary["inputs"] = {"envelope": {"path": str(args.envelope), "hash": envelope.hash},
                         "samples": sample_inputs,
                         "holds_from": ({"session": str(session),
                                         "events_sha256": _sha256((session / "events.csv").read_bytes())}
                                        if session else {"file": str(args.holds),
                                                         "sha256": _sha256(Path(args.holds).read_bytes())}),
                         "registration": (None if reg_doc is None else
                                          {"path": str(reg_path), "sha256": _sha256(reg_path.read_bytes())}),
                         "settings": {"settle_s": args.settle_s, "hold_s": args.hold_s, "jitter_s": args.jitter_s,
                                      "still_max_m": args.still_max_mm / 1000.0, "min_frames": args.min_frames}}
    try:
        from .. import __version__ as kit_version
    except ImportError:            # pragma: no cover
        kit_version = None
    summary["tool"] = {"name": "be instrument check", "kit_version": kit_version}
    out.mkdir(parents=True, exist_ok=True)
    (out / "accuracy.csv").write_bytes(data)
    (out / "accuracy-check.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    say(f"be instrument check: point {point}, {len(targets.refs)} surveyed point(s)"
        + (f", run {run_times.path}" if run_times else ""))
    _say_table(say, summary)
    for n in summary["notes"]:
        say(f"note: {n}")
    for p in summary["problems"]:
        say(f"PROBLEM: {p}")
    say(f"wrote {out / 'accuracy.csv'} and {out / 'accuracy-check.json'}")
    return EXIT_PROBLEMS if summary["problems"] else EXIT_OK
