# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""Bring an instrument's timestamps onto the run timebase (`be report --clock`).

The run timebase is the runner's ``time.monotonic()`` with run start = 0 (CONTRACT.md):
commands.csv and state.csv are on it. An instrument stamps its samples on its own clock
(``be fit`` writes the recording host's CLOCK_MONOTONIC). `be report` adds one offset to every
sample's ``t_s`` and declares it in the report's ``run.clock``: ``offset_s`` (run time =
instrument time + offset_s), ``uncertainty_s`` (U: the scorer widens its window by U at both
ends) and ``method`` (how both were found, in words).

``monotonic``
    The instrument ran on the runner's host and stamped CLOCK_MONOTONIC, the clock the runner's
    ``time.monotonic()`` reads. offset = -(the runner's monotonic reading at run start, which
    declared.json records as ``timestamps.monotonic_start_s``). U = the offset's rounding to the
    microsecond plus the clock's resolution: 1 us. Given the recorder's clock.csv, the method is
    cross-checked: the recorder's wall-minus-monotonic at the pair nearest the run start must
    equal the runner's to within 50 ms, or the two are not one clock and the method is refused.
``wall``
    The instrument's clock is mapped to wall time by the recorder's clock.csv (pairs of
    ``time.time()`` and ``time.monotonic()``, every 5 s) and the run start is the runner's wall
    time. offset = c - W0, where c is the midrange of wall-minus-monotonic over the pairs within
    10 s of the run and W0 the runner's ``time.time()`` at run start. U is the sum of: the
    agreement of the two hosts' wall clocks, as the lab declares it (0 when one host recorded
    and ran); half the range of c over those pairs (NTP slewing during the run); the largest
    pair-reading uncertainty; 1 us for clock.csv's six decimals; the uncertainty of W0 (a few
    microseconds from the runner's bracketed reading; 0.5 s when declared.json has only the
    whole-second ``started_at`` of an older runner); the runner's own wall-minus-monotonic drift
    from start to end; and the offset's rounding.
``run``
    The samples are already on the run timebase (the runner's simulated instrument writes
    raw/samples.csv that way): offset 0, U 0.

Arithmetic is decimal: the offset is rounded to the microsecond and added to each ``t_s`` cell
exactly (a converted cell keeps at least six decimals, more if it had more), U is rounded up to
the microsecond. Nothing else in a samples file is touched.
"""
from __future__ import annotations

import csv
import datetime as dt
import decimal
import gzip
import hashlib
import io
import math
import re
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from decimal import ROUND_CEILING, ROUND_HALF_EVEN, Decimal

from bounded_embodiment import rawdata

__all__ = ["CLOCK_HEADER", "EVIDENCE_FILE", "METHODS", "Clock", "ClockPair", "Converted", "RunTimes",
           "TimebaseError", "convert_samples", "monotonic_clock", "read_clock_csv", "run_clock",
           "run_times", "utc_text", "wall_clock"]

METHODS = ("monotonic", "wall", "run")
EVIDENCE_FILE = "raw/instrument/recorder-clock.csv"
"""Where `be report` puts the recorder's clock.csv a method used, so a reader can redo it."""
CLOCK_HEADER = ("kind", "t_wall_s", "t_mono_s", "uncertainty_s")
US = Decimal("0.000001")
SAME_HOST_TOLERANCE_S = Decimal("0.05")
"""monotonic: the most the recorder's and the runner's wall-minus-monotonic may differ."""
PAIR_WINDOW_S = 10.0
"""wall: clock.csv pairs within this many seconds of the run are used (the recorder writes one
every 5 s)."""
CSV_ROUNDING_S = Decimal("0.000001")
"""clock.csv writes t_wall_s and t_mono_s with six decimals: +-0.5 us each."""
SAMPLES_HEADER = ("t_s", "point", "x_m", "y_m", "z_m", "valid", "residual_m")

_CTX = decimal.Context(prec=60)
_NUMBER = re.compile(r"-?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?")
_ISO = re.compile(r"([0-9]{4}-[0-9]{2}-[0-9]{2})T([0-9]{2}:[0-9]{2}:[0-9]{2})(?:\.([0-9]{1,9}))?"
                  r"(Z|[+-]00:00)")


class TimebaseError(ValueError):
    """The instrument's clock cannot be brought onto the run timebase as asked."""


def _dec(x: float) -> Decimal:
    """A float as the decimal of its shortest repr (what JSON and CSV show)."""
    return Decimal(repr(float(x)))


def _round_us(x: Decimal) -> Decimal:
    return x.quantize(US, rounding=ROUND_HALF_EVEN, context=_CTX)


def _ceil_us(x: Decimal) -> Decimal:
    return max(x.quantize(US, rounding=ROUND_CEILING, context=_CTX), Decimal(0))


def utc_text(epoch_s: float) -> str:
    """An epoch time as the report's UTC form, to the microsecond: 2026-10-13T18:30:00.123456Z."""
    moment = dt.datetime.fromtimestamp(epoch_s, dt.timezone.utc)
    return moment.strftime("%Y-%m-%dT%H:%M:%S.%fZ")


# ----- the run's times -----------------------------------------------------------------------------
@dataclass(frozen=True)
class RunTimes:
    """When the run started and ended on the runner's clocks (declared.json ``timestamps``)."""

    started_at: str
    ended_at: str
    wall_start_s: float
    wall_start_uncertainty_s: float
    wall_end_s: float
    monotonic_start_s: float | None = None
    monotonic_end_s: float | None = None
    monotonic_clock: str | None = None
    monotonic_resolution_s: float | None = None

    @property
    def runner_drift_s(self) -> Decimal | None:
        """How far the runner's wall clock moved against its monotonic clock during the run."""
        if self.monotonic_start_s is None or self.monotonic_end_s is None:
            return None
        start = _dec(self.wall_start_s) - _dec(self.monotonic_start_s)
        end = _dec(self.wall_end_s) - _dec(self.monotonic_end_s)
        return abs(end - start)


def _number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        return None
    return float(value)


def _iso(text: object, what: str) -> tuple[str, float, float]:
    """(report form, epoch of its start, length of the interval it names) of an RFC 3339 UTC
    time as the runner writes it (2026-10-13T18:30:00+00:00, whole seconds, truncated)."""
    m = _ISO.fullmatch(text) if isinstance(text, str) else None
    if m is None:
        raise TimebaseError(f"declared.json timestamps.{what} is not a UTC time: {text!r}")
    day, clock, frac, _ = m.groups()
    moment = dt.datetime.fromisoformat(f"{day}T{clock}+00:00")
    epoch = moment.timestamp() + (int(frac) / 10 ** len(frac) if frac else 0.0)
    width = 10.0 ** -len(frac) if frac else 1.0
    return f"{day}T{clock}{'.' + frac if frac else ''}Z", epoch, width


def run_times(timestamps: Mapping) -> RunTimes:
    """RunTimes from declared.json's ``timestamps``. A runner that records ``wall_start_s`` gives
    the start to the microsecond; an older one only ``started_at`` in whole seconds (truncated),
    so the start is taken as the middle of that second, give or take half a second."""
    if not isinstance(timestamps, Mapping):
        raise TimebaseError("declared.json has no timestamps block")
    wall_start = _number(timestamps.get("wall_start_s"))
    if wall_start is not None:
        started_at = utc_text(wall_start)
        u0 = _number(timestamps.get("wall_start_uncertainty_s")) or 0.0
    else:
        started_at, epoch, width = _iso(timestamps.get("started_at"), "started_at")
        wall_start, u0 = epoch + width / 2, width / 2
    wall_end = _number(timestamps.get("wall_end_s"))
    if wall_end is not None:
        ended_at = utc_text(wall_end)
    else:
        if timestamps.get("ended_at") is None:
            raise TimebaseError("declared.json has no timestamps.ended_at: the runner did not "
                                "finish writing it, so the run has no declared end")
        ended_at, epoch, width = _iso(timestamps.get("ended_at"), "ended_at")
        wall_end = epoch + width
    return RunTimes(
        started_at=started_at, ended_at=ended_at, wall_start_s=wall_start,
        wall_start_uncertainty_s=u0, wall_end_s=wall_end,
        monotonic_start_s=_number(timestamps.get("monotonic_start_s")),
        monotonic_end_s=_number(timestamps.get("monotonic_end_s")),
        monotonic_clock=timestamps.get("monotonic_clock") if isinstance(
            timestamps.get("monotonic_clock"), str) else None,
        monotonic_resolution_s=_number(timestamps.get("monotonic_resolution_s")),
    )


# ----- the recorder's clock.csv --------------------------------------------------------------------
@dataclass(frozen=True)
class ClockPair:
    """One row of a recorder's clock.csv: time.time() and time.monotonic() read together."""

    kind: str
    wall_s: float
    mono_s: float
    uncertainty_s: float

    @property
    def wall_minus_mono(self) -> Decimal:
        return _dec(self.wall_s) - _dec(self.mono_s)


def read_clock_csv(data: bytes) -> list[ClockPair]:
    """The pairs of a clock.csv (``kind,t_wall_s,t_mono_s,uncertainty_s``), strictly."""
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as e:
        raise TimebaseError(f"clock.csv is not UTF-8: {e}") from None
    rows = list(csv.reader(io.StringIO(text, newline="")))
    if not rows or tuple(rows[0]) != CLOCK_HEADER:
        raise TimebaseError(f"clock.csv must start with the header {','.join(CLOCK_HEADER)}")
    pairs = []
    for line, row in enumerate(rows[1:], start=2):
        if len(row) != 4 or not all(_NUMBER.fullmatch(c) for c in row[1:]):
            raise TimebaseError(f"clock.csv line {line} is not kind,t_wall_s,t_mono_s,uncertainty_s")
        wall, mono, unc = (float(c) for c in row[1:])
        if not all(math.isfinite(v) for v in (wall, mono, unc)) or unc < 0:
            raise TimebaseError(f"clock.csv line {line} has a value out of range")
        pairs.append(ClockPair(row[0], wall, mono, unc))
    if not pairs:
        raise TimebaseError("clock.csv has no pairs")
    return pairs


# ----- the three methods ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Clock:
    """How the instrument's timestamps become run time: ``offset`` (run = instrument + offset),
    ``uncertainty`` (U), ``description`` (the report's ``run.clock.method``), ``record`` (what
    raw/instrument/timebase.json holds) and ``evidence`` (the clock.csv used, or None)."""

    method: str
    offset: Decimal
    uncertainty: Decimal
    description: str
    record: dict = field(default_factory=dict)
    evidence: bytes | None = None

    def block(self) -> dict:
        """The report's ``run.clock``."""
        return {"offset_s": float(self.offset), "uncertainty_s": float(self.uncertainty),
                "method": self.description}


def _record(method: str, offset: Decimal, uncertainty: Decimal, terms: Mapping[str, Decimal],
            **inputs) -> dict:
    return {"format": "be-timebase/0.1", "method": method, "offset_s": float(offset),
            "uncertainty_s": float(uncertainty),
            "uncertainty_terms_s": {k: float(v) for k, v in terms.items()}, **inputs}


def run_clock(*, by_runner: bool) -> Clock:
    """The samples are on the run timebase already."""
    who = ("the runner's simulated instrument stamped them with the run clock itself"
           if by_runner else "the lab declares they were stamped on the run timebase")
    text = f"run: the samples are on the run timebase already ({who}); no offset, U = 0"
    zero = Decimal(0)
    return Clock("run", zero, zero, text, _record("run", zero, zero, {}))


def monotonic_clock(times: RunTimes, pairs: Sequence[ClockPair] | None = None,
                    evidence: bytes | None = None) -> Clock:
    """Same host, CLOCK_MONOTONIC (module docstring)."""
    if times.monotonic_start_s is None:
        raise TimebaseError("declared.json has no timestamps.monotonic_start_s (runners before this "
                            "kit did not record it): use --clock wall with the recorder's clock.csv")
    m0 = _dec(times.monotonic_start_s)
    offset = _round_us(-m0)
    terms = {"offset_rounding": abs(offset + m0),
             "monotonic_resolution": _dec(times.monotonic_resolution_s or 1e-9)}
    uncertainty = max(_ceil_us(sum(terms.values())), US)
    check = "not cross-checked (no clock.csv given)"
    inputs: dict = {"runner": {"monotonic_start_s": times.monotonic_start_s,
                               "monotonic_clock": times.monotonic_clock}}
    if pairs:
        runner_c = _dec(times.wall_start_s) - m0
        nearest = min(pairs, key=lambda p: (abs(p.wall_s - times.wall_start_s), p.wall_s))
        differs = nearest.wall_minus_mono - runner_c
        if abs(differs) > SAME_HOST_TOLERANCE_S:
            raise TimebaseError(
                "--clock monotonic says the instrument stamped the runner's CLOCK_MONOTONIC, but "
                f"the recorder's clock.csv puts its monotonic clock {float(differs):+.3f} s away from "
                "the runner's (wall minus monotonic, at the pair nearest the run start): they are "
                "not one clock (another host, or a reboot). Use --clock wall.")
        check = (f"cross-checked with the recorder's clock.csv: wall minus monotonic agrees with the "
                 f"runner's to {abs(float(differs)) * 1000:.3f} ms")
        inputs["recorder_clock"] = {"file": EVIDENCE_FILE, "pairs": len(pairs),
                                    "nearest_pair_wall_minus_monotonic_differs_s": float(differs)}
    text = (f"monotonic: the instrument stamped its samples with CLOCK_MONOTONIC on the runner's "
            f"host; offset = minus the runner's time.monotonic() at run start ({m0} s), to the "
            f"microsecond; U = that rounding plus the clock's resolution; {check}")
    return Clock("monotonic", offset, uncertainty, text,
                 _record("monotonic", offset, uncertainty, terms, **inputs),
                 evidence if pairs else None)


def wall_clock(times: RunTimes, pairs: Sequence[ClockPair], agreement_s: float,
               evidence: bytes | None = None) -> Clock:
    """The recorder's wall clock against the runner's (module docstring)."""
    if not isinstance(agreement_s, (int, float)) or not math.isfinite(agreement_s) or agreement_s < 0:
        raise TimebaseError("the hosts' wall-clock agreement must be a number of seconds >= 0")
    lo, hi = times.wall_start_s - PAIR_WINDOW_S, times.wall_end_s + PAIR_WINDOW_S
    used = [p for p in pairs if lo <= p.wall_s <= hi]
    if not used:
        raise TimebaseError(
            f"the recorder's clock.csv has no pair within {PAIR_WINDOW_S:.0f} s of the run "
            f"({times.started_at} to {times.ended_at}): was it recording during the run?")
    cs = [p.wall_minus_mono for p in used]
    c_mid = (max(cs) + min(cs)) / 2
    w0 = _dec(times.wall_start_s)
    exact = c_mid - w0
    offset = _round_us(exact)
    drift = times.runner_drift_s
    terms = {
        "host_wall_clock_agreement": _dec(agreement_s),
        "recorder_wall_minus_monotonic_half_range": (max(cs) - min(cs)) / 2,
        "recorder_pair_reading": max(_dec(p.uncertainty_s) for p in used),
        "clock_csv_rounding": CSV_ROUNDING_S,
        "runner_start_reading": _dec(times.wall_start_uncertainty_s),
        "runner_wall_drift": drift if drift is not None else Decimal(0),
        "offset_rounding": abs(offset - exact),
    }
    uncertainty = max(_ceil_us(sum(terms.values())), US)
    half_range = terms["recorder_wall_minus_monotonic_half_range"]
    drift_text = (f"the runner's wall clock moved {float(drift) * 1000:.3f} ms against its monotonic "
                  "clock during the run" if drift is not None
                  else "the runner's wall-clock drift during the run was not recorded")
    text = (f"wall: the recorder's clock.csv ({len(used)} pairs of time.time() and time.monotonic() "
            f"within {PAIR_WINDOW_S:.0f} s of the run) maps its clock to wall time, and the run "
            f"starts at the runner's time.time() {times.started_at}; offset = the recorder's wall "
            f"minus monotonic ({_round_us(c_mid)} s) minus that start. U = the two hosts' "
            f"wall-clock agreement as the lab declares it ({_dec(agreement_s)} s) + half the range "
            f"of wall minus monotonic over those pairs ({float(half_range) * 1000:.3f} ms) + "
            f"reading, rounding and drift terms; {drift_text}")
    record = _record("wall", offset, uncertainty, terms,
                     runner={"wall_start_s": times.wall_start_s,
                             "wall_start_uncertainty_s": times.wall_start_uncertainty_s,
                             "wall_end_s": times.wall_end_s,
                             "monotonic_start_s": times.monotonic_start_s,
                             "monotonic_end_s": times.monotonic_end_s},
                     recorder_clock={"file": EVIDENCE_FILE, "pairs": len(pairs),
                                     "pairs_used": len(used),
                                     "wall_minus_monotonic_s": {"min": float(min(cs)),
                                                                "max": float(max(cs))}})
    return Clock("wall", offset, uncertainty, text, record, evidence)


# ----- converting samples --------------------------------------------------------------------------
@dataclass(frozen=True)
class Converted:
    """be-samples-csv/1 on the run timebase, and what went into it."""

    data: bytes
    rows: int
    sources: list[dict]
    valid_times: list[Decimal]


def _text(data: bytes) -> str:
    if data[:2] == b"\x1f\x8b":
        data = gzip.decompress(data)
    return data.decode("utf-8")


def _shift(cell: str, offset: Decimal) -> tuple[str, Decimal]:
    value = Decimal(cell)
    exponent = value.as_tuple().exponent
    digits = max(6, -exponent if isinstance(exponent, int) and exponent < 0 else 0)
    out = (value + offset).quantize(Decimal(1).scaleb(-digits), context=_CTX)
    if out == 0:
        out = abs(out)  # never "-0.000000"
    return format(out, "f"), out


def convert_samples(sources: Sequence[tuple[str, bytes]], offset: Decimal) -> Converted:
    """Every row of every source (be-samples-csv/1, gzip or not), in order, with ``t_s`` + offset.
    Each source is read with core's strict reader first; a (t_s, point) that two sources both
    give is refused."""
    out = io.StringIO()
    writer = csv.writer(out, lineterminator="\n")
    writer.writerow(SAMPLES_HEADER)
    seen: dict[tuple[str, str], int] = {}
    described, valid_times, total = [], [], 0
    for index, (name, data) in enumerate(sources):
        try:
            rawdata.read_samples(data)
        except rawdata.RawDataError as e:
            raise TimebaseError(f"{name} is not be-samples-csv/1: {e}") from None
        rows = list(csv.reader(io.StringIO(_text(data), newline=""), strict=True))[1:]
        first = last = None
        for row in rows:
            t_text, t_value = _shift(row[0], offset)
            key = (t_text, row[1])
            if seen.get(key, index) != index:
                raise TimebaseError(f"{name} and an earlier --samples file both have {row[1]} at "
                                    f"t_s {row[0]} (overlapping instrument output?)")
            seen[key] = index
            writer.writerow([t_text, *row[1:]])
            if row[5] == "1":
                valid_times.append(t_value)
            first = row[0] if first is None else first
            last = row[0]
        total += len(rows)
        described.append({"name": name, "sha256": "sha256:" + hashlib.sha256(data).hexdigest(),
                          "rows": len(rows), "first_t_s": first, "last_t_s": last})
    return Converted(out.getvalue().encode("utf-8"), total, described, valid_times)
