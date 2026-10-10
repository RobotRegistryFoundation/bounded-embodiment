"""Strict readers for a run's raw data (CONTRACT.md, "Raw data formats").

=====================  ==============================================================
format                 header (exact, in this order)
=====================  ==============================================================
``be-samples-csv/1``   ``t_s,point,x_m,y_m,z_m,valid,residual_m``
``be-commands-csv/1``  ``seq,t_sent_s,t_done_s,strategy,payload_json,result,error_kind``
``be-state-csv/1``     ``t_s,source,joint,value``
``be-accuracy-csv/1``  ``phase,label,ref_x_m,ref_y_m,ref_z_m,meas_x_m,meas_y_m,meas_z_m,n``
=====================  ==============================================================

Each reader takes a path or the file's bytes and returns a list of rows, one ``NamedTuple`` per
data row, whose field names are the format's column names. Content that starts with the gzip
magic bytes is decompressed first, so ``samples.csv.gz`` and ``samples.csv`` read the same way; a
path whose name ends in ``.gz`` must hold gzip data, though.

Nothing is skipped or repaired. The text must be UTF-8 without a byte order mark, and the header
row must equal the format's header exactly. Every data row must have exactly as many fields as
the header (a blank line is a row with no fields). Cells are checked as follows:

* A number must match ``-?(digits[.digits] | .digits)([eE][+-]?digits)`` before ``float()``
  parses it, which refuses the whitespace, underscores, leading ``+``, ``nan`` and ``inf`` that
  ``float()`` alone would take; a value that overflows a double is refused too. ``float()`` of
  such a decimal is correctly rounded, so every platform reads the same double.
* An integer is ASCII digits only.
* Identifiers (``point``, ``strategy``, ``phase``, ``label``, ``source``, ``joint``) must be
  non-empty with no leading or trailing whitespace.
* An empty cell becomes ``None`` only where the format allows it (listed per format below).

A reader collects the problems it finds (at most ``MAX_FINDINGS`` errors) and raises
``RawDataError``, whose ``findings`` say where each one is (``where`` is ``"<file>:<line>"``).

Finding codes: ``RAW_UNREADABLE`` (cannot read, or bad gzip), ``RAW_ENCODING`` (not UTF-8, or a
byte order mark), ``RAW_HEADER`` (missing or different header), ``RAW_SYNTAX`` (CSV quoting
error), ``RAW_ROW`` (wrong number of fields), ``RAW_VALUE`` (a cell that is not allowed),
``RAW_DUPLICATE`` (a key repeated where the format allows one row per key), ``RAW_ORDER`` (rows
out of the order the format requires), and ``RAW_TOO_MANY_ERRORS`` (info: reading stopped).
"""

from __future__ import annotations

import codecs
import csv
import gzip
import io
import math
import os
import re
import zlib
from collections.abc import Callable
from pathlib import Path
from typing import NamedTuple

from .findings import Finding

__all__ = [
    "ACCURACY",
    "COMMANDS",
    "FORMATS",
    "MAX_FINDINGS",
    "SAMPLES",
    "STATE",
    "AccuracyRow",
    "Command",
    "RawDataError",
    "Sample",
    "StateRow",
    "read",
    "read_accuracy",
    "read_commands",
    "read_samples",
    "read_state",
]

SAMPLES = "be-samples-csv/1"
COMMANDS = "be-commands-csv/1"
STATE = "be-state-csv/1"
ACCURACY = "be-accuracy-csv/1"

MAX_FINDINGS = 20
"""A reader stops after this many errors and says so with an info finding."""

_GZIP_MAGIC = b"\x1f\x8b"
_DECIMAL = re.compile(r"-?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?")
_INTEGER = re.compile(r"[0-9]+")


class RawDataError(ValueError):
    """A raw data file cannot be read as its format. ``findings`` lists what was found."""

    def __init__(self, findings: list[Finding]):
        self.findings = list(findings)
        errs = [f for f in self.findings if f.severity == "error"] or self.findings
        head = "; ".join(f"{f.where + ': ' if f.where else ''}{f.detail}" for f in errs[:3])
        more = f" (+{len(errs) - 3} more)" if len(errs) > 3 else ""
        super().__init__(f"invalid raw data: {head}{more}" if head else "invalid raw data")


# --------------------------------------------------------------------------- row types


class Sample(NamedTuple):
    """One ``be-samples-csv/1`` row. Coordinates are in the envelope frame, in metres.

    ``x_m``, ``y_m`` and ``z_m`` are all present or all ``None`` (``None`` only when
    ``valid`` is 0); ``residual_m`` may be ``None``. Only rows with ``valid == 1`` are
    measurements."""

    t_s: float
    point: str
    x_m: float | None
    y_m: float | None
    z_m: float | None
    valid: int
    residual_m: float | None


class Command(NamedTuple):
    """One ``be-commands-csv/1`` row. ``t_done_s``, ``payload_json``, ``result`` and
    ``error_kind`` may be ``None``; ``payload_json`` is kept as text and never parsed. Rows
    are in sending order: ``seq`` strictly increases and ``t_sent_s`` never decreases."""

    seq: int
    t_sent_s: float
    t_done_s: float | None
    strategy: str
    payload_json: str | None
    result: str | None
    error_kind: str | None


class StateRow(NamedTuple):
    """One ``be-state-csv/1`` row (long format). ``source`` is ``encoder`` or ``fk_tool``."""

    t_s: float
    source: str
    joint: str
    value: float


class AccuracyRow(NamedTuple):
    """One ``be-accuracy-csv/1`` row.

    ``pre`` and ``post`` rows: one per surveyed point (``label``, at most one per phase),
    ``ref_*`` the surveyed position and ``meas_*`` the instrument's mean over ``n`` frames.
    ``jitter`` rows: one per raw static sample, so ``n`` is 1 and ``ref_*`` are ``None``."""

    phase: str
    label: str
    ref_x_m: float | None
    ref_y_m: float | None
    ref_z_m: float | None
    meas_x_m: float
    meas_y_m: float
    meas_z_m: float
    n: int


# --------------------------------------------------------------------------- cells


class _CellError(ValueError):
    pass


def _number(cell: str) -> float:
    if _DECIMAL.fullmatch(cell) is None:
        raise _CellError(f"{cell!r} is not a decimal number")
    value = float(cell)
    if not math.isfinite(value):
        raise _CellError(f"{cell!r} is out of range for an IEEE 754 double")
    return value


def _optional_number(cell: str) -> float | None:
    return None if cell == "" else _number(cell)


def _integer(cell: str) -> int:
    if _INTEGER.fullmatch(cell) is None:
        raise _CellError(f"{cell!r} is not a non-negative integer")
    try:
        return int(cell)
    except ValueError:  # more digits than int() will convert
        raise _CellError(f"{cell!r} is too long for an integer") from None


def _identifier(cell: str) -> str:
    if cell == "":
        raise _CellError("must not be empty")
    if cell != cell.strip():
        raise _CellError(f"{cell!r} has leading or trailing whitespace")
    return cell


def _optional_text(cell: str) -> str | None:
    return None if cell == "" else cell


def _one_of(*allowed: str) -> Callable[[str], str]:
    def parse(cell: str) -> str:
        if cell not in allowed:
            raise _CellError(f"{cell!r} is not one of {', '.join(allowed)}")
        return cell

    return parse


# --------------------------------------------------------------------------- reading


class _Problems:
    """Findings for one file; raises once MAX_FINDINGS errors have been collected."""

    def __init__(self, label: str):
        self.label = label
        self.findings: list[Finding] = []

    def add(self, code: str, line: int | None, detail: str) -> None:
        where = self.label if line is None else f"{self.label}:{line}"
        self.findings.append(Finding(code, "error", detail, where))
        if len(self.findings) >= MAX_FINDINGS:
            self.findings.append(
                Finding(
                    "RAW_TOO_MANY_ERRORS",
                    "info",
                    f"stopped reading after {MAX_FINDINGS} errors",
                    where,
                )
            )
            raise RawDataError(self.findings)

    def raise_if_any(self) -> None:
        if self.findings:
            raise RawDataError(self.findings)


def _fail(code: str, detail: str, where: str) -> RawDataError:
    return RawDataError([Finding(code, "error", detail, where)])


def _text(src: str | os.PathLike | bytes | bytearray | memoryview) -> tuple[str, str]:
    """(decoded text, label for findings) of a path or of a file's bytes."""
    if isinstance(src, (bytes, bytearray, memoryview)):
        data, label, named_gz = bytes(src), "<bytes>", None
    elif isinstance(src, (str, os.PathLike)):
        path = Path(src)
        label, named_gz = path.name, path.name.endswith(".gz")
        try:
            data = path.read_bytes()
        except OSError as e:
            raise _fail("RAW_UNREADABLE", f"cannot read {path}: {e.strerror or e}", label) from None
    else:
        raise TypeError(f"expected a path or bytes, not {type(src).__name__}")
    is_gz = data[:2] == _GZIP_MAGIC
    if named_gz is True and not is_gz:
        raise _fail("RAW_UNREADABLE", "the file name ends in .gz but it is not gzip data", label)
    if is_gz:
        try:
            data = gzip.decompress(data)
        except (OSError, EOFError, zlib.error) as e:
            raise _fail("RAW_UNREADABLE", f"not a valid gzip stream: {e}", label) from None
    if data.startswith(codecs.BOM_UTF8):
        raise _fail("RAW_ENCODING", "the text starts with a UTF-8 byte order mark", label)
    try:
        return data.decode("utf-8"), label
    except UnicodeDecodeError as e:
        raise _fail("RAW_ENCODING", f"the text is not valid UTF-8: {e}", label) from None


def _read_rows(
    src: str | os.PathLike | bytes,
    format_name: str,
    row_type: type,
    columns: tuple[tuple[str, Callable[[str], object]], ...],
    check: Callable[[tuple, int, _Problems], None] | None = None,
) -> list:
    """Parse every data row of src into row_type; check(row, line, problems) adds the rules
    that span cells or rows."""
    text, label = _text(src)
    header = row_type._fields
    problems = _Problems(label)
    reader = csv.reader(io.StringIO(text, newline=""), strict=True)
    rows: list = []
    try:
        first = next(reader, None)
        if first is None:
            raise _fail(
                "RAW_HEADER", f"the file is empty; {format_name} needs the header "
                f"{','.join(header)}", label
            )
        if tuple(first) != header:
            raise _fail(
                "RAW_HEADER",
                f"the header is {','.join(first)!r}; {format_name} needs "
                f"{','.join(header)!r}",
                f"{label}:1",
            )
        for row in reader:
            line = reader.line_num
            if len(row) != len(header):
                what = "a blank line" if not row else f"{len(row)} fields"
                problems.add("RAW_ROW", line, f"{what}; {format_name} rows have {len(header)}")
                continue
            values, ok = [], True
            for (name, parse), cell in zip(columns, row):
                try:
                    values.append(parse(cell))
                except _CellError as e:
                    problems.add("RAW_VALUE", line, f"{name}: {e}")
                    ok = False
            if ok:
                parsed = row_type(*values)
                if check is not None:
                    before = len(problems.findings)
                    check(parsed, line, problems)
                    if len(problems.findings) != before:
                        continue
                rows.append(parsed)
    except csv.Error as e:
        problems.add("RAW_SYNTAX", reader.line_num, f"CSV syntax error: {e}")
    problems.raise_if_any()
    return rows


def _xyz_all_or_none(values: tuple, names: tuple[str, str, str]) -> str | None:
    present = [v is not None for v in values]
    if any(present) and not all(present):
        return f"{', '.join(names)} must be all present or all empty"
    return None


# --------------------------------------------------------------------------- formats


def read_samples(src: str | os.PathLike | bytes) -> list[Sample]:
    """Read ``be-samples-csv/1``. Empty allowed in ``x_m``/``y_m``/``z_m`` (all three
    together, and only when ``valid`` is 0) and ``residual_m``. ``valid`` is ``0`` or ``1``."""

    def check(row: Sample, line: int, problems: _Problems) -> None:
        bad = _xyz_all_or_none((row.x_m, row.y_m, row.z_m), ("x_m", "y_m", "z_m"))
        if bad:
            problems.add("RAW_VALUE", line, bad)
        elif row.valid == 1 and row.x_m is None:
            problems.add("RAW_VALUE", line, "a valid row (valid=1) needs x_m, y_m and z_m")

    columns = (
        ("t_s", _number),
        ("point", _identifier),
        ("x_m", _optional_number),
        ("y_m", _optional_number),
        ("z_m", _optional_number),
        ("valid", lambda c: int(_one_of("0", "1")(c))),
        ("residual_m", _optional_number),
    )
    return _read_rows(src, SAMPLES, Sample, columns, check)


def read_commands(src: str | os.PathLike | bytes) -> list[Command]:
    """Read ``be-commands-csv/1``. Empty allowed in ``t_done_s``, ``payload_json``, ``result``
    and ``error_kind``. ``seq`` must strictly increase and ``t_sent_s`` must never decrease
    from one row to the next, and ``t_done_s`` (when present) is not before ``t_sent_s``."""
    previous: list[Command] = []

    def check(row: Command, line: int, problems: _Problems) -> None:
        if row.t_done_s is not None and row.t_done_s < row.t_sent_s:
            problems.add("RAW_VALUE", line, f"t_done_s {row.t_done_s!r} is before t_sent_s")
        if previous:
            last = previous[0]
            if row.seq <= last.seq:
                problems.add("RAW_ORDER", line, f"seq {row.seq} does not follow seq {last.seq}")
            if row.t_sent_s < last.t_sent_s:
                problems.add(
                    "RAW_ORDER",
                    line,
                    f"t_sent_s {row.t_sent_s!r} is before the previous row's {last.t_sent_s!r}",
                )
        previous[:] = [row]

    columns = (
        ("seq", _integer),
        ("t_sent_s", _number),
        ("t_done_s", _optional_number),
        ("strategy", _identifier),
        ("payload_json", _optional_text),
        ("result", _optional_text),
        ("error_kind", _optional_text),
    )
    return _read_rows(src, COMMANDS, Command, columns, check)


def read_state(src: str | os.PathLike | bytes) -> list[StateRow]:
    """Read ``be-state-csv/1``. No cell may be empty; ``source`` is ``encoder`` or
    ``fk_tool``."""
    columns = (
        ("t_s", _number),
        ("source", _one_of("encoder", "fk_tool")),
        ("joint", _identifier),
        ("value", _number),
    )
    return _read_rows(src, STATE, StateRow, columns)


def read_accuracy(src: str | os.PathLike | bytes) -> list[AccuracyRow]:
    """Read ``be-accuracy-csv/1``. ``pre`` and ``post`` rows need all of ``ref_*`` and
    ``meas_*`` and at most one row per label; ``jitter`` rows have empty ``ref_*`` and
    ``n`` = 1. ``n`` is at least 1."""
    seen: dict[tuple[str, str], int] = {}

    def check(row: AccuracyRow, line: int, problems: _Problems) -> None:
        refs = (row.ref_x_m, row.ref_y_m, row.ref_z_m)
        bad = _xyz_all_or_none(refs, ("ref_x_m", "ref_y_m", "ref_z_m"))
        if bad:
            problems.add("RAW_VALUE", line, bad)
        if row.n < 1:
            problems.add("RAW_VALUE", line, "n must be at least 1")
        if row.phase == "jitter":
            if row.ref_x_m is not None:
                problems.add("RAW_VALUE", line, "jitter rows have empty ref_x_m, ref_y_m, ref_z_m")
            if row.n != 1:
                problems.add(
                    "RAW_VALUE", line, f"jitter rows are single samples (n=1), not n={row.n}"
                )
            return
        if row.ref_x_m is None and not bad:
            problems.add("RAW_VALUE", line, f"{row.phase} rows need ref_x_m, ref_y_m and ref_z_m")
        key = (row.phase, row.label)
        if key in seen:
            problems.add(
                "RAW_DUPLICATE",
                line,
                f"{row.phase} label {row.label!r} already appears on line {seen[key]}",
            )
        else:
            seen[key] = line

    columns = (
        ("phase", _one_of("pre", "post", "jitter")),
        ("label", _identifier),
        ("ref_x_m", _optional_number),
        ("ref_y_m", _optional_number),
        ("ref_z_m", _optional_number),
        ("meas_x_m", _number),
        ("meas_y_m", _number),
        ("meas_z_m", _number),
        ("n", _integer),
    )
    return _read_rows(src, ACCURACY, AccuracyRow, columns, check)


FORMATS: dict[str, Callable[[str | os.PathLike | bytes], list]] = {
    SAMPLES: read_samples,
    COMMANDS: read_commands,
    STATE: read_state,
    ACCURACY: read_accuracy,
}
"""Reader by format name, as ``report.raw_data[].format`` names them."""


def read(format_name: str, src: str | os.PathLike | bytes) -> list:
    """Read ``src`` as the named format; ``KeyError`` for an unknown format name."""
    try:
        reader = FORMATS[format_name]
    except KeyError:
        raise KeyError(
            f"unknown raw data format {format_name!r}; known: {', '.join(FORMATS)}"
        ) from None
    return reader(src)
