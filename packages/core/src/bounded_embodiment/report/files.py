"""The run directory: its fixed layout, and reading it without following links.

A run directory holds ``report.json`` and the files the report was built from, at fixed paths
(docs/seams/report.md). :func:`scan` walks it once, never following a symbolic link (a link is
refused, not read), reads every regular file once with a size cap, and hashes the same bytes it
keeps, so what is hashed is what is parsed. :func:`classify` gives a path's role and format, or
None for a path the layout does not allow.

Finding codes (all errors): RUN_DIR_INVALID, RUN_DIR_TOO_LARGE, SYMLINK_REFUSED,
NOT_A_REGULAR_FILE, FILE_UNREADABLE, FILE_TOO_LARGE, REPORT_MISSING.
"""

from __future__ import annotations

import errno
import hashlib
import os
import re
import stat
from dataclasses import dataclass, field
from pathlib import Path

from ..findings import Finding
from ..rawdata import ACCURACY, COMMANDS, SAMPLES, STATE

__all__ = [
    "COMMAND_SPACE_FILE",
    "ENVELOPE_FILE",
    "MAX_FILE_BYTES",
    "MAX_REPORT_BYTES",
    "PARSED_ROLES",
    "REPORT_FILE",
    "WAYPOINTS_FILE",
    "RunFile",
    "RunFiles",
    "classify",
    "raw_data_entries",
    "scan",
    "show",
]

REPORT_FILE = "report.json"
ENVELOPE_FILE = "envelope.json"
COMMAND_SPACE_FILE = "command-space.json"
WAYPOINTS_FILE = "ev10-waypoints.json"

MAX_REPORT_BYTES = 20 * 1024 * 1024
"""report.json is refused above this (the results repository's per-file limit)."""
MAX_FILE_BYTES = 64 * 1024 * 1024
"""Any other file is refused above this."""
MAX_ENTRIES = 10_000
"""Files and directories a run directory may hold."""
MAX_DEPTH = 12
"""Directory levels below the run directory."""

_NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}")

# path -> (role, format) for the files at fixed places.
_FIXED: dict[str, tuple[str, str]] = {
    ENVELOPE_FILE: ("envelope", "json"),
    COMMAND_SPACE_FILE: ("command_space", "json"),
    WAYPOINTS_FILE: ("waypoints", "json"),
    "raw/samples.csv": ("samples", SAMPLES),
    "raw/samples.csv.gz": ("samples", SAMPLES),
    "raw/commands.csv": ("commands", COMMANDS),
    "raw/commands.csv.gz": ("commands", COMMANDS),
    "raw/state.csv": ("state", STATE),
    "raw/state.csv.gz": ("state", STATE),
    "raw/accuracy.csv": ("accuracy", ACCURACY),
    "notes/NOTES.md": ("notes", "markdown"),
}

PARSED_ROLES = frozenset(
    {"envelope", "command_space", "waypoints", "samples", "commands", "state", "accuracy"}
)
"""Roles whose content core reads; their bytes are kept after hashing."""


def classify(path: str) -> tuple[str, str] | None:
    """``(role, format)`` of a run-directory path (POSIX, relative), or None when the layout does
    not allow it. Every path component must be 1-128 of ``A-Z a-z 0-9 . _ -`` and start with a
    letter or digit (no hidden files)."""
    parts = path.split("/")
    if not all(_NAME.fullmatch(p) for p in parts):
        return None
    fixed = _FIXED.get(path)
    if fixed is not None:
        return fixed
    if len(parts) >= 3 and parts[0] == "raw" and parts[1] == "instrument":
        return ("instrument", "bytes")
    if (
        len(parts) == 3
        and parts[0] == "notes"
        and parts[1] == "photos"
        and parts[2].endswith(".jpg")
    ):
        return ("photo", "jpeg")
    return None


def show(path: str) -> str:
    """A path as printable ASCII-safe text: bytes that are not UTF-8 and control characters are
    written as escapes, so a hostile file name cannot break the output."""
    text = path.encode("utf-8", "surrogateescape").decode("utf-8", "backslashreplace")
    return re.sub(r"[\x00-\x1f\x7f]", lambda m: f"\\x{ord(m.group()):02x}", text)


@dataclass(frozen=True)
class RunFile:
    """One regular file: its path, size and ``sha256:<hex>``, and its bytes when kept."""

    path: str
    size: int
    sha256: str
    data: bytes | None = field(default=None, repr=False)


@dataclass
class RunFiles:
    """What :func:`scan` found. ``files`` holds every regular file but report.json, by path;
    ``refused`` the paths that exist but were not read (links, special files, too large)."""

    root: Path
    files: dict[str, RunFile] = field(default_factory=dict)
    report: RunFile | None = None
    refused: set[str] = field(default_factory=set)
    findings: list[Finding] = field(default_factory=list)
    complete: bool = True

    def present(self, path: str) -> bool:
        return path in self.files or path in self.refused

    def with_role(self, role: str) -> list[RunFile]:
        """The files of one role, by path."""
        out = []
        for path in sorted(self.files):
            kind = classify(path)
            if kind is not None and kind[0] == role:
                out.append(self.files[path])
        return out


def _error(code: str, detail: str, where: str = "") -> Finding:
    return Finding(code, "error", detail, where)


def _read(path: Path, rel: str, cap: int, keep: bool) -> tuple[RunFile | None, Finding | None]:
    flags = os.O_RDONLY
    for name in ("O_NOFOLLOW", "O_NONBLOCK", "O_BINARY", "O_NOCTTY"):
        flags |= getattr(os, name, 0)
    try:
        fd = os.open(path, flags)
    except OSError as e:
        if e.errno == errno.ELOOP:
            return None, _error(
                "SYMLINK_REFUSED",
                f"{show(rel)} is a symbolic link; links are not followed",
                show(rel),
            )
        return None, _error(
            "FILE_UNREADABLE", f"cannot read {show(rel)}: {e.strerror or e}", show(rel)
        )
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode):
            return None, _error(
                "NOT_A_REGULAR_FILE", f"{show(rel)} is not a regular file", show(rel)
            )
        if info.st_size > cap:
            return None, _error(
                "FILE_TOO_LARGE",
                f"{show(rel)} is {info.st_size} bytes; the limit is {cap}",
                show(rel),
            )
        digest = hashlib.sha256()
        chunks: list[bytes] = []
        total = 0
        while True:
            chunk = os.read(fd, 1 << 20)
            if not chunk:
                break
            total += len(chunk)
            if total > cap:
                return None, _error("FILE_TOO_LARGE", f"{show(rel)} is over {cap} bytes", show(rel))
            digest.update(chunk)
            if keep:
                chunks.append(chunk)
        data = b"".join(chunks) if keep else None
        return RunFile(rel, total, "sha256:" + digest.hexdigest(), data), None
    except OSError as e:
        return None, _error(
            "FILE_UNREADABLE", f"cannot read {show(rel)}: {e.strerror or e}", show(rel)
        )
    finally:
        os.close(fd)


def scan(run_dir: str | os.PathLike, *, read_report: bool = True) -> RunFiles:
    """Walk ``run_dir`` without following links and read every regular file in it once.

    report.json is read (up to MAX_REPORT_BYTES) when ``read_report``; every other file is hashed,
    and kept in memory when its role is one core parses. Never raises for anything found on
    disk: problems become findings."""
    root = Path(run_dir)
    out = RunFiles(root=root)
    try:
        info = os.lstat(root)
    except OSError as e:
        out.findings.append(
            _error("RUN_DIR_INVALID", f"cannot read the run directory: {e.strerror or e}")
        )
        out.complete = False
        return out
    if stat.S_ISLNK(info.st_mode):
        out.findings.append(
            _error(
                "RUN_DIR_INVALID", "the run directory is a symbolic link; links are not followed"
            )
        )
        out.complete = False
        return out
    if not stat.S_ISDIR(info.st_mode):
        out.findings.append(_error("RUN_DIR_INVALID", "the run directory is not a directory"))
        out.complete = False
        return out

    regular: list[tuple[str, Path]] = []
    stack: list[tuple[str, Path, int]] = [("", root, 0)]
    count = 0
    while stack:
        rel_dir, abs_dir, depth = stack.pop()
        try:
            with os.scandir(abs_dir) as it:
                entries = sorted(it, key=lambda e: e.name)
        except OSError as e:
            where = show(rel_dir) or "."
            out.findings.append(
                _error("FILE_UNREADABLE", f"cannot list {where}: {e.strerror or e}", where)
            )
            out.refused.add(rel_dir)
            continue
        for entry in entries:
            count += 1
            if count > MAX_ENTRIES:
                out.findings.append(
                    _error(
                        "RUN_DIR_TOO_LARGE",
                        f"the run directory holds more than {MAX_ENTRIES} entries",
                    )
                )
                out.complete = False
                stack.clear()
                break
            rel = f"{rel_dir}/{entry.name}" if rel_dir else entry.name
            try:
                if entry.is_symlink():
                    out.findings.append(
                        _error(
                            "SYMLINK_REFUSED",
                            f"{show(rel)} is a symbolic link; links are not followed",
                            show(rel),
                        )
                    )
                    out.refused.add(rel)
                    continue
                if entry.is_dir(follow_symlinks=False):
                    if depth + 1 > MAX_DEPTH:
                        out.findings.append(
                            _error(
                                "RUN_DIR_TOO_LARGE",
                                f"{show(rel)} is nested more than {MAX_DEPTH} levels deep",
                                show(rel),
                            )
                        )
                        out.refused.add(rel)
                        out.complete = False
                        continue
                    stack.append((rel, Path(entry.path), depth + 1))
                    continue
                if not entry.is_file(follow_symlinks=False):
                    out.findings.append(
                        _error(
                            "NOT_A_REGULAR_FILE", f"{show(rel)} is not a regular file", show(rel)
                        )
                    )
                    out.refused.add(rel)
                    continue
            except OSError as e:
                out.findings.append(
                    _error(
                        "FILE_UNREADABLE", f"cannot read {show(rel)}: {e.strerror or e}", show(rel)
                    )
                )
                out.refused.add(rel)
                continue
            regular.append((rel, Path(entry.path)))

    for rel, path in sorted(regular):
        if rel == REPORT_FILE:
            if not read_report:
                continue
            got, problem = _read(path, rel, MAX_REPORT_BYTES, keep=True)
            if problem is not None:
                out.findings.append(problem)
                out.refused.add(rel)
            else:
                out.report = got
            continue
        kind = classify(rel)
        got, problem = _read(
            path, rel, MAX_FILE_BYTES, keep=kind is not None and kind[0] in PARSED_ROLES
        )
        if problem is not None:
            out.findings.append(problem)
            out.refused.add(rel)
        else:
            assert got is not None
            out.files[rel] = got
    if read_report and out.report is None and REPORT_FILE not in out.refused:
        out.findings.append(
            _error("REPORT_MISSING", "the run directory has no report.json", REPORT_FILE)
        )
    return out


def raw_data_entries(files: RunFiles) -> list[dict]:
    """The report's ``raw_data``: every file the layout allows, sorted by path, with its
    sha256, size, role and format. Files the layout does not allow are left out (and refused
    elsewhere)."""
    entries = []
    for path in sorted(files.files):
        kind = classify(path)
        if kind is None:
            continue
        f = files.files[path]
        entries.append(
            {"path": path, "sha256": f.sha256, "bytes": f.size, "role": kind[0], "format": kind[1]}
        )
    return entries
