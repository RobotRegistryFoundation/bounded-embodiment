# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""Names in the results repository: lab ids, run directory names and result ids.

A run directory is ``<test>-<UTC yyyymmddThhmmssZ>-s<seed>`` (results/README.md): the report's
``test`` in lower case without punctuation, ``run.started_at`` to the second, and
``generator.seed``. A result id is ``<lab>/<run directory>``.
"""
from __future__ import annotations

import re
from collections.abc import Mapping
from pathlib import Path

from bounded_embodiment.canonical import CanonicalError, loads_strict

__all__ = ["LAB_ID_RE", "RUN_DIR_RE", "IdError", "control_run_id", "reproduces_id", "run_dir_name"]

# As the results repository's tools/_common.py and the report schema write them.
LAB_ID_RE = re.compile(r"[a-z0-9](?:[a-z0-9-]{0,62}[a-z0-9])?")
RUN_DIR_RE = re.compile(r"[a-z0-9]+-[0-9]{8}T[0-9]{6}Z-s[0-9]{1,20}")
_STARTED = re.compile(r"([0-9]{4})-([0-9]{2})-([0-9]{2})T([0-9]{2}):([0-9]{2}):([0-9]{2})(?:\.[0-9]+)?Z")


class IdError(ValueError):
    pass


def run_dir_name(report: Mapping) -> str:
    """The run directory a report goes in: ``ev03-20261013T183000Z-s7``."""
    test = report.get("test")
    started = report.get("run", {}).get("started_at") if isinstance(report.get("run"), Mapping) else None
    gen = report.get("generator")
    seed = gen.get("seed") if isinstance(gen, Mapping) else None
    m = _STARTED.fullmatch(started) if isinstance(started, str) else None
    if not isinstance(test, str) or m is None or isinstance(seed, bool) or not isinstance(seed, int):
        raise IdError("the report needs test, run.started_at and generator.seed to name its run directory")
    name = f"{re.sub(r'[^a-z0-9]', '', test.lower())}-{''.join(m.groups()[:3])}T{''.join(m.groups()[3:])}Z-s{seed}"
    if not RUN_DIR_RE.fullmatch(name):
        raise IdError(f"{name!r} is not a run directory name")
    return name


def _parts(text: str) -> list[str]:
    return [p for p in re.split(r"[/\\]", text.strip()) if p]


def _from_submission(path: Path) -> str | None:
    """The run directory name of a local submission (a directory with a report.json)."""
    report = path / "report.json"
    if not report.is_file():
        return None
    try:
        doc = loads_strict(report.read_bytes())
    except (OSError, CanonicalError) as e:
        raise IdError(f"cannot read {report}: {e}") from None
    if not isinstance(doc, Mapping):
        raise IdError(f"{report} is not a report")
    return run_dir_name(doc)


def reproduces_id(text: str) -> str:
    """``--reproduces``: a result id ``<lab>/<run directory>``, or a path ending in
    ``results/<lab>/<run directory>`` in a clone of the results repository."""
    parts = _parts(text)
    if len(parts) >= 2 and LAB_ID_RE.fullmatch(parts[-2]) and RUN_DIR_RE.fullmatch(parts[-1]) and (
            len(parts) == 2 or parts[-3] == "results"):
        return f"{parts[-2]}/{parts[-1]}"
    raise IdError(f"--reproduces {text!r}: expected a result id <lab>/<test>-<UTC>-s<seed>, or a path "
                  "to results/<lab>/<run> in the results repository")


def control_run_id(text: str) -> str:
    """``--control-run``: ``<run directory>``, ``<lab>/<run directory>``, a path ending in
    ``results/<lab>/<run directory>``, or a local submission directory (named by its report)."""
    path = Path(text)
    if path.is_dir():
        name = _from_submission(path)
        if name is not None:
            parts = _parts(str(path.resolve()))
            if len(parts) >= 3 and parts[-3] == "results" and parts[-1] == name and LAB_ID_RE.fullmatch(parts[-2]):
                return f"{parts[-2]}/{name}"
            return name
    parts = _parts(text)
    if len(parts) == 1 and RUN_DIR_RE.fullmatch(parts[0]):
        return parts[0]
    if len(parts) >= 2 and LAB_ID_RE.fullmatch(parts[-2]) and RUN_DIR_RE.fullmatch(parts[-1]) and (
            len(parts) == 2 or parts[-3] == "results"):
        return f"{parts[-2]}/{parts[-1]}"
    raise IdError(f"--control-run {text!r}: expected <test>-<UTC>-s<seed>, <lab>/<test>-<UTC>-s<seed>, "
                  "a path to results/<lab>/<run>, or a submission directory made by be report")
