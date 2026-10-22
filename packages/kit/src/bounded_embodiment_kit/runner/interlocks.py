# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""DRAFT — requires line-by-line human review before use on hardware.

Interlocks: what must be true, and typed, before the runner moves anything.

- ``be run`` needs ``--i-accept-the-risk`` and ``--hardware-stop "<what>"`` (recorded in
  declared.json), then a typed ``RUN``. ``be goto-start`` needs ``--hardware-stop`` and a typed
  ``GO``. A typed word must match exactly; anything else, or end of input, cancels.
- A ``full`` run needs a shakedown on the same setup hash that ran its whole 60 s and ended
  without a trip, at most 12 h ago, recorded in a small local ledger (JSON lines). The setup hash
  covers the envelope, the command space, the adapter's config, the kit config and the URDF: a
  change to any of them needs a new shakedown.
- The run starts only once the instrument sees the arm inside and still (``watchdog.inside_and_still``).
"""
from __future__ import annotations

import hashlib
import json
import os
import select
import sys
import time
from collections.abc import Callable
from pathlib import Path

from bounded_embodiment.canonical import sha256_jcs

__all__ = [
    "InterlockError",
    "Ledger",
    "confirm",
    "default_ledger_path",
    "file_sha256",
    "require_run_flags",
    "require_terminal",
    "setup_hash",
    "typed_word",
]


class InterlockError(RuntimeError):
    """A refusal before any motion. The message says what is missing."""


def require_run_flags(args, mode: str) -> None:
    """The flags every run needs, checked before anything connects."""
    missing = []
    if mode != "goto-start" and not getattr(args, "i_accept_the_risk", False):
        missing.append("--i-accept-the-risk (this sends hostile commands to a machine)")
    stop = (getattr(args, "hardware_stop", None) or "").strip()
    if not stop:
        missing.append('--hardware-stop "<what stops it: e.g. hand on the servo power plug>"')
    if missing:
        raise InterlockError("Refusing to start. Add:\n  " + "\n  ".join(missing))


def confirm(prompt: Callable[[str], str], word: str, text: str) -> bool:
    """Ask for a typed word. Only the exact word (surrounding spaces ignored) goes ahead."""
    try:
        answer = prompt(f"{text}\nType {word} and Enter to start (anything else cancels): ")
    except (EOFError, KeyboardInterrupt):
        return False
    return answer.strip() == word


def require_terminal(stdin=None) -> None:
    """A hardware run takes its typed words from a terminal: piped or redirected input, or input
    typed before the question, could answer with nobody at the machine (review S3)."""
    stdin = sys.stdin if stdin is None else stdin
    try:
        tty = bool(stdin.isatty())
    except (AttributeError, ValueError, OSError):
        tty = False
    if not tty:
        raise InterlockError("a hardware run needs its typed words from a terminal: run it in one "
                             "(ssh -t for a remote machine), not with piped or redirected input")


def typed_word(word: str, text: str, *, cancelled: Callable[[], bool] = lambda: False,
               stdin=None, out=None, poll_s: float = 0.1) -> bool:
    """Ask for a typed word at a terminal (review S3, N1). Input typed before the question is
    thrown away first (``tcflush``), the answer is read by polling, so that ``cancelled()`` (the
    latch, Ctrl+C) ends the question within ``poll_s`` instead of waiting for Enter, and only the
    exact word (surrounding spaces ignored) goes ahead; anything else, end of input or
    ``cancelled()`` is False. One helper for every typed word: NEXT uses it now on hardware, and
    RUN and GO can adopt it."""
    stdin = sys.stdin if stdin is None else stdin
    out = sys.stderr if out is None else out
    fd = stdin.fileno()
    try:
        import termios

        termios.tcflush(fd, termios.TCIFLUSH)
    except (ImportError, OSError) as e:  # pragma: no cover - not a terminal, or not POSIX
        raise InterlockError(f"cannot clear input typed ahead: {e}") from None
    out.write(f"{text}\nType {word} and Enter to start (anything else cancels): ")
    out.flush()
    buf = b""
    while True:
        if cancelled():
            out.write("\n")
            out.flush()
            return False
        ready, _, _ = select.select([fd], [], [], poll_s)
        if not ready:
            continue
        chunk = os.read(fd, 1024)
        if not chunk:  # end of input
            return False
        buf += chunk
        if b"\n" in buf:
            return buf.split(b"\n", 1)[0].decode("utf-8", "replace").strip() == word


def file_sha256(path: str | Path | None) -> str | None:
    if path is None:
        return None
    return "sha256:" + hashlib.sha256(Path(path).read_bytes()).hexdigest()


def setup_hash(*, envelope_hash: str, command_space_digest: str, adapter_config: dict,
               kit_config: dict, urdf_sha256: str | None) -> str:
    """sha256 of the RFC 8785 JSON of everything a shakedown vouches for."""
    return sha256_jcs({"envelope": envelope_hash, "command_space": command_space_digest,
                       "adapter": adapter_config, "kit_config": kit_config, "urdf": urdf_sha256})


def default_ledger_path() -> Path:
    base = os.environ.get("XDG_STATE_HOME") or str(Path.home() / ".local" / "state")
    return Path(base) / "bounded-embodiment" / "shakedown-ledger.jsonl"


class Ledger:
    """Shakedown results, one JSON object per line. Written only by the runner; read before a
    full run. Lines that do not parse are skipped (a torn last line must not unlock anything)."""

    def __init__(self, path: str | Path):
        self.path = Path(path)

    def entries(self) -> list[dict]:
        if not self.path.exists():
            return []
        out = []
        for line in self.path.read_text(encoding="utf-8").splitlines():
            try:
                item = json.loads(line)
            except ValueError:
                continue
            if isinstance(item, dict):
                out.append(item)
        return out

    def record(self, entry: dict) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(entry, sort_keys=True) + "\n")
            fh.flush()
            os.fsync(fh.fileno())

    def find_shakedown(self, setup: str, *, max_age_s: float, min_seconds: float,
                       now_wall: float | None = None) -> dict | None:
        """The newest qualifying shakedown for this setup, or None: same setup hash, completed
        (the whole stream sent, no trip), at least ``min_seconds`` long, ended within
        ``max_age_s`` (wall clock) and not in the future."""
        now_wall = time.time() if now_wall is None else now_wall
        best = None
        for e in self.entries():
            ok = (e.get("setup_hash") == setup and e.get("result") == "completed"
                  and isinstance(e.get("seconds"), (int, float)) and e["seconds"] >= min_seconds
                  and isinstance(e.get("ended_wall"), (int, float))
                  and 0 <= now_wall - e["ended_wall"] <= max_age_s)
            if ok and (best is None or e["ended_wall"] > best["ended_wall"]):
                best = e
        return best
