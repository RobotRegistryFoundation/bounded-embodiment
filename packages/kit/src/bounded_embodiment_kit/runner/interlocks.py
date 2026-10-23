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
import shlex
import shutil
import sys
import time
from collections.abc import Callable
from pathlib import Path

from bounded_embodiment.canonical import sha256_jcs

__all__ = [
    "InterlockError",
    "Ledger",
    "check_command",
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
    typed before the question, could answer with nobody at the machine (review S3). ``typed_word``
    clears input typed ahead with termios: on a platform without it every typed word would be
    refused once the robot is connected, so the run is refused here instead, before anything
    connects, and says why (round-2 review of #13, item 2)."""
    stdin = sys.stdin if stdin is None else stdin
    try:
        import termios  # noqa: F401
    except ImportError:
        raise InterlockError("a hardware run needs a POSIX terminal: its typed words clear input "
                             "typed ahead with termios, which this platform does not have") from None
    try:
        tty = bool(stdin.isatty())
    except (AttributeError, ValueError, OSError):
        tty = False
    if not tty:
        raise InterlockError("a hardware run needs its typed words from a terminal: run it in one "
                             "(ssh -t for a remote machine, inside tmux or screen so that a dropped "
                             "connection does not end the session), not with piped or redirected input")


def check_command(cmd: str | None, flag: str) -> None:
    """A command the runner starts without a shell (``shlex.split``), checked before anything
    connects: it must split, and its program must be found (``shutil.which``), so that a typo is
    found now rather than when the command is needed (round-2 reviews: N-b for the stop sources'
    commands, item 4 of #13 for ``--estop-cmd``). Nothing is run. None (not given) passes."""
    if cmd is None:
        return
    try:
        argv = shlex.split(cmd)
    except ValueError as e:
        raise InterlockError(f"{flag} {cmd!r} does not split into a program and its arguments: {e}") from None
    if not argv:
        raise InterlockError(f"{flag} is empty")
    if shutil.which(argv[0]) is None:
        raise InterlockError(f"{flag}: {argv[0]!r} is not a program on PATH, or not executable; the runner "
                             "starts it without a shell, and it would fail only when it is needed")


def typed_word(word: str, text: str, *, cancelled: Callable[[], bool] = lambda: False,
               stdin=None, out=None, poll_s: float = 0.1) -> bool:
    """Ask for a typed word at a terminal (review S3, N1). Input typed before the question is
    thrown away first (``tcflush``), the answer is read by polling, so that ``cancelled()`` (the
    latch, Ctrl+C) ends the question within ``poll_s`` instead of waiting for Enter, and only the
    exact word (surrounding spaces ignored) goes ahead; anything else, end of input or
    ``cancelled()`` is False. One helper for every typed word: NEXT uses it now on hardware, and
    RUN and GO can adopt it. POSIX only, from a terminal only: anything else is refused
    (InterlockError), never answered, and so is a terminal that fails while it asks (an OSError
    such as EIO after a hangup)."""
    stdin = sys.stdin if stdin is None else stdin
    out = sys.stderr if out is None else out
    try:
        import termios
    except ImportError:  # pragma: no cover - not POSIX
        raise InterlockError("cannot clear input typed ahead: no termios on this platform") from None
    try:
        fd = stdin.fileno()
        termios.tcflush(fd, termios.TCIFLUSH)
    except (termios.error, OSError, ValueError) as e:
        raise InterlockError(f"cannot clear input typed ahead (not a terminal?): {e}") from None
    out.write(f"{text}\nType {word} and Enter to start (anything else cancels): ")
    out.flush()
    buf = b""
    while True:
        if cancelled():
            out.write("\n")
            out.flush()
            return False
        try:
            ready, _, _ = select.select([fd], [], [], poll_s)
            if not ready:
                continue
            chunk = os.read(fd, 1024)
        except (OSError, ValueError) as e:  # the terminal went away (EIO after a hangup): no answer
            raise InterlockError(f"the terminal failed while asking for {word}: {e}") from None
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
