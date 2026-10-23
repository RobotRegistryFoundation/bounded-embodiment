# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""The typed words a hardware run asks for: only from a terminal, never answered by input typed
before the question, and ended by the latch (Ctrl+C) without waiting for Enter."""
from __future__ import annotations

import io
import os
import threading
import time

import pytest
from bounded_embodiment_kit.runner.interlocks import (
    InterlockError,
    check_command,
    require_terminal,
    typed_word,
)


def test_a_typed_word_at_a_terminal_ignores_what_was_typed_ahead_and_ends_on_the_latch():
    """Review S3 and N1, on a pseudo-terminal: input typed before the question is thrown away;
    the word typed after it goes ahead; a latch trip ends the question at once; and input that is
    not a terminal is refused."""
    master, slave = os.openpty()
    with os.fdopen(slave, "r", closefd=True) as tty:
        os.write(master, b"NEXT\n")  # typed ahead, before the question
        time.sleep(0.05)
        out = io.StringIO()
        threading.Timer(0.3, lambda: os.write(master, b"NEXT\n")).start()
        t0 = time.monotonic()
        assert typed_word("NEXT", "trial 1 done.", stdin=tty, out=out)
        assert time.monotonic() - t0 >= 0.25  # it waited for the word typed after the question
        assert "Type NEXT" in out.getvalue()
        os.write(master, b"next\n")
        time.sleep(0.05)
        tripped = threading.Event()
        threading.Timer(0.2, tripped.set).start()
        t0 = time.monotonic()
        assert not typed_word("NEXT", "trial 2 done.", stdin=tty, out=io.StringIO(),
                              cancelled=tripped.is_set)
        assert time.monotonic() - t0 < 1.0  # Ctrl+C (the latch) ends it without Enter
        require_terminal(tty)
    os.close(master)
    with pytest.raises(InterlockError, match="ssh -t") as refused:
        require_terminal(io.StringIO("NEXT\n"))
    assert "tmux or screen" in str(refused.value)


def test_a_typed_word_from_anything_but_a_terminal_is_refused():
    """typed_word cannot throw away input typed ahead unless stdin is a terminal: it refuses with
    InterlockError (the runner's refusal), not with termios.error or io.UnsupportedOperation."""
    r, w = os.pipe()
    with os.fdopen(r, "r") as pipe, os.fdopen(w, "w") as writer:
        writer.write("NEXT\n")
        writer.flush()
        with pytest.raises(InterlockError, match="not a terminal"):
            typed_word("NEXT", "trial 1 done.", stdin=pipe, out=io.StringIO())
    with pytest.raises(InterlockError, match="not a terminal"):
        typed_word("NEXT", "trial 1 done.", stdin=io.StringIO("NEXT\n"), out=io.StringIO())


def test_without_termios_a_hardware_run_is_refused_before_anything_connects(monkeypatch):
    """Round-2 review of #13, item 2: without termios every typed word was refused once the robot
    was connected, as a bare cancel; require_terminal now refuses first, and says why."""
    import sys

    master, slave = os.openpty()
    with os.fdopen(slave, "r", closefd=True) as tty:
        monkeypatch.setitem(sys.modules, "termios", None)  # import termios: ImportError
        with pytest.raises(InterlockError, match="needs a POSIX terminal"):
            require_terminal(tty)
    os.close(master)


def test_a_terminal_that_fails_while_asking_is_a_refusal_not_a_crash(monkeypatch):
    """Round-2 kit review N-e: an OSError from the terminal (EIO once the other end has hung up)
    left typed_word's read loop as an OSError, which nothing caught; it is an InterlockError now."""
    import errno
    import types

    from bounded_embodiment_kit.runner import interlocks

    def hung_up(*_):
        raise OSError(errno.EIO, "Input/output error")

    master, slave = os.openpty()
    with os.fdopen(slave, "r", closefd=True) as tty:
        monkeypatch.setattr(interlocks, "select", types.SimpleNamespace(select=hung_up))
        with pytest.raises(InterlockError, match="the terminal failed while asking for NEXT"):
            typed_word("NEXT", "trial 1 done.", stdin=tty, out=io.StringIO())
    os.close(master)


def test_a_command_is_checked_before_anything_connects(tmp_path):
    """Round-2 reviews (N-b of #12, item 4 of #13): a command the runner starts without a shell
    must split and name a program on PATH, or a typo is found only when the command is needed."""
    check_command(None, "--estop-cmd")
    check_command("true", "--estop-cmd")
    check_command(f"touch {tmp_path / 'estop'}", "--power-loss-cmd")
    for cmd, words in (("relay-of-no-such-name --cut", "is not a program on PATH"),
                       ("sh -c 'unbalanced", "does not split"), ("  ", "is empty")):
        with pytest.raises(InterlockError, match=words) as refused:
            check_command(cmd, "--power-loss-cmd")
        assert str(refused.value).startswith("--power-loss-cmd")
