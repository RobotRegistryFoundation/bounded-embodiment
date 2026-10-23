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
    with pytest.raises(InterlockError, match="ssh -t"):
        require_terminal(io.StringIO("NEXT\n"))


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
