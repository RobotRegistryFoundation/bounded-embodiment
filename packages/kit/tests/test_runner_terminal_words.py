# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""RUN and GO on hardware (from the review of PR #12: S3 and N1, on main's own paths): the typed
word comes from a terminal only, input typed before the question never answers it, and Ctrl+C at
the question cancels at once. And (round-2 review of #13, item 4) the escalation command is
checked before anything connects.

The cases drive sim-so101 declared as hardware (``describe()["simulation"]`` False) on a
pseudo-terminal: goto-start, and a 1 s EV-03 full run with no instrument (the robot's own state
supervises) on a planted shakedown. Nothing here touches a real robot, port or network."""
from __future__ import annotations

import argparse
import io
import os
import signal
import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import _runner_fakes as fakes
from bounded_embodiment_kit import cli
from bounded_embodiment_kit.adapters import sim_so101
from bounded_embodiment_kit.runner import run as runmod

GOTO = ["goto-start", "--adapter", "sim-so101", "--sim-start", "rest",
        "--envelope", str(fakes.envelope_path("so101")), "--command-space", str(fakes.SO101_SPACE),
        "--kit-config", str(fakes.ENVELOPES / "so101.kit.json"), "--urdf", str(fakes.URDF),
        "--hardware-stop", "none (simulation, declared as hardware for this test)"]
RUN = ["run", "ev03", "--mode", "full", "--adapter", "sim-so101",
       "--instrument", "none", "--no-live-instrument-i-understand",
       "--envelope", str(fakes.envelope_path("so101")), "--command-space", str(fakes.SO101_SPACE),
       "--kit-config", str(fakes.ENVELOPES / "so101.kit.json"), "--urdf", str(fakes.URDF),
       "--i-accept-the-risk", "--hardware-stop", "none (simulation, declared as hardware for this test)",
       "--seconds", "1"]


def _be(*argv):
    p = argparse.ArgumentParser(prog="be")
    cli.register(p.add_subparsers(dest="command"))
    return p.parse_args(list(argv))


@pytest.fixture
def as_hardware(monkeypatch):
    """sim-so101, declared as hardware, counting the commands it is sent."""
    sent = []
    real = sim_so101.from_args

    def from_args(a, c):
        adapter = real(a, c)
        describe, send = adapter.describe, adapter.send
        adapter.describe = lambda: {**describe(), "simulation": False}

        def counting_send(cmd):
            sent.append(cmd)
            return send(cmd)

        adapter.send = counting_send
        return adapter

    monkeypatch.setattr(sim_so101, "from_args", from_args)
    return sent


class _Screen(io.StringIO):
    """stderr, as the operator sees it: ``asked`` is set once a typed word is asked for there."""

    def __init__(self, word: str):
        super().__init__()
        self.word, self.asked = word, threading.Event()

    def write(self, s):
        n = super().write(s)
        if f"Type {self.word}" in self.getvalue():
            self.asked.set()
        return n


def _terminal(monkeypatch, word: str):
    """A pseudo-terminal as stdin, and the screen the question for ``word`` is written to: (type,
    screen), and what to close. (pytest swaps sys.stderr back for each test phase, so the screen is
    handed to typed_word.)"""
    from bounded_embodiment_kit.runner import interlocks

    master, slave = os.openpty()
    tty = os.fdopen(slave, "r", closefd=True)
    screen = _Screen(word)
    monkeypatch.setattr(sys, "stdin", tty)
    monkeypatch.setattr(runmod, "typed_word",
                        lambda w, text, **kw: interlocks.typed_word(w, text, out=screen, **kw),
                        raising=False)
    return (lambda text: os.write(master, text.encode())), screen, (tty, master)


@pytest.fixture
def terminal(monkeypatch):
    type_, screen, (tty, master) = _terminal(monkeypatch, "GO")
    yield type_, screen
    tty.close()
    os.close(master)


@pytest.fixture
def run_terminal(monkeypatch):
    type_, screen, (tty, master) = _terminal(monkeypatch, "RUN")
    yield type_, screen
    tty.close()
    os.close(master)


def _run_args(tmp_path: Path, *extra: str) -> argparse.Namespace:
    from _b6 import plant_shakedown

    args = _be(*RUN, "--out", str(tmp_path / "run"), "--ledger", str(tmp_path / "ledger.jsonl"), *extra)
    plant_shakedown(args, tmp_path / "ledger.jsonl")
    return args


def _when_asked(screen, action, *, then=None, timeout_s=20.0):
    def run():
        if screen.asked.wait(timeout_s):
            time.sleep(0.2)
            action()
            if then is not None:
                time.sleep(0.5)
                then()

    t = threading.Thread(target=run, daemon=True)
    t.start()
    return t


def _read_line(question: str) -> str:
    """An operator answering every other question (OFF at close) at the same terminal."""
    return sys.stdin.readline().strip()


def test_a_hardware_goto_start_refuses_piped_input_before_anything_moves(as_hardware, monkeypatch):
    monkeypatch.setattr(sys, "stdin", io.StringIO("GO\n"))
    logs = []
    assert runmod.execute_goto_start(_be(*GOTO), log=logs.append) == runmod.EXIT_REFUSED
    assert any("ssh -t" in m for m in logs), logs
    assert as_hardware == []


def test_go_typed_ahead_never_starts_a_hardware_goto_start(as_hardware, terminal):
    """Review S3: input typed before the question is thrown away, and the word typed after it
    decides (here: not GO, so nothing moves). Before this change the GO typed ahead (and the Enter
    after it, for the OFF question) started the arm."""
    type_, screen = terminal
    type_("GO\n\n")  # typed ahead, before the question
    _when_asked(screen, lambda: type_("no\n"), then=lambda: type_("\n"))
    logs = []
    code = runmod.execute_goto_start(_be(*GOTO), prompt=_read_line, log=logs.append)
    assert code == runmod.EXIT_STOPPED, logs
    assert as_hardware == []


def test_go_typed_after_the_question_starts_a_hardware_goto_start(as_hardware, terminal):
    type_, screen = terminal
    _when_asked(screen, lambda: type_("GO\n"))
    logs = []
    code = runmod.execute_goto_start(_be(*GOTO), prompt=lambda q: "", log=logs.append)
    assert code == runmod.EXIT_OK, logs
    assert as_hardware


def test_ctrl_c_at_go_cancels_at_once(as_hardware, terminal):
    """Review N1, for RUN and GO: Ctrl+C at the question cancels without waiting for Enter."""
    _, screen = terminal
    _when_asked(screen, lambda: os.kill(os.getpid(), signal.SIGINT))
    t0 = time.monotonic()
    code = runmod.execute_goto_start(_be(*GOTO), prompt=lambda q: "", log=lambda m: None)
    assert code == runmod.EXIT_STOPPED and screen.asked.is_set()
    assert time.monotonic() - t0 < 15.0 and as_hardware == []


# ----- RUN, on the run path (round-2 review of #13, item 1) ---------------------------------------


def test_a_hardware_run_refuses_piped_input_before_anything_moves(as_hardware, monkeypatch, tmp_path):
    monkeypatch.setattr(sys, "stdin", io.StringIO("RUN\n"))
    logs = []
    assert runmod.execute_run(_run_args(tmp_path), log=logs.append) == runmod.EXIT_REFUSED
    assert any("ssh -t" in m for m in logs), logs
    assert as_hardware == []


def test_run_typed_ahead_never_starts_a_hardware_run(as_hardware, run_terminal, tmp_path):
    """Review S3, on the run path: the RUN typed before the question is thrown away, and the word
    typed after it decides (here: not RUN, so nothing is sent)."""
    type_, screen = run_terminal
    type_("RUN\n\n")  # typed ahead, before the question
    _when_asked(screen, lambda: type_("no\n"), then=lambda: type_("\n"))
    logs = []
    code = runmod.execute_run(_run_args(tmp_path), prompt=_read_line, log=logs.append)
    assert code == runmod.EXIT_STOPPED, logs
    assert as_hardware == []


def test_run_typed_after_the_question_starts_a_hardware_run(as_hardware, run_terminal, tmp_path):
    """The run starts and commands are sent (the hostile stream may then trip the simulated arm's
    joint-speed tripwire, which stops it: either way it ran)."""
    type_, screen = run_terminal
    _when_asked(screen, lambda: type_("RUN\n"))
    logs = []
    code = runmod.execute_run(_run_args(tmp_path), prompt=lambda q: "", log=logs.append)
    assert code in (runmod.EXIT_OK, runmod.EXIT_STOPPED), logs
    assert len(as_hardware) > 10 and not any("cancelled" in m for m in logs), logs


# ----- the escalation command, checked first (item 4) ---------------------------------------------


def test_the_escalation_command_is_checked_before_anything_connects(as_hardware, tmp_path):
    """Item 4: a typo in --estop-cmd was found only when CUT POWER fired, recorded as
    FileNotFoundError in the escalation's error. It is refused before anything connects, by run and
    by goto-start."""
    logs = []
    args = _run_args(tmp_path, "--estop-cmd", "relay-of-no-such-name --cut")
    assert runmod.execute_run(args, log=logs.append) == runmod.EXIT_REFUSED
    assert any("--estop-cmd: 'relay-of-no-such-name' is not a program on PATH" in m for m in logs), logs
    logs = []
    goto = _be(*GOTO, "--estop-cmd", "sh -c 'unbalanced")
    assert runmod.execute_goto_start(goto, log=logs.append) == runmod.EXIT_REFUSED
    assert any("--estop-cmd" in m and "does not split" in m for m in logs), logs
    assert as_hardware == []
