# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""Interlocks: refusals without the flags or the typed word, the shakedown ledger, and starting
only when the instrument sees the arm inside and still."""
from __future__ import annotations

import argparse
import dataclasses
import json
import re
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import _runner_fakes as fakes
from bounded_embodiment import rawdata
from bounded_embodiment_kit import cli
from bounded_embodiment_kit.runner import run as runmod
from bounded_embodiment_kit.runner.interlocks import (
    InterlockError,
    Ledger,
    confirm,
    require_run_flags,
)
from bounded_embodiment_kit.runner.limits import DEFAULT


def _parse(*argv):
    p = argparse.ArgumentParser(prog="be")
    cli.register(p.add_subparsers(dest="command"))
    return p.parse_args(list(argv))


def _run_args(tmp_path, *extra, mode="shakedown", flags=True, sim_start="start"):
    base = ["run", "ev03", "--mode", mode, "--adapter", "sim-so101",
            "--envelope", str(fakes.envelope_path("so101")), "--command-space", str(fakes.SO101_SPACE),
            "--kit-config", str(fakes.ENVELOPES / "so101.kit.json"), "--urdf", str(fakes.URDF),
            "--instrument", "sim", "--out", str(tmp_path / "run"), "--seconds", "1",
            "--ledger", str(tmp_path / "ledger.jsonl"), "--sim-start", sim_start]
    if flags:
        base += ["--i-accept-the-risk", "--hardware-stop", "none (simulation)"]
    return _parse(*base, *extra)


class Prompt:
    def __init__(self, *answers):
        self.answers, self.asked = list(answers), []

    def __call__(self, question):
        self.asked.append(question)
        return self.answers.pop(0) if self.answers else ""


def test_flags_are_required_before_anything_connects():
    ns = argparse.Namespace(i_accept_the_risk=False, hardware_stop="hand on plug")
    with pytest.raises(InterlockError, match="--i-accept-the-risk"):
        require_run_flags(ns, "shakedown")
    ns = argparse.Namespace(i_accept_the_risk=True, hardware_stop="  ")
    with pytest.raises(InterlockError, match="--hardware-stop"):
        require_run_flags(ns, "full")
    require_run_flags(argparse.Namespace(hardware_stop="hand on plug"), "goto-start")  # GO only


def test_a_run_without_the_flags_is_refused_and_opens_nothing(tmp_path):
    prompt = Prompt("RUN")
    code = runmod.execute_run(_run_args(tmp_path, flags=False), prompt=prompt, log=lambda m: None)
    assert code == runmod.EXIT_REFUSED
    assert prompt.asked == []  # never got as far as asking
    assert not (tmp_path / "run").exists()


def test_typed_words_must_match_exactly():
    assert confirm(lambda q: " RUN ", "RUN", "x")
    for answer in ("run", "RUN!", "", "GO"):
        assert not confirm(lambda q, a=answer: a, "RUN", "x")

    def eof(q):
        raise EOFError

    def ctrl_c(q):
        raise KeyboardInterrupt

    assert not confirm(eof, "RUN", "x") and not confirm(ctrl_c, "RUN", "x")


def test_a_wrong_typed_word_cancels_with_no_command_and_no_stop(tmp_path):
    prompt = Prompt("yes", "")
    code = runmod.execute_run(_run_args(tmp_path), prompt=prompt, log=lambda m: None)
    assert code == runmod.EXIT_STOPPED
    assert rawdata.read_commands(tmp_path / "run" / "raw" / "commands.csv") == []
    declared = json.loads((tmp_path / "run" / "declared.json").read_text())
    assert declared["end"]["status"] == "cancelled" and declared["end"]["stop_calls"] == 0
    assert "OFF" in prompt.asked[-1]  # the adapter was still closed with the typed-OFF question


def test_the_ledger_only_vouches_for_a_whole_recent_trip_free_shakedown_on_this_setup(tmp_path):
    ledger = Ledger(tmp_path / "l.jsonl")
    now = 1_000_000.0
    good = {"setup_hash": "sha256:a", "result": "completed", "seconds": 60.0, "ended_wall": now - 3600}
    ledger.record(good)
    ledger.record({**good, "setup_hash": "sha256:b"})
    ledger.record({**good, "result": "stopped", "ended_wall": now - 10})
    with ledger.path.open("a") as fh:
        fh.write('{"setup_hash": "sha256:a", "result": "comp')  # a torn last line
    find = ledger.find_shakedown
    assert find("sha256:a", max_age_s=12 * 3600, min_seconds=60, now_wall=now) == good
    assert find("sha256:c", max_age_s=12 * 3600, min_seconds=60, now_wall=now) is None
    assert find("sha256:a", max_age_s=1800, min_seconds=60, now_wall=now) is None  # too old
    assert find("sha256:a", max_age_s=12 * 3600, min_seconds=61, now_wall=now) is None  # too short
    assert find("sha256:a", max_age_s=12 * 3600, min_seconds=60, now_wall=now - 7200) is None


def test_a_full_run_needs_a_shakedown_on_the_same_setup(tmp_path):
    logs = []
    code = runmod.execute_run(_run_args(tmp_path, mode="full"), prompt=Prompt("RUN"), log=logs.append)
    assert code == runmod.EXIT_REFUSED and not (tmp_path / "run").exists()
    setup = re.search(r"\((sha256:[0-9a-f]{64})\)", " ".join(logs)).group(1)

    # A shakedown shorter than 60 s (--seconds 1) is recorded but does not count.
    code = runmod.execute_run(_run_args(tmp_path / "s"), prompt=Prompt("RUN", ""), log=lambda m: None)
    assert code in (runmod.EXIT_OK, runmod.EXIT_ESCALATED)
    entries = Ledger(tmp_path / "s" / "ledger.jsonl").entries()
    assert entries[-1]["setup_hash"] == setup and entries[-1]["seconds"] == 1.0

    Ledger(tmp_path / "ledger.jsonl").record({"setup_hash": setup, "result": "completed",
                                              "seconds": 60.0, "ended_wall": time.time() - 60})
    code = runmod.execute_run(_run_args(tmp_path, mode="full"), prompt=Prompt("RUN", ""),
                              log=lambda m: None)
    assert code != runmod.EXIT_REFUSED
    declared = json.loads((tmp_path / "run" / "declared.json").read_text())
    assert declared["safety"]["shakedown_used"]["setup_hash"] == setup


def test_a_different_setup_needs_its_own_shakedown(tmp_path):
    logs = []
    runmod.execute_run(_run_args(tmp_path, mode="full"), prompt=Prompt(), log=logs.append)
    setup = re.search(r"\((sha256:[0-9a-f]{64})\)", " ".join(logs)).group(1)
    Ledger(tmp_path / "ledger.jsonl").record({"setup_hash": setup, "result": "completed",
                                              "seconds": 60.0, "ended_wall": time.time()})
    other = _run_args(tmp_path, "--sim-sag", "0.5", mode="full")  # the adapter config changed
    assert runmod.execute_run(other, prompt=Prompt("RUN"), log=lambda m: None) == runmod.EXIT_REFUSED


def test_the_run_never_starts_unless_the_instrument_sees_the_arm_still(tmp_path):
    """3 mm of instrument jitter: the tool never looks still to 5 mm, so the runner waits, then
    gives up without sending a command (and without a stop: nothing was commanded)."""
    limits = dataclasses.replace(DEFAULT, start_timeout_s=3.0)
    args = _run_args(tmp_path, "--sim-sigma-m", "0.003")
    code = runmod.execute_run(args, prompt=Prompt("RUN", ""), log=lambda m: None, limits=limits)
    assert code == runmod.EXIT_STOPPED
    assert rawdata.read_commands(tmp_path / "run" / "raw" / "commands.csv") == []
    declared = json.loads((tmp_path / "run" / "declared.json").read_text())
    assert declared["end"]["status"] == "cancelled" and declared["end"]["stop_calls"] == 0
    assert "not inside and still" in declared["end"]["why"]


def test_an_arm_outside_the_margin_trips_before_any_command(tmp_path):
    """Folded at rest the simulated arm is far outside the example envelope: the FK tripwire stops
    it while the runner is still waiting, and no command is ever sent."""
    args = _run_args(tmp_path, sim_start="rest")
    code = runmod.execute_run(args, prompt=Prompt("RUN", ""), log=lambda m: None)
    assert code in (runmod.EXIT_STOPPED, runmod.EXIT_ESCALATED)
    assert rawdata.read_commands(tmp_path / "run" / "raw" / "commands.csv") == []
    declared = json.loads((tmp_path / "run" / "declared.json").read_text())
    assert declared["end"]["latch"]["kind"] == "tripwire" and not declared["end"]["started"]
    assert "outside" in declared["end"]["latch"]["reason"]


def test_runs_without_fk_need_the_override_and_it_is_recorded(tmp_path):
    p = _parse("run", "ev03", "--mode", "shakedown", "--adapter", "sim-so101",
               "--envelope", str(fakes.envelope_path("so101")), "--command-space", str(fakes.SO101_SPACE),
               "--kit-config", str(fakes.ENVELOPES / "so101.kit.json"), "--instrument", "sim",
               "--out", str(tmp_path / "run"), "--i-accept-the-risk", "--hardware-stop", "x")
    assert runmod.execute_run(p, prompt=Prompt(), log=lambda m: None) == runmod.EXIT_REFUSED
    args = _run_args(tmp_path, "--no-fk-tripwire-i-understand")
    runmod.execute_run(args, prompt=Prompt("RUN", ""), log=lambda m: None)
    declared = json.loads((tmp_path / "run" / "declared.json").read_text())
    assert declared["safety"]["fk_tripwire"] == "override"
    assert declared["safety"]["tripwires"]["tool_unseen_s"] == DEFAULT.tool_unseen_no_fk_s


def test_a_run_directory_is_written_once(tmp_path):
    (tmp_path / "run").mkdir()
    (tmp_path / "run" / "declared.json").write_text("{}")
    assert runmod.execute_run(_run_args(tmp_path), prompt=Prompt("RUN"), log=lambda m: None) == \
        runmod.EXIT_REFUSED
