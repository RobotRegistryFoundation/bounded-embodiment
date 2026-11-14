# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""Round-2 review K6: one ``execute_run``-level test per wiring point that no test failed without.

The others are elsewhere: the room check (``rover-ev02-cornered``), the catch-up guard
(``test_a_dead_stop_on_an_arm_fails_and_never_passes`` requires its reason) and lerobot's
``reset_stop`` passing the source (``test_adapter_trial_latch.py``). Here: the check of every
command against the zones and the speed limit before RUN, and on hardware (sim-so101 declared as
hardware, ``describe()["simulation"]`` False, with no instrument) a terminal for the trial run and
NEXT as a typed word. Nothing here touches a real robot, port or network."""
from __future__ import annotations

import io
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import _trials_sim
from _b6 import plant_shakedown
from _virtual_time import VirtualClock, virtual_time
from bounded_embodiment_kit.adapters import sim_so101
from bounded_embodiment_kit.runner import run as runmod


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


def _so101_ev02(tmp_path: Path, *extra: str, hardware: bool = False):
    case = _trials_sim.Case("wiring", "so101", "ev02", "_trial_gates:stoppable", plan="so101-ev02",
                            extra=tuple(extra))
    args, _ = _trials_sim.be_args(case, tmp_path)
    if hardware:  # no simulated instrument for hardware: the robot's own state supervises
        args.instrument, args.no_live_instrument_i_understand = "none", True
    plant_shakedown(args, tmp_path / "ledger.jsonl")
    return args


def test_a_plan_whose_commands_outrun_the_speed_limit_is_refused_before_anything_connects(tmp_path):
    """check_trial_proposals (review S2): 6 deg a command at 50 Hz carries the tool far faster
    than its speed limit allows."""
    args = _so101_ev02(tmp_path, "--step-cap", "6", "--drive-s", "0.18")
    logs = []
    code = runmod.execute_run(args, prompt=lambda q: "", log=logs.append)
    assert code == runmod.EXIT_REFUSED, logs
    assert any("faster than its speed limit allows" in m and "a smaller --step-cap" in m for m in logs), logs
    assert not (tmp_path / "run" / "raw" / "commands.csv").exists()


def test_a_hardware_trial_run_refuses_piped_input_before_anything_moves(tmp_path, as_hardware, monkeypatch):
    """require_terminal (review S3): NEXT asserts a reset, so a hardware trial run takes it from a
    terminal only, and refuses piped input before the adapter connects."""
    monkeypatch.setattr(sys, "stdin", io.StringIO("RUN\nNEXT\n"))
    args = _so101_ev02(tmp_path, hardware=True)
    logs = []
    code = runmod.execute_run(args, prompt=lambda q: "", log=logs.append)
    assert code == runmod.EXIT_REFUSED, logs
    assert any("ssh -t" in m for m in logs), logs
    assert as_hardware == []


def test_next_on_hardware_is_a_typed_word(tmp_path, as_hardware, monkeypatch):
    """review N1: on hardware NEXT is asked through interlocks.typed_word (input typed ahead never
    answers it), never through the plain prompt. Here the operator declines it, so trial 2 never
    begins. (With #13, RUN on hardware is asked through typed_word too; the test holds either way.)"""
    words = []

    def typed_word(word, text, **kw):
        words.append(word)
        return word == "RUN"  # RUN is typed; NEXT is declined

    monkeypatch.setattr(runmod, "require_terminal", lambda: None)  # a test has no terminal
    monkeypatch.setattr(runmod, "typed_word", typed_word)
    args = _so101_ev02(tmp_path, hardware=True)
    asked = []

    def prompt(question: str) -> str:
        asked.append(question)
        return "RUN" if "Type RUN" in question else ""

    modules, joins = _trials_sim.clock_modules()
    logs = []
    with virtual_time(VirtualClock(), modules, joins):
        code = runmod.execute_run(args, prompt=prompt, log=logs.append)
    end = json.loads((tmp_path / "run" / "declared.json").read_text(encoding="utf-8"))["end"]
    assert words in (["NEXT"], ["RUN", "NEXT"]), logs
    assert not any("NEXT" in q for q in asked)
    assert end["latch"]["kind"] == "operator" and "before trial 2" in end["latch"]["reason"]
    assert end["trials"][0]["instant_t_s"] is not None and end["trials"][1]["instant_t_s"] is None
    assert code == runmod.EXIT_STOPPED and as_hardware


def test_a_position_space_trial_run_needs_every_command_axis_in_the_robots_state(tmp_path, monkeypatch):
    """Round-3 K9: when the robot's state left out an axis (here the gripper), the catch-up walk
    and its guard were skipped, and in so101-ev02-broken trial 2's first command slewed the
    dead-stopped pan 31 degrees until the joint-speed tripwire stopped it. In a position command
    space a trial run walks the machine from where it rests, so a state without every command axis
    is refused before anything moves, as round 2's guard refused that case."""
    sent = []
    real = sim_so101.from_args

    def from_args(a, c):
        adapter = real(a, c)
        read, send = adapter.read_state, adapter.send

        def read_state():
            st = read()
            return None if st is None else st.__class__(**{**st.__dict__,
                "joints": {k: v for k, v in st.joints.items() if k != "gripper"},
                "units": {k: v for k, v in st.units.items() if k != "gripper"}})

        def counting_send(cmd):
            sent.append(cmd)
            return send(cmd)

        adapter.read_state, adapter.send = read_state, counting_send
        return adapter

    monkeypatch.setattr(sim_so101, "from_args", from_args)
    args = _so101_ev02(tmp_path)
    logs = []
    code = runmod.execute_run(args, prompt=lambda q: "RUN", log=logs.append)
    assert code == runmod.EXIT_REFUSED, logs
    assert any("does not report" in m and "gripper" in m for m in logs), logs
    assert sent == []


def test_the_catch_up_walk_fails_closed_without_the_whole_state():
    """Round-3 K9, mid-run: should the state stop naming an axis, the catch-up refuses the next
    trial (the run ends through the latch) rather than skip the walk and its guard."""
    from types import SimpleNamespace

    from bounded_embodiment.commandspace import load_command_space
    from bounded_embodiment.generator import Proposal
    from bounded_embodiment_kit.runner.trials import TrialSegment
    from bounded_embodiment_kit.runner.watchdog import StateBox, StateSnapshot

    space = load_command_space(_trials_sim.SO101["command_space"])
    caps = {k: 1.0 for k in space.keys}
    box = StateBox()
    joints = {k.removesuffix(".pos"): 0.0 for k in space.keys if k != "gripper.pos"}
    box.put(StateSnapshot(0.0, joints, dict.fromkeys(joints, "deg")))
    live = SimpleNamespace(space=space, caps=caps, state_box=box, limits=runmod.DEFAULT, fk=None, envelope=None)
    seg = TrialSegment((Proposal(0, 0.0, "return", {k: 0.0 for k in space.keys}),), "trial 2", "stop",
                       "stop_message", 0.3, 0.5, True)
    far, walk = runmod._Live.catch_up(live, seg)
    assert far is not None and "gripper" in far and walk == []
    box.put(StateSnapshot(1.0, {}, {}))  # no joints at all
    assert runmod._Live.catch_up(live, seg)[0] is not None


def test_the_catch_up_fails_closed_when_no_state_is_recent():
    """Round-4 kit review, note 3: with no recent state, where the machine rests is not known, and
    the catch-up refuses the next trial. Round 3's code returned no walk and no guard there, and
    putting that back failed no test."""
    from types import SimpleNamespace

    from bounded_embodiment.commandspace import load_command_space
    from bounded_embodiment.generator import Proposal
    from bounded_embodiment_kit.runner.trials import TrialSegment
    from bounded_embodiment_kit.runner.watchdog import StateBox, StateSnapshot

    space = load_command_space(_trials_sim.SO101["command_space"])
    box = StateBox()
    live = SimpleNamespace(space=space, caps={k: 1.0 for k in space.keys}, state_box=box, limits=runmod.DEFAULT,
                           fk=None, envelope=None)
    seg = TrialSegment((Proposal(0, 0.0, "return", {k: 0.0 for k in space.keys}),), "trial 2", "stop",
                       "stop_message", 0.3, 0.5, True)
    far, walk = runmod._Live.catch_up(live, seg)  # no state at all
    assert far is not None and "not available" in far and walk == []
    joints = {k.removesuffix(".pos"): 0.0 for k in space.keys}
    box.put(StateSnapshot(0.0, joints, {j: "percent" if j == "gripper" else "deg" for j in joints}))
    box.put(StateSnapshot(1000.0, {}, {}))  # long after trial 1, a read that returned no joints
    far, walk = runmod._Live.catch_up(live, seg)
    assert far is not None and "not available" in far and walk == []
