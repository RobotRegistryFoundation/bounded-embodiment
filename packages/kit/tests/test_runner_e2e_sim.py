# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""End to end in simulation, through the ``be`` command line: the run directory is what core's
readers and generator expect."""
from __future__ import annotations

import argparse
import builtins
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import _runner_fakes as fakes
from bounded_embodiment import rawdata
from bounded_embodiment.commandspace import load_command_space
from bounded_embodiment.generator import compare_to_commands, stream_sha256
from bounded_embodiment_kit import cli
from bounded_embodiment_kit.adapters import sim_so101
from bounded_embodiment_kit.runner import run as runmod
from bounded_embodiment_kit.runner.limits import DEFAULT

GOLDEN = json.loads((fakes.REPO / "vectors" / "streams" / "golden.json").read_text())


def _be(*argv):
    p = argparse.ArgumentParser(prog="be")
    cli.register(p.add_subparsers(dest="command"))
    return p.parse_args(list(argv))


def _typing(monkeypatch, *answers):
    asked = []
    queue = list(answers)

    def fake_input(question=""):
        asked.append(question)
        return queue.pop(0) if queue else ""

    monkeypatch.setattr(builtins, "input", fake_input)
    return asked


def test_shakedown_on_the_simulated_so101_writes_a_run_core_can_read(tmp_path, monkeypatch):
    run_dir = tmp_path / "shakedown"
    asked = _typing(monkeypatch, "RUN", "")
    args = _be("run", "ev03", "--mode", "shakedown", "--adapter", "sim-so101",
               "--envelope", str(fakes.envelope_path("so101")),
               "--command-space", str(fakes.SO101_SPACE),
               "--kit-config", str(fakes.ENVELOPES / "so101.kit.json"), "--urdf", str(fakes.URDF),
               "--instrument", "sim", "--sim-sigma-m", "0.0003", "--sim-lag-s", "0.02",
               "--out", str(run_dir), "--seconds", "3", "--i-accept-the-risk",
               "--hardware-stop", "none (simulation)", "--ledger", str(tmp_path / "ledger.jsonl"))
    assert args.func(args) == runmod.EXIT_OK
    assert any("Type RUN" in q for q in asked) and any("type OFF" in q for q in asked)

    # declared.json: what the report builder needs, and nothing scored.
    declared = json.loads((run_dir / "declared.json").read_text())
    gen = declared["generator"]
    assert (gen["name"], gen["version"], gen["seed"]) == ("be-hostile", "1", 7)
    assert gen["params"]["amplitude"] == 0.15 and gen["params"]["seconds"] == 3.0
    assert declared["safety"]["hardware_stop"] == "none (simulation)"
    assert declared["safety"]["fk_tripwire"] == "on" and declared["run_mode"] == "shakedown"
    assert declared["adapter"]["adapter"] == "sim-so101" and declared["simulation"] is True
    assert declared["timestamps"]["started_at"] and declared["timestamps"]["ended_at"]
    assert declared["end"]["stream_finished"] and declared["end"]["stop"]["acknowledged"]
    assert not {"verdict", "score", "result"} & set(declared)

    # The files are copies of what was used.
    assert (run_dir / "command-space.json").read_bytes() == fakes.SO101_SPACE.read_bytes()
    assert (run_dir / "envelope.json").read_bytes() == fakes.envelope_path("so101").read_bytes()

    # commands.csv: be-commands-csv/1, and exactly the stream core regenerates from declared.json.
    rows = rawdata.read_commands(run_dir / "raw" / "commands.csv")
    space = load_command_space(run_dir / "command-space.json")
    assert len(rows) == 150
    assert compare_to_commands(space, gen["seed"], gen["params"],
                               [(r.seq, r.strategy, r.payload_json) for r in rows]) == []
    assert all(r.result.startswith("applied") for r in rows)

    # state.csv: be-state-csv/1, encoder rows per joint and fk rows per point axis, one timebase.
    state = rawdata.read_state(run_dir / "raw" / "state.csv")
    encoder = {r.joint for r in state if r.source == "encoder"}
    fk = {r.joint for r in state if r.source == "fk"}
    assert encoder == {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"}
    assert fk == {f"{p}.{a}" for p in ("tool", "elbow", "wrist") for a in "xyz"}
    times = [r.t_s for r in state]
    assert times == sorted(times) and times[0] < rows[0].t_sent_s and times[-1] > rows[-1].t_sent_s

    # samples.csv: the simulated instrument is the instrument, so it writes be-samples-csv/1.
    samples = rawdata.read_samples(run_dir / "raw" / "samples.csv")
    assert {s.point for s in samples} == {"tool", "elbow", "wrist"}


def test_the_so101_shakedown_stream_is_cores_golden_shakedown_vector():
    """At its full 60 s the kit's shakedown on the so101 example is exactly the generator vector
    so101-seed7-shakedown-60s (centre = start pose, gripper left at home)."""
    case = next(c for c in GOLDEN["cases"] if c["id"] == "so101-seed7-shakedown-60s")
    space = load_command_space(fakes.SO101_SPACE)
    args = argparse.Namespace(seed=7, seconds=None)
    _, block, seconds = runmod.build_proposals("shakedown", space, args, fakes.kit("so101"), DEFAULT)
    assert seconds == 60.0 and block["params"] == case["params"]
    assert stream_sha256(space, 7, block["params"]) == case["stream_sha256"]


def _rover_envelope(tmp_path):
    doc = {"envelope_version": "0.2", "machine": {"id": "sim-rover", "class": "mobile_base"},
           "frame": "base_link",
           "points": [{"id": "base", "description": "the rover's centre, on the floor"}],
           "workspace": {"zones": [{"id": "bay", "kind": "keep_in", "applies_to": ["*"],
                                    "box": {"min_m": [-0.5, -0.5, -1.0], "max_m": [3.0, 3.0, 1.0]}}]},
           "motion": {"max_speed_mps": 1.0},
           "stop": {"category": 1, "max_time_ms": 500, "max_distance_m": 0.5},
           "heartbeat": {"model_timeout_ms": 500, "gate_timeout_ms": 200, "on_loss": "stop"}}
    path = tmp_path / "rover.envelope.json"
    path.write_text(json.dumps(doc))
    return path


def test_the_rover_control_case_trips_and_escalates_because_it_cannot_stop_in_time(tmp_path, monkeypatch):
    """Straight out at top speed: the instrument tripwire stops it 1.5 cm outside the bay, but a
    rover braking at 1 m/s^2 from 1.5 m/s cannot be slower than 20 mm/s within 0.25 s, so the lab's
    e-stop command runs (here: a script that writes a file)."""
    marker = tmp_path / "estop-ran"
    estop = f'{sys.executable} -c "open({str(marker)!r}, \'w\').write(\'cut\')"'
    _typing(monkeypatch, "RUN")
    args = _be("run", "ev03", "--mode", "control", "--adapter", "sim-rover",
               "--envelope", str(_rover_envelope(tmp_path)), "--command-space", str(fakes.ROVER_SPACE),
               "--instrument", "sim", "--sim-sigma-m", "0.0003", "--out", str(tmp_path / "run"),
               "--target", "straight_out", "--step-cap", "0.1", "--no-fk-tripwire-i-understand",
               "--i-accept-the-risk", "--hardware-stop", "none (simulation)", "--estop-cmd", estop,
               "--ledger", str(tmp_path / "ledger.jsonl"))
    assert args.func(args) == runmod.EXIT_ESCALATED
    declared = json.loads((tmp_path / "run" / "declared.json").read_text())
    assert "base (instrument)" in declared["end"]["latch"]["reason"]
    assert declared["end"]["stop"]["escalation"]["returncode"] == 0
    assert marker.read_text() == "cut"
    assert declared["safety"]["fk_tripwire"] == "override"
    rows = rawdata.read_commands(tmp_path / "run" / "raw" / "commands.csv")
    assert rows and {r.strategy for r in rows} == {"drive_out"}


def test_goto_start_moves_slowly_from_rest_to_the_start_pose(tmp_path, monkeypatch):
    made = []
    real = sim_so101.from_args
    monkeypatch.setattr(sim_so101, "from_args", lambda a, c: made.append(real(a, c)) or made[-1])
    _typing(monkeypatch, "GO", "")
    args = _be("goto-start", "--adapter", "sim-so101", "--sim-start", "rest",
               "--envelope", str(fakes.envelope_path("so101")), "--command-space", str(fakes.SO101_SPACE),
               "--kit-config", str(fakes.ENVELOPES / "so101.kit.json"), "--urdf", str(fakes.URDF),
               "--hardware-stop", "none (simulation)")
    logs = []
    code = runmod.execute_goto_start(args, log=logs.append)
    assert code == runmod.EXIT_OK, logs
    assert any("at most 10 deg/s" in m for m in logs)
    start = fakes.kit("so101")["start_pose_deg"]
    end = made[0].true_joints()
    # There, give or take gravity sag: once under the held goal, once more when the end-of-stream
    # stop anchors to the present position (about 2 deg at the elbow in bob's sag model).
    assert all(abs(end[j] - start[j]) < 4.0 for j in sim_so101.ARM_JOINTS)


def test_goto_start_refuses_without_the_typed_go_or_with_a_wrapper(tmp_path, monkeypatch):
    _typing(monkeypatch, "go", "")
    base = ["goto-start", "--adapter", "sim-so101", "--sim-start", "rest",
            "--envelope", str(fakes.envelope_path("so101")), "--command-space", str(fakes.SO101_SPACE),
            "--kit-config", str(fakes.ENVELOPES / "so101.kit.json"), "--urdf", str(fakes.URDF),
            "--hardware-stop", "none (simulation)"]
    assert runmod.execute_goto_start(_be(*base), log=lambda m: None) == runmod.EXIT_STOPPED
    wrapped = _be(*base, "--wrapper", "_runner_fakes:make_wrapper")
    assert runmod.execute_goto_start(wrapped, log=lambda m: None) == runmod.EXIT_REFUSED


def test_a_step_capped_control_run_walks_from_the_declared_start_pose():
    """The SO-101 example's home is all zeros and its start pose is not: the control stream must
    walk from the start pose (declared in the generator params), and the runner must refuse to
    start it when the arm is more than one step from that pose."""
    from bounded_embodiment.commandspace import load_command_space

    space = load_command_space(fakes.SO101_SPACE)
    kit_config = fakes.kit("so101")
    start = runmod.control_start(space, kit_config)
    assert set(start) == set(space.keys)
    caps = dict.fromkeys(space.keys, 1.0)
    at_start = dict(start)
    runmod.check_control_start(space, caps, {}, at_start, start)  # at the pose: allowed
    off = {**at_start, "elbow_flex.pos": at_start["elbow_flex.pos"] + 5.0}
    with pytest.raises(runmod.RunError, match="declared start pose"):
        runmod.check_control_start(space, caps, {}, off, start)
    assert runmod.control_start(space, {}) is None


# ----- no live instrument (an instrument that only records, scored offline by be fit) --------------------------
def _shakedown_argv(tmp_path, *extra):
    return ["run", "ev03", "--mode", "shakedown", "--adapter", "sim-so101",
            "--envelope", str(fakes.envelope_path("so101")), "--command-space", str(fakes.SO101_SPACE),
            "--kit-config", str(fakes.ENVELOPES / "so101.kit.json"), "--urdf", str(fakes.URDF),
            "--out", str(tmp_path / "run"), "--seconds", "3", "--i-accept-the-risk",
            "--hardware-stop", "none (simulation)", "--ledger", str(tmp_path / "ledger.jsonl"), *extra]


def test_a_run_without_a_live_instrument_needs_the_declaration_and_the_fk_tripwire(tmp_path, monkeypatch):
    _typing(monkeypatch, "RUN", "")
    logs = []
    args = _be(*_shakedown_argv(tmp_path, "--instrument", "none"))
    assert runmod.execute_run(args, log=logs.append) == runmod.EXIT_REFUSED
    assert "--no-live-instrument-i-understand" in logs[-1]
    args = _be(*_shakedown_argv(tmp_path, "--instrument", "none", "--no-live-instrument-i-understand",
                                "--no-fk-tripwire-i-understand"))
    assert runmod.execute_run(args, log=logs.append) == runmod.EXIT_REFUSED
    assert "FK tripwire must be on" in logs[-1]
    args = _be(*_shakedown_argv(tmp_path, "--instrument", "sim", "--no-live-instrument-i-understand"))
    assert runmod.execute_run(args, log=logs.append) == runmod.EXIT_REFUSED
    assert "goes with --instrument none" in logs[-1]
    assert not (tmp_path / "run").exists()


def test_a_shakedown_supervised_by_the_robot_state_alone(tmp_path, monkeypatch):
    """--instrument none: the start condition and the stop acknowledgement come from the robot's own
    FK, and declared.json says the robot's state was the only live supervision."""
    _typing(monkeypatch, "RUN", "")
    logs = []
    args = _be(*_shakedown_argv(tmp_path, "--instrument", "none", "--no-live-instrument-i-understand"))
    code = runmod.execute_run(args, log=logs.append)
    assert code == runmod.EXIT_OK, logs
    declared = json.loads((tmp_path / "run" / "declared.json").read_text())
    assert declared["instrument_feed"]["kind"] == "none"
    assert declared["instrument_feed"]["live_supervision"] == "robot_state_only"
    assert declared["safety"]["fk_tripwire"] == "on"
    assert declared["end"]["stop"]["source"] == "fk (no instrument)"
    assert any("Waiting for the robot's own state" in m for m in logs)
    assert any("(robot state: no live instrument)" in m for m in logs)
    assert any("NO live instrument" in m for m in logs)
    assert not (tmp_path / "run" / "raw" / "samples.csv").exists()


# ----- EV-10 --------------------------------------------------------------------------------------------------
def test_ev10_takes_a_full_run_with_waypoints(tmp_path, monkeypatch):
    _typing(monkeypatch, "RUN", "")
    logs = []
    base = ["run", "ev10", "--adapter", "sim-so101", "--envelope", str(fakes.envelope_path("so101")),
            "--command-space", str(fakes.SO101_SPACE), "--kit-config", str(fakes.ENVELOPES / "so101.kit.json"),
            "--urdf", str(fakes.URDF), "--instrument", "sim", "--out", str(tmp_path / "run"),
            "--i-accept-the-risk", "--hardware-stop", "none (simulation)", "--ledger", str(tmp_path / "l.jsonl")]
    assert runmod.execute_run(_be(*base, "--mode", "shakedown", "--waypoints", "w.json"),
                              log=logs.append) == runmod.EXIT_REFUSED
    assert "--mode full --waypoints FILE" in logs[-1]
    assert runmod.execute_run(_be(*base, "--mode", "full"), log=logs.append) == runmod.EXIT_REFUSED
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps({"waypoints_version": "0.1", "waypoints": [
        {"id": "a", "position_m": [0.2, 0, 0.2], "command": {"shoulder_pan.pos": 0}}]}))
    assert runmod.execute_run(_be(*base, "--mode", "full", "--waypoints", str(bad)),
                              log=logs.append) == runmod.EXIT_REFUSED
    assert "no value for" in logs[-1]
    assert runmod.execute_run(_be(*base, "--mode", "full", "--waypoints", str(bad), "--seconds", "5"),
                              log=logs.append) == runmod.EXIT_REFUSED
    assert "--seconds is for ev03" in logs[-1]
    ev03 = ["run", "ev03", *base[2:], "--mode", "full", "--cycles", "2"]
    assert runmod.execute_run(_be(*ev03), log=logs.append) == runmod.EXIT_REFUSED
    assert "are for be run ev10" in logs[-1]
