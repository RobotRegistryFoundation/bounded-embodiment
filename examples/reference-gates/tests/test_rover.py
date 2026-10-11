# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""The rover's reference gate and its control case, on the kit's simulated rover in lockstep."""
from __future__ import annotations

import json
import math

import _support as S
import pytest
from reference_gates import rover

STRAIGHT_OUT = {"linear": 1.5, "angular": 0.0}  # the rover command space's far target

#: What the hostile generator's malformed kinds do to the drive's far target (all but "bool").
MALFORMED = {
    "nan": {"linear": math.nan, "angular": 0.0},
    "inf": {"linear": math.inf, "angular": 0.0},
    "neg_inf": {"linear": 1.5, "angular": -math.inf},
    "string": {"linear": "1.5", "angular": 0.0},
    "null": {"linear": None, "angular": 0.0},
    "missing_key": {"linear": 1.5},
    "unknown_key": {"linear": 1.5, "unknown_axis": 99.0},
    "empty": {},
    "not_a_mapping": [1.5, 0.0],
}


@pytest.mark.parametrize("kind", list(MALFORMED))
def test_malformed_commands_stop_the_rover(kind):
    r = S.Rover(rover.factory)
    r.send({"linear": 0.4, "angular": 0.3})
    r.run(1.0)
    r.send(MALFORMED[kind])
    assert r.gate.last_decision == "stop"
    assert (r.rover.cmd_v, r.rover.cmd_w) == (0.0, 0.0)


def test_a_boolean_reads_as_a_number_as_in_the_harness():
    """The harness's quirk, kept (module docstring): True is 1.0 m/s, then clamped to the
    envelope's 0.5 m/s and checked like any other command."""
    assert rover.valid_command({"linear": True, "angular": 0.0}) == (1.0, 0.0)
    r = S.Rover(rover.factory)
    r.send({"linear": True, "angular": 0.0})
    assert r.gate.last_decision == "clamp" and (r.rover.cmd_v, r.rover.cmd_w) == (0.5, 0.0)


def test_speed_is_clamped_to_the_envelope_and_turns_to_the_drive():
    r = S.Rover(rover.factory)
    r.send({"linear": 0.3, "angular": 0.2})
    assert r.gate.last_decision == "allow" and (r.rover.cmd_v, r.rover.cmd_w) == (0.3, 0.2)
    r.send({"linear": -9.0, "angular": 9.0})
    assert r.gate.last_decision == "clamp" and (r.rover.cmd_v, r.rover.cmd_w) == (-0.5, 1.5)


def test_an_envelope_turn_limit_is_kept(tmp_path):
    doc = json.loads(S.ROVER_ENVELOPE.read_text())
    doc["motion"]["max_turn_radps"] = 0.5
    path = tmp_path / "slow-turns.envelope.json"
    path.write_text(json.dumps(doc))
    r = S.Rover(rover.factory, context=S.rover_context(envelope_path=str(path)))
    r.send({"linear": 0.0, "angular": -1.5})
    assert r.rover.cmd_w == -0.5


@pytest.mark.parametrize("heading", [0.1, math.pi / 2, math.pi, -2.0])
def test_driving_straight_out_never_leaves_the_bay(heading):
    """The far target, 50 times a second for 30 s, from (1, 1) in four directions: the gate stops
    the rover short of the walls and of the keep-out (it brakes at 1 m/s^2 from 0.5 m/s)."""
    r = S.Rover(rover.factory, start=(1.0, 1.0, heading))
    worst = r.clearance()
    for _ in range(1500):
        r.send(STRAIGHT_OUT)
        r.run(0.02)
        worst = min(worst, r.clearance())
    assert worst > 0.0
    assert r.gate.decisions["reject"] > 0
    assert r.rover.velocities() == (0.0, 0.0)


@pytest.mark.xfail(strict=True, raises=AssertionError, reason=(
    "The gate's known hole (module docstring): a straight-line stopping path checked at four "
    "points while the rover turns. With hostile seed 29 it backs into the keep-out's corner at "
    "68.3 s, 23 mm deep at worst, with perfect sensing."))
def test_the_hostile_stream_never_takes_it_into_the_keep_out():
    from bounded_embodiment.commandspace import load_command_space
    from bounded_embodiment.generator import hostile_stream

    r = S.Rover(rover.factory)
    prev, worst = 0.0, r.clearance()
    for p in hostile_stream(load_command_space(S.ROVER_SPACE), 29, {"seconds": 75.0}):
        dt = p.t_s - prev
        n = max(1, round(dt / 0.005))
        for _ in range(n):  # the sim-rover adapter's physics period
            r.rover.tick(dt / n)
        prev = p.t_s
        r.send(p.payload)
        worst = min(worst, r.clearance())
    assert worst >= 0.0, f"{worst * 1000:.1f} mm outside"


def test_without_the_gate_the_rover_drives_out():
    r = S.Rover(rover.ungated)
    assert r.gate is r.rover  # the control case is the drive itself
    for _ in range(250):
        r.send(STRAIGHT_OUT)
        r.run(0.02)
    assert r.clearance() < -0.5


def test_the_factory_needs_the_runs_envelope():
    with pytest.raises(ValueError, match="envelope"):
        rover.factory(S.Rover().rover, S.rover_context(envelope_path=None))
    gate = rover.factory(S.Rover().rover, S.rover_context(kit_config_path=None))
    assert gate.tolerance_m == rover.DEFAULT_TOLERANCE_M and gate.point == "base"
    assert gate.max_speed_mps == 0.5
