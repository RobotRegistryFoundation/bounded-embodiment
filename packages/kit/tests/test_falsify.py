# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""``bounded_embodiment_kit.falsify`` (rule set ``be-falsify/1``, draft): the search is
deterministic, its snapshots are faithful, it rediscovers the rover reference gate's documented
hole in fewer steps than the hostile stream's seed 29 does, and the recorded traces in
``tests/fixtures/falsification`` replay. Simulation only: nothing here touches hardware.

The gates are loaded by name (``module:factory``), as ``be run --wrapper`` does; the kit never
imports them."""
from __future__ import annotations

import ast
import json
import math
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
GATES = REPO / "examples" / "reference-gates"
if str(GATES) not in sys.path:
    sys.path.insert(0, str(GATES))

from bounded_embodiment_kit import falsify as F  # noqa: E402

FIXTURES = HERE / "fixtures" / "falsification"
EX = REPO / "examples"
ROVER_CTX = {"mode": "full", "speed_factor": None,
             "envelope_path": str(EX / "envelopes" / "rover.envelope.json"),
             "command_space_path": str(EX / "command-spaces" / "rover.command-space.json"),
             "kit_config_path": None, "urdf_path": None}
ARM_CTX = {"mode": "full", "speed_factor": None,
           "envelope_path": str(EX / "envelopes" / "so101.envelope.json"),
           "command_space_path": str(EX / "command-spaces" / "so101.command-space.json"),
           "kit_config_path": str(EX / "envelopes" / "so101.kit.json"),
           "urdf_path": str(EX / "robots" / "so101" / "so101_new_calib.urdf")}
ROVER_GATE = "reference_gates.rover:factory"
#: Steps the hostile stream ``be-hostile/1`` with seed 29 needs to take the rover gate into the
#: keep-out (README of the reference gates; the B6 matrix). Pinned by a test below.
HOSTILE_SEED_29_STEPS = 9737


def rover(gate=ROVER_GATE):
    return F.RoverSystem(gate, ROVER_CTX)


def test_the_search_is_deterministic():
    a = F.falsify(rover(), "position", seed=3, budget=1500)
    b = F.falsify(rover(), "position", seed=3, budget=1500)
    assert a.to_json() == b.to_json()
    assert a.steps_used == 1500 and not a.found  # the whole budget, nothing found in it
    c = F.falsify(rover(), "position", seed=4, budget=1500)
    assert c.trace["segments"] != a.trace["segments"]  # the seed matters


def test_it_rediscovers_the_rover_hole_faster_than_hostile_seed_29():
    """The documented hole (examples/reference-gates/README.md): the stopping path is checked on
    a straight line while the rover turns, and a reject brakes unchecked."""
    res = F.falsify(rover(), "position", seed=0, budget=HOSTILE_SEED_29_STEPS - 1)
    assert res.found
    assert res.steps_used == res.steps_to_violation < HOSTILE_SEED_29_STEPS
    rows = res.trace["steps"]
    first = next(r for r in rows if r["robustness"] < -F.VIOLATION_EPS)
    assert F.refused(first["decision"])  # the gate was refusing: braking, unchecked
    assert abs(first["velocities"][1]) > 0.5  # and the rover was turning
    assert first["velocities"][0] != 0.0
    assert res.trace["robustness"] < -0.01  # centimetres into the keep-out, not rounding


def test_the_hostile_baseline_matches_the_gates_own_lockstep_test():
    out = F.random_baseline(rover(), "position", seeds=[29])
    assert out == {"found": True, "seed": 29, "steps": HOSTILE_SEED_29_STEPS,
                   "steps_in_seed": HOSTILE_SEED_29_STEPS, "t_s": pytest.approx(68.288)}


def test_the_control_case_is_found_at_once():
    res = F.falsify(rover("reference_gates.rover:ungated"), "speed", seed=0, budget=2000)
    assert res.found and res.steps_used < 1000


@pytest.mark.parametrize("make", [
    lambda: rover(),
    lambda: F.ArmSystem("reference_gates.so101_reference:factory", ARM_CTX, servos="sag"),
], ids=["rover", "arm"])
def test_a_resumed_rollout_is_the_same_as_a_fresh_one(make):
    """Snapshots at segment boundaries are a simulation-only shortcut; they must not change what
    the machine does."""
    system = make()
    axes = system.space.axes
    base = [[a.hi for a in axes], [a.lo for a in axes], [a.home for a in axes]]
    other = [base[0], base[1], [a.hi if i % 2 else a.lo for i, a in enumerate(axes)]]
    cache = F._Snapshots()
    for prop in ("position", "speed"):
        F.rollout(system, prop, base, 10, cache=cache)
        resumed = F.rollout(system, prop, other, 10, cache=cache)
        fresh = F.rollout(system, prop, other, 10)
        assert resumed["steps"] == 10 and fresh["steps"] == 30
        assert resumed["robustness"] == fresh["robustness"]
        assert resumed["refused_segments"] == fresh["refused_segments"]
        cache = F._Snapshots()


def test_a_trace_replays_bit_for_bit(tmp_path):
    res = F.falsify(rover(), "position", seed=1, budget=3000)
    assert res.found
    path = tmp_path / "trace.json"
    F.write_trace(res, path)
    doc = json.loads(path.read_text())
    assert doc["rule_set"] == F.RULE_SET == "be-falsify/1"
    assert doc["draft"] is True and doc["simulation_only"] is True
    out = F.replay(doc, root=Path.cwd())
    assert out["matches"] and out["violated"]


def test_a_changed_trace_does_not_replay(tmp_path):
    res = F.falsify(rover(), "position", seed=1, budget=3000)
    doc = json.loads(json.dumps(res.to_json()))
    doc["trace"]["steps"][5]["decision"] = "allow" if doc["trace"]["steps"][5]["decision"] != \
        "allow" else "reject"
    out = F.replay(doc, root=Path.cwd())
    assert not out["matches"] and out["first_mismatch"] == 5


@pytest.mark.parametrize("name", sorted(p.name for p in FIXTURES.glob("*.json")))
def test_the_recorded_traces_replay(name):
    """docs/falsification-findings.md: every trace it cites replays, and a trace marked found
    still violates its property."""
    doc = json.loads((FIXTURES / name).read_text())
    assert doc["rule_set"] == F.RULE_SET and doc["simulation_only"] is True
    out = F.replay(doc, root=REPO, atol=1e-9)
    assert out["matches"], f"first mismatch at step {out['first_mismatch']}"
    assert out["violated"] is doc["found"]
    assert math.isclose(out["robustness"], doc["trace"]["robustness"], abs_tol=1e-9)


def test_there_is_a_recorded_trace_for_every_gate_and_property():
    names = {p.stem for p in FIXTURES.glob("*.json")}
    for gate in ("rover-reference", "arm-reference-sag"):
        for prop in F.PROPERTIES:
            assert f"{gate}-{prop}-seed0" in names


def test_the_kit_module_imports_no_gate():
    tree = ast.parse(Path(F.__file__).read_text())
    names = [a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names]
    names += [n.module or "" for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)]
    assert not [m for m in names if m.startswith(("reference_gates", "rcan"))]


def test_the_command_line_writes_a_trace(tmp_path, capsys):
    out = tmp_path / "t.json"
    rc = F.main(["--machine", "sim-rover", "--gate", ROVER_GATE, "--gate-path", str(GATES),
                 "--envelope", ROVER_CTX["envelope_path"],
                 "--command-space", ROVER_CTX["command_space_path"],
                 "--budget", "300", "--out", str(out)])
    assert rc == 0
    summary = json.loads(capsys.readouterr().out)
    assert summary["steps_used"] == 300 and "trace" not in summary
    assert json.loads(out.read_text())["budget_steps"] == 300


def test_an_unknown_property_is_refused():
    with pytest.raises(ValueError, match="property"):
        F.falsify(rover(), "force")
