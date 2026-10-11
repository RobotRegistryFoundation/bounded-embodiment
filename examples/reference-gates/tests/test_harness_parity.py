# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""The port against the original: ev03-harness's ``ReferenceArmGate`` (``ev03/arm/gate.py``) and
its first gate (``ev03/arm/legacy/gate_v1.py``) beside this package's, each on its own copy of the
kit's simulated SO-101, fed the same hostile stream on the same lockstep clock. After every
command both must have made the same decision and written the same goal registers.

Needs an ev03-harness checkout: ``EV03_HARNESS=/path/to/ev03-harness pytest``. Skipped otherwise
(the harness is not public). The harness's gates need only numpy; their envelope and kinematics
come from the harness's own ``examples/so101_envelope.json``, the file the repo's
``so101.envelope.json`` was converted from."""
from __future__ import annotations

import os
import sys
from pathlib import Path

import _support as S
import pytest
from bounded_embodiment.commandspace import load_command_space
from bounded_embodiment.generator import hostile_stream
from reference_gates import so101_reference, so101_reference_v1

HARNESS = os.environ.get("EV03_HARNESS")
pytestmark = pytest.mark.skipif(not HARNESS or not Path(HARNESS, "ev03", "arm", "gate.py").is_file(),
                                reason="set EV03_HARNESS to an ev03-harness checkout")


def _harness():
    if HARNESS not in sys.path:
        sys.path.insert(0, HARNESS)
    from ev03.arm.envelope import ArmEnvelope
    from ev03.arm.gate import ReferenceArmGate
    from ev03.arm.kinematics import SO101Kinematics
    from ev03.arm.legacy.gate_v1 import ReferenceArmGateV1

    env = ArmEnvelope.load(Path(HARNESS, "examples", "so101_envelope.json"))
    return env, SO101Kinematics(joint_map=env.joint_map), ReferenceArmGate, ReferenceArmGateV1


@pytest.mark.parametrize(("seed", "servos", "speed_factor", "first"), [
    (7, "ideal", None, False), (11, "sag", None, False), (23, "sag", 0.5, False),
    (7, "sag", None, True), (11, "ideal", None, True)])
def test_the_port_decides_as_the_harness_does(seed, servos, speed_factor, first):
    env, kin, harness_gate, harness_v1 = _harness()
    theirs, ours = S.Arm(servos=servos), S.Arm(servos=servos)
    clock = S.Clock()
    context = S.so101_context(speed_factor=speed_factor)
    if first:
        a = harness_v1(theirs.robot, env, kin, clock, speed_factor=speed_factor or 0.8)
        b = so101_reference_v1.factory(ours.robot, context)
    else:
        a = harness_gate(theirs.robot, env, kin, clock, speed_factor=speed_factor or 0.8)
        b = so101_reference.factory(ours.robot, context)
    b.clock = clock
    assert (a.margin, a.ticks) == (pytest.approx(b.margin), b.ticks)
    prev = 0.0
    for p in hostile_stream(load_command_space(S.SO101_SPACE), seed, {"seconds": 20.0}):
        theirs.adapter.step(p.t_s - prev)
        ours.adapter.step(p.t_s - prev)
        clock.t = prev = p.t_s
        a.propose(dict(p.payload))
        b.send_action(dict(p.payload))
        assert (b.last_decision, ours.servos.goal) == (a.last_decision, theirs.servos.goal), p
    assert a.decisions == b.decisions
    assert ours.servos.present == theirs.servos.present
