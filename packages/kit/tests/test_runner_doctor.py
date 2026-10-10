# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""``be doctor`` is strictly read-only: a bus that records every call sees no write, and torque is
left exactly as found."""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import _runner_fakes as fakes
from bounded_embodiment import envelope as envelope_mod
from bounded_embodiment_kit.runner.doctor import doctor_arm, environment
from bounded_embodiment_kit.runner.kin import ArmFK


def _robot(**kw):
    adapter = fakes.sim_adapter(record=True, **kw)
    return adapter, adapter._make_robot()


def test_doctor_writes_nothing_and_leaves_torque_as_found():
    for goals_zero in (True, False):  # bob at power-up (torque off), and an arm holding (torque on)
        adapter, robot = _robot(power_up_goals_zero=goals_zero)
        torque_before = dict(adapter.servos.torque)
        goal_before = dict(adapter.servos.goal)
        env = envelope_mod.load(fakes.envelope_path("so101"))
        fk = ArmFK.build(fakes.URDF, None, env)
        report = doctor_arm(lambda r=robot: r, envelope=env, fk=fk)
        calls = robot.bus.calls
        assert [c for c in calls if c.is_write] == []
        assert calls[0].method == "connect" and calls[-1].method == "disconnect"
        assert calls[-1].values is False  # disable_torque=False: the port closes, torque untouched
        assert robot.calls == []  # never the robot's connect()/configure()/disconnect()
        assert adapter.servos.torque == torque_before and adapter.servos.goal == goal_before
        assert report["calibration_matches_file"] is True
        assert set(report["fk_clearance_m"]) == {"tool", "elbow", "wrist"}
        assert ("set every goal" in report["be_run_connect_would"]) is goals_zero


def test_environment_lists_without_touching_anything():
    env = environment()
    assert {"python", "lerobot", "serial_ports", "calibration_files"} <= set(env)
