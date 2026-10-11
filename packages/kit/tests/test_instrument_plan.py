# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""`be instrument plan` (poses for the accuracy check's surveyed points), the IK under it, and
`be goto-start --pose`, which drives to one of them."""
from __future__ import annotations

import argparse
import builtins
import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import _runner_fakes as fakes
from bounded_embodiment import cli as core_cli
from bounded_embodiment import envelope as envelope_mod

from bounded_embodiment_kit import cli
from bounded_embodiment_kit.adapters import sim_so101
from bounded_embodiment_kit.instruments import plan as plan_mod
from bounded_embodiment_kit.kinematics import JointMap, URDFModel
from bounded_embodiment_kit.kinematics.ik import solve_point
from bounded_embodiment_kit.runner import run as runmod

REPO = fakes.REPO
BOB = REPO / "examples" / "envelopes" / "bob.envelope.json"
BOB_KIT = REPO / "examples" / "envelopes" / "bob.kit.json"
URDF = REPO / "examples" / "robots" / "so101" / "so101_new_calib.urdf"
SPACE = REPO / "examples" / "command-spaces" / "so101.command-space.json"


@pytest.fixture(scope="module")
def model():
    return URDFModel.from_file(URDF)


def test_ik_reaches_a_point_and_levels_an_axis(model):
    arm = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
    seed = {j: 0.0 for j in model.actuated}
    seed["wrist_roll"] = math.pi / 2
    r = solve_point(model, "gripper_frame_link", (0.25, 0.05, 0.20), joints=arm, seed=seed,
                    level=("gripper_link", 0))
    assert r.ok and r.error_m < 1e-4 and abs(r.level_deg) < 2.0
    p = model.points({"tool": ("gripper_frame_link", None)}, r.q)["tool"]
    assert p == pytest.approx((0.25, 0.05, 0.20), abs=1e-4)
    lim = model.limits()
    assert all(lim[j][0] <= r.q[j] <= lim[j][1] for j in arm)
    far = solve_point(model, "gripper_frame_link", (1.5, 0.0, 0.2), joints=arm, seed=seed)
    assert not far.ok and far.error_m > 0.5


def test_bob_plan_spreads_eight_points_through_the_keep_in_and_off_the_keep_out(model):
    env = envelope_mod.load(BOB)
    kit = json.loads(BOB_KIT.read_text())
    targets = plan_mod.plan_targets(env, model, JointMap.from_dict(kit["joint_map"]), kit["start_pose_deg"],
                                    level=("gripper_link", 0))
    assert [t["label"] for t in targets] == [f"T{i}" for i in range(1, 9)]
    pts = [t["planned_m"] for t in targets]
    lo, hi = plan_mod.keep_in_box(env, "tool")
    for i in range(3):   # near both extremes of every axis (a quarter of the extent)
        w = hi[i] - lo[i]
        assert min(p[i] for p in pts) <= lo[i] + 0.25 * w and max(p[i] for p in pts) >= hi[i] - 0.25 * w
    for t in targets:
        assert min(t["clearance_m"].values()) >= 0.0099      # every point 1 cm inside its zones
        assert abs(t["level_deg"]) < 2.0
        assert t["pose_deg"]["gripper"] == kit["start_pose_deg"]["gripper"]
    # The corner inside keep_out_0 (x 0.30-0.36, y 0.05-0.10, z 0.26-0.32) was moved off it.
    assert any("off a keep-out" in t["corner_note"] for t in targets)
    # The poses reproduce the planned positions through the same joint map (what the FK tripwire uses).
    jm = JointMap.from_dict(kit["joint_map"])
    for t in targets:
        q = jm.to_urdf(t["pose_deg"])
        p = model.points({"tool": ("gripper_frame_link", None)}, q)["tool"]
        assert p == pytest.approx(t["planned_m"], abs=0.002)    # poses are rounded to 0.1 deg


def test_the_plan_command_writes_poses_for_the_kit_config(tmp_path, capsys):
    out = tmp_path / "plan.json"
    rc = core_cli.main(["instrument", "plan", "--envelope", str(BOB), "--urdf", str(URDF), "--kit-config",
                        str(BOB_KIT), "--command-space", str(SPACE), "--level", "gripper_link:x", "--out", str(out)])
    text = capsys.readouterr().out
    assert rc == 0, text
    doc = json.loads(out.read_text())
    assert doc["format"] == "be-target-plan/0.1" and set(doc["poses_deg"]) == {f"T{i}" for i in range(1, 9)}
    assert "never reference positions" in doc["note"]
    assert '"poses_deg"' in text and "--pose T1" in text


def test_plan_refusals(tmp_path, capsys):
    base = ["instrument", "plan", "--envelope", str(BOB), "--urdf", str(URDF), "--kit-config", str(BOB_KIT)]
    assert core_cli.main(base + ["--inset-mm", "90"]) == 2
    assert "leaves no room" in capsys.readouterr().err
    assert core_cli.main(base + ["--level", "nolink:x"]) == 2
    assert "LINK:AXIS" in capsys.readouterr().err
    kit = tmp_path / "k.json"
    kit.write_text(json.dumps({"joint_map": {}}))
    assert core_cli.main(base[:-1] + [str(kit)]) == 2
    assert "start_pose_deg" in capsys.readouterr().err


# ----- be goto-start --pose ------------------------------------------------------------------------------------
def _be(*argv):
    p = argparse.ArgumentParser(prog="be")
    cli.register(p.add_subparsers(dest="command"))
    return p.parse_args(list(argv))


def _typing(monkeypatch, *answers):
    queue = list(answers)
    monkeypatch.setattr(builtins, "input", lambda q="": queue.pop(0) if queue else "")


def test_goto_start_drives_to_a_planned_pose(tmp_path, monkeypatch, model):
    env = envelope_mod.load(fakes.envelope_path("so101"))
    kit = fakes.kit("so101")
    targets = plan_mod.plan_targets(env, model, JointMap.from_dict(kit.get("joint_map")), kit["start_pose_deg"])
    kit["poses_deg"] = {t["label"]: t["pose_deg"] for t in targets}
    kit_path = tmp_path / "so101.kit.json"
    kit_path.write_text(json.dumps(kit))
    made = []
    real = sim_so101.from_args
    monkeypatch.setattr(sim_so101, "from_args", lambda a, c: made.append(real(a, c)) or made[-1])
    _typing(monkeypatch, "GO", "")
    args = _be("goto-start", "--adapter", "sim-so101", "--envelope", str(fakes.envelope_path("so101")),
               "--command-space", str(fakes.SO101_SPACE), "--kit-config", str(kit_path), "--urdf", str(fakes.URDF),
               "--hardware-stop", "none (simulation)", "--pose", "T3")
    logs = []
    assert runmod.execute_goto_start(args, log=logs.append) == runmod.EXIT_OK, logs
    assert any("goto-start (pose T3)" in m for m in logs)
    end = made[0].true_joints()
    want = kit["poses_deg"]["T3"]
    assert all(abs(end[j] - want[j]) < 4.0 for j in sim_so101.ARM_JOINTS)   # give or take the sim's sag


def test_goto_start_refuses_an_unknown_pose(tmp_path, monkeypatch):
    _typing(monkeypatch, "GO", "")
    args = _be("goto-start", "--adapter", "sim-so101", "--envelope", str(fakes.envelope_path("so101")),
               "--command-space", str(fakes.SO101_SPACE), "--kit-config", str(fakes.ENVELOPES / "so101.kit.json"),
               "--urdf", str(fakes.URDF), "--hardware-stop", "none (simulation)", "--pose", "T9")
    logs = []
    assert runmod.execute_goto_start(args, log=logs.append) == runmod.EXIT_REFUSED
    assert any("no poses_deg.T9" in m for m in logs)


def test_the_plan_as_ev10_waypoints(tmp_path, capsys):
    """--waypoints-out: a waypoints 0.1 file core accepts, every waypoint 3 cm inside and 5 cm apart; a
    plan too near the faces for EV-10 is refused, as is one without the command space."""
    from bounded_embodiment import report as core_report
    from bounded_embodiment.commandspace import load_command_space
    from bounded_embodiment.report.waypoints import check_commands

    base = ["instrument", "plan", "--envelope", str(BOB), "--urdf", str(URDF), "--kit-config", str(BOB_KIT)]
    out = tmp_path / "ev10-waypoints.json"
    rc = core_cli.main(base + ["--command-space", str(SPACE), "--inset-mm", "35", "--margin-mm", "30",
                               "--waypoints-out", str(out)])
    assert rc == 0, capsys.readouterr().err
    wps = core_report.load_waypoints(out.read_bytes())
    assert len(wps.waypoints) == 8 and wps.point == "tool"
    assert check_commands(wps, load_command_space(SPACE)) == []
    env = envelope_mod.load(BOB)
    assert all(env.clearance("tool", w.position_m) >= 0.03 for w in wps.waypoints)
    capsys.readouterr()
    assert core_cli.main(base + ["--command-space", str(SPACE), "--waypoints-out", str(tmp_path / "x.json")]) == 2
    assert "EV-10 wants 3 cm" in capsys.readouterr().err
    assert core_cli.main(base + ["--waypoints-out", str(tmp_path / "y.json")]) == 2
    assert "needs --command-space" in capsys.readouterr().err
