"""The round-4 method and core review's reproductions (review 5901657149), kept as regression
tests. As in ``test_review_round3.py``, each run is built with core's own generators and the
golden runs' command space and parameters, and scored with core's own scorers."""

from __future__ import annotations

import bisect

import _scoring_synth as T
import pytest
from _report_helpers import golden, in_repo
from bounded_embodiment import report as rep
from bounded_embodiment.commandspace import load_command_space
from bounded_embodiment.generator import encode_payload
from bounded_embodiment.rawdata import Command
from bounded_embodiment.scoring import ScoringInput, ev02, ev05
from bounded_embodiment.scoring import stops as stops_mod

pytestmark = in_repo


# ----- finding 10: the walk rule follows every axis the drive moved -------------------------------------
def _falls_behind(top_speed_mps: float) -> dict:
    """The reproduction: a cartesian space with a gripper axis, two drives in which x lands 11 rows
    (0.22 s) before the stop, a drive of 1.84 s commanding x at 0.25 m/s, and a stop that does
    nothing. The machine chases every command, the continuing ones included, but tops out at
    ``top_speed_mps``, so it falls behind for as long as the drive lasts."""
    G = golden()
    x_axis, y_axis, z_axis = G.COMMAND_SPACE["axes"]
    space = load_command_space(dict(
        G.COMMAND_SPACE,
        axes=[dict(x_axis, lo=0.0, hi=1.0, home=0.5), y_axis, z_axis,
              {"key": "gripper", "unit": "percent", "lo": 0.0, "hi": 100.0, "home": 0.0}],
        far_targets=[{"id": "down", "values": {"x": 0.5, "y": 0.0, "z": 0.0, "gripper": 0.0}, "description": "down"}]))

    def c(x, g):
        return {"x": x, "y": 0.0, "z": 0.175, "gripper": g}

    trials = [("stop_message", "a"), ("stop_message", "b"), ("stop_message", "a"),
              ("hardwired_stop", "b"), ("hardwired_stop", "a"), ("hardwired_stop", "b")]
    plan = rep.load_stop_plan({
        "stop_plan_version": "0.1", "point": "tool", "description": "TEST-ONLY", "start": c(0.5, 0),
        "drives": [{"id": "a", "command": c(0.91, 100)}, {"id": "b", "command": c(0.909, 100)}],
        "sources": [{"source": s, "description": s} for s in ("stop_message", "hardwired_stop")],
        "trials": [{"source": s, "drive": d} for s, d in trials]})
    params = {"hz": 50.0, "drive_s": 1.841, "return_s": 3.0, "after_s": 3.0,
              "step_cap": {"x": 0.005, "y": 0.005, "z": 0.005, "gripper": 0.5}}
    segs = rep.split_segments(rep.stop_stream(space, plan, params))  # accepted by core
    times = G._segment_times(segs, [1.0] * (len(segs) - 1))
    rows, cmd_t, cmd_x = [], [], []
    for seg, ts in zip(segs, times):
        for p, t in zip(seg, ts):
            stop = p.strategy.startswith("stop:")
            rows.append(Command(p.seq, t, t + 0.003, p.strategy, "{}" if stop else encode_payload(p.payload),
                                "triggered" if stop else "ok", None))
            if not stop:  # the stop does nothing: the machine chases every command
                cmd_t.append(t)
                cmd_x.append(p.payload["x"])
    x, xs = 0.5, []
    for i in range(round((rows[-1].t_sent_s + 1.5) / 0.001)):
        k = bisect.bisect_right(cmd_t, -0.5 + i * 0.001) - 1
        target = cmd_x[k] if k >= 0 else 0.5
        x += max(-top_speed_mps * 0.001, min(top_speed_mps * 0.001, target - x))
        xs.append(x)

    def path(t):
        return (xs[min(len(xs) - 1, max(0, round((t + 0.5) / 0.001)))], 0.0, 0.175)

    samples = T.sample_rows(path, -0.5, rows[-1].t_sent_s + 1.0, 100.0, ("tool",), 0.0003)
    env = G.stop_envelope_doc()
    env["workspace"]["zones"] = [dict(env["workspace"]["zones"][0],
                                      box={"min_m": [-0.05, -0.15, 0.05], "max_m": [1.05, 0.15, 0.3]})]
    return ev02.score(ScoringInput(envelope=T.load(env), samples=samples, commands=rows,
                                   accuracy=T.accuracy_rows(jitter_sigma_m=T.STOP_JITTER_M),
                                   declared=T.declared(), profile=T.ev02_profile(),
                                   plan=plan.for_scoring(space)))


def test_a_dead_stop_cannot_pass_on_a_machine_that_falls_behind_over_a_long_drive():
    """Finding 10: a machine that tops out at 0.215 m/s, inside the approach band (0.2 to 0.25 m/s)
    but under the commanded 0.25 m/s, falls behind its commands for the whole 1.84 s drive. x, the
    axis that carries the stop point, lands 0.22 s before the stop, more than the speed window, while
    the machine still moves at full speed; the window rule followed only the gripper, which walks
    on, and the dead stop PASSed with six adequate trials (10.7 to 12.9 mm). Every axis the drive
    moved is followed now, x among them, so no trial is adequate."""
    out = _falls_behind(0.215)
    trials = out["stops"]["trials"]
    assert out["verdict"]["result"] != "PASS"
    assert [t["status"] for t in trials] == ["inadequate"] * 6
    assert all(0.2 <= t["approach_speed_mps"] <= 0.25 for t in trials)  # the approach is inside the band


def test_the_walk_rule_follows_an_axis_that_landed_before_the_window():
    """``walk_end`` with the trial's first drive row: an axis whose command changed during the
    drive, but not in the last w before the stop, is followed, and its standing still ends the walk
    at the stop. Without the drive (the window rule alone) the gripper carries the walk on."""
    before = [(0.00, '{"x": 0.0, "g": 0}'), (0.02, '{"x": 0.1, "g": 1}'), (0.04, '{"x": 0.2, "g": 2}'),
              (0.30, '{"x": 0.2, "g": 15}'), (0.50, '{"x": 0.2, "g": 25}')]
    after = [(0.52, '{"x": 0.2, "g": 26}'), (0.54, '{"x": 0.2, "g": 27}'), (0.56, '{"x": 0.2, "g": 28}')]
    assert ev02.walk_end(before, after, 0.51, window_s=0.2) == 0.56  # x landed at 0.04 s: not seen
    assert ev02.walk_end(before, after, 0.51, window_s=0.2, drive_t=0.02) == 0.51
