"""The round-3 method and core review's reproductions (review 5900310515), kept as regression
tests. As in ``test_review_round2.py``, each run is built with core's own generators and the
golden runs' command space, plan and parameters, and scored with core's own scorers; where the
machine lags its commands it follows every command exactly, 40 ms late."""

from __future__ import annotations

import _scoring_synth as T
import pytest
from _report_helpers import golden, in_repo
from bounded_embodiment import report as rep
from bounded_embodiment.commandspace import load_command_space
from bounded_embodiment.envelope import load
from bounded_embodiment.generator import encode_payload
from bounded_embodiment.rawdata import Command
from bounded_embodiment.scoring import ScoringInput, ev02

pytestmark = in_repo

LAG_S = 0.04  # the machine follows every command, this late


def _lagging_dead_stop(monkeypatch, drives, step_cap, lag_s=LAG_S, space_doc=None, point_of=None):
    """The golden synthetic machine on core's be-stops/1 stream, with a stop that does nothing:
    the tool follows every command ``lag_s`` late, the commands after the stop included."""
    G = golden()
    space = load_command_space(space_doc or G.COMMAND_SPACE)
    monkeypatch.setattr(G, "STOP_DRIVES", drives)
    params = dict(G.STOP_PARAMS, step_cap=step_cap)
    plan = rep.load_stop_plan(G.stop_plan_doc(G.EV02_TRIALS))
    segs = rep.split_segments(rep.stop_stream(space, plan, params))  # accepted by core
    times = G._segment_times(segs, [1.0] * (len(segs) - 1))
    point_of = point_of or (lambda p: (p["x"], p["y"], p["z"]))
    rows, pts = [], []
    for seg, ts in zip(segs, times):
        if pts:  # between trials the tool stays where the last command left it
            pts.append((ts[0] - 0.001 + lag_s, pts[-1][1]))
        for p, t in zip(seg, ts):
            stop = p.strategy.startswith("stop:")
            rows.append(Command(p.seq, t, t + 0.003, p.strategy, "{}" if stop else encode_payload(p.payload),
                                "triggered" if stop else "ok", None))
            if not stop:  # the stop does nothing
                pts.append((t + lag_s, point_of(p.payload)))
    script = G.Script(G.CENTRE)
    script.add(pts[0][0], G._follow([t for t, _ in pts], [x for _, x in pts]))
    samples = T.sample_rows(script, -0.5, rows[-1].t_sent_s + 1.0, 100.0, ("tool",), 0.0003)
    return ev02.score(ScoringInput(envelope=load(G.stop_envelope_doc()), samples=samples, commands=rows,
                                   accuracy=T.accuracy_rows(jitter_sigma_m=T.STOP_JITTER_M),
                                   declared=T.declared(), profile=T.ev02_profile(),
                                   plan=plan.for_scoring(space)))


# ----- finding 6: the walk rule looks at every step within the speed window ------------------------
def test_a_dead_stop_cannot_pass_when_the_carrying_axis_lands_just_before_the_stop(monkeypatch):
    """Finding 6 (M1 reopened): with the drives 10 mm closer, x, which carries the tool, lands on
    drive row 13 of 14, while z creeps on 20 um a row. A lagging machine still carries the tool at
    full speed through the stop, so both chords read an ok approach. Taking the moving axes from the
    last step alone left only z, the walk ran 3.0 s, about 0.75 m was credited, and the dead stop
    PASSed. x changed within the speed window before the stop, so it counts as moving, and the
    walk ends where it stops changing: at the stop."""
    out = _lagging_dead_stop(monkeypatch, {"x_plus": (0.365, 0.0, 0.179), "x_minus": (0.235, 0.0, 0.179)},
                             {"x": 0.005, "y": 0.005, "z": 0.00002})
    assert out["verdict"]["result"] != "PASS"
    for t in out["stops"]["trials"]:
        assert t["status"] == "inadequate", t
        assert t["walk_end_s"] == pytest.approx(t["stop_s"])


def test_a_gripper_that_walks_on_does_not_stand_for_the_axis_that_carries_the_tool(monkeypatch):
    """Finding 6, the reviewer's second case: a cartesian space with a gripper (percent, step cap
    1). x lands on drive row 13 of 14, as above, and the gripper walks on to 100. The gripper moves
    nothing the instrument sees, so its walk shows nothing about a stop."""
    G = golden()
    doc = dict(G.COMMAND_SPACE, axes=G.COMMAND_SPACE["axes"] + [
        {"key": "gripper", "unit": "percent", "lo": 0.0, "hi": 100.0, "home": 0.0}])
    doc["far_targets"] = [dict(f, values=dict(f["values"], gripper=0.0)) for f in G.COMMAND_SPACE["far_targets"]]
    monkeypatch.setattr(G, "stop_plan_doc", _with_gripper(G.stop_plan_doc))
    out = _lagging_dead_stop(monkeypatch, {"x_plus": (0.365, 0.0, 0.179), "x_minus": (0.235, 0.0, 0.179)},
                             {"x": 0.005, "y": 0.005, "z": 0.005, "gripper": 1.0}, space_doc=doc)
    assert out["verdict"]["result"] != "PASS"
    assert all(t["status"] == "inadequate" for t in out["stops"]["trials"])


def _with_gripper(make):
    """The golden stop plan with a gripper axis: closed (0) at the start, open (100) on every
    drive, so that the gripper walks on long after x has landed."""

    def doc(trials):
        d = make(trials)
        d["start"] = dict(d["start"], gripper=0.0)
        d["drives"] = [dict(x, command=dict(x["command"], gripper=100.0)) for x in d["drives"]]
        return d

    return doc


def test_the_walk_rule_takes_every_step_within_the_window():
    """The rule itself: an axis that changed in any step whose later row is within the speed window
    before the stop counts as moving, with the direction of its last change, and the last step
    before the stop always counts."""
    rows = [(0.00, '{"x": 0.0, "z": 0.0}'), (0.02, '{"x": 0.1, "z": 0.0}'), (0.04, '{"x": 0.2, "z": 0.1}'),
            (0.06, '{"x": 0.2, "z": 0.2}')]  # x lands one row before the stop; z moves on
    after = [(0.10, '{"x": 0.2, "z": 0.3}'), (0.12, '{"x": 0.2, "z": 0.4}')]
    # x moved within 0.2 s of the stop at 0.08 s: it does not change after the stop, so the walk ends there
    assert ev02.walk_end(rows, after, 0.08, window_s=0.2) == 0.08
    # with a window that holds only the last step, z alone moves, and the walk runs on
    assert ev02.walk_end(rows, after, 0.08, window_s=0.01) == 0.12
    # a step older than the window does not count: here x's, so z alone moves
    old = [(0.00, '{"x": 0.0, "z": 0.0}'), (0.02, '{"x": 0.1, "z": 0.0}'), (0.30, '{"x": 0.1, "z": 0.1}'),
           (0.32, '{"x": 0.1, "z": 0.2}')]
    later = [(0.36, '{"x": 0.1, "z": 0.3}'), (0.38, '{"x": 0.1, "z": 0.4}')]
    assert ev02.walk_end(old, later, 0.34, window_s=0.2) == 0.38
    # an axis that turned back within the window moves in the direction of its last change
    back = [(0.00, '{"x": 0.0}'), (0.02, '{"x": 0.1}'), (0.04, '{"x": 0.05}')]
    assert ev02.walk_end(back, [(0.08, '{"x": 0.0}'), (0.10, '{"x": 0.1}')], 0.06, window_s=0.2) == 0.08
    # no step at all before the stop: nothing moved, and the walk ends at the stop
    assert ev02.walk_end([(0.00, '{"x": 0.0}')], after, 0.02, window_s=0.2) == 0.02
