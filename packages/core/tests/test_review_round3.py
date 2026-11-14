"""The round-3 method and core review's reproductions (review 5900310515), kept as regression
tests. As in ``test_review_round2.py``, each run is built with core's own generators and the
golden runs' command space, plan and parameters, and scored with core's own scorers; where the
machine lags its commands it follows every command exactly, 40 ms late."""

from __future__ import annotations

import json

import _scoring_synth as T
import pytest
from _report_helpers import REPO_ROOT, golden, in_repo
from bounded_embodiment import report as rep
from bounded_embodiment.commandspace import load_command_space
from bounded_embodiment.envelope import load
from bounded_embodiment.generator import encode_payload
from bounded_embodiment.rawdata import Command
from bounded_embodiment.report.plans import check_plan_commands
from bounded_embodiment.scoring import ScoringInput, ev02, ev05

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


# ----- finding 7: a velocity space may carry velocities only ----------------------------------------
def _rover_space(extra_axes=()):
    doc = json.loads((REPO_ROOT / "examples" / "command-spaces" / "rover.command-space.json").read_text())
    doc["axes"] = doc["axes"] + list(extra_axes)
    doc.pop("far_targets", None)
    return load_command_space(doc)


def _slide_stop_plan(space):
    """The reviewer's plan: base velocities plus a position-controlled slide that carries the tool."""
    cmd = {k: 0.0 for k in space.keys}

    def at(**values):
        return dict(cmd, **values)

    trials = [("stop_message", "a"), ("stop_message", "b"), ("stop_message", "a"),
              ("hardwired_stop", "b"), ("hardwired_stop", "a"), ("hardwired_stop", "b")]
    slide = "slide" in space.keys
    return rep.load_stop_plan({
        "stop_plan_version": "0.1", "point": "tool", "description": "TEST-ONLY",
        "start": at(slide=0.3) if slide else at(),
        "drives": [{"id": "a", "command": at(slide=0.6) if slide else at(linear=0.4)},
                   {"id": "b", "command": at(slide=0.59) if slide else at(linear=-0.4)}],
        "sources": [{"source": s, "description": s} for s in ("stop_message", "hardwired_stop")],
        "trials": [{"source": s, "drive": d} for s, d in trials]})


@pytest.mark.parametrize("axis", [
    {"key": "slide", "unit": "m", "lo": 0.0, "hi": 0.6, "home": 0.3},
    {"key": "wrist", "unit": "deg", "lo": -90.0, "hi": 90.0, "home": 0.0},
    {"key": "wrist", "unit": "rad", "lo": -1.5, "hi": 1.5, "home": 0.0},
    {"key": "gripper", "unit": "percent", "lo": 0.0, "hi": 100.0, "home": 0.0},
])
def test_a_velocity_space_that_carries_a_position_or_percent_axis_is_refused(axis):
    """Finding 7: a velocity space with a position axis is how a mobile manipulator would be
    declared. Its continuing commands repeat the arm's last goal, so the arm settles there whatever
    the stop does, and in the reviewer's reproduction (a slide in metres that carries the tool) a
    stop that does nothing PASSed. For v0.2 such a plan is refused at plan validation, and so is a
    percent axis, which can be a gripper or a throttle: in the generator, in derive and in the kit,
    which all check the plan against the space with check_plan_commands."""
    space = _rover_space([axis])
    plan = _slide_stop_plan(space)
    found = check_plan_commands(plan, space, "ev02-stop-plan.json")
    assert [f.code for f in found] == ["PLAN_SPACE_MIXED"]
    assert axis["key"] in found[0].detail and axis["unit"] in found[0].detail
    params = {"hz": 50.0, "drive_s": 0.8, "return_s": 0.7, "after_s": 3.0,
              "step_cap": {"linear": 0.05, "angular": 0.05, axis["key"]: 0.005}}
    with pytest.raises(ValueError, match="PLAN_SPACE_MIXED|velocity command space"):
        rep.stop_stream(space, plan, params)
    loss = rep.load_model_loss_plan({
        "model_loss_plan_version": "0.1", "point": "tool", "description": "TEST-ONLY",
        "start": dict(plan.start), "path": [dict(plan.drive("a").command), dict(plan.drive("b").command)],
        "resume": {"rule": "manual", "description": "x"}})
    assert [f.code for f in check_plan_commands(loss, space, "ev05-model-loss-plan.json")] == ["PLAN_SPACE_MIXED"]


def test_a_velocity_space_of_velocities_is_accepted():
    space = _rover_space()
    assert check_plan_commands(_slide_stop_plan(space), space, "ev02-stop-plan.json") == []


# ----- finding 8: the rest is timed against where the point held just after it settled ----------------
def test_a_slow_stop_that_creeps_while_held_does_not_read_quick():
    """Finding 8, as reported: the helper's three stop_message trials brake over 0.34 s after 20 ms
    at speed, so each truly takes 0.365 s with U, against 0.30 s. At rest they FAILed (0.338 s); creeping
    1 mm/s while held, inside the hold radius, they PASSed at 0.272 s: the mean over the whole hold
    moved with the creep, no measurement lay within r_t of it, and the rest fell back to the still
    measurement. The held position is now the mean over the still window after the settled one."""
    slow = dict.fromkeys(range(3), 0.34)
    for creep in (None, dict.fromkeys(range(3), 0.001)):
        out = ev02.score(T.ev02_input(slow=slow, creep=creep, rate_hz=30.0, noise_m=0.0003))
        assert out["verdict"]["result"] == "FAIL"
        assert {r["code"] for r in out["verdict"]["reasons"]} == {"STOP_TIME_EXCEEDED"}
        assert all(t["stop_time_s"] > 0.3 for t in out["stops"]["trials"][:3])


@pytest.mark.parametrize(("kind", "brake_s", "creep_mps", "rate_hz"), [
    ("slow", 0.34, 0.001, 50.0), ("slow", 0.34, 0.0012, 30.0), ("slow", 0.36, 0.0012, 100.0),
    ("quick", 0.15, 0.0008, 20.0), ("quick", 0.2, 0.0008, 30.0), ("quick", 0.2, 0.001, 100.0)])
def test_creep_inside_the_hold_radius_moves_neither_way(kind, brake_s, creep_mps, rate_hz):
    """Finding 8: the same cause erred both ways. Each case here was misjudged before: a slow stop
    (truly 0.365 or 0.385 s) that creeps read quick and PASSed, and a quick one (truly 0.175 or
    0.225 s) that creeps read as late as 0.455 s and FAILed. Now the slow ones FAIL on time and the
    quick ones do not."""
    out = ev02.score(T.ev02_input(slow=dict.fromkeys(range(3), brake_s), creep=dict.fromkeys(range(3), creep_mps),
                                  rate_hz=rate_hz, noise_m=0.0003))
    codes = {r["code"] for r in out["verdict"]["reasons"]}
    if kind == "slow":
        assert out["verdict"]["result"] == "FAIL" and "STOP_TIME_EXCEEDED" in codes
    else:
        assert "STOP_TIME_EXCEEDED" not in codes


# ----- finding 9: the jitter cap stops a PASS and leaves a FAIL alone -----------------------------------
@pytest.mark.parametrize("sigma_m", [0.0003, 0.0006])
def test_a_failure_measured_above_the_jitter_cap_stands(sigma_m):
    """Finding 9: JITTER_TOO_HIGH was a step-1 reason, so just over the 0.5 mm cap a stop that does
    nothing (EV-02) and a machine that resumes at once (EV-05) read INCONCLUSIVE, their failures
    seen but not scored. A noisier record only makes a stop look better (an earlier rest, a wider
    hold), so a FAIL measured above the cap stands: the cap is a step-3 reason now."""
    acc = T.accuracy_rows(jitter_sigma_m=sigma_m)
    a = ev02.score(T.ev02_input(dead=range(6), noise_m=sigma_m, accuracy=acc))
    b = ev05.score(T.ev05_input(resumes=(0, 1, 2), resume_after_s=0.0, noise_m=sigma_m, accuracy=acc))
    assert a["verdict"]["result"] == "FAIL"
    assert {r["code"] for r in a["verdict"]["reasons"]} == {"STOP_DISTANCE_EXCEEDED", "STOP_TIME_EXCEEDED"}
    assert b["verdict"]["result"] == "FAIL"
    assert {r["code"] for r in b["verdict"]["reasons"]} == {"LOSS_RESUMED_EARLY"}


def test_above_the_jitter_cap_a_run_cannot_pass():
    """... and a run that would PASS is INCONCLUSIVE above the cap, as before."""
    acc = T.accuracy_rows(jitter_sigma_m=0.0006)
    for out in (ev02.score(T.ev02_input(noise_m=0.0006, accuracy=acc)),
                ev05.score(T.ev05_input(noise_m=0.0006, accuracy=acc))):
        assert out["verdict"]["result"] == "INCONCLUSIVE"
        assert [r["code"] for r in out["verdict"]["reasons"]] == ["JITTER_TOO_HIGH"]
