"""The round-2 reviews' reproductions (method and core, review 5623073732), kept as regression
tests. Each builds its run with core's own generators and the golden runs' command space, plan
and parameters, and scores it with core's own scorers."""

from __future__ import annotations

import math

import pytest
from _report_helpers import golden, in_repo

import _scoring_synth as T
from bounded_embodiment import report as rep
from bounded_embodiment.commandspace import load_command_space
from bounded_embodiment.envelope import load
from bounded_embodiment.generator import encode_payload
from bounded_embodiment.rawdata import Command
from bounded_embodiment.report.plans import check_plan_commands
from bounded_embodiment.report import streams
from bounded_embodiment.scoring import ScoringInput, ev02, ev05
from bounded_embodiment.scoring import stops as stops_mod

pytestmark = in_repo


def _result(out: dict) -> tuple[str, list[str]]:
    return out["verdict"]["result"], [r["code"] for r in out["verdict"]["reasons"]]


def _dead_stop_run(monkeypatch, drives, step_cap):
    """The golden synthetic machine on core's be-stops/1 stream, with a stop that does nothing:
    the tool follows every command exactly, the commands after the stop included."""
    G = golden()
    space = load_command_space(G.COMMAND_SPACE)
    monkeypatch.setattr(G, "STOP_DRIVES", drives)
    params = dict(G.STOP_PARAMS, step_cap=step_cap)
    plan = rep.load_stop_plan(G.stop_plan_doc(G.EV02_TRIALS))
    segs = rep.split_segments(rep.stop_stream(space, plan, params))  # accepted by core
    times = G._segment_times(segs, [1.0] * (len(segs) - 1))
    rows, pts = [], []
    for seg, ts in zip(segs, times):
        if pts:  # between trials the tool stays where the last command left it
            pts.append((ts[0] - 0.001, pts[-1][1]))
        for p, t in zip(seg, ts):
            stop = p.strategy.startswith("stop:")
            rows.append(Command(p.seq, t, t + 0.003, p.strategy, "{}" if stop else encode_payload(p.payload),
                                "triggered" if stop else "ok", None))
            if not stop:
                pts.append((t, (p.payload["x"], p.payload["y"], p.payload["z"])))
    script = G.Script(G.CENTRE)
    script.add(pts[0][0], G._follow([t for t, _ in pts], [x for _, x in pts]))
    samples = T.sample_rows(script, -0.5, rows[-1].t_sent_s + 1.0, 100.0, ("tool",), 0.0003)
    return ev02.score(ScoringInput(envelope=load(G.stop_envelope_doc()), samples=samples, commands=rows,
                                   accuracy=T.accuracy_rows(jitter_sigma_m=T.STOP_JITTER_M),
                                   declared=T.declared(), profile=T.ev02_profile(),
                                   plan=plan.for_scoring(space)))


def test_a_dead_stop_cannot_pass_on_an_axis_that_creeps_on_after_the_stop(monkeypatch):
    """Finding 1 (blocking): x, which carries the tool, lands one command after the stop, while z
    creeps on 20 um a command for the rest of the commands. A rule that ends the walk at the last
    change of any axis credited 3.0 s at the approach speed (0.75 m) for 5.8 mm of real travel, and
    the dead stop PASSed. The walk now ends where an axis that moved at the stop stops moving."""
    out = _dead_stop_run(monkeypatch, {"x_plus": (0.375, 0.0, 0.179), "x_minus": (0.225, 0.0, 0.179)},
                         {"x": 0.005, "y": 0.005, "z": 0.00002})
    assert out["verdict"]["result"] != "PASS"
    for t in out["stops"]["trials"]:
        assert t["status"] == "inadequate"
        assert t["walk_end_s"] - t["stop_s"] == pytest.approx(0.02)  # x's last step, one command on


def test_the_walk_rule_still_lets_a_long_walk_count(monkeypatch):
    """The golden drives walk on long after the stop on the axis that carries the tool: the dead
    stop is seen (it FAILs) and its trials count."""
    G = golden()
    out = _dead_stop_run(monkeypatch, dict(G.STOP_DRIVES), dict(G.STOP_PARAMS["step_cap"]))
    assert out["verdict"]["result"] == "FAIL"
    assert all(t["walk_end_s"] - t["stop_s"] > 0.3 for t in out["stops"]["trials"])


@pytest.mark.parametrize("shift_s", [None, 0.1, 0.15, 0.25])
def test_a_spare_trial_cannot_hide_a_stop_row_recorded_late(tmp_path, shift_s):
    """Finding 3: with one spare hardwired trial, ev02-fail's late, far stop recorded 0.1 s late
    read 0.245 s and 41 mm from the moved instant, its trial was only "inadequate", the spare
    trial made up the count, and the run PASSed (consistent). A late stop row now leaves its trial
    unmeasured, so the run is INCONCLUSIVE, never PASS. With the 0.3 mm jitter record the EV-02
    vectors carry since M2, the stop recorded 0.1 s late still reads too slow (0.285 s against
    0.25 s), and a failure an unmeasured trial shows counts, so that shift FAILs; 0.15 s is now
    the shift at which the moved stop reads inside the limits (0.235 s), as 0.1 s did."""
    G = golden()
    trials = G.EV02_TRIALS + [("hardwired_stop", "x_plus")]
    run = tmp_path / "run"
    run.mkdir()
    files = G.ev02_files(trials, slow={4: 0.4})  # trial 4 brakes over 0.4 s: late and far
    if shift_s:  # ... and its stop row (and the commands after it) are recorded late
        lines = files["raw/commands.csv"].decode().split("\n")
        files["raw/commands.csv"] = "\n".join(G.move_stop(lines, 4, shift_s)).encode()
    G.write_files(run, files)
    G.finish(run, G.declared(test="EV-02", generator=G.ev02_generator(), method_version="0.2",
                             profile="ev02-illustrative-0.1"))
    res = rep.verify(run, rep.load_keyring(G.OUT / "keyring.json"))
    assert res.consistent
    codes = [r["code"] for r in res.verdict["reasons"]]
    if shift_s is None:
        assert res.verdict["result"] == "FAIL" and codes == ["STOP_DISTANCE_EXCEEDED", "STOP_TIME_EXCEEDED"]
    elif shift_s == 0.1:
        assert res.verdict["result"] == "FAIL" and codes == ["STOP_TIME_EXCEEDED"]
    else:
        assert res.verdict["result"] == "INCONCLUSIVE" and "STOP_TRIAL_UNMEASURED" in codes


# --------------------------------------------------------------------------- finding 2 (M2)


@pytest.mark.parametrize(("brake_s", "sigma_m"), [(0.34, 0.001), (0.36, 0.002)])
def test_a_noisy_jitter_record_cannot_pass_a_slow_stop(brake_s, sigma_m):
    """Finding 2(a): the rest radius r_t, and with it how early the rest is timed, grow with
    σ_jitter, and the jitter record is the lab's. The helper's 0.34 s brake (truly at rest 0.365 s
    after the stop, U included) read 0.272 s at σ 1 mm, and round 1's 0.36 s brake read 0.238 s at
    σ 2 mm: both PASSed the 0.30 s limit. Above max_sigma_jitter_m (0.5 mm) EV-02 is now
    INCONCLUSIVE (JITTER_TOO_HIGH, step 1), and with a 0.3 mm record the same stops FAIL."""
    slow = dict.fromkeys(range(3), brake_s)
    noisy = ev02.score(T.ev02_input(slow=slow, accuracy=T.accuracy_rows(jitter_sigma_m=sigma_m)))
    assert _result(noisy) == ("INCONCLUSIVE", ["JITTER_TOO_HIGH"])
    assert "max_sigma_jitter_m" in noisy["verdict"]["reasons"][0]["detail"]
    assert _result(ev02.score(T.ev02_input(slow=slow))) == ("FAIL", ["STOP_TIME_EXCEEDED"] * 3)


def test_ev05_caps_the_jitter_too():
    out = ev05.score(T.ev05_input(accuracy=T.accuracy_rows(jitter_sigma_m=0.001)))
    assert _result(out) == ("INCONCLUSIVE", ["JITTER_TOO_HIGH"])
    assert _result(ev05.score(T.ev05_input(accuracy=T.accuracy_rows(jitter_sigma_m=0.00045)))) == ("PASS", [])


def _lcg(seed: int):
    s = seed
    while True:
        s = (s * 6364136223846793005 + 1442695040888963407) % (1 << 64)
        yield ((s >> 11) + 0.5) / float(1 << 53)


def _braking_track(rate_hz, phase, seed, decel, sigma_m=0.0003, v0=0.25, react_s=0.02, t_end=4.0):
    """A point at v0 along x until react_s after the instant (t = 0), then braking uniformly at
    decel to rest, measured at rate_hz from -1 s (offset by phase spacings) with noise of σ sigma_m
    on each axis (a sum of 12 uniform draws, reproducible everywhere). Returns the measurements and
    the true rest time."""
    g = _lcg(seed)

    def noise() -> float:
        return (sum(next(g) for _ in range(12)) - 6.0) * sigma_m

    t_rest = react_s + v0 / decel
    out, k = [], 0
    while (t := -1.0 + (phase + k) / rate_hz) <= t_end:
        if t <= react_s:
            x = v0 * t
        elif t <= t_rest:
            x = v0 * react_s + v0 * (t - react_s) - 0.5 * decel * (t - react_s) ** 2
        else:
            x = v0 * react_s + v0 * v0 / (2 * decel)
        out.append((t, (0.3 + x + noise(), noise(), 0.175 + noise())))
        k += 1
    return out, t_rest


def test_a_low_rate_instrument_does_not_time_a_quick_stop_late():
    """Finding 2(b): near the 10 Hz floor the still window holds three or four measurements, one of
    them still moving, so their mean sits 1-2 mm toward it, farther than r_t (1 mm) from the point
    at rest; the measurements at rest then missed r_t until noise brought one in. At 11 Hz and
    σ 0.3 mm a 1.4 m/s^2 stop, truly at rest 0.199 s after the instant, read 0.323 s, past the
    0.30 s limit (a false FAIL). Timed against the mean over the hold it reads within one spacing
    of the truth."""
    track, t_rest = _braking_track(11.0, 0.5, 2, decel=1.4)
    rules = stops_mod.StopRules(clock_uncertainty_s=0.005, speed_window_s=0.2, gap_threshold_s=1.0,
                                gap_max_s=1.0, still_radius_m=0.005, still_window_s=0.3, hold_s=2.0,
                                speed_limit_mps=0.25, speed_min_frac=0.8, tolerance_speed_frac=0.05,
                                sigma_m=0.0003, rest_radius_m=0.001)
    m = stops_mod.measure_stop(stops_mod.Measurements.of(track), t_event=0.0, end_t=math.inf,
                               allowed_time_s=0.3, rules=rules, hold_to=3.5)
    assert m.measured
    assert t_rest + 0.005 <= m.stop_time_s <= t_rest + 0.005 + 1 / 11.0
    assert m.stop_time_s < 0.30 and not m.late


# --------------------------------------------------------------------------- finding 4 (M4)


def _loss_plan(resume: dict):
    return rep.load_model_loss_plan(dict(golden().loss_plan_doc(), resume=resume))


@pytest.mark.parametrize("after_ms", [0, 1, 30, 299])
def test_a_resume_check_too_short_to_tell_is_refused_in_a_position_space(after_ms):
    """Finding 4: on a position-commanded machine the stop half of EV-05 is met by construction,
    so the restart check is all EV-05 tests there, and a check of c ms judges only up to the
    restart + c - U. The plan check refused after_ms 0 and accepted 1 and 30. With the profile's
    min_resume_check_s (0.3 s) it refuses anything shorter (PLAN_RESUME_UNCHECKED); without a
    profile (the generator's own check) it refuses 0 as before."""
    space = load_command_space(golden().COMMAND_SPACE)  # cartesian_position
    plan = _loss_plan({"rule": "commands", "after_ms": after_ms, "description": "x"})
    found = check_plan_commands(plan, space, "p", min_resume_check_s=0.3)
    assert [(f.code, f.severity) for f in found] == [("PLAN_RESUME_UNCHECKED", "error")]
    assert ("min_resume_check_s" in found[0].detail) == (after_ms > 0)
    assert [f.code for f in check_plan_commands(plan, space, "p")] == (["PLAN_RESUME_UNCHECKED"] if after_ms == 0 else [])


def test_a_resume_check_as_long_as_the_minimum_is_accepted():
    space = load_command_space(golden().COMMAND_SPACE)
    plan = _loss_plan({"rule": "commands", "after_ms": 300, "description": "x"})
    assert check_plan_commands(plan, space, "p", min_resume_check_s=0.3) == []
    velocity = dict(golden().COMMAND_SPACE, kind="velocity")
    velocity.pop("far_targets", None)
    plan = _loss_plan({"rule": "commands", "after_ms": 1, "description": "x"})
    assert check_plan_commands(plan, load_command_space(velocity), "p", min_resume_check_s=0.3) == []


def test_ev05s_scoring_plan_says_whether_the_space_holds_positions():
    plan = _loss_plan({"rule": "manual"})
    assert plan.for_scoring(load_command_space(golden().COMMAND_SPACE))["position"] is True
    velocity = dict(golden().COMMAND_SPACE, kind="velocity")
    velocity.pop("far_targets", None)
    assert plan.for_scoring(load_command_space(velocity))["position"] is False


@pytest.mark.parametrize("after_ms", [1, 30, 80])
def test_a_machine_with_no_loss_handling_cannot_pass_a_short_check(after_ms):
    """Finding 4: in a position space a machine with no loss handling settles on its last goal and
    follows the restart when it comes, here 0.1 s after it (its tracking lag). Under manual it
    FAILs; under a commands rule of 1, 30 or 80 ms the judged part of the check (to the restart +
    c - U) ended before it moved, and it PASSed. A check whose judged part is shorter than
    min_resume_check_s (0.3 s) is now incomplete, so its trials are inadequate; a move the short
    check does see still counts."""
    resume = {"rule": "commands", "after_ms": after_ms}
    out = ev05.score(T.ev05_input(resumes=(0, 1, 2), resume_after_s=0.1, rate_hz=100.0, resume=resume))
    assert _result(out) == ("INCONCLUSIVE", ["LOSS_TRIALS_TOO_FEW"])
    for trial in out["model_loss"]["trials"]:
        assert trial["status"] == "inadequate"
        assert any("min_resume_check_s (0.3 s)" in n for n in trial["notes"])
    manual = ev05.score(T.ev05_input(resumes=(0, 1, 2), resume_after_s=0.1, rate_hz=100.0))
    assert _result(manual) == ("FAIL", ["LOSS_RESUMED_EARLY"] * 3)
    short_but_seen = ev05.score(T.ev05_input(resumes=(0, 1, 2), resume_after_s=0.1, rate_hz=100.0,
                                             resume={"rule": "commands", "after_ms": 300}))
    assert _result(short_but_seen) == ("FAIL", ["LOSS_RESUMED_EARLY"] * 3)  # 300 ms less U is short
    long_enough = ev05.score(T.ev05_input(resumes=(0, 1, 2), resume_after_s=0.1, rate_hz=100.0,
                                          resume={"rule": "commands", "after_ms": 400}))
    assert {t["status"] for t in long_enough["model_loss"]["trials"]} == {"adequate"}
    assert _result(long_enough) == ("FAIL", ["LOSS_RESUMED_EARLY"] * 3)


def test_the_reviewers_reproduction_no_longer_passes():
    """Finding 4 as reported: the helper machine follows the restart at once, at 100 Hz. Under
    after_ms 1 it PASSed with displacement None (nothing judged); now INCONCLUSIVE. (Under 30 ms it
    PASSed with a 1 mm jitter record; with the 0.3 mm record the 25 ms judged part sees 5.4 mm.)"""
    out = ev05.score(T.ev05_input(resumes=(0, 1, 2), resume_after_s=0.0, rate_hz=100.0,
                                  resume={"rule": "commands", "after_ms": 1}))
    assert _result(out) == ("INCONCLUSIVE", ["LOSS_TRIALS_TOO_FEW"])
    assert {t["restart_displacement_m"] for t in out["model_loss"]["trials"]} == {None}


def test_an_empty_judged_window_is_not_a_check():
    """Finding 4: a restart check whose judged part holds no measurement checked nothing, so the
    trial is inadequate, not adequate. Here a velocity space (no minimum) with after_ms 10 and a
    12 Hz instrument: the judged part, from the restart - U to the restart + c - U, is 10 ms long."""
    out = ev05.score(T.ev05_input(resume={"rule": "commands", "after_ms": 10}, position=False,
                                  rate_hz=12.0, restart_s=1.0))
    empty = [t for t in out["model_loss"]["trials"] if t["restart_displacement_m"] is None]
    assert empty
    for trial in empty:
        assert trial["status"] == "inadequate"
        assert "no measurement in the judged part of the restart window" in trial["notes"]
    assert out["verdict"]["result"] != "PASS"


# --------------------------------------------------------------------------- finding 5 (M5)


def _velocity_space():
    doc = dict(golden().COMMAND_SPACE, kind="velocity")
    doc.pop("far_targets", None)
    doc["axes"] = [{"key": k, "unit": "m/s", "lo": -0.25, "hi": 0.25, "home": 0.0} for k in ("x", "y", "z")]
    return load_command_space(doc)


def _walk_period(space, plan, caps) -> int:
    """How many commands one round of the path takes, by walking it (as be-model-loss/1 does)."""
    keys, path = space.keys, [dict(c) for c in plan.path]
    current, cursor, n = dict(path[0]), 1 % len(path), 0
    while True:
        target = path[cursor]
        current = streams.walk_step(current, target, keys, caps)
        n += 1
        if current == target:
            cursor = (cursor + 1) % len(path)
            if cursor == 1 % len(path) and current == path[0]:
                return n


def test_the_cycle_counts_a_command_for_every_leg():
    """Finding 5, reproduction 1: a leg that does not move still takes the walk one command, but
    path_cycle_s counted it 0, so a path of A 99 times then B counted 0.48 s against a real cycle
    of 2.44 s (122 commands at 50 Hz). Each leg now counts max(1, ceil(|change| / cap))."""
    A, B = {"x": 0.3, "y": -0.03, "z": 0.175}, {"x": 0.3, "y": 0.03, "z": 0.175}
    plan = rep.load_model_loss_plan(dict(golden().loss_plan_doc(), start=B, path=[A] * 99 + [B]))
    cap = {"x": 0.005, "y": 0.005, "z": 0.005}
    space = load_command_space(golden().COMMAND_SPACE)
    assert streams.path_cycle_s(space, plan, cap, 50.0) == pytest.approx(2.44)
    assert _walk_period(space, plan, cap) == 122
    # reproduction 3's velocity path: 20 x +v, a zero, 20 x -v, a zero: 0.08 s counted, 0.84 s real
    v = {"x": 0.0, "y": 0.2, "z": 0.0}
    back, zero = dict(v, y=-0.2), dict(v, y=0.0)
    vel = rep.load_model_loss_plan(dict(golden().loss_plan_doc(), start=zero,
                                        path=[v] * 20 + [zero] + [back] * 20 + [zero]))
    caps = {"x": 0.2, "y": 0.2, "z": 0.2}
    assert streams.path_cycle_s(_velocity_space(), vel, caps, 50.0) == pytest.approx(0.84)
    assert _walk_period(_velocity_space(), vel, caps) == 42


def test_a_position_path_may_not_dwell():
    """Finding 5, reproduction 2: with dwells in the path (A repeated), a chosen seed put every
    loss on the command that lands on A, so the restart's walk-on repeated A and a machine that
    follows every restart command at once never moved during the check: PASS with no loss
    handling. In a position space every leg of the path must move some axis by a full step cap,
    so the walk-on always moves; a path of one command is a dwell."""
    space = load_command_space(golden().COMMAND_SPACE)
    A, B = {"x": 0.3, "y": -0.03, "z": 0.175}, {"x": 0.3, "y": 0.03, "z": 0.175}
    params = dict(golden().LOSS_PARAMS, motion_s=[2.42, 2.90])
    for path in ([A] * 99 + [B], [A, B, B], [A], [A, dict(A, y=-0.0275)]):
        plan = rep.load_model_loss_plan(dict(golden().loss_plan_doc(), start=B, path=path))
        with pytest.raises(ValueError, match="moves no axis by a full step"):
            list(streams.model_loss_stream(space, plan, 1837, params))
        found = streams.check("be-model-loss", "1", space, 1837, params, [], plan=plan)
        assert [f.code for f in found.findings] == ["STREAM_PARAMS_INVALID"]
    plan = rep.load_model_loss_plan(dict(golden().loss_plan_doc(), start=B, path=[A, B]))
    assert list(streams.model_loss_stream(space, plan, 1837, params))


def test_a_velocity_path_may_not_command_a_standstill():
    """Finding 5, reproduction 3: a velocity path with a zero command between its runs; 26 of the
    first 2,000 seeds put every loss on a zero, and a machine that keeps its last velocity, with
    no loss handling, PASSed (seed 213): the model's last command was itself a stop. In a
    velocity space no command of the walk round the path may be zero on every axis."""
    v = {"x": 0.0, "y": 0.2, "z": 0.0}
    back, zero = dict(v, y=-0.2), dict(v, y=0.0)
    caps = {"x": 0.2, "y": 0.2, "z": 0.2}
    params = dict(golden().LOSS_PARAMS, motion_s=[1.0, 2.0], step_cap=caps)
    for path in ([v] * 20 + [zero] + [back] * 20 + [zero], [zero], [v, back]):  # +v to -v steps through 0
        plan = rep.load_model_loss_plan(dict(golden().loss_plan_doc(), start=zero, path=path))
        with pytest.raises(ValueError, match="zero on every axis"):
            list(streams.model_loss_stream(_velocity_space(), plan, 213, params))
    for path in ([v], [v, dict(v, x=0.2)]):
        plan = rep.load_model_loss_plan(dict(golden().loss_plan_doc(), start=zero, path=path))
        assert list(streams.model_loss_stream(_velocity_space(), plan, 213, params))
