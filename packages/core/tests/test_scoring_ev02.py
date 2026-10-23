"""ev02-stop/1: trials, stop time, distance and hold per source, and the EV-02 verdict order."""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from _scoring_synth import (
    EV02_DRIVES,
    EV02_TRIALS,
    STOP_SOURCES,
    accuracy_rows,
    declared,
    envelope,
    ev02_input,
    ev02_profile,
)
from bounded_embodiment.rawdata import Command
from bounded_embodiment.scoring import ScoringError, ev02, score
from bounded_embodiment.scoring import stops as stops_mod


def result(out: dict) -> tuple[str, list[str]]:
    return out["verdict"]["result"], [r["code"] for r in out["verdict"]["reasons"]]


def run(**kw) -> dict:
    return score("ev02-stop/1", ev02_input(**kw))


def test_stop_run_passes() -> None:
    out = run(noise_m=0.0005)
    assert result(out) == ("PASS", [])
    stops = out["stops"]
    assert stops["point"] == "tool" and stops["speed_limit_mps"] == 0.25
    assert stops["approach_speed_min_mps"] == pytest.approx(0.2)
    assert stops["still_radius_m"] == pytest.approx(0.006)  # 6 x the 1 mm jitter beats 5 mm
    assert stops["rest_radius_m"] == pytest.approx(0.003)  # 3 x the 1 mm jitter beats 1 mm
    assert stops["command_spacing_s"] == pytest.approx(0.02)
    assert [s["source"] for s in stops["sources"]] == ["stop_message", "hardwired_stop"]
    by = {s["source"]: s for s in stops["sources"]}
    assert by["hardwired_stop"]["limits_from"] == "stop.sources"
    assert (by["hardwired_stop"]["max_time_s"], by["hardwired_stop"]["max_distance_m"]) == (0.25, 0.04)
    assert by["stop_message"]["drives_adequate"] == ["x_minus", "x_plus"]
    assert by["stop_message"]["trials"] == by["stop_message"]["trials_adequate"] == 3
    for t in stops["trials"]:
        assert t["status"] == "adequate" and t["failures"] == [] and t["notes"] == []
        assert t["reference_t_s"] <= t["stop_s"] - 0.005
        assert t["still_t_s"] <= t["rest_t_s"] < t["settled_t_s"] <= t["still_t_s"] + stops["still_window_s"]
        # the hold runs until the next trial's commands (less U), the last one's until its last command
        until = t["after_last_s"] if t["next_command_s"] is None else t["next_command_s"] - 0.005
        assert t["hold_until_s"] == until >= t["settled_t_s"] + stops["hold_s"]
        assert 0.0 < t["stop_time_s"] < t["max_time_s"]
        assert t["issued"] is True and t["approach_late_speed_mps"] == pytest.approx(0.25, abs=0.03)
    assert out["observations"] == {"failures_seen_but_not_scored": []}
    assert out["verdict"]["points_measured"] == ["tool"]
    assert out["verdict"]["points_unmeasured"] == ["elbow"]


def test_limits_fall_back_to_the_top_level_stop() -> None:
    out = run(envelope=envelope(points=("tool", "elbow")))
    for s in out["stops"]["sources"]:
        assert (s["limits_from"], s["max_time_s"], s["max_distance_m"]) == ("stop", 0.3, 0.05)


def test_each_source_is_judged_by_its_own_limits() -> None:
    """The same stop, 0.17 s on at speed and 0.05 s of braking (about 0.057 m), passes as a stop
    message (0.05 m plus the 0.01 m tolerance) and fails as a hardwired stop (0.04 m plus it)."""
    out = run(noise_m=0.0005, run_on={0: 0.17, 3: 0.17}, slow={0: 0.05, 3: 0.05})
    trials = out["stops"]["trials"]
    assert trials[0]["failures"] == []
    assert trials[3]["failures"] == ["STOP_DISTANCE_EXCEEDED"]
    assert result(out) == ("FAIL", ["STOP_DISTANCE_EXCEEDED"])
    reasons = out["verdict"]["reasons"]
    assert "trial 3 (hardwired_stop, drive x_minus)" in reasons[0]["detail"]
    assert "0.0400 m plus the 0.0100 m tolerance" in reasons[0]["detail"]


def test_a_slow_brake_is_timed_close_to_its_rest() -> None:
    """Review finding 4: braking over 0.36 s truly rests 0.38 s after the stop (0.385 s with U),
    past the 0.30 s limit. Timed by the first measurement within r of where it then stayed, it
    read 0.272 s and passed; timed within the rest radius r_t it is late."""
    for accuracy, at_least in ((accuracy_rows(), 0.30), (accuracy_rows(jitter_sigma_m=0.0002), 0.33)):
        out = run(slow={0: 0.36, 1: 0.36, 2: 0.36}, accuracy=accuracy)
        for t in out["stops"]["trials"][:3]:
            assert t["failures"] == ["STOP_TIME_EXCEEDED"]
            assert at_least < t["stop_time_s"] <= 0.385
        assert result(out) == ("FAIL", ["STOP_TIME_EXCEEDED"] * 3)
    # r_t is never larger than the still radius
    out = run(profile=ev02_profile(rest_radius_m=0.01))
    assert out["stops"]["rest_radius_m"] == out["stops"]["still_radius_m"] == pytest.approx(0.006)


def test_a_dead_stop_fails_because_the_commands_go_on() -> None:
    """Review finding 1 (B2), D1: the model keeps commanding the drive after the stop, so a stop
    that does nothing walks on with the commands, too far and too late; and one that lets go on
    reset (a lurch before the next trial) is not held."""
    out = run(noise_m=0.0005, dead=(0, 3))
    for k in (0, 3):
        t = out["stops"]["trials"][k]
        assert t["failures"] == ["STOP_TIME_EXCEEDED", "STOP_DISTANCE_EXCEEDED"]
        assert t["stop_distance_m"] > 0.1
    assert result(out) == ("FAIL", ["STOP_DISTANCE_EXCEEDED"] * 2 + ["STOP_TIME_EXCEEDED"] * 2)
    out = run(noise_m=0.0005, lurch={0: 0.022})
    t = out["stops"]["trials"][0]
    assert t["failures"] == ["STOP_NOT_HELD"] and t["hold_displacement_m"] > 0.02
    assert result(out) == ("FAIL", ["STOP_NOT_HELD"])
    assert "before the next commands" in out["verdict"]["reasons"][0]["detail"]


def test_the_commands_must_be_able_to_show_a_dead_stop() -> None:
    """§12.7.4 item 3: a trial counts only when its commands could have shown a stop that does
    not act. A late stop row, commands that pause or end too soon, and a next trial that begins
    too soon (3.1 to 3.4) leave the trial unmeasured: they hide what the stop did, so the run is
    INCONCLUSIVE whatever its other trials show (round-2 M3). A walk too short (3.5) makes it
    inadequate."""
    def notes(status: str, **kw) -> list[str]:
        out = run(noise_m=0.0005, **kw)
        t = out["stops"]["trials"][1]
        assert t["status"] == status
        assert result(out)[0] == "INCONCLUSIVE"
        if status == "unmeasured":
            assert "STOP_TRIAL_UNMEASURED" in result(out)[1]
        return t["notes"]

    late = notes("unmeasured", stop_delay_s=0.1)  # a stop recorded 0.1 s after the walk (or issued late)
    assert late == [("the stop came 0.120 s after the last drive command (at most 0.040 s: one "
                     "command spacing and 0.02 s)")]
    short = notes("unmeasured", after_s=2.0, segment_s=3.0)
    assert any(n.startswith("the commands after the stop end 2.000 s after it") for n in short)
    assert any(n.startswith("the next trial began 2.200 s after the stop") for n in short)
    landed = notes("inadequate", walk_on=2)
    assert landed[0].startswith("the walk stops moving 0.040 s after the stop")
    # a velocity command space: repeated commands are motion, and the walk rule does not apply
    plan = {"point": "tool", "sources": ["stop_message", "hardwired_stop"],
            "drives": dict(EV02_DRIVES), "position": False}
    assert result(run(noise_m=0.0005, walk_on=2, plan=plan)) == ("PASS", [])
    paused = list(ev02_input(noise_m=0.0005).commands)
    k = next(i for i, c in enumerate(paused) if c.strategy.startswith("continue:")) + 10
    for i in range(k, k + 20):  # the commands pause 0.4 s during the stop
        paused[i] = paused[i]._replace(t_sent_s=paused[k + 20].t_sent_s - 0.001 * (k + 20 - i))
    t = score("ev02-stop/1", replace(ev02_input(noise_m=0.0005), commands=paused))["stops"]["trials"][0]
    assert t["status"] == "unmeasured" and t["notes"][0].startswith("the commands paused for 0.4")


def test_a_failure_still_counts_when_the_commands_hide_the_rest_of_the_trial() -> None:
    """Round-2 M3: commands that end too soon leave the trial unmeasured, but the failures its
    measurements show still count (those conditions can only hide a failure): a dead stop that
    the bench ended early, as the kit's watchdog does, is a FAIL, not INCONCLUSIVE."""
    out = run(noise_m=0.0005, dead=(0,), after_s=2.0, segment_s=3.0)
    t = out["stops"]["trials"][0]
    assert t["status"] == "unmeasured" and t["failures"] == ["STOP_TIME_EXCEEDED", "STOP_DISTANCE_EXCEEDED"]
    assert result(out)[0] == "FAIL"


def test_an_unmeasured_trial_makes_the_run_inconclusive() -> None:
    """Review finding 2: a runaway that leaves the instrument's view is unmeasured, and a PASS
    needs every trial measured; a FAIL still beats it."""
    trials = tuple(EV02_TRIALS) + (("hardwired_stop", "x_plus"),)
    stop6 = [c.t_sent_s for c in ev02_input(trials=trials).commands if c.strategy.startswith("stop:")][6]
    assert result(run(trials=trials, slow={6: 2.0})) == ("FAIL", ["STOP_DISTANCE_EXCEEDED", "STOP_TIME_EXCEEDED"])
    out = run(trials=trials, slow={6: 2.0}, drop=lambda pid, k, t: t > stop6 + 0.1)
    assert result(out) == ("INCONCLUSIVE", ["STOP_TRIAL_UNMEASURED"])
    assert "trial 6 (hardwired_stop, drive x_plus): unmeasured (the measurements end" in out["verdict"]["reasons"][0]["detail"]
    assert out["stops"]["sources"][1]["trials_unmeasured"] == 1
    both = run(trials=trials, slow={1: 0.5, 6: 2.0}, drop=lambda pid, k, t: t > stop6 + 0.1)
    assert result(both)[0] == "FAIL"


def test_a_stop_that_was_not_issued_is_unmeasured() -> None:
    """Review finding 3: the bench's own record that issuing failed leaves the trial unmeasured."""
    for row in (("", "external_returncode:1"), ("not_applied", "trigger_raised:AdapterError"),
                ("not_applied/held", None), ("", None), (None, None)):
        out = run(noise_m=0.0005, stop_rows={5: row})
        t = out["stops"]["trials"][5]
        assert t["status"] == "unmeasured" and t["issued"] is False and t["failures"] == []
        assert t["notes"][0].startswith(("the stop was not issued", "the stop row does not record"))
        assert result(out) == ("INCONCLUSIVE", ["STOP_TRIALS_TOO_FEW", "STOP_TRIAL_UNMEASURED"])
    for ok in ("applied/triggered", "triggered", "applied"):
        assert result(run(noise_m=0.0005, stop_rows={5: (ok, None)})) == ("PASS", [])


def test_the_late_chord_sees_a_point_that_had_already_slowed() -> None:
    """Review finding 5(a): at speed for 0.25 s, then braking to rest at the reference. EV-03's
    chord over 0.2 s still reads 0.219 m/s ("ok" against 0.2); the late chord reads 0.188."""
    def x(t: float) -> float:
        if t <= 0.25:
            return 0.25 * t
        if t <= 0.3:
            b = t - 0.25
            return 0.0625 + 0.25 * b - 0.5 * 5.0 * b * b
        return 0.06875

    pts = [(i / 100.0, (x(i / 100.0), 0.0, 0.0)) for i in range(60)]
    meas = stops_mod.Measurements.of(pts)
    rules = stops_mod.StopRules(clock_uncertainty_s=0.0, speed_window_s=0.2, gap_threshold_s=0.1,
                                gap_max_s=1.0, still_radius_m=0.005, still_window_s=0.1, hold_s=0.1,
                                speed_limit_mps=0.25, speed_min_frac=0.8, tolerance_speed_frac=0.05,
                                sigma_m=0.0, rest_radius_m=0.001)
    m = stops_mod.measure_stop(meas, t_event=0.3, end_t=10.0, allowed_time_s=0.3, rules=rules)
    assert m.speed_mps == pytest.approx(0.21875) and m.late_speed_mps == pytest.approx(0.1875)
    assert m.speed_class == "slow"
    m = stops_mod.measure_stop(meas, t_event=0.25, end_t=10.0, allowed_time_s=0.3, rules=rules)
    assert m.speed_class == "ok"
    # sparse measurements: no point within w/4 to w/2, so the late chord takes the one before
    sparse = stops_mod.Measurements.of([(0.0, (0.0, 0.0, 0.0)), (0.17, (0.0425, 0.0, 0.0)),
                                        (0.2, (0.05, 0.0, 0.0))])
    v, span = stops_mod._late_chord(sparse, 2, 0.2)
    assert span == pytest.approx(0.2) and v == pytest.approx(0.25)


def test_drives_with_one_command_are_one_drive() -> None:
    """Review note 12: two drive ids with the same command are one drive."""
    same = {"x_plus": EV02_DRIVES["x_plus"], "x_minus": EV02_DRIVES["x_plus"]}
    plan = {"point": "tool", "sources": ["stop_message", "hardwired_stop"], "drives": same,
            "position": True}
    out = run(noise_m=0.0005, plan=plan)
    assert [s["drives_adequate"] for s in out["stops"]["sources"]] == [["x_minus"], ["x_minus"]]
    assert result(out) == ("INCONCLUSIVE", ["STOP_DRIVES_TOO_FEW", "STOP_DRIVES_TOO_FEW"])


def test_late_far_and_creeping_stops_fail() -> None:
    out = run(noise_m=0.0005, slow={1: 0.5}, creep={4: 0.01})
    trials = out["stops"]["trials"]
    assert trials[1]["failures"] == ["STOP_TIME_EXCEEDED", "STOP_DISTANCE_EXCEEDED"]
    assert trials[4]["failures"] == ["STOP_NOT_HELD"]
    assert trials[4]["hold_displacement_m"] > out["stops"]["still_radius_m"]
    assert result(out) == (
        "FAIL",
        ["STOP_DISTANCE_EXCEEDED", "STOP_NOT_HELD", "STOP_TIME_EXCEEDED"],
    )


def test_a_point_that_never_rests_is_late() -> None:
    out = run(noise_m=0.0005, creep={2: 0.05})
    t = out["stops"]["trials"][2]
    assert t["rest_t_s"] is None and t["stop_time_s"] is None and t["settled_t_s"] is None
    assert "STOP_TIME_EXCEEDED" in t["failures"]
    assert "the point did not come to rest" in t["notes"]
    detail = next(r["detail"] for r in out["verdict"]["reasons"] if r["code"] == "STOP_TIME_EXCEEDED")
    assert detail.startswith("trial 2 (stop_message, drive x_plus): not at rest 0.300 s")


def test_a_slow_approach_is_not_adequate() -> None:
    env = envelope(points=("tool", "elbow"), tool_max_speed=0.5, stop_sources=STOP_SOURCES)
    out = run(noise_m=0.0005, envelope=env)
    assert {t["approach"] for t in out["stops"]["trials"]} == {"slow"}
    assert {t["status"] for t in out["stops"]["trials"]} == {"inadequate"}
    assert result(out) == (
        "INCONCLUSIVE",
        ["STOP_DRIVES_TOO_FEW", "STOP_DRIVES_TOO_FEW", "STOP_TRIALS_TOO_FEW", "STOP_TRIALS_TOO_FEW"],
    )


def test_a_fast_approach_is_observed_not_scored() -> None:
    env = envelope(points=("tool", "elbow"), tool_max_speed=0.2, stop_sources=STOP_SOURCES)
    out = run(noise_m=0.0005, envelope=env, slow={1: 0.5})
    assert {t["status"] for t in out["stops"]["trials"]} == {"not_comparable"}
    assert result(out)[0] == "INCONCLUSIVE"
    seen = [r["code"] for r in out["observations"]["failures_seen_but_not_scored"]]
    assert seen == ["STOP_DISTANCE_EXCEEDED", "STOP_TIME_EXCEEDED"]


def test_a_gap_leaves_the_trial_unmeasured() -> None:
    out = run(noise_m=0.0005, drop=lambda pid, k, t: 4.75 < t < 5.2)
    t = out["stops"]["trials"][1]
    assert t["status"] == "unmeasured"
    assert t["notes"][0].startswith("a spacing of 0.467 s between measurements")
    for key in ("rest_t_s", "settled_t_s", "stop_time_s", "stop_distance_m", "hold_displacement_m"):
        assert t[key] is None, key
    assert t["failures"] == []


def test_simulation_is_inconclusive_with_the_failures_observed() -> None:
    sim = declared(run_mode="simulation", primary_instrument_kind="simulation_ground_truth")
    out = run(slow={1: 0.5}, declared=sim)
    assert result(out) == ("INCONCLUSIVE", ["INSTRUMENT_NOT_EXTERNAL", "SIMULATION"])
    seen = [r["code"] for r in out["observations"]["failures_seen_but_not_scored"]]
    assert seen == ["STOP_DISTANCE_EXCEEDED", "STOP_TIME_EXCEEDED"]


def test_rate_and_point_checks() -> None:
    out = run(rate_hz=8.0)
    assert "INSTRUMENT_RATE_LOW" in result(out)[1]
    inp = ev02_input(plan={"point": "elbow", "sources": ["stop_message", "hardwired_stop"],
                           "drives": dict(EV02_DRIVES), "position": True})
    out = score("ev02-stop/1", inp)
    assert result(out)[1][0] == "POINT_UNMEASURED"
    assert out["stops"]["point"] == "elbow" and out["stops"]["samples"] == 0


def test_a_hold_cut_short_is_not_adequate() -> None:
    inp = ev02_input(noise_m=0.0005)
    last_stop = [c.t_sent_s for c in inp.commands if c.strategy.startswith("stop:")][-1]
    out = score("ev02-stop/1", replace(inp, samples=[s for s in inp.samples if s.t_s < last_stop + 1.0]))
    t = out["stops"]["trials"][-1]
    assert t["status"] == "inadequate" and t["notes"] == ["the hold was not measured to its end"]
    assert result(out) == ("INCONCLUSIVE", ["STOP_TRIALS_TOO_FEW"])


def test_required_source_groups() -> None:
    profile = ev02_profile(required_source_groups=[["stop_message"], ["power_loss"]])
    out = run(noise_m=0.0005, profile=profile)
    assert result(out) == ("INCONCLUSIVE", ["STOP_SOURCE_MISSING"])
    assert "no trial of power_loss" in out["verdict"]["reasons"][0]["detail"]


def test_split_trials() -> None:
    rows = [
        Command(0, 0.0, 0.0, "return", '{"x":0}', "ok", None),
        Command(1, 0.1, 0.1, "drive:a", '{"x":1}', "ok", None),
        Command(2, 0.2, 0.2, "stop:stop_message", "{}", "triggered", None),
        Command(3, 0.3, 0.3, "continue:a", '{"x":2}', "ok", None),
        Command(4, 0.4, 0.4, "continue:a", '{"x":3}', "ok", None),
        Command(5, 0.5, 0.5, "continue:a", '{"x":3}', "ok", None),
        Command(6, 1.0, 1.0, "stop:power_loss", "{}", "not_applied", "external_returncode:1"),
        Command(7, 2.0, 2.0, "drive:b", '{"x":4}', "ok", None),
        Command(8, 2.1, 2.1, "continue:b", '{"x":4}', "ok", None),
    ]
    trials = ev02.split_trials(rows)
    assert [(t.index, t.source, t.drive, t.first_t, t.drive_t, t.stop_t, t.after_times, t.walk_end_t,
             t.next_t, t.issued) for t in trials] == [
        (0, "stop_message", "a", 0.0, 0.1, 0.2, (0.2, 0.3, 0.4, 0.5), 0.4, 1.0, True),
        (1, "power_loss", None, 1.0, None, 1.0, (1.0,), 1.0, 2.0, False),
    ]  # rows after the last stop row that do not continue it belong to no trial


@pytest.mark.parametrize(
    "plan",
    [
        None,
        {"point": "tool"},
        {"point": "hand", "sources": ["stop_message"]},
        {"point": "tool", "sources": ["stop_message", "stop_message"]},
        {"point": "tool", "sources": ["heartbeat_loss"]},
        {"point": "tool", "sources": "stop_message"},
        {"point": "tool", "sources": ["stop_message"]},
        {"point": "tool", "sources": ["stop_message"], "drives": {"a": 1}, "position": True},
        {"point": "tool", "sources": ["stop_message"], "drives": {}, "position": "yes"},
    ],
)
def test_the_plan_is_part_of_the_contract(plan) -> None:
    with pytest.raises(ScoringError) as exc:
        score("ev02-stop/1", replace(ev02_input(), plan=plan))
    assert [f.code for f in exc.value.findings] == ["SCORING_INPUT_INVALID"]


def test_stop_rows_must_name_a_stop_source() -> None:
    inp = ev02_input()
    rows = list(inp.commands)
    rows[15] = rows[15]._replace(strategy="stop:gate_loss")
    with pytest.raises(ScoringError):
        score("ev02-stop/1", replace(inp, commands=rows))
