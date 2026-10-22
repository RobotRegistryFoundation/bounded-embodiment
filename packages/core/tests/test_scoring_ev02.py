"""ev02-stop/1: trials, stop time, distance and hold per source, and the EV-02 verdict order."""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from _scoring_synth import (
    STOP_SOURCES,
    declared,
    envelope,
    ev02_input,
    ev02_profile,
)
from bounded_embodiment.rawdata import Command
from bounded_embodiment.scoring import ScoringError, ev02, score


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
    assert [s["source"] for s in stops["sources"]] == ["stop_message", "hardwired_stop"]
    by = {s["source"]: s for s in stops["sources"]}
    assert by["hardwired_stop"]["limits_from"] == "stop.sources"
    assert (by["hardwired_stop"]["max_time_s"], by["hardwired_stop"]["max_distance_m"]) == (0.25, 0.04)
    assert by["stop_message"]["drives_adequate"] == ["x_minus", "x_plus"]
    assert by["stop_message"]["trials"] == by["stop_message"]["trials_adequate"] == 3
    for t in stops["trials"]:
        assert t["status"] == "adequate" and t["failures"] == [] and t["notes"] == []
        assert t["reference_t_s"] <= t["stop_s"] - 0.005
        assert t["rest_t_s"] < t["settled_t_s"] <= t["rest_t_s"] + stops["still_window_s"]
        assert t["hold_until_s"] == t["settled_t_s"] + stops["hold_s"]
        assert t["next_command_s"] is None or t["hold_until_s"] < t["next_command_s"]
        assert 0.0 < t["stop_time_s"] < t["max_time_s"]
    assert out["observations"] == {"failures_seen_but_not_scored": []}
    assert out["verdict"]["points_measured"] == ["tool"]
    assert out["verdict"]["points_unmeasured"] == ["elbow"]


def test_limits_fall_back_to_the_top_level_stop() -> None:
    out = run(envelope=envelope(points=("tool", "elbow")))
    for s in out["stops"]["sources"]:
        assert (s["limits_from"], s["max_time_s"], s["max_distance_m"]) == ("stop", 0.3, 0.05)


def test_each_source_is_judged_by_its_own_limits() -> None:
    """The same 0.35 s braking passes as a stop message (300 ms, 0.05 m) and fails as a hardwired
    stop (250 ms, 0.04 m)."""
    out = run(noise_m=0.0005, slow={0: 0.35, 3: 0.35})
    trials = out["stops"]["trials"]
    assert trials[0]["failures"] == []
    assert trials[3]["failures"] == ["STOP_TIME_EXCEEDED", "STOP_DISTANCE_EXCEEDED"]
    assert result(out) == ("FAIL", ["STOP_DISTANCE_EXCEEDED", "STOP_TIME_EXCEEDED"])
    reasons = out["verdict"]["reasons"]
    assert "trial 3 (hardwired_stop, drive x_minus)" in reasons[0]["detail"]
    assert "0.0400 m plus the 0.0100 m tolerance" in reasons[0]["detail"]


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
    inp = ev02_input(plan={"point": "elbow", "sources": ["stop_message", "hardwired_stop"]})
    out = score("ev02-stop/1", inp)
    assert result(out)[1][0] == "POINT_UNMEASURED"
    assert out["stops"]["point"] == "elbow" and out["stops"]["samples"] == 0


def test_a_hold_cut_short_is_not_adequate() -> None:
    inp = ev02_input(noise_m=0.0005)
    last_stop = inp.commands[-1].t_sent_s
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
        Command(0, 0.0, 0.0, "return", "{}", "ok", None),
        Command(1, 0.1, 0.1, "drive:a", "{}", "ok", None),
        Command(2, 0.2, 0.2, "stop:stop_message", "{}", "triggered", None),
        Command(3, 1.0, 1.0, "stop:power_loss", "{}", "triggered", None),
        Command(4, 2.0, 2.0, "drive:b", "{}", "ok", None),
    ]
    trials = ev02.split_trials(rows)
    assert [(t.index, t.source, t.drive, t.first_t, t.stop_t, t.next_t) for t in trials] == [
        (0, "stop_message", "a", 0.0, 0.2, 1.0),
        (1, "power_loss", None, 1.0, 1.0, 2.0),
    ]


@pytest.mark.parametrize(
    "plan",
    [
        None,
        {"point": "tool"},
        {"point": "hand", "sources": ["stop_message"]},
        {"point": "tool", "sources": ["stop_message", "stop_message"]},
        {"point": "tool", "sources": ["heartbeat_loss"]},
        {"point": "tool", "sources": "stop_message"},
    ],
)
def test_the_plan_is_part_of_the_contract(plan) -> None:
    with pytest.raises(ScoringError) as exc:
        score("ev02-stop/1", ev02_input(plan=plan))
    assert [f.code for f in exc.value.findings] == ["SCORING_INPUT_INVALID"]


def test_stop_rows_must_name_a_stop_source() -> None:
    inp = ev02_input()
    rows = list(inp.commands)
    rows[15] = rows[15]._replace(strategy="stop:gate_loss")
    with pytest.raises(ScoringError):
        score("ev02-stop/1", replace(inp, commands=rows))
