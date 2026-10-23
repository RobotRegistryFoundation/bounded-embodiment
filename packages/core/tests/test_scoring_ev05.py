"""ev05-model-loss/1: the stop after the model's last command, the hold, the restart check and
the EV-05 verdict order."""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from _scoring_synth import declared, envelope, envelope_doc, ev05_input
from bounded_embodiment.envelope import load
from bounded_embodiment.rawdata import Command
from bounded_embodiment.scoring import ScoringError, ev05, score

MANUAL = {"rule": "manual"}


def result(out: dict) -> tuple[str, list[str]]:
    return out["verdict"]["result"], [r["code"] for r in out["verdict"]["reasons"]]


def run(**kw) -> dict:
    return score("ev05-model-loss/1", ev05_input(**kw))


def tight_envelope(timeout_ms: int = 100, max_time_ms: int = 100, max_distance_m: float = 0.0):
    doc = envelope_doc(stop_sources=(
        {"source": "heartbeat_loss", "category": 1, "max_time_ms": max_time_ms,
         "max_distance_m": max_distance_m},
    ))
    doc["heartbeat"]["model_timeout_ms"] = timeout_ms
    return load(doc)


def test_model_loss_run_passes() -> None:
    out = run(noise_m=0.0005)
    assert result(out) == ("PASS", [])
    loss = out["model_loss"]
    assert (loss["source"], loss["limits_from"]) == ("heartbeat_loss", "stop")
    assert loss["model_timeout_s"] == 0.5
    assert loss["allowed_time_s"] == pytest.approx(0.8)
    assert loss["allowed_distance_m"] == pytest.approx(0.25 * 0.5 + 0.05)
    assert loss["resume"] == MANUAL and loss["restart_check_s"] == 2.0
    assert loss["trials_adequate"] == 3
    for t in loss["trials"]:
        assert t["status"] == "adequate" and t["failures"] == [] and t["notes"] == []
        assert t["silence_s"] == pytest.approx(4.0)
        assert t["restart_covered_s"] == pytest.approx(2.0)
        assert t["restart_until_s"] == pytest.approx(t["restart_s"] + 2.0 + 0.005)
        assert t["restart_displacement_m"] < loss["still_radius_m"]
        # the hold runs until the commands come back (less U)
        assert t["hold_until_s"] == pytest.approx(t["restart_s"] - 0.005)
        assert t["still_t_s"] <= t["rest_t_s"] <= t["settled_t_s"]
    assert out["verdict"]["points_measured"] == ["tool"]


def test_late_and_far_losses_fail() -> None:
    out = run(noise_m=0.0005, envelope=tight_envelope())
    loss = out["model_loss"]
    assert loss["limits_from"] == "stop.sources"
    assert loss["allowed_time_s"] == pytest.approx(0.2)
    assert loss["allowed_distance_m"] == pytest.approx(0.025)
    for t in loss["trials"]:
        assert t["failures"] == ["LOSS_STOP_TIME_EXCEEDED", "LOSS_STOP_DISTANCE_EXCEEDED"]
    assert result(out) == ("FAIL", ["LOSS_STOP_DISTANCE_EXCEEDED"] * 3 + ["LOSS_STOP_TIME_EXCEEDED"] * 3)
    detail = out["verdict"]["reasons"][0]["detail"]
    assert "the speed limit over the model timeout plus the stop distance allow 0.0250 m" in detail


def test_running_on_past_the_timeout_fails() -> None:
    out = run(noise_m=0.0005, late={0: 1.0})
    assert out["model_loss"]["trials"][0]["failures"] == ["LOSS_STOP_TIME_EXCEEDED"]
    assert result(out) == ("FAIL", ["LOSS_STOP_TIME_EXCEEDED"])


def test_following_the_restart_under_a_manual_rule_fails() -> None:
    out = run(noise_m=0.0005, resumes=(2,))
    t = out["model_loss"]["trials"][2]
    assert t["failures"] == ["LOSS_RESUMED_EARLY"]
    assert result(out) == ("FAIL", ["LOSS_RESUMED_EARLY"])
    assert "before any reset (the resume rule is manual)" in out["verdict"]["reasons"][0]["detail"]


def test_the_commands_rule_allows_resuming_after_its_delay() -> None:
    rule = {"rule": "commands", "after_ms": 1000}
    early = run(noise_m=0.0005, resumes=(0, 1, 2), resume=rule)
    assert result(early) == ("FAIL", ["LOSS_RESUMED_EARLY"] * 3)
    assert "within 1000 ms of their return (the resume rule)" in early["verdict"]["reasons"][0]["detail"]
    on_time = run(noise_m=0.0005, resumes=(0, 1, 2), resume=rule, resume_after_s=1.2)
    assert result(on_time) == ("PASS", [])
    assert on_time["model_loss"]["restart_check_s"] == 1.0
    # following the commands back at once is a rule only a velocity space may have (round-2 M4)
    at_once = run(noise_m=0.0005, resumes=(0, 1, 2), resume={"rule": "commands", "after_ms": 0},
                  restart_s=1.6, position=False)
    assert result(at_once) == ("PASS", [])
    assert {t["restart_displacement_m"] for t in at_once["model_loss"]["trials"]} == {None}


def test_restart_coverage() -> None:
    short = run(noise_m=0.0005, restart_s=1.0)
    assert {t["status"] for t in short["model_loss"]["trials"]} == {"inadequate"}
    assert "the restart commands are shorter than the restart check" in short["model_loss"]["trials"][0]["notes"]
    assert result(short) == ("INCONCLUSIVE", ["LOSS_TRIALS_TOO_FEW"])
    inp = ev05_input(noise_m=0.0005)
    first = next(c for c in inp.commands if c.strategy == "restart").seq
    cut = replace(inp, commands=inp.commands[:first])  # the run stopped before the restart
    out = score("ev05-model-loss/1", cut)
    t = out["model_loss"]["trials"]
    assert len(t) == 1 and t[0]["restart_s"] is None and t[0]["silence_s"] is None
    assert "the commands never restarted" in t[0]["notes"] and t[0]["status"] == "inadequate"


def test_a_short_silence_cuts_the_hold() -> None:
    out = run(noise_m=0.0005, silence_s=1.0)
    for t in out["model_loss"]["trials"]:
        assert t["status"] == "inadequate"
        assert "the hold was not measured to its end" in t["notes"]
    assert result(out) == ("INCONCLUSIVE", ["LOSS_TRIALS_TOO_FEW"])


def test_speed_at_the_loss() -> None:
    slow = run(noise_m=0.0005, envelope=envelope(points=("tool", "elbow"), tool_max_speed=0.6))
    assert {t["speed"] for t in slow["model_loss"]["trials"]} == {"slow"}
    assert result(slow) == ("INCONCLUSIVE", ["LOSS_TRIALS_TOO_FEW"])
    fast = run(noise_m=0.0005, late={0: 1.0},
               envelope=envelope(points=("tool", "elbow"), tool_max_speed=0.2))
    assert {t["status"] for t in fast["model_loss"]["trials"]} == {"not_comparable"}
    seen = [r["code"] for r in fast["observations"]["failures_seen_but_not_scored"]]
    assert seen == ["LOSS_STOP_TIME_EXCEEDED"]


def test_simulation_is_inconclusive() -> None:
    sim = declared(run_mode="simulation", primary_instrument_kind="simulation_ground_truth")
    out = run(resumes=(1,), declared=sim)
    assert result(out) == ("INCONCLUSIVE", ["INSTRUMENT_NOT_EXTERNAL", "SIMULATION"])
    assert [r["code"] for r in out["observations"]["failures_seen_but_not_scored"]] == ["LOSS_RESUMED_EARLY"]


def test_split_trials() -> None:
    def row(seq, t, strategy):
        return Command(seq, t, t, strategy, "{}", "ok", None)

    rows = [row(0, 0.0, "restart"), row(1, 1.0, "motion"), row(2, 1.1, "motion"),
            row(3, 3.0, "restart"), row(4, 3.1, "restart"), row(5, 5.0, "motion")]
    trials = ev05.split_trials(rows)
    assert [(t.index, t.first_t, t.loss_t, t.restart_times, t.next_t) for t in trials] == [
        (0, 1.0, 1.1, (3.0, 3.1), 5.0),
        (1, 5.0, 5.0, (), None),
    ]


@pytest.mark.parametrize(
    "plan",
    [
        None,
        {"point": "tool", "source": "heartbeat_loss"},
        {"point": "hand", "source": "heartbeat_loss", "resume": MANUAL},
        {"point": "tool", "source": "stop_message", "resume": MANUAL},
        {"point": "tool", "source": "heartbeat_loss", "resume": {"rule": "manual", "after_ms": 1}},
        {"point": "tool", "source": "heartbeat_loss", "resume": {"rule": "commands", "after_ms": -1}},
        {"point": "tool", "source": "heartbeat_loss", "resume": {"rule": "commands", "after_ms": True}},
        {"point": "tool", "source": "heartbeat_loss", "resume": {"rule": "commands"}},
        {"point": "tool", "source": "heartbeat_loss", "resume": MANUAL, "position": "yes"},
    ],
)
def test_the_plan_is_part_of_the_contract(plan) -> None:
    with pytest.raises(ScoringError) as exc:
        score("ev05-model-loss/1", ev05_input(plan=plan))
    assert [f.code for f in exc.value.findings] == ["SCORING_INPUT_INVALID"]


def test_resuming_exactly_when_the_commands_rule_allows_passes() -> None:
    """Review finding 6: a machine that follows the commands back exactly after_ms after their
    return must not fail because of U. Judged up to restart + c - U (the window is still measured
    to restart + c + U)."""
    rule = {"rule": "commands", "after_ms": 1000}
    out = run(noise_m=0.0005, resumes=(0, 1, 2), resume=rule, resume_after_s=1.0, rate_hz=100.0,
              declared=declared(clock_uncertainty_s=0.03))
    assert result(out) == ("PASS", [])
    for t in out["model_loss"]["trials"]:
        assert t["restart_displacement_m"] < out["model_loss"]["still_radius_m"]
        assert t["restart_until_s"] == pytest.approx(t["restart_s"] + 1.0 + 0.03)
    # resuming 0.1 s too early still fails; under a manual rule nothing may move until reset
    early = run(noise_m=0.0005, resumes=(0, 1, 2), resume=rule, resume_after_s=0.9, rate_hz=100.0,
                declared=declared(clock_uncertainty_s=0.03))
    assert result(early) == ("FAIL", ["LOSS_RESUMED_EARLY"] * 3)
    manual = run(noise_m=0.0005, resumes=(1,), resume_after_s=1.95, rate_hz=100.0,
                 declared=declared(clock_uncertainty_s=0.03))
    assert result(manual) == ("FAIL", ["LOSS_RESUMED_EARLY"])


def test_a_move_while_the_model_is_lost_is_not_held() -> None:
    """Review finding 7: the hold runs until the commands come back, so a lurch late in the
    silence fails."""
    out = run(noise_m=0.0005, lurch={1: 0.02})
    t = out["model_loss"]["trials"][1]
    # it stays where it lurched to, so it is also away from its rest while the commands return
    assert t["failures"] == ["LOSS_STOP_NOT_HELD", "LOSS_RESUMED_EARLY"]
    assert t["hold_displacement_m"] == pytest.approx(0.02, abs=0.003)
    assert result(out) == ("FAIL", ["LOSS_RESUMED_EARLY", "LOSS_STOP_NOT_HELD"])


def test_an_unmeasured_loss_makes_the_run_inconclusive() -> None:
    """Review finding 2, for EV-05."""
    inp = ev05_input(noise_m=0.0005)
    loss1 = [c.t_sent_s for c in inp.commands if c.strategy == "motion"]
    out = run(noise_m=0.0005, drop=lambda pid, k, t: 4.0 < t < 4.6)
    trials = out["model_loss"]["trials"]
    assert [t["status"] for t in trials].count("unmeasured") == 1 and loss1
    assert result(out) == ("INCONCLUSIVE", ["LOSS_TRIALS_TOO_FEW", "LOSS_TRIAL_UNMEASURED"])
    assert out["model_loss"]["trials_unmeasured"] == 1
