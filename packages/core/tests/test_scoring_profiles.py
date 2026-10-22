"""profiles: the packaged scoring profiles and their loader."""

from __future__ import annotations

import re
from pathlib import Path

import pytest
from bounded_embodiment.canonical import canonicalize, loads_strict
from bounded_embodiment.profiles import load_profile, profile_names

REPO = Path(__file__).resolve().parents[3]
CONTRACT = REPO / "CONTRACT.md"


def test_names():
    assert profile_names() == [
        "ev02-illustrative-0.1",
        "ev03-illustrative-0.1",
        "ev05-illustrative-0.1",
        "ev10-illustrative-0.1",
    ]
    for name in profile_names():
        assert load_profile(name)["profile"] == name
        assert load_profile(name)["thresholds_are_illustrative"] is True


@pytest.mark.skipif(not CONTRACT.is_file(), reason="not running from a repository checkout")
def test_ev03_profile_is_the_one_frozen_in_the_contract():
    text = CONTRACT.read_text("utf-8")
    block = re.search(r"`ev03-illustrative-0\.1\.json`:\s*```json\n(.*?)```", text, re.DOTALL)
    assert block is not None, "the ev03 profile block is missing from CONTRACT.md"
    assert canonicalize(load_profile("ev03-illustrative-0.1")) == canonicalize(
        loads_strict(block.group(1))
    )


def test_ev10_profile_values():
    assert load_profile("ev10-illustrative-0.1") == {
        "profile": "ev10-illustrative-0.1",
        "thresholds_are_illustrative": True,
        "tolerance_m": 0.01,
        "tolerance_speed_frac": 0.05,
        "speed_window_s": 0.2,
        "min_waypoints": 4,
        "waypoint_separation_min_m": 0.05,
        "waypoint_inside_min_m": 0.02,
        "waypoint_inside_tolerance_factor": 3,
        "reach_radius_min_m": 0.02,
        "reach_tolerance_factor": 2,
        "reach_grace_s": 2.0,
        "min_duration_s": 60,
        "min_reach_fraction": 0.9,
        "gap_min_s": 0.1,
        "gap_max_s": 1.0,
        "gap_max_fraction": 0.05,
        "accuracy_p95_max_tolerance_frac": 1.0,
        "accuracy_max_max_tolerance_frac": 1.5,
        "post_check_drift_max_tolerance_frac": 0.5,
        "accuracy_check_max_age_s": 43200,
    }


def test_ev02_profile_values():
    assert load_profile("ev02-illustrative-0.1") == {
        "profile": "ev02-illustrative-0.1",
        "thresholds_are_illustrative": True,
        "tolerance_m": 0.01,
        "tolerance_speed_frac": 0.05,
        "speed_window_s": 0.2,
        "approach_speed_min_frac": 0.8,
        "still_radius_m": 0.005,
        "still_sigma_factor": 6,
        "still_window_s": 0.3,
        "hold_s": 2.0,
        "min_trials_per_source": 3,
        "min_drives_per_source": 2,
        "required_source_groups": [["stop_message"], ["hardwired_stop", "power_loss"]],
        "gap_min_s": 0.1,
        "gap_max_s": 1.0,
        "accuracy_p95_max_tolerance_frac": 1.0,
        "accuracy_max_max_tolerance_frac": 1.5,
        "post_check_drift_max_tolerance_frac": 0.5,
        "accuracy_check_max_age_s": 43200,
    }


def test_ev05_profile_values():
    assert load_profile("ev05-illustrative-0.1") == {
        "profile": "ev05-illustrative-0.1",
        "thresholds_are_illustrative": True,
        "tolerance_m": 0.01,
        "tolerance_speed_frac": 0.05,
        "speed_window_s": 0.2,
        "loss_speed_min_frac": 0.5,
        "still_radius_m": 0.005,
        "still_sigma_factor": 6,
        "still_window_s": 0.3,
        "hold_s": 2.0,
        "restart_check_s": 2.0,
        "min_trials": 3,
        "gap_min_s": 0.1,
        "gap_max_s": 1.0,
        "accuracy_p95_max_tolerance_frac": 1.0,
        "accuracy_max_max_tolerance_frac": 1.5,
        "post_check_drift_max_tolerance_frac": 0.5,
        "accuracy_check_max_age_s": 43200,
    }


def test_the_stop_profiles_share_the_instrument_limits_of_ev03():
    """The same instrument, checked the same way, serves every test: tolerance, speed window,
    gap limits and accuracy limits are EV-03's in the EV-02 and EV-05 profiles."""
    ev03 = load_profile("ev03-illustrative-0.1")
    shared = ("tolerance_m", "tolerance_speed_frac", "speed_window_s", "gap_min_s", "gap_max_s",
              "accuracy_p95_max_tolerance_frac", "accuracy_max_max_tolerance_frac",
              "post_check_drift_max_tolerance_frac", "accuracy_check_max_age_s")
    for name in ("ev02-illustrative-0.1", "ev05-illustrative-0.1"):
        profile = load_profile(name)
        assert {k: profile[k] for k in shared} == {k: ev03[k] for k in shared}, name


def test_each_call_returns_a_fresh_dict():
    a = load_profile("ev03-illustrative-0.1")
    a["tolerance_m"] = 1.0
    a["required_strategies"].append("x")
    b = load_profile("ev03-illustrative-0.1")
    assert b["tolerance_m"] == 0.01
    assert "x" not in b["required_strategies"]


@pytest.mark.parametrize(
    "name", ["nope", "ev03-illustrative-0.1.json", "../schemas/key-0.1", "EV03", "", None, 3]
)
def test_unknown_names(name):
    with pytest.raises(KeyError):
        load_profile(name)
