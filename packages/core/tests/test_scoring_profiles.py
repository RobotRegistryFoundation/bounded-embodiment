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
    assert profile_names() == ["ev03-illustrative-0.1", "ev10-illustrative-0.1"]
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
