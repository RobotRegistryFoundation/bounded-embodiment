# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""Each tripwire condition at its limit, per mode. The checks are pure functions of their inputs,
so these tests need no threads and no clock."""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from _runner_fakes import envelope_path
from bounded_embodiment import envelope as envelope_mod
from bounded_embodiment_kit.runner.feeds import FeedSample, FeedSnapshot
from bounded_embodiment_kit.runner.limits import DEFAULT
from bounded_embodiment_kit.runner.watchdog import (
    StateSnapshot,
    TripwireConfig,
    fk_trip,
    inside_and_still,
    instrument_trip,
    stop_speed,
)

ENV = envelope_mod.load(envelope_path("so101"))  # tool keep-in x in [0.10, 0.36]
INSIDE = (0.25, 0.0, 0.15)
NOW = 10.0


def _cfg(mode="shakedown", fk=True, joints=None, feed=True):
    return TripwireConfig.for_mode(mode, ENV, "tool", feed=feed, fk=fk,
                                   joint_speed_limits=joints or {}, limits=DEFAULT)


def _snap(tool_xyz=INSIDE, t=NOW, arrival=NOW, moved=False, others=None, valid=True):
    s = FeedSample(t, "tool", tool_xyz if valid else None, valid, arrival)
    latest = {"tool": s, **(others or {})}
    latest_valid = {k: v for k, v in latest.items() if v.valid}
    return FeedSnapshot(latest, latest_valid, arrival, moved, 0.0)


def _beyond_x_max(margin, extra):
    return (0.36 + margin + extra, 0.0, 0.15)


@pytest.mark.parametrize("mode", ["full", "shakedown", "control"])
def test_instrument_position_trips_just_past_the_mode_margin(mode):
    margin = DEFAULT.margin(mode)
    cfg = _cfg(mode)
    assert instrument_trip(_snap(_beyond_x_max(margin, -0.001)), [], NOW, cfg, ENV) is None
    reason = instrument_trip(_snap(_beyond_x_max(margin, 0.001)), [], NOW, cfg, ENV)
    assert reason and "tool (instrument)" in reason and mode in reason


def test_margins_are_the_plan_values():
    assert (DEFAULT.margin("full"), DEFAULT.margin("shakedown"), DEFAULT.margin("control")) == (
        0.05, 0.03, 0.015)


def test_goto_start_checks_keep_outs_only():
    cfg = _cfg("goto-start")
    far_out = (0.6, 0.0, 0.15)  # 24 cm outside the keep-in: allowed while moving to the start
    assert instrument_trip(_snap(far_out), [], NOW, cfg, ENV) is None
    # keep_out_0 is x [0.28, 0.36], y [0.12, 0.20], z [0.03, 0.20]: 2 cm deep in it
    deep = (0.32, 0.16, 0.10)
    assert "outside" in instrument_trip(_snap(deep), [], NOW, cfg, ENV)
    shallow = (0.32, 0.13, 0.10)  # 1 cm inside the keep-out: within the 1.5 cm margin
    assert instrument_trip(_snap(shallow), [], NOW, cfg, ENV) is None


def test_link_points_use_their_own_zones():
    cfg = _cfg("full")
    elbow = FeedSample(NOW, "elbow", (0.0, 0.0, 0.03 - 0.051), True, NOW)  # 5.1 cm below link box
    assert "elbow (instrument)" in instrument_trip(_snap(others={"elbow": elbow}), [], NOW, cfg, ENV)


def test_undeclared_points_are_ignored():
    stray = FeedSample(NOW, "marker_7", (9.0, 9.0, 9.0), True, NOW)
    assert instrument_trip(_snap(others={"marker_7": stray}), [], NOW, _cfg(), ENV) is None


def test_tool_speed_trips_above_three_times_the_limit():
    cfg = _cfg()
    v_trip = cfg.tool_speed_trip_mps
    assert v_trip == pytest.approx(3 * 0.25)

    def history(v):
        return [(NOW - 0.2 + i * 0.02, (0.2 + v * (i * 0.02), 0.0, 0.15)) for i in range(11)]

    assert instrument_trip(_snap(), history(0.99 * v_trip), NOW, cfg, ENV) is None
    assert "tool speed" in instrument_trip(_snap(), history(1.01 * v_trip), NOW, cfg, ENV)


def test_feed_silence_trips_after_half_a_second():
    cfg = _cfg()
    assert instrument_trip(_snap(t=NOW - 0.49, arrival=NOW - 0.49), [], NOW, cfg, ENV) is None
    assert "silent" in instrument_trip(_snap(t=NOW - 0.51, arrival=NOW - 0.51), [], NOW, cfg, ENV)
    never = FeedSnapshot({}, {}, None, False, NOW - 0.51)  # nothing since the feed started
    assert "silent" in instrument_trip(never, [], NOW, cfg, ENV)


def test_tool_unseen_trips_after_one_second_or_02_without_fk():
    for fk, limit in ((True, 1.0), (False, 0.2)):
        cfg = _cfg(fk=fk)
        assert cfg.tool_unseen_s == limit
        last_valid = FeedSample(NOW - limit + 0.01, "tool", INSIDE, True, NOW - limit + 0.01)
        invalid_now = FeedSample(NOW, "tool", None, False, NOW)
        ok = FeedSnapshot({"tool": invalid_now}, {"tool": last_valid}, NOW, False, 0.0)
        assert instrument_trip(ok, [], NOW, cfg, ENV) is None
        stale = FeedSample(NOW - limit - 0.01, "tool", INSIDE, True, NOW - limit - 0.01)
        bad = FeedSnapshot({"tool": invalid_now}, {"tool": stale}, NOW, False, 0.0)
        assert "unseen" in instrument_trip(bad, [], NOW, cfg, ENV)


def test_camera_moved_trips():
    assert "camera_moved" in instrument_trip(_snap(moved=True), [], NOW, _cfg(), ENV)


def _state(t=NOW, tool=INSIDE, joints=None, error=None, fk=True):
    joints = joints or {"shoulder_pan": 0.0, "elbow_flex": 0.0}
    return StateSnapshot(t, joints, dict.fromkeys(joints, "deg"),
                         {"tool": tool} if fk else None, error)


@pytest.mark.parametrize("mode", ["full", "shakedown", "control"])
def test_fk_position_trips_just_past_the_same_margin(mode):
    margin, cfg = DEFAULT.margin(mode), _cfg(mode)
    assert fk_trip([_state(tool=_beyond_x_max(margin, -0.001))], NOW, cfg, ENV) is None
    assert "tool (FK)" in fk_trip([_state(tool=_beyond_x_max(margin, 0.001))], NOW, cfg, ENV)


def test_fk_trips_on_stale_failed_or_missing_state():
    cfg = _cfg()
    assert fk_trip([_state(t=NOW - 0.24)], NOW, cfg, ENV) is None
    assert "old" in fk_trip([_state(t=NOW - 0.26)], NOW, cfg, ENV)
    assert "unusable" in fk_trip([_state(error="read_state raised OSError")], NOW, cfg, ENV)
    assert "FK unavailable" in fk_trip([_state(fk=False)], NOW, cfg, ENV)
    assert "no robot state" in fk_trip([], NOW, cfg, ENV)


def test_joint_speed_trips_above_the_adapters_limit():
    cfg = _cfg(joints={"elbow_flex": 150.0})

    def states(rate):
        return [_state(t=NOW - 0.2 + i * 0.02, joints={"elbow_flex": rate * i * 0.02}) for i in range(11)]

    assert fk_trip(states(149.0), NOW, cfg, ENV) is None
    assert "elbow_flex moved at 151" in fk_trip(states(151.0), NOW, cfg, ENV)


def test_inside_and_still():
    def hist(spread=0.0, x=0.25, seconds=2.2, fps=30):
        n = int(seconds * fps)
        return {"tool": [(NOW - seconds + i / fps, (x + (spread if i % 2 else 0.0), 0.0, 0.15))
                         for i in range(n + 1)]}

    ok, why = inside_and_still(hist(), ENV, "tool", NOW, 2.0, 0.005, 0.5)
    assert ok, why
    assert not inside_and_still(hist(spread=0.012), ENV, "tool", NOW, 2.0, 0.005, 0.5)[0]
    assert not inside_and_still(hist(x=0.37), ENV, "tool", NOW, 2.0, 0.005, 0.5)[0]
    assert not inside_and_still(hist(seconds=1.5), ENV, "tool", NOW, 2.0, 0.005, 0.5)[0]
    assert not inside_and_still(hist(), ENV, "tool", NOW + 1.0, 2.0, 0.005, 0.5)[0]  # stale


def test_stop_speed_ignores_samples_from_before_the_stop():
    # 0.5 m/s until t = 0.5 s, then still; samples every 10 ms up to t = 0.62 s.
    series = [(t / 100, ((0.5 * t / 100) if t < 50 else 0.25, 0.0, 0.15)) for t in range(63)]
    assert stop_speed(series, 0.495, 0.1, 0.2) == pytest.approx(0.0)
    assert stop_speed(series, 0.45, 0.1, 0.2) > 0.1  # the stop came late: the samples show motion
    assert stop_speed(series, 0.55, 0.1, 0.2) is None  # not 0.1 s of samples after it yet
