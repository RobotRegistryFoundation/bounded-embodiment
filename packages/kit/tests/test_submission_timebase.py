# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""Instrument clock to run timebase: the monotonic and wall methods, their offsets and
uncertainties, the run's own times, and the exact conversion of samples."""
from __future__ import annotations

from decimal import Decimal

import pytest
from bounded_embodiment_kit.runner.session import RunClock, make_clock
from bounded_embodiment_kit.submission import timebase as tb

W0 = 1791651600.0  # 2026-10-10T17:00:00Z


def _times(**kw) -> tb.RunTimes:
    base = {"started_at": tb.utc_text(W0), "ended_at": tb.utc_text(W0 + 60), "wall_start_s": W0,
            "wall_start_uncertainty_s": 2e-7, "wall_end_s": W0 + 60,
            "monotonic_start_s": 8000.123456789, "monotonic_end_s": 8060.123456789,
            "monotonic_clock": "clock_gettime(CLOCK_MONOTONIC)", "monotonic_resolution_s": 1e-9}
    return tb.RunTimes(**{**base, **kw})


def _pairs(c: float, n=20, start=-20.0, jitter=()) -> list[tb.ClockPair]:
    """clock.csv pairs every 5 s whose wall minus monotonic is c (plus jitter[k])."""
    out = []
    for k in range(n):
        wall = W0 + start + 5 * k
        j = jitter[k % len(jitter)] if jitter else 0.0
        out.append(tb.ClockPair("periodic", round(wall + j, 6), round(wall - c, 6), 2e-6))
    return out


# ----- the runner's clock --------------------------------------------------------------------------
def test_the_runners_clock_records_where_its_zero_sits():
    clock = make_clock()
    assert isinstance(clock, RunClock) and 0 <= clock() < 1.0
    start = clock.as_dict()
    assert start["wall_start_uncertainty_s"] < 0.01 and start["monotonic_clock"]
    end = clock.end_pair()
    assert end["monotonic_end_s"] >= start["monotonic_start_s"]
    # One host, one moment: wall minus monotonic agrees at start and end (no step in between).
    drift = (end["wall_end_s"] - end["monotonic_end_s"]) - (start["wall_start_s"] - start["monotonic_start_s"])
    assert abs(drift) < 0.01


def test_run_times_from_this_runner_and_from_an_older_one():
    new = tb.run_times({"started_at": "2026-10-10T17:00:00+00:00", "ended_at": "2026-10-10T17:01:00+00:00",
                        "wall_start_s": W0 + 0.25, "wall_start_uncertainty_s": 1e-7,
                        "wall_end_s": W0 + 60.5, "monotonic_start_s": 5.0})
    assert (new.started_at, new.ended_at) == ("2026-10-10T17:00:00.250000Z", "2026-10-10T17:01:00.500000Z")
    old = tb.run_times({"started_at": "2026-10-10T17:00:00+00:00", "ended_at": "2026-10-10T17:01:00+00:00"})
    assert old.started_at == "2026-10-10T17:00:00Z" and old.ended_at == "2026-10-10T17:01:00Z"
    assert (old.wall_start_s, old.wall_start_uncertainty_s) == (W0 + 0.5, 0.5)
    assert old.monotonic_start_s is None
    with pytest.raises(tb.TimebaseError, match="no timestamps.ended_at"):
        tb.run_times({"started_at": "2026-10-10T17:00:00+00:00", "ended_at": None})
    with pytest.raises(tb.TimebaseError, match="not a UTC time"):
        tb.run_times({"started_at": "10/10/2026", "ended_at": "2026-10-10T17:01:00+00:00"})


# ----- monotonic -----------------------------------------------------------------------------------
def test_monotonic_offset_is_minus_the_runners_monotonic_start_to_the_microsecond():
    clock = tb.monotonic_clock(_times())
    assert clock.offset == Decimal("-8000.123457")
    assert clock.uncertainty == Decimal("0.000001")
    assert clock.record["uncertainty_terms_s"] == {"offset_rounding": 2.11e-07, "monotonic_resolution": 1e-09}
    assert clock.block()["offset_s"] == -8000.123457 and "not cross-checked" in clock.description
    assert clock.evidence is None


def test_monotonic_is_cross_checked_against_the_recorders_clock_csv():
    times = _times()
    same_host = W0 - times.monotonic_start_s + 0.0004  # NTP moved the wall clock 0.4 ms
    clock = tb.monotonic_clock(times, _pairs(same_host), b"clock.csv bytes")
    assert "agrees with the runner's to 0.400 ms" in clock.description
    assert clock.evidence == b"clock.csv bytes" and clock.uncertainty == Decimal("0.000001")
    other_host = W0 - 91234.5  # another machine, booted at another time
    with pytest.raises(tb.TimebaseError, match="not one clock"):
        tb.monotonic_clock(times, _pairs(other_host))
    with pytest.raises(tb.TimebaseError, match="use --clock wall"):
        tb.monotonic_clock(_times(monotonic_start_s=None))


# ----- wall ----------------------------------------------------------------------------------------
def test_wall_offset_and_every_term_of_its_uncertainty():
    """The recorder's monotonic clock reads 1234.5 s at run start, so c = W0 - 1234.5."""
    c = W0 - 1234.5
    pairs = _pairs(c, jitter=(0.0, 4e-5, -4e-5))
    clock = tb.wall_clock(_times(), pairs, 0.002, b"csv")
    assert clock.offset == Decimal("-1234.500000")
    terms = clock.record["uncertainty_terms_s"]
    assert terms == {"host_wall_clock_agreement": 0.002,
                     "recorder_wall_minus_monotonic_half_range": 4e-05,
                     "recorder_pair_reading": 2e-06, "clock_csv_rounding": 1e-06,
                     "runner_start_reading": 2e-07, "runner_wall_drift": 0.0,
                     "offset_rounding": 0.0}
    assert clock.uncertainty == Decimal("0.002044")  # the sum, rounded up to the microsecond
    within = [p for p in pairs if W0 - 10 <= p.wall_s <= W0 + 70]  # within 10 s of the run
    assert clock.record["recorder_clock"]["pairs_used"] == len(within) == 16
    assert clock.evidence == b"csv" and clock.description.startswith("wall: ")


def test_wall_uncertainty_grows_with_an_old_runners_whole_second_start_and_with_drift():
    c = W0 - 1234.5
    old = _times(started_at="2026-10-10T17:00:00Z", wall_start_s=W0 + 0.5, wall_start_uncertainty_s=0.5,
                 monotonic_start_s=None, monotonic_end_s=None)
    clock = tb.wall_clock(old, _pairs(c), 0.0)
    assert clock.offset == Decimal("-1235.000000") and clock.uncertainty == Decimal("0.500003")
    assert "drift during the run was not recorded" in clock.description
    drifting = _times(wall_end_s=W0 + 60.003)  # its wall clock gained 3 ms on its monotonic one
    assert tb.wall_clock(drifting, _pairs(c), 0.0).uncertainty == Decimal("0.003004")


def test_wall_refusals():
    with pytest.raises(tb.TimebaseError, match="no pair within 10 s of the run"):
        tb.wall_clock(_times(), _pairs(W0, start=-3600.0, n=3), 0.0)
    for bad in (-0.001, float("nan"), float("inf")):
        with pytest.raises(tb.TimebaseError, match="agreement"):
            tb.wall_clock(_times(), _pairs(W0), bad)


def test_the_run_method_changes_nothing():
    clock = tb.run_clock(by_runner=True)
    assert (clock.offset, clock.uncertainty) == (0, 0)
    assert clock.block() == {"offset_s": 0.0, "uncertainty_s": 0.0, "method": clock.description}
    assert "simulated instrument" in clock.description
    assert "the lab declares" in tb.run_clock(by_runner=False).description


# ----- clock.csv and samples -----------------------------------------------------------------------
def test_clock_csv_is_read_strictly():
    good = b"kind,t_wall_s,t_mono_s,uncertainty_s\nstart,1791651600.000001,1234.500000,0.000002000\n"
    assert tb.read_clock_csv(good) == [tb.ClockPair("start", 1791651600.000001, 1234.5, 2e-6)]
    for bad in (b"", b"kind,wall,mono,u\n", b"kind,t_wall_s,t_mono_s,uncertainty_s\n",
                b"kind,t_wall_s,t_mono_s,uncertainty_s\nstart,nan,1,0\n",
                b"kind,t_wall_s,t_mono_s,uncertainty_s\nstart,1,2\n",
                b"kind,t_wall_s,t_mono_s,uncertainty_s\nstart,1,2,-1\n"):
        with pytest.raises(tb.TimebaseError):
            tb.read_clock_csv(bad)


def test_samples_shift_exactly_and_only_in_t_s():
    data = (b"t_s,point,x_m,y_m,z_m,valid,residual_m\n"
            b"1234.500000,tool,0.123456789012,-0.5,0.25,1,0.0004\n"
            b"1234.533333,elbow,,,,0,\n1234.5,tool,1e-05,0,0,1,\n")
    out = tb.convert_samples([("samples.csv", data)], Decimal("-1234.500000"))
    assert out.data.decode().splitlines() == [
        "t_s,point,x_m,y_m,z_m,valid,residual_m",
        "0.000000,tool,0.123456789012,-0.5,0.25,1,0.0004",
        "0.033333,elbow,,,,0,",
        "0.000000,tool,1e-05,0,0,1,"]
    assert out.rows == 3 and out.valid_times == [Decimal("0.000000"), Decimal("0.000000")]
    assert out.sources[0]["first_t_s"] == "1234.500000" and out.sources[0]["sha256"].startswith("sha256:")


def test_samples_from_gzip_and_several_files_and_bad_ones():
    import gzip

    head = b"t_s,point,x_m,y_m,z_m,valid,residual_m\n"
    a, b = head + b"10.0,tool,0,0,0,1,\n", head + b"11.0,tool,0,0,0,1,\n"
    out = tb.convert_samples([("a.csv.gz", gzip.compress(a)), ("b.csv", b)], Decimal(-10))
    assert out.data.decode().splitlines()[1:] == ["0.000000,tool,0,0,0,1,", "1.000000,tool,0,0,0,1,"]
    with pytest.raises(tb.TimebaseError, match="overlapping"):
        tb.convert_samples([("a.csv", a), ("again.csv", a)], Decimal(0))
    with pytest.raises(tb.TimebaseError, match="is not be-samples-csv/1"):
        tb.convert_samples([("bad.csv", head + b"nan,tool,0,0,0,1,\n")], Decimal(0))
