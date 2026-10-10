# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""The instrument feeds: the UDP/JSON format, its clock mapping, and the simulated instrument."""
from __future__ import annotations

import json
import socket
import time

import pytest
from bounded_embodiment import rawdata
from bounded_embodiment_kit.runner.feeds import (
    SimFeed,
    UDPFeed,
    parse_datagram,
    parse_instrument,
)
from bounded_embodiment_kit.runner.session import make_clock


def test_parse_datagram_accepts_the_documented_format():
    m = parse_datagram(b'{"t": 12.5, "point": "tool", "xyz": [0.1, -0.2, 0.3], "valid": 1}')
    assert m == {"t": 12.5, "point": "tool", "xyz": (0.1, -0.2, 0.3), "valid": True,
                 "camera_moved": False}
    m = parse_datagram(b'{"t": 1, "point": "tool", "valid": 0}')
    assert m["valid"] is False and m["xyz"] is None
    assert parse_datagram(b'{"t": 1, "camera_moved": 1}')["camera_moved"] is True


@pytest.mark.parametrize("bad", [
    b"[1, 2]", b'{"point": "tool", "xyz": [0, 0, 0], "valid": 1}', b'{"t": NaN, "point": "tool"}',
    b'{"t": 1, "point": "tool", "xyz": [0, 0], "valid": 1}', b'{"t": 1, "point": "tool", "valid": 2}',
    b'{"t": 1, "point": "", "valid": 0}', b'{"t": 1}', b'{"t": 1, "point": "tool", "xyz": [0, 0, "a"], "valid": 1}',
    b"not json",
])
def test_parse_datagram_refuses_anything_else(bad):
    with pytest.raises(ValueError):
        parse_datagram(bad)


def test_udp_feed_maps_instrument_time_with_the_smallest_delay():
    feed = UDPFeed(0, lambda: 0.0, points={"tool"})
    feed.handle(b'{"t": 100.0, "point": "tool", "xyz": [0, 0, 0], "valid": 1}', arrival=5.05)
    feed.handle(b'{"t": 100.1, "point": "tool", "xyz": [0, 0, 0], "valid": 1}', arrival=5.12)
    s = feed.snapshot().latest_valid["tool"]
    assert s.t_s == pytest.approx(100.1 + (5.12 - 100.1))  # offset = min(arrival - t) so far
    feed.handle(b'{"t": 1, "point": "marker_9", "xyz": [0, 0, 0], "valid": 1}', arrival=5.2)
    assert "marker_9" not in feed.snapshot().latest  # not an envelope point
    feed.handle(b"garbage", arrival=5.3)
    assert feed.rejected == 1 and feed.snapshot().last_arrival_s == 5.2


def test_udp_feed_over_a_real_socket():
    clock = make_clock()
    feed = UDPFeed(0, clock, host="127.0.0.1")
    feed.start()
    try:
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for i in range(5):
            tx.sendto(json.dumps({"t": time.monotonic(), "point": "tool", "xyz": [0.2, 0.0, 0.1],
                                  "valid": 1}).encode(), ("127.0.0.1", feed.bound_port))
        tx.sendto(b'{"t": 1, "camera_moved": 1}', ("127.0.0.1", feed.bound_port))
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not feed.snapshot().camera_moved:
            time.sleep(0.01)
        snap = feed.snapshot()
        assert snap.camera_moved and snap.latest_valid["tool"].xyz == (0.2, 0.0, 0.1)
        assert feed.received == 6
    finally:
        feed.close()


def test_sim_feed_adds_bias_noise_dropouts_and_lag_and_writes_samples(tmp_path):
    clock = make_clock()
    truth = {"tool": (0.25, 0.0, 0.15)}
    feed = SimFeed(lambda: truth, clock, fps=100.0, sigma_m=0.0, bias_m=0.003, dropout=0.5,
                   lag_s=0.1, seed=3, samples_csv=tmp_path / "samples.csv")
    feed.start()
    time.sleep(0.5)
    snap = feed.snapshot()
    feed.close()
    s = snap.latest_valid["tool"]
    b = 0.003 / 3 ** 0.5
    assert s.xyz == pytest.approx((0.25 + b, b, 0.15 + b))
    assert s.arrival_s - s.t_s == pytest.approx(0.1)  # visible only after the lag
    rows = rawdata.read_samples(tmp_path / "samples.csv")
    assert 30 <= len(rows) <= 60 and 0.2 < sum(1 - r.valid for r in rows) / len(rows) < 0.8


def test_parse_instrument():
    assert parse_instrument("udp:5005") == ("udp", "0.0.0.0", 5005)
    assert parse_instrument("udp:10.0.0.2:5005") == ("udp", "10.0.0.2", 5005)
    assert parse_instrument("sim")[0] == "sim" and parse_instrument("none")[0] == "none"
    with pytest.raises(ValueError):
        parse_instrument("tcp:5005")
