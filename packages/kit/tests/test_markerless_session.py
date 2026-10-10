"""The session reader the markerless instrument uses (instruments/session.py), on small recordings written
with the recorder's own writers."""

from __future__ import annotations

import json

import numpy as np
import pytest
from _markerless_support import write_tiny_session
from bounded_embodiment_kit.instruments import record
from bounded_embodiment_kit.instruments.session import (
    Session,
    SessionError,
    parse_measurements,
)

K1280 = [[920.0, 0.0, 641.3], [0.0, 921.0, 359.2], [0.0, 0.0, 1.0]]
COLOR = [(0, 100.000, 0), (1, 100.033, 0), (2, 100.067, 0), (3, 100.100, 0),     # oak_session 0
         (0, 101.000, 1), (1, 101.033, 1)]                                          # reconnected: seq restarts
DEPTH = [(0, 100.001, 0),          # same session and seq as colour 0
         (2, 100.068, 0),          # colour 2
         (1, 100.500, 0),          # seq 1 matches colour 1, but 0.47 s apart: nearest in time (colour 3)
         (1, 101.034, 1),          # (session 1, seq 1) = colour 5, not (session 0, seq 1) = colour 1
         (7, 101.200, 1)]          # no such seq: nearest in time (colour 5)
EVENTS = [(100.2, "board_placed"), (100.9, "board_placed"), (101.5, "background_start"),
          (103.0, "background_end"), (103.5, "nominal_1")]


@pytest.fixture
def sess(tmp_path):
    root = write_tiny_session(tmp_path / "s", color=COLOR, depth=DEPTH, events=EVENTS,
                              intrinsics={"rgb_1280x720": K1280, "rgb_distortion": [0.01] * 14},
                              measurements="corner_O_x_mm = 90  # O\nboard_face = up\nnotes =\n")
    with Session(root) as s:
        yield s


def test_not_a_recording(tmp_path):
    with pytest.raises(SessionError, match="meta.json"):
        Session(tmp_path)


def test_events_and_times(sess):
    assert sess.t0 == 100.0
    assert sess.event_time("board_placed") == 100.9                         # the last one counts
    assert sess.event_time("board_placed", first=True) == 100.2
    assert sess.next_event_after(100.9) == (101.5, "background_start")
    assert sess.resolve_time("board_placed") == 100.9
    assert sess.resolve_time("board_placed+1.5") == pytest.approx(102.4)
    assert sess.resolve_time("nominal_1-0.5") == pytest.approx(103.0)
    assert sess.resolve_time("2.5") == pytest.approx(102.5)                 # seconds since the start
    assert sess.resolve_time(2.5) == pytest.approx(102.5)
    assert sess.resolve_time(None) is None
    with pytest.raises(SessionError, match="background_start"):           # lists what there is
        sess.resolve_time("hold_1")


def test_intrinsics(sess, tmp_path):
    K, dist = sess.intrinsics("1280x720")
    assert np.allclose(K, K1280) and dist.size == 14
    K2, _ = sess.intrinsics("640x360")                                      # derived: the colour image halved
    assert np.allclose(K2[:2], np.array(K1280)[:2] / 2) and K2[2, 2] == 1.0
    odd = write_tiny_session(tmp_path / "odd", intrinsics={"rgb_640x360": K1280, "rgb_distortion": [0.1, 0.2, 0.3]})
    assert np.array_equal(Session(odd).intrinsics("640x360")[1], np.zeros(5))   # unusable length: none
    none = write_tiny_session(tmp_path / "none")
    with pytest.raises(SessionError, match="intrinsics"):
        Session(none).intrinsics("640x360")


def test_pairing_uses_session_and_sequence_then_time(sess):
    assert [sess.color_for_depth(i) for i in range(len(DEPTH))] == [0, 2, 3, 5, 5]
    assert sess.pairing_summary() == {"by_device_seq": 3, "by_nearest_time": 2, "unpaired": 0}


def test_depth_frames_and_ranges(sess):
    assert int(sess.depth_frame(3)[0, 0]) == 4                             # frame k holds k + 1
    assert sess.depth_indices().tolist() == [0, 1, 2, 3, 4]
    assert sess.depth_indices(100.05, 101.1).tolist() == [1, 2, 3]
    assert sess.depth_indices(every=2).tolist() == [0, 2, 4]
    frames = list(sess.iter_depth(101.0))
    assert [(f.index, f.device_seq, f.color_index) for f in frames] == [(3, 1, 5), (4, 7, 5)]
    assert frames[0].t_mono_s == pytest.approx(101.034)
    assert sess.color_indices(100.05, 101.01).tolist() == [2, 3, 4]


def test_measurements(sess, tmp_path):
    assert sess.measurements() == {"corner_O_x_mm": 90.0, "board_face": "up", "notes": None}
    assert Session(write_tiny_session(tmp_path / "m")).measurements() == {}
    text = record.MEASUREMENTS_TEMPLATE + "\ncorner_X_y_mm = -12.5   # right of the axis\nnote = 'a b'  # q\n"
    m = parse_measurements(text)
    assert m["corner_X_y_mm"] == -12.5 and m["note"] == "a b"
    assert all(not k.startswith("#") for k in m)


def test_board_spec_precedence(tmp_path):
    plain = Session(write_tiny_session(tmp_path / "a"))
    assert plain.board_spec()["squares_x"] == record.BOARD_SPEC["squares_x"]
    in_meta = Session(write_tiny_session(tmp_path / "b", board={**record.BOARD_SPEC, "squares_x": 7}))
    assert in_meta.board_spec()["squares_x"] == 7 and in_meta.board_spec()["source"] == "meta.json"
    root = write_tiny_session(tmp_path / "c", board={**record.BOARD_SPEC, "squares_x": 7})
    (root / "board.json").write_text(json.dumps({"squares_x": 9}), encoding="utf-8")
    spec = Session(root).board_spec()
    assert spec["squares_x"] == 9 and spec["squares_y"] == record.BOARD_SPEC["squares_y"]
