# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""The recorder's writer threads: MJPEG stream + index, depth chunks + index, drops."""
from __future__ import annotations

import csv
import threading
import time

import numpy as np
import pytest

from bounded_embodiment_kit.instruments import record


def _rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def test_mjpeg_writer_indexes_every_byte(tmp_path):
    st = record.StreamStats("color", 30)
    w = record.MjpegWriter(tmp_path / "c.mjpeg", tmp_path / "c.csv", record.COLOR_COLUMNS, st, max_queue=1000)
    w.start()
    rng = np.random.default_rng(1)
    blobs = [b"\xff\xd8" + rng.integers(0, 256, int(n), dtype=np.uint8).tobytes() + b"\xff\xd9"
             for n in rng.integers(100, 5000, 40)]
    for i, b in enumerate(blobs):
        assert w.put(((i, 100.0 + i / 30), b, (None, 1.5, 100.01 + i / 30, 0)))
    w.finish()
    assert w.error is None
    rows = _rows(tmp_path / "c.csv")
    assert list(rows[0].keys()) == list(record.COLOR_COLUMNS)
    data = (tmp_path / "c.mjpeg").read_bytes()
    assert len(rows) == 40 and len(data) == sum(map(len, blobs))
    for i, r in enumerate(rows):
        off, n = int(r["byte_offset"]), int(r["byte_len"])
        assert data[off:off + n] == blobs[i]
        assert int(r["frame_idx"]) == i and int(r["device_seq"]) == i
        assert float(r["t_mono_s"]) == pytest.approx(100.0 + i / 30, abs=1e-6)
        assert r["t_mid_s"] == ""                      # None is written as an empty cell
    assert st.written == 40 and st.bytes == len(data)


def test_mjpeg_writer_encodes_bgr_frames(tmp_path):
    cv2 = pytest.importorskip("cv2")
    st = record.StreamStats("wrist", 30)
    w = record.MjpegWriter(tmp_path / "w.mjpeg", tmp_path / "w.csv", record.WRIST_COLUMNS, st, jpeg_quality=90)
    w.start()
    img = np.zeros((480, 640, 3), np.uint8)
    img[100:200, 100:300] = (0, 0, 255)
    w.put(((5.0,), img, (4.99, 0, 0)))
    w.finish()
    r = _rows(tmp_path / "w.csv")[0]
    data = (tmp_path / "w.mjpeg").read_bytes()
    dec = cv2.imdecode(np.frombuffer(data[int(r["byte_offset"]):][: int(r["byte_len"])], np.uint8), cv2.IMREAD_COLOR)
    assert dec.shape == (480, 640, 3) and dec[150, 200, 2] > 200
    assert r["raw"] == "0"


def test_index_never_points_past_the_stream_while_writing(tmp_path):
    """`check` may read a live session: every indexed frame must already be in the stream file."""
    st = record.StreamStats("color", 30)
    w = record.MjpegWriter(tmp_path / "c.mjpeg", tmp_path / "c.csv", record.COLOR_COLUMNS, st, flush_s=0.02)
    w.start()
    stop = threading.Event()
    bad = []

    def reader():
        while not stop.is_set():
            rows = record._read_rows(tmp_path / "c.csv")
            size = (tmp_path / "c.mjpeg").stat().st_size if (tmp_path / "c.mjpeg").exists() else 0
            for r in rows:
                if int(r["byte_offset"]) + int(r["byte_len"]) > size:
                    bad.append(r)
            time.sleep(0.003)

    t = threading.Thread(target=reader)
    t.start()
    for i in range(150):
        w.put(((i, float(i)), b"x" * 3000, (None, None, None, 0)))
        time.sleep(0.002)
    w.finish()
    stop.set()
    t.join()
    assert not bad


def test_full_queue_drops_and_counts_without_blocking():
    st = record.StreamStats("color", 30)
    w = record.MjpegWriter("/nonexistent/c.mjpeg", "/nonexistent/c.csv", record.COLOR_COLUMNS, st, max_queue=2)
    t0 = time.monotonic()
    results = [w.put(((i, float(i)), b"x", ())) for i in range(5)]
    assert time.monotonic() - t0 < 0.1
    assert results == [True, True, False, False, False]
    assert st.dropped == 3


def test_writer_error_is_reported_and_producers_never_block(tmp_path):
    st = record.StreamStats("color", 30)
    w = record.MjpegWriter(tmp_path / "missing" / "c.mjpeg", tmp_path / "missing" / "c.csv", record.COLOR_COLUMNS,
                           st, max_queue=4)
    w.start()
    for i in range(50):
        w.put(((i, float(i)), b"x", ()))
    w.finish(timeout=5)
    assert w.error and "FileNotFoundError" in w.error
    assert not w.is_alive()


def test_depth_chunks_round_trip(tmp_path):
    st = record.StreamStats("depth", 30)
    w = record.DepthChunkWriter(tmp_path / "depth", tmp_path / "d.csv", st, chunk_frames=4)
    w.start()
    frames = []
    for i in range(10):
        a = np.full((360, 640), 500 + i, np.uint16)
        a[:10] = 0
        frames.append(a)
        w.put(((i + 7, 50.0 + i / 30), a, (None, 1.0, 50.02 + i / 30, 0)))
    w.finish()
    assert w.error is None and w.chunks == 3
    rows = _rows(tmp_path / "d.csv")
    assert list(rows[0].keys()) == list(record.DEPTH_COLUMNS)
    assert [r["chunk"] for r in rows] == ["depth_000000.npz"] * 4 + ["depth_000001.npz"] * 4 + ["depth_000002.npz"] * 2
    assert [int(r["index_in_chunk"]) for r in rows] == [0, 1, 2, 3, 0, 1, 2, 3, 0, 1]
    assert float(rows[0]["valid_frac"]) == pytest.approx(350 / 360, abs=1e-4)
    for i, r in enumerate(rows):
        with np.load(tmp_path / "depth" / r["chunk"]) as z:
            assert z["depth"].dtype == np.uint16
            np.testing.assert_array_equal(z["depth"][int(r["index_in_chunk"])], frames[i])
            assert int(z["frame_idx"][int(r["index_in_chunk"])]) == i
            assert int(z["device_seq"][int(r["index_in_chunk"])]) == i + 7
            assert float(z["t_mono_s"][int(r["index_in_chunk"])]) == pytest.approx(50.0 + i / 30)
    assert not list((tmp_path / "depth").glob("*.part"))


def test_depth_shape_change_starts_a_new_chunk(tmp_path):
    st = record.StreamStats("depth", 30)
    w = record.DepthChunkWriter(tmp_path / "depth", tmp_path / "d.csv", st, chunk_frames=10)
    w.start()
    for i in range(3):
        w.put(((i, float(i)), np.ones((360, 640), np.uint16), (None, None, None, 0)))
    for i in range(3, 5):
        w.put(((i, float(i)), np.ones((400, 640), np.uint16), (None, None, None, 1)))
    w.finish()
    assert w.chunks == 2 and w.shapes == [[360, 640], [400, 640]]


def test_write_npz_is_a_normal_npz(tmp_path):
    a = (np.arange(360 * 640) % 1500).astype(np.uint16).reshape(360, 640)
    size = record.write_npz(tmp_path / "x.npz", 1, depth=a[None], frame_idx=np.array([3]))
    assert size == (tmp_path / "x.npz").stat().st_size < a.nbytes
    with np.load(tmp_path / "x.npz") as z:
        np.testing.assert_array_equal(z["depth"][0], a)
        assert z["frame_idx"].tolist() == [3]


def test_stream_stats_counts_device_gaps_and_resets():
    st = record.StreamStats("color", 30)
    for seq, t in ((0, 1.0), (1, 1.033), (4, 1.13), (5, 1.16), (0, 1.2), (1, 1.19)):
        st.on_frame(t, seq)
    assert st.seq_gaps == 2 and st.seq_resets == 1 and st.not_increasing == 1
