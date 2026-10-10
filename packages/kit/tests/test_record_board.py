# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""`be record board`: a ChArUco board that prints at exactly the stated size and is detectable."""
from __future__ import annotations

import json
import re

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
from bounded_embodiment_kit.instruments import record  # noqa: E402


@pytest.fixture(scope="module")
def board_files(tmp_path_factory):
    d = tmp_path_factory.mktemp("board")
    desc = record.make_board(d / "board.png", d / "board.pdf", d / "board.json")
    return d, desc


def _page(d):
    img = cv2.imread(str(d / "board.png"), cv2.IMREAD_GRAYSCALE)
    assert img is not None
    return img


def test_page_is_us_letter_at_10_px_per_mm(board_files):
    d, desc = board_files
    page = _page(d)
    assert page.shape == (2794, 2159)              # 279.4 x 215.9 mm
    assert set(np.unique(page)) <= {0, 255}        # pure black and white, nothing to blur
    assert desc["page"]["dpi"] == pytest.approx(254.0)


def test_pdf_page_is_exactly_letter(board_files):
    d, desc = board_files
    if desc["files"]["pdf"] is None:
        pytest.skip("Pillow not installed")
    data = (d / "board.pdf").read_bytes()
    box = re.search(rb"/MediaBox\s*\[\s*([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s*\]", data)
    assert box is not None
    assert [float(x) for x in box.groups()] == [0.0, 0.0, 612.0, 792.0]
    assert b"/CCITTFaxDecode" in data or b"/FlateDecode" in data   # lossless, not JPEG


def test_squares_are_exactly_30_mm_and_the_board_is_found(board_files):
    d, desc = board_files
    page = _page(d)
    board = record.charuco_board(cv2)
    small = cv2.resize(page, None, fx=0.4, fy=0.4, interpolation=cv2.INTER_AREA)
    r = record.detect_charuco(cv2, board, small)
    assert r is not None and len(r[0]) == 24
    obj, img, ids = r
    full = img / 0.4
    # neighbouring corners are one square apart: 30 mm = 300 px at 10 px/mm
    by_id = dict(zip(ids.tolist(), full, strict=True))
    dx = [np.linalg.norm(by_id[i + 1] - by_id[i]) for i in range(24) if i % 4 != 3 and i + 1 in by_id]
    dy = [np.linalg.norm(by_id[i + 4] - by_id[i]) for i in range(20) if i + 4 in by_id]
    assert np.median(dx) == pytest.approx(300.0, abs=1.5)
    assert np.median(dy) == pytest.approx(300.0, abs=1.5)
    # corner 0 is 30 mm right of and below corner O, which sits at (33, 25) mm on the page
    o_mm = desc["page"]["corner_O_on_page_mm"]
    assert by_id[0] / 10.0 == pytest.approx(np.array(o_mm) + 30.0, abs=0.3)


def test_corners_o_x_and_y_are_black_square_corners(board_files):
    d, desc = board_files
    page = _page(d)
    x0, y0 = (int(round(v * 10)) for v in desc["page"]["corner_O_on_page_mm"])
    x1, y1 = x0 + 1500, y0 + 2100
    assert page[y0 + 5, x0 + 5] == 0 and page[y0 - 5, x0 + 5] == 255 and page[y0 + 5, x0 - 5] == 255
    assert page[y0 + 5, x1 - 5] == 0 and page[y0 + 5, x1 + 5] == 255
    assert page[y1 - 5, x0 + 5] == 0 and page[y1 + 5, x0 + 5] == 255 and page[y1 - 5, x0 - 5] == 255


def test_scale_bar_is_100_mm(board_files):
    d, desc = board_files
    page = _page(d)
    x0, _ = (int(round(v * 10)) for v in desc["page"]["corner_O_on_page_mm"])
    y_bar = int(round((desc["page"]["corner_O_on_page_mm"][1] + 210 + 10 + 1.5) * 10))
    row = page[y_bar]
    black = np.flatnonzero(row == 0)
    run = black[(black >= x0 - 2) & (black <= x0 + 1100)]
    assert run.min() == x0 and run.max() - run.min() + 1 == 1000


def test_board_json_describes_the_board(board_files):
    d, desc = board_files
    j = json.loads((d / "board.json").read_text())
    assert j["dictionary"] == "DICT_5X5_100"
    assert (j["squares_x"], j["squares_y"]) == (5, 7)
    assert (j["square_length_m"], j["marker_length_m"]) == (0.03, 0.022)
    assert j["marker_ids"] == list(range(17))
    assert j["chessboard_corners"] == 24
    assert j["corner_X_m"] == [0.15, 0.0, 0.0] and j["corner_Y_m"] == [0.0, 0.21, 0.0]
    assert "100%" in j["print"]
