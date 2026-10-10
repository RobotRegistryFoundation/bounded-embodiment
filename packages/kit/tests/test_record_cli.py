# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""`be record` on the command line: registration, parsing, standalone use, small helpers."""
from __future__ import annotations

import argparse
import os
import sys

import pytest

from bounded_embodiment_kit.instruments import record


def _be():
    """A parser shaped like core's `be`: subcommands added by register()."""
    p = argparse.ArgumentParser(prog="be")
    sub = p.add_subparsers(dest="command")
    return p, sub


def test_kit_cli_registers_record():
    from bounded_embodiment_kit import cli
    p, sub = _be()
    cli.register(sub)
    a = p.parse_args(["record", "--out", "/tmp/s", "--seconds", "60", "--depth-fps", "15", "--wrist", "none"])
    assert a.command == "record" and a.func is record.run and a.record_command is None
    assert (a.out, a.seconds, a.depth_fps, a.wrist) == ("/tmp/s", 60.0, 15.0, "none")


def test_subcommands_parse():
    p, sub = _be()
    record.register(sub)
    a = p.parse_args(["record", "check", "/data/s1", "--quick"])
    assert (a.record_command, a.session, a.quick) == ("check", "/data/s1", True)
    a = p.parse_args(["record", "board", "--out", "b.png", "--pdf", "b.pdf"])
    assert (a.record_command, a.board_out, a.board_pdf) == ("board", "b.png", "b.pdf")
    a = p.parse_args(["record", "mark", "board_placed", "--fifo", "/tmp/f"])
    assert (a.record_command, a.label, a.mark_fifo) == ("mark", "board_placed", "/tmp/f")
    a = p.parse_args(["record", "stop", "/data/s1"])
    assert (a.record_command, a.stop_session) == ("stop", "/data/s1")
    a = p.parse_args(["record", "--wrist-size", "1280x720"])
    assert a.wrist_size == (1280, 720)


def test_defaults_match_the_session_plan():
    p = argparse.ArgumentParser()
    record.add_arguments(p)
    a = p.parse_args([])
    assert (a.color_fps, a.depth_fps, a.mjpeg_quality, a.chunk_frames) == (30.0, 30.0, 85, 150)
    assert (a.wrist, a.wrist_size, a.wrist_fps, a.wrist_fourcc) == ("auto", (640, 480), 30.0, "MJPG")
    assert a.stereo_preset == "HIGH_DENSITY" and a.depth_compress_level == 1


def test_standalone_main_writes_the_board(tmp_path, capsys):
    pytest.importorskip("cv2")
    rc = record.main(["board", "--out", str(tmp_path / "b.png"), "--pdf", str(tmp_path / "b.pdf")])
    assert rc == 0
    assert (tmp_path / "b.png").exists() and (tmp_path / "board.json").exists()
    assert "100%" in capsys.readouterr().out


def test_module_runs_as_a_script(tmp_path):
    import subprocess
    pytest.importorskip("cv2")
    env = dict(os.environ, PYTHONPATH=os.pathsep.join(p for p in sys.path if p))
    r = subprocess.run([sys.executable, record.__file__, "board", "--out", str(tmp_path / "b.png")],
                       capture_output=True, text=True, env=env, timeout=120)
    assert r.returncode == 0, r.stderr
    assert (tmp_path / "b.png").exists()


def test_mark_without_a_recorder_is_a_clear_error(tmp_path, capsys):
    rc = record.main(["mark", "x", "--fifo", str(tmp_path / "nope")])
    assert rc == 2 and "no FIFO" in capsys.readouterr().err
    os.mkfifo(tmp_path / "f")
    rc = record.main(["mark", "x", "--fifo", str(tmp_path / "f")])
    assert rc == 2 and "not running" in capsys.readouterr().err


def test_stop_without_a_recorder_is_a_clear_error(tmp_path, capsys):
    rc = record.main(["stop", str(tmp_path)])
    assert rc == 2 and "not running" in capsys.readouterr().err


def test_estimate_and_disk_need():
    r = record.estimate_rates()
    assert r["total"] == pytest.approx(r["color"] + r["depth"] + r["wrist"], abs=1e-3)
    assert 9.0 < r["total"] < 14.0                               # MB/s at 30/30/30 fps
    half = record.estimate_rates(depth_fps=15)
    assert half["depth"] == pytest.approx(r["depth"] / 2, rel=1e-3)
    assert record.disk_need(r, 1200) == pytest.approx(1200 * r["total"] * 1e6 * 1.5)


def test_labels_are_cleaned():
    assert record.clean_label("  board placed \n") == "board_placed"
    assert record.clean_label("") == "mark"
    assert record.clean_label("hold#1 (start)") == "hold1_start"


def test_version_parsing():
    assert record._version_tuple("2.33.0.0")[:2] == (2, 33)
    assert record._version_tuple("3.2.1")[:2] == (3, 2)
    assert record._version_tuple("2.24.0.0.dev0+abc")[:2] == (2, 24)


def test_core_be_finds_record_through_the_entry_point(tmp_path, monkeypatch):
    """Core's `be` loads the kit through entry points; with this one in place, `be record board` runs."""
    pytest.importorskip("cv2")
    core_cli = pytest.importorskip("bounded_embodiment.cli")
    from importlib import metadata
    ep = metadata.EntryPoint(name="kit", value="bounded_embodiment_kit.cli:register", group="bounded_embodiment.cli")
    real = metadata.entry_points

    def fake_entry_points(**kw):
        return [ep] if kw.get("group") == "bounded_embodiment.cli" else real(**kw)

    monkeypatch.setattr(metadata, "entry_points", fake_entry_points)
    rc = core_cli.main(["record", "board", "--out", str(tmp_path / "b.png")])
    assert rc == 0 and (tmp_path / "b.png").exists()
