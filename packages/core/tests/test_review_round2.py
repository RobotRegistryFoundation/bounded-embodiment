"""The round-2 reviews' reproductions (method and core, review 5623073732), kept as regression
tests. Each builds its run with core's own generators and the golden runs' command space, plan
and parameters, and scores it with core's own scorers."""

from __future__ import annotations

import pytest
from _report_helpers import golden, in_repo

import _scoring_synth as T
from bounded_embodiment import report as rep
from bounded_embodiment.commandspace import load_command_space
from bounded_embodiment.envelope import load
from bounded_embodiment.generator import encode_payload
from bounded_embodiment.rawdata import Command
from bounded_embodiment.scoring import ScoringInput, ev02

pytestmark = in_repo


def _dead_stop_run(monkeypatch, drives, step_cap):
    """The golden synthetic machine on core's be-stops/1 stream, with a stop that does nothing:
    the tool follows every command exactly, the commands after the stop included."""
    G = golden()
    space = load_command_space(G.COMMAND_SPACE)
    monkeypatch.setattr(G, "STOP_DRIVES", drives)
    params = dict(G.STOP_PARAMS, step_cap=step_cap)
    plan = rep.load_stop_plan(G.stop_plan_doc(G.EV02_TRIALS))
    segs = rep.split_segments(rep.stop_stream(space, plan, params))  # accepted by core
    times = G._segment_times(segs, [1.0] * (len(segs) - 1))
    rows, pts = [], []
    for seg, ts in zip(segs, times):
        if pts:  # between trials the tool stays where the last command left it
            pts.append((ts[0] - 0.001, pts[-1][1]))
        for p, t in zip(seg, ts):
            stop = p.strategy.startswith("stop:")
            rows.append(Command(p.seq, t, t + 0.003, p.strategy, "{}" if stop else encode_payload(p.payload),
                                "triggered" if stop else "ok", None))
            if not stop:
                pts.append((t, (p.payload["x"], p.payload["y"], p.payload["z"])))
    script = G.Script(G.CENTRE)
    script.add(pts[0][0], G._follow([t for t, _ in pts], [x for _, x in pts]))
    samples = T.sample_rows(script, -0.5, rows[-1].t_sent_s + 1.0, 100.0, ("tool",), 0.0003)
    return ev02.score(ScoringInput(envelope=load(G.stop_envelope_doc()), samples=samples, commands=rows,
                                   accuracy=T.accuracy_rows(), declared=T.declared(), profile=T.ev02_profile(),
                                   plan=plan.for_scoring(space)))


def test_a_dead_stop_cannot_pass_on_an_axis_that_creeps_on_after_the_stop(monkeypatch):
    """Finding 1 (blocking): x, which carries the tool, lands one command after the stop, while z
    creeps on 20 um a command for the rest of the commands. A rule that ends the walk at the last
    change of any axis credited 3.0 s at the approach speed (0.75 m) for 5.8 mm of real travel, and
    the dead stop PASSed. The walk now ends where an axis that moved at the stop stops moving."""
    out = _dead_stop_run(monkeypatch, {"x_plus": (0.375, 0.0, 0.179), "x_minus": (0.225, 0.0, 0.179)},
                         {"x": 0.005, "y": 0.005, "z": 0.00002})
    assert out["verdict"]["result"] != "PASS"
    for t in out["stops"]["trials"]:
        assert t["status"] == "inadequate"
        assert t["walk_end_s"] - t["stop_s"] == pytest.approx(0.02)  # x's last step, one command on


def test_the_walk_rule_still_lets_a_long_walk_count(monkeypatch):
    """The golden drives walk on long after the stop on the axis that carries the tool: the dead
    stop is seen (it FAILs) and its trials count."""
    G = golden()
    out = _dead_stop_run(monkeypatch, dict(G.STOP_DRIVES), dict(G.STOP_PARAMS["step_cap"]))
    assert out["verdict"]["result"] == "FAIL"
    assert all(t["walk_end_s"] - t["stop_s"] > 0.3 for t in out["stops"]["trials"])


@pytest.mark.parametrize("shift_s", [None, 0.1, 0.25])
def test_a_spare_trial_cannot_hide_a_stop_row_recorded_late(tmp_path, shift_s):
    """Finding 3: with one spare hardwired trial, ev02-fail's late, far stop recorded 0.1 s late
    read 0.245 s and 41 mm from the moved instant, its trial was only "inadequate", the spare
    trial made up the count, and the run PASSed (consistent). A late stop row now leaves its trial
    unmeasured, so the run is INCONCLUSIVE, never PASS."""
    G = golden()
    trials = G.EV02_TRIALS + [("hardwired_stop", "x_plus")]
    run = tmp_path / "run"
    run.mkdir()
    files = G.ev02_files(trials, slow={4: 0.4})  # trial 4 brakes over 0.4 s: late and far
    if shift_s:  # ... and its stop row (and the commands after it) are recorded late
        lines = files["raw/commands.csv"].decode().split("\n")
        files["raw/commands.csv"] = "\n".join(G.move_stop(lines, 4, shift_s)).encode()
    G.write_files(run, files)
    G.finish(run, G.declared(test="EV-02", generator=G.ev02_generator(), method_version="0.2",
                             profile="ev02-illustrative-0.1"))
    res = rep.verify(run, rep.load_keyring(G.OUT / "keyring.json"))
    assert res.consistent
    codes = [r["code"] for r in res.verdict["reasons"]]
    if shift_s is None:
        assert res.verdict["result"] == "FAIL" and codes == ["STOP_DISTANCE_EXCEEDED", "STOP_TIME_EXCEEDED"]
    else:
        assert res.verdict["result"] == "INCONCLUSIVE" and "STOP_TRIAL_UNMEASURED" in codes
