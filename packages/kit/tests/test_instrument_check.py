"""`be instrument check`: the pre-run accuracy check, the jitter record and the post-run check (method §5.3 to
§5.5) from an instrument's samples, the surveyed targets and the holds."""

from __future__ import annotations

import csv
import io
import json
import math
import random
from pathlib import Path

import pytest
from bounded_embodiment import cli as core_cli
from bounded_embodiment import envelope as envelope_mod
from bounded_embodiment import rawdata
from bounded_embodiment.profiles import load_profile
from bounded_embodiment.scoring import ev03 as core_ev03

from bounded_embodiment_kit.instruments import accuracy as acc

REPO = Path(__file__).resolve().parents[3]
ENVELOPE = REPO / "examples" / "envelopes" / "bob.envelope.json"
F_IN_BL = (0.0388, 0.0, -0.0024)          # bob's base_link = F + this (be fit register's T_bl_F)
WALL = 1_760_000_000.0                    # the session's wall clock minus its monotonic clock
# Eight surveyed points spread through bob's tool keep-in (x 0.21-0.36, y -0.1-0.1, z 0.26-0.42), clear
# of keep_out_0 (x 0.30-0.36, y 0.05-0.10, z 0.26-0.32), in base_link metres.
TARGETS_BL = {"T1": (0.235, -0.075, 0.285), "T2": (0.235, 0.075, 0.285), "T3": (0.335, -0.075, 0.285),
              "T4": (0.335, 0.025, 0.285), "T5": (0.235, -0.075, 0.395), "T6": (0.235, 0.075, 0.395),
              "T7": (0.335, -0.075, 0.395), "T8": (0.335, 0.075, 0.395)}
START_POSE = (0.28, 0.0, 0.33)
FPS = 15.0


def _f_mm(p_bl):
    return tuple(round((p_bl[i] - F_IN_BL[i]) * 1000, 3) for i in range(3))


def targets_text(points=TARGETS_BL, *, method="Plumb line and steel rule from the tool-point dot", frame="F"):
    lines = [f"format = {acc.TARGETS_FORMAT}", f"frame = {frame}", "units = mm" if frame == "F" else "units = m"]
    if method:
        lines.append(f"method = {method}   # how")
    for k, p in points.items():
        v = _f_mm(p) if frame == "F" else p
        lines.append(f"{k} = {v[0]}, {v[1]}, {v[2]}")
    return "\n".join(lines) + "\n"


class Scenario:
    """A synthetic session: pre holds at each target (the instrument off by ``bias``), a jitter record at the
    start pose, a run, and post holds; samples at 15 Hz on the session's monotonic clock."""

    def __init__(self, root: Path, *, bias=None, sigma=0.0004, seed=3, jitter_s=12.0, moving=None,
                 post_labels=("T1",), extra_events=(), targets=TARGETS_BL, post_shift=(0.0, 0.0, 0.0)):
        rng = random.Random(seed)
        self.root = root
        self.session = root / "session"
        (self.session / "fit").mkdir(parents=True)
        (self.session / "meta.json").write_text('{"format": "be-camera-recording/1"}\n')
        T = [[1, 0, 0, F_IN_BL[0]], [0, 1, 0, F_IN_BL[1]], [0, 0, 1, F_IN_BL[2]], [0, 0, 0, 1]]
        (self.session / "fit" / "registration.json").write_text(json.dumps({"steps": {"T_bl_F": T}}))
        self.bias = bias or {k: (0.002 + 0.0002 * i, -0.001, 0.0015) for i, k in enumerate(targets)}
        events, track = [], []                  # track: (t0, t1, f(t) -> xyz)
        t = 1010.0
        prev = START_POSE
        for k, p in targets.items():
            b = self.bias.get(k, (0.0, 0.0, 0.0))
            pos = tuple(p[i] + b[i] for i in range(3))
            track.append((t - 4.0, t, _lerp(prev, pos, t - 4.0, t)))
            events.append((t, f"pre:{k}"))
            if moving == k:   # the arm creeps 6 mm during this hold
                track.append((t, t + 5.0, _lerp(pos, (pos[0] + 0.006, pos[1], pos[2]), t, t + 5.0)))
            else:
                track.append((t, t + 5.0, _const(pos)))
            events.append((t + 5.0, "moving"))
            prev, t = pos, t + 10.0
        track.append((t - 4.0, t, _lerp(prev, START_POSE, t - 4.0, t)))
        events.append((t, "jitter"))
        track.append((t, t + jitter_s + 0.5, _const(START_POSE)))
        events.append((t + jitter_s + 0.5, "go"))
        self.run_start = t + jitter_s + 3.0
        self.run_end = self.run_start + 60.0
        track.append((t + jitter_s + 0.5, self.run_end, _wander(START_POSE, rng)))
        t = self.run_end + 5.0
        for k in post_labels:
            b = self.bias[k]
            pos = tuple(targets[k][i] + b[i] + post_shift[i] for i in range(3))
            track.append((t - 5.0, t, _lerp(START_POSE, pos, t - 5.0, t)))
            events.append((t, f"post:{k}"))
            track.append((t, t + 5.0, _const(pos)))
            events.append((t + 5.0, "done"))
            t += 10.0
        events.extend(extra_events)
        events.sort()
        self.events = events
        with open(self.session / "events.csv", "w", newline="") as fh:
            w = csv.writer(fh, lineterminator="\n")
            w.writerow(["n", "t_mono_s", "t_wall_s", "label", "source"])
            for n, (te, label) in enumerate(events):
                w.writerow([n, f"{te:.6f}", f"{te + WALL:.6f}", label, "keyboard"])
        self.samples = root / "samples.csv"
        rows = ["t_s,point,x_m,y_m,z_m,valid,residual_m"]
        tt = 1000.0
        while tt < t:
            xyz = _at(track, tt)
            if xyz is None:
                xyz = START_POSE
            n = [rng.gauss(0.0, sigma) for _ in range(3)]
            rows.append(f"{tt:.6f},tool,{xyz[0] + n[0]:.6f},{xyz[1] + n[1]:.6f},{xyz[2] + n[2]:.6f},1,0.002")
            rows.append(f"{tt:.6f},elbow,0.1,0.0,0.3,1,0.002")
            tt += 1.0 / FPS
        self.samples.write_text("\n".join(rows) + "\n")
        self.targets = root / "targets.txt"
        self.targets.write_text(targets_text(targets))
        self.run = root / "run"
        self.run.mkdir()
        (self.run / "declared.json").write_text(json.dumps({"timestamps": {
            "started_at": "x", "ended_at": "y", "monotonic_start_s": self.run_start, "monotonic_end_s": self.run_end,
            "wall_start_s": self.run_start + WALL, "wall_end_s": self.run_end + WALL}}))

    def argv(self, *extra, run=True):
        a = ["instrument", "check", "--envelope", str(ENVELOPE), "--targets", str(self.targets),
             "--session", str(self.session), "--samples", str(self.samples), "--out", str(self.root / "out")]
        return a + (["--run", str(self.run)] if run else []) + list(extra)

    def outputs(self):
        out = self.root / "out"
        return (out / "accuracy.csv").read_bytes(), json.loads((out / "accuracy-check.json").read_text())


def _const(p):
    return lambda t: p


def _lerp(a, b, t0, t1):
    return lambda t: tuple(a[i] + (b[i] - a[i]) * (t - t0) / (t1 - t0) for i in range(3))


def _wander(p, rng):
    ph = [rng.uniform(0, 6.28) for _ in range(3)]
    return lambda t: tuple(p[i] + 0.03 * math.sin(0.7 * t + ph[i]) for i in range(3))


def _at(track, t):
    for t0, t1, f in track:
        if t0 <= t < t1:
            return f(t)
    return None


def run_cli(argv, capsys=None):
    rc = core_cli.main(argv)
    text = capsys.readouterr() if capsys is not None else None
    return rc, text


# ----- the whole check ---------------------------------------------------------------------------------------
def test_a_good_check_writes_rows_core_reads_and_the_lab_config_blocks(tmp_path, capsys):
    sc = Scenario(tmp_path)
    rc, out = run_cli(sc.argv(), capsys)
    assert rc == 0, out.out + out.err
    data, summary = sc.outputs()
    rows = rawdata.read_accuracy(data)
    pre = [r for r in rows if r.phase == "pre"]
    post = [r for r in rows if r.phase == "post"]
    jit = [r for r in rows if r.phase == "jitter"]
    assert sorted(r.label for r in pre) == sorted(TARGETS_BL)
    assert [r.label for r in post] == ["T1"]
    assert jit and {r.label for r in jit} == {"start_pose"} and all(r.n == 1 and r.ref_x_m is None for r in jit)
    # References are the targets, converted from frame F (mm) to base_link (m).
    for r in pre:
        assert (r.ref_x_m, r.ref_y_m, r.ref_z_m) == pytest.approx(TARGETS_BL[r.label], abs=1e-9)
    # Each error is the instrument's bias at that point (the noise averages out over ~37 frames).
    profile = load_profile("ev03-illustrative-0.1")
    block = core_ev03.accuracy(rows, 100.0, profile)
    for e in block["pre"]["errors"]:
        b = sc.bias[e["label"]]
        assert e["error_m"] == pytest.approx(math.sqrt(sum(v * v for v in b)), abs=0.0004)
    assert summary["accuracy"]["pre"] == block["pre"]          # the numbers core will compute, by core
    assert summary["accuracy"]["jitter"]["sigma_m"] == pytest.approx(0.0004, rel=0.2)
    assert summary["accuracy"]["post"]["max_drift_m"] < 0.0005
    assert summary["problems"] == []
    assert summary["spread"]["ok"] is True
    assert summary["order"] == {"checked": True, "clock": "monotonic", "run": str(sc.run)}
    # The check's time: the end of the last pre hold (T8 at 1080 s + 0.5 s settle + 3 s), on the wall clock.
    lc = summary["lab_config"]
    assert lc["accuracy_check"]["points"] == 8
    assert lc["accuracy_check"]["method"] == "Plumb line and steel rule from the tool-point dot"
    assert lc["accuracy_check"]["at"] == acc._iso(1080.0 + 3.5 + WALL)
    assert summary["accuracy"]["age_s"] == pytest.approx(sc.run_start - 1083.5)
    assert lc["post_run_check"]["at"] == acc._iso(sc.run_end + 5.0 + 3.5 + WALL)
    assert "T1" in lc["post_run_check"]["method"]
    assert "for the lab config" in out.out


def test_the_rows_are_the_valid_frames_of_the_hold_only(tmp_path):
    sc = Scenario(tmp_path)
    env = envelope_mod.load(ENVELOPE)
    targets = acc.to_envelope_frame(acc.load_targets(sc.targets), "base_link",
                                    [[1, 0, 0, F_IN_BL[0]], [0, 1, 0, 0], [0, 0, 1, F_IN_BL[2]], [0, 0, 0, 1]])
    holds, _ = acc.holds_from_events(acc.read_events(sc.session), settle_s=0.5, hold_s=3.0, jitter_s=15.0)
    samples = rawdata.read_samples(sc.samples.read_bytes())
    # Make every T3 frame in its hold invalid but one, and one other frame invalid.
    t3 = next(h for h in holds if h.label == "T3")
    seen = 0
    for i, s in enumerate(samples):
        if s.point == "tool" and t3.t_start <= s.t_s <= t3.t_end:
            seen += 1
            if seen > 1:
                samples[i] = s._replace(x_m=None, y_m=None, z_m=None, valid=0)
    data, summary = acc.check(envelope=env, targets=targets, holds=holds, samples=samples, point="tool",
                              run=acc.load_run(sc.run))
    row = next(r for r in rawdata.read_accuracy(data) if r.phase == "pre" and r.label == "T3")
    assert row.n == 1
    assert any("T3: 1 valid samples in the hold, fewer than 5" in p for p in summary["problems"])


def test_a_hold_with_motion_is_a_problem(tmp_path, capsys):
    sc = Scenario(tmp_path, moving="T4")
    rc, out = run_cli(sc.argv(), capsys)
    assert rc == 1
    _, summary = sc.outputs()
    assert any(p.startswith("pre T4: the point moved") for p in summary["problems"])
    assert "PROBLEM: pre T4: the point moved" in out.out


def test_errors_over_the_limit_are_written_and_reported(tmp_path, capsys):
    bias = {k: (0.0, 0.0, 0.0) for k in TARGETS_BL}
    bias["T2"] = (0.016, 0.0, 0.0)
    sc = Scenario(tmp_path, bias=bias)
    rc, _ = run_cli(sc.argv(), capsys)
    assert rc == 1
    data, summary = sc.outputs()
    assert len([r for r in rawdata.read_accuracy(data) if r.phase == "pre"]) == 8
    assert [p.split(":")[0] for p in summary["problems"]] == ["ACCURACY_CHECK_FAILED"]


def test_clustered_points_fail_the_spread(tmp_path, capsys):
    near = {f"C{i}": (0.30 + 0.005 * i, 0.0, 0.33) for i in range(6)}
    sc = Scenario(tmp_path, targets=near, post_labels=("C0",))
    rc, _ = run_cli(sc.argv(), capsys)
    assert rc == 1
    _, summary = sc.outputs()
    sp = [p for p in summary["problems"] if "extremes" in p]
    assert sp and "x min" in sp[0] and "y min" in sp[0] and "y max" in sp[0] and "z max" in sp[0]


def test_too_few_points_and_a_missing_post_check(tmp_path, capsys):
    few = dict(list(TARGETS_BL.items())[:5])
    sc = Scenario(tmp_path, targets=few, post_labels=())
    rc, _ = run_cli(sc.argv(), capsys)
    assert rc == 1
    _, summary = sc.outputs()
    codes = [p.split(":")[0] for p in summary["problems"]]
    assert "ACCURACY_CHECK_TOO_FEW_POINTS" in codes and "POST_CHECK_MISSING" in codes


def test_post_check_drift(tmp_path, capsys):
    sc = Scenario(tmp_path, post_shift=(0.0, 0.0, 0.007))
    rc, _ = run_cli(sc.argv(), capsys)
    assert rc == 1
    _, summary = sc.outputs()
    assert [p.split(":")[0] for p in summary["problems"]] == ["POST_CHECK_DRIFT"]


# ----- choosing holds against the run --------------------------------------------------------------------------
def test_holds_are_chosen_against_the_run(tmp_path, capsys):
    """A pre hold after the run start, a post hold during the run, a repeated pre hold: only holds in the
    right place count, and the latest pre hold at each point before the run wins."""
    sc0 = Scenario(tmp_path / "probe")
    extra = ((sc0.run_start + 10.0, "pre:T1"), (sc0.run_start + 15.0, "x"),      # during the run: not pre
             (sc0.run_start + 20.0, "post:T2"), (sc0.run_start + 25.0, "x"),     # during the run: not post
             (1005.0, "pre:T5"), (1006.0, "x"))                                  # before: superseded by T5
    sc = Scenario(tmp_path / "s", extra_events=extra)
    rc, out = run_cli(sc.argv(), capsys)
    _, summary = sc.outputs()
    whys = {(h["phase"], h["label"], round(h["t_start_s"])): h["why"] for h in summary["holds"] if not h["used"]}
    assert whys[("pre", "T1", round(sc0.run_start + 10.5))] == "ends after the run started"
    assert whys[("post", "T2", round(sc0.run_start + 20.5))] == "starts before the run ended"
    assert whys[("pre", "T5", 1006)] == "superseded by a later pre hold at T5"
    used = [(h["phase"], h["label"]) for h in summary["holds"] if h["used"]]
    assert used.count(("pre", "T1")) == 1 and ("post", "T2") not in used
    # The moving arm during the extra windows is irrelevant: those holds were not used.
    assert rc == 0, summary["problems"]


def test_without_a_run_the_order_is_not_checked_and_two_post_holds_are_ambiguous(tmp_path, capsys):
    sc = Scenario(tmp_path)
    rc, _ = run_cli(sc.argv(run=False), capsys)
    assert rc == 0
    _, summary = sc.outputs()
    assert summary["order"]["checked"] is False and summary["accuracy"]["age_s"] is None
    assert any("no --run" in n for n in summary["notes"])
    sc2 = Scenario(tmp_path / "two", post_labels=("T1",),
                   extra_events=((5000.0, "post:T1"), (5005.0, "x")))
    rc, out = run_cli(sc2.argv(run=False), capsys)
    assert rc == 2 and "several post holds at T1" in out.err
    assert not (sc2.root / "out").exists()


def test_a_check_on_another_host_is_ordered_by_the_wall_clock(tmp_path, capsys):
    sc = Scenario(tmp_path)
    d = json.loads((sc.run / "declared.json").read_text())
    d["timestamps"]["monotonic_start_s"] += 5000.0          # another host's monotonic clock
    d["timestamps"]["monotonic_end_s"] += 5000.0
    (sc.run / "declared.json").write_text(json.dumps(d))
    rc, _ = run_cli(sc.argv(), capsys)
    assert rc == 0
    assert sc.outputs()[1]["order"]["clock"] == "wall"


def test_a_short_jitter_record_is_noted(tmp_path, capsys):
    sc = Scenario(tmp_path, jitter_s=6.0)
    rc, _ = run_cli(sc.argv(), capsys)
    _, summary = sc.outputs()
    assert rc == 0
    assert any("jitter record spans 5." in n for n in summary["notes"])


# ----- inputs ----------------------------------------------------------------------------------------------
def test_event_labels_as_typed():
    evs = [{"n": i, "t_mono_s": str(10.0 * i), "t_wall_s": str(10.0 * i + WALL), "label": lab}
           for i, lab in enumerate(["pre_T1", "PRE:T2", "post:T1", "jitter", "jitter:wrist_still", "pre",
                                    "pressure", "pre:T1:again", "moving"])]
    holds, notes = acc.holds_from_events(evs, settle_s=0.5, hold_s=3.0, jitter_s=15.0)
    assert [(h.phase, h.label) for h in holds] == [("pre", "T1"), ("pre", "T2"), ("post", "T1"),
                                                   ("jitter", "start_pose"), ("jitter", "wrist_still")]
    assert holds[0].t_start == 0.5 and holds[0].t_end == 3.5
    assert holds[3].t_end == 40.0                     # the next event, before the 15 s cap
    assert holds[0].wall(holds[0].t_end) == pytest.approx(3.5 + WALL)
    assert any("'pre' names no surveyed point" in n for n in notes)
    assert any("'pre:T1:again' looks like a hold" in n for n in notes)
    assert not any("pressure" in n for n in notes)


@pytest.mark.parametrize("text, msg", [
    ("frame = F\nunits = mm\nT1 = 1, 2, 3\n", "format = be-targets/0.1"),
    ("format = be-targets/0.1\nunits = mm\nT1 = 1, 2, 3\n", "which frame"),
    ("format = be-targets/0.1\nframe = F\nunits = cm\nT1 = 1, 2, 3\n", "units = mm or units = m"),
    ("format = be-targets/0.1\nframe = F\nunits = mm\nT1 = 1, 2\n", "three numbers"),
    ("format = be-targets/0.1\nframe = F\nunits = mm\nT1 = 1, 2, 3\nT1 = 1, 2, 3\n", "given twice"),
    ("format = be-targets/0.1\nframe = F\nunits = mm\nT 1 = 1, 2, 3\n", "neither a setting"),
    ("format = be-targets/0.1\nframe = F\nunits = mm\nT1 = 1, 2, nan\n", "three numbers"),
    ("format = be-targets/0.1\nframe = F\nunits = mm\n", "no surveyed points"),
    ("format = be-targets/0.1\nframe = F\nunits = mm\njust words\n", "expected KEY = VALUE"),
])
def test_targets_are_read_strictly(text, msg):
    with pytest.raises(acc.CheckError, match=msg):
        acc.parse_targets(text)


def test_targets_in_the_envelope_frame_need_no_registration(tmp_path):
    t = acc.parse_targets(targets_text(frame="base_link"))
    t = acc.to_envelope_frame(t, "base_link", None)
    assert t.refs["T3"] == pytest.approx(TARGETS_BL["T3"])
    with pytest.raises(acc.CheckError, match="give the registration"):
        acc.to_envelope_frame(acc.parse_targets(targets_text()), "base_link", None)
    with pytest.raises(acc.CheckError, match="frame 'world'"):
        acc.to_envelope_frame(acc.parse_targets(targets_text(frame="world")), "base_link", None)


def test_no_method_line_is_a_problem(tmp_path, capsys):
    sc = Scenario(tmp_path)
    sc.targets.write_text(targets_text(method=None))
    rc, _ = run_cli(sc.argv(), capsys)
    assert rc == 1
    assert any("no method line" in p for p in sc.outputs()[1]["problems"])


def test_a_holds_file_for_any_instrument(tmp_path, capsys):
    """Another instrument: its own samples and a holds file on its timebase, with wall times."""
    sc = Scenario(tmp_path)
    holds, _ = acc.holds_from_events(acc.read_events(sc.session), settle_s=0.5, hold_s=3.0, jitter_s=15.0)
    text = io.StringIO()
    text.write(",".join(acc.HOLDS_HEADER) + "\n")
    for h in holds:
        text.write(f"{h.phase},{h.label},{h.t_start},{h.t_end},{h.wall_start}\n")
    (tmp_path / "holds.csv").write_text(text.getvalue())
    reg = sc.session / "fit" / "registration.json"
    argv = ["instrument", "check", "--envelope", str(ENVELOPE), "--targets", str(sc.targets), "--holds",
            str(tmp_path / "holds.csv"), "--samples", str(sc.samples), "--registration", str(reg),
            "--run", str(sc.run), "--out", str(tmp_path / "out")]
    rc, out = run_cli(argv, capsys)
    assert rc == 0, out.out
    data, summary = sc.outputs()
    assert summary["order"]["clock"] == "wall"
    assert len([r for r in rawdata.read_accuracy(data) if r.phase == "pre"]) == 8


@pytest.mark.parametrize("header, row, msg", [
    ("phase,label,t_start_s,t_end_s", "pre,T1,1,2", "header must be"),
    (",".join(acc.HOLDS_HEADER), "pre,T1,2,1,", "t_end_s > t_start_s"),
    (",".join(acc.HOLDS_HEADER), "during,T1,1,2,", "phase must be"),
    (",".join(acc.HOLDS_HEADER), "pre,T 1,1,2,", "is not a label"),
    (",".join(acc.HOLDS_HEADER), "pre,T1,a,2,", "must be numbers"),
])
def test_holds_files_are_read_strictly(header, row, msg):
    with pytest.raises(acc.CheckError, match=msg):
        acc.holds_from_csv(header + "\n" + row + "\n")


def test_numbers_are_plain_decimals_core_accepts():
    assert acc._cell(1e-05) == "0.00001"
    assert acc._cell(-0.0) == "0"
    assert acc._cell(0.2350000001) == "0.2350000001"
    assert float(acc._cell(1.5e-12)) == 1.5e-12
    with pytest.raises(acc.CheckError):
        acc._cell(math.inf)


def test_the_check_reads_no_robot_state(tmp_path, capsys):
    """Planted state and command files in the run directory are never opened."""
    from _markerless_support import recording_opens

    sc = Scenario(tmp_path)
    (sc.run / "raw").mkdir()
    for name in ("state.csv", "commands.csv"):
        (sc.run / "raw" / name).write_text("planted\n")
    with recording_opens() as log:
        rc, _ = run_cli(sc.argv(), capsys)
    assert rc == 0
    opened = log.under(tmp_path)
    assert sc.samples.resolve() in opened and (sc.run / "declared.json").resolve() in opened
    assert not {p.name for p in opened} & {"state.csv", "commands.csv"}


# ----- the markerless tracker, end to end ---------------------------------------------------------------------
@pytest.mark.slow
def test_markerless_holds_are_fitted_and_checked_against_the_truth(tmp_path, capsys):
    """A rendered session: the arm still at six poses (pre holds), a jitter record and a post hold. With
    --session and no --samples the check fits only the hold windows; the references are the true tool
    positions, so each error is the tracker's own."""
    from _markerless_support import URDF, need_fit_stack

    need_fit_stack()
    import numpy as np

    from bounded_embodiment_kit.instruments.markerless import background, synth
    from bounded_embodiment_kit.kinematics import URDFModel

    u = URDFModel.from_file(URDF)
    joints = list(u.actuated)
    fps = 15.0
    rest = {j: math.radians(synth.REST_DEG.get(j, 0.0)) for j in joints}
    poses = {name: {**{j: math.radians(v) for j, v in synth.NOMINAL_DEG[name].items()},
                    "gripper": math.radians(synth.OPEN_DEG)}
             for name in ("READY", "A_UP", "B_UP", "A_DOWN", "B_DOWN")}
    poses["READY_ROLL"] = {**poses["READY"], "wrist_roll": math.radians(35)}
    seq = [("rest", rest, 0.7)] + [(k, v, 2.5) for k, v in poses.items()] + [("jitter", poses["READY"], 3.0),
                                                                            ("post", poses["A_UP"], 2.5)]
    Q, events, t, prev = [], [], 100.0, rest
    for name, q, hold in seq:
        if name != "rest":
            n = 8
            Q.extend(synth.interpolate(prev, q, joints, n))
            t += n / fps
            label = {"jitter": "jitter", "post": "post:A_UP"}.get(name, f"pre:{name}")
            events.append((t, label))
        else:
            events.append((t - 0.01, "background_start"))
        n = round(hold * fps)
        Q.extend([[q[j] for j in joints]] * n)
        t += n / fps
        events.append((t - 0.01, "background_end" if name == "rest" else "moving"))
        prev = q
    times = 100.0 + np.arange(len(Q)) / fps
    T = synth.default_camera(u)
    scene = synth.Scene(u, T, synth.default_K(640), clutter_F=synth.CLUTTER_F)
    session = tmp_path / "session"
    truth = synth.write_session(session, scene, times=times, qs=np.array(Q), events=events, seed=4, color_every=30)
    synth.write_registration(session, T)
    background.build_background(session, say=lambda s: None)
    # References: the true tool position during each hold.
    refs = {}
    for (te, label) in events:
        if label.startswith("pre:"):
            i = int(np.searchsorted(times, te + 1.0))
            refs[label[4:]] = truth["points"]["tool"][i]
    lines = [f"format = {acc.TARGETS_FORMAT}", "frame = base_link", "units = m", "method = synthetic truth"]
    lines += [f"{k} = {float(p[0])!r}, {float(p[1])!r}, {float(p[2])!r}" for k, p in refs.items()]
    (tmp_path / "targets.txt").write_text("\n".join(lines) + "\n")
    rc, out = run_cli(["instrument", "check", "--envelope", str(ENVELOPE), "--targets", str(tmp_path / "targets.txt"),
                       "--session", str(session), "--out", str(tmp_path / "out"), "--hold-s", "1.5",
                       "--jitter-s", "2.0", "--urdf", str(URDF)], capsys)
    assert (tmp_path / "out" / "accuracy-check.json").is_file(), out.out + out.err
    summary = json.loads((tmp_path / "out" / "accuracy-check.json").read_text())
    errors = {e["label"]: e["error_m"] for e in summary["accuracy"]["pre"]["errors"]}
    assert sorted(errors) == sorted(poses), out.out
    assert max(errors.values()) < 0.003, errors
    assert summary["accuracy"]["jitter"]["sigma_m"] < 0.0015
    assert summary["accuracy"]["post"]["max_drift_m"] < 0.003
    assert (tmp_path / "out" / "fit").is_dir()
    # Only the hold windows were fitted.
    fitted = sum(json.loads((d / "fit_meta.json").read_text())["counts"]["frames"]
                 for d in (tmp_path / "out" / "fit").iterdir())
    assert fitted < len(times) / 2
    assert rc == 1   # six poses near READY do not spread through bob's keep-in, and the jitter is short
    assert any("extremes" in p for p in summary["problems"])
