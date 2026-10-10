"""The markerless fit (`be fit run`): its rules (independence, no smoothing, honest validity, determinism)
and its accuracy on synthetic recordings with known truth."""

from __future__ import annotations

import csv
import dataclasses
import hashlib
import inspect
import json
import math
from importlib import metadata
from types import SimpleNamespace

import numpy as np
import pytest
from _markerless_support import (
    ENVELOPE,
    HAND,
    NAMED,
    POINTS,
    URDF,
    read_samples_csv,
    shifts_mm,
)
from bounded_embodiment_kit.instruments.markerless import fit, synth


# ----- no robot meshes needed ---------------------------------------------------------------------------
def test_distortion_matches_opencv_and_inverts():
    rng = np.random.default_rng(0)
    x, y = rng.uniform(-0.6, 0.6, (2, 500))
    dist = [-0.12, 0.08, 1e-4, -2e-4, -0.02, 0.01, 0.005, -0.003, 1e-4, -1e-4, 2e-4, -2e-4]
    xd, yd = fit.distort_normalized(x, y, dist)
    xu, yu = fit.undistort_normalized(xd, yd, dist)
    assert np.allclose(xu, x, atol=1e-9) and np.allclose(yu, y, atol=1e-9)
    cv2 = pytest.importorskip("cv2")
    P = np.column_stack([x, y, np.ones_like(x)])
    uv, _ = cv2.projectPoints(P, np.zeros(3), np.zeros(3), np.eye(3), np.array(dist))
    assert np.allclose(uv.reshape(-1, 2), np.column_stack([xd, yd]), atol=1e-9)


def test_camera_refuses_a_tilted_sensor_model():
    K = np.array([[100.0, 0, 4], [0, 100.0, 3], [0, 0, 1]])
    with pytest.raises(fit.FitError, match="tilted"):
        fit.Camera(K, [0.0] * 12 + [0.01, 0.0], (8, 6), np.eye(4))


def test_load_registration_checks_the_transform(tmp_path):
    with pytest.raises(fit.FitError, match="be fit register"):
        fit.load_registration(tmp_path / "registration.json")
    with pytest.raises(fit.FitError, match="4x4"):
        fit.load_registration({"T_bl_cam": [[1, 0], [0, 1]]})
    stretched = np.eye(4)
    stretched[0, 0] = 1.01
    mirrored = np.diag([1.0, 1.0, -1.0, 1.0])
    for T in (stretched, mirrored):
        with pytest.raises(fit.FitError, match="rigid"):
            fit.load_registration({"T_bl_cam": T.tolist()})
    T = np.eye(4)
    T[:3, 3] = (0.5, 0.2, 0.4)
    assert np.array_equal(fit.load_registration({"T_bl_cam": T.tolist()})["T_bl_cam"], T)


def test_thresholds_live_in_one_frozen_config():
    cfg = fit.FitConfig()
    d = cfg.as_dict()
    json.dumps(d)
    assert (d["max_points"], d["min_depth_m"], d["above_table_m"]) == (4000, 0.30, 0.008)
    assert (d["huber_m"], d["gate_m"], d["max_iterations"]) == (0.005, 0.025, 15)
    assert (d["max_rms_m"], d["jump_speed_mps"], d["jump_margin_m"]) == (0.006, 1.5, 0.030)
    with pytest.raises(dataclasses.FrozenInstanceError):
        cfg.max_rms_m = 1.0


def test_the_api_takes_only_camera_side_inputs():
    params = list(inspect.signature(fit.fit_session).parameters)
    assert params[:4] == ["session", "urdf", "envelope", "registration"]
    assert set(params[4:]) <= {"background", "config", "out", "t_from", "t_to", "every", "debug_every", "model",
                               "say", "max_frames"}


# ----- one rendered frame ---------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def frame(fit_model):
    u = fit_model.urdf
    T = synth.default_camera(u)
    scene = synth.Scene(u, T, synth.default_K(640), clutter_F=synth.CLUTTER_F)
    q = np.radians([20.0, -50.0, 35.0, 60.0, 30.0, 20.0])
    depth, ids = scene.render(fit_model.q_dict(q))
    rng = np.random.default_rng(7)
    static, _ = scene._static_layer()
    bg = np.where(np.isfinite(static), np.rint(static * 1000), 0).reshape(depth.shape).astype(np.uint16)
    return SimpleNamespace(q=q, depth=synth.noisify(depth, ids, rng), ids=ids, background=bg,
                           clean=np.where(np.isfinite(depth), np.rint(depth * 1000), 0).astype(np.uint16),
                           cam=fit.Camera(synth.default_K(640), np.zeros(5), (640, 360), T))


def test_observe_keeps_the_arm_and_drops_the_rest(fit_model, frame):
    cfg = fit.FitConfig()
    ids = frame.ids.ravel()
    base = fit_model.links.index(fit_model.root)
    moving = (ids >= 0) & (ids != base)
    clean = fit.observe(frame.clean, frame.cam, fit_model, cfg, frame.background)
    assert moving[clean.data_mask].all()                            # exact depth: nothing but the moving links
    assert clean.n_candidates > 0.95 * moving.sum()                 # ... and nearly all of them
    obs = fit.observe(frame.depth, frame.cam, fit_model, cfg, frame.background)
    assert moving[obs.data_mask].mean() > 0.97                      # noisy: stray wall and table pixels are rare
    assert obs.n_points == min(cfg.max_points, obs.n_candidates)
    assert (obs.points[:, 2] >= fit_model.table_z + cfg.above_table_m - 1e-9).all()


def test_the_estimate_does_not_depend_on_the_starting_guess(fit_model, frame):
    """No prior pulls the estimate toward the previous one: from anywhere in the basin, the same answer."""
    cfg = fit.FitConfig()
    obs = fit.observe(frame.depth, frame.cam, fit_model, cfg, frame.background)
    ref = fit.solve(fit_model, frame.cam, obs, frame.q, cfg)
    assert ref.converged
    p_ref = fit_model.urdf.points(NAMED, fit_model.q_dict(ref.q))
    p_true = fit_model.urdf.points(NAMED, fit_model.q_dict(frame.q))
    assert max(np.linalg.norm(p_ref[k] - p_true[k]) for k in POINTS) < 0.003
    for signs in ((1, -1, 1, -1, 1, 0), (-1, 1, -1, 1, -1, 0), (1, 1, -1, -1, 1, 0)):
        q0 = frame.q + np.radians(3.0) * np.array(signs)
        s = fit.solve(fit_model, frame.cam, obs, q0, cfg)
        p = fit_model.urdf.points(NAMED, fit_model.q_dict(s.q))
        assert s.converged
        assert max(np.linalg.norm(p[k] - p_ref[k]) for k in POINTS) < 0.0005


@pytest.mark.slow
def test_after_a_lost_frame_the_tracker_starts_from_scratch(fit_model, frame):
    tr = fit.Tracker(fit_model, frame.cam, NAMED, fit.FitConfig(), frame.background)
    blank = np.zeros_like(frame.depth)
    r = tr.step(blank, 0.0)
    assert (r.status, r.valid, tr.prev_q) == ("LOST", False, None) and not any(r.point_valid.values())
    assert "observed points" in r.reason
    assert tr.step(frame.depth, 1 / 30).status == "REINIT"
    assert tr.step(frame.depth, 2 / 30).status == "TRACK"
    assert tr.step(blank, 3 / 30).status == "LOST" and tr.prev_q is None
    r = tr.step(frame.depth, 4 / 30)
    assert r.status == "REINIT" and r.valid                       # a global search, not the old estimate


# ----- the whole procedure on a 10 s recording -------------------------------------------------------------
@pytest.mark.slow
def test_exact_model_accuracy_over_10_s_at_30_fps(full_run, capsys):
    res, t = full_run.score, full_run.meta["timing_ms_per_frame"]
    with capsys.disabled():
        print(f"\nexact model, 10 s at 30 fps: {full_run.meta['counts']['status']}; ms/frame mean {t['mean']:.0f} "
              f"p50 {t['p50']:.0f} p95 {t['p95']:.0f} max {t['max']:.0f}")
        for k in POINTS:
            v = res[k]
            print(f"  {k:5s} valid {v['valid']}/{v['n']}  p50 {v['p50_mm']:.2f}  p95 {v['p95_mm']:.2f}  "
                  f"max {v['max_mm']:.2f} mm")
    for k in POINTS:
        assert res[k]["n"] == 300
        assert res[k]["valid_frac"] >= 0.9
        assert res[k]["p95_mm"] <= 3.0


@pytest.mark.slow
def test_outputs_are_be_samples_csv_and_say_where_they_came_from(full_run):
    from bounded_embodiment.rawdata import read_samples
    from bounded_embodiment_kit.instruments.session import Session

    out = full_run.root / "fit"
    rows = read_samples(out / "samples.csv")
    assert len(rows) == 300 * len(POINTS)
    with Session(full_run.root) as sess:
        t_depth = sess.depth["t"][sess.depth_indices(sess.resolve_time("nominal_1"))]
    assert np.allclose(sorted({r.t_s for r in rows}), t_depth, atol=1e-6)    # t_s = the depth frame's t_mono_s
    for r in rows:
        assert r.point in POINTS and r.residual_m is not None
        assert (r.x_m is None) == (r.valid == 0)
    with open(out / "fit_log.csv", newline="", encoding="utf-8") as fh:
        log = list(csv.DictReader(fh))
    assert len(log) == 300
    assert {"cost", "alt_cost", "points", "inliers", "rms_m", "free_space_frac", "status", "valid", "reason",
            "ms", *full_run.meta["model"]["joints"]} <= set(log[0])
    assert {r["status"] for r in log} <= {"TRACK", "REINIT", "LOST"} and log[0]["status"] == "REINIT"
    meta = json.loads((out / "fit_meta.json").read_text(encoding="utf-8"))
    assert meta["instrument"]["independent_of_robot_state"] is True
    inputs = meta["inputs"]
    assert inputs["urdf"]["sha256"] == hashlib.sha256(URDF.read_bytes()).hexdigest()
    assert "base_so101_v2.stl" in inputs["meshes"]
    assert inputs["registration"]["sha256"] and inputs["background"]["sha256"]
    assert meta["config"] == fit.FitConfig().as_dict()
    assert meta["counts"]["frames"] == 300 and meta["outputs"]["samples_format"] == "be-samples-csv/1"


@pytest.mark.slow
def test_never_opens_robot_state_or_the_truth(full_run):
    """Harness logs and probe files planted in the session, and the renderer's truth file, are never opened
    by register, background or run; everything opened in the session is a camera file or the fit's own."""
    root = full_run.root.resolve()
    opened = full_run.opened.under(root)
    for p in full_run.planted:
        assert p not in opened, p
    assert (root / "synth_truth.npz") not in opened
    for p in opened:
        rel = p.relative_to(root)
        assert rel.parts[0] in ("oakd", "wrist", "fit") or rel.name in (
            "meta.json", "events.csv", "measurements.txt", "board.json"), rel
    assert any(p.suffix == ".npz" and "depth" in p.parts for p in opened)     # the log does see real reads


# ----- short recordings: determinism, holes, a blackout, a hand ---------------------------------------------
@pytest.mark.slow
def test_two_runs_write_identical_bytes(motion, tmp_path):
    a = motion.fit("base")                                     # the shared model ...
    fit.fit_session(a.root, URDF, ENVELOPE, t_from="nominal_1", out=tmp_path, say=lambda s: None)   # ... a fresh one
    assert (a.out / "samples.csv").read_bytes() == (tmp_path / "samples.csv").read_bytes()

    def without_ms(p):
        with open(p, newline="", encoding="utf-8") as fh:
            return [{k: v for k, v in r.items() if k != "ms"} for r in csv.DictReader(fh)]

    assert without_ms(a.out / "fit_log.csv") == without_ms(tmp_path / "fit_log.csv")


@pytest.mark.slow
def test_twenty_percent_holes(motion, capsys):
    r = motion.fit("holes20", noise=synth.NoiseModel(holes=0.20))
    with capsys.disabled():
        print("\n20 % holes: " + ", ".join(f"{k} valid {v['valid']}/{v['n']} p95 {v['p95_mm']:.2f} mm"
                                           for k, v in r.score.items()))
    for k in POINTS:
        assert r.score[k]["valid_frac"] >= 0.75
        assert r.score[k]["p95_mm"] <= 3.0


@pytest.mark.slow
def test_a_one_second_blackout_is_invalid_and_tracking_resumes_after(motion):
    r = motion.fit("blackout", blackout=(1.0, 2.0))
    truth = r.truth
    with open(r.out / "fit_log.csv", newline="", encoding="utf-8") as fh:
        log = list(csv.DictReader(fh))
    dark = [bool(truth["blackout_mask"][int(np.argmin(np.abs(truth["times"] - float(row["t_s"]))))]) for row in log]
    assert sum(dark) == 30
    for row, d in zip(log, dark, strict=True):
        if d:
            assert (row["status"], row["valid"]) == ("LOST", "0") and "observed points" in row["reason"]
    first_after = dark.index(True) + 30
    assert (log[first_after]["status"], log[first_after]["valid"]) == ("REINIT", "1")
    after = [row for row, d in zip(log[first_after:], dark[first_after:], strict=True) if not d]
    assert sum(row["valid"] == "1" for row in after) >= 0.9 * len(after)
    samples = read_samples_csv(r.out / "samples.csv")
    dark_t = {row["t_s"] for row, d in zip(log, dark, strict=True) if d}
    assert all(s["valid"] == "0" and s["x_m"] == "" for s in samples if s["t_s"] in dark_t)
    after_rows = [s for s in samples if float(s["t_s"]) >= float(log[first_after]["t_s"])]
    assert r.score["tool"]["p95_mm"] <= 3.0
    assert sum(s["valid"] == "1" for s in after_rows) >= 0.8 * len(after_rows)


@pytest.mark.slow
def test_a_hand_near_the_base_moves_the_fit_less_than_5_mm(motion, capsys):
    """A 40 mm sphere appears beside the shoulder after the background was recorded (so it is foreground
    data, within ~9 mm of the arm at its closest); compared with the same recording without it."""
    base = motion.fit("base")
    hand = motion.fit("hand", hand=HAND)
    shift = shifts_mm(read_samples_csv(base.out / "samples.csv"), read_samples_csv(hand.out / "samples.csv"))
    with capsys.disabled():
        print("\nhand near the base, shift: " + ", ".join(
            f"{k} n {len(v)} p95 {np.percentile(v, 95):.2f} max {v.max():.2f} mm" for k, v in shift.items()))
    for k in POINTS:
        assert len(shift[k]) >= 0.5 * base.score[k]["n"]
        assert shift[k].max() < 5.0


@pytest.mark.slow
def test_sensitivity_to_registration_yaw_and_link_length_is_measured(motion, capsys):
    """Measured and printed, not judged: a registration yawed 1 deg about the board's corner O, and an arm
    whose upper arm and forearm are each 2 mm longer than the URDF the fitter uses."""
    from bounded_embodiment_kit.instruments.markerless.model import base_offset
    from bounded_embodiment_kit.kinematics import URDFModel

    exact = motion.fit("base")
    u = URDFModel.from_file(URDF)
    O_bl = (base_offset(u).T_root_F() @ synth.default_board_T_F())[:3, 3]
    reg = synth.write_registration(exact.root, synth.yawed(exact.truth["T_bl_cam"], 1.0, O_bl),
                                   out=exact.root / "yaw_registration")
    yaw = motion.fit("base", out_name="fit_yaw", registration=reg)
    longer = motion.fit("longer", scene_urdf=synth.lengthened_urdf(URDF, 0.002))
    with capsys.disabled():
        for label, r in (("exact", exact), ("registration yaw 1 deg", yaw), ("links 2 mm longer", longer)):
            print(f"\n{label:24s}" + "  ".join(f"{k} p50 {v['p50_mm']:.2f} p95 {v['p95_mm']:.2f} max "
                                               f"{v['max_mm']:.2f} mm ({v['valid']}/{v['n']})"
                                               for k, v in r.score.items()), end="")
        print()
    for r in (yaw, longer):
        assert set(r.score) == set(POINTS) and all(math.isfinite(v["max_mm"] or math.nan) for v in r.score.values())


# ----- the command line --------------------------------------------------------------------------------------
@pytest.mark.slow
def test_be_fit_command_line(motion, tmp_path, monkeypatch):
    core_cli = pytest.importorskip("bounded_embodiment.cli")
    ep = metadata.EntryPoint(name="kit", value="bounded_embodiment_kit.cli:register", group="bounded_embodiment.cli")
    real = metadata.entry_points
    monkeypatch.setattr(metadata, "entry_points",
                        lambda **kw: [ep] if kw.get("group") == "bounded_embodiment.cli" else real(**kw))
    root, _ = motion.session("base")
    assert core_cli.main(["fit"]) == 2
    assert core_cli.main(["fit", "background", str(root), "--out", str(tmp_path)]) == 0
    assert (tmp_path / "background.npz").is_file()
    rc = core_cli.main(["fit", "run", str(root), "--envelope", str(ENVELOPE), "--urdf", str(URDF),
                        "--background", str(tmp_path / "background.npz"), "--from", "nominal_1+1",
                        "--max-frames", "3", "--debug-every", "2", "--out", str(tmp_path / "run")])
    assert rc == 0
    assert len(read_samples_csv(tmp_path / "run" / "samples.csv")) == 3 * len(POINTS)
    assert sorted(p.name for p in (tmp_path / "run" / "debug").iterdir()) == ["frame_000000.png", "frame_000002.png"]
    assert core_cli.main(["fit", "run", str(root), "--envelope", str(ENVELOPE), "--from", "no_such_event"]) == 2
