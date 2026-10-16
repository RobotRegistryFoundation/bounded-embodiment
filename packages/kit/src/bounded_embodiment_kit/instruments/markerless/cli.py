# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""`be fit`: the markerless model tracker on a recording made by `be record`.

  be fit register SESSION [--urdf URDF] [--board-json PATH]
        camera -> base_link from the ChArUco board after the last board_placed and measurements.txt;
        writes SESSION/fit/registration.json (exit 1 if it has warnings)
  be fit background SESSION [--window START:END ...]
        static depth background (default: background_start..background_end); SESSION/fit/background.npz
  be fit run SESSION --envelope ENV [--urdf URDF] [--from LABEL] [--to LABEL] [--every N] [--out DIR]
        [--debug-every N]
        fits the URDF to every N-th depth frame; writes samples.csv (be-samples-csv/1), fit_log.csv and
        fit_meta.json to DIR (default SESSION/fit), and debug overlays to DIR/debug every N frames
  be fit selftest [--quick] [--sensitivity] [--keep DIR]
        renders a synthetic session with known truth, runs all three steps, prints the accuracy;
        --sensitivity also measures a 1 deg registration yaw and a 2 mm longer upper arm and forearm

--from/--to take an event label (the last one with that label), LABEL+S / LABEL-S, or seconds since the
recording started. --urdf defaults to the URDF the registration used, then to the repository's SO-101.
The fit never reads encoder values, commands, harness logs or probe files.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import tempfile
import time
from pathlib import Path

__all__ = ["build_synthetic_session", "register", "run", "score", "selftest", "sensitivity"]

REPO_URDF = Path("examples") / "robots" / "so101" / "so101_new_calib.urdf"
REPO_ENVELOPE = Path("examples") / "envelopes" / "bob.envelope.json"


def _repo_file(rel: Path) -> Path | None:
    for parent in Path(__file__).resolve().parents:
        p = parent / rel
        if p.is_file():
            return p
    p = Path.cwd() / rel
    return p if p.is_file() else None


def _default_urdf(session: Path | None) -> Path:
    if session is not None:
        reg = session / "fit" / "registration.json"
        if reg.is_file():
            try:
                u = (json.loads(reg.read_text(encoding="utf-8")).get("inputs") or {}).get("urdf") or {}
                p = Path(u["path"]) if u.get("path") else None
                if p is not None and p.is_file() and (not u.get("sha256") or
                                                      hashlib.sha256(p.read_bytes()).hexdigest() == u["sha256"]):
                    return p
            except (ValueError, KeyError, OSError):
                pass
    p = _repo_file(REPO_URDF)
    if p is None:
        raise SystemExit("be fit: give --urdf (no registration.json naming one, and no repository SO-101 URDF found)")
    return p


def register(subparsers) -> argparse.ArgumentParser:
    """Adds `fit` to the `be` command line (entry-point group bounded_embodiment.cli)."""
    p = subparsers.add_parser("fit", help="markerless model tracker: register the camera, fit the URDF to depth",
                              description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="fit_command", metavar="{register,background,run,selftest}")
    r = sub.add_parser("register", help="camera -> base_link from the board and measurements.txt")
    r.add_argument("session", help="the session directory be record wrote")
    r.add_argument("--urdf", default=None, help="the robot's URDF (default: the repository's SO-101)")
    r.add_argument("--board-json", default=None, help="board spec (default: SESSION/board.json, else meta.json)")
    r.add_argument("--out", default=None, help="directory for registration.json (default SESSION/fit)")
    b = sub.add_parser("background", help="static depth background")
    b.add_argument("session", help="the session directory be record wrote")
    b.add_argument("--window", action="append", default=None, metavar="START:END",
                   help="still window(s); several combine as the farthest valid depth (default: background_start:"
                        "background_end)")
    b.add_argument("--out", default=None, help="directory for background.npz (default SESSION/fit)")
    f = sub.add_parser("run", help="fit the URDF to the depth frames")
    f.add_argument("session", help="the session directory be record wrote")
    f.add_argument("--envelope", required=True, help="envelope JSON (its points that name a link are reported)")
    f.add_argument("--urdf", default=None, help="default: the URDF the registration used")
    f.add_argument("--registration", default=None, help="default SESSION/fit/registration.json")
    f.add_argument("--background", default=None, help="default SESSION/fit/background.npz")
    f.add_argument("--from", dest="t_from", default=None,
                   help="start: an event label (its last occurrence), LABEL+S, LABEL-S, or seconds")
    f.add_argument("--to", dest="t_to", default=None, help="end, in the same forms as --from")
    f.add_argument("--every", type=int, default=1, help="fit every Nth depth frame (default 1, every frame)")
    f.add_argument("--out", default=None, help="output directory (default SESSION/fit)")
    f.add_argument("--debug-every", type=int, default=0, help="write a debug overlay every N frames (default 0, none)")
    f.add_argument("--max-frames", type=int, default=None, help="stop after this many frames")
    s = sub.add_parser("selftest", help="synthetic end-to-end test: accuracy and ms/frame")
    s.add_argument("--quick", action="store_true", help="2 s of motion instead of 10 s")
    s.add_argument("--sensitivity", action="store_true",
                   help="also measure (not judge) a 1 deg registration yaw and 2 mm longer links")
    s.add_argument("--urdf", default=None, help="the robot's URDF (default: the repository's SO-101)")
    s.add_argument("--keep", default=None, metavar="DIR", help="write the synthetic session(s) here and keep them")
    p.set_defaults(func=run, _fit_parser=p)
    return p


def run(args) -> int:
    from .register import RegistrationError

    cmd = getattr(args, "fit_command", None)
    try:
        if cmd == "register":
            from .register import register_session

            session = Path(args.session)
            spec = json.loads(Path(args.board_json).read_text()) if args.board_json else None
            doc = register_session(session, args.urdf or _default_urdf(None), board_spec=spec, out=args.out)
            return 1 if doc["warnings"] else 0
        if cmd == "background":
            from .background import build_background

            build_background(Path(args.session), windows=args.window, out=args.out)
            return 0
        if cmd == "run":
            from .fit import fit_session

            session = Path(args.session)
            meta = fit_session(session, args.urdf or _default_urdf(session), args.envelope,
                               registration=args.registration, background=args.background, out=args.out,
                               t_from=args.t_from, t_to=args.t_to, every=args.every,
                               debug_every=args.debug_every, max_frames=args.max_frames)
            c = meta["counts"]
            print(f"fit: {c['frames']} frames, {c['valid_frames']} valid ({c['status']}); "
                  f"{meta['timing_ms_per_frame']['mean']:.0f} ms/frame -> {Path(meta['outputs']['samples'])}")
            return 0
        if cmd == "selftest":
            return selftest(quick=args.quick, urdf=args.urdf, keep=args.keep, with_sensitivity=args.sensitivity)
        args._fit_parser.print_help()
        return 2
    except (RegistrationError, ValueError, FileNotFoundError, RuntimeError) as e:
        print(f"be fit: {e}", file=sys.stderr)
        return 2


# ----- synthetic sessions and scoring --------------------------------------------------------------------
def build_synthetic_session(root, urdf_path, *, nominal_s=10.0, fps=30.0, board_s=10.0, board_fps=3.0,
                            background_s=2.0, sweep_s=2.0, seed=1, noise=None, measurements_round_mm=None,
                            clutter=True, say=print) -> dict:
    """The whole session, as the 10 Oct runbook has it: board on the table (arm resting folded), board away,
    background (arm still resting), a slow sweep to the ready pose, then the pick-and-place."""
    import numpy as np

    from ...kinematics import URDFModel
    from . import synth

    u = URDFModel.from_file(urdf_path)
    joints = list(u.actuated)
    T = synth.default_camera(u)
    board = synth.default_board_T_F()
    scene = synth.Scene(u, T, synth.default_K(640), board_T_F=board, clutter_F=synth.CLUTTER_F if clutter else [])
    rest = {j: math.radians(synth.REST_DEG.get(j, 0.0)) for j in joints}
    ready = {**{j: math.radians(v) for j, v in synth.NOMINAL_DEG["READY"].items()},
             "gripper": math.radians(synth.OPEN_DEG)}
    t0 = 100.0
    seg = []                                         # (times, q, board)
    tb = t0 + np.arange(0.0, board_s, 1.0 / board_fps)
    seg.append((tb, np.tile([rest[j] for j in joints], (len(tb), 1)), True))
    t = tb[-1] + 2.0
    tg = t + np.arange(0.0, background_s, 1.0 / fps)  # background: arm resting, board away
    seg.append((tg, np.tile([rest[j] for j in joints], (len(tg), 1)), False))
    t = tg[-1] + 1.0 / fps
    ts = t + np.arange(0.0, sweep_s, 1.0 / fps)      # goto-start sweep
    seg.append((ts, synth.interpolate(rest, ready, joints, len(ts)), False))
    t = ts[-1] + 1.0 / fps
    tn, qn = synth.nominal_trajectory(u, nominal_s, fps)
    seg.append((t + tn, qn, False))
    times = np.concatenate([s[0] for s in seg])
    Q = np.concatenate([s[1] for s in seg])
    boardf = np.concatenate([np.full(len(s[0]), s[2]) for s in seg])
    events = [(t0 + 0.2, "board_placed"), (tb[-1] + 0.5, "board_removed"), (tg[0] - 0.01, "background_start"),
              (tg[-1] + 0.01, "background_end"), (ts[0] - 0.01, "goto_start"), (t - 0.01, "nominal_1"),
              (times[-1] + 0.01, "nominal_1_done")]
    meas = synth.measurements_text(board, scene.board_height_m, round_mm=measurements_round_mm)
    kw = {} if noise is None else {"noise": noise}
    tw = time.perf_counter()
    truth = synth.write_session(root, scene, times=times, qs=Q, events=events, board_frames=boardf,
                                measurements=meas, seed=seed, **kw)
    say(f"selftest: rendered {len(times)} frames in {time.perf_counter() - tw:.0f} s -> {root}")
    truth["T_bl_cam"] = T
    truth["T_bl_board"] = scene.T_bl_F @ board
    truth["t_nominal"] = t - 0.01
    return truth


def score(samples_csv, truth, t_from=None) -> dict:
    """Per point p50/p95/max error (mm) of the valid samples against the truth, and the valid fraction."""
    import csv

    import numpy as np

    tt = truth["times"]
    err, n_all, n_valid = {}, {}, {}
    with open(samples_csv, newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            t = float(r["t_s"])
            if t_from is not None and t < t_from:
                continue
            k = r["point"]
            n_all[k] = n_all.get(k, 0) + 1
            if r["valid"] != "1":
                continue
            n_valid[k] = n_valid.get(k, 0) + 1
            i = int(np.argmin(np.abs(tt - t)))
            p = np.array([float(r["x_m"]), float(r["y_m"]), float(r["z_m"])])
            err.setdefault(k, []).append(float(np.linalg.norm(p - truth["points"][k][i])) * 1000)
    out = {}
    for k, n in n_all.items():
        e = np.array(err.get(k, []))
        out[k] = {"n": n, "valid": n_valid.get(k, 0), "valid_frac": n_valid.get(k, 0) / n,
                  "p50_mm": float(np.median(e)) if e.size else None,
                  "p95_mm": float(np.percentile(e, 95)) if e.size else None,
                  "max_mm": float(e.max()) if e.size else None}
    return out


def _say_score(say, label: str, res: dict) -> None:
    for k, v in res.items():
        nan = float("nan")
        say(f"{label}{k:6s} valid {v['valid']}/{v['n']}  error p50 {v['p50_mm'] if v['p50_mm'] is not None else nan:.2f}"
            f"  p95 {v['p95_mm'] if v['p95_mm'] is not None else nan:.2f}"
            f"  max {v['max_mm'] if v['max_mm'] is not None else nan:.2f} mm")


def sensitivity(root, urdf_path, env_path, *, model=None, nominal_s=4.0, yaw_deg=1.0, longer_m=0.002, seed=2,
                say=print) -> dict:
    """Measure (not judge) how two errors the synthetic truth does not have move the reported points: the
    registration yawed by ``yaw_deg`` about the board's corner O, and an arm whose upper arm and forearm are
    each ``longer_m`` longer than the URDF the fitter uses. Writes two motion sessions under ``root``."""
    from ...kinematics import URDFModel
    from . import background, fit, synth
    from .model import ArmModel, base_offset

    root = Path(root)
    u = URDFModel.from_file(urdf_path)
    model = model or ArmModel(urdf_path)
    out = {}
    truth = synth.write_motion_session(root / "exact", u, nominal_s=nominal_s, seed=seed)
    background.build_background(root / "exact", say=lambda s: None)
    fit.fit_session(root / "exact", urdf_path, env_path, model=model, t_from="nominal_1", say=lambda s: None)
    out["exact"] = score(root / "exact" / "fit" / "samples.csv", truth)
    O_bl = (base_offset(u).T_root_F() @ synth.default_board_T_F())[:3, 3]
    T_yaw = synth.yawed(truth["T_bl_cam"], yaw_deg, O_bl)
    reg = synth.write_registration(root / "exact", T_yaw, note=f"{yaw_deg} deg yaw about board corner O",
                                   out=root / "exact" / "fit_yaw")
    fit.fit_session(root / "exact", urdf_path, env_path, reg, model=model, t_from="nominal_1",
                    out=root / "exact" / "fit_yaw", say=lambda s: None)
    out[f"registration_yaw_{yaw_deg:g}deg"] = score(root / "exact" / "fit_yaw" / "samples.csv", truth)
    truth_m = synth.write_motion_session(root / "longer", u, nominal_s=nominal_s, seed=seed,
                                         scene_urdf=synth.lengthened_urdf(urdf_path, longer_m))
    background.build_background(root / "longer", say=lambda s: None)
    fit.fit_session(root / "longer", urdf_path, env_path, model=model, t_from="nominal_1", say=lambda s: None)
    out[f"links_{longer_m * 1000:g}mm_longer"] = score(root / "longer" / "fit" / "samples.csv", truth_m)
    for label, res in out.items():
        _say_score(say, f"sensitivity [{label}] ", res)
    return out


def selftest(*, quick=False, urdf=None, keep=None, with_sensitivity=False, say=print) -> int:
    import numpy as np

    from ...kinematics import URDFModel
    from . import background, fit
    from . import register as reg
    from .mesh import MeshError, check_meshes
    from .model import ArmModel

    urdf_path = Path(urdf) if urdf else _repo_file(REPO_URDF)
    env_path = _repo_file(REPO_ENVELOPE)
    if urdf_path is None or env_path is None:
        say("selftest: needs the repository's examples/robots/so101 URDF and examples/envelopes/bob.envelope.json")
        return 2
    try:
        check_meshes(URDFModel.from_file(urdf_path))
    except MeshError as e:
        say(f"selftest: {e}")
        return 2
    tmp = None
    if keep:
        base = Path(keep)
        if base.exists() and any(base.iterdir()):
            say(f"selftest: {base} is not empty")
            return 2
    else:
        tmp = tempfile.TemporaryDirectory(prefix="be-fit-selftest-")
        base = Path(tmp.name)
    root = base / "session"
    try:
        truth = build_synthetic_session(root, urdf_path, nominal_s=2.0 if quick else 10.0, say=say)
        doc = reg.register_session(root, urdf_path, say=lambda s: None)
        T = np.array(doc["T_bl_cam"])
        dR = T[:3, :3].T @ truth["T_bl_cam"][:3, :3]
        rot = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(dR) - 1) / 2))))
        say(f"selftest: registration off by {np.linalg.norm(T[:3, 3] - truth['T_bl_cam'][:3, 3]) * 1000:.2f} mm, "
            f"{rot:.3f} deg (PnP {doc['checks']['pnp']['reprojection_rms_px']:.2f} px)")
        background.build_background(root, say=lambda s: None)
        tm = time.perf_counter()
        model = ArmModel(urdf_path)
        say(f"selftest: model built in {time.perf_counter() - tm:.1f} s")
        meta = fit.fit_session(root, urdf_path, env_path, model=model, t_from="nominal_1", say=lambda s: None)
        res = score(root / "fit" / "samples.csv", truth)
        _say_score(say, "selftest: ", res)
        ok = all(v["valid_frac"] >= 0.9 and v["p95_mm"] is not None and v["p95_mm"] <= 3.0 for v in res.values())
        tms = meta["timing_ms_per_frame"]
        say(f"selftest: {meta['counts']['status']}; ms/frame mean {tms['mean']:.0f}, p50 {tms['p50']:.0f}, p95 "
            f"{tms['p95']:.0f}, max {tms['max']:.0f}")
        if with_sensitivity:
            sensitivity(base / "sensitivity", urdf_path, env_path, model=model, say=say)
        say("selftest: " + ("PASS (p95 <= 3 mm, >= 90 % valid)" if ok else "FAIL"))
        return 0 if ok else 1
    finally:
        if tmp is not None:
            tmp.cleanup()
