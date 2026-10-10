# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""Where the camera is in base_link (`be fit register`).

The chain is ``T_bl_cam = T_bl_F . T_F_board . T_board_cam``:

* ``T_board_cam``: the ChArUco board lying on the table, seen by the colour camera (1280x720 intrinsics and
  distortion from meta.json). Every colour frame from 1 s to 9 s after the last ``board_placed`` event (and
  before the next event) is searched; all corner observations are stacked into one PnP (IPPE on the best
  frame to start, then iterative, then Levenberg-Marquardt refinement).
* ``T_F_board``: the board in the measurements frame F (origin on the table under the first joint's axis,
  +x forward, +y left, +z up), from the tape-measured corners O, X, Y in measurements.txt and the board's
  surface height h: x = unit(X - O), y = the part of (Y - O) orthogonal to x, z = x cross y, which must
  come out (0, 0, -1) (the board face up, its z into the table); else the measurements are refused.
* ``T_bl_F``: F is not base_link. base_link = F + (pan_x, pan_y, z_min), with (pan_x, pan_y) the first
  joint's origin in base_link and z_min the lowest point of base_link's meshes (it rests on the table);
  :func:`~.model.base_offset` computes both from the URDF.

The board's printed scale is taken from the two 100 mm bars measured with a ruler (when given). The depth
check fits a plane to the depth inside the board's outline and reports how far it sits from the PnP plane;
it is reported, never applied. Everything (inputs, intermediate transforms, checks, warnings) goes to
``<session>/fit/registration.json``.
"""
from __future__ import annotations

import datetime as _dt
import hashlib
import json
import math
from collections.abc import Callable
from pathlib import Path

import numpy as np

from .. import record
from ..session import Session
from .model import base_offset

__all__ = ["RegistrationError", "board_frame_in_F", "register_session"]

FORMAT = "be-fit-registration/0.1"


class RegistrationError(ValueError):
    """The registration cannot be made from this session (no board, unusable measurements)."""


def _unit(v):
    v = np.asarray(v, float)
    n = float(np.linalg.norm(v))
    if n == 0:
        raise RegistrationError("two measured corners coincide")
    return v / n


def _T(R=None, t=None) -> np.ndarray:
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = t
    return T


def _inv(T: np.ndarray) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    return _T(R.T, -R.T @ t)


def _rot_deg(R) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2))))


def board_scale(meas: dict, warnings: list[str]) -> dict:
    """Print scale along the board's x and y from the two measured 100 mm bars (1.0 when not measured)."""
    out = {"x": 1.0, "y": 1.0, "source": "nominal (bars not measured)"}
    h, v = meas.get("printed_100mm_bar_horizontal_mm"), meas.get("printed_100mm_bar_vertical_mm")
    if isinstance(h, float) and isinstance(v, float):
        if abs(h - 100) > 3 or abs(v - 100) > 3:
            raise RegistrationError(f"the printed 100 mm bars measure {h} and {v} mm: reprint at 100 % (Actual size)")
        out = {"x": h / 100.0, "y": v / 100.0, "source": "printed_100mm_bar_horizontal_mm / _vertical_mm"}
        if abs(h - 100) > 0.5 or abs(v - 100) > 0.5:
            warnings.append(f"the 100 mm bars measure {h:.1f} / {v:.1f} mm (more than 0.5 mm off); the board "
                            "is scaled to match")
    else:
        warnings.append("printed_100mm_bar_*_mm not given: the board is assumed printed exactly to scale")
    return out


def board_frame_in_F(meas: dict, size_m: tuple[float, float], scale: dict, warnings: list[str]
                     ) -> tuple[np.ndarray, dict]:
    """T_F_board from corners O, X, Y (mm, in F) and the board surface height, and the checks on them."""
    keys = ["corner_O_x_mm", "corner_O_y_mm", "corner_X_x_mm", "corner_X_y_mm", "corner_Y_x_mm",
            "corner_Y_y_mm", "board_surface_above_table_mm"]
    missing = [k for k in keys if not isinstance(meas.get(k), float)]
    if missing:
        raise RegistrationError(f"measurements.txt lacks {', '.join(missing)} (millimetres)")
    face = meas.get("board_face")
    if isinstance(face, str) and face.strip().lower() not in ("up", ""):
        raise RegistrationError(f"board_face = {face!r}: the board must lie face up")
    O = np.array([meas["corner_O_x_mm"], meas["corner_O_y_mm"], 0.0]) / 1000.0
    X = np.array([meas["corner_X_x_mm"], meas["corner_X_y_mm"], 0.0]) / 1000.0
    Y = np.array([meas["corner_Y_x_mm"], meas["corner_Y_y_mm"], 0.0]) / 1000.0
    h = meas["board_surface_above_table_mm"] / 1000.0
    x = _unit(X - O)
    yv = (Y - O) - ((Y - O) @ x) * x
    y = _unit(yv)
    z = np.cross(x, y)
    if z[2] > -0.999:
        raise RegistrationError(
            f"x = unit(X - O) and y (from Y - O) give z = {np.round(z, 3).tolist()}, not (0, 0, -1): with the board "
            "face up, going from X to Y about O must turn clockwise seen from above. Check the signs of the y "
            "measurements (+y is the robot's left) and which corners are X and Y.")
    T = _T(np.column_stack([x, y, z]), [O[0], O[1], h])
    bw, bh = size_m[0] * scale["x"], size_m[1] * scale["y"]
    ox, oy = float(np.linalg.norm(X - O)) * 1000, float(np.linalg.norm(Y - O)) * 1000
    ang = math.degrees(math.acos(float(np.clip(_unit(X - O) @ _unit(Y - O), -1, 1))))
    # the same three corners by a least-squares rigid fit (both edges weigh in), for comparison
    nominal = np.array([[0, 0], [bw, 0], [0, bh]])
    nominal = np.column_stack([nominal[:, 0], -nominal[:, 1]])         # board +y is clockwise of +x seen from above
    meas_xy = np.array([O[:2], X[:2], Y[:2]])
    cn, cm = nominal.mean(0), meas_xy.mean(0)
    Hm = (nominal - cn).T @ (meas_xy - cm)
    U, _, Vt = np.linalg.svd(Hm)
    Rk = (U @ Vt).T
    if np.linalg.det(Rk) < 0:
        Vt[-1] *= -1
        Rk = (U @ Vt).T
    resid = (Rk @ (nominal - cn).T).T + cm - meas_xy
    yaw_edge = math.degrees(math.atan2(x[1], x[0]))
    yaw_fit = math.degrees(math.atan2(Rk[1, 0], Rk[0, 0]))
    checks = {
        "O_to_X_mm": ox, "O_to_X_expected_mm": bw * 1000, "O_to_Y_mm": oy, "O_to_Y_expected_mm": bh * 1000,
        "angle_XOY_deg": ang, "z_axis_in_F": z.tolist(), "board_yaw_from_OX_deg": yaw_edge,
        "board_yaw_least_squares_deg": yaw_fit, "yaw_difference_deg": yaw_fit - yaw_edge,
        "least_squares_corner_residuals_mm": (np.linalg.norm(resid, axis=1) * 1000).tolist(),
        "about": "T_F_board uses x = unit(X - O) as specified; the least-squares fit of all three corners is a "
                 "cross-check: its residuals show the measuring error, and yaw_difference_deg what using both "
                 "edges would change",
    }
    if abs(ox - bw * 1000) > 3 or abs(oy - bh * 1000) > 3:
        warnings.append(f"O-X {ox:.1f} mm (expected {bw * 1000:.1f}) / O-Y {oy:.1f} mm (expected {bh * 1000:.1f}): "
                        "more than 3 mm off; measure the corners again")
    if abs(ang - 90) > 1.0:
        warnings.append(f"angle XOY is {ang:.2f} deg, not 90: the corner measurements disagree")
    return T, checks


def _pnp(cv2, obj, img, K, dist, start_obj, start_img):
    ok, rv, tv = cv2.solvePnP(start_obj, start_img, K, dist, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        ok, rv, tv = cv2.solvePnP(start_obj, start_img, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RegistrationError("PnP failed on the board corners")
    ok, rv, tv = cv2.solvePnP(obj, img, K, dist, rv, tv, useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RegistrationError("PnP (iterative) failed on the stacked board corners")
    rv, tv = cv2.solvePnPRefineLM(obj, img, K, dist, rv, tv)
    R, _ = cv2.Rodrigues(rv)
    proj, _ = cv2.projectPoints(obj, rv, tv, K, dist)
    err = np.linalg.norm(proj.reshape(-1, 2) - img.reshape(-1, 2), axis=1)
    return R, tv.reshape(3), err


def _plane_fit(P: np.ndarray, thr: float, seed: int = 0, iters: int = 300):
    """RANSAC then least squares on the inliers: (unit normal, point on the plane, inlier mask)."""
    rng = np.random.default_rng(seed)
    best = None
    n = len(P)
    for _ in range(iters):
        a, b, c = P[rng.choice(n, 3, replace=False)]
        nrm = np.cross(b - a, c - a)
        ln = np.linalg.norm(nrm)
        if ln < 1e-12:
            continue
        nrm /= ln
        inl = np.abs((P - a) @ nrm) < thr
        if best is None or inl.sum() > best.sum():
            best = inl
    Q = P[best]
    c = Q.mean(axis=0)
    _, _, Vt = np.linalg.svd(Q - c, full_matrices=False)
    nrm = Vt[-1]
    inl = np.abs((P - c) @ nrm) < thr
    Q = P[inl]
    c = Q.mean(axis=0)
    _, _, Vt = np.linalg.svd(Q - c, full_matrices=False)
    return Vt[-1], c, inl


def depth_check(sess: Session, t0: float, t1: float, T_cam_board: np.ndarray, size_m, inset_m: float = 0.012,
                max_frames: int = 60) -> dict:
    """Plane fitted to the depth (median over the window) inside the board's outline, against the PnP plane."""
    import cv2

    from .background import median_nonzero
    from .fit import Camera

    idx = sess.depth_indices(t0, t1)
    if not idx.size:
        return {"status": "no depth frames in the window"}
    pick = idx[np.linspace(0, idx.size - 1, min(max_frames, idx.size)).astype(int)]
    med, _iqr, _ = median_nonzero(np.stack([sess.depth_frame(int(i)) for i in pick]), 0.5)
    K, dist = sess.intrinsics("640x360")
    cam = Camera(K, dist, (med.shape[1], med.shape[0]), np.eye(4))
    bw, bh = size_m
    corners = np.array([[inset_m, inset_m, 0], [bw - inset_m, inset_m, 0], [bw - inset_m, bh - inset_m, 0],
                        [inset_m, bh - inset_m, 0]])
    Pc = corners @ T_cam_board[:3, :3].T + T_cam_board[:3, 3]
    u, v = cam.project_cam(Pc)
    mask = np.zeros(med.shape, np.uint8)
    cv2.fillConvexPoly(mask, np.round(np.column_stack([u, v])).astype(np.int32), 1)
    sel = np.flatnonzero((mask.ravel() > 0) & (med.ravel() > 0))
    if sel.size < 50:
        return {"status": f"only {sel.size} depth pixels inside the board's outline", "pixels": int(sel.size)}
    z = med.ravel()[sel].astype(np.float64) / 1000.0
    P = cam.rays[sel] * z[:, None]
    n_pnp = T_cam_board[:3, 2]                       # board z (into the table), camera frame
    p_pnp = T_cam_board[:3, 3]
    rays = cam.rays[sel]
    z_pnp = (n_pnp @ p_pnp) / (rays @ n_pnp)
    along = (z - z_pnp) * 1000.0
    signed = ((P - p_pnp) @ n_pnp) * 1000.0          # + : the depth surface lies behind (below) the PnP plane
    nrm, _c, inl = _plane_fit(P, thr=0.005)
    if nrm @ n_pnp < 0:
        nrm = -nrm
    ang = math.degrees(math.acos(float(np.clip(nrm @ n_pnp, -1, 1))))
    return {"status": "ok", "frames": int(pick.size), "pixels": int(sel.size), "plane_inliers": int(inl.sum()),
            "depth_minus_pnp_along_ray_mm": {"mean": float(along.mean()), "median": float(np.median(along)),
                                             "p90_abs": float(np.percentile(np.abs(along), 90))},
            "signed_offset_from_pnp_plane_mm": {"mean": float(signed[inl].mean()), "median": float(np.median(signed))},
            "angle_between_planes_deg": ang,
            "about": "depth plane (RANSAC + least squares over the median depth inside the board outline, inset "
                     f"{inset_m * 1000:.0f} mm) against the PnP board plane; positive offsets mean the depth "
                     "reads farther than the board. Reported only; the registration does not use depth."}


def register_session(session, urdf, *, board_spec: dict | None = None, out=None, window_s=(1.0, 9.0),
                     max_frames: int = 100, min_corners: int = 6, measurements: dict | None = None,
                     say: Callable[[str], None] = print) -> dict:
    """Register the camera in base_link and write ``<out or session/fit>/registration.json``."""
    import cv2

    from ...kinematics import URDFModel

    sess = session if isinstance(session, Session) else Session(session)
    warnings: list[str] = []
    urdf_model = urdf if isinstance(urdf, URDFModel) else URDFModel.from_file(urdf)
    urdf_path = None if isinstance(urdf, URDFModel) else Path(urdf).resolve()
    spec = dict(board_spec) if board_spec else sess.board_spec()
    t_ev = sess.event_time("board_placed")
    if t_ev is None:
        raise RegistrationError("no board_placed event in events.csv")
    nxt = sess.next_event_after(t_ev)
    t0 = t_ev + window_s[0]
    t1 = t_ev + window_s[1]
    if nxt is not None and nxt[0] < t1:
        t1 = nxt[0]
    cidx = sess.color_indices(t0, t1)
    if not cidx.size:
        raise RegistrationError(f"no colour frames between +{window_s[0]:g} s and +{window_s[1]:g} s after board_placed"
                                + (f" (the next event, {nxt[1]}, comes at +{nxt[0] - t_ev:.1f} s)" if nxt else ""))
    picks = cidx[np.linspace(0, cidx.size - 1, min(max_frames, cidx.size)).astype(int)]
    board = record.charuco_board(cv2, spec)
    detector = cv2.aruco.CharucoDetector(board)
    scale = board_scale(measurements if measurements is not None else sess.measurements(), warnings)
    S = np.diag([scale["x"], scale["y"], 1.0])
    views = []
    for i in picks:
        img = sess.color_frame(int(i))
        if img is None:
            continue
        r = record.detect_charuco(cv2, board, img, detector)
        if r is not None and len(r[0]) >= min_corners:
            views.append((int(i), r[0] @ S.T, r[1], r[2]))
    if not views:
        raise RegistrationError(f"the board was not found (>= {min_corners} corners) in {picks.size} colour frames "
                                "after board_placed")
    K, dist = sess.intrinsics("1280x720")
    obj = np.concatenate([v[1] for v in views]).astype(np.float64)
    img = np.concatenate([v[2] for v in views]).astype(np.float64)
    best = max(views, key=lambda v: len(v[1]))
    R, t, err = _pnp(cv2, obj, img, K, dist, best[1], best[2])
    T_cam_board = _T(R, t)
    per_frame, k0 = [], 0
    for v in views:
        e = err[k0:k0 + len(v[1])]
        k0 += len(v[1])
        per_frame.append({"color_index": v[0], "corners": len(v[1]), "rms_px": float(np.sqrt(np.mean(e ** 2)))})
    rms = float(np.sqrt(np.mean(err ** 2)))
    if rms > 1.0:
        warnings.append(f"PnP reprojection RMS {rms:.2f} px: the board or the camera moved, or the intrinsics are off")
    meas = measurements if measurements is not None else sess.measurements()
    size_m = (spec["squares_x"] * spec["square_length_m"], spec["squares_y"] * spec["square_length_m"])
    T_F_board, mchecks = board_frame_in_F(meas, size_m, scale, warnings)
    off = base_offset(urdf_model)
    T_bl_F = off.T_root_F()
    T_bl_cam = T_bl_F @ T_F_board @ _inv(T_cam_board)
    dcheck = depth_check(sess, t0, t1, T_cam_board, (size_m[0] * scale["x"], size_m[1] * scale["y"]))
    if dcheck.get("status") == "ok":
        mo = dcheck["signed_offset_from_pnp_plane_mm"]["mean"]
        if abs(mo) > 10 or dcheck["angle_between_planes_deg"] > 3:
            warnings.append(f"depth and the board disagree: {mo:+.1f} mm, {dcheck['angle_between_planes_deg']:.1f} "
                            "deg between the planes (depth scale, alignment, or the intrinsics)")
    cam_pos = T_bl_cam[:3, 3]
    look = T_bl_cam[:3, 2]
    tape = {}
    dist_tape, h_tape = meas.get("oakd_distance_to_base_centre_mm"), meas.get("oakd_height_above_table_mm")
    pos_F = cam_pos - T_bl_F[:3, 3]
    if isinstance(dist_tape, float):
        tape["distance_to_base_centre_mm"] = {"tape": dist_tape, "registration": float(np.linalg.norm(pos_F) * 1000)}
    if isinstance(h_tape, float):
        tape["height_above_table_mm"] = {"tape": h_tape, "registration": float(pos_F[2] * 1000)}
    doc = {
        "format": FORMAT,
        "created": _dt.datetime.now(_dt.timezone.utc).isoformat(timespec="seconds"),
        "session": str(sess.path),
        "T_bl_cam": T_bl_cam.tolist(),
        "T_cam_bl": _inv(T_bl_cam).tolist(),
        "camera_in_base_link": {
            "position_m": cam_pos.tolist(), "optical_axis": look.tolist(),
            "tilt_down_deg": float(math.degrees(math.asin(max(-1.0, min(1.0, -look[2]))))),
            "distance_to_pan_axis_m": float(math.hypot(cam_pos[0] - off.pan_x, cam_pos[1] - off.pan_y)),
            "height_above_table_m": float(pos_F[2])},
        "chain": "T_bl_cam = T_bl_F . T_F_board . inverse(T_cam_board)",
        "steps": {"T_cam_board": T_cam_board.tolist(), "T_F_board": T_F_board.tolist(), "T_bl_F": T_bl_F.tolist(),
                  "base_offset": {"pan_x_m": off.pan_x, "pan_y_m": off.pan_y, "z_min_m": off.z_min, "joint": off.joint,
                                  "explanation": off.explanation}},
        "inputs": {
            "board_spec": spec, "board_scale": scale,
            "measurements": {k: meas.get(k) for k in sorted(meas)},
            "urdf": {"path": str(urdf_path) if urdf_path else None,
                     "sha256": hashlib.sha256(urdf_path.read_bytes()).hexdigest() if urdf_path else None},
            "intrinsics_1280x720": {"K": K.tolist(), "dist": np.asarray(dist).tolist()},
            "window": {"board_placed_t_mono_s": t_ev, "from_s": window_s[0], "to_s": window_s[1], "t0": t0, "t1": t1,
                       "ended_by": (nxt[1] if nxt is not None and nxt[0] <= t_ev + window_s[1] else None)},
            "frames_searched": int(picks.size)},
        "checks": {
            "pnp": {"frames_with_board": len(views), "corner_observations": len(obj),
                    "distinct_corners": len({int(c) for v in views for c in v[3]}),
                    "corners_possible": int(board.getChessboardCorners().shape[0]),
                    "reprojection_rms_px": rms, "reprojection_max_px": float(err.max()),
                    "camera_to_board_m": float(np.linalg.norm(t)),
                    "per_frame_rms_px": {"min": min(f["rms_px"] for f in per_frame),
                                         "max": max(f["rms_px"] for f in per_frame)}},
            "measurements": mchecks,
            "depth": dcheck,
            "camera_vs_tape": tape},
        "warnings": warnings,
    }
    out = Path(out) if out else sess.path / "fit"
    out.mkdir(parents=True, exist_ok=True)
    (out / "registration.json").write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
    say(f"registration: {len(views)} frames, {len(obj)} corner observations, reprojection {rms:.2f} px; camera "
        f"{np.linalg.norm(t):.3f} m from the board, {doc['camera_in_base_link']['distance_to_pan_axis_m']:.3f} m from the "
        f"{off.joint} axis, {pos_F[2]:.3f} m above the table")
    say(f"  board: O-X {mchecks['O_to_X_mm']:.1f} mm (expect {mchecks['O_to_X_expected_mm']:.1f}), O-Y "
        f"{mchecks['O_to_Y_mm']:.1f} mm (expect {mchecks['O_to_Y_expected_mm']:.1f}), angle XOY {mchecks['angle_XOY_deg']:.2f} deg")
    if dcheck.get("status") == "ok":
        say(f"  depth vs board: {dcheck['signed_offset_from_pnp_plane_mm']['mean']:+.1f} mm, planes "
            f"{dcheck['angle_between_planes_deg']:.2f} deg apart ({dcheck['pixels']} px)")
    else:
        say(f"  depth vs board: {dcheck.get('status')}")
    say("  " + off.explanation)
    for w in warnings:
        say(f"  WARNING: {w}")
    say(f"  -> {out / 'registration.json'}")
    return doc
