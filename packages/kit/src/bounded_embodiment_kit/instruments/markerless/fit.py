# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""The per-frame articulated fit and the tracker (`be fit run`).

For each depth frame of a recording, find the joint values q of the URDF (shoulder_pan ... gripper for the
SO-101) that make the posed meshes agree with what the camera saw, and report the envelope's points at
that pose. The rules that make this an instrument:

* **Independence.** The inputs are the session's camera files, the URDF and its meshes, the envelope, the
  registration and the background. Nothing else is opened: no encoder values, commands, harness logs or
  probe files, even when they sit in the session directory. Frame k starts from the camera's own estimate
  at frame k-1; the first frame, and every frame after an invalid one, starts from a global search.
* **No smoothing.** Nothing pulls an estimate toward the previous one; the previous estimate is only the
  starting point of the optimiser. Reported positions are the per-frame fit, unfiltered.
* **Honest validity.** Every row says valid 0/1 and carries the fit residual; a doubtful frame or point
  is invalid rather than guessed. Invalid rows have empty coordinates.
* **Deterministic.** Fixed sampling (seeded once per session), fixed grids, fixed iteration caps.

The cost, for the observed points p_i (depth back-projected into base_link, kept if valid, >= 0.3 m from the
camera, inside the URDF-derived ROI, >= 8 mm above the table and nearer than the static background; at most
4000, chosen by a fixed per-pixel priority):

(a) data -> model: point-to-plane distance from p_i to the nearest dense surface sample (1.5 mm apart) of the
    nearest link, n . (p - m), Huber (delta 5 mm). Association is a gather from per-link voxel fields (see
    :mod:`.model`); an exact k-d tree query replaces it in the polish of global-search candidates. Each
    point's weight ramps from 1 at 15 mm to 0 at the 25 mm gate (40 mm in the first step after a global
    search), so a point far from the model costs a constant and nothing jumps when one crosses the gate.
(b) model -> data, free space: the moving links' samples 9 mm apart that face the camera, z-buffered at
    quarter resolution, against the observed depth eroded by a 5x5 minimum. Where the observed depth is
    farther than the sample by > 20 mm (ramping in over 10 mm), the camera saw through the model there; the
    sample costs (its image distance to the nearest kept data pixel, in metres at its depth, capped at
    20 mm)^2 / 2, which pulls it toward the observed arm. Pixels with no depth, or nearer than the model
    (occlusion), add nothing.

Levenberg-Marquardt with analytic Jacobians, re-associating at every evaluation, at most 15 steps per frame.
It has converged when the undamped Gauss-Newton step would move no link origin more than 0.1 mm.

Global search (the first frame, and after an invalid one): a 15 deg grid over the first three joints scored by
render-and-compare with the links they move; for the best ten, a grid over the distal joints; the robust cost
for the best of those; LM (on 1500 of the points) from the best six and from the best in each quarter of the
wrist_flex and wrist_roll ranges; 1-D scans over wrist_flex, wrist_roll and the gripper; wrist_roll +-180 deg
(the closed gripper is nearly symmetric about it); then the best fit and the strongest distinct alternative
are polished on all the points.

Validity, per frame: converged; >= 300 inliers (within 10 mm of the model) and >= half the points; inlier
point-to-plane RMS <= 6 mm; <= 10 % of the visible model samples seen through; no point moved more than
1.5 m/s x dt + 30 mm since the last valid frame. Per point, also: its formal 1-sigma from the fit's normal
equations <= 2 mm (the joints it depends on are seen), and no competing fit within 5 % of the cost puts it
more than 3 mm away (that doubt holds until a fresh global search, every 15 frames, settles it).
"""
from __future__ import annotations

import csv
import dataclasses
import hashlib
import json
import math
import platform
import sys
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ..session import Session
from . import mesh
from .model import ArmModel

__all__ = [
    "SAMPLES_HEADER",
    "Camera",
    "FitConfig",
    "FitError",
    "FrameResult",
    "Observation",
    "Tracker",
    "evaluate",
    "fit_session",
    "global_search",
    "load_registration",
    "observe",
    "solve",
]

SAMPLES_HEADER = ("t_s", "point", "x_m", "y_m", "z_m", "valid", "residual_m")
FIT_VERSION = "0.1.0"
STATUS = ("TRACK", "REINIT", "LOST")


class FitError(RuntimeError):
    """A problem with the inputs the operator can fix; the CLI prints it without a traceback."""


@dataclass(frozen=True)
class FitConfig:
    """Every threshold of the fit in one place. Lengths in metres, angles in radians unless named _deg."""

    # observation
    max_points: int = 4000              # observed points kept per frame (fixed pseudo-random pixel priority)
    min_depth_m: float = 0.30           # nearer than this is not trusted (the OAK-D gives none < ~0.35 m)
    above_table_m: float = 0.008        # points within this of the table plane are the table
    fg_min_m: float = 0.015             # foreground: nearer than the background by > max(this, k sigma_z)
    fg_sigma_k: float = 3.0
    sigma_z0_m: float = 0.0015          # depth noise model: sigma_z(z) = sigma_z0 (z / sigma_z_ref)^2
    sigma_z_ref_m: float = 0.6
    depth_distorted: bool = True        # depth aligned to colour follows the colour lens model
    # cost
    huber_m: float = 0.005
    gate_m: float = 0.025               # data points farther than this from the model are gated out ...
    gate_full_frac: float = 0.6         # ... with full weight up to this fraction of the gate, ramping to 0
    gate_first_m: float = 0.040         # the gate in the first iteration after a global search
    facing_full: float = 2.0            # association weight 1 while the model normal . view <= this, ramping
    facing_zero: float = 3.0            # to 0 at facing_zero (n.v <= 1 always: off by default; a filter at
    #                                     0.1..0.5 cost accuracy on synthetic data: p95 1.8 vs 0.8 mm)
    fs_margin_m: float = 0.020          # free space: observed depth farther than the model by more than this
    fs_erode_px: int = 2                # ... everywhere within this many pixels (tolerates small misregistration)
    fs_ramp_m: float = 0.010            # ... the residual ramps in over this much more
    fs_cap_m: float = 0.020             # cap of one free-space residual
    fs_weight: float = 1.0
    # optimiser
    max_iterations: int = 15            # per frame when tracking ...
    search_iterations: int = 40         # ... and from each global-search candidate (rare, starts far away)
    exact_iterations: int = 0           # of which the last ones use exact nearest-sample association (1 on
    #                                     synthetic data: worst-point p95 0.79 vs 0.86 mm, 130 vs 80 ms/frame)
    step_tol_m: float = 1e-4            # converged when the Gauss-Newton step moves no link origin more than this
    #                                     (synthetic, 120 frames: 0.25 mm -> worst-point p95 1.01 mm at 72 ms/frame;
    #                                     0.1 mm -> 0.88 mm at 94 ms; 0.05 mm -> 0.88 mm at 113 ms)
    stall_tol_m: float = 0.001          # ... or, when no damped step lowers the cost, when it is below this
    polish_tol_m: float = 0.001         # ... and the exact-association polish moves none more than this
    search_exact_iterations: int = 4    # exact-association polish of each global-search candidate
    lm_lambda0: float = 1e-3
    lm_retries: int = 5
    candidate_margin_m: float = 0.010   # per-link candidate points are recomputed after this much motion
    # validity (per frame unless named per point)
    inlier_m: float = 0.010             # an inlier is within this of the model surface
    min_inliers: int = 300
    min_inlier_frac: float = 0.5
    max_rms_m: float = 0.006            # point-to-plane RMS of the inliers
    max_free_space_frac: float = 0.10   # visible model samples the camera sees through
    max_point_sigma_m: float = 0.002    # per point: formal 1-sigma from the fit's normal equations
    jump_speed_mps: float = 1.5         # a point may move at most this x dt + jump_margin between valid frames
    jump_margin_m: float = 0.030
    # global search
    search_proximal: int = 3            # the first joints, searched on a grid with the links they move
    search_step_deg: float = 15.0
    search_distal_steps_deg: tuple[float, ...] = (15.0, 45.0, 40.0)   # the remaining joints, in order
    search_top_proximal: int = 10
    search_top_full: int = 24
    search_refine: int = 6
    search_points: int = 1500           # candidate fits use this many of the frame's points; winners all of them
    search_support_m: float = 0.030
    search_violation_m: float = 0.030
    scan_joints: tuple[str, ...] = ("wrist_flex", "wrist_roll", "gripper")   # re-init: 1-D scans of these
    diverse_joints: tuple[str, ...] = ("wrist_flex", "wrist_roll")   # re-init: refine the best candidate in
    #                                     every quarter of these joints' ranges, not only the best few overall
    scan_step_deg: float = 10.0
    scan_starts: int = 2                # LM from this many of each scan's best local minima
    flip_joints: tuple[str, ...] = ("wrist_roll",)   # near-symmetric (the closed gripper about its roll axis):
    #                                     re-inits refine one candidate per quarter of the range, and tracking
    flip_check_every: int = 15          # tries the joint turned 180 deg every this many frames (0: never)
    ambiguity_ratio: float = 1.05       # a competing fit within this factor of the cost ...
    ambiguity_m: float = 0.003          # ... that puts a point more than this away makes the point invalid,
    recheck_every: int = 15             # until a fresh global search (every this many frames) settles it
    seed: int = 0

    def as_dict(self) -> dict:
        return {k: (list(v) if isinstance(v, tuple) else v) for k, v in dataclasses.asdict(self).items()}


# ----- camera ---------------------------------------------------------------------------------------
def _dist14(dist) -> np.ndarray:
    d = np.zeros(14)
    a = np.asarray(dist, dtype=np.float64).ravel()[:14]
    d[: a.size] = a
    return d


def distort_normalized(x: np.ndarray, y: np.ndarray, dist) -> tuple[np.ndarray, np.ndarray]:
    """OpenCV's lens model (k1 k2 p1 p2 k3 k4 k5 k6 s1 s2 s3 s4; a tilted sensor is not handled here)."""
    k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4, _tx, _ty = _dist14(dist)
    r2 = x * x + y * y
    r4 = r2 * r2
    rad = (1 + k1 * r2 + k2 * r4 + k3 * r4 * r2) / (1 + k4 * r2 + k5 * r4 + k6 * r4 * r2)
    xd = x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x) + s1 * r2 + s2 * r4
    yd = y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y + s3 * r2 + s4 * r4
    return xd, yd


def undistort_normalized(xd: np.ndarray, yd: np.ndarray, dist, iters: int = 40) -> tuple[np.ndarray, np.ndarray]:
    """Inverse of :func:`distort_normalized` by fixed-point iteration (as OpenCV does, more iterations)."""
    k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4, _tx, _ty = _dist14(dist)
    x, y = xd.copy(), yd.copy()
    for _ in range(iters):
        r2 = x * x + y * y
        r4 = r2 * r2
        rad = (1 + k1 * r2 + k2 * r4 + k3 * r4 * r2) / (1 + k4 * r2 + k5 * r4 + k6 * r4 * r2)
        dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x) + s1 * r2 + s2 * r4
        dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y + s3 * r2 + s4 * r4
        x = (xd - dx) / rad
        y = (yd - dy) / rad
    return x, y


class Camera:
    """A pinhole camera with OpenCV distortion, placed in base_link by ``T_bl_cam`` (camera -> base)."""

    def __init__(self, K, dist, size: tuple[int, int], T_bl_cam, *, distorted: bool = True, seed: int = 0):
        self.K = np.asarray(K, dtype=np.float64)
        self.dist = _dist14(dist)
        self.w, self.h = int(size[0]), int(size[1])
        self.fx, self.fy, self.cx, self.cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        self.T_bl_cam = np.asarray(T_bl_cam, dtype=np.float64)
        self.R = self.T_bl_cam[:3, :3]               # base <- camera
        self.t = self.T_bl_cam[:3, 3]                # camera centre in base_link
        self.distorted = bool(distorted and np.any(self.dist != 0))
        if self.distorted and np.any(self.dist[12:] != 0):
            raise FitError("a tilted-sensor distortion model (tau_x, tau_y) is not supported")
        u, v = np.meshgrid(np.arange(self.w, dtype=np.float64), np.arange(self.h, dtype=np.float64))
        xn, yn = (u.ravel() - self.cx) / self.fx, (v.ravel() - self.cy) / self.fy
        if self.distorted:
            xn, yn = undistort_normalized(xn, yn, self.dist)
        self.rays = np.column_stack([xn, yn, np.ones_like(xn)])       # (h*w, 3), z = 1
        self.priority = np.random.default_rng(seed).permutation(self.w * self.h)
        self._roi = (None, None, None)

    def roi_interval(self, model: ArmModel, above_table: float, min_depth: float) -> tuple[np.ndarray, np.ndarray]:
        """Per pixel, the camera-depth interval [lo, hi] (m) over which its ray is inside the model's ROI
        (the cylinder about the first joint's axis, above the table by ``above_table``, below the top), and
        at least ``min_depth`` from the camera. Empty intervals have lo > hi. Cached per model."""
        key = (id(model), above_table, min_depth)
        if self._roi[0] == key:
            return self._roi[1], self._roi[2]
        d = self.rays @ self.R.T                     # base-frame displacement per metre of camera depth
        ax, ay = model.roi["axis_xy_m"]
        r = model.roi["radius_m"]
        cx_, cy_ = self.t[0] - ax, self.t[1] - ay
        A = d[:, 0] ** 2 + d[:, 1] ** 2
        B = cx_ * d[:, 0] + cy_ * d[:, 1]
        C = cx_ * cx_ + cy_ * cy_ - r * r
        disc = B * B - A * C
        lo = np.full(len(d), -np.inf)
        hi = np.full(len(d), np.inf)
        par = A < 1e-15
        with np.errstate(invalid="ignore", divide="ignore"):
            s = np.sqrt(np.maximum(disc, 0.0))
            lo = np.where(par, np.where(C <= 0, -np.inf, np.inf), np.where(disc >= 0, (-B - s) / A, np.inf))
            hi = np.where(par, np.where(C <= 0, np.inf, -np.inf), np.where(disc >= 0, (-B + s) / A, -np.inf))
            zb, zt = model.table_z + above_table - self.t[2], model.roi["z_top_m"] - self.t[2]
            dz = d[:, 2]
            a1, a2 = zb / dz, zt / dz
            s_lo = np.where(dz > 0, a1, np.where(dz < 0, a2, np.where((zb <= 0) & (zt >= 0), -np.inf, np.inf)))
            s_hi = np.where(dz > 0, a2, np.where(dz < 0, a1, np.where((zb <= 0) & (zt >= 0), np.inf, -np.inf)))
        lo = np.maximum(np.maximum(lo, s_lo), min_depth).astype(np.float32)
        hi = np.minimum(hi, s_hi).astype(np.float32)
        self._roi = (key, lo, hi)
        return lo, hi

    def to_cam(self, P: np.ndarray) -> np.ndarray:
        return (P - self.t) @ self.R

    def project_cam(self, Pc: np.ndarray, distorted: bool | None = None) -> tuple[np.ndarray, np.ndarray]:
        """Pixel coordinates (u, v) of camera-frame points (z > 0)."""
        x, y = Pc[:, 0] / Pc[:, 2], Pc[:, 1] / Pc[:, 2]
        if self.distorted if distorted is None else distorted:
            x, y = distort_normalized(x, y, self.dist)
        return self.fx * x + self.cx, self.fy * y + self.cy

    def scaled(self, K, size) -> Camera:
        """The same camera pose with other intrinsics (the colour stream at 1280x720)."""
        c = object.__new__(Camera)
        c.K = np.asarray(K, dtype=np.float64)
        c.dist = self.dist
        c.w, c.h = int(size[0]), int(size[1])
        c.fx, c.fy, c.cx, c.cy = c.K[0, 0], c.K[1, 1], c.K[0, 2], c.K[1, 2]
        c.T_bl_cam, c.R, c.t, c.distorted = self.T_bl_cam, self.R, self.t, self.distorted
        return c


def sigma_z(z, cfg: FitConfig):
    return cfg.sigma_z0_m * (np.asarray(z) / cfg.sigma_z_ref_m) ** 2


# ----- observation ------------------------------------------------------------------------------------
@dataclass
class Observation:
    """What one depth frame says, ready for the cost."""

    points: np.ndarray                 # (n, 3) observed points kept for the fit, base_link
    pixels: np.ndarray                 # (n,) flat pixel index of each
    view: np.ndarray                   # (n, 3) unit vectors camera -> point, base_link
    depth: np.ndarray                  # (h*w,) observed depth, m, 0 = none (also < min_depth)
    depth_min3: np.ndarray             # (h*w,) nearest valid depth within fs_erode_px (5x5), 0 = none
    data_mask: np.ndarray              # (h*w,) bool: pixels that passed every filter (before subsampling)
    dt: np.ndarray                     # (h, w) image distance (px) to the nearest data pixel
    n_valid: int                       # pixels with depth
    n_candidates: int                  # pixels that passed the filters

    @property
    def n_points(self) -> int:
        return len(self.points)

    def subset(self, n: int) -> Observation:
        """The same frame with about n of its points (an even stride through them): for the global search's
        many candidate fits, before the winners are polished on every point."""
        if self.n_points <= n:
            return self
        sel = np.linspace(0, self.n_points - 1, n).astype(np.int64)
        return dataclasses.replace(self, points=self.points[sel], pixels=self.pixels[sel], view=self.view[sel])


def _cv2():
    try:
        import cv2

        return cv2
    except ImportError:                                       # pragma: no cover - cv2 is in the fit extra
        return None


def observe(depth_mm: np.ndarray, cam: Camera, model: ArmModel, cfg: FitConfig,
            background_mm: np.ndarray | None = None) -> Observation:
    """Back-project a depth frame into base_link and keep the points that can be the arm: valid depth,
    >= min_depth from the camera, inside the URDF-derived ROI and above the table (a per-pixel depth
    interval precomputed from the camera pose), and nearer than the background by
    max(fg_min, fg_sigma_k sigma_z(z)) or where the background is unknown."""
    h, w = cam.h, cam.w
    if depth_mm.shape != (h, w):
        raise FitError(f"depth frame is {depth_mm.shape[1]}x{depth_mm.shape[0]}, the camera model {w}x{h}")
    raw = depth_mm.ravel()
    z = raw.astype(np.float32) * np.float32(1e-3)
    valid = (raw > 0) & (z >= cfg.min_depth_m)
    lo, hi = cam.roi_interval(model, cfg.above_table_m, cfg.min_depth_m)
    keep = valid & (z >= lo) & (z <= hi)
    if background_mm is not None:
        B = background_mm.ravel().astype(np.float32) * np.float32(1e-3)
        thr = np.maximum(np.float32(cfg.fg_min_m),
                         np.float32(cfg.fg_sigma_k * cfg.sigma_z0_m) * (z / np.float32(cfg.sigma_z_ref_m)) ** 2)
        keep &= (B <= 0) | (z < B - thr)
    kidx = np.flatnonzero(keep)
    n_cand = int(kidx.size)
    if n_cand > cfg.max_points:
        sel = np.sort(np.argpartition(cam.priority[kidx], cfg.max_points)[: cfg.max_points])
        kidx = kidx[sel]
    P = (cam.rays[kidx] * z[kidx, None].astype(np.float64)) @ cam.R.T + cam.t
    view = P - cam.t
    view /= np.linalg.norm(view, axis=1, keepdims=True)
    D = np.where(valid, z, np.float32(0.0))
    cv2 = _cv2()
    big = np.float32(1e3)
    D2 = np.where(valid, z, big).reshape(h, w)
    ksz = 2 * max(int(cfg.fs_erode_px), 0) + 1
    if cv2 is not None:
        Dm = cv2.erode(D2, np.ones((ksz, ksz), np.uint8))
    else:                                                     # pragma: no cover
        from scipy import ndimage

        Dm = ndimage.minimum_filter(D2, size=ksz)
    Dm = np.where(Dm >= big, np.float32(0.0), Dm).ravel()
    if n_cand:
        not_data = (~keep).reshape(h, w).astype(np.uint8)
        if cv2 is not None:
            dt = cv2.distanceTransform(not_data, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
        else:                                                 # pragma: no cover
            from scipy import ndimage

            dt = ndimage.distance_transform_edt(not_data).astype(np.float32)
    else:
        dt = np.full((h, w), 1e4, dtype=np.float32)
    return Observation(P, kidx, view, D, Dm, keep, dt, int(valid.sum()), n_cand)


# ----- the cost ----------------------------------------------------------------------------------------
@dataclass
class Evaluation:
    cost: float
    H: np.ndarray | None               # normal matrix (data + free space)
    g: np.ndarray | None               # gradient
    H_data: np.ndarray | None
    associated: np.ndarray             # (n,) bool: within the gate and facing the camera
    link: np.ndarray                   # (n,) index into model.links, -1 = none
    r: np.ndarray                      # (n,) point-to-plane residual, m
    d: np.ndarray                      # (n,) distance to the associated sample, m
    inliers: int
    rms: float                         # point-to-plane RMS of the inliers, m
    fs_violations: int
    fs_considered: int
    fs_frac: float
    fs_pixels: np.ndarray = field(default=None, repr=False)   # (k, 2) u, v of violating samples (debug)


def _skew(a) -> np.ndarray:
    """S with S @ v = a x v."""
    return np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])


def _skews(A: np.ndarray) -> np.ndarray:
    """(3, 3 n) [S_1^T | ... | S_n^T], so that (V @ _skews(A))[i, 3 j:3 j + 3] = a_j x v_i."""
    return np.concatenate([_skew(a).T for a in A], axis=1)


def _huber(r, delta):
    a = np.abs(r)
    return np.where(a <= delta, 0.5 * r * r, delta * (a - 0.5 * delta))


def _bilinear(img: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
    h, w = img.shape
    u = np.clip(u, 0, w - 1.000001)
    v = np.clip(v, 0, h - 1.000001)
    u0, v0 = np.floor(u).astype(np.int64), np.floor(v).astype(np.int64)
    fu, fv = u - u0, v - v0
    a = img[v0, u0] * (1 - fu) + img[v0, u0 + 1] * fu
    b = img[v0 + 1, u0] * (1 - fu) + img[v0 + 1, u0 + 1] * fu
    return a * (1 - fv) + b * fv


def link_candidates(model: ArmModel, P: np.ndarray, poses, extra: float = 0.0) -> dict[int, np.ndarray]:
    """``{link index: indices of the points inside that link's field box grown by extra}`` at these poses.
    A solve computes this once with a margin and recomputes it only after the model has moved that far."""
    out = {}
    for li, link in enumerate(model.links):
        T = poses[link]
        f = model.fields[link]
        PL = (P - T[:3, 3]) @ T[:3, :3]
        lo, hi = f.lo - extra, f.hi + extra
        idx = np.flatnonzero((PL[:, 0] >= lo[0]) & (PL[:, 0] < hi[0]) & (PL[:, 1] >= lo[1]) & (PL[:, 1] < hi[1])
                             & (PL[:, 2] >= lo[2]) & (PL[:, 2] < hi[2]))
        if idx.size:
            out[li] = idx
    return out


def evaluate(model: ArmModel, cam: Camera, obs: Observation, q: np.ndarray, cfg: FitConfig, *,
             gate: float, exact: bool, jac: bool = True, poses=None, cand=None) -> Evaluation:
    """The robust cost at q (and, with ``jac``, its Gauss-Newton normal equations).

    Data term: each observed point is associated with the nearest surface sample of the link whose sample
    is nearest (per-link fields; with ``exact``, an exact k-d tree query on that link), residual
    n . (p - m) in the link frame, Huber; its weight ramps to 0 at the gate (and on surface facing away
    from the camera, if ``facing_zero`` < 1), and a point with weight 0 costs the constant rho(gate).
    Free space: the moving links' "fs" samples that face the camera and survive a quarter-resolution
    z-buffer are compared with the observed depth's (2 fs_erode_px + 1)^2 minimum; where it is farther than
    the sample by > fs_margin (ramping in over fs_ramp) the sample costs
    (fs_weight x min(image distance to the data x z / f, fs_cap))^2 / 2.
    """
    poses = poses if poses is not None else model.poses(q)
    P = obs.points
    n = len(P)
    nj = model.nj
    best_d = np.full(n, np.inf)
    best_l = np.full(n, -1, dtype=np.int64)
    r = np.zeros(n)
    nl = np.zeros((n, 3))                                       # associated normal, link frame
    local = {}
    for li, idx in (cand if cand is not None else link_candidates(model, P, poses)).items():
        link = model.links[li]
        T = poses[link]
        f = model.fields[link]
        PL = (P[idx] - T[:3, 3]) @ T[:3, :3]
        k = f.lookup(PL)
        ok = np.flatnonzero(k >= 0)
        pn = f.pn[k[ok]]
        PLo = PL[ok]
        diff = PLo - pn[:, :3]
        d = np.sqrt(np.einsum("ij,ij->i", diff, diff))
        pts = idx[ok]
        better = d < best_d[pts]
        sel = pts[better]
        best_d[sel], best_l[sel] = d[better], li
        r[sel] = np.einsum("ij,ij->i", diff[better], pn[better, 3:])
        nl[sel] = pn[better, 3:]
        if exact:
            local[li] = (idx, PL)
    if exact:                                                   # exact nearest sample on the chosen link
        for li, (idx, PL) in local.items():
            mine = best_l[idx] == li
            if not mine.any():
                continue
            sel = idx[mine]
            f = model.fields[model.links[li]]
            dd, kk = model.tree(model.links[li]).query(PL[mine], k=1)
            pn = f.pn[kk]
            best_d[sel] = dd
            r[sel] = np.einsum("ij,ij->i", PL[mine] - pn[:, :3], pn[:, 3:])
            nl[sel] = pn[:, 3:]
    nb = np.zeros((n, 3))                                       # the same normal, base frame
    for li in np.unique(best_l[best_l >= 0]):
        sel = best_l == li
        nb[sel] = nl[sel] @ poses[model.links[li]][:3, :3].T
    # association weight: 1 near the model and on surface facing the camera, ramping to 0 at the gate and
    # on surface facing away; cost = rho_c + w (rho(r) - rho_c), so the cost has no jumps when a point
    # crosses the gate or the silhouette
    nv = np.einsum("ij,ij->i", nb, obs.view)
    w_face = np.clip((cfg.facing_zero - nv) / (cfg.facing_zero - cfg.facing_full), 0.0, 1.0)
    g_full = cfg.gate_full_frac * gate
    w_gate = np.clip((gate - best_d) / (gate - g_full), 0.0, 1.0)
    wa = np.where(best_l >= 0, w_face * w_gate, 0.0)
    assoc = wa > 0
    rho_c = 0.5 * gate * gate if gate <= cfg.huber_m else cfg.huber_m * (gate - 0.5 * cfg.huber_m)
    cost = rho_c * n + float((wa[assoc] * (_huber(r[assoc], cfg.huber_m) - rho_c)).sum())
    inl = (best_l >= 0) & (best_d <= cfg.inlier_m) & (nv <= 0.5 * (cfg.facing_full + cfg.facing_zero))
    inliers = int(inl.sum())
    rms = float(np.sqrt(np.mean(r[inl] ** 2))) if inliers else float("inf")

    A = O = H = g = H_data = None
    if jac:
        A, O = model.axes(poses)
        a_idx = np.flatnonzero(assoc)
        ra = r[a_idx]
        w = wa[a_idx] * np.where(np.abs(ra) <= cfg.huber_m, 1.0, cfg.huber_m / np.maximum(np.abs(ra), 1e-12))
        # d r / d q_j = -a_j . ((p - o_j) x n) = (p - o_j) . (a_j x n), zero unless joint j moves the link
        C = (nb[a_idx] @ _skews(A)).reshape(-1, nj, 3)          # (m, nj, 3): a_j x n
        J = np.einsum("ik,ijk->ij", P[a_idx], C) - np.einsum("jk,ijk->ij", O, C)
        J *= model.moves[best_l[a_idx]]
        Jw = J * w[:, None]
        H_data = J.T @ Jw
        H = H_data.copy()
        g = Jw.T @ ra

    # free space: the moving links' samples facing the camera, z-buffered, against the observed depth
    Pm, Nm, Lm = model.posed_points(poses, "fs", links=model.moving)
    Pc = cam.to_cam(Pm)
    zc = Pc[:, 2]
    front = (np.einsum("ij,ij->i", Nm @ cam.R, Pc) < 0) & (zc > 0.05)
    fi = np.flatnonzero(front)
    u, v = cam.project_cam(Pc[fi])
    ui, vi = np.rint(u).astype(np.int64), np.rint(v).astype(np.int64)
    inside = (ui >= 0) & (ui < cam.w) & (vi >= 0) & (vi < cam.h)
    fi, u, v, ui, vi = fi[inside], u[inside], v[inside], ui[inside], vi[inside]
    z = zc[fi]
    # quarter-resolution z-buffer, each sample splatted over 2x2 cells; nearest per cell by sorting
    W4 = cam.w // 4 + 2
    cell = (vi // 4) * W4 + ui // 4
    cells = np.concatenate([cell, cell + 1, cell + W4, cell + W4 + 1])
    zz = np.tile(z, 4)
    order = np.lexsort((zz, cells))
    cs = cells[order]
    first = np.ones(cs.size, dtype=bool)
    first[1:] = cs[1:] != cs[:-1]
    ucell, zmin = cs[first], zz[order][first]
    visible = z <= zmin[np.searchsorted(ucell, cell)] + 0.005
    zo = obs.depth_min3[vi * cam.w + ui]
    considered = visible & (zo > 0)
    excess = zo - z - cfg.fs_margin_m
    viol = considered & (excess > 0)
    fs_considered, fs_violations = int(considered.sum()), int(viol.sum())
    fs_frac = fs_violations / fs_considered if fs_considered else 0.0
    mi = np.flatnonzero(viol)
    fs_pixels = np.column_stack([u[mi], v[mi]])
    if mi.size:
        zv = z[mi]
        dtv = _bilinear(obs.dt, u[mi], v[mi])
        lin = dtv * zv / cam.fx
        ramp = np.clip(excess[mi] / cfg.fs_ramp_m, 0.0, 1.0)
        rf = cfg.fs_weight * ramp * np.minimum(lin, cfg.fs_cap_m)
        cost += 0.5 * float((rf ** 2).sum())
        if jac:
            live = lin < cfg.fs_cap_m                         # a capped residual has no gradient
            vv, uu = vi[mi], ui[mi]                           # central differences of the distance transform
            u1, u0 = np.minimum(uu + 1, cam.w - 1), np.maximum(uu - 1, 0)
            v1, v0 = np.minimum(vv + 1, cam.h - 1), np.maximum(vv - 1, 0)
            gx = (obs.dt[vv, u1] - obs.dt[vv, u0]) / np.maximum(u1 - u0, 1)
            gy = (obs.dt[v1, uu] - obs.dt[v0, uu]) / np.maximum(v1 - v0, 1)
            pc = Pc[fi[mi]]
            x, y = pc[:, 0], pc[:, 1]
            scale = cfg.fs_weight * ramp * zv / cam.fx
            lm = Lm[fi[mi]]
            pm = Pm[fi[mi]]
            # image gradient of the residual as a camera-frame direction: d r / d p_cam = scale * G
            G = np.column_stack([gx * cam.fx / zv, gy * cam.fy / zv,
                                 -(gx * cam.fx * x + gy * cam.fy * y) / zv ** 2]) * scale[:, None]
            Gb = G @ cam.R.T                                    # the same, base frame
            # point velocity for joint j: a_j x (p - o_j); r_j = Gb . (a_j x (p - o_j)) = (p - o_j) . (Gb x a_j)
            C = -(Gb @ _skews(A)).reshape(-1, nj, 3)            # (k, nj, 3): Gb x a_j = -(a_j x Gb)
            Jf = np.einsum("ik,ijk->ij", pm, C) - np.einsum("jk,ijk->ij", O, C)
            Jf *= model.moves[lm] & live[:, None]
            H += Jf.T @ Jf
            g += Jf.T @ rf
    return Evaluation(cost, H, g, H_data, assoc, np.where(assoc, best_l, -1), r, best_d, inliers, rms,
                      fs_violations, fs_considered, fs_frac, fs_pixels)


# ----- the optimiser -----------------------------------------------------------------------------------
@dataclass
class Solution:
    q: np.ndarray
    ev: Evaluation
    iterations: int
    converged: bool
    alternative_cost: float | None = None   # re-init / flip test: the strongest competing explanation's cost
    alternative: Solution | None = field(default=None, repr=False)


def sample_levers(model: ArmModel, poses) -> np.ndarray:
    """Per joint, the largest speed (m per rad) of any (coarse) model sample that joint moves, at these poses."""
    A, O = model.axes(poses)
    lev = np.zeros(model.nj)
    for li, link in enumerate(model.links):
        js = np.flatnonzero(model.moves[li])
        if not js.size:
            continue
        p = model.search[link][0]
        T = poses[link]
        W = p @ T[:3, :3].T + T[:3, 3]
        for j in js:
            lev[j] = max(lev[j], float(np.linalg.norm(np.cross(A[j], W - O[j]), axis=1).max()))
    return lev


def levers(model: ArmModel, poses) -> np.ndarray:
    """Per joint, the largest speed (m per rad) it gives any link origin (the joint centres and frames such as
    the tool's gripper_frame_link), at these poses. A joint that moves no origin (the jaw, about its own
    axis) has lever 0: it never holds up convergence, since no reported point depends on it."""
    A, O = model.axes(poses)
    lev = np.zeros(model.nj)
    for link in model.urdf.links:
        if link == model.root:
            continue
        p = poses[link][:3, 3]
        for j, jn in enumerate(model.joints):
            if model.joint_child[j] == link or any(jj.name == jn for jj in model.urdf.chain(link)):
                lev[j] = max(lev[j], float(np.linalg.norm(np.cross(A[j], p - O[j]))))
    return lev


def solve(model: ArmModel, cam: Camera, obs: Observation, q0: np.ndarray, cfg: FitConfig, *,
          wide_first: bool = False, exact_iterations: int | None = None) -> Solution:
    """Levenberg-Marquardt from q0, re-associating at every evaluation.

    Main phase: up to ``max_iterations - exact_iterations`` steps with the per-link fields (from a
    global-search candidate, ``wide_first``: ``search_iterations`` steps and ``search_exact_iterations`` of
    polish). Before each step the undamped Gauss-Newton step is solved (unobserved directions get none); if
    it would move no model sample more than ``step_tol_m`` (joint step x the joint's lever arm), the phase
    has converged. If no damped step lowers the cost, the phase stops, converged only if that Gauss-Newton
    step was below ``stall_tol_m``. Running out of steps is not convergence. Polish: the same with exact
    nearest-sample association; when its budget runs out, it counts as settled if its last step moved the
    model less than ``polish_tol_m``. Bounds: the URDF limits widened by the joint margin."""
    nj = model.nj
    q = np.clip(np.asarray(q0, dtype=np.float64), model.lo, model.hi)
    poses0 = model.poses(q)
    lev = levers(model, poses0)                     # convergence: motion of the link origins
    slev = np.maximum(sample_levers(model, poses0), 1e-3)   # candidate lists: motion of any surface sample
    gate = cfg.gate_first_m if wide_first else cfg.gate_m
    cap = cfg.search_iterations if wide_first else cfg.max_iterations
    if exact_iterations is not None:
        n_exact = exact_iterations
    else:
        n_exact = cfg.search_exact_iterations if wide_first else cfg.exact_iterations
    phases = [(False, max(1, cap - max(n_exact, 0)))]
    if n_exact > 0:
        phases.append((True, n_exact))
    it = 0
    converged = False
    ev = None
    extra = cfg.candidate_margin_m
    cand, moved = link_candidates(model, obs.points, model.poses(q), extra), 0.0
    for exact, budget in phases:
        ev = evaluate(model, cam, obs, q, cfg, gate=gate, exact=exact, cand=cand)
        lam = cfg.lm_lambda0
        settled, last = False, np.inf
        for _ in range(budget):
            gn = np.linalg.lstsq(ev.H, -ev.g, rcond=1e-9)[0]
            gn_m = float(np.max(np.abs(np.clip(q + gn, model.lo, model.hi) - q) * lev))
            if gn_m < cfg.step_tol_m:
                settled = True
                break
            it += 1
            accepted = False
            for _ in range(cfg.lm_retries):
                Hd = ev.H + lam * np.diag(np.diag(ev.H)) + 1e-12 * np.eye(nj)
                try:
                    dq = np.linalg.solve(Hd, -ev.g)
                except np.linalg.LinAlgError:
                    dq = np.linalg.lstsq(Hd, -ev.g, rcond=None)[0]
                qn = np.clip(q + dq, model.lo, model.hi)
                trial = float(np.max(np.abs(qn - q) * lev))
                swept = float(np.max(np.abs(qn - q) * slev))
                if moved + swept > extra:            # the candidate lists would no longer be safe
                    cand, moved = link_candidates(model, obs.points, model.poses(q), extra), 0.0
                    ev = evaluate(model, cam, obs, q, cfg, gate=gate, exact=exact, cand=cand)
                evn = evaluate(model, cam, obs, qn, cfg, gate=gate, exact=exact, cand=cand)
                if evn.cost <= ev.cost:
                    last = trial
                    moved += swept
                    q, ev, accepted = qn, evn, True
                    lam = max(lam * 0.3, 1e-9)
                    break
                lam *= 5.0
            if gate != cfg.gate_m:                  # the wide gate is for the first step only
                gate = cfg.gate_m
                ev = evaluate(model, cam, obs, q, cfg, gate=gate, exact=exact, cand=cand)
            if not accepted:                        # no damped step lowers the cost: a minimum of this phase
                settled = exact or gn_m < cfg.stall_tol_m
                break
        else:                                       # out of steps: settled only if they had become small
            settled = last < cfg.polish_tol_m if exact else last < cfg.stall_tol_m
        converged = settled if not exact else (converged and settled)
    return Solution(q, ev, it, converged)


# ----- global search -------------------------------------------------------------------------------------
def _grid(lo: float, hi: float, step: float) -> np.ndarray:
    n = max(1, math.ceil((hi - lo) / step - 1e-9) + 1)
    return np.linspace(lo, hi, n) if n > 1 else np.array([(lo + hi) / 2])


def _render_compare(model: ArmModel, cam: Camera, obs: Observation, Q: np.ndarray, links, cfg: FitConfig,
                    chunk: int = 256) -> np.ndarray:
    """Cheap agreement score of each candidate pose: model samples (the coarse 'search' set) that face the
    camera and land on observed data within search_support_m, minus twice those the camera sees through.
    Candidates that put the model below the table score -inf."""
    pts = [model.search[link] for link in links]
    out = np.empty(len(Q))
    D, M = obs.depth, obs.data_mask
    for c0 in range(0, len(Q), chunk):
        Qc = Q[c0:c0 + chunk]
        poses = model.urdf.fk_batch({j: Qc[:, k] for k, j in enumerate(model.joints)})
        W, WN = [], []
        for link, (p, nrm) in zip(links, pts, strict=True):
            T = poses[link]
            W.append(np.matmul(T[:, :3, :3], p.T).transpose(0, 2, 1) + T[:, None, :3, 3])
            WN.append(np.matmul(T[:, :3, :3], nrm.T).transpose(0, 2, 1))
        W, WN = np.concatenate(W, axis=1), np.concatenate(WN, axis=1)
        below = (W[..., 2] < model.table_z - 0.01).any(axis=1)
        Pc = (W - cam.t) @ cam.R
        Nc = WN @ cam.R
        z = Pc[..., 2]
        front = (np.einsum("cij,cij->ci", Nc, Pc) < 0) & (z > 0.05)
        zs = np.where(front, z, 1.0)
        u = np.rint(cam.fx * Pc[..., 0] / zs + cam.cx).astype(np.int64)
        v = np.rint(cam.fy * Pc[..., 1] / zs + cam.cy).astype(np.int64)
        ok = front & (u >= 0) & (u < cam.w) & (v >= 0) & (v < cam.h)
        flat = np.where(ok, v * cam.w + u, 0)
        zo = D[flat]
        sup = ok & M[flat] & (np.abs(zo - z) < cfg.search_support_m)
        vio = ok & (zo > 0) & (zo > z + cfg.search_violation_m)
        s = sup.sum(axis=1) - 2.0 * vio.sum(axis=1)
        s[below] = -np.inf
        out[c0:c0 + chunk] = s
    return out


def _distinct(Q: np.ndarray, scores: np.ndarray, k: int, sep: np.ndarray) -> list[int]:
    """Indices of up to k best-scoring candidates, each differing from every one already picked by more
    than ``sep`` in at least one joint (descending score; ties keep input order)."""
    order = np.argsort(-scores, kind="stable")
    picked: list[int] = []
    for i in order:
        if not np.isfinite(scores[i]):
            break
        if all(np.any(np.abs(Q[i] - Q[p]) > sep) for p in picked):
            picked.append(int(i))
            if len(picked) == k:
                break
    return picked


def global_search(model: ArmModel, cam: Camera, obs: Observation, cfg: FitConfig) -> Solution | None:
    """Find the pose from scratch: a grid over the first joints scored with the links they move, a grid over
    the remaining joints for the best few, the robust cost for the best of those, then Levenberg-Marquardt
    from the best handful. Returns the solution with the lowest final cost, or None without data."""
    if obs.n_points == 0:
        return None
    nj = model.nj
    npx = min(cfg.search_proximal, nj)
    step = math.radians(cfg.search_step_deg)
    lo, hi = model.limits_lo, model.limits_hi
    mid = np.clip(np.zeros(nj), lo, hi)
    grids = [_grid(lo[j], hi[j], step) for j in range(npx)]
    mesh_ = np.meshgrid(*grids, indexing="ij")
    Q1 = np.tile(mid, (mesh_[0].size, 1))
    for j in range(npx):
        Q1[:, j] = mesh_[j].ravel()
    root = model.root
    prox_links = [link for li, link in enumerate(model.links)
                  if link != root and not model.moves[li, npx:].any()]
    s1 = _render_compare(model, cam, obs, Q1, prox_links, cfg)
    sep1 = np.full(nj, np.inf)
    sep1[:npx] = 1.01 * step
    top1 = _distinct(Q1, s1, cfg.search_top_proximal, sep1)
    # distal joints
    dsteps = list(cfg.search_distal_steps_deg) + [cfg.search_distal_steps_deg[-1]] * nj
    dgrids = [_grid(lo[j], hi[j], math.radians(dsteps[j - npx])) for j in range(npx, nj)]
    if dgrids:
        dm = np.meshgrid(*dgrids, indexing="ij")
        D2 = np.column_stack([m.ravel() for m in dm])
    else:
        D2 = np.zeros((1, 0))
    Q2 = np.concatenate([np.column_stack([np.tile(Q1[i, :npx], (len(D2), 1)), D2]) for i in top1]) \
        if top1 else np.zeros((0, nj))
    if not len(Q2):
        return None
    moving = [link for link in model.links if link != root]
    s2 = _render_compare(model, cam, obs, Q2, moving, cfg)
    sep2 = np.array([1.01 * step] * npx + [1.01 * math.radians(dsteps[j - npx]) for j in range(npx, nj)])
    top2 = _distinct(Q2, s2, cfg.search_top_full, sep2)
    if not top2:
        return None
    costs = np.array([evaluate(model, cam, obs, Q2[i], cfg, gate=cfg.gate_first_m, exact=False,
                               jac=False).cost for i in top2])
    ranked = [top2[i] for i in np.argsort(costs, kind="stable")]
    refine = ranked[: cfg.search_refine]
    # distal joints: also refine the best candidate in each quarter of their range. The coarse scores favour
    # poses that put model surface over the data (a wrist folded back over the forearm, a mirrored gripper),
    # which must not crowd out the right one when the true distal links are partly hidden.
    for name in cfg.diverse_joints:
        if name not in model.joints:
            continue
        j = model.joints.index(name)
        quarter = max((hi[j] - lo[j]) / 4.0, 1e-9)
        seen = {int((Q2[i, j] - lo[j]) // quarter) for i in refine}
        for i in ranked:
            b = int((Q2[i, j] - lo[j]) // quarter)
            if b not in seen:
                seen.add(b)
                refine.append(i)
    few = obs.subset(cfg.search_points)
    sols = [solve(model, cam, few, Q2[i], cfg, wide_first=True, exact_iterations=0) for i in refine]
    best = min(sols, key=lambda s: s.ev.cost)
    best = scan_refine(model, cam, few, best, cfg, sols)
    sols += flip_solutions(model, cam, few, best, cfg)
    alt = distinct_alternative(model, best, sols)
    best = polish(model, cam, obs, best, cfg)
    if alt is not None:
        alt = polish(model, cam, obs, alt, cfg)
        if alt.ev.cost < best.ev.cost:
            best, alt = alt, best
    best.alternative = alt
    best.alternative_cost = alt.ev.cost if alt is not None else None
    return best


def polish(model: ArmModel, cam: Camera, obs: Observation, sol: Solution, cfg: FitConfig) -> Solution:
    """The solution refined with exact nearest-sample association (``search_exact_iterations`` steps)."""
    return solve(model, cam, obs, sol.q, cfg, exact_iterations=cfg.search_exact_iterations)


def flip_solutions(model: ArmModel, cam: Camera, obs: Observation, sol: Solution, cfg: FitConfig
                   ) -> list[Solution]:
    """Fits started from ``sol`` with each flip joint turned by +-180 deg (clipped into the bounds, when that
    turns it by more than 90 deg)."""
    out = []
    for name in cfg.flip_joints:
        if name not in model.joints:
            continue
        j = model.joints.index(name)
        for s in (1.0, -1.0):
            v = float(np.clip(sol.q[j] + s * math.pi, model.lo[j], model.hi[j]))
            if abs(v - sol.q[j]) > math.pi / 2:
                qq = sol.q.copy()
                qq[j] = v
                out.append(solve(model, cam, obs, qq, cfg, wide_first=True, exact_iterations=0))
    return out


def configuration_distance(model: ArmModel, qa: np.ndarray, qb: np.ndarray) -> float:
    """Largest distance (m) between the same link origin in two configurations."""
    pa, pb = model.urdf.fk(model.q_dict(qa)), model.urdf.fk(model.q_dict(qb))
    return max(float(np.linalg.norm(pa[k][:3, 3] - pb[k][:3, 3])) for k in pa)


def distinct_alternative(model: ArmModel, best: Solution, sols, min_distance: float = 0.010) -> Solution | None:
    """The lowest-cost solution whose configuration differs from the best's (some link origin > 10 mm away):
    the strongest competing explanation of the same data."""
    alt = None
    for s in sols:
        if s is best or (alt is not None and s.ev.cost >= alt.ev.cost):
            continue
        if configuration_distance(model, s.q, best.q) > min_distance:
            alt = s
    return alt


def scan_refine(model: ArmModel, cam: Camera, obs: Observation, sol: Solution | None, cfg: FitConfig,
                tried: list | None = None) -> Solution | None:
    """The distal joints have narrow basins and near-symmetries (the SO-101's closed gripper looks much
    the same rolled by 180 deg; its small jaw is easily mis-set). For each of ``cfg.scan_joints``: the cost
    over the joint's whole range at ``scan_step_deg`` with the other joints held; Levenberg-Marquardt from
    each of the best ``scan_starts`` local minima away from the current value; keep the lowest final cost.
    Every fit made is appended to ``tried``."""
    if sol is None:
        return None
    step = math.radians(cfg.scan_step_deg)
    for name in cfg.scan_joints:
        if name not in model.joints:
            continue
        j = model.joints.index(name)
        vals = _grid(model.limits_lo[j], model.limits_hi[j], step)
        costs = []
        for val in vals:
            qq = sol.q.copy()
            qq[j] = val
            costs.append(evaluate(model, cam, obs, qq, cfg, gate=cfg.gate_m, exact=False, jac=False).cost)
        c = np.array(costs)
        left = np.r_[np.inf, c[:-1]]
        right = np.r_[c[1:], np.inf]
        minima = [int(k) for k in np.argsort(c, kind="stable") if c[k] <= left[k] and c[k] <= right[k]]
        starts = [k for k in minima if abs(vals[k] - sol.q[j]) > 2 * step][: cfg.scan_starts]
        base = sol
        for k in starts:
            qq = base.q.copy()
            qq[j] = float(vals[k])
            other = solve(model, cam, obs, qq, cfg, wide_first=True, exact_iterations=0)
            if tried is not None:
                tried.append(other)
            if other.ev.cost < sol.ev.cost:
                sol = other
    return sol


# ----- validity ---------------------------------------------------------------------------------------
def frame_problems(sol: Solution | None, obs: Observation, cfg: FitConfig) -> list[str]:
    if obs.n_points < cfg.min_inliers:
        return [f"only {obs.n_points} observed points (need {cfg.min_inliers})"]
    if sol is None:
        return ["no pose found"]
    ev = sol.ev
    out = []
    if not sol.converged:
        out.append("did not converge")
    if ev.inliers < cfg.min_inliers:
        out.append(f"{ev.inliers} inliers < {cfg.min_inliers}")
    frac = ev.inliers / max(obs.n_points, 1)
    if frac < cfg.min_inlier_frac:
        out.append(f"inlier fraction {frac:.2f} < {cfg.min_inlier_frac}")
    if ev.rms > cfg.max_rms_m:
        out.append(f"inlier rms {ev.rms * 1e3:.1f} mm > {cfg.max_rms_m * 1e3:.1f}")
    if ev.fs_frac > cfg.max_free_space_frac:
        out.append(f"free-space violations {ev.fs_frac:.2f} > {cfg.max_free_space_frac}")
    return out


def point_sigmas(model: ArmModel, sol: Solution, named: Mapping[str, tuple[str, object]],
                 poses=None) -> dict[str, float]:
    """Formal 1-sigma (m) of each named point: the inlier residual variance through the fit's normal
    equations. Small whenever the joints a point depends on are constrained by data; large when they are
    not (no data on the links that fix them)."""
    ev = sol.ev
    poses = poses if poses is not None else model.poses(sol.q)
    A, O = model.axes(poses)
    s2 = ev.rms ** 2 if np.isfinite(ev.rms) else 1.0
    Hd = ev.H_data if ev.H_data is not None else np.zeros((model.nj, model.nj))
    cov = s2 * np.linalg.inv(Hd + 1e-9 * np.eye(model.nj))
    out = {}
    for pid, (link, off) in named.items():
        T = poses[link]
        p = T[:3, :3] @ (np.zeros(3) if off is None else np.asarray(off, float)) + T[:3, 3]
        Jp = np.zeros((3, model.nj))
        chain = {j.name for j in model.urdf.chain(link)} if link != model.root else set()
        for k, jn in enumerate(model.joints):
            if jn in chain:
                Jp[:, k] = np.cross(A[k], p - O[k])
        out[pid] = float(np.sqrt(max(np.trace(Jp @ cov @ Jp.T), 0.0)))
    return out


# ----- the tracker --------------------------------------------------------------------------------------
@dataclass
class FrameResult:
    index: int
    t: float
    status: str                       # TRACK | REINIT | LOST
    valid: bool
    q: np.ndarray | None
    points: dict[str, np.ndarray]
    point_valid: dict[str, bool]
    point_sigma: dict[str, float]
    cost: float
    n_points: int
    inliers: int
    rms: float
    fs_frac: float
    iterations: int
    reason: str
    ms: float
    solution: Solution | None = field(default=None, repr=False)
    observation: Observation | None = field(default=None, repr=False)
    alt_cost: float | None = None     # re-inits and flip tests: the best cost found in the other flip basin


class Tracker:
    """Frame-to-frame state: the previous frame's own estimate (only if that frame was valid), the last valid
    points with their time (for the jump check), the points currently in doubt (an alternative explanation
    of the data nearly as good puts them elsewhere) and a frame counter for the periodic checks. Nothing pulls
    one frame's estimate toward another's."""

    def __init__(self, model: ArmModel, cam: Camera, named: Mapping[str, tuple[str, object]],
                 cfg: FitConfig, background_mm: np.ndarray | None = None):
        self.model, self.cam, self.named, self.cfg, self.bg = model, cam, dict(named), cfg, background_mm
        self.prev_q = None                 # set only after a valid frame
        self.last_valid_t = None
        self.last_valid_points = None
        self.doubtful: dict[str, str] = {}
        self.since_check = 0

    def _points(self, q) -> dict[str, np.ndarray]:
        return self.model.urdf.points(self.named, self.model.q_dict(q))

    def _jump(self, pts, t) -> str | None:
        if self.last_valid_points is None:
            return None
        dt = max(0.0, t - self.last_valid_t)
        lim = self.cfg.jump_speed_mps * dt + self.cfg.jump_margin_m
        worst = max(float(np.linalg.norm(pts[k] - self.last_valid_points[k])) for k in pts)
        return None if worst <= lim else f"jump {worst * 1e3:.0f} mm > {lim * 1e3:.0f} mm in {dt * 1e3:.0f} ms"

    def _check(self, sol, obs, t) -> list[str]:
        reasons = frame_problems(sol, obs, self.cfg)
        if not reasons:
            j = self._jump(self._points(sol.q), t)
            reasons = [j] if j else []
        return reasons

    def _ambiguous(self, sol: Solution) -> dict[str, str]:
        """Points that the strongest competing explanation (within ambiguity_ratio of the cost) puts more than
        ambiguity_m away."""
        alt = sol.alternative
        if alt is None or sol.ev.cost <= 0:
            return {}
        ratio = alt.ev.cost / sol.ev.cost
        if ratio >= self.cfg.ambiguity_ratio:
            return {}
        pb, pa = self._points(sol.q), self._points(alt.q)
        out = {}
        for k in pb:
            d = float(np.linalg.norm(pb[k] - pa[k]))
            if d > self.cfg.ambiguity_m:
                out[k] = f"{k} ambiguous: a fit {ratio:.3f}x the cost puts it {d * 1e3:.0f} mm away"
        return out

    def step(self, depth_mm: np.ndarray, t: float, index: int = -1) -> FrameResult:
        t0 = time.perf_counter()
        cfg = self.cfg
        obs = observe(depth_mm, self.cam, self.model, cfg, self.bg)
        status, sol, reasons, notes = "LOST", None, [], []
        if obs.n_points < cfg.min_inliers:
            reasons = [f"only {obs.n_points} observed points (need {cfg.min_inliers})"]
        else:
            if self.prev_q is not None:
                sol = solve(self.model, self.cam, obs, self.prev_q, cfg)
                reasons = self._check(sol, obs, t)
                status = "TRACK"
                self.since_check += 1
                every = cfg.recheck_every if self.doubtful else cfg.flip_check_every
                if not reasons and every and self.since_check >= every:
                    self.since_check = 0
                    if self.doubtful:                       # still in doubt: look again from scratch
                        gs = global_search(self.model, self.cam, obs, cfg)
                        if gs is not None and not self._check(gs, obs, t):
                            sol, status = gs, "REINIT"
                            notes.append("recheck")
                            self.doubtful = self._ambiguous(gs)
                    else:                                   # the mirror basin of the near-symmetric joints
                        flips = flip_solutions(self.model, self.cam, obs, sol, cfg)
                        alt = min(flips, key=lambda s: s.ev.cost) if flips else None
                        if alt is not None and alt.ev.cost < sol.ev.cost and not self._check(alt, obs, t):
                            sol, alt, status = alt, sol, "REINIT"
                            notes.append("flip test found a lower cost")
                        if alt is not None and configuration_distance(self.model, alt.q, sol.q) > 0.010:
                            sol.alternative, sol.alternative_cost = alt, alt.ev.cost
                        self.doubtful = self._ambiguous(sol)
            if self.prev_q is None or reasons:
                tracked_reasons = reasons
                gs = global_search(self.model, self.cam, obs, cfg)
                status = "REINIT"
                self.since_check = 0
                if gs is not None:
                    sol = gs
                reasons = self._check(gs, obs, t) if gs is not None else frame_problems(None, obs, cfg)
                if reasons and tracked_reasons:
                    reasons = [f"track: {'; '.join(tracked_reasons)}", f"reinit: {'; '.join(reasons)}"]
                self.doubtful = self._ambiguous(gs) if gs is not None else {}
        valid = not reasons and sol is not None
        if not valid:
            status = "LOST"
            self.doubtful = {}
        pts, psig, pval = {}, {}, {}
        if sol is not None:
            poses = self.model.poses(sol.q)
            pts = self._points(sol.q)
            psig = point_sigmas(self.model, sol, self.named, poses)
            pval = {k: bool(valid and psig[k] <= cfg.max_point_sigma_m and k not in self.doubtful) for k in pts}
            if valid:
                notes += [self.doubtful[k] for k in pts if k in self.doubtful]
        if valid:
            self.prev_q = sol.q.copy()
            self.last_valid_t, self.last_valid_points = t, pts
        else:
            self.prev_q = None
        ev = sol.ev if sol is not None else None
        return FrameResult(
            index=index, t=t, status=status, valid=valid, q=None if sol is None else sol.q.copy(), points=pts,
            point_valid=pval, point_sigma=psig, cost=float(ev.cost) if ev else float("nan"),
            n_points=obs.n_points, inliers=ev.inliers if ev else 0, rms=float(ev.rms) if ev else float("nan"),
            fs_frac=float(ev.fs_frac) if ev else float("nan"), iterations=sol.iterations if sol else 0,
            reason="; ".join(reasons + notes), ms=(time.perf_counter() - t0) * 1e3, solution=sol, observation=obs,
            alt_cost=sol.alternative_cost if sol is not None else None)


# ----- inputs ------------------------------------------------------------------------------------------
def load_registration(src) -> dict:
    """registration.json (path or dict) -> dict with ``T_bl_cam`` as a 4x4 array, checked."""
    if isinstance(src, Mapping):
        doc = dict(src)
    else:
        p = Path(src)
        if not p.is_file():
            raise FitError(f"no registration at {p}: run `be fit register SESSION` first")
        doc = json.loads(p.read_text(encoding="utf-8"))
    T = np.asarray(doc.get("T_bl_cam") or (doc.get("outputs") or {}).get("T_bl_cam"), dtype=np.float64)
    if T.shape != (4, 4) or not np.isfinite(T).all() or not np.allclose(T[3], [0, 0, 0, 1]):
        raise FitError("registration has no valid 4x4 T_bl_cam")
    R = T[:3, :3]
    if not np.allclose(R @ R.T, np.eye(3), atol=1e-6) or np.linalg.det(R) < 0:
        raise FitError("registration T_bl_cam is not a rigid transform")
    doc["T_bl_cam"] = T
    return doc


def _load_envelope(src):
    from bounded_embodiment.envelope import load as load_envelope

    env = load_envelope(src if isinstance(src, Mapping) else str(src))
    return env


def _sha256_file(p: Path) -> str:
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def _fmt(x: float, nd: int = 6) -> str:
    if x is None or not math.isfinite(x):
        return ""
    s = f"{x:.{nd}f}"
    return "0." + "0" * nd if s == "-0." + "0" * nd else s


# ----- the session runner ------------------------------------------------------------------------------
def fit_session(session, urdf, envelope, registration=None, *, background=None, config: FitConfig | None = None,
                out=None, t_from=None, t_to=None, every: int = 1, debug_every: int = 0,
                model: ArmModel | None = None, say: Callable[[str], None] = print, max_frames: int | None = None
                ) -> dict:
    """Fit every ``every``-th depth frame of ``session`` between ``t_from`` and ``t_to`` (labels, offsets or
    seconds; see :meth:`Session.resolve_time`) and write samples.csv, fit_log.csv and fit_meta.json to
    ``out`` (default ``<session>/fit``). ``registration`` defaults to ``<session>/fit/registration.json``
    and ``background`` to ``<session>/fit/background.npz`` (fit without one, with a warning, if absent)."""
    cfg = config or FitConfig()
    sess = session if isinstance(session, Session) else Session(session)
    out = Path(out) if out else sess.path / "fit"
    out.mkdir(parents=True, exist_ok=True)
    reg_path = None
    if registration is None or not isinstance(registration, Mapping):
        reg_path = Path(registration) if registration is not None else sess.path / "fit" / "registration.json"
    reg = load_registration(registration if isinstance(registration, Mapping) else reg_path)
    if model is None:
        if urdf is None:
            raise FitError("give the URDF")
        model = ArmModel(urdf)
    urdf_path = model.urdf_path or (Path(urdf) if urdf is not None and not isinstance(urdf, ArmModel) else None)
    env = _load_envelope(envelope)
    frame = env.normalized.get("frame", "")
    if frame != model.root:
        raise FitError(f"the envelope's frame is {frame!r}; the fitter reports in the URDF root {model.root!r}")
    named = {pid: (p.link, p.offset_m) for pid, p in env.points.items() if p.link}
    if not named:
        raise FitError("the envelope has no points that name a link")
    unknown = sorted({link for link, _ in named.values() if link not in model.urdf.links})
    if unknown:
        raise FitError(f"envelope points name links the URDF does not have: {unknown}")
    K, dist = sess.intrinsics("640x360")
    cam = Camera(K, dist, (640, 360), reg["T_bl_cam"], distorted=cfg.depth_distorted, seed=cfg.seed)
    bg, bg_path, bg_info = None, None, None
    if background is not None or (sess.path / "fit" / "background.npz").is_file():
        bg_path = Path(background) if background is not None else sess.path / "fit" / "background.npz"
        with np.load(bg_path) as z:
            bg = np.asarray(z["background_mm"])
            bg_info = {k: z[k].tolist() for k in z.files if k != "background_mm" and z[k].size < 64}
        if bg.shape != (cam.h, cam.w):
            raise FitError(f"background is {bg.shape}, depth frames are {(cam.h, cam.w)}")
    else:
        say("note: no background.npz (run `be fit background SESSION`); fitting without background subtraction")
    a = sess.resolve_time(t_from) if t_from is not None else None
    b = sess.resolve_time(t_to) if t_to is not None else None
    indices = sess.depth_indices(a, b, every)
    if max_frames is not None:
        indices = indices[:max_frames]
    if not len(indices):
        raise FitError("no depth frames in the requested range")
    tracker = Tracker(model, cam, named, cfg, bg)
    debug_dir = out / "debug"
    if debug_every:
        debug_dir.mkdir(exist_ok=True)
    counts = {s: 0 for s in STATUS}
    pvalid = {k: 0 for k in named}
    ms = []
    log_cols = ["frame", "depth_index", "t_s", *model.joints, "cost", "alt_cost", "points", "inliers", "rms_m",
                "free_space_frac", "iterations", "status", "valid", "reason", "ms"]
    t_start = time.perf_counter()
    with open(out / "samples.csv", "w", newline="", encoding="utf-8") as fs, \
            open(out / "fit_log.csv", "w", newline="", encoding="utf-8") as fl:
        fs.write(",".join(SAMPLES_HEADER) + "\n")
        log = csv.writer(fl, lineterminator="\n")
        log.writerow(log_cols)
        for n, i in enumerate(indices):
            i = int(i)
            t = float(sess.depth["t"][i])
            res = tracker.step(sess.depth_frame(i), t, i)
            counts[res.status] += 1
            ms.append(res.ms)
            for pid in named:
                ok = bool(res.valid and res.point_valid.get(pid))
                pvalid[pid] += ok
                if ok:
                    x, y, z = res.points[pid]
                    fs.write(f"{_fmt(t)},{pid},{_fmt(x)},{_fmt(y)},{_fmt(z)},1,{_fmt(res.rms)}\n")
                else:
                    fs.write(f"{_fmt(t)},{pid},,,,0,{_fmt(res.rms)}\n")
            q = res.q if res.q is not None else [float("nan")] * model.nj
            reason = res.reason
            if res.valid:
                bad = [f"{k} sigma {res.point_sigma[k] * 1e3:.1f} mm" for k in named
                       if not res.point_valid[k] and res.point_sigma[k] > cfg.max_point_sigma_m]
                reason = "; ".join([r for r in [res.reason] if r] + bad)
            log.writerow([n, i, _fmt(t), *(_fmt(float(v)) for v in q), _fmt(res.cost, 9),
                          _fmt(res.alt_cost, 9) if res.alt_cost is not None else "", res.n_points,
                          res.inliers, _fmt(res.rms), _fmt(res.fs_frac, 4), res.iterations, res.status,
                          int(res.valid), reason, f"{res.ms:.1f}"])
            if debug_every and n % debug_every == 0:
                try:
                    debug_overlay(sess, i, cam, model, res, debug_dir / f"frame_{n:06d}.png")
                except Exception as e:  # noqa: BLE001 - a debug image never stops the fit
                    say(f"note: debug image for frame {n} failed: {e}")
            if n % 100 == 0 or n == len(indices) - 1:
                say(f"  frame {n + 1}/{len(indices)}  t {t - sess.t0:8.2f} s  {res.status:6s} "
                    f"rms {res.rms * 1e3 if math.isfinite(res.rms) else float('nan'):5.2f} mm  "
                    f"inliers {res.inliers:5d}/{res.n_points:5d}  fs {res.fs_frac:.2f}  {res.ms:6.1f} ms")
    wall = time.perf_counter() - t_start
    meta = {
        "format": "be-fit/0.1",
        "instrument": {"kind": "markerless_model_tracker", "independent_of_robot_state": True,
                       "external": True, "implementation": "bounded_embodiment_kit.instruments.markerless.fit",
                       "version": FIT_VERSION},
        "versions": _versions(),
        "session": {"path": str(sess.path), "format": sess.meta.get("format"),
                    "pairing": sess.pairing_summary() if debug_every else None},
        "inputs": {
            "urdf": {"path": str(urdf_path) if urdf_path else None,
                     "sha256": _sha256_file(urdf_path) if urdf_path and urdf_path.is_file() else None},
            "meshes": mesh.describe(model.urdf),
            "envelope": {"path": None if isinstance(envelope, Mapping) else str(envelope), "hash": env.hash,
                         "points": {k: {"link": v[0], "offset_m": v[1]} for k, v in named.items()}},
            "registration": {"path": str(reg_path) if reg_path else None,
                             "sha256": _sha256_file(reg_path) if reg_path else None,
                             "T_bl_cam": reg["T_bl_cam"].tolist()},
            "background": {"path": str(bg_path) if bg_path else None,
                           "sha256": _sha256_file(bg_path) if bg_path else None, "info": bg_info},
            "intrinsics_640x360": {"K": cam.K.tolist(), "dist": cam.dist.tolist(), "distorted": cam.distorted},
        },
        "range": {"from": t_from, "to": t_to, "t_from_s": a, "t_to_s": b, "every": every,
                  "frames": len(indices)},
        "config": cfg.as_dict(),
        "model": model.describe(),
        "counts": {"frames": len(indices), "status": counts, "valid_frames": counts["TRACK"] + counts["REINIT"],
                   "valid_points": pvalid},
        "timing_ms_per_frame": {"mean": float(np.mean(ms)), "p50": float(np.median(ms)),
                                "p95": float(np.percentile(ms, 95)), "max": float(np.max(ms))},
        "wall_s": wall,
        "outputs": {"samples": "samples.csv", "samples_format": "be-samples-csv/1", "log": "fit_log.csv",
                    "t_s": "the depth frame's t_mono_s (the recording host's CLOCK_MONOTONIC); convert to a run's "
                           "timebase with clock.csv"},
    }
    (out / "fit_meta.json").write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
    sess.close()
    return meta


def _versions() -> dict:
    v = {"python": platform.python_version(), "numpy": np.__version__, "fit": FIT_VERSION}
    for name in ("scipy", "cv2"):
        mod = sys.modules.get(name)
        v[name] = getattr(mod, "__version__", None) if mod else None
    try:
        from ... import __version__ as kit_version

        v["bounded_embodiment_kit"] = kit_version
    except ImportError:                                       # pragma: no cover
        pass
    return v


# ----- debug overlays ----------------------------------------------------------------------------------
def debug_overlay(sess: Session, depth_index: int, cam: Camera, model: ArmModel, res: FrameResult, path: Path):
    """The paired colour frame with the fitted model's outline (green: valid, red: invalid) and the observed
    points (green within 3 mm of the model, yellow within 6 mm, red beyond / not associated)."""
    import cv2

    ci = sess.color_for_depth(depth_index)
    img = sess.color_frame(ci) if ci is not None else None
    if img is None:
        img = np.zeros((720, 1280, 3), np.uint8)
    Kc, _ = sess.intrinsics("1280x720")
    ccam = cam.scaled(Kc, (img.shape[1], img.shape[0]))
    if res.q is not None:
        poses = model.poses(res.q)
        mask = np.zeros(img.shape[:2], np.uint8)
        for link in model.links:
            p = model.fields[link].points[::3]
            T = poses[link]
            Pc = ccam.to_cam(p @ T[:3, :3].T + T[:3, 3])
            ok = Pc[:, 2] > 0.05
            u, v = ccam.project_cam(Pc[ok])
            ui, vi = np.rint(u).astype(int), np.rint(v).astype(int)
            good = (ui >= 0) & (ui < img.shape[1]) & (vi >= 0) & (vi < img.shape[0])
            mask[vi[good], ui[good]] = 255
        mask = cv2.morphologyEx(cv2.dilate(mask, np.ones((3, 3), np.uint8)), cv2.MORPH_CLOSE,
                                np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, contours, -1, (0, 255, 0) if res.valid else (0, 0, 255), 2)
        for pid, p in res.points.items():
            pc = ccam.to_cam(p[None])
            if pc[0, 2] > 0:
                u, v = ccam.project_cam(pc)
                col = (255, 255, 0) if res.point_valid.get(pid) else (0, 0, 255)
                cv2.circle(img, (round(float(u[0])), round(float(v[0]))), 7, col, 2)
                cv2.putText(img, pid, (int(u[0]) + 9, int(v[0]) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
    obs, sol = res.observation, res.solution
    if obs is not None and obs.n_points:
        pc = ccam.to_cam(obs.points)
        u, v = ccam.project_cam(pc)
        if sol is not None:
            r = np.abs(sol.ev.r)
            col = np.where((sol.ev.associated & (r <= 0.003))[:, None], [0, 200, 0],
                           np.where((sol.ev.associated & (r <= 0.006))[:, None], [0, 220, 220], [0, 0, 230]))
        else:
            col = np.tile([0, 0, 230], (len(u), 1))
        for k in range(0, len(u), 2):
            cv2.circle(img, (int(u[k]), int(v[k])), 1, tuple(int(c) for c in col[k]), -1)
    text = (f"{res.status} {'valid' if res.valid else 'INVALID'}  rms {res.rms * 1e3:.2f} mm  inliers "
            f"{res.inliers}/{res.n_points}  fs {res.fs_frac:.2f}  it {res.iterations}  {res.ms:.0f} ms")
    cv2.putText(img, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 3, cv2.LINE_AA)
    cv2.putText(img, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (20, 20, 20), 1, cv2.LINE_AA)
    if res.reason:
        cv2.putText(img, res.reason[:150], (12, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
    cv2.imwrite(str(path), img)
