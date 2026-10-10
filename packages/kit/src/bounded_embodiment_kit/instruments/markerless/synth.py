# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""Synthetic recordings with known truth, for testing the markerless instrument end to end.

:class:`Scene` renders what a fixed depth camera would see: the posed URDF meshes (z-buffered triangle
rasterisation, exact depth from each pixel ray's intersection with the triangle's plane; the meshes are
simplified by 0.3 mm vertex clustering, which changes the arm's depth by p95 0.01 mm), a table, two walls,
an optional clutter box, an optional sphere (a hand), and the ChArUco board lying on the table.
:func:`noisify` makes it look like passive stereo: depth noise sigma_z(z) (half white, half spatially
correlated), holes in blobs (biased to large flat faces of the plain plastic links), flying pixels at depth
edges, optional foreground fattening, millimetre quantisation. :func:`write_session` writes it all in the
recorder's own on-disk format (with the recorder's own writers), so `be fit register`, `be fit background`
and `be fit run` run on it unchanged. The truth goes to ``synth_truth.npz`` inside the session, which the
fitter must never open (a test checks that).

Frames: base_link is the URDF root; F is the measurements frame (origin on the table under the first
joint's axis); the camera frame is x right, y down, z forward.
"""
from __future__ import annotations

import json
import math
import re
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ...kinematics import URDFModel
from .. import record
from . import mesh
from .model import base_offset

__all__ = ["BACKGROUND_IDS", "NOMINAL_DEG", "NoiseModel", "Scene", "cluster_vertices", "lengthened_urdf",
           "look_at", "noisify", "nominal_trajectory", "write_motion_session", "write_registration",
           "write_session", "yawed"]

# ids in the label image: links are 0..n-1 in model order; the rest are negative
BACKGROUND_IDS = {"none": -1, "table": -2, "wall": -3, "clutter": -4, "board": -5, "sphere": -6}

# The benign pick-and-place of ev03-harness ev03/arm/nominal.py (LeRobot degrees), used here directly as
# URDF degrees: READY, then A_UP, A_DOWN, A_UP, B_UP, B_DOWN, B_UP, READY, with the gripper open/closed.
NOMINAL_DEG = {
    "READY": {"shoulder_pan": 0, "shoulder_lift": -40, "elbow_flex": 25, "wrist_flex": 75, "wrist_roll": 0},
    "A_UP": {"shoulder_pan": 25, "shoulder_lift": -60, "elbow_flex": 40, "wrist_flex": 55, "wrist_roll": 0},
    "A_DOWN": {"shoulder_pan": 25, "shoulder_lift": -40, "elbow_flex": 46, "wrist_flex": 50, "wrist_roll": 0},
    "B_UP": {"shoulder_pan": -25, "shoulder_lift": -60, "elbow_flex": 40, "wrist_flex": 55, "wrist_roll": 0},
    "B_DOWN": {"shoulder_pan": -25, "shoulder_lift": -40, "elbow_flex": 46, "wrist_flex": 50, "wrist_roll": 0},
}
OPEN_DEG, CLOSED_DEG = 30.0, 5.0
CYCLE = [("A_UP", OPEN_DEG, 0.0, OPEN_DEG), ("A_DOWN", OPEN_DEG, 0.5, CLOSED_DEG),
         ("A_UP", CLOSED_DEG, 0.0, CLOSED_DEG), ("B_UP", CLOSED_DEG, 0.0, CLOSED_DEG),
         ("B_DOWN", CLOSED_DEG, 0.5, OPEN_DEG), ("B_UP", OPEN_DEG, 0.0, OPEN_DEG),
         ("READY", OPEN_DEG, 0.3, OPEN_DEG)]
REST_DEG = {"shoulder_pan": 0, "shoulder_lift": -95, "elbow_flex": 95, "wrist_flex": 40, "wrist_roll": 0,
            "gripper": 0}


def look_at(eye, target, up=(0.0, 0.0, 1.0)) -> np.ndarray:
    """T (camera -> base) for a camera at ``eye`` looking at ``target``; camera x right, y down, z forward."""
    eye, target, up = (np.asarray(a, dtype=np.float64) for a in (eye, target, up))
    z = target - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, up)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x, y, z])
    T[:3, 3] = eye
    return T


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def box_triangles(lo, hi) -> np.ndarray:
    """12 outward-wound triangles of an axis-aligned box."""
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    v = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]], dtype=np.float64)
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
         (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return v[np.array(f)]


def cluster_vertices(tris: np.ndarray, cell: float) -> np.ndarray:
    """The mesh simplified by vertex clustering: vertices in the same ``cell``-sized voxel merge into their
    mean, and triangles that collapse are dropped. The SO-101 meshes have ~400k triangles, most far smaller
    than a pixel; at 0.3 mm this keeps ~150k and renders 2.5x faster, while the rendered depth of the arm
    changes by p95 0.01 mm, p99.9 0.14 mm, and 0.02 % of its silhouette pixels."""
    V = tris.reshape(-1, 3)
    keys = np.floor(V / cell).astype(np.int64)
    keys -= keys.min(axis=0)
    dims = keys.max(axis=0) + 1
    flat = (keys[:, 0] * dims[1] + keys[:, 1]) * dims[2] + keys[:, 2]
    _, inv = np.unique(flat, return_inverse=True)
    inv = inv.ravel()
    cnt = np.bincount(inv)
    rep = np.stack([np.bincount(inv, weights=V[:, k]) / cnt for k in range(3)], axis=1)
    F = inv.reshape(-1, 3)
    ok = (F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2])
    return rep[F[ok]]


def _raster(tris_c: np.ndarray, ids: np.ndarray, K: np.ndarray, w: int, h: int, zbuf: np.ndarray,
            idbuf: np.ndarray, normals_c: np.ndarray | None = None) -> None:
    """Z-buffer camera-frame triangles into zbuf/idbuf (flat, h*w): pixel centres inside a triangle get the
    depth of the pixel ray's intersection with the triangle's plane. Back faces are skipped."""
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    z0, z1, z2 = tris_c[:, 0, 2], tris_c[:, 1, 2], tris_c[:, 2, 2]
    if normals_c is None:
        v0 = tris_c[:, 0]
        n = np.cross(tris_c[:, 1] - v0, tris_c[:, 2] - v0)
        ok = (np.minimum(np.minimum(z0, z1), z2) > 0.05) & (np.einsum("ij,ij->i", n, v0) < 0)
    else:                                    # the caller culled back faces already
        n = normals_c
        ok = np.minimum(np.minimum(z0, z1), z2) > 0.05
    if not ok.all():
        tris_c, n, ids = tris_c[ok], n[ok], ids[ok]
    if not len(tris_c):
        return
    U = fx * tris_c[:, :, 0] / tris_c[:, :, 2] + cx
    V = fy * tris_c[:, :, 1] / tris_c[:, :, 2] + cy
    u0, u1, u2 = U[:, 0], U[:, 1], U[:, 2]
    v0_, v1_, v2_ = V[:, 0], V[:, 1], V[:, 2]
    umin = np.maximum(np.ceil(np.minimum(np.minimum(u0, u1), u2)), 0).astype(np.int64)
    umax = np.minimum(np.floor(np.maximum(np.maximum(u0, u1), u2)), w - 1).astype(np.int64)
    vmin = np.maximum(np.ceil(np.minimum(np.minimum(v0_, v1_), v2_)), 0).astype(np.int64)
    vmax = np.minimum(np.floor(np.maximum(np.maximum(v0_, v1_), v2_)), h - 1).astype(np.int64)
    nu, nv = umax - umin + 1, vmax - vmin + 1
    has = (nu > 0) & (nv > 0)                # a pixel centre lies in the bounding box
    sel = np.flatnonzero(has)
    if not sel.size:
        return
    nu, nv, umin, vmin = nu[sel], nv[sel], umin[sel], vmin[sel]
    cnt = nu * nv
    total = int(cnt.sum())
    loc = np.repeat(np.arange(sel.size), cnt)
    start = np.cumsum(cnt) - cnt
    k = np.arange(total) - start[loc]
    pu = umin[loc] + k % nu[loc]
    pv = vmin[loc] + k // nu[loc]
    tid = sel[loc]
    ax, ay, bx, by, qx, qy = u0[tid], v0_[tid], u1[tid], v1_[tid], u2[tid], v2_[tid]
    area = (bx - ax) * (qy - ay) - (by - ay) * (qx - ax)
    e0 = (qx - bx) * (pv - by) - (qy - by) * (pu - bx)
    e1 = (ax - qx) * (pv - qy) - (ay - qy) * (pu - qx)
    e2 = (bx - ax) * (pv - ay) - (by - ay) * (pu - ax)
    s = np.sign(area)
    eps = -1e-9 * np.abs(area)
    inside = (area != 0) & (e0 * s >= eps) & (e1 * s >= eps) & (e2 * s >= eps)
    tid, pu, pv = tid[inside], pu[inside], pv[inside]
    rx, ry = (pu - cx) / fx, (pv - cy) / fy
    nn = n[tid]
    denom = nn[:, 0] * rx + nn[:, 1] * ry + nn[:, 2]
    zz = np.einsum("ij,ij->i", nn, tris_c[tid, 0]) / np.where(np.abs(denom) > 1e-12, denom, 1e-12)
    good = zz > 0.05
    pix = (pv * w + pu)[good]
    zz, ii = zz[good], ids[tid[good]]
    order = np.lexsort((zz, pix))
    pix, zz, ii = pix[order], zz[order], ii[order]
    first = np.ones(len(pix), dtype=bool)
    first[1:] = pix[1:] != pix[:-1]
    pix, zz, ii = pix[first], zz[first], ii[first]
    better = zz < zbuf[pix]
    zbuf[pix[better]] = zz[better]
    idbuf[pix[better]] = ii[better]


@dataclass
class Scene:
    """A fixed camera looking at an arm on a table."""

    urdf: URDFModel
    T_bl_cam: np.ndarray
    K: np.ndarray                                   # depth (640x360) intrinsics
    size: tuple[int, int] = (640, 360)
    table_xy_F: tuple[tuple[float, float], tuple[float, float]] = ((-0.35, 0.80), (-0.55, 0.70))
    walls_F: tuple[tuple[str, float], ...] = (("x", -0.40), ("y", -0.60))   # vertical planes x_F = .. / y_F = ..
    clutter_F: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = field(default_factory=list)
    board_T_F: np.ndarray | None = None             # 4x4 board frame -> F (z into the table)
    board_size_m: tuple[float, float] = (0.15, 0.21)
    board_margin_m: float = 0.012                   # white paper around the printed pattern
    board_height_m: float = 0.005                   # board surface above the table
    render_cell_m: float = 0.0003                   # vertex clustering of the meshes for speed (0: off)

    def __post_init__(self):
        self.offset = base_offset(self.urdf)
        self.T_bl_F = self.offset.T_root_F()
        self.links = [link for link in self.urdf.order if self.urdf.visuals.get(link)]
        self.tris = mesh.link_triangles(self.urdf)
        if self.render_cell_m:
            self.tris = {k: cluster_vertices(t, self.render_cell_m) for k, t in self.tris.items()}
        self.normals = {k: np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]) for k, t in self.tris.items()}
        self.d0 = {k: np.einsum("ij,ij->i", self.normals[k], t[:, 0]) for k, t in self.tris.items()}
        self.T_cam_bl = np.linalg.inv(self.T_bl_cam)
        w, h = self.size
        u, v = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
        self.rays = np.column_stack([(u.ravel() - self.K[0, 2]) / self.K[0, 0],
                                     (v.ravel() - self.K[1, 2]) / self.K[1, 1], np.ones(w * h)])
        self._static = None

    # points and planes in the camera frame
    def _F_to_cam(self, P_F: np.ndarray) -> np.ndarray:
        T = self.T_cam_bl @ self.T_bl_F
        return P_F @ T[:3, :3].T + T[:3, 3]

    def _plane_depth(self, p_F, n_F) -> np.ndarray:
        T = self.T_cam_bl @ self.T_bl_F
        n = T[:3, :3] @ np.asarray(n_F, float)
        p = T[:3, :3] @ np.asarray(p_F, float) + T[:3, 3]
        den = self.rays @ n
        with np.errstate(divide="ignore", invalid="ignore"):
            z = (n @ p) / den
        return np.where((np.abs(den) > 1e-9) & (z > 0.05), z, np.inf)

    def _static_layer(self):
        """Depth and ids of the table and the walls (do not change between frames)."""
        if self._static is None:
            w, h = self.size
            zb = np.full(w * h, np.inf)
            ib = np.full(w * h, BACKGROUND_IDS["none"], dtype=np.int64)
            T = self.T_cam_bl @ self.T_bl_F
            Rinv, tinv = T[:3, :3].T, -T[:3, :3].T @ T[:3, 3]
            zt = self._plane_depth((0, 0, 0), (0, 0, 1))
            P_F = (self.rays * np.where(np.isfinite(zt), zt, 0)[:, None]) @ Rinv.T + tinv
            (x0, x1), (y0, y1) = self.table_xy_F
            on = np.isfinite(zt) & (P_F[:, 0] >= x0) & (P_F[:, 0] <= x1) & (P_F[:, 1] >= y0) & (P_F[:, 1] <= y1)
            zb[on], ib[on] = zt[on], BACKGROUND_IDS["table"]
            for axis, c in self.walls_F:
                n = (1.0, 0.0, 0.0) if axis == "x" else (0.0, 1.0, 0.0)
                p = (c, 0.0, 0.0) if axis == "x" else (0.0, c, 0.0)
                zw = self._plane_depth(p, n)
                Pw = (self.rays * np.where(np.isfinite(zw), zw, 0)[:, None]) @ Rinv.T + tinv
                ok = np.isfinite(zw) & (Pw[:, 2] >= -0.05) & (zw < zb)
                zb[ok], ib[ok] = zw[ok], BACKGROUND_IDS["wall"]
            for lo, hi in self.clutter_F:
                tri = self._F_to_cam(box_triangles(lo, hi).reshape(-1, 3)).reshape(-1, 3, 3)
                _raster(tri, np.full(len(tri), BACKGROUND_IDS["clutter"]), self.K, w, h, zb, ib)
            root = self.urdf.root
            if root in self.tris:                 # the root link never moves
                self._link_raster(root, self.links.index(root), np.eye(4), zb, ib)
            self._static = (zb, ib)
        return self._static[0].copy(), self._static[1].copy()

    def _link_raster(self, link: str, li: int, pose: np.ndarray, zb, ib, only=None):
        T = self.T_cam_bl @ pose
        R, t = T[:3, :3], T[:3, 3]
        c_link = -R.T @ t                                      # camera centre in the link frame
        front = self.d0[link] - self.normals[link] @ c_link < 0
        tri = self.tris[link][front]
        tc = (tri.reshape(-1, 3) @ R.T + t).reshape(-1, 3, 3)
        _raster(tc, np.full(len(tc), li, dtype=np.int64), self.K, self.size[0], self.size[1], zb, ib,
                normals_c=self.normals[link][front] @ R.T)

    def board_triangles_F(self) -> np.ndarray:
        """The board as a slab in F: the printed face at board z = 0, its underside at z = +height (board z
        points into the table). The board frame is right-handed, so the box keeps its outward winding."""
        bw, bh = self.board_size_m
        m = self.board_margin_m
        tri = box_triangles((-m, -m, 0.0), (bw + m, bh + m, self.board_height_m)).reshape(-1, 3)
        P = tri @ self.board_T_F[:3, :3].T + self.board_T_F[:3, 3]
        return P.reshape(-1, 3, 3)

    def render(self, q: Mapping[str, float], *, board: bool = False,
               spheres: Sequence[tuple[Sequence[float], float]] = ()) -> tuple[np.ndarray, np.ndarray]:
        """(depth (h, w) metres, inf = nothing; ids (h, w)) of the scene with the arm at q (URDF values).
        Sphere centres are in base_link."""
        w, h = self.size
        zb, ib = self._static_layer()
        poses = self.urdf.fk(q)
        for li, link in enumerate(self.links):
            if link != self.urdf.root:
                self._link_raster(link, li, poses[link], zb, ib)
        if board and self.board_T_F is not None:
            bt = self._F_to_cam(self.board_triangles_F().reshape(-1, 3)).reshape(-1, 3, 3)
            _raster(bt, np.full(len(bt), BACKGROUND_IDS["board"], dtype=np.int64), self.K, w, h, zb, ib)
        for c, r in spheres:
            cc = self.T_cam_bl[:3, :3] @ np.asarray(c, float) + self.T_cam_bl[:3, 3]
            d = self.rays / np.linalg.norm(self.rays, axis=1, keepdims=True)
            b = d @ cc
            disc = b * b - (cc @ cc - r * r)
            hit = disc > 0
            tt = np.where(hit, b - np.sqrt(np.where(hit, disc, 0)), np.inf)
            zz = tt * d[:, 2]
            ok = hit & (tt > 0.05) & (zz < zb)
            zb[ok], ib[ok] = zz[ok], BACKGROUND_IDS["sphere"]
        return zb.reshape(h, w), ib.reshape(h, w)


@dataclass(frozen=True)
class NoiseModel:
    """How the synthetic depth departs from the truth (defaults: the brief's passive-stereo model)."""

    sigma0_m: float = 0.0015           # sigma_z at 0.6 m, growing with z^2
    correlated_frac: float = 0.5       # share of the variance that is spatially correlated
    correlation_px: float = 2.0
    holes: float = 0.15                # fraction of arm pixels without depth, in blobs
    hole_blob_px: float = 4.0
    hole_flat_bias: float = 1.0        # holes prefer large flat faces
    other_holes: float = 0.03          # fraction of the rest (table, walls) without depth
    flying: float = 0.3                # share of depth-edge pixels given an in-between depth
    edge_m: float = 0.03
    fatten_px: int = 0                 # foreground fattening at left/right depth edges (stereo artefact)
    min_depth_m: float = 0.35
    max_depth_m: float = 4.0


DEFAULT_NOISE = NoiseModel()


def noisify(depth: np.ndarray, ids: np.ndarray, rng: np.random.Generator, noise: NoiseModel = DEFAULT_NOISE
            ) -> np.ndarray:
    """uint16 millimetres, 0 = no depth."""
    import cv2

    def blur(a, sigma):
        return cv2.GaussianBlur(a, (0, 0), sigmaX=sigma, borderType=cv2.BORDER_REFLECT)

    box5 = np.ones((5, 5), np.uint8)
    h, w = depth.shape
    none = ~np.isfinite(depth)
    zf = np.where(none, 0.0, depth)
    if noise.fatten_px:
        for _ in range(int(noise.fatten_px)):
            for shift in (1, -1):
                nb = np.roll(zf, shift, axis=1)
                take = (nb > 0) & ((zf <= 0) | (nb < zf - noise.edge_m))
                zf = np.where(take, nb, zf)
        none = zf <= 0
    s = noise.sigma0_m * (zf / 0.6) ** 2
    white = rng.standard_normal((h, w))
    corr = blur(rng.standard_normal((h, w)), noise.correlation_px)
    corr /= max(float(corr.std()), 1e-12)
    c = noise.correlated_frac
    zf = zf + s * (math.sqrt(1 - c) * white + math.sqrt(c) * corr)
    # flying pixels: at depth discontinuities, some pixels take a depth between the two sides
    if noise.flying > 0:
        for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            nb = np.roll(np.roll(zf, dy, axis=0), dx, axis=1)
            edge = (~none) & (nb > 0) & (np.abs(nb - zf) > noise.edge_m)
            pick = edge & (rng.random((h, w)) < noise.flying / 4)
            zf = np.where(pick, zf + rng.random((h, w)) * (nb - zf), zf)
    # holes in blobs: arm pixels on large flat faces first
    arm = ids >= 0
    blob = blur(rng.standard_normal((h, w)), noise.hole_blob_px)
    blob /= max(float(blob.std()), 1e-12)
    if noise.hole_flat_bias and arm.any():
        lap = np.abs(cv2.Laplacian(np.where(arm, depth, 0.0), cv2.CV_64F, ksize=1))
        curved = cv2.dilate((arm & (lap > 0.002)).astype(np.uint8), box5) > 0
        same = cv2.erode(np.where(arm, ids, -99).astype(np.int16), box5) == cv2.dilate(
            np.where(arm, ids, 99).astype(np.int16), box5)
        flat = arm & ~curved & same
        blob = blob + noise.hole_flat_bias * flat
    holes = np.zeros((h, w), dtype=bool)
    if noise.holes > 0 and arm.any():
        thr = np.quantile(blob[arm], 1 - noise.holes)
        holes |= arm & (blob > thr)
    rest = ~arm & ~none
    if noise.other_holes > 0 and rest.any():
        thr = np.quantile(blob[rest], 1 - noise.other_holes)
        holes |= rest & (blob > thr)
    zf = np.where(none | holes | (zf < noise.min_depth_m) | (zf > noise.max_depth_m), 0.0, zf)
    return np.clip(np.rint(zf * 1000.0), 0, 65535).astype(np.uint16)


def nominal_trajectory(urdf: URDFModel, seconds: float = 10.0, fps: float = 30.0, *, tool_speed_mps: float = 0.12,
                       start: Mapping[str, float] | None = None, cycle=None) -> tuple[np.ndarray, np.ndarray]:
    """(times (n,), Q (n, nj) URDF radians) of the pick-and-place: joint-space lines between the waypoints
    with a cosine speed profile, each segment slow enough that the mean tool speed is ``tool_speed_mps``."""
    joints = list(urdf.actuated)
    rad = {k: {j: math.radians(v) for j, v in d.items()} for k, d in NOMINAL_DEG.items()}
    here = {j: 0.0 for j in joints}
    here.update(start or {**rad["READY"], "gripper": math.radians(OPEN_DEG)})
    keys = []                                                  # (t, q dict)
    t = 0.0
    keys.append((t, dict(here)))
    tool = ("gripper_frame_link", None)
    cyc = cycle or CYCLE
    while t < seconds:
        for name, grip, dwell, grip_dwell in cyc:
            goal = {**here, **rad[name], "gripper": math.radians(grip)}
            us = np.linspace(0, 1, 30)
            P = np.array([urdf.points({"t": tool}, {j: here[j] + u * (goal[j] - here[j]) for j in joints})["t"]
                          for u in us])
            dur = max(0.4, float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum()) / tool_speed_mps)
            t += dur
            keys.append((t, dict(goal)))
            here = goal
            if dwell:
                t += dwell
                here = {**here, "gripper": math.radians(grip_dwell)}
                keys.append((t, dict(here)))
            if t >= seconds:
                break
    times = np.arange(0.0, seconds - 1e-9, 1.0 / fps)
    Q = np.zeros((len(times), len(joints)))
    kt = np.array([k[0] for k in keys])
    for i, tt in enumerate(times):
        s = int(np.clip(np.searchsorted(kt, tt, side="right") - 1, 0, len(keys) - 2))
        a, b = keys[s], keys[s + 1]
        tau = (tt - a[0]) / max(b[0] - a[0], 1e-9)
        e = 0.5 - 0.5 * math.cos(math.pi * min(max(tau, 0.0), 1.0))
        Q[i] = [a[1][j] + e * (b[1][j] - a[1][j]) for j in joints]
    return times, Q


def interpolate(qa: Mapping[str, float], qb: Mapping[str, float], joints, n: int) -> np.ndarray:
    out = np.zeros((n, len(joints)))
    for i in range(n):
        e = 0.5 - 0.5 * math.cos(math.pi * i / max(n - 1, 1))
        out[i] = [qa.get(j, 0.0) + e * (qb.get(j, 0.0) - qa.get(j, 0.0)) for j in joints]
    return out


def default_camera(urdf: URDFModel | None = None, *, distance=0.68, azimuth_deg=32.0, elevation_deg=36.0,
                   target_bl=(0.14, 0.06, 0.08)) -> np.ndarray:
    """T_bl_cam of a tripod front-left of the arm, looking down at it (as in the session runbook): the whole
    arm (rest pose and every nominal waypoint) and the board in view, no arm point nearer than 0.55 m."""
    a, e = math.radians(azimuth_deg), math.radians(elevation_deg)
    d = np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
    target = np.asarray(target_bl, float)
    return look_at(target + distance * d, target)


def default_K(width=1280, f=920.0) -> np.ndarray:
    s = width / 1280.0
    return np.array([[f * s, 0.0, 641.3 * s], [0.0, f * s, 359.2 * s], [0.0, 0.0, 1.0]])


def default_board_T_F(o_xy=(0.09, 0.28), yaw_deg=-3.0, height_m=0.005) -> np.ndarray:
    """Board frame -> F for a board lying face up: +x along robot +x (rotated by yaw), +y toward the robot's
    right (so +z, x cross y, points down into the table)."""
    R = rot_z(math.radians(yaw_deg)) @ np.diag([1.0, -1.0, -1.0])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = (o_xy[0], o_xy[1], height_m)
    return T


def board_image(cv2, spec, ppm: float = 4.0) -> np.ndarray:
    b = record.charuco_board(cv2, spec)
    return b.generateImage((round(spec["squares_x"] * spec["square_length_m"] * 1000 * ppm),
                            round(spec["squares_y"] * spec["square_length_m"] * 1000 * ppm)),
                           marginSize=0, borderBits=1)


def render_color(cv2, scene: Scene, ids: np.ndarray, depth: np.ndarray, K_color: np.ndarray,
                 size=(1280, 720), board_img=None, spec=None, rng=None, ppm: float = 4.0) -> np.ndarray:
    """A plain colour image: flat colours by surface, shaded by depth, the board warped in where visible."""
    W, H = size
    up_ids = cv2.resize(ids.astype(np.int16), (W, H), interpolation=cv2.INTER_NEAREST).astype(np.int64)
    up_z = cv2.resize(np.where(np.isfinite(depth), depth, 0).astype(np.float32), (W, H),
                      interpolation=cv2.INTER_NEAREST)
    img = np.zeros((H, W, 3), np.float32)
    colours = {BACKGROUND_IDS["none"]: (30, 30, 30), BACKGROUND_IDS["table"]: (70, 110, 150),
               BACKGROUND_IDS["wall"]: (200, 200, 195), BACKGROUND_IDS["clutter"]: (160, 60, 40),
               BACKGROUND_IDS["board"]: (235, 235, 235), BACKGROUND_IDS["sphere"]: (120, 150, 210)}
    for k, c in colours.items():
        img[up_ids == k] = c
    arm = up_ids >= 0
    img[arm] = (40, 190, 235)
    shade = np.clip(1.25 - 0.5 * up_z, 0.6, 1.2)[..., None]
    img = img * np.where(arm[..., None], shade, 1.0)
    if board_img is not None and scene.board_T_F is not None:
        T = scene.T_cam_bl @ scene.T_bl_F @ scene.board_T_F
        s = 1.0 / (ppm * 1000.0)
        S = np.array([[s, 0, 0.5 * s], [0, s, 0.5 * s], [0, 0, 1.0]])
        Hm = K_color @ np.column_stack([T[:3, 0], T[:3, 1], T[:3, 3]]) @ S
        warped = cv2.warpPerspective(board_img, Hm, (W, H), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                                     borderValue=255)
        cover = cv2.warpPerspective(np.full(board_img.shape, 255, np.uint8), Hm, (W, H), flags=cv2.INTER_NEAREST,
                                    borderMode=cv2.BORDER_CONSTANT, borderValue=0) > 0
        show = cover & (up_ids == BACKGROUND_IDS["board"])
        g = warped.astype(np.float32)[..., None] * (235.0 / 255.0)
        img = np.where(show[..., None], np.repeat(g, 3, axis=2) + 10.0, img)
    if rng is not None:
        img = img + rng.normal(0.0, 2.0, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def measurements_text(board_T_F: np.ndarray, height_m: float, board_size_m=(0.15, 0.21), round_mm: float | None = None,
                      bars_mm=(100.0, 100.0)) -> str:
    """measurements.txt filled in from the true board pose (rounded to ``round_mm`` if given)."""
    def F(p):
        v = board_T_F[:3, :3] @ np.asarray(p, float) + board_T_F[:3, 3]
        x, y = v[0] * 1000, v[1] * 1000
        if round_mm:
            x, y = round(x / round_mm) * round_mm, round(y / round_mm) * round_mm
        return x, y
    o, x, y = F((0, 0, 0)), F((board_size_m[0], 0, 0)), F((0, board_size_m[1], 0))
    vals = {"printed_100mm_bar_horizontal_mm": bars_mm[0], "printed_100mm_bar_vertical_mm": bars_mm[1],
            "board_width_mm": board_size_m[0] * 1000, "board_height_mm": board_size_m[1] * 1000,
            "corner_O_x_mm": o[0], "corner_O_y_mm": o[1], "corner_X_x_mm": x[0], "corner_X_y_mm": x[1],
            "corner_Y_x_mm": y[0], "corner_Y_y_mm": y[1], "board_surface_above_table_mm": height_m * 1000,
            "board_face": "up", "oakd_direction": "synthetic", "notes": "written by synth.py"}
    lines = []
    for line in record.MEASUREMENTS_TEMPLATE.splitlines():
        key = line.split("=", 1)[0].strip() if "=" in line and not line.lstrip().startswith("#") else None
        if key in vals:
            v = vals[key]
            text = f"{v:.4f}" if isinstance(v, float) else str(v)
            rest = line.split("=", 1)[1]
            comment = rest[rest.index("#"):] if "#" in rest else ""
            lines.append(f"{key} = {text}    {comment}".rstrip())
        else:
            lines.append(line)
    return "\n".join(lines) + "\n"


def write_session(root, scene: Scene, *, times: np.ndarray, qs: np.ndarray, events: Sequence[tuple[float, str]] = (),
                  board_frames: np.ndarray | None = None, color: str = "board", color_every: int = 15,
                  noise: NoiseModel = DEFAULT_NOISE, seed: int = 0, spheres=None, blackout=None,
                  K_color: np.ndarray | None = None, measurements: str | None = None, named=None,
                  depth_fn=None) -> dict:
    """Write a recording in the recorder's format and return the truth.

    ``times`` are t_mono_s of the frames (30 fps, say) and ``qs`` the URDF joint values (radians, in
    ``urdf.actuated`` order). ``board_frames`` (bool per frame) puts the board on the table. Colour frames
    are written for every board frame and every ``color_every``-th other frame (``color="all"``: every
    frame). ``spheres``: per-frame list of (centre_bl, radius). ``blackout``: bool per frame, depth all
    zero. ``depth_fn(i, depth_mm) -> depth_mm`` may change a frame after the noise."""
    import cv2

    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    (root / "oakd").mkdir(exist_ok=True)
    rng = np.random.default_rng(seed)
    joints = list(scene.urdf.actuated)
    K_color = default_K(1280) if K_color is None else K_color
    spec = dict(record.BOARD_SPEC)
    bimg = board_image(cv2, spec)
    stats = {n: record.StreamStats(n, 30) for n in ("color", "depth")}
    cw = record.MjpegWriter(root / record.PATHS["color_stream"], root / record.PATHS["color_index"],
                            record.COLOR_COLUMNS, stats["color"], max_queue=1_000_000, jpeg_quality=92)
    dw = record.DepthChunkWriter(root / record.PATHS["depth_dir"], root / record.PATHS["depth_index"],
                                 stats["depth"], chunk_frames=150, max_queue=1_000_000)
    cw.start()
    dw.start()
    n = len(times)
    board_frames = np.zeros(n, bool) if board_frames is None else np.asarray(board_frames, bool)
    named = named or {"tool": ("gripper_frame_link", None), "elbow": ("lower_arm_link", None),
                      "wrist": ("wrist_link", None)}
    truth_pts = {k: np.zeros((n, 3)) for k in named}
    try:
        for i, t in enumerate(times):
            q = {j: float(v) for j, v in zip(joints, qs[i], strict=True)}
            sph = spheres[i] if spheres is not None else ()
            depth, ids = scene.render(q, board=bool(board_frames[i]), spheres=sph)
            d = noisify(depth, ids, rng, noise)
            if blackout is not None and blackout[i]:
                d = np.zeros_like(d)
            if depth_fn is not None:
                d = depth_fn(i, d)
            dw.put(((i, float(t)), d, (None, None, float(t) + 0.02, 0)))
            if color == "all" or board_frames[i] or (color_every and i % color_every == 0):
                img = render_color(cv2, scene, ids, depth, K_color, board_img=bimg if board_frames[i] else None,
                                   spec=spec, rng=rng)
                cw.put(((i, float(t)), img, (None, None, float(t) + 0.02, 0)))
            pts = scene.urdf.points(named, q)
            for k in named:
                truth_pts[k][i] = pts[k]
    finally:
        cw.finish()
        dw.finish()
    if cw.error or dw.error:
        raise RuntimeError(f"writing the synthetic session failed: {cw.error or dw.error}")
    with open(root / record.PATHS["events"], "w", encoding="utf-8") as f:
        f.write(",".join(record.EVENT_COLUMNS) + "\n")
        for k, (t, label) in enumerate(sorted(events)):
            f.write(f"{k},{t:.6f},{t + 1.7e9:.6f},{label},keyboard\n")
    t0, t1 = float(times[0]) - 0.5, float(times[-1]) + 0.5
    with open(root / record.PATHS["clock"], "w", encoding="utf-8") as f:
        f.write(",".join(record.CLOCK_COLUMNS) + "\n")
        f.write(f"start,{t0 + 1.7e9:.6f},{t0:.6f},0.000001\nstop,{t1 + 1.7e9:.6f},{t1:.6f},0.000001\n")
    K640 = scene.K
    meta = {"format": record.FORMAT, "status": "finished", "t0_mono_s": t0, "t0_wall_s": t0 + 1.7e9,
            "recorder": {"name": "bounded_embodiment_kit.instruments.markerless.synth"},
            "settings": {"color_fps": 30.0, "depth_fps": 30.0, "wrist_fps": 30.0},
            "oakd": {"intrinsics": {"rgb_1280x720": K_color.tolist(), "rgb_640x360": K640.tolist(),
                                    "rgb_distortion": [0.0] * 14}},
            "wrist": None, "board": record.BOARD_SPEC,
            "synthetic": {"noise": {k: getattr(noise, k) for k in noise.__dataclass_fields__}, "seed": seed}}
    record.write_json_atomic(root / record.PATHS["meta"], meta)
    if measurements is not None:
        (root / record.PATHS["measurements"]).write_text(measurements, encoding="utf-8")
    truth = {"times": np.asarray(times, float), "q": np.asarray(qs, float), "T_bl_cam": scene.T_bl_cam,
             **{f"point_{k}": v for k, v in truth_pts.items()}}
    np.savez(root / "synth_truth.npz", **truth)
    return {"times": np.asarray(times, float), "q": np.asarray(qs, float), "points": truth_pts,
            "T_bl_cam": scene.T_bl_cam.copy()}


def write_registration(session, T_bl_cam: np.ndarray, note: str = "synthetic truth", out=None) -> Path:
    """Write ``registration.json`` (default ``<session>/fit``) holding a given T_bl_cam (for tests that skip
    the board)."""
    p = Path(out) if out is not None else Path(session) / "fit"
    p.mkdir(parents=True, exist_ok=True)
    doc = {"format": "be-fit-registration/0.1", "T_bl_cam": np.asarray(T_bl_cam).tolist(), "note": note}
    (p / "registration.json").write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
    return p / "registration.json"


CLUTTER_F = [((0.02, 0.22, 0.0), (0.08, 0.30, 0.10))]      # a box beside the arm, behind the board's place


def write_motion_session(root, urdf: URDFModel, *, nominal_s: float = 2.0, still_s: float = 0.5,
                         fps: float = 30.0, seed: int = 1, noise: NoiseModel = DEFAULT_NOISE, scene_urdf=None,
                         hand=None, blackout=None, registration: bool = True) -> dict:
    """A session without the board: the arm resting folded (``background_start`` .. ``background_end``),
    a sweep to READY (``goto_start``), then ``nominal_s`` of the pick-and-place (``nominal_1``). The
    registration is the true camera pose (written to ``fit/registration.json`` unless ``registration`` is
    False). ``scene_urdf`` renders a different arm from the one the fitter is given (a model mismatch).
    ``hand``: (centre_bl, radius) of a sphere that appears at ``nominal_1`` (after the background).
    ``blackout``: (start, end) seconds after ``nominal_1`` with no depth at all."""
    joints = list(urdf.actuated)
    T = default_camera(urdf)
    scene = Scene(scene_urdf or urdf, T, default_K(640), clutter_F=CLUTTER_F)
    rest = {j: math.radians(REST_DEG.get(j, 0.0)) for j in joints}
    ready = {**{j: math.radians(v) for j, v in NOMINAL_DEG["READY"].items()}, "gripper": math.radians(OPEN_DEG)}
    n_still = max(3, round(still_s * fps))
    _, q_nom = nominal_trajectory(urdf, nominal_s, fps)
    Q = np.concatenate([np.tile([rest[j] for j in joints], (n_still, 1)), interpolate(rest, ready, joints, n_still),
                        q_nom])
    t0 = 100.0
    times = t0 + np.arange(len(Q)) / fps
    t_nom = float(times[2 * n_still])
    events = [(t0 - 0.01, "background_start"), (float(times[n_still - 1]) + 0.01, "background_end"),
              (float(times[n_still]) - 0.01, "goto_start"), (t_nom - 0.01, "nominal_1"),
              (float(times[-1]) + 0.01, "nominal_1_done")]
    spheres = [([hand] if hand is not None and t >= t_nom else []) for t in times]
    dark = None if blackout is None else np.array([blackout[0] <= t - t_nom < blackout[1] for t in times])
    truth = write_session(root, scene, times=times, qs=Q, events=events, seed=seed, noise=noise, spheres=spheres,
                          blackout=dark, color_every=30)
    if registration:
        write_registration(root, T)
    truth["t_nominal"] = t_nom
    truth["blackout_mask"] = dark
    return truth


def lengthened_urdf(path, delta_m: float = 0.002, joints=("elbow_flex", "wrist_flex")) -> URDFModel:
    """The URDF at ``path`` with the origin of each named joint moved ``delta_m`` farther from its parent
    (for the SO-101: the upper arm and the forearm each ``delta_m`` longer), to render an arm that is not
    quite the model the fitter uses."""
    path = Path(path)
    text = path.read_text(encoding="utf-8")
    for joint in joints:
        m = re.search(rf'<joint name="{re.escape(joint)}".*?<origin xyz="([^"]+)"', text, flags=re.DOTALL)
        if m is None:
            raise ValueError(f"{path.name} has no joint {joint!r} with an origin")
        xyz = np.array([float(v) for v in m.group(1).split()])
        new = xyz * (np.linalg.norm(xyz) + delta_m) / np.linalg.norm(xyz)
        text = text[:m.start(1)] + " ".join(f"{v:.9g}" for v in new) + text[m.end(1):]
    return URDFModel.from_string(text, base_dir=path.parent)


def yawed(T_bl_cam: np.ndarray, yaw_deg: float, about_bl) -> np.ndarray:
    """T_bl_cam turned by ``yaw_deg`` about the vertical line through ``about_bl`` (a registration error)."""
    R = rot_z(math.radians(yaw_deg))
    o = np.asarray(about_bl, float)
    Ty = np.eye(4)
    Ty[:3, :3] = R
    Ty[:3, 3] = o - R @ o
    return Ty @ np.asarray(T_bl_cam, float)
