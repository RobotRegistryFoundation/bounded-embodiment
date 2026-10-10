# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""The robot as the markerless fitter sees it: its URDF, its surface samples and, per link, a field that
answers "which surface sample is nearest this point" in constant time.

Why fields and not a k-d tree per link. The fitter re-associates every observed point with the model at
every evaluation (ICP style), and the global search scores thousands of candidate poses. A k-d tree query
of 4000 points against each of 7 links costs about 25 ms here, so 15 iterations would take ~0.4 s per
frame. A voxel grid (2 mm) in each link's frame that stores, for every voxel, a surface sample near it
turns each association into a gather: under 2 ms for all links. Each voxel holding samples keeps the one
nearest its centre; every other voxel takes the sample of the nearest such voxel (an exact Euclidean
distance transform over the grid, built in about 2 s for the SO-101). That is the nearest sample to within
about a voxel; the residual is then computed against that sample exactly (its position and outward
normal), so the voxel size does not quantise the distance, it only decides which neighbouring sample is
used, which on a smooth surface gives the same plane. Where it matters (the polish of global-search
candidates), :meth:`ArmModel.tree` gives the exact nearest sample instead.

``ArmModel`` also holds the joint bounds (the URDF limits widened by ``joint_margin_deg``), the ROI the
fitter keeps points in (derived from the URDF's own reach), and where the table is (the lowest point of
the root link's meshes).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ...kinematics import URDFModel
from . import mesh

__all__ = ["ArmModel", "BaseOffset", "LinkField", "base_offset"]


@dataclass(frozen=True)
class BaseOffset:
    """Where the root link sits relative to the table frame F (origin on the table under the first joint's
    axis, axes parallel to the root link's): ``P_root = P_F + (pan_x, pan_y, z_min)``."""

    pan_x: float
    pan_y: float
    z_min: float                 # lowest point of the root link's meshes, in the root frame (the table)
    joint: str                   # the joint whose axis defines "under the axis"
    axis_root: tuple[float, float, float]
    explanation: str

    def T_root_F(self) -> np.ndarray:
        T = np.eye(4)
        T[:3, 3] = (self.pan_x, self.pan_y, self.z_min)
        return T


def base_offset(urdf: URDFModel) -> BaseOffset:
    """Compute (not assume) where the table frame F's origin is in the root link's frame.

    F's origin is on the table directly under the first actuated joint's axis (for the SO-101,
    shoulder_pan). Its x, y are that joint's origin in the root frame (the axis must be vertical); its z is
    the lowest vertex of the root link's visual meshes, which rest on the table.
    """
    first = None
    for link in urdf.order[1:]:
        j = urdf.parent_joint[link]
        if j.actuated and j.parent == urdf.root:
            first = j
            break
    if first is None:
        raise ValueError("no actuated joint is attached to the root link")
    axis = first.origin[:3, :3] @ first.axis
    if abs(abs(axis[2]) - 1.0) > 1e-3:
        raise ValueError(f"{first.name}'s axis in {urdf.root} is {axis.round(4).tolist()}, not vertical: the "
                         "table frame 'under the axis' is not defined")
    tris = mesh.link_triangles(urdf).get(urdf.root)
    if tris is None:
        raise ValueError(f"the root link {urdf.root} has no mesh visuals, so the table height is unknown")
    z_min = float(tris[..., 2].min())
    px, py = float(first.origin[0, 3]), float(first.origin[1, 3])
    text = (f"The measurements frame F has its origin on the table directly under the {first.name} axis. "
            f"URDF {urdf.root} is not F: {first.name}'s joint origin is ({px:.6f}, {py:.6g}, "
            f"{first.origin[2, 3]:.6f}) m in {urdf.root}, so the axis is the vertical line x = {px:.4f}, "
            f"y = {py:.3g}; the lowest point of {urdf.root}'s meshes (its underside, on the table) is "
            f"z = {z_min:.4f} m. So {urdf.root} = F + ({px:.4f}, {py:.3g}, {z_min:.4f}) m: its origin is "
            f"{px * 1000:.1f} mm behind the axis and {-z_min * 1000:.1f} mm above the table.")
    return BaseOffset(px, py, z_min, first.name, tuple(float(a) for a in axis), text)


@dataclass
class LinkField:
    """Nearest-surface-sample lookup in one link's frame."""

    link: str
    points: np.ndarray = field(repr=False)     # (n, 3) dense samples, link frame
    normals: np.ndarray = field(repr=False)    # (n, 3) unit outward normals
    origin: np.ndarray = field(repr=False)     # (3,) centre of voxel (0, 0, 0)
    voxel: float = 0.002
    shape: tuple[int, int, int] = (0, 0, 0)
    nearest: np.ndarray = field(repr=False, default=None)   # (nx*ny*nz,) int32 sample index, -1 = none
    dist: np.ndarray = field(repr=False, default=None)      # (nx*ny*nz,) float32 voxel centre -> sample, m

    def __post_init__(self):
        self.pn = np.ascontiguousarray(np.hstack([self.points, self.normals]))   # (n, 6): one gather for both

    @property
    def lo(self) -> np.ndarray:
        return self.origin - 0.5 * self.voxel

    @property
    def hi(self) -> np.ndarray:
        return self.origin + (np.asarray(self.shape) - 0.5) * self.voxel

    def inside(self, P: np.ndarray) -> np.ndarray:
        """Indices of the points (link frame) inside the field's box."""
        lo, hi = self.lo, self.hi
        return np.flatnonzero((P[:, 0] >= lo[0]) & (P[:, 0] < hi[0]) & (P[:, 1] >= lo[1]) & (P[:, 1] < hi[1])
                              & (P[:, 2] >= lo[2]) & (P[:, 2] < hi[2]))

    def index(self, P: np.ndarray) -> np.ndarray:
        """Flat voxel index of each point (link frame), -1 outside the grid."""
        g = np.rint((P - self.origin) / self.voxel).astype(np.int64)
        nx, ny, nz = self.shape
        inside = (g[:, 0] >= 0) & (g[:, 0] < nx) & (g[:, 1] >= 0) & (g[:, 1] < ny) & (g[:, 2] >= 0) & (g[:, 2] < nz)
        flat = (g[:, 0] * ny + g[:, 1]) * nz + g[:, 2]
        return np.where(inside, flat, -1)

    def lookup(self, P: np.ndarray) -> np.ndarray:
        """Index of the surface sample assigned to the voxel each point falls in (link frame), -1 when
        none is within the field's margin. Approximate near edges; the fitter can replace it with an exact
        nearest-sample query (:meth:`ArmModel.tree`)."""
        flat = self.index(P)
        out = np.full(len(P), -1, dtype=np.int64)
        ok = flat >= 0
        out[ok] = self.nearest[flat[ok]]
        return out

    def lookup_inside(self, P: np.ndarray) -> np.ndarray:
        """:meth:`lookup` for points already known to be inside the box (:meth:`inside`)."""
        g = ((P - self.origin) * (1.0 / self.voxel) + 0.5).astype(np.int64)
        _, ny, nz = self.shape
        return self.nearest[(g[:, 0] * ny + g[:, 1]) * nz + g[:, 2]]

    def distance(self, P: np.ndarray, far: float) -> np.ndarray:
        """Approximate distance to the surface (voxel-centre value), ``far`` outside the margin."""
        flat = self.index(P)
        out = np.full(len(P), far, dtype=np.float64)
        ok = flat >= 0
        d = self.dist[flat[ok]].astype(np.float64)
        out[ok] = np.where(np.isfinite(d), d, far)
        return out


def _build_field(link: str, pts: np.ndarray, nrm: np.ndarray, voxel: float, margin: float) -> LinkField:
    """Nearest sample for every voxel centre: each voxel that holds samples keeps the one nearest its
    centre; an exact Euclidean distance transform then gives every voxel its nearest such voxel."""
    from scipy import ndimage

    lo = pts.min(axis=0) - margin
    hi = pts.max(axis=0) + margin
    shape = tuple(math.ceil((h - lo_) / voxel) + 1 for lo_, h in zip(lo, hi, strict=True))
    nx, ny, nz = shape
    g = np.rint((pts - lo) / voxel).astype(np.int64)
    flat = (g[:, 0] * ny + g[:, 1]) * nz + g[:, 2]
    centre = lo + g * voxel
    d2 = np.einsum("ij,ij->i", pts - centre, pts - centre)
    order = np.lexsort((d2, flat))                    # by voxel, then by distance to its centre
    first = np.ones(len(order), dtype=bool)
    first[1:] = flat[order][1:] != flat[order][:-1]
    rep = np.full(nx * ny * nz, -1, dtype=np.int64)
    rep[flat[order][first]] = order[first]
    occupied = (rep >= 0).reshape(shape)
    edt, idx = ndimage.distance_transform_edt(~occupied, return_indices=True)
    near_vox = (idx[0].astype(np.int64) * ny + idx[1]) * nz + idx[2]
    nearest = rep[near_vox.ravel()]
    del idx, near_vox
    within = edt.ravel() * voxel <= margin
    nearest = np.where(within, nearest, -1).astype(np.int32)
    gi, gj, gk = np.unravel_index(np.arange(nx * ny * nz), shape)
    cen = lo + np.column_stack([gi, gj, gk]) * voxel
    ok = nearest >= 0
    dist = np.full(nx * ny * nz, np.inf, dtype=np.float32)
    dist[ok] = np.linalg.norm(cen[ok] - pts[nearest[ok]], axis=1)
    return LinkField(link, pts, nrm, lo, voxel, shape, nearest, dist)


class ArmModel:
    """The URDF plus what the fitter needs from its meshes. Building it takes a second or two."""

    def __init__(self, urdf: URDFModel | str | Path, *, dense_spacing: float = 0.0015,
                 sparse_spacing: float = 0.006, fs_spacing: float = 0.009, search_spacing: float = 0.012,
                 voxel: float = 0.002, margin: float = 0.04, joint_margin_deg: float = 5.0, seed: int = 0,
                 roi_margin: float = 0.05):
        if not isinstance(urdf, URDFModel):
            self.urdf_path = Path(urdf).resolve()
            urdf = URDFModel.from_file(self.urdf_path)
        else:
            self.urdf_path = None
        mesh.check_meshes(urdf)
        self.urdf = urdf
        self.params = {"dense_spacing_m": dense_spacing, "sparse_spacing_m": sparse_spacing,
                       "fs_spacing_m": fs_spacing, "search_spacing_m": search_spacing, "voxel_m": voxel,
                       "margin_m": margin,
                       "joint_margin_deg": joint_margin_deg, "seed": seed, "roi_margin_m": roi_margin}
        self.joints = list(urdf.actuated)
        self.nj = len(self.joints)
        lims = urdf.limits()
        m = math.radians(joint_margin_deg)
        self.lo = np.array([lims.get(j, (-math.pi, math.pi))[0] - m for j in self.joints])
        self.hi = np.array([lims.get(j, (-math.pi, math.pi))[1] + m for j in self.joints])
        self.limits_lo = np.array([lims.get(j, (-math.pi, math.pi))[0] for j in self.joints])
        self.limits_hi = np.array([lims.get(j, (-math.pi, math.pi))[1] for j in self.joints])
        dense = mesh.link_clouds(urdf, dense_spacing, seed=seed)
        self.links = [link for link in urdf.order if link in dense]
        self.link_index = {link: i for i, link in enumerate(self.links)}
        self.root = urdf.root
        self.moving = [link for link in self.links if link != self.root]
        # which joints move which link
        self.moves = np.zeros((len(self.links), self.nj), dtype=bool)
        for li, link in enumerate(self.links):
            for j in urdf.chain(link) if link != urdf.root else []:
                if j.actuated:
                    self.moves[li, self.joints.index(j.name)] = True
        self.joint_child = [urdf.joints[j].child for j in self.joints]
        self.fields = {link: _build_field(link, dense[link].points, dense[link].normals, voxel, margin)
                       for link in self.links}
        self.area = {link: dense[link].area for link in self.links}
        # coarser sets are the dense samples thinned (deterministic, near-even spacing)
        self.sparse = {link: mesh.thin(dense[link].points, sparse_spacing, dense[link].normals)
                       for link in self.links}
        self.fs = {link: mesh.thin(dense[link].points, fs_spacing, dense[link].normals) for link in self.links}
        self.search = {link: mesh.thin(dense[link].points, search_spacing, dense[link].normals)
                       for link in self.links}
        self._trees = {}
        for link in self.links:
            self.tree(link)
        self.offset = base_offset(urdf)
        self.table_z = self.offset.z_min
        self._roi()

    # ----- geometry helpers -------------------------------------------------------------------------
    def q_dict(self, q) -> dict[str, float]:
        return {j: float(v) for j, v in zip(self.joints, q, strict=True)}

    def poses(self, q) -> dict[str, np.ndarray]:
        return self.urdf.fk(self.q_dict(q))

    def axes(self, poses) -> tuple[np.ndarray, np.ndarray]:
        """(axes (nj, 3), origins (nj, 3)) of the actuated joints in the root frame at these poses."""
        A = np.empty((self.nj, 3))
        O = np.empty((self.nj, 3))
        for k, j in enumerate(self.joints):
            T = poses[self.joint_child[k]]
            A[k] = T[:3, :3] @ self.urdf.joints[j].axis
            O[k] = T[:3, 3]
        return A, O

    def posed_points(self, poses, which: str = "sparse", links=None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(points, normals, link index) of the chosen sample set ("sparse", "fs" or "search"), posed, in the
        root frame."""
        src = {"sparse": self.sparse, "fs": self.fs, "search": self.search}[which]
        P, N, L = [], [], []
        for li, link in enumerate(self.links):
            if links is not None and link not in links:
                continue
            p, n = src[link]
            T = poses[link]
            P.append(p @ T[:3, :3].T + T[:3, 3])
            N.append(n @ T[:3, :3].T)
            L.append(np.full(len(p), li, dtype=np.int64))
        return np.concatenate(P), np.concatenate(N), np.concatenate(L)

    def tree(self, link: str):
        """A k-d tree over a link's dense samples (built on first use), for exact association."""
        t = self._trees.get(link)
        if t is None:
            from scipy.spatial import cKDTree

            t = self._trees[link] = cKDTree(self.fields[link].points)
        return t

    def _roi(self, n: int = 2000):
        """The space the arm can occupy, from its own URDF: a vertical cylinder about the first joint's axis
        (radius = the farthest sample from the axis over n deterministic random poses, plus roi_margin) from
        the table up to the highest sample plus roi_margin."""
        rng = np.random.default_rng(12345)
        Q = rng.uniform(self.limits_lo, self.limits_hi, size=(n, self.nj))
        poses = self.urdf.fk_batch({j: Q[:, k] for k, j in enumerate(self.joints)})
        rmax, zmax = 0.0, -np.inf
        ax, ay = self.offset.pan_x, self.offset.pan_y
        for link in self.links:
            p = self.search[link][0]
            T = poses[link]
            W = np.matmul(T[:, :3, :3], p.T).transpose(0, 2, 1) + T[:, None, :3, 3]
            r = np.hypot(W[..., 0] - ax, W[..., 1] - ay)
            rmax = max(rmax, float(r.max()))
            zmax = max(zmax, float(W[..., 2].max()))
        m = self.params["roi_margin_m"]
        self.roi = {"axis_xy_m": [ax, ay], "radius_m": rmax + m, "z_top_m": zmax + m, "table_z_m": self.table_z,
                    "about": f"cylinder about the {self.offset.joint} axis: farthest model point from the axis over "
                             f"{n} random poses within the URDF limits ({rmax:.3f} m) + {m:.3f} m; up to the highest "
                             f"({zmax:.3f} m) + {m:.3f} m"}

    def in_roi(self, P: np.ndarray, above_table: float) -> np.ndarray:
        ax, ay = self.roi["axis_xy_m"]
        r2 = (P[:, 0] - ax) ** 2 + (P[:, 1] - ay) ** 2
        return (r2 <= self.roi["radius_m"] ** 2) & (P[:, 2] >= self.table_z + above_table) & \
            (P[:, 2] <= self.roi["z_top_m"])

    def describe(self) -> dict:
        return {"links": {link: {"dense_samples": len(self.fields[link].points),
                                 "sparse_samples": len(self.sparse[link][0]),
                                 "area_m2": round(self.area[link], 6),
                                 "field_shape": list(self.fields[link].shape)} for link in self.links},
                "joints": self.joints, "bounds_rad": {j: [float(a), float(b)] for j, a, b in
                                                      zip(self.joints, self.lo, self.hi, strict=True)},
                "roi": self.roi, "base_offset": {"pan_x_m": self.offset.pan_x, "pan_y_m": self.offset.pan_y,
                                                 "z_min_m": self.offset.z_min, "joint": self.offset.joint},
                "params": self.params}
