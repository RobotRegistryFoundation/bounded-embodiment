# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""Meshes for the markerless instrument: STL in, surface samples with normals out.

numpy only. Three things live here:

- :func:`read_stl` reads binary and ASCII STL;
- :func:`sample_surface` draws area-weighted surface samples with outward normals, deterministically
  (a seeded generator, then thinning to one sample per voxel, so the spacing is close to even);
- :func:`link_clouds` and :func:`link_triangles` put every :class:`~bounded_embodiment_kit.kinematics.Visual`
  of a URDF link into that link's frame, applying the visual's ``origin`` and ``scale``.

Units are whatever the mesh files use; the SO-101 meshes are in metres, as their URDF expects (its
visuals have no scale). Outward normals come from the triangle winding (counter-clockwise seen from
outside, as STL specifies); a mesh whose winding encloses a negative volume is flipped, and a mirroring
``scale`` (negative determinant) reverses the winding first.
"""
from __future__ import annotations

import hashlib
import re
import struct
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

__all__ = [
    "LinkCloud",
    "MeshError",
    "check_meshes",
    "describe",
    "file_sha256",
    "link_clouds",
    "link_triangles",
    "mesh_files",
    "missing_meshes",
    "read_stl",
    "sample_surface",
    "signed_volume",
    "thin",
    "triangle_areas_normals",
    "visual_triangles",
]


class MeshError(ValueError):
    """A mesh file that cannot be read as STL."""


_STL_DTYPE = np.dtype([("normal", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attr", "<u2")])
_CACHE: dict[tuple[str, int, int], np.ndarray] = {}


def read_stl(path: str | Path) -> np.ndarray:
    """The triangles of an STL file as an (n, 3, 3) float64 array of vertices, in the file's units.

    Binary STL is recognised by its size (84 + 50 n bytes), whatever its 80-byte header says, so a binary
    file whose header starts with ``solid`` still reads as binary. Anything else must be ASCII STL.
    Results are cached per file (path, size and modification time).
    """
    path = Path(path)
    st = path.stat()
    key = (str(path.resolve()), st.st_size, st.st_mtime_ns)
    hit = _CACHE.get(key)
    if hit is not None:
        return hit
    data = path.read_bytes()
    tris = _parse_stl(data, str(path))
    tris.flags.writeable = False
    _CACHE[key] = tris
    return tris


def _parse_stl(data: bytes, name: str = "<bytes>") -> np.ndarray:
    if len(data) >= 84:
        n = struct.unpack_from("<I", data, 80)[0]
        if 84 + 50 * n == len(data):
            rec = np.frombuffer(data, dtype=_STL_DTYPE, count=n, offset=84)
            return rec["v"].astype(np.float64)
    head = data[:512].lstrip()
    if not head[:5].lower() == b"solid":
        raise MeshError(f"{name}: neither binary STL (size does not match its triangle count) nor ASCII STL")
    text = data.decode("ascii", errors="replace")
    nums = re.findall(r"^\s*vertex\s+(\S+)\s+(\S+)\s+(\S+)", text, flags=re.MULTILINE | re.IGNORECASE)
    if not nums or len(nums) % 3:
        raise MeshError(f"{name}: ASCII STL with {len(nums)} vertices (need a multiple of 3, at least 3)")
    try:
        v = np.array(nums, dtype=np.float64)
    except ValueError as e:
        raise MeshError(f"{name}: bad number in an ASCII STL vertex: {e}") from e
    return v.reshape(-1, 3, 3)


def triangle_areas_normals(tris: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """(areas (n,), unit normals (n, 3)) from the winding; degenerate triangles get area 0, normal 0."""
    c = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n2 = np.linalg.norm(c, axis=1)
    normals = np.zeros_like(c)
    ok = n2 > 0
    normals[ok] = c[ok] / n2[ok, None]
    return 0.5 * n2, normals


def signed_volume(tris: np.ndarray) -> float:
    """Volume enclosed by a closed, consistently wound mesh (negative if wound inside out)."""
    return float(np.einsum("ij,ij->i", tris[:, 0], np.cross(tris[:, 1], tris[:, 2])).sum() / 6.0)


_VCACHE: dict[tuple, np.ndarray] = {}


def visual_triangles(visual) -> np.ndarray:
    """A :class:`Visual`'s triangles in its link's frame: scale, then the visual's origin; outward wound.
    Cached per (file, scale, origin); the returned array is read-only."""
    tris = read_stl(visual.path)
    key = (id(tris), tuple(visual.scale), visual.origin.tobytes())
    hit = _VCACHE.get(key)
    if hit is not None:
        return hit
    s = np.asarray(visual.scale, dtype=np.float64)
    v = tris * s
    if np.prod(s) < 0:                       # a mirror reverses the winding
        v = v[:, ::-1]
    if signed_volume(v) < 0:                 # wound inside out: flip so normals point outward
        v = v[:, ::-1]
    R, t = visual.origin[:3, :3], visual.origin[:3, 3]
    out = np.ascontiguousarray(v @ R.T + t)
    out.flags.writeable = False
    _VCACHE[key] = out
    return out


def link_triangles(model) -> dict[str, np.ndarray]:
    """``{link: (n, 3, 3) triangles in the link frame}`` for every link with mesh visuals."""
    out = {}
    for link in model.order:
        vis = model.visuals.get(link) or []
        if vis:
            out[link] = np.concatenate([visual_triangles(v) for v in vis], axis=0)
    return out


def thin(points: np.ndarray, spacing: float, *arrays: np.ndarray) -> tuple[np.ndarray, ...]:
    """Keep the first point in each cubic voxel of side ``spacing`` (in input order), and the same rows
    of ``arrays``. Deterministic; the result keeps the input order."""
    if len(points) == 0:
        return (points, *arrays)
    keys = np.floor(points / spacing).astype(np.int64)
    keys -= keys.min(axis=0)
    dims = keys.max(axis=0) + 1
    flat = (keys[:, 0] * dims[1] + keys[:, 1]) * dims[2] + keys[:, 2]
    _, first = np.unique(flat, return_index=True)
    first.sort()
    return (points[first], *(a[first] for a in arrays))


def sample_surface(tris: np.ndarray, spacing: float, seed: int = 0, oversample: float = 6.0
                   ) -> tuple[np.ndarray, np.ndarray]:
    """Surface samples about ``spacing`` apart, with the outward normal of the triangle each lies on.

    Area-weighted random points (``oversample`` per spacing^2 of area, from ``np.random.default_rng(seed)``)
    thinned to one per voxel of side ``spacing``. Same inputs, same output.
    """
    areas, normals = triangle_areas_normals(tris)
    total = float(areas.sum())
    if total <= 0:
        return np.zeros((0, 3)), np.zeros((0, 3))
    n = max(16, int(np.ceil(oversample * total / spacing ** 2)))
    rng = np.random.default_rng(seed)
    cdf = np.cumsum(areas)
    idx = np.searchsorted(cdf, rng.random(n) * cdf[-1], side="right")
    idx = np.minimum(idx, len(tris) - 1)
    r1, r2 = rng.random(n), rng.random(n)
    s = np.sqrt(r1)
    a, b, c = tris[idx, 0], tris[idx, 1], tris[idx, 2]
    pts = (1 - s)[:, None] * a + (s * (1 - r2))[:, None] * b + (s * r2)[:, None] * c
    pts, nrm = thin(pts, spacing, normals[idx])
    return pts, nrm


@dataclass(frozen=True)
class LinkCloud:
    """Surface samples of one link, in the link's frame."""

    link: str
    points: np.ndarray = field(repr=False)      # (n, 3)
    normals: np.ndarray = field(repr=False)     # (n, 3), unit, outward
    area: float = 0.0                           # total visual surface area


def link_clouds(model, spacing: float, seed: int = 0, links=None) -> dict[str, LinkCloud]:
    """``{link: LinkCloud}`` sampled ``spacing`` apart over every mesh visual of each link.

    Each visual gets its own seed (``seed`` + 7919 x its index in the URDF order of visuals), so adding a
    link does not change the samples of the others.
    """
    out = {}
    k = 0
    for link in model.order:
        vis = model.visuals.get(link) or []
        pts, nrm, area = [], [], 0.0
        for v in vis:
            tris = visual_triangles(v)
            p, n = sample_surface(tris, spacing, seed=seed + 7919 * k)
            k += 1
            pts.append(p)
            nrm.append(n)
            area += float(triangle_areas_normals(tris)[0].sum())
        if vis and (links is None or link in links):
            P, N = np.concatenate(pts), np.concatenate(nrm)
            P, N = thin(P, spacing * 0.5, N)          # where two visuals touch, drop exact duplicates
            out[link] = LinkCloud(link, P, N, area)
    return out


def mesh_files(model) -> list[Path]:
    """Every distinct mesh file the URDF's visuals use, in URDF order."""
    seen, out = set(), []
    for link in model.order:
        for v in model.visuals.get(link) or []:
            if v.path not in seen:
                seen.add(v.path)
                out.append(v.path)
    return out


def file_sha256(path: str | Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def missing_meshes(model) -> list[Path]:
    """Mesh files the URDF names that are not on disk (fetch them with tools/fetch_robot_assets.py)."""
    return [p for p in mesh_files(model) if not Path(p).is_file()]


def check_meshes(model, hint: str = "") -> None:
    missing = missing_meshes(model)
    if missing:
        names = ", ".join(p.name for p in missing[:4]) + (" ..." if len(missing) > 4 else "")
        raise MeshError(f"{len(missing)} mesh file(s) the URDF needs are missing ({names}) under "
                        f"{missing[0].parent}. " + (hint or "Fetch them: python tools/fetch_robot_assets.py "
                                                    "examples/robots/so101"))


def describe(model) -> Mapping[str, object]:
    """Mesh file names and sha256, for provenance records."""
    return {str(p.name): file_sha256(p) for p in mesh_files(model)}
