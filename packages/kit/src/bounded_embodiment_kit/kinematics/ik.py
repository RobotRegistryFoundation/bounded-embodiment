# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""Position inverse kinematics, for planning poses only (numpy only).

`be instrument plan` uses it to find joint values that put a machine's measured point near each target
of the accuracy check. A planned pose is where the robot's model says the point will be: it is never a
measurement and never a reference position (method §5.3 item 3: references are surveyed). Nothing here
commands a machine.

The solver is Levenberg-Marquardt on the point's position error (and, optionally, on keeping one axis of
one link horizontal), with a forward-difference Jacobian from :meth:`URDFModel.fk_batch`, clamped to the
URDF limits after every step.
"""
from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

import numpy as np

from .urdf import URDFModel

__all__ = ["IKResult", "solve_point"]


@dataclass(frozen=True)
class IKResult:
    q: dict[str, float]            # URDF values (radians) of every joint in the seed
    position: tuple[float, float, float]
    error_m: float
    level_deg: float | None        # the levelled axis's angle from horizontal (None: no level constraint)
    iterations: int
    ok: bool


def _residual(model: URDFModel, link: str, offset: np.ndarray, target: np.ndarray, joints: Sequence[str],
              base: Mapping[str, float], Q: np.ndarray, level: tuple[str, int] | None, level_weight_m: float
              ) -> np.ndarray:
    """Residuals for each row of ``Q`` (N, len(joints)): position error (m), then level_weight_m * axis_z."""
    q = {k: np.full(len(Q), v) for k, v in base.items()}
    for i, j in enumerate(joints):
        q[j] = Q[:, i]
    poses = model.fk_batch(q, n=len(Q))
    T = poses[link]
    p = np.einsum("nij,j->ni", T[:, :3, :3], offset) + T[:, :3, 3]
    r = [p - target]
    if level is not None:
        lk, axis = level
        r.append(level_weight_m * poses[lk][:, 2, axis][:, None])
    return np.concatenate(r, axis=1)


def solve_point(model: URDFModel, link: str, target: Sequence[float], *, joints: Sequence[str],
                seed: Mapping[str, float], offset: Sequence[float] | None = None,
                level: tuple[str, int] | None = None, level_weight_m: float = 0.05, tol_m: float = 1e-4,
                level_tol_deg: float = 2.0, max_iter: int = 200, limit_margin_rad: float = 0.0) -> IKResult:
    """Joint values (the ``joints``; every other joint stays at its ``seed`` value) that put the point fixed to
    ``link`` (``offset`` in the link frame) at ``target`` in the root frame. ``level = (link, axis)`` also asks
    that axis (0 x, 1 y, 2 z of that link) be horizontal. ``ok`` when the point is within ``tol_m`` and the
    axis within ``level_tol_deg`` of horizontal. ``limit_margin_rad`` keeps every joint that far inside its
    URDF limits."""
    limits = model.limits()
    lo = np.array([limits[j][0] for j in joints]) + limit_margin_rad
    hi = np.array([limits[j][1] for j in joints]) - limit_margin_rad
    if np.any(lo > hi):
        raise ValueError("limit_margin_rad leaves a joint no range")
    base = {k: float(v) for k, v in seed.items() if k in model.joints}
    x = np.clip(np.array([float(seed[j]) for j in joints]), lo, hi)
    off = np.zeros(3) if offset is None else np.asarray(offset, float)
    tgt = np.asarray(target, float)
    h = 1e-6
    lam = 1e-3
    n = len(joints)

    def res(X):
        return _residual(model, link, off, tgt, joints, base, X, level, level_weight_m)

    it = 0
    r = res(x[None])[0]
    cost = float(r @ r)
    for it in range(1, max_iter + 1):  # noqa: B007 - the count is reported
        X = np.vstack([x] + [x + h * np.eye(n)[i] for i in range(n)])
        R = res(X)
        J = (R[1:] - R[0]).T / h                       # (m, n)
        g = J.T @ R[0]
        A = J.T @ J
        improved = False
        for _ in range(12):
            step = np.linalg.solve(A + lam * (np.diag(np.diag(A)) + 1e-9 * np.eye(n)), -g)
            xn = np.clip(x + step, lo, hi)
            rn = res(xn[None])[0]
            cn = float(rn @ rn)
            if cn < cost:
                x, r, cost, lam = xn, rn, cn, max(lam / 3, 1e-9)
                improved = True
                break
            lam *= 4
        if not improved or cost < 1e-16:
            break
    q = {**base, **{j: float(v) for j, v in zip(joints, x, strict=True)}}
    pos = r[:3] + tgt
    err = float(np.linalg.norm(r[:3]))
    lvl = None
    if level is not None:
        z = float(r[3] / level_weight_m)
        lvl = math.degrees(math.asin(max(-1.0, min(1.0, z))))
    ok = err <= tol_m and (lvl is None or abs(lvl) <= level_tol_deg)
    return IKResult(q, (float(pos[0]), float(pos[1]), float(pos[2])), err, lvl, it, ok)
