# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""`be instrument plan`: poses for the accuracy check's surveyed points.

    be instrument plan --envelope ENV --urdf URDF --kit-config KIT [--command-space CS] [--point tool]
        [--inset-mm 20] [--margin-mm 10] [--level LINK:AXIS] [--out PLAN.json]
        [--waypoints-out ev10-waypoints.json]

The method asks for 6 or more surveyed points spread through the measured point's keep-in, near both
extremes of each axis (§5.3 item 2). This plans eight: the corners of the keep-in region (the intersection
of the point's keep-in boxes), ``--inset-mm`` inside each face, each moved off any keep-out it falls in. For
each it finds joint values that put the point there by the robot's model (the URDF and the kit config's
joint map), close to the start pose, every point at least ``--margin-mm`` inside its zones and every joint
3 degrees inside its URDF limits. ``--level gripper_link:x`` also keeps that link's x axis horizontal: on
the SO-101 that is the fixed jaw's inner face vertical, so a plumb line can hang from it.

It prints the poses as a ``poses_deg`` block for the kit config, which `be goto-start --pose T1` drives
to. A planned position is only where the model says the point will go. The reference position is what
the lab surveys there afterwards, independently of the robot (§5.3 item 3); the plan never supplies one.
Nothing here moves a machine.

``--waypoints-out`` also writes the targets as an EV-10 waypoints file (waypoints 0.1, method §8.1):
each waypoint's position is the planned position (inverse kinematics, which §8.1 item 4 allows) and its
command the planned pose on every axis of ``--command-space``. EV-10 wants every waypoint 3 cm inside
the zones and 5 cm from the next, so use a larger inset (``--inset-mm 35 --margin-mm 30``).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections.abc import Callable, Mapping
from pathlib import Path

from bounded_embodiment import envelope as envelope_mod

__all__ = ["PlanError", "plan_targets", "run_plan"]

FORMAT = "be-target-plan/0.1"
EV10_INSIDE_M = 0.03         # max(waypoint_inside_tolerance_factor x tolerance_m, waypoint_inside_min_m)
EV10_SEPARATION_M = 0.05     # waypoint_separation_min_m
LIMIT_MARGIN_RAD = math.radians(3.0)
MOVE_STEP_M = 0.01


class PlanError(ValueError):
    """The targets cannot be planned (no keep-in, unreachable, bad config)."""


def keep_in_box(envelope, point: str) -> tuple[list[float], list[float]]:
    """The bounding box of the intersection of the point's keep-in zones (prisms by their bounding box)."""
    zones = [z for z in envelope.zones_for(point) if z.kind == "keep_in"]
    if not zones:
        raise PlanError(f"no keep-in zone applies to {point}: nothing to spread the points through")
    lo, hi = [-math.inf] * 3, [math.inf] * 3
    for z in zones:
        if z.box_min_m is not None:
            zlo, zhi = list(z.box_min_m), list(z.box_max_m)
        else:
            xs, ys = [v[0] for v in z.polygon_m], [v[1] for v in z.polygon_m]
            zlo = [min(xs), min(ys), z.z_m[0] if z.z_m else -math.inf]
            zhi = [max(xs), max(ys), z.z_m[1] if z.z_m else math.inf]
        lo = [max(a, b) for a, b in zip(lo, zlo, strict=True)]
        hi = [min(a, b) for a, b in zip(hi, zhi, strict=True)]
    if not all(math.isfinite(v) for v in lo + hi) or any(h <= low for low, h in zip(lo, hi, strict=True)):
        raise PlanError(f"{point}'s keep-in region is empty or unbounded on some axis")
    return lo, hi


def corner_targets(envelope, point: str, inset_m: float, margin_m: float) -> list[tuple[str, list[float], str]]:
    """Eight corners of the keep-in box inset by ``inset_m``; a corner within ``margin_m`` of leaving the
    allowed region (in a keep-out, say) has one coordinate moved to the box's centre, whichever leaves the
    most clearance."""
    lo, hi = keep_in_box(envelope, point)
    a = [lo[i] + inset_m for i in range(3)]
    b = [hi[i] - inset_m for i in range(3)]
    if any(a[i] >= b[i] for i in range(3)):
        raise PlanError(f"--inset-mm {inset_m * 1000:g} leaves no room inside {point}'s keep-in box")
    mid = [(a[i] + b[i]) / 2 for i in range(3)]
    out, k = [], 1
    for z in (a[2], b[2]):
        for x in (a[0], b[0]):
            for y in (a[1], b[1]):
                p = [x, y, z]
                note = ""
                if envelope.clearance(point, p) < margin_m:
                    alts = []
                    for i in range(3):
                        q = list(p)
                        q[i] = mid[i]
                        alts.append((envelope.clearance(point, q), i, q))
                    c, i, q = max(alts)
                    if c < margin_m:
                        raise PlanError(f"corner T{k} {p} is not clear of the keep-outs, nor is any move to the centre")
                    note = f"moved to {'xyz'[i]} = {q[i]:.3f} m, off a keep-out"
                    p = q
                out.append((f"T{k}", p, note))
                k += 1
    return out


def plan_targets(envelope, model, joint_map, start_deg: Mapping[str, float], *, point: str = "tool",
                 inset_m: float = 0.02, margin_m: float = 0.01, level: tuple[str, int] | None = None,
                 limits_deg: Mapping[str, tuple[float, float]] | None = None) -> list[dict]:
    """For each planned target: the joint values (URDF radians and the robot's own degrees) that put the
    point there by the model, the clearance of every point, and how far the target was moved to make
    it reachable. ``limits_deg``: the robot's own axis limits (from the command space), checked too."""
    from ..kinematics import envelope_points
    from ..kinematics.ik import solve_point

    pt = envelope.points.get(point)
    if pt is None or not pt.link:
        raise PlanError(f"{point} is not an envelope point with a link")
    named = envelope_points(envelope.normalized)
    missing = sorted({link for link, _ in named.values() if link not in model.links})
    if missing:
        raise PlanError(f"the envelope's points name links the URDF does not have: {missing}")
    joints = [j.name for j in model.chain(pt.link) if j.actuated]
    start = joint_map.to_urdf({k: float(v) for k, v in start_deg.items() if k in model.joints})
    absent = [j for j in joints if j not in start]
    if absent:
        raise PlanError(f"the kit config's start_pose_deg has no {', '.join(absent)}")
    seeds = [dict(start)]
    last = joints[-1]
    for d in (math.pi / 2, -math.pi / 2):
        s = dict(start)
        s[last] = start[last] + d
        seeds.append(s)
    lo, hi = keep_in_box(envelope, point)
    centre = [(lo[i] + hi[i]) / 2 for i in range(3)]
    out = []
    for label, target, note in corner_targets(envelope, point, inset_m, margin_m):
        best = None
        tgt = list(target)
        for _attempt in range(4):
            for seed in seeds:
                r = solve_point(model, pt.link, tgt, joints=joints, seed=seed, offset=pt.offset_m, level=level,
                                limit_margin_rad=LIMIT_MARGIN_RAD)
                if not r.ok:
                    continue
                pos = model.points(named, r.q)
                clear = {pid: envelope.clearance(pid, pos[pid]) for pid in pos}
                if min(clear.values()) < margin_m:
                    continue
                deg = joint_map.from_urdf({j: r.q[j] for j in joints})
                if limits_deg and any(j in limits_deg and not limits_deg[j][0] <= v <= limits_deg[j][1]
                                      for j, v in deg.items()):
                    continue
                dist = math.sqrt(sum((r.q[j] - start[j]) ** 2 for j in joints))
                if best is None or dist < best[0]:
                    best = (dist, r, clear, deg)
            if best is not None:
                break
            # Unreachable as asked: move the target 1 cm toward the box's centre and try again.
            v = [centre[i] - tgt[i] for i in range(3)]
            n = math.sqrt(sum(c * c for c in v))
            if n < MOVE_STEP_M:
                break
            tgt = [tgt[i] + MOVE_STEP_M * v[i] / n for i in range(3)]
        if best is None:
            raise PlanError(f"{label} at {[round(c, 3) for c in target]} m: no pose reaches it within "
                            f"{3 * MOVE_STEP_M * 100:g} cm with every point {margin_m * 100:g} cm inside its zones")
        _, r, clear, deg = best
        moved = math.dist(tgt, target)
        pose = {j: round(v, 1) for j, v in deg.items()}
        for k, v in start_deg.items():          # the rest (the gripper) as at the start pose
            pose.setdefault(k, v)
        out.append({"label": label, "planned_m": [round(c, 4) for c in tgt], "corner_note": note,
                    "moved_m": round(moved, 4), "pose_deg": pose,
                    "level_deg": None if r.level_deg is None else round(r.level_deg, 2),
                    "clearance_m": {k: round(v, 4) for k, v in clear.items()}})
    return out


def waypoints_doc(targets: list[dict], space, envelope, point: str) -> dict:
    """The plan as an ev10-waypoints.json document; PlanError if a waypoint would make the run
    INCONCLUSIVE (too near its zones' faces, or too near another waypoint)."""
    out = []
    for t in targets:
        pos = t["planned_m"]
        c = envelope.clearance(point, pos)
        if c < EV10_INSIDE_M:
            raise PlanError(f"{t['label']} is {c * 100:.1f} cm inside {point}'s zones; EV-10 wants "
                            f"{EV10_INSIDE_M * 100:g} cm (try --inset-mm 35 --margin-mm 30)")
        command = {}
        for a in space.axes:
            name = a.key.removesuffix(".pos")
            value = t["pose_deg"].get(a.key, t["pose_deg"].get(name))
            if value is None:
                raise PlanError(f"{t['label']}: the pose has no value for axis {a.key}")
            command[a.key] = float(value)
        out.append({"id": t["label"], "position_m": [float(v) for v in pos], "command": command})
    for i, a in enumerate(out):
        for b in out[i + 1:]:
            d = math.dist(a["position_m"], b["position_m"])
            if d < EV10_SEPARATION_M:
                raise PlanError(f"{a['id']} and {b['id']} are {d * 100:.1f} cm apart; EV-10 wants "
                                f"{EV10_SEPARATION_M * 100:g} cm")
    return {"waypoints_version": "0.1", "point": point,
            "description": "Planned by be instrument plan: positions by inverse kinematics through the kit "
                           "config's joint map; commands are those poses.",
            "waypoints": out}


def run_plan(args, *, say: Callable[[str], None] = print) -> int:
    from ..kinematics import JointMap, URDFModel

    try:
        envelope = envelope_mod.load(args.envelope)
        model = URDFModel.from_file(args.urdf)
        kit = json.loads(Path(args.kit_config).read_text(encoding="utf-8"))
        start = kit.get("start_pose_deg")
        if not isinstance(start, dict):
            raise PlanError("the kit config has no start_pose_deg")
        if envelope.frame != model.root:
            raise PlanError(f"the envelope's frame is {envelope.frame!r}, the URDF's root {model.root!r}")
        level = None
        if args.level:
            link, _, axis = args.level.partition(":")
            if link not in model.links or axis not in ("x", "y", "z"):
                raise PlanError(f"--level {args.level!r}: give LINK:AXIS with a URDF link and x, y or z")
            level = (link, "xyz".index(axis))
        limits, space = None, None
        if args.command_space:
            from bounded_embodiment.commandspace import load_command_space

            space = load_command_space(args.command_space)
            limits = {a.key.removesuffix(".pos"): (a.lo, a.hi) for a in space.axes}
        if args.waypoints_out and space is None:
            raise PlanError("--waypoints-out needs --command-space (the waypoints' commands name its axes)")
        point = args.point or ("tool" if "tool" in envelope.points else next(iter(envelope.points)))
        targets = plan_targets(envelope, model, JointMap.from_dict(kit.get("joint_map")), start, point=point,
                               inset_m=args.inset_mm / 1000, margin_m=args.margin_mm / 1000, level=level,
                               limits_deg=limits)
        tour = waypoints_doc(targets, space, envelope, point) if args.waypoints_out else None
    except (PlanError, envelope_mod.EnvelopeError, OSError, ValueError, KeyError) as e:
        print(f"be instrument plan: {e}", file=sys.stderr)
        return 2
    doc = {"format": FORMAT, "point": point, "frame": envelope.frame, "inset_m": args.inset_mm / 1000,
           "margin_m": args.margin_mm / 1000, "level": args.level, "envelope_hash": envelope.hash,
           "targets": targets,
           "poses_deg": {t["label"]: t["pose_deg"] for t in targets},
           "note": "Planned positions are where the robot's model puts the point. Survey where it actually is: "
                   "the planned numbers are never reference positions."}
    say(f"be instrument plan: {len(targets)} targets for {point}, {args.inset_mm:g} mm inside its keep-in")
    for t in targets:
        p = t["planned_m"]
        extra = f"  ({t['corner_note']})" if t["corner_note"] else ""
        extra += f"  moved {t['moved_m'] * 1000:.0f} mm to reach it" if t["moved_m"] else ""
        say(f"  {t['label']}  planned ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) m  closest zone face "
            f"{min(t['clearance_m'].values()) * 100:.1f} cm  pose " + json.dumps(t["pose_deg"]) + extra)
    say('Add this to the kit config, then drive to each with `be goto-start ... --pose T1`:')
    say(json.dumps({"poses_deg": doc["poses_deg"]}, indent=2))
    if args.out:
        Path(args.out).write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
        say(f"wrote {args.out}")
    if tour is not None:
        Path(args.waypoints_out).write_text(json.dumps(tour, indent=2) + "\n", encoding="utf-8")
        say(f"wrote {args.waypoints_out} ({len(tour['waypoints'])} EV-10 waypoints)")
    return 0


def add_parser(sub) -> argparse.ArgumentParser:
    p = sub.add_parser("plan", help="poses that put the measured point near each corner of its keep-in",
                       description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--envelope", required=True, help="the envelope JSON the run uses")
    p.add_argument("--urdf", required=True, help="the robot's URDF")
    p.add_argument("--kit-config", required=True, help="start_pose_deg and joint_map")
    p.add_argument("--command-space", help="also keep every pose inside the robot's own axis limits")
    p.add_argument("--point", default=None, help="the measured point (default: tool, else the envelope's first)")
    p.add_argument("--inset-mm", type=float, default=20.0, help="targets this far inside each face (default 20)")
    p.add_argument("--margin-mm", type=float, default=10.0,
                   help="every envelope point at least this far inside its zones (default 10)")
    p.add_argument("--level", default=None, metavar="LINK:AXIS",
                   help="keep this link axis horizontal (SO-101: gripper_link:x, the fixed jaw's face vertical)")
    p.add_argument("--out", default=None, help="also write the plan as JSON (be-target-plan/0.1)")
    p.add_argument("--waypoints-out", default=None, metavar="PATH",
                   help="also write the targets as EV-10 waypoints (needs --command-space)")
    return p
