"""URDF kinematics for any robot described by a URDF: parse it, pose its links, find named points.

numpy only, no robotics libraries. Three parts of the kit use it:

- the markerless instrument, to pose the robot's own meshes when it fits them to camera data;
- the runner's FK tripwire, which predicts where the envelope's points are from the robot's
  reported joint state;
- the state CSV, which logs the FK tool point next to the encoder values.

Values here are always URDF joint values: radians for revolute and continuous joints, metres for
prismatic ones. Converting a robot's own units (LeRobot degrees, with its calibration) into URDF
values is :class:`JointMap`'s job, and is configuration the lab declares, not something this module
guesses.

Frames follow the URDF: every link pose is a 4x4 homogeneous transform from the link's frame to the
root link's frame (for an arm, ``base_link``). An envelope point ``{"link": L, "offset_m": o}`` is the
point ``o`` (metres, in L's frame) carried into the root frame.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

__all__ = [
    "Joint",
    "JointMap",
    "URDFError",
    "URDFModel",
    "Visual",
    "axis_angle_matrix",
    "rpy_matrix",
    "transform",
]

JOINT_TYPES = ("revolute", "continuous", "prismatic", "fixed")


class URDFError(ValueError):
    """The URDF can't be used: malformed XML, a joint type this module doesn't support, or a
    structure that isn't a tree."""


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF ``rpy``: fixed-axis roll about x, then pitch about y, then yaw about z (R = Rz·Ry·Rx)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def axis_angle_matrix(axis: Sequence[float], theta: float) -> np.ndarray:
    """Rotation by ``theta`` radians about the unit vector ``axis`` (Rodrigues)."""
    x, y, z = _unit(axis)
    c, s = math.cos(theta), math.sin(theta)
    C = 1.0 - c
    return np.array(
        [
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ]
    )


def transform(R: np.ndarray | None = None, p: Sequence[float] | None = None) -> np.ndarray:
    """A 4x4 homogeneous transform from a rotation and a translation."""
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if p is not None:
        T[:3, 3] = p
    return T


def _unit(v: Sequence[float]) -> np.ndarray:
    a = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(a))
    if n == 0.0:
        raise URDFError("a joint axis has zero length")
    return a / n


def _floats(text: str | None, n: int, default: float = 0.0) -> list[float]:
    if text is None:
        return [default] * n
    parts = text.split()
    if len(parts) != n:
        raise URDFError(f"expected {n} numbers, got {text!r}")
    return [float(v) for v in parts]


def _origin(elem: ET.Element | None) -> np.ndarray:
    o = elem.find("origin") if elem is not None else None
    if o is None:
        return np.eye(4)
    return transform(rpy_matrix(*_floats(o.get("rpy"), 3)), _floats(o.get("xyz"), 3))


@dataclass(frozen=True)
class Joint:
    name: str
    type: str  # revolute | continuous | prismatic | fixed
    parent: str
    child: str
    origin: np.ndarray = field(repr=False)  # 4x4: child joint frame in the parent link frame, at q = 0
    axis: np.ndarray = field(repr=False)  # unit vector in the joint frame
    lower: float | None = None
    upper: float | None = None

    @property
    def actuated(self) -> bool:
        return self.type != "fixed"

    def motion(self, q: float) -> np.ndarray:
        """The 4x4 transform the joint adds at value ``q``."""
        if self.type in ("revolute", "continuous"):
            return transform(axis_angle_matrix(self.axis, q))
        if self.type == "prismatic":
            return transform(None, self.axis * q)
        return np.eye(4)


@dataclass(frozen=True)
class Visual:
    """One mesh of a link's visual geometry, as the URDF places it."""

    link: str
    path: Path  # resolved against the URDF's directory
    origin: np.ndarray = field(repr=False)  # 4x4: mesh frame in the link frame
    scale: tuple[float, float, float] = (1.0, 1.0, 1.0)


class URDFModel:
    """A parsed URDF: its tree of links and joints, its limits and its visual meshes."""

    def __init__(self, name: str, links: list[str], joints: list[Joint], visuals: list[Visual]):
        self.name = name
        self.links = list(links)
        self.joints = {j.name: j for j in joints}
        if len(self.joints) != len(joints):
            raise URDFError("two joints share a name")
        self.parent_joint: dict[str, Joint] = {}
        for j in joints:
            for end in (j.parent, j.child):
                if end not in self.links:
                    raise URDFError(f"joint {j.name!r} names unknown link {end!r}")
            if j.child in self.parent_joint:
                raise URDFError(f"link {j.child!r} has two parent joints; a URDF must be a tree")
            self.parent_joint[j.child] = j
        roots = [link for link in self.links if link not in self.parent_joint]
        if len(roots) != 1:
            raise URDFError(f"a URDF tree has exactly one root link, found {roots}")
        self.root = roots[0]
        # Links in an order where every parent comes before its children.
        children: dict[str, list[Joint]] = {link: [] for link in self.links}
        for j in joints:
            children[j.parent].append(j)
        order, stack = [], [self.root]
        while stack:
            link = stack.pop()
            order.append(link)
            stack.extend(j.child for j in reversed(children[link]))
        if len(order) != len(self.links):
            raise URDFError("some links are not reachable from the root (a cycle?)")
        self.order = order
        self.children = children
        self.actuated = [self.parent_joint[link].name for link in order[1:]
                         if self.parent_joint[link].actuated]
        self.visuals: dict[str, list[Visual]] = {link: [] for link in self.links}
        for v in visuals:
            self.visuals[v.link].append(v)

    # ----- construction -------------------------------------------------------------------------

    @classmethod
    def from_file(cls, path: str | Path) -> URDFModel:
        path = Path(path)
        return cls.from_string(path.read_text(encoding="utf-8"), base_dir=path.parent)

    @classmethod
    def from_string(cls, xml: str, base_dir: str | Path = ".") -> URDFModel:
        try:
            root = ET.fromstring(xml)
        except ET.ParseError as e:
            raise URDFError(f"not well-formed XML: {e}") from e
        if root.tag != "robot":
            raise URDFError(f"the root element is <{root.tag}>, not <robot>")
        base_dir = Path(base_dir)
        links, visuals = [], []
        for le in root.findall("link"):
            name = le.get("name")
            if not name:
                raise URDFError("a <link> has no name")
            links.append(name)
            for ve in le.findall("visual"):
                mesh = ve.find("geometry/mesh")
                if mesh is None or not mesh.get("filename"):
                    continue  # primitives (box, cylinder, sphere) are not used by the kit yet
                fname = mesh.get("filename")
                for prefix in ("package://", "file://"):
                    if fname.startswith(prefix):
                        fname = fname[len(prefix):]
                visuals.append(
                    Visual(
                        link=name,
                        path=(base_dir / fname).resolve(),
                        origin=_origin(ve),
                        scale=tuple(_floats(mesh.get("scale"), 3, 1.0)),
                    )
                )
        joints = []
        for je in root.findall("joint"):
            jtype = je.get("type")
            if jtype not in JOINT_TYPES:
                raise URDFError(f"joint {je.get('name')!r}: type {jtype!r} is not supported")
            parent, child = je.find("parent"), je.find("child")
            if parent is None or child is None:
                raise URDFError(f"joint {je.get('name')!r} lacks a parent or a child")
            if je.find("mimic") is not None:
                raise URDFError(f"joint {je.get('name')!r}: mimic joints are not supported")
            ax = je.find("axis")
            axis = _unit(_floats(ax.get("xyz"), 3)) if (ax is not None and jtype != "fixed") else (
                np.array([1.0, 0.0, 0.0]))
            lim = je.find("limit")
            lower = upper = None
            if lim is not None and jtype in ("revolute", "prismatic"):
                lower, upper = float(lim.get("lower", "0")), float(lim.get("upper", "0"))
            joints.append(
                Joint(
                    name=je.get("name"),
                    type=jtype,
                    parent=parent.get("link"),
                    child=child.get("link"),
                    origin=_origin(je),
                    axis=axis,
                    lower=lower,
                    upper=upper,
                )
            )
        return cls(root.get("name", ""), links, joints, visuals)

    # ----- queries ------------------------------------------------------------------------------

    def limits(self) -> dict[str, tuple[float, float]]:
        """URDF limits of the revolute and prismatic joints."""
        return {n: (j.lower, j.upper) for n, j in self.joints.items()
                if j.lower is not None and j.upper is not None}

    def chain(self, link: str) -> list[Joint]:
        """The joints from the root to ``link``, root first."""
        if link not in self.parent_joint and link != self.root:
            raise KeyError(link)
        out = []
        while link != self.root:
            j = self.parent_joint[link]
            out.append(j)
            link = j.parent
        return out[::-1]

    def fk(self, q: Mapping[str, float] | None = None, links: Iterable[str] | None = None
           ) -> dict[str, np.ndarray]:
        """Pose of every link (or of the named ``links``) in the root frame, as 4x4 transforms.

        Joints missing from ``q`` are at 0. Values outside the URDF limits are used as given: limits
        are for the caller to enforce.
        """
        q = q or {}
        unknown = [k for k in q if k not in self.joints]
        if unknown:
            raise KeyError(f"unknown joints {unknown}")
        wanted = None if links is None else set(links)
        poses = {self.root: np.eye(4)}
        for link in self.order[1:]:
            j = self.parent_joint[link]
            T = poses[j.parent] @ j.origin
            if j.actuated:
                T = T @ j.motion(float(q.get(j.name, 0.0)))
            poses[link] = T
        if wanted is None:
            return poses
        missing = wanted - poses.keys()
        if missing:
            raise KeyError(f"unknown links {sorted(missing)}")
        return {k: poses[k] for k in wanted}

    def fk_batch(self, q: Mapping[str, Sequence[float] | np.ndarray], n: int | None = None
                 ) -> dict[str, np.ndarray]:
        """Vectorised :meth:`fk`: each joint value is an array of length N; returns link -> (N, 4, 4).

        Joints missing from ``q`` are at 0; ``n`` is required only when ``q`` is empty.
        """
        arrays = {k: np.asarray(v, dtype=float).reshape(-1) for k, v in q.items()}
        unknown = [k for k in arrays if k not in self.joints]
        if unknown:
            raise KeyError(f"unknown joints {unknown}")
        sizes = {a.shape[0] for a in arrays.values()}
        if n is None:
            if len(sizes) != 1:
                raise ValueError("pass n, or give every joint the same number of values")
            n = sizes.pop()
        elif sizes and sizes != {n}:
            raise ValueError(f"every joint needs {n} values")
        poses = {self.root: np.broadcast_to(np.eye(4), (n, 4, 4)).copy()}
        for link in self.order[1:]:
            j = self.parent_joint[link]
            T = poses[j.parent] @ j.origin
            if j.actuated:
                th = arrays.get(j.name, np.zeros(n))
                M = np.broadcast_to(np.eye(4), (n, 4, 4)).copy()
                if j.type == "prismatic":
                    M[:, :3, 3] = th[:, None] * j.axis[None, :]
                else:
                    x, y, z = j.axis
                    c, s = np.cos(th), np.sin(th)
                    C = 1.0 - c
                    M[:, 0, 0], M[:, 0, 1], M[:, 0, 2] = c + x * x * C, x * y * C - z * s, x * z * C + y * s
                    M[:, 1, 0], M[:, 1, 1], M[:, 1, 2] = y * x * C + z * s, c + y * y * C, y * z * C - x * s
                    M[:, 2, 0], M[:, 2, 1], M[:, 2, 2] = z * x * C - y * s, z * y * C + x * s, c + z * z * C
                T = T @ M
            poses[link] = T
        return poses

    def point(self, link: str, offset_m: Sequence[float] | None = None,
              q: Mapping[str, float] | None = None) -> np.ndarray:
        """A point fixed to ``link`` (``offset_m`` in the link frame), in the root frame."""
        T = self.fk(q, links=[link])[link]
        o = np.zeros(3) if offset_m is None else np.asarray(offset_m, dtype=float)
        return T[:3, :3] @ o + T[:3, 3]

    def points(self, named: Mapping[str, tuple[str, Sequence[float] | None]],
               q: Mapping[str, float] | None = None) -> dict[str, np.ndarray]:
        """Several named points at once: ``{id: (link, offset_m or None)}`` -> ``{id: xyz}``."""
        poses = self.fk(q, links={link for link, _ in named.values()})
        out = {}
        for pid, (link, off) in named.items():
            T = poses[link]
            o = np.zeros(3) if off is None else np.asarray(off, dtype=float)
            out[pid] = T[:3, :3] @ o + T[:3, 3]
        return out


def envelope_points(envelope_doc: Mapping) -> dict[str, tuple[str, list[float] | None]]:
    """``{point id: (link, offset_m)}`` for the envelope points that name a link.

    Points without a ``link`` (a rover's reference point, say) are left out: an instrument measures
    those some other way.
    """
    out = {}
    for p in envelope_doc.get("points", []):
        if p.get("link"):
            out[p["id"]] = (p["link"], p.get("offset_m"))
    return out


@dataclass(frozen=True)
class JointMap:
    """Converts a robot's own joint values to URDF values: ``urdf = radians(sign·value + offset_deg)``.

    This is the convention ev03-harness used for LeRobot arms, whose positions are degrees in the
    arm's own calibration. Joints not listed map one to one (degrees to radians). The map is declared
    configuration (for bob, ``examples/envelopes/bob.kit.json``); an external instrument can check it,
    but this class never estimates it.
    """

    entries: Mapping[str, tuple[float, float]] = field(default_factory=dict)  # joint -> (sign, offset_deg)

    @classmethod
    def from_dict(cls, d: Mapping[str, Mapping[str, float]] | None) -> JointMap:
        entries = {}
        for j, m in (d or {}).items():
            sign = float(m.get("sign", 1))
            if sign not in (1.0, -1.0):
                raise ValueError(f"joint_map[{j!r}].sign must be 1 or -1")
            entries[j] = (sign, float(m.get("offset_deg", 0.0)))
        return cls(entries)

    def to_urdf(self, values_deg: Mapping[str, float]) -> dict[str, float]:
        out = {}
        for j, v in values_deg.items():
            sign, off = self.entries.get(j, (1.0, 0.0))
            out[j] = math.radians(sign * float(v) + off)
        return out

    def from_urdf(self, values_rad: Mapping[str, float]) -> dict[str, float]:
        out = {}
        for j, v in values_rad.items():
            sign, off = self.entries.get(j, (1.0, 0.0))
            out[j] = (math.degrees(float(v)) - off) * sign
        return out
