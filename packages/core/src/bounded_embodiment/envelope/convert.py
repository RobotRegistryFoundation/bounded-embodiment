"""Convert other envelope formats to envelope 0.2. Conversion only goes one way.

``from_harness_arm`` reads the arm envelope JSON of ev03-harness (``ev03/arm/envelope.py``,
``ArmEnvelope.load``), whose semantics are:

- ``keep_in`` bounds the tool point only;
- ``link_keep_in``, when present, bounds the link points (``link_points``, by default the elbow and
  wrist joint centres), and only then are the links checked at all;
- every ``keep_out`` box is a solid object, so it bounds the tool and, when links are checked, the
  links as well;
- ``max_tool_speed_mps`` limits the tool point; ``max_joint_speed_dps`` caps the arm joints.

Tolerances, the start pose and the joint map are not envelope content: they go to the returned kit
config. ``_comment`` keys go to ``notes``.
"""

from __future__ import annotations

import copy
import math

from .load import load

#: The ev03-harness tool point: the origin of gripper_frame_link (kinematics.py TOOL_FRAME).
TOOL_POINT = {
    "id": "tool",
    "description": (
        "Tool point: the origin of gripper_frame_link in the SO-101 URDF (so101_new_calib.urdf), "
        "fixed to the gripper 98 mm along the wrist_roll axis from that joint's frame origin (and "
        "8 mm off the axis), toward the jaw tips."
    ),
    "link": "gripper_frame_link",
}

#: The ev03-harness link points (kinematics.py LINK_JOINTS): each the origin of a joint frame.
LINK_POINTS = {
    "elbow": {
        "id": "elbow",
        "description": (
            "Elbow joint centre: the origin of the elbow_flex joint frame in the SO-101 URDF (link "
            "lower_arm_link), on the elbow_flex axis at the end of the upper arm."
        ),
        "link": "lower_arm_link",
    },
    "wrist": {
        "id": "wrist",
        "description": (
            "Wrist joint centre: the origin of the wrist_flex joint frame in the SO-101 URDF (link "
            "wrist_link), on the wrist_flex axis at the end of the forearm."
        ),
        "link": "wrist_link",
    },
}

DEFAULT_LINK_POINTS = ("elbow", "wrist")

_HARNESS_ARM_KEYS = (
    "keep_in",
    "keep_out",
    "link_keep_in",
    "max_tool_speed_mps",
    "max_joint_speed_dps",
    "tolerance_m",
    "tolerance_speed",
    "start_pose_deg",
    "joint_map",
    "link_points",
)
_KIT_KEYS = ("tolerance_m", "tolerance_speed", "start_pose_deg", "joint_map")


def from_harness_arm(
    doc: dict,
    *,
    machine_id: str,
    stop: dict,
    heartbeat: dict,
    frame: str = "base_link",
    machine_class: str = "arm",
    notes: str | None = None,
) -> tuple[dict, dict]:
    """Convert an ev03-harness arm envelope to ``(envelope 0.2 document, kit_config)``.

    ``stop`` and ``heartbeat`` are required because the harness format declares neither; they are
    copied in as given. ``notes`` is appended to the generated notes. The envelope is validated
    with :func:`bounded_embodiment.envelope.load` before it is returned (EnvelopeError if it is not
    valid, for example because of a bad ``stop``); a harness file with keys this converter does not
    know raises ValueError rather than losing them."""
    if not isinstance(doc, dict):
        raise ValueError("a harness arm envelope is a JSON object")  # noqa: TRY004 (bad input file)
    comments: list[str] = []
    _take_comments(doc, "", comments)
    unknown = sorted(k for k in doc if not k.startswith("_") and k not in _HARNESS_ARM_KEYS)
    if unknown:
        raise ValueError(f"unknown keys in the harness arm envelope: {', '.join(unknown)}")
    for key in ("keep_in", "max_tool_speed_mps"):
        if key not in doc:
            raise ValueError(f"the harness arm envelope has no {key!r}")

    keep_in = _box(doc["keep_in"], "keep_in", comments)
    keep_outs = [_box(b, f"keep_out_{i}", comments) for i, b in enumerate(doc.get("keep_out", []))]
    link_box = (
        _box(doc["link_keep_in"], "link_keep_in", comments) if "link_keep_in" in doc else None
    )
    link_names = list(dict.fromkeys(doc.get("link_points", DEFAULT_LINK_POINTS)))
    if link_box is not None and not link_names:
        raise ValueError("link_keep_in is declared but link_points is empty, so it bounds nothing")
    for name in link_names:
        if name != "tool" and name not in LINK_POINTS:
            raise ValueError(
                f"link point {name!r} is not one the harness computes "
                f"({', '.join(['tool', *LINK_POINTS])})"
            )

    points = [dict(TOOL_POINT, max_speed_mps=doc["max_tool_speed_mps"])]
    zones = [{"id": "keep_in", "kind": "keep_in", "applies_to": ["tool"], "box": keep_in}]
    if link_box is not None:
        points.extend(dict(LINK_POINTS[n]) for n in link_names if n != "tool")
        zones.append(
            {"id": "link_keep_in", "kind": "keep_in", "applies_to": link_names, "box": link_box}
        )
    for i, b in enumerate(keep_outs):
        zones.append({"id": f"keep_out_{i}", "kind": "keep_out", "applies_to": ["*"], "box": b})

    motion: dict = {"max_speed_mps": doc["max_tool_speed_mps"]}
    joint_speed = doc.get("max_joint_speed_dps")
    if isinstance(joint_speed, dict):
        _take_comments(joint_speed, "max_joint_speed_dps", comments)
        joints = [
            {"name": j, "max_speed_radps": math.radians(v)}
            for j, v in joint_speed.items()
            if not j.startswith("_")
        ]
        if joints:
            motion["joints"] = joints
    elif joint_speed:  # the harness reads 0 and null alike as "no cap"
        motion["max_joint_speed_radps"] = math.radians(joint_speed)

    text = [
        "Converted from an ev03-harness arm envelope.",
        "As in the harness, keep_in bounds the tool point only"
        + (f", link_keep_in bounds the link points ({', '.join(link_names)})" if link_box else "")
        + (", and every keep_out applies to every point." if keep_outs else "."),
    ]
    if link_box is None and "link_points" in doc:
        text.append(
            "The harness file lists link_points but no link_keep_in, so the harness never "
            "checked the links and no link points are declared here."
        )
    if "max_joint_speed_radps" in motion:
        text.append(
            "The harness applied max_joint_speed_dps to the five arm joints, not the gripper."
        )
    text.append(
        "The stop and heartbeat blocks were supplied at conversion; the harness format declares "
        "neither."
    )
    text.extend(comments)
    if notes:
        text.append(notes)

    envelope = {
        "envelope_version": "0.2",
        "machine": {"id": machine_id, "class": machine_class},
        "frame": frame,
        "points": points,
        "workspace": {"zones": zones},
        "motion": motion,
        "stop": copy.deepcopy(stop),
        "heartbeat": copy.deepcopy(heartbeat),
        "notes": " ".join(text),
    }
    load(envelope)

    kit_config: dict = {"machine_id": machine_id, "converted_from": "ev03-harness arm envelope"}
    kit_comments: list[str] = []
    for key in _KIT_KEYS:
        if key in doc:
            kit_config[key] = _strip_comments(copy.deepcopy(doc[key]), key, kit_comments)
    kept_out = (
        "Kept out of the envelope: tolerance is a parameter of the test method (the "
        "profile's values govern scoring); the start pose and the joint map are kit "
        "configuration."
    )
    kit_config["notes"] = " ".join([kept_out, *kit_comments])
    return envelope, kit_config


def _box(b: dict, name: str, comments: list[str]) -> dict:
    if not isinstance(b, dict) or "lo" not in b or "hi" not in b:
        raise ValueError(f"{name} must be a box with lo and hi")
    unknown = sorted(k for k in b if not k.startswith("_") and k not in ("lo", "hi"))
    if unknown:
        raise ValueError(f"unknown keys in {name}: {', '.join(unknown)}")
    _take_comments(b, name, comments)
    return {"min_m": list(b["lo"]), "max_m": list(b["hi"])}


def _take_comments(d: dict, where: str, out: list[str]) -> None:
    for k, v in d.items():
        if k.startswith("_"):
            if not isinstance(v, str):
                raise ValueError(f"{where + '.' if where else ''}{k} must be a string comment")
            out.append(f"{where}: {_sentence(v)}" if where else _sentence(v))


def _strip_comments(node, where: str, out: list[str]):
    """Copy of a kit-config value without its ``_`` keys, whose text goes to ``out``."""
    if isinstance(node, dict):
        _take_comments(node, where, out)
        return {
            k: _strip_comments(v, f"{where}.{k}", out)
            for k, v in node.items()
            if not k.startswith("_")
        }
    return node


def _sentence(s: str) -> str:
    s = s.strip()
    return s if s.endswith((".", "!", "?")) else s + "."
