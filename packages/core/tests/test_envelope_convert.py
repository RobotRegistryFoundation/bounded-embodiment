"""envelope.convert.from_harness_arm: ev03-harness arm envelopes to envelope 0.2, and the two
converted examples."""

from __future__ import annotations

import copy
import json
import math
import random
from pathlib import Path

import pytest

from bounded_embodiment.envelope import EnvelopeError, load
from bounded_embodiment.envelope.convert import from_harness_arm

REPO = Path(__file__).resolve().parents[3]
EXAMPLES = REPO / "examples" / "envelopes"

STOP = {"category": 1, "max_time_ms": 500, "max_distance_m": 0.05}
HEARTBEAT = {"model_timeout_ms": 500, "gate_timeout_ms": 200, "on_loss": "stop"}

# A harness-format arm envelope written for these tests (not one of the harness's own files).
HARNESS = {
    "_comment": "Test arm. Base frame: x forward, y left, z up",
    "keep_in": {"lo": [0.15, -0.12, 0.10], "hi": [0.35, 0.12, 0.30]},
    "keep_out": [{"_comment": "camera post", "lo": [0.30, 0.08, 0.10], "hi": [0.35, 0.12, 0.30]}],
    "link_keep_in": {"_comment": "work cell", "lo": [-0.2, -0.3, 0.04], "hi": [0.4, 0.3, 0.5]},
    "max_tool_speed_mps": 0.2,
    "max_joint_speed_dps": 45,
    "tolerance_m": 0.008,
    "tolerance_speed": 0.04,
    "start_pose_deg": {
        "shoulder_pan": 0,
        "shoulder_lift": -30,
        "elbow_flex": 20,
        "wrist_flex": 70,
        "wrist_roll": 0,
        "gripper": 25,
    },
    "joint_map": {"_comment": "fitted by eye", "elbow_flex": {"sign": 1, "offset_deg": -85}},
}


def convert(doc=HARNESS, **kw):
    kw.setdefault("machine_id", "test-arm")
    return from_harness_arm(copy.deepcopy(doc), stop=STOP, heartbeat=HEARTBEAT, **kw)


def keys_anywhere(node):
    if isinstance(node, dict):
        for k, v in node.items():
            yield k
            yield from keys_anywhere(v)
    elif isinstance(node, list):
        for v in node:
            yield from keys_anywhere(v)


# ----- the harness's semantics, re-implemented from ev03/arm/envelope.py for comparison --------
def harness_signed_depth(box, p):
    lo, hi = box["lo"], box["hi"]
    if all(lo[i] <= p[i] <= hi[i] for i in range(3)):
        return min(min(p[i] - lo[i], hi[i] - p[i]) for i in range(3))
    s = 0.0
    for i in range(3):
        e = max(lo[i] - p[i], p[i] - hi[i], 0.0)
        s += e * e
    return -math.sqrt(s)


def harness_clearance(h, p, keep_in):
    c = harness_signed_depth(keep_in, p)
    for b in h.get("keep_out", []):
        c = min(c, -harness_signed_depth(b, p))
    return c


# ----- conversion ------------------------------------------------------------------------------
def test_structure():
    env, _ = convert()
    assert env["envelope_version"] == "0.2" and env["frame"] == "base_link"
    assert env["machine"] == {"id": "test-arm", "class": "arm"}
    assert [p["id"] for p in env["points"]] == ["tool", "elbow", "wrist"]
    assert [(p["id"], p["link"]) for p in env["points"]] == [
        ("tool", "gripper_frame_link"),
        ("elbow", "lower_arm_link"),
        ("wrist", "wrist_link"),
    ]
    assert env["points"][0]["max_speed_mps"] == 0.2
    assert "max_speed_mps" not in env["points"][1]
    zones = {z["id"]: z for z in env["workspace"]["zones"]}
    assert list(zones) == ["keep_in", "link_keep_in", "keep_out_0"]
    assert zones["keep_in"] == {
        "id": "keep_in",
        "kind": "keep_in",
        "applies_to": ["tool"],
        "box": {"min_m": [0.15, -0.12, 0.10], "max_m": [0.35, 0.12, 0.30]},
    }
    assert zones["link_keep_in"]["applies_to"] == ["elbow", "wrist"]
    assert zones["keep_out_0"]["kind"] == "keep_out" and zones["keep_out_0"]["applies_to"] == ["*"]
    assert env["motion"] == {"max_speed_mps": 0.2, "max_joint_speed_radps": math.radians(45)}
    assert env["stop"] == STOP and env["heartbeat"] == HEARTBEAT
    assert not [k for k in keys_anywhere(env) if k.startswith("_")]
    for absent in ("tolerance_m", "tolerance_speed", "start_pose_deg", "joint_map"):
        assert absent not in set(keys_anywhere(env))
    load(env)


def test_comments_go_to_notes_and_kit_values_go_to_the_kit_config():
    env, kit = convert()
    for text in (
        "Test arm. Base frame: x forward, y left, z up.",
        "keep_out_0: camera post.",
        "link_keep_in: work cell.",
        "stop and heartbeat blocks were supplied at conversion",
    ):
        assert text in env["notes"]
    assert "fitted by eye" not in env["notes"]
    assert kit["machine_id"] == "test-arm"
    assert kit["tolerance_m"] == 0.008 and kit["tolerance_speed"] == 0.04
    assert kit["start_pose_deg"] == HARNESS["start_pose_deg"]
    assert kit["joint_map"] == {"elbow_flex": {"sign": 1, "offset_deg": -85}}
    assert "joint_map: fitted by eye." in kit["notes"]


def test_converted_envelope_scores_exactly_like_the_harness():
    env = load(convert()[0])
    rng = random.Random(11)
    for _ in range(500):
        p = [rng.uniform(-0.3, 0.5), rng.uniform(-0.4, 0.4), rng.uniform(-0.05, 0.6)]
        assert env.clearance("tool", p) == pytest.approx(
            harness_clearance(HARNESS, p, HARNESS["keep_in"]), abs=1e-15
        )
        for link in ("elbow", "wrist"):
            assert env.clearance(link, p) == pytest.approx(
                harness_clearance(HARNESS, p, HARNESS["link_keep_in"]), abs=1e-15
            )
    assert [f for f, _, _ in env.faces("tool")] == [
        "x_min",
        "x_max",
        "y_min",
        "y_max",
        "z_min",
        "z_max",
    ]
    assert env.speed_limit_mps("tool") == 0.2


def test_without_a_link_box_only_the_tool_is_declared():
    doc = {k: v for k, v in HARNESS.items() if k != "link_keep_in"}
    doc["link_points"] = ["elbow", "wrist"]
    env_doc, _ = convert(doc)
    assert [p["id"] for p in env_doc["points"]] == ["tool"]
    assert "never checked the links" in env_doc["notes"]
    env = load(env_doc)
    assert [z.id for z in env.zones_for("tool")] == ["keep_in", "keep_out_0"]


def test_link_points_choose_the_points():
    env, _ = convert(dict(HARNESS, link_points=["wrist"]))
    assert [p["id"] for p in env["points"]] == ["tool", "wrist"]
    env, _ = convert(dict(HARNESS, link_points=["tool", "elbow"]))
    assert [p["id"] for p in env["points"]] == ["tool", "elbow"]
    assert [z.id for z in load(env).zones_for("tool")] == ["keep_in", "link_keep_in", "keep_out_0"]


def test_per_joint_speeds_become_motion_joints():
    env, _ = convert(
        dict(
            HARNESS,
            max_joint_speed_dps={"_comment": "slow wrist", "shoulder_pan": 90, "wrist_flex": 30},
        )
    )
    assert env["motion"]["joints"] == [
        {"name": "shoulder_pan", "max_speed_radps": math.radians(90)},
        {"name": "wrist_flex", "max_speed_radps": math.radians(30)},
    ]
    assert "max_joint_speed_radps" not in env["motion"]
    assert "max_joint_speed_dps: slow wrist." in env["notes"]


@pytest.mark.parametrize("value", [None, 0])
def test_no_joint_cap(value):
    env, _ = convert(dict(HARNESS, max_joint_speed_dps=value))
    assert env["motion"] == {"max_speed_mps": 0.2}


def test_keyword_arguments():
    env, kit = convert(
        machine_id="x", frame="table", machine_class="mobile_manipulator", notes="Extra."
    )
    assert env["frame"] == "table" and env["machine"] == {"id": "x", "class": "mobile_manipulator"}
    assert env["notes"].endswith("Extra.") and kit["machine_id"] == "x"


def test_minimal_harness_file():
    env, kit = convert({"keep_in": {"lo": [0, 0, 0], "hi": [1, 1, 1]}, "max_tool_speed_mps": 0.1})
    assert [z["id"] for z in env["workspace"]["zones"]] == ["keep_in"]
    assert set(kit) == {"machine_id", "converted_from", "notes"}


@pytest.mark.parametrize(
    "change, match",
    [
        (lambda d: d.update(max_tool_speed=0.2), "unknown keys"),
        (lambda d: d.pop("keep_in"), "keep_in"),
        (lambda d: d.pop("max_tool_speed_mps"), "max_tool_speed_mps"),
        (lambda d: d.update(link_points=["shoulder"]), "shoulder"),
        (lambda d: d.update(link_points=[]), "bounds nothing"),
        (lambda d: d["keep_out"][0].update(min=[0, 0, 0]), "unknown keys in keep_out_0"),
        (lambda d: d.update(_comment=["not", "text"]), "string comment"),
        (lambda d: d.update(keep_in=[[0, 0, 0], [1, 1, 1]]), "lo and hi"),
        (lambda d: d.clear() or d.update(x=1), "unknown keys"),
    ],
)
def test_bad_harness_files_raise_value_error(change, match):
    doc = copy.deepcopy(HARNESS)
    change(doc)
    with pytest.raises(ValueError, match=match):
        convert(doc)


def test_invalid_results_raise_envelope_error():
    with pytest.raises(EnvelopeError, match="ENVELOPE_BOX_INVERTED"):
        convert(dict(HARNESS, keep_in={"lo": [0.35, -0.12, 0.10], "hi": [0.15, 0.12, 0.30]}))
    with pytest.raises(EnvelopeError):
        from_harness_arm(
            copy.deepcopy(HARNESS),
            machine_id="a",
            stop=STOP,
            heartbeat=dict(HEARTBEAT, on_loss="continue"),
        )
    with pytest.raises(EnvelopeError):
        convert(machine_class="tabletop_arm")


def test_input_is_not_modified():
    doc = copy.deepcopy(HARNESS)
    from_harness_arm(doc, machine_id="a", stop=STOP, heartbeat=HEARTBEAT)
    assert doc == HARNESS


# ----- the converted examples ------------------------------------------------------------------
@pytest.mark.parametrize(
    "name, box, joint_dps",
    [
        ("bob", ([0.21, -0.1, 0.26], [0.36, 0.1, 0.42]), 60),
        ("so101", ([0.1, -0.2, 0.03], [0.36, 0.2, 0.3]), 90),
    ],
)
def test_examples(name, box, joint_dps):
    env = load(EXAMPLES / f"{name}.envelope.json")
    kit = json.loads((EXAMPLES / f"{name}.kit.json").read_text("utf-8"))
    assert list(env.points) == ["tool", "elbow", "wrist"]
    keep_in = next(z for z in env.zones if z.id == "keep_in")
    assert (list(keep_in.box_min_m), list(keep_in.box_max_m)) == box
    assert [z.id for z in env.zones] == ["keep_in", "link_keep_in", "keep_out_0"]
    assert env.motion == {"max_speed_mps": 0.25, "max_joint_speed_radps": math.radians(joint_dps)}
    assert env.stop == STOP and env.heartbeat == HEARTBEAT
    assert "illustrative placeholders, not measured" in env.get("notes")
    assert env.findings == ()
    assert kit["machine_id"] == env.machine["id"]
    assert set(kit) >= {"tolerance_m", "tolerance_speed", "start_pose_deg"}
    assert not [k for k in keys_anywhere(env.document) if k.startswith("_")]


def test_non_object_input_is_a_value_error():
    with pytest.raises(ValueError, match="JSON object"):
        from_harness_arm([HARNESS], machine_id="a", stop=STOP, heartbeat=HEARTBEAT)
