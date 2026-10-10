"""load_command_space: the shared vectors, the two examples, hashing, and the schema itself."""

from __future__ import annotations

import copy
import json
import math
from pathlib import Path

import pytest
from jsonschema import Draft202012Validator

from bounded_embodiment import canonical, schema
from bounded_embodiment.commandspace import (
    KINDS,
    MALFORMED_KINDS,
    UNITS,
    Axis,
    CommandSpace,
    CommandSpaceError,
    FarTarget,
    load_command_space,
)

REPO = Path(__file__).resolve().parents[3]
VECTORS = REPO / "vectors" / "command-space"
EXAMPLES = REPO / "examples" / "command-spaces"
VALID = sorted(p for p in (VECTORS / "valid").glob("*.json") if not p.name.endswith(".expect.json"))
INVALID = sorted(
    p for p in (VECTORS / "invalid").glob("*.json") if not p.name.endswith(".expect.json")
)


def sidecar(path: Path) -> dict:
    expect = path.with_name(path.name[: -len(".json")] + ".expect.json")
    return json.loads(expect.read_text("utf-8"))


def read(path: Path) -> dict:
    return json.loads(path.read_text("utf-8"))


SO101 = read(EXAMPLES / "so101.command-space.json")
ROVER = read(EXAMPLES / "rover.command-space.json")
MINIMAL = read(VECTORS / "valid" / "minimal.json")


def codes(exc: CommandSpaceError) -> list[tuple[str, str]]:
    return [(f.code, f.where) for f in exc.findings if f.severity == "error"]


# ----- shared vectors --------------------------------------------------------------------------
def test_vector_sets_are_not_empty():
    assert len(VALID) >= 2 and len(INVALID) >= 30
    for p in VALID + INVALID:
        assert p.with_name(p.name[:-5] + ".expect.json").exists(), f"{p.name} has no .expect.json"


@pytest.mark.parametrize("path", VALID, ids=lambda p: p.stem)
def test_valid_vector(path):
    exp = sidecar(path)
    space = load_command_space(path)
    assert space.digest == exp["digest"] == canonical.sha256_jcs(read(path))
    assert space.document == read(path)
    assert space.kind == exp["kind"]
    assert [[a.key, a.unit, a.lo, a.hi, a.home] for a in space.axes] == exp["axes"]
    assert [ft.id for ft in space.far_targets] == exp["far_targets"]
    assert list(space.malformed) == exp["malformed"]


@pytest.mark.parametrize("path", INVALID, ids=lambda p: p.stem)
def test_invalid_vector(path):
    """Each invalid vector differs from a valid document in one way, so it fails for one reason."""
    exp = sidecar(path)
    with pytest.raises(CommandSpaceError) as info:
        load_command_space(path)
    errs = codes(info.value)
    assert len(errs) == 1, errs
    assert errs[0][0] == exp["code"], errs
    if "where" in exp:
        assert errs[0][1] == exp["where"], errs


# ----- the examples ----------------------------------------------------------------------------
# The SO-101 URDF limits as the ev03-harness arm fuzzer declares them (JOINT_LIMITS_DEG).
SO101_LIMITS_DEG = {
    "shoulder_pan": (-110.0, 110.0),
    "shoulder_lift": (-100.0, 100.0),
    "elbow_flex": (-96.8, 96.8),
    "wrist_flex": (-95.0, 95.0),
    "wrist_roll": (-157.2, 162.8),
}


def test_so101_example():
    space = load_command_space(EXAMPLES / "so101.command-space.json")
    assert space.kind == "joint_position"
    assert space.keys == tuple(f"{j}.pos" for j in SO101_LIMITS_DEG) + ("gripper.pos",)
    for joint, (lo, hi) in SO101_LIMITS_DEG.items():
        assert space.axis(f"{joint}.pos") == Axis(f"{joint}.pos", "deg", lo, hi, 0.0)
    assert space.axis("gripper.pos") == Axis("gripper.pos", "percent", 0.0, 100.0, 30.0)
    assert [ft.id for ft in space.far_targets] == [
        "into_table",
        "overhead",
        "far_right",
        "far_left",
        "full_reach_forward",
        "into_keep_out",
        "behind_base",
    ]
    assert space.far_target("full_reach_forward").values == {
        "shoulder_pan.pos": 0.0,
        "shoulder_lift.pos": 50.0,
        "elbow_flex.pos": -25.0,
        "wrist_flex.pos": -30.0,
        "wrist_roll.pos": 0.0,
        "gripper.pos": 30.0,
    }
    for ft in space.far_targets:
        assert ft.values["gripper.pos"] == 30.0
        assert all(type(v) is float for v in ft.values.values())
        assert ft.description
    with pytest.raises(TypeError):  # read-only: the stream cannot drift from the digest
        space.far_targets[0].values["gripper.pos"] = 99.0
    assert space.malformed == MALFORMED_KINDS
    assert space.digest == canonical.sha256_jcs(SO101)


def test_rover_example():
    space = load_command_space(EXAMPLES / "rover.command-space.json")
    assert space.kind == "velocity"
    assert space.axes == (
        Axis("linear", "m/s", -1.5, 1.5, 0.0),
        Axis("angular", "rad/s", -1.5, 1.5, 0.0),
    )
    assert [ft.id for ft in space.far_targets] == ["straight_out"]
    assert space.malformed == MALFORMED_KINDS


# ----- reading and hashing ---------------------------------------------------------------------
def test_path_and_dict_inputs(tmp_path):
    p = tmp_path / "cs.json"
    p.write_text(json.dumps(SO101), encoding="utf-8")
    digests = {load_command_space(src).digest for src in (p, str(p), SO101)}
    assert len(digests) == 1


def test_dict_input_is_copied():
    doc = copy.deepcopy(MINIMAL)
    space = load_command_space(doc)
    doc["axes"][0]["hi"] = 50
    assert space.document["axes"][0]["hi"] == 100 and space.axes[0].hi == 100.0


def test_digest_ignores_key_order_and_number_spelling_but_nothing_else():
    base = load_command_space(SO101).digest
    reordered = dict(reversed(list(SO101.items())))
    assert load_command_space(reordered).digest == base
    text = json.dumps(SO101).replace('"hi": 110.0', '"hi": 110')
    assert '"hi": 110,' in text
    assert load_command_space(canonical.loads_strict(text)).digest == base
    changed = copy.deepcopy(SO101)
    changed["axes"][0]["hi"] = 109.0
    assert load_command_space(changed).digest != base
    assert load_command_space(dict(SO101, description="other words")).digest != base


def test_numbers_are_floats_and_negative_zero_is_zero():
    doc = copy.deepcopy(MINIMAL)
    doc["axes"][0].update(lo=-1, home=-0.0)
    axis = load_command_space(doc).axes[0]
    assert type(axis.lo) is float and type(axis.hi) is float
    assert axis.home == 0.0 and math.copysign(1.0, axis.home) == 1.0
    plain = copy.deepcopy(doc)
    plain["axes"][0]["home"] = 0.0
    assert load_command_space(doc).digest == load_command_space(plain).digest


def test_non_finite_values_in_a_dict():
    doc = copy.deepcopy(MINIMAL)
    doc["axes"][0]["hi"] = math.inf
    with pytest.raises(CommandSpaceError) as info:
        load_command_space(doc)
    assert codes(info.value) == [("COMMAND_SPACE_NOT_FINITE", "/axes/0/hi")]


def test_a_document_that_cannot_be_hashed():
    with pytest.raises(CommandSpaceError) as info:
        load_command_space(dict(MINIMAL, description="lone surrogate \ud800"))
    assert [c for c, _ in codes(info.value)] == ["COMMAND_SPACE_NOT_CANONICAL"]


def test_unreadable_path(tmp_path):
    with pytest.raises(CommandSpaceError) as info:
        load_command_space(tmp_path / "missing.json")
    assert [c for c, _ in codes(info.value)] == ["COMMAND_SPACE_UNREADABLE"]


def test_only_paths_and_dicts():
    with pytest.raises(TypeError):
        load_command_space(42)


def test_error_message_names_the_errors():
    doc = copy.deepcopy(MINIMAL)
    doc["axes"][0]["home"] = 101
    with pytest.raises(CommandSpaceError, match="COMMAND_SPACE_HOME_OUTSIDE at /axes/0/home"):
        load_command_space(doc)


def test_every_semantic_error_is_reported_together():
    doc = copy.deepcopy(SO101)
    doc["axes"][0]["lo"] = 200.0  # above hi
    doc["axes"][5]["home"] = -1.0  # below lo
    doc["far_targets"][1]["id"] = "into_table"
    with pytest.raises(CommandSpaceError) as info:
        load_command_space(doc)
    assert codes(info.value) == [
        ("COMMAND_SPACE_RANGE_INVERTED", "/axes/0"),
        ("COMMAND_SPACE_HOME_OUTSIDE", "/axes/5/home"),
        ("COMMAND_SPACE_DUPLICATE_FAR_TARGET", "/far_targets/1/id"),
    ]


def test_malformed_subset_keeps_the_fixed_order():
    doc = dict(MINIMAL, malformed=["empty", "inf", "string"])
    assert load_command_space(doc).malformed == ("inf", "string", "empty")


def test_accessors():
    space = load_command_space(SO101)
    assert space.keys[0] == "shoulder_pan.pos"
    assert space.axis("gripper.pos").home == 30.0
    assert isinstance(space.far_target("overhead"), FarTarget)
    with pytest.raises(KeyError, match="unknown axis"):
        space.axis("elbow")
    with pytest.raises(KeyError, match="unknown far target"):
        space.far_target("nowhere")
    with pytest.raises(KeyError, match="no far targets"):
        load_command_space(MINIMAL).far_target("anything")
    assert isinstance(space, CommandSpace)


# ----- the schema ------------------------------------------------------------------------------
def subschemas(node, path=""):
    if isinstance(node, dict):
        yield path, node
        for k, v in node.items():
            yield from subschemas(v, f"{path}/{k}")
    elif isinstance(node, list):
        for i, v in enumerate(node):
            yield from subschemas(v, f"{path}/{i}")


def test_schema_is_shipped_and_valid_2020_12():
    assert "command-space-0.1" in schema.schema_names()
    s = schema.get_schema("command-space-0.1")
    Draft202012Validator.check_schema(s)
    assert s["$schema"] == "https://json-schema.org/draft/2020-12/schema"
    assert s["$id"] == (
        "https://robotregistryfoundation.github.io/bounded-embodiment/schemas/"
        "command-space-0.1.json"
    )
    assert "CC-BY-4.0" in s["$comment"]
    assert s == read(REPO / "schemas" / "command-space-0.1.json")


def test_every_structural_object_rejects_unknown_keys():
    s = schema.get_schema("command-space-0.1")
    objects = [(p, n) for p, n in subschemas(s) if n.get("type") == "object"]
    assert len(objects) == 4
    for path, node in objects:
        if path.endswith("/far_target/properties/values"):  # axis key -> value
            assert node["additionalProperties"] == {"$ref": "#/$defs/value"}
        else:
            assert node.get("additionalProperties") is False, path


def test_schema_enums_match_the_module():
    s = schema.get_schema("command-space-0.1")
    assert tuple(s["properties"]["kind"]["enum"]) == KINDS
    assert tuple(s["$defs"]["axis"]["properties"]["unit"]["enum"]) == UNITS
    assert tuple(s["properties"]["malformed"]["items"]["enum"]) == MALFORMED_KINDS


def test_validate_through_the_schema_registry():
    assert schema.validate(SO101, "command-space-0.1") == []
    assert schema.validate(ROVER, "command-space-0.1") == []
    findings = schema.validate(dict(SO101, kind="torque"), "command-space-0.1")
    assert [(f.code, f.where) for f in findings] == [("SCHEMA_INVALID", "/kind")]
