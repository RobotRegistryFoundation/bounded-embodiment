"""schemas/envelope-0.2.json on its own, checked with jsonschema directly (no loader involved)."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from jsonschema import Draft202012Validator

REPO = Path(__file__).resolve().parents[3]
SCHEMA_PATH = REPO / "schemas" / "envelope-0.2.json"
SCHEMA = json.loads(SCHEMA_PATH.read_text("utf-8"))
V01 = json.loads((REPO / "schemas" / "envelope-0.1.json").read_text("utf-8"))
VECTORS = REPO / "vectors" / "envelope"


def subschemas(node, path=""):
    if isinstance(node, dict):
        yield path, node
        for k, v in node.items():
            yield from subschemas(v, f"{path}/{k}")
    elif isinstance(node, list):
        for i, v in enumerate(node):
            yield from subschemas(v, f"{path}/{i}")


def strip_descriptions(node):
    if isinstance(node, dict):
        return {k: strip_descriptions(v) for k, v in node.items() if k != "description"}
    if isinstance(node, list):
        return [strip_descriptions(v) for v in node]
    return node


def test_is_a_valid_2020_12_schema_with_the_published_id():
    Draft202012Validator.check_schema(SCHEMA)
    assert SCHEMA["$schema"] == "https://json-schema.org/draft/2020-12/schema"
    assert SCHEMA["$id"] == (
        "https://robotregistryfoundation.github.io/bounded-embodiment/schemas/envelope-0.2.json"
    )
    assert "CC-BY-4.0" in SCHEMA["$comment"]


def test_every_object_rejects_unknown_keys():
    objects = [(p, s) for p, s in subschemas(SCHEMA) if s.get("type") == "object"]
    assert len(objects) > 15
    for path, s in objects:
        assert s.get("additionalProperties") is False, path


def test_blocks_kept_from_0_1_are_unchanged():
    for key in ("force", "proximity", "sensing", "heartbeat", "authority"):
        assert SCHEMA["properties"][key] == V01["properties"][key], key
    assert SCHEMA["properties"]["heartbeat"]["properties"]["on_loss"] == {
        "const": "stop",
        "description": V01["properties"]["heartbeat"]["properties"]["on_loss"]["description"],
    }
    for key in ("motion", "stop"):
        new = strip_descriptions(SCHEMA["properties"][key])
        old = strip_descriptions(V01["properties"][key])
        assert new["required"] == old["required"]
        for name, rule in old["properties"].items():
            assert new["properties"][name] == rule, (key, name)
    assert (
        SCHEMA["properties"]["signature"]["oneOf"][0]["pattern"]
        == V01["properties"]["signature"]["pattern"]
    )


def test_zone_has_exactly_one_shape_and_at_least_one_keep_in_is_required():
    zone = SCHEMA["$defs"]["zone"]
    assert zone["oneOf"] == [{"required": ["box"]}, {"required": ["prism"]}]
    contains = SCHEMA["properties"]["workspace"]["properties"]["zones"]["contains"]
    assert contains["properties"]["kind"] == {"const": "keep_in"}
    assert SCHEMA["$defs"]["point"]["required"] == ["id", "description"]
    assert "declared_level" not in SCHEMA["required"]


VALIDATOR = Draft202012Validator(SCHEMA)


def _pointer(path) -> str:
    return "".join(f"/{p}" for p in path)


@pytest.mark.parametrize(
    "path",
    [
        p
        for p in sorted((VECTORS / "valid").glob("v02-*.json"))
        if not p.name.endswith(".expect.json")
    ],
    ids=lambda p: p.stem,
)
def test_valid_0_2_vectors_validate(path):
    errors = [
        (_pointer(e.absolute_path), e.message)
        for e in VALIDATOR.iter_errors(json.loads(path.read_text()))
    ]
    assert errors == []


def _schema_vectors():
    out = []
    for path in sorted((VECTORS / "invalid").glob("*.expect.json")):
        exp = json.loads(path.read_text("utf-8"))
        doc_path = path.with_name(path.name.replace(".expect.json", ".json"))
        if exp["code"] != "SCHEMA_INVALID":
            continue
        doc = json.loads(doc_path.read_text("utf-8"))
        if doc.get("envelope_version") == "0.2":
            out.append(pytest.param(doc, exp, id=doc_path.stem))
    return out


@pytest.mark.parametrize("doc, exp", _schema_vectors())
def test_invalid_0_2_vectors_fail_where_expected(doc, exp):
    paths = [_pointer(e.absolute_path) for e in VALIDATOR.iter_errors(doc)]
    assert exp["where"] in paths
