"""envelope.load: the shared vectors, the 0.1 upgrade, hashing, and the Envelope queries."""

from __future__ import annotations

import copy
import json
import math
from pathlib import Path

import pytest

from bounded_embodiment import canonical
from bounded_embodiment.envelope import Envelope, EnvelopeError, Point, Zone, load
from bounded_embodiment.envelope.load import check_rules, upgrade_0_1

REPO = Path(__file__).resolve().parents[3]
VECTORS = REPO / "vectors" / "envelope"
VALID = sorted(p for p in (VECTORS / "valid").glob("*.json") if not p.name.endswith(".expect.json"))
INVALID = sorted(
    p for p in (VECTORS / "invalid").glob("*.json") if not p.name.endswith(".expect.json")
)


def sidecar(path: Path) -> dict:
    return json.loads(
        path.with_name(path.name[: -len(".json")] + ".expect.json").read_text("utf-8")
    )


def read(path: Path) -> dict:
    return json.loads(path.read_text("utf-8"))


MINIMAL = read(VECTORS / "valid" / "v02-minimal.json")
ARM = read(VECTORS / "valid" / "v02-arm-points.json")


# ----- shared vectors --------------------------------------------------------------------------
def test_vector_sets_are_not_empty():
    assert len(VALID) >= 6 and len(INVALID) >= 40
    for p in VALID + INVALID:
        assert p.with_name(p.name[:-5] + ".expect.json").exists(), f"{p.name} has no .expect.json"


@pytest.mark.parametrize("path", VALID, ids=lambda p: p.stem)
def test_valid_vector(path):
    exp = sidecar(path)
    env = load(path)
    assert env.version == "0.2"
    assert env.hash == exp["hash"]
    assert env.upgraded_from == exp["upgraded_from"]
    assert env.frame == exp["frame"]
    assert env.declared_level == exp["declared_level"]
    assert env.document == read(path)
    assert [
        {"id": z.id, "kind": z.kind, "shape": z.shape, "applies_to": list(z.applies_to)}
        for z in env.zones
    ] == exp["zones"]
    assert list(env.points) == list(exp["points"])
    for pid, p in exp["points"].items():
        assert env.speed_limit_mps(pid) == p["speed_limit_mps"]
        assert [z.id for z in env.zones_for(pid)] == p["zones"]
        assert [[f, z] for f, z, _ in env.faces(pid)] == p["faces"]
    for c in exp["clearance"]:
        assert env.clearance(c["point"], c["xyz"]) == pytest.approx(
            c["expect_m"], abs=exp["abs_tol_m"]
        ), c
    assert sorted({f.code for f in env.findings}) == sorted(set(exp["warnings"]))
    assert all(f.severity != "error" for f in env.findings)


@pytest.mark.parametrize("path", INVALID, ids=lambda p: p.stem)
def test_invalid_vector(path):
    exp = sidecar(path)
    with pytest.raises(EnvelopeError) as info:
        load(path)
    errs = [f for f in info.value.findings if f.severity == "error"]
    assert errs, "an EnvelopeError must carry at least one error finding"
    matching = [f for f in errs if f.code == exp["code"]]
    assert matching, [f.code for f in errs]
    if "where" in exp:
        assert exp["where"] in [f.where for f in matching], [f.where for f in matching]


# ----- 0.1 upgrade -----------------------------------------------------------------------------
def test_upgrade_follows_the_contract():
    doc = read(VECTORS / "valid" / "v01-tabletop-arm.json")
    up = upgrade_0_1(doc)
    assert up["envelope_version"] == "0.2"
    assert up["frame"] == "base_link" and "frame" not in up["workspace"]
    assert up["declared_level"] == "BE-2" and "level" not in up
    assert up["points"] == [
        {"id": "machine", "description": "the machine's reference point as declared in 0.1"}
    ]
    assert up["workspace"]["zones"] == [
        {
            "id": "keep_in",
            "kind": "keep_in",
            "applies_to": ["*"],
            "prism": {"polygon_m": doc["workspace"]["keep_in"], "z_m": [0.02, 0.35]},
        },
        {
            "id": "keep_out_0",
            "kind": "keep_out",
            "applies_to": ["*"],
            "prism": {"polygon_m": doc["workspace"]["keep_out"][0], "z_m": [0.02, 0.35]},
        },
    ]
    assert up["machine"]["class"] == "arm"  # free-text "tabletop_arm" mapped onto the 0.2 enum
    assert up["machine"]["description"] == "0.1 class: tabletop_arm"
    assert {k: v for k, v in up["machine"].items() if k not in ("class", "description")} == {
        k: v for k, v in doc["machine"].items() if k != "class"
    }
    for key in (
        "motion",
        "force",
        "proximity",
        "sensing",
        "stop",
        "heartbeat",
        "authority",
    ):
        assert up[key] == doc[key]


def test_upgrade_without_height_band_omits_z():
    up = upgrade_0_1(read(VECTORS / "valid" / "v01-mobile-base.json"))
    assert all("z_m" not in z["prism"] for z in up["workspace"]["zones"])
    assert up["signature"] == "ed25519:NOT-A-REAL-SIGNATURE-test-vector"


def test_upgraded_envelope_hashes_and_keeps_the_original_document():
    doc = read(VECTORS / "valid" / "v01-mobile-base.json")
    env = load(doc)
    assert env.document == doc and env.document["envelope_version"] == "0.1"
    assert env.normalized["envelope_version"] == "0.2"
    unsigned = {k: v for k, v in doc.items() if k != "signature"}
    assert env.hash == canonical.sha256_jcs(unsigned)
    assert env.hash != canonical.sha256_jcs(
        {k: v for k, v in env.normalized.items() if k != "signature"}
    )
    assert env.machine["class"] == "mobile_base"  # 0.1 free text mapped onto the 0.2 enum
    assert env.document["machine"]["class"] == "mobile_ground"  # the signed 0.1 text is untouched


def test_upgrade_does_not_touch_the_input():
    doc = read(VECTORS / "valid" / "v01-tabletop-arm.json")
    before = copy.deepcopy(doc)
    env = load(doc)
    env.normalized["workspace"]["zones"][0]["prism"]["polygon_m"][0][0] = 99
    assert doc == before and env.document == before


# ----- hashing ---------------------------------------------------------------------------------
def test_hash_ignores_only_the_signature():
    base = load(MINIMAL).hash
    signed = dict(MINIMAL, signature=[{"alg": "Ed25519", "kid": "abc", "sig": "def"}])
    assert load(signed).hash == base
    assert load(dict(MINIMAL, signature="ed25519:xyz")).hash == base
    changed = copy.deepcopy(MINIMAL)
    changed["motion"]["max_speed_mps"] = 0.999
    assert load(changed).hash != base
    assert load(dict(MINIMAL, notes="x")).hash != base


def test_hash_does_not_depend_on_key_order_or_number_spelling():
    reordered = dict(reversed(list(MINIMAL.items())))
    assert load(reordered).hash == load(MINIMAL).hash
    text = json.dumps(MINIMAL).replace('"max_speed_mps": 1', '"max_speed_mps": 1.0')
    assert '"max_speed_mps": 1.0' in text
    assert load(canonical.loads_strict(text)).hash == load(MINIMAL).hash


def test_signed_envelope_round_trip():
    signing = pytest.importorskip("bounded_embodiment.signing")
    private, public = signing.generate_key()
    signed = signing.sign(MINIMAL, private)
    env = load(signed)
    assert env.hash == load(MINIMAL).hash
    assert [
        f
        for f in signing.verify(signed, {signing.thumbprint(public): public})
        if f.severity == "error"
    ] == []


# ----- inputs ----------------------------------------------------------------------------------
def test_path_inputs(tmp_path):
    p = tmp_path / "e.json"
    p.write_text(json.dumps(MINIMAL), encoding="utf-8")
    assert load(p).hash == load(str(p)).hash == load(MINIMAL).hash


def test_dict_input_is_copied():
    doc = copy.deepcopy(MINIMAL)
    env = load(doc)
    doc["workspace"]["zones"][0]["box"]["max_m"][0] = 50
    assert env.document["workspace"]["zones"][0]["box"]["max_m"][0] == 1
    assert env.clearance("p", (0.5, 0.5, 0.5)) == 0.5


def test_unreadable_path(tmp_path):
    with pytest.raises(EnvelopeError) as info:
        load(tmp_path / "missing.json")
    assert [f.code for f in info.value.findings] == ["ENVELOPE_UNREADABLE"]


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_non_finite_numbers_in_a_dict_are_rejected(bad):
    doc = copy.deepcopy(MINIMAL)
    doc["workspace"]["zones"][0]["box"]["max_m"][1] = bad
    with pytest.raises(EnvelopeError) as info:
        load(doc)
    (f,) = info.value.findings
    assert (f.code, f.where) == ("ENVELOPE_NOT_FINITE", "/workspace/zones/0/box/max_m/1")


def test_other_input_types_are_a_type_error():
    with pytest.raises(TypeError):
        load(42)
    with pytest.raises(TypeError):
        load(json.dumps(MINIMAL).encode())


def test_error_message_names_the_codes():
    doc = copy.deepcopy(MINIMAL)
    doc["workspace"]["zones"][0]["box"]["min_m"] = [2, 2, 2]
    with pytest.raises(EnvelopeError, match="ENVELOPE_BOX_INVERTED"):
        load(doc)


def test_all_rule_violations_are_reported_together():
    doc = copy.deepcopy(ARM)
    doc["workspace"]["zones"][0]["box"]["min_m"][2] = 0.9
    doc["workspace"]["zones"][2]["id"] = "cell"
    doc["stop"]["sources"][0]["max_time_ms"] = 999
    with pytest.raises(EnvelopeError) as info:
        load(doc)
    assert {f.code for f in info.value.findings} >= {
        "ENVELOPE_BOX_INVERTED",
        "ENVELOPE_DUPLICATE_ZONE_ID",
        "ENVELOPE_STOP_SOURCE_EXCEEDS",
    }


def test_stop_sources_equal_to_the_worst_case_are_fine():
    doc = copy.deepcopy(ARM)
    doc["stop"]["sources"] = [
        {"source": s, "category": 1, "max_time_ms": 400, "max_distance_m": 0.06}
        for s in (
            "stop_message",
            "hardwired_stop",
            "heartbeat_loss",
            "comms_loss",
            "power_loss",
            "gate_loss",
        )
    ]
    assert load(doc).stop["sources"][5]["source"] == "gate_loss"


def test_check_rules_is_usable_on_its_own():
    assert check_rules(ARM) == []


# ----- queries ---------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def arm() -> Envelope:
    return load(ARM)


def test_points_and_zones_are_model_objects(arm):
    assert isinstance(arm.points["tool"], Point) and all(isinstance(z, Zone) for z in arm.zones)
    tool = arm.points["tool"]
    assert (tool.link, tool.offset_m, tool.max_speed_mps) == ("gripper", (0, 0, -0.03), 0.25)
    assert arm.points["elbow"].offset_m is None


def test_wildcard_zones_apply_to_every_point(arm):
    for pid in arm.points:
        assert "fixture" in [z.id for z in arm.zones_for(pid)]


def test_clearance_is_the_minimum_over_the_points_zones(arm):
    q = (0.25, 0.0, 0.10)
    per_zone = [z.clearance(q) for z in arm.zones_for("tool")]
    assert arm.clearance("tool", q) == min(per_zone)
    keep_out = next(z for z in arm.zones if z.id == "fixture")
    assert keep_out.clearance(q) == -keep_out.signed_distance(q)


def test_point_without_zones_has_infinite_clearance_and_a_warning():
    doc = copy.deepcopy(MINIMAL)
    doc["points"].append({"id": "flag", "description": "Tip of the antenna."})
    doc["workspace"]["zones"][0]["applies_to"] = ["p"]
    env = load(doc)
    assert env.clearance("flag", (100, 100, 100)) == math.inf
    assert env.faces("flag") == [] and env.zones_for("flag") == []
    assert [(f.code, f.severity, f.where) for f in env.findings] == [
        ("ENVELOPE_POINT_NO_KEEP_IN", "warn", "/points/1")
    ]


def test_faces_cover_keep_in_zones_only(arm):
    zone_ids = {z for _, z, _ in arm.faces("tool")}
    assert zone_ids == {"tool_box"}
    fn = {(f, z): fn for f, z, fn in arm.faces("tool")}[("x_max", "tool_box")]
    assert fn((0.39, 0.0, 0.1)) == pytest.approx(0.01)
    assert fn((0.42, 0.0, 0.1)) == pytest.approx(-0.02)


def test_unknown_point_is_a_key_error(arm):
    for call in (
        lambda: arm.zones_for("nose"),
        lambda: arm.clearance("nose", (0, 0, 0)),
        lambda: arm.faces("nose"),
        lambda: arm.speed_limit_mps("nose"),
    ):
        with pytest.raises(KeyError, match="nose"):
            call()


def test_declared_blocks_are_exposed(arm):
    assert arm.motion["joints"][0]["name"] == "shoulder_pan"
    assert arm.heartbeat["on_loss"] == "stop"
    assert arm.get("force") == {"max_contact_force_n": 20}
    assert arm.get("proximity") is None
    assert arm.machine["class"] == "arm"


def test_envelope_is_frozen(arm):
    with pytest.raises(AttributeError):
        arm.hash = "sha256:" + "0" * 64


def test_stop_source_category_is_not_a_magnitude():
    """IEC 60204-1 categories name kinds of stop; a category 2 source under a category 1 worst
    case is allowed. Only time and distance are compared."""
    import copy
    doc = copy.deepcopy(read(VECTORS / "valid" / "v02-minimal.json"))
    doc["stop"] = {"category": 1, "max_time_ms": 500, "max_distance_m": 0.05,
                   "sources": [{"source": "stop_message", "category": 2,
                                "max_time_ms": 400, "max_distance_m": 0.04}]}
    env = load(doc)
    assert not [f for f in env.findings if f.code == "ENVELOPE_STOP_SOURCE_EXCEEDS"]


def test_upgrade_maps_free_text_classes():
    from bounded_embodiment.envelope.load import _class_0_2
    assert _class_0_2("tabletop_arm") == "arm"
    assert _class_0_2("mobile_ground") == "mobile_base"
    assert _class_0_2("Mobile Manipulator") == "mobile_manipulator"
    assert _class_0_2("quadruped") == "legged"
    assert _class_0_2("conveyor") == "other"
