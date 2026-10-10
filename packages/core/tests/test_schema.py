"""Schema validation with a local-only registry, and the key-0.1 schema."""

from __future__ import annotations

import json
import socket
import urllib.request
from pathlib import Path

import pytest
from jsonschema import Draft202012Validator

from bounded_embodiment import schema, signing
from bounded_embodiment.canonical import CanonicalError
from bounded_embodiment.schema import _Catalog, get_schema, schema_names, validate

RFC8037_PUBLIC = {"kty": "OKP", "crv": "Ed25519", "x": "11qYAYKxCrfVS_7TyWQHOg7hcvPapiMlrwIaaPcHURo"}
RFC8037_KID = "kPrK_qmxVWaYVA9wwBF6Iuo3vVzz7TxHCTwXBygrS4k"


def _entry(**changes: object) -> dict:
    entry = {
        "key_version": "0.1",
        "kid": RFC8037_KID,
        "alg": "Ed25519",
        "public_jwk": dict(RFC8037_PUBLIC),
        "lab": "example-lab",
        "contact": "lab@example.org",
        "added": "2026-10-10",
    }
    entry.update(changes)
    return {k: v for k, v in entry.items() if v is not None}


def _where(findings: list) -> list[tuple[str, str]]:
    return [(f.code, f.where) for f in findings]


@pytest.fixture
def no_network(monkeypatch: pytest.MonkeyPatch) -> None:
    def refuse(*args: object, **kwargs: object) -> None:
        raise AssertionError("schema validation tried to use the network")

    monkeypatch.setattr(urllib.request, "urlopen", refuse)
    monkeypatch.setattr(socket, "create_connection", refuse)
    monkeypatch.setattr(socket.socket, "connect", refuse)


# --------------------------------------------------------------------------- shipped schemas


def test_shipped_schemas_are_listed() -> None:
    names = schema_names()
    assert {"envelope-0.1", "gate-decision-0.1", "key-0.1"} <= set(names)
    assert names == sorted(names)


@pytest.mark.parametrize("name", schema_names())
def test_shipped_schema_is_valid_draft_2020_12(name: str) -> None:
    s = get_schema(name)
    assert s["$schema"] == "https://json-schema.org/draft/2020-12/schema"
    assert isinstance(s.get("$id"), str) and s["$id"]
    Draft202012Validator.check_schema(s)


@pytest.mark.parametrize("name", schema_names())
def test_every_reference_in_shipped_schemas_resolves_locally(name: str, no_network: None) -> None:
    catalog = schema._package_catalog()
    s = get_schema(name)
    resolver = catalog._registry.resolver(base_uri=s["$id"])
    refs: list[str] = []

    def walk(node: object) -> None:
        if isinstance(node, dict):
            for key, value in node.items():
                if key in ("$ref", "$dynamicRef") and isinstance(value, str):
                    refs.append(value)
                walk(value)
        elif isinstance(node, list):
            for item in node:
                walk(item)

    walk(s)
    for ref in refs:
        resolver.lookup(ref)  # raises if unresolvable


def test_get_schema_returns_a_copy() -> None:
    first = get_schema("key-0.1")
    first["title"] = "changed"
    assert get_schema("key-0.1")["title"] != "changed"


def test_unknown_schema_raises_key_error() -> None:
    with pytest.raises(KeyError):
        validate({}, "no-such-schema-9.9")
    with pytest.raises(KeyError):
        validate({}, "../schemas/key-0.1")


def test_json_suffix_is_accepted() -> None:
    assert validate(_entry(), "key-0.1.json") == []


def test_gate_decision_schema_validates_with_findings() -> None:
    findings = validate({"type": "gate_decision"}, "gate-decision-0.1")
    assert findings and all(f.code == "SCHEMA_INVALID" and f.severity == "error" for f in findings)


# --------------------------------------------------------------------------- key-0.1


def test_key_entry_valid() -> None:
    assert validate(_entry(), "key-0.1") == []
    assert validate(_entry(revoked="2026-12-01"), "key-0.1") == []
    assert signing.check_key_entry(_entry()) == []


@pytest.mark.parametrize(
    ("changes", "where"),
    [
        ({"kid": "short"}, "/kid"),
        ({"kid": RFC8037_KID[:-1] + "l"}, "/kid"),  # non-canonical last character
        ({"alg": "ML-DSA-65"}, "/alg"),
        ({"key_version": "0.2"}, "/key_version"),
        ({"public_jwk": {**RFC8037_PUBLIC, "d": "nWGxne_9WmC6hEr0kuwsxERJxWl7MmkZcDusAxyuf2A"}}, "/public_jwk"),
        ({"public_jwk": {**RFC8037_PUBLIC, "crv": "Ed448"}}, "/public_jwk/crv"),
        ({"public_jwk": {"kty": "OKP", "crv": "Ed25519"}}, "/public_jwk"),
        ({"lab": "Example Lab"}, "/lab"),
        ({"lab": "-lab"}, "/lab"),
        ({"lab": ""}, "/lab"),
        ({"contact": "not-an-email"}, "/contact"),
        ({"added": "2026-02-30"}, "/added"),  # format "date" is asserted
        ({"added": "10/10/2026"}, "/added"),
        ({"revoked": "2026-13-01"}, "/revoked"),
        ({"comment": "extra members are not allowed"}, ""),
        ({"lab": None}, ""),  # missing required member
    ],
)
def test_key_entry_invalid(changes: dict, where: str) -> None:
    findings = validate(_entry(**changes), "key-0.1")
    assert findings, changes
    assert all(f.code == "SCHEMA_INVALID" and f.severity == "error" for f in findings)
    assert where in [f.where for f in findings]


def test_key_entry_from_generated_key_and_kid_rules() -> None:
    _, public = signing.generate_key()
    entry = signing.key_entry(public, "example-lab", "lab@example.org", "2026-10-10")
    assert signing.check_key_entry(entry) == []
    assert entry["kid"] == signing.thumbprint(public)
    # kid of another key: schema-valid, caught in code
    wrong = dict(entry, kid=RFC8037_KID)
    assert validate(wrong, "key-0.1") == []
    assert _where(signing.check_key_entry(wrong)) == [("KID_MISMATCH", "/kid")]
    bad_dates = dict(entry, added="2026-10-10", revoked="2026-10-01")
    assert _where(signing.check_key_entry(bad_dates)) == [("KEY_DATES_INVALID", "/revoked")]
    assert _where(signing.check_key_entry({"kid": 1}))[0][0] == "SCHEMA_INVALID"


def test_keyring_from_entries() -> None:
    _, public2 = signing.generate_key()
    e1 = _entry()
    e2 = signing.key_entry(public2, "other-lab", "x@example.org", "2026-10-11")
    e3 = dict(e2, revoked="2026-10-12")
    assert signing.keyring_from_entries([e1, e2]) == {RFC8037_KID: RFC8037_PUBLIC, e2["kid"]: public2}
    assert signing.keyring_from_entries([e1, e3]) == {RFC8037_KID: RFC8037_PUBLIC}
    assert signing.keyring_from_entries([e3], include_revoked=True) == {e2["kid"]: public2}
    with pytest.raises(ValueError, match="KID_MISMATCH"):
        signing.keyring_from_entries([dict(e1, kid=e2["kid"])])


# --------------------------------------------------------------------------- catalog mechanics


def _write(directory: Path, name: str, doc: object) -> None:
    (directory / name).write_text(json.dumps(doc), encoding="utf-8")


def test_cross_references_resolve_by_file_name(tmp_path: Path, no_network: None) -> None:
    _write(tmp_path, "base-0.1.json", {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "$id": "https://elsewhere.example/schemas/base.json",
        "$defs": {"positive": {"type": "number", "exclusiveMinimum": 0}},
    })
    _write(tmp_path, "uses-0.1.json", {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "$id": "https://example.org/schemas/uses-0.1.json",
        "type": "object",
        "properties": {
            "by_file": {"$ref": "base-0.1.json#/$defs/positive"},
            "by_id": {"$ref": "https://elsewhere.example/schemas/base.json#/$defs/positive"},
        },
    })
    catalog = _Catalog(tmp_path)
    assert catalog.validate({"by_file": 1, "by_id": 2}, "uses-0.1") == []
    findings = catalog.validate({"by_file": -1, "by_id": 0}, "uses-0.1")
    assert [f.where for f in findings] == ["/by_file", "/by_id"]


def test_remote_references_are_never_fetched(tmp_path: Path, no_network: None) -> None:
    _write(tmp_path, "remote-0.1.json", {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "$id": "https://example.org/schemas/remote-0.1.json",
        "properties": {"x": {"$ref": "https://json-schema.example/not-shipped.json"}},
    })
    catalog = _Catalog(tmp_path)
    with pytest.raises(Exception) as excinfo:
        catalog.validate({"x": 1}, "remote-0.1")
    assert not isinstance(excinfo.value, AssertionError)  # failed to resolve, did not try the network


def test_json_pointer_escaping_and_order(tmp_path: Path) -> None:
    _write(tmp_path, "p-0.1.json", {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "$id": "https://example.org/p-0.1.json",
        "type": "object",
        "properties": {"a/b~c": {"type": "integer"}, "list": {"items": {"type": "string"}}},
        "additionalProperties": False,
    })
    catalog = _Catalog(tmp_path)
    findings = catalog.validate({"zzz": 1, "list": ["ok", 3], "a/b~c": "x"}, "p-0.1")
    assert [f.where for f in findings] == ["", "/a~1b~0c", "/list/1"]
    assert "zzz" in findings[0].detail


def test_one_of_detail_names_the_closest_alternative(tmp_path: Path) -> None:
    _write(tmp_path, "o-0.1.json", {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "$id": "https://example.org/o-0.1.json",
        "oneOf": [
            {"type": "object", "required": ["box"], "properties": {"box": {"type": "array"}}},
            {"type": "object", "required": ["prism"]},
        ],
    })
    findings = _Catalog(tmp_path).validate({"box": "not an array", "pad": "x" * 2000}, "o-0.1")
    assert len(findings) == 1 and "closest" in findings[0].detail
    assert len(findings[0].detail) <= 400


def test_formats_are_asserted_without_optional_packages(tmp_path: Path) -> None:
    _write(tmp_path, "f-0.1.json", {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "$id": "https://example.org/f-0.1.json",
        "properties": {
            "dt": {"type": "string", "format": "date-time"},
            "d": {"type": "string", "format": "date"},
            "e": {"type": "string", "format": "email"},
            "u": {"type": "string", "format": "uri"},
        },
    })
    catalog = _Catalog(tmp_path)
    for good in ("2026-10-10T12:00:00Z", "2026-10-10t23:59:60.25+05:30", "2026-02-28T00:00:00-08:00"):
        assert catalog.validate({"dt": good}, "f-0.1") == [], good
    for bad in ("2026-10-10 12:00:00Z", "2026-10-10T12:00:00", "2026-13-10T00:00:00Z",
                "2026-10-10T24:00:00Z", "2026-10-10T12:61:00Z", "2026-10-10T12:00:00+24:00",
                "2026-02-29T00:00:00Z", "\u0662026-10-10T00:00:00Z"):
        assert [f.where for f in catalog.validate({"dt": bad}, "f-0.1")] == ["/dt"], bad
    assert catalog.validate({"d": "2024-02-29"}, "f-0.1") == []
    assert [f.where for f in catalog.validate({"d": "2023-02-29"}, "f-0.1")] == ["/d"]
    assert [f.where for f in catalog.validate({"e": "no at sign"}, "f-0.1")] == ["/e"]
    assert catalog.validate({"u": "not a uri, but uri is only an annotation"}, "f-0.1") == []


def test_schema_files_are_loaded_strictly(tmp_path: Path) -> None:
    (tmp_path / "dup-0.1.json").write_text('{"type": "object", "type": "array"}', encoding="utf-8")
    with pytest.raises(CanonicalError):
        _Catalog(tmp_path)


def test_duplicate_ids_are_refused(tmp_path: Path) -> None:
    for name in ("a-0.1.json", "b-0.1.json"):
        _write(tmp_path, name, {"$schema": "https://json-schema.org/draft/2020-12/schema",
                                "$id": "https://example.org/same.json"})
    with pytest.raises(ValueError, match="both declare"):
        _Catalog(tmp_path)
