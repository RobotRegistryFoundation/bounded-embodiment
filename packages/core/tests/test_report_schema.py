"""schemas/test-report-0.1.json and schemas/waypoints-0.1.json: closed objects and the rules
they state."""

from __future__ import annotations

import copy

import pytest
from _report_helpers import RUNS, in_repo, read_report
from bounded_embodiment import schema

pytestmark = in_repo

REPORT = "test-report-0.1"


FRAGMENTS = ("if", "then", "else", "contains", "not")


def _objects(node: object, path: str = "#"):
    """Every subschema that declares an object's properties; the fragments of if/then/else,
    contains and not only add rules to objects that are already closed, so they are skipped."""
    if isinstance(node, dict):
        if "properties" in node:
            yield path, node
        for key, value in node.items():
            if key not in FRAGMENTS:
                yield from _objects(value, f"{path}/{key}")
    elif isinstance(node, list):
        for i, value in enumerate(node):
            yield from _objects(value, f"{path}/{i}")


@pytest.mark.parametrize("name", [REPORT, "waypoints-0.1"])
def test_every_object_is_closed(name: str) -> None:
    open_ = []
    for path, node in _objects(schema.get_schema(name)):
        if node.get("type") != "object" and "required" not in node:
            continue  # an if/then fragment that only constrains members
        if node.get("additionalProperties") is not False:
            open_.append(path)
    assert open_ == []


def test_schema_is_registered() -> None:
    assert {REPORT, "waypoints-0.1"} <= set(schema.schema_names())
    assert schema.get_schema(REPORT)["$comment"].endswith("SPDX-License-Identifier: CC-BY-4.0.")


@pytest.fixture
def ev03() -> dict:
    return read_report(RUNS / "ev03-pass")


@pytest.fixture
def ev10() -> dict:
    return read_report(RUNS / "ev10-pass")


def _codes_at(doc: dict) -> list[str]:
    return [f.where for f in schema.validate(doc, REPORT)]


def test_golden_reports_validate(ev03: dict, ev10: dict) -> None:
    assert schema.validate(ev03, REPORT) == []
    assert schema.validate(ev10, REPORT) == []


def test_unknown_members_are_refused(ev03: dict) -> None:
    for where in ("", "run", "generator", "verdict", "score"):
        doc = copy.deepcopy(ev03)
        (doc[where] if where else doc)["declared_level"] = "BE-1"
        assert schema.validate(doc, REPORT), where


def test_motion_is_ev03_only_and_liveness_ev10_only(ev03: dict, ev10: dict) -> None:
    doc = copy.deepcopy(ev03)
    doc["liveness"] = ev10["liveness"]
    assert "/liveness" in _codes_at(doc)
    doc = copy.deepcopy(ev10)
    doc["motion"] = ev03["motion"]
    assert "/motion" in _codes_at(doc)
    doc = copy.deepcopy(ev03)
    del doc["motion"]
    assert schema.validate(doc, REPORT)
    doc = copy.deepcopy(ev03)
    doc["scoring"]["method"] = "ev10-waypoints/1"
    assert "/scoring/method" in _codes_at(doc)


def test_exactly_one_primary_instrument(ev03: dict) -> None:
    doc = copy.deepcopy(ev03)
    doc["instruments"][0]["primary"] = False
    assert "/instruments" in _codes_at(doc)
    doc = copy.deepcopy(ev03)
    doc["instruments"].append(copy.deepcopy(doc["instruments"][0]))
    assert "/instruments" in _codes_at(doc)


def test_instruments_that_are_never_external(ev03: dict) -> None:
    for kind in ("robot_encoders_fk", "simulation_ground_truth"):
        doc = copy.deepcopy(ev03)
        doc["instruments"][0].update(kind=kind, external=True)
        assert "/instruments/0/external" in _codes_at(doc), kind
    doc = copy.deepcopy(ev03)
    doc["instruments"][0].update(kind="markerless_model_tracker", external=True)
    assert schema.validate(doc, REPORT)
    doc["instruments"][0]["independent_of_robot_state"] = True
    assert schema.validate(doc, REPORT) == []


def test_control_run_and_reproduces(ev03: dict) -> None:
    doc = copy.deepcopy(ev03)
    doc["control_run"] = "golden-lab/ev03-20261010T170000Z-s0"
    doc["reproduces"] = "other-lab/ev03-20261009T120000Z-s7"
    assert schema.validate(doc, REPORT) == []
    doc["control_run"] = "ev03-20261010T170000Z-s0"
    assert schema.validate(doc, REPORT) == []
    for bad in ("results/golden-lab/ev03-20261010T170000Z-s0", "../x", ""):
        doc["reproduces"] = bad
        assert "/reproduces" in _codes_at(doc), bad
    doc = copy.deepcopy(ev03)
    doc["role"] = "control"
    doc["control_run"] = "ev03-20261010T170000Z-s0"
    assert "/control_run" in _codes_at(doc)


def test_statements_are_fixed(ev03: dict) -> None:
    for member, value in (
        ("not_certification", False),
        ("thresholds_are_illustrative", False),
        ("measurement_source", "lab"),
    ):
        doc = copy.deepcopy(ev03)
        doc["statements"][member] = value
        assert f"/statements/{member}" in _codes_at(doc)


def test_times_are_utc(ev03: dict) -> None:
    for bad in (
        "2026-10-10T18:00:00+02:00",
        "2026-10-10 18:00:00Z",
        "2026-13-10T18:00:00Z",
        "2026-10-10T18:00:60Z",
    ):
        doc = copy.deepcopy(ev03)
        doc["run"]["started_at"] = bad
        assert "/run/started_at" in _codes_at(doc), bad


def test_generator_params_follow_the_generator(ev03: dict) -> None:
    doc = copy.deepcopy(ev03)
    doc["generator"]["name"] = "be-control"
    assert any(w.startswith("/generator/params") for w in _codes_at(doc))
    doc["generator"]["params"] = {
        "target": "into_table",
        "seconds": 2.0,
        "hz": 50.0,
        "step_cap": None,
        "start": None,
    }
    assert schema.validate(doc, REPORT) == []
    doc["generator"]["version"] = "2"
    assert "/generator/version" in _codes_at(doc)


def test_no_assurance_level_and_no_registry_number() -> None:
    text = str(schema.get_schema(REPORT)["properties"].keys())
    assert "level" not in text and "rrn" not in text.lower()
