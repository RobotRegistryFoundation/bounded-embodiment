"""The EV-02 stop plan and the EV-05 model-loss plan: loading, the findings for bad plans, and
the check of every plan command against the command space."""

from __future__ import annotations

import copy
import json

import pytest
from _report_helpers import golden, in_repo
from bounded_embodiment import canonical
from bounded_embodiment.commandspace import load_command_space
from bounded_embodiment.report import plans
from bounded_embodiment.report.plans import (
    PlanError,
    check_plan_commands,
    load_model_loss_plan,
    load_stop_plan,
)

pytestmark = in_repo


def stop_doc() -> dict:
    return golden().stop_plan_doc(list(golden().EV02_TRIALS))


def loss_doc() -> dict:
    return golden().loss_plan_doc()


def codes(exc: pytest.ExceptionInfo) -> list[str]:
    return sorted(f.code for f in exc.value.findings)


# --------------------------------------------------------------------------- the stop plan


def test_stop_plan_loads() -> None:
    doc = stop_doc()
    plan = load_stop_plan(doc)
    assert plan.point == "tool"
    assert dict(plan.start) == {"x": 0.3, "y": 0.0, "z": 0.175}
    assert [d.id for d in plan.drives] == ["x_plus", "x_minus"]
    assert plan.sources == ("stop_message", "hardwired_stop")
    assert plan.trials == tuple(golden().EV02_TRIALS)
    assert plan.digest == canonical.sha256_jcs(doc)
    assert plan.for_scoring() == {"point": "tool", "sources": ["stop_message", "hardwired_stop"]}
    assert plan.drive("x_minus").command["x"] == 0.2
    with pytest.raises(KeyError):
        plan.drive("nowhere")
    assert [p for p, _ in plan.commands()] == [
        "/start",
        "/drives/0/command",
        "/drives/1/command",
    ]


def test_stop_plan_from_bytes_is_the_same_plan() -> None:
    doc = stop_doc()
    plan = load_stop_plan(json.dumps(doc).encode("utf-8"))
    assert plan.digest == load_stop_plan(doc).digest


def test_stop_plan_commands_are_read_only_floats() -> None:
    doc = stop_doc()
    doc["start"] = {"x": 0, "y": -0.0, "z": 0.175}
    plan = load_stop_plan(doc)
    assert all(type(v) is float for v in plan.start.values())
    assert str(plan.start["y"]) == "0.0"
    with pytest.raises(TypeError):
        plan.start["x"] = 1.0  # type: ignore[index]


@pytest.mark.parametrize(
    ("src", "code"),
    [
        (b'{"stop_plan_version": "0.1", "stop_plan_version": "0.1"}', "PLAN_PARSE_ERROR"),
        (b"[1, 2]", "PLAN_NOT_OBJECT"),
        (b'{"stop_plan_version": NaN}', "PLAN_PARSE_ERROR"),
        (b'{"stop_plan_version": "0.2"}', "PLAN_VERSION_UNSUPPORTED"),
        (b'{"stop_plan_version": "0.1"}', "SCHEMA_INVALID"),
    ],
)
def test_bad_stop_plan_documents(src: bytes, code: str) -> None:
    with pytest.raises(PlanError) as exc:
        load_stop_plan(src)
    assert code in codes(exc)


def _edited(change) -> dict:
    doc = copy.deepcopy(stop_doc())
    change(doc)
    return doc


@pytest.mark.parametrize(
    ("change", "code"),
    [
        (lambda d: d.update(extra=1), "SCHEMA_INVALID"),
        (lambda d: d["drives"].append(dict(d["drives"][0])), "PLAN_DUPLICATE_ID"),
        (lambda d: d["sources"].append({"source": "stop_message"}), "PLAN_DUPLICATE_SOURCE"),
        (lambda d: d["trials"].append({"source": "power_loss", "drive": "x_plus"}), "PLAN_UNKNOWN_SOURCE"),
        (lambda d: d["trials"].append({"source": "stop_message", "drive": "up"}), "PLAN_UNKNOWN_DRIVE"),
        (lambda d: d["sources"].append({"source": "lost_wifi"}), "SCHEMA_INVALID"),
        (lambda d: d.update(trials=[]), "SCHEMA_INVALID"),
        (lambda d: d["drives"][0].update(id="has space"), "SCHEMA_INVALID"),
    ],
)
def test_bad_stop_plans(change, code: str) -> None:
    with pytest.raises(PlanError) as exc:
        load_stop_plan(_edited(change))
    assert code in codes(exc), codes(exc)
    assert "invalid plan" in str(exc.value)


# --------------------------------------------------------------------------- the model-loss plan


def test_model_loss_plan_loads() -> None:
    doc = loss_doc()
    plan = load_model_loss_plan(doc)
    assert plan.point == "tool" and plan.source == "heartbeat_loss"
    assert [dict(c) for c in plan.path] == [
        {"x": 0.3, "y": -0.1, "z": 0.175},
        {"x": 0.3, "y": 0.1, "z": 0.175},
    ]
    assert dict(plan.resume) == {"rule": "manual"}  # the description is dropped
    assert plan.for_scoring() == {"point": "tool", "source": "heartbeat_loss", "resume": {"rule": "manual"}}
    assert plan.digest == canonical.sha256_jcs(doc)
    assert [p for p, _ in plan.commands()] == ["/start", "/path/0", "/path/1"]


def test_model_loss_plan_defaults_and_commands_rule() -> None:
    doc = loss_doc()
    del doc["source"], doc["point"]
    doc["resume"] = {"rule": "commands", "after_ms": 1500, "description": "resumes on its own"}
    plan = load_model_loss_plan(doc)
    assert plan.source == plans.DEFAULT_LOSS_SOURCE == "heartbeat_loss"
    assert plan.point is None
    assert dict(plan.resume) == {"rule": "commands", "after_ms": 1500}


@pytest.mark.parametrize(
    "resume",
    [
        {"rule": "commands"},
        {"rule": "commands", "after_ms": -1},
        {"rule": "commands", "after_ms": 1.5},
        {"rule": "manual", "after_ms": 0},
        {"rule": "never"},
    ],
)
def test_bad_resume_rules(resume: dict) -> None:
    doc = loss_doc()
    doc["resume"] = resume
    with pytest.raises(PlanError) as exc:
        load_model_loss_plan(doc)
    assert codes(exc) and set(codes(exc)) == {"SCHEMA_INVALID"}


def test_model_loss_plan_version_and_source() -> None:
    doc = loss_doc()
    doc["model_loss_plan_version"] = "1.0"
    with pytest.raises(PlanError) as exc:
        load_model_loss_plan(doc)
    assert codes(exc) == ["PLAN_VERSION_UNSUPPORTED"]
    doc = loss_doc()
    doc["source"] = "stop_message"
    with pytest.raises(PlanError) as exc:
        load_model_loss_plan(doc)
    assert "SCHEMA_INVALID" in codes(exc)


# --------------------------------------------------------------------------- commands in the space


def test_plan_commands_must_fit_the_command_space() -> None:
    space = load_command_space(golden().COMMAND_SPACE)
    assert check_plan_commands(load_stop_plan(stop_doc()), space, plans.STOP_PLAN_FILE) == []
    assert check_plan_commands(load_model_loss_plan(loss_doc()), space, plans.MODEL_LOSS_PLAN_FILE) == []
    doc = stop_doc()
    doc["start"] = {"x": 0.3, "y": 0.0, "w": 1.0}
    doc["drives"][1]["command"]["x"] = 0.5  # hi is 0.45
    found = check_plan_commands(load_stop_plan(doc), space, plans.STOP_PLAN_FILE)
    assert [(f.code, f.where) for f in found] == [
        ("PLAN_COMMAND_UNKNOWN_AXIS", "ev02-stop-plan.json /start"),
        ("PLAN_COMMAND_MISSING_AXIS", "ev02-stop-plan.json /start"),
        ("PLAN_COMMAND_OUTSIDE_LIMITS", "ev02-stop-plan.json /drives/1/command"),
    ]
    assert all(f.severity == "error" for f in found)
