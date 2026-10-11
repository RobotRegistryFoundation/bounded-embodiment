# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""The lab config (be-lab-config/0.1): strict, with the report schema's own definitions."""
from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import _submission_support as sup
from bounded_embodiment_kit.submission.labconfig import (
    LabConfigError,
    load_lab_config,
    validate_lab_config,
)

GOOD = sup.lab_config()


def _bad(mutate) -> list[str]:
    doc = copy.deepcopy(GOOD)
    mutate(doc)
    problems = validate_lab_config(doc)
    assert problems
    return problems


def test_a_complete_config_is_valid_and_optional_software_too(tmp_path):
    assert validate_lab_config(GOOD) == []
    with_software = {**GOOD, "software": [{"name": "lerobot", "version": "0.6.1", "role": "driver"}]}
    assert validate_lab_config(with_software) == []
    withheld = copy.deepcopy(GOOD)
    withheld["machine"]["model"] = "withheld"
    del withheld["machine"]["physical_max_tool_speed_mps"]
    assert validate_lab_config(withheld) == []
    assert load_lab_config(sup.write_json(tmp_path / "lab.json", GOOD)) == GOOD


@pytest.mark.parametrize(("mutate", "where"), [
    (lambda d: d.update(notes="x"), "/: Additional properties are not allowed ('notes'"),
    (lambda d: d["machine"].update(serial="123"), "/machine: Additional properties"),
    (lambda d: d["instruments"][0].update(owner="x"), "/instruments/0: Additional properties"),
    (lambda d: d["signer"].update(kid="A" * 43), "/signer: Additional properties"),
    (lambda d: d["method"].update(url="https://x"), "/method: Additional properties"),
    (lambda d: d.pop("operator"), "'operator' is a required property"),
    (lambda d: d.update(format="be-lab-config/0.2"), "/format:"),
    (lambda d: d["machine"].update({"class": "drone"}), "/machine/class:"),
    (lambda d: d["gate"].update(placement="cloud"), "/gate/placement:"),
    (lambda d: d["method"].update(text_sha256="abc"), "/method/text_sha256:"),
    (lambda d: d["instruments"][0].update(rate_hz=0), "/instruments/0/rate_hz:"),
    (lambda d: d["instruments"][0]["accuracy_check"].update(at="yesterday"), "/accuracy_check/at:"),
    (lambda d: d["instruments"][0]["calibration"].update(files=["/home/ana/cal.json"]), "/calibration/files/0:"),
    (lambda d: d["instruments"].append(dict(d["instruments"][0])), "/instruments:"),
    (lambda d: d["instruments"][0].update(primary=False), "/instruments:"),
    (lambda d: d["instruments"][0].update(kind="robot_encoders_fk", external=True), "/instruments/0/external:"),
    (lambda d: d["instruments"][0].update(kind="markerless_model_tracker", external=True),
     "'independent_of_robot_state' is a required property"),
    (lambda d: d.update(operator="line\nbreak"), "/operator:"),
])
def test_unknown_missing_and_malformed_members_are_refused(mutate, where):
    problems = _bad(mutate)
    assert any(where in p for p in problems), problems


def test_the_file_is_parsed_strictly(tmp_path):
    dup = tmp_path / "dup.json"
    dup.write_text('{"format": "be-lab-config/0.1", "format": "be-lab-config/0.1"}')
    with pytest.raises(LabConfigError, match="duplicate"):
        load_lab_config(dup)
    nan = tmp_path / "nan.json"
    nan.write_text(json.dumps(GOOD).replace('"rate_hz": 25.0', '"rate_hz": NaN'))
    with pytest.raises(LabConfigError, match="not strict JSON"):
        load_lab_config(nan)
    with pytest.raises(LabConfigError, match="cannot read"):
        load_lab_config(tmp_path / "missing.json")
    many = copy.deepcopy(GOOD)
    many.pop("operator")
    many["extra"] = 1
    with pytest.raises(LabConfigError) as e:
        load_lab_config(sup.write_json(tmp_path / "many.json", many))
    assert len(e.value.problems) == 2
