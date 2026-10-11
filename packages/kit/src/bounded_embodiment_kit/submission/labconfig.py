# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""The lab configuration `be report --lab-config` reads: the declared fields the runner cannot
know (``be-lab-config/0.1``, schema ``lab-config-0.1.json`` beside this module).

The schema takes each definition from core's test report 0.1 schema by reference, so a lab
config that validates gives report members the report schema accepts. Unknown members are
refused at every level. Validation is local: the report schema comes from the installed core,
nothing is fetched.
"""
from __future__ import annotations

import functools
from importlib import resources
from pathlib import Path

from bounded_embodiment import schema as core_schema
from bounded_embodiment.canonical import CanonicalError, loads_strict
from jsonschema import Draft202012Validator
from referencing import Registry, Resource
from referencing.jsonschema import DRAFT202012

__all__ = ["FORMAT", "LabConfigError", "load_lab_config", "validate_lab_config"]

FORMAT = "be-lab-config/0.1"
SCHEMA_FILE = "lab-config-0.1.json"
MAX_BYTES = 1024 * 1024


class LabConfigError(ValueError):
    """The lab config is unusable; ``problems`` says why, one line each."""

    def __init__(self, problems: list[str]):
        self.problems = list(problems)
        super().__init__("; ".join(self.problems[:3]) + (f" (+{len(self.problems) - 3} more)"
                                                          if len(self.problems) > 3 else ""))


@functools.lru_cache(maxsize=1)
def _validator() -> Draft202012Validator:
    lab_schema = loads_strict(resources.files(__package__).joinpath(SCHEMA_FILE).read_bytes())
    report_schema = core_schema.get_schema("test-report-0.1")
    registry = Registry().with_resource(
        report_schema["$id"], Resource.from_contents(report_schema, default_specification=DRAFT202012))
    return Draft202012Validator(lab_schema, registry=registry)


def _pointer(path) -> str:
    return "".join("/" + str(p).replace("~", "~0").replace("/", "~1") for p in path) or "/"


def validate_lab_config(doc: object) -> list[str]:
    """Every way ``doc`` breaks the lab config schema, as ``<JSON pointer>: <message>`` lines,
    sorted; empty when it is valid."""
    problems = {f"{_pointer(e.absolute_path)}: {e.message}" for e in _validator().iter_errors(doc)}
    return sorted(problems)


def load_lab_config(path: str | Path) -> dict:
    """The lab config at ``path``, strictly parsed (no duplicate keys, NaN or Infinity) and
    validated; LabConfigError otherwise."""
    p = Path(path)
    try:
        data = p.read_bytes()
    except OSError as e:
        raise LabConfigError([f"cannot read {p}: {e.strerror or e}"]) from None
    if len(data) > MAX_BYTES:
        raise LabConfigError([f"{p} is larger than {MAX_BYTES} bytes"])
    try:
        doc = loads_strict(data)
    except CanonicalError as e:
        raise LabConfigError([f"{p} is not strict JSON: {e}"]) from None
    problems = validate_lab_config(doc)
    if problems:
        raise LabConfigError(problems)
    assert isinstance(doc, dict)
    return doc
