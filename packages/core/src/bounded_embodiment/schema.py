"""Validate documents against the JSON Schemas shipped inside this package.

``validate(doc, schema_name)`` takes the file stem of a schema under
``bounded_embodiment/schemas/`` (for example ``"envelope-0.2"`` or
``"key-0.1"``) and returns one ``SCHEMA_INVALID`` finding per violation, with a
JSON Pointer to the offending part of ``doc`` in ``where``. An unknown name
raises ``KeyError``.

Every schema file is parsed strictly (duplicate keys are refused) and
registered under its ``$id``. A reference that no ``$id`` matches is resolved
by its last path segment against the shipped file names, so
``"envelope-0.1.json#/$defs/polygon"`` finds the local copy whatever base URI
it was written against. Nothing is ever fetched over the network: anything
else is unresolvable.

Draft 2020-12 treats ``format`` as an annotation. Here ``date``, ``date-time``
and ``email`` are asserted, with checks that need no optional packages, so a
document validates the same way on every installation. Other formats stay
annotations; schemas that need a hard rule should use ``pattern``.
"""

from __future__ import annotations

import functools
import re
from collections.abc import Iterable
from datetime import date
from importlib import resources
from typing import Any

from jsonschema import Draft202012Validator, FormatChecker
from jsonschema.exceptions import ValidationError, best_match
from jsonschema.validators import validator_for
from referencing import Registry, Resource
from referencing.exceptions import NoSuchResource
from referencing.jsonschema import DRAFT202012

from .canonical import loads_strict
from .findings import Finding

__all__ = ["get_schema", "schema_names", "validate"]

_MAX_DETAIL = 400

# --------------------------------------------------------------------------- formats

_DATE = re.compile(r"([0-9]{4})-([0-9]{2})-([0-9]{2})")
_TIME = re.compile(r"([0-9]{2}):([0-9]{2}):([0-9]{2})(\.[0-9]+)?([Zz]|[+-]([0-9]{2}):([0-9]{2}))")
_EMAIL = re.compile(r"[^@\s]+@[^@\s]+")

FORMAT_CHECKER = FormatChecker(formats=())


def _valid_date(text: str) -> bool:
    m = _DATE.fullmatch(text)
    if m is None:
        return False
    try:
        date(int(m.group(1)), int(m.group(2)), int(m.group(3)))
    except ValueError:
        return False
    return True


@FORMAT_CHECKER.checks("date")
def _is_date(instance: object) -> bool:
    return not isinstance(instance, str) or _valid_date(instance)


@FORMAT_CHECKER.checks("date-time")
def _is_date_time(instance: object) -> bool:
    """RFC 3339 date-time (a leap second, :60, is accepted)."""
    if not isinstance(instance, str):
        return True
    day, sep, time = instance.partition("T") if "T" in instance else instance.partition("t")
    if not sep or not _valid_date(day):
        return False
    m = _TIME.fullmatch(time)
    if m is None:
        return False
    hour, minute, second = int(m.group(1)), int(m.group(2)), int(m.group(3))
    if hour > 23 or minute > 59 or second > 60:
        return False
    if m.group(6) is not None and (int(m.group(6)) > 23 or int(m.group(7)) > 59):
        return False
    return True


@FORMAT_CHECKER.checks("email")
def _is_email(instance: object) -> bool:
    return not isinstance(instance, str) or _EMAIL.fullmatch(instance) is not None


# --------------------------------------------------------------------------- catalog


def _pointer(path: Iterable[object]) -> str:
    return "".join("/" + str(part).replace("~", "~0").replace("/", "~1") for part in path)


def _detail(error: ValidationError) -> str:
    message = error.message
    if error.validator in ("anyOf", "oneOf") and error.context:
        closest = best_match(error.context)
        if closest is not None:
            message = (
                f"does not match any of the allowed forms ({error.validator}); "
                f"closest: {closest.message}"
            )
    if len(message) > _MAX_DETAIL:
        message = message[: _MAX_DETAIL - 3] + "..."
    return message


class _Catalog:
    """The schemas of one directory, with a registry that resolves only locally."""

    def __init__(self, directory: Any) -> None:
        self._schemas: dict[str, dict] = {}
        self._by_filename: dict[str, Resource] = {}
        pairs: list[tuple[str, Resource]] = []
        seen_ids: dict[str, str] = {}
        for entry in sorted(directory.iterdir(), key=lambda e: e.name):
            if not entry.name.endswith(".json") or not entry.is_file():
                continue
            schema = loads_strict(entry.read_bytes())
            if not isinstance(schema, dict):
                raise ValueError(f"{entry.name} is not a JSON Schema object")
            resource = Resource.from_contents(schema, default_specification=DRAFT202012)
            self._schemas[entry.name[: -len(".json")]] = schema
            self._by_filename[entry.name] = resource
            uri = schema.get("$id")
            if isinstance(uri, str) and uri:
                if uri in seen_ids:
                    raise ValueError(f"{entry.name} and {seen_ids[uri]} both declare $id {uri}")
                seen_ids[uri] = entry.name
                pairs.append((uri, resource))
            pairs.append((f"urn:bounded-embodiment:schema:{entry.name}", resource))
        self._registry: Registry = Registry(retrieve=self._retrieve).with_resources(pairs).crawl()
        self._validators: dict[str, Any] = {}

    def _retrieve(self, uri: str) -> Resource:
        last = uri.split("#", 1)[0].rstrip("/").rsplit("/", 1)[-1].rsplit(":", 1)[-1]
        resource = self._by_filename.get(last) or self._by_filename.get(last + ".json")
        if resource is None:
            raise NoSuchResource(ref=uri)
        return resource

    def names(self) -> list[str]:
        return sorted(self._schemas)

    def schema(self, schema_name: str) -> dict:
        name = schema_name[: -len(".json")] if schema_name.endswith(".json") else schema_name
        try:
            return self._schemas[name]
        except KeyError:
            raise KeyError(
                f"unknown schema {schema_name!r}; known schemas: {', '.join(self.names()) or 'none'}"
            ) from None

    def validator(self, schema_name: str) -> Any:
        schema = self.schema(schema_name)
        key = schema_name[: -len(".json")] if schema_name.endswith(".json") else schema_name
        validator = self._validators.get(key)
        if validator is None:
            cls = validator_for(schema, default=Draft202012Validator)
            validator = cls(schema, registry=self._registry, format_checker=FORMAT_CHECKER)
            self._validators[key] = validator
        return validator

    def validate(self, doc: object, schema_name: str) -> list[Finding]:
        validator = self.validator(schema_name)
        findings: list[Finding] = []
        seen: set[tuple[str, str]] = set()
        for error in validator.iter_errors(doc):
            where, detail = _pointer(error.absolute_path), _detail(error)
            if (where, detail) not in seen:
                seen.add((where, detail))
                findings.append(Finding("SCHEMA_INVALID", "error", detail, where))
        findings.sort(key=lambda f: (f.where, f.detail))
        return findings


@functools.lru_cache(maxsize=1)
def _package_catalog() -> _Catalog:
    return _Catalog(resources.files("bounded_embodiment").joinpath("schemas"))


def validate(doc: object, schema_name: str) -> list[Finding]:
    """Validate ``doc`` against a shipped schema; ``[]`` means valid."""
    return _package_catalog().validate(doc, schema_name)


def schema_names() -> list[str]:
    """The names ``validate`` accepts, sorted."""
    return _package_catalog().names()


def get_schema(schema_name: str) -> dict:
    """A copy of a shipped schema, by name."""
    import copy

    return copy.deepcopy(_package_catalog().schema(schema_name))
