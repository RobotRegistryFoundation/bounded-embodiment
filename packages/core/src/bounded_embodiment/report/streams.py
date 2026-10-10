"""The command streams a report's ``generator`` block can name, and the check of a run's
``raw/commands.csv`` against them.

``be-hostile/1``
    :func:`bounded_embodiment.generator.hostile_stream` (EV-03). Its digest is
    :func:`bounded_embodiment.generator.stream_sha256`.
``be-control/1``
    :func:`bounded_embodiment.generator.control_stream` over one lab far target (an EV-03 control
    or rehearsal). Params: ``{"target", "seconds", "hz", "step_cap", "start"}`` (``start`` is
    the declared start pose, every axis, or null for each axis's home).
``be-waypoints/1``
    The EV-10 tour (:func:`waypoint_stream`): the waypoints of ``ev10-waypoints.json`` in declared
    order, ``cycles`` times round, each held for ``commands_per_visit`` commands at ``hz``; every
    command is the waypoint's declared command with the strategy ``waypoint:<id>``. Params:
    ``{"cycles", "commands_per_visit", "hz"}``.

be-control and be-waypoints have no randomness. Their digest has the same shape as
``stream_sha256``: SHA-256 over the UTF-8 text::

    <name>/1
    <RFC 8785 JSON of the normalized params>
    <seed>
    <command space digest>
    <waypoints digest>                      (be-waypoints only)
    <seq>,<strategy>,<payload_json>         (one line per command, in order)

Finding codes: STREAM_PARAMS_INVALID, STREAM_TOO_LARGE, GENERATOR_UNSUPPORTED (errors), and the
generator's STREAM_SEQ_GAP, STREAM_TOO_LONG, STREAM_MISMATCH (errors) and STREAM_PREFIX (info).
"""

from __future__ import annotations

import hashlib
import math
from collections.abc import Iterable, Iterator, Mapping
from dataclasses import dataclass

from .. import canonical, generator
from ..commandspace import CommandSpace
from ..findings import Finding
from ..generator import Proposal, encode_payload
from .waypoints import Waypoints

__all__ = [
    "CONTROL",
    "GENERATORS",
    "HOSTILE",
    "MAX_STREAM_COMMANDS",
    "VERSION",
    "WAYPOINTS",
    "StreamCheck",
    "check",
    "control_stream",
    "normalize",
    "stream",
    "stream_digest",
    "waypoint_stream",
]

HOSTILE = generator.GENERATOR  # "be-hostile"
CONTROL = "be-control"
WAYPOINTS = "be-waypoints"
VERSION = "1"
GENERATORS = (HOSTILE, CONTROL, WAYPOINTS)

MAX_STREAM_COMMANDS = 2_000_000
"""A declared stream longer than this is refused (STREAM_TOO_LARGE) rather than regenerated: a
600 s EV-03 run at 50 Hz with 500 Hz bursts is under 100,000 commands."""

_CONTROL_KEYS = ("target", "seconds", "hz", "step_cap", "start")
_WAYPOINT_KEYS = ("cycles", "commands_per_visit", "hz")
_POSITION_KINDS = ("joint_position", "cartesian_position")


# --------------------------------------------------------------------------- parameters


def _number(name: str, value: object, upper: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number, not {value!r}")  # noqa: TRY004
    try:
        x = float(value) + 0.0
    except OverflowError:
        raise ValueError(f"{name} {value!r} is too large") from None
    if not math.isfinite(x) or not 0.0 < x <= upper:
        raise ValueError(f"{name} must be above 0 and at most {upper:g}, not {value!r}")
    return x


def _finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number, not {value!r}")  # noqa: TRY004
    try:
        x = float(value) + 0.0
    except OverflowError:
        raise ValueError(f"{name} {value!r} is too large") from None
    if not math.isfinite(x):
        raise ValueError(f"{name} must be finite, not {value!r}")
    return x


def _count(name: str, value: object, upper: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= upper:
        raise ValueError(f"{name} must be a whole number from 1 to {upper}, not {value!r}")
    return value


def _exact_keys(params: object, keys: tuple[str, ...], what: str) -> Mapping:
    if not isinstance(params, Mapping):
        raise ValueError(f"{what} params must be an object, not {type(params).__name__}")  # noqa: TRY004
    unknown = [k for k in params if k not in keys]
    missing = [k for k in keys if k not in params]
    if unknown or missing:
        raise ValueError(
            f"{what} params are exactly {list(keys)}"
            + (f"; unknown: {unknown}" if unknown else "")
            + (f"; missing: {missing}" if missing else "")
        )
    return params


def normalize(name: str, params: object) -> dict:
    """The params of generator ``name`` with numbers as floats (counts as integers) and every
    value checked; ValueError if any is unknown, missing or bad. Normalizing twice changes
    nothing."""
    if name == HOSTILE:
        if not isinstance(params, Mapping):
            raise ValueError(f"be-hostile params must be an object, not {type(params).__name__}")
        return generator.normalize_params(params)
    if name == CONTROL:
        p = _exact_keys(params, _CONTROL_KEYS, CONTROL)
        target = p["target"]
        if not isinstance(target, str) or not target:
            raise ValueError(f"target must be a far target id, not {target!r}")
        cap = p["step_cap"]
        if cap is not None:
            if not isinstance(cap, Mapping) or not cap:
                raise ValueError(f"step_cap must be null or a mapping of axis to cap, not {cap!r}")
            cap = {str(k): _number(f"step_cap[{k!r}]", v, math.inf) for k, v in cap.items()}
        start = p["start"]
        if start is not None:
            if not isinstance(start, Mapping) or not start:
                raise ValueError(f"start must be null or a mapping of axis to value, not {start!r}")
            start = {str(k): _finite(f"start[{k!r}]", v) for k, v in start.items()}
        return {
            "target": target,
            "seconds": _number("seconds", p["seconds"], generator.MAX_SECONDS),
            "hz": _number("hz", p["hz"], generator.MAX_HZ),
            "step_cap": cap,
            "start": start,
        }
    if name == WAYPOINTS:
        p = _exact_keys(params, _WAYPOINT_KEYS, WAYPOINTS)
        return {
            "cycles": _count("cycles", p["cycles"], 10_000),
            "commands_per_visit": _count("commands_per_visit", p["commands_per_visit"], 1_000_000),
            "hz": _number("hz", p["hz"], generator.MAX_HZ),
        }
    raise ValueError(f"unknown generator {name!r}; known: {', '.join(GENERATORS)}")


def _expected_commands(name: str, p: dict, waypoints: Waypoints | None) -> float:
    """An upper bound on the number of commands the stream holds."""
    if name == HOSTILE:
        return p["seconds"] * max(p["base_hz"], p["burst_hz"]) + 1.0
    if name == CONTROL:
        return p["seconds"] * p["hz"] + 1.0
    count = 0 if waypoints is None else len(waypoints.waypoints)
    return float(p["cycles"]) * float(p["commands_per_visit"]) * float(count)


# --------------------------------------------------------------------------- streams


def control_stream(space: CommandSpace, params: Mapping) -> Iterator[Proposal]:
    """The be-control stream for (normalized) params: ``generator.control_stream`` over the far
    target ``params["target"]`` (KeyError if the command space has no such far target)."""
    p = normalize(CONTROL, params)
    return generator.control_stream(
        space, p["target"], p["seconds"], p["hz"], p["step_cap"], p["start"]
    )


def waypoint_stream(
    space: CommandSpace, waypoints: Waypoints, params: Mapping
) -> Iterator[Proposal]:
    """The be-waypoints/1 stream: each declared waypoint in order, ``cycles`` times round, held for
    ``commands_per_visit`` commands; command ``seq`` is scheduled at ``seq / hz``. Each payload is
    the waypoint's command and each strategy ``waypoint:<id>``. The command space must hold
    positions (joint_position or cartesian_position), and every waypoint command must name each
    of its axes within limits (checked by ``waypoints.check_commands``)."""
    p = normalize(WAYPOINTS, params)
    if space.kind not in _POSITION_KINDS:
        raise ValueError(f"be-waypoints needs a position command space, not {space.kind!r}")
    if not waypoints.waypoints:
        raise ValueError("be-waypoints needs at least one waypoint")
    return _tour(waypoints, p)


def _tour(waypoints: Waypoints, p: dict) -> Iterator[Proposal]:
    hz, per = p["hz"], p["commands_per_visit"]
    seq = 0
    for _ in range(p["cycles"]):
        for w in waypoints.waypoints:
            strategy = "waypoint:" + w.id
            for _ in range(per):
                yield Proposal(seq, seq / hz, strategy, dict(w.command))
                seq += 1


def stream(
    name: str, space: CommandSpace, seed: int, params: Mapping, waypoints: Waypoints | None = None
) -> Iterator[Proposal]:
    """The stream generator ``name`` makes; ValueError (or KeyError for an unknown far target) if
    the arguments are refused."""
    if name == HOSTILE:
        return generator.hostile_stream(space, seed, params)
    _seed(seed)
    if name == CONTROL:
        return control_stream(space, params)
    if name == WAYPOINTS:
        if waypoints is None:
            raise ValueError("be-waypoints needs the run's ev10-waypoints.json")
        return waypoint_stream(space, waypoints, params)
    raise ValueError(f"unknown generator {name!r}; known: {', '.join(GENERATORS)}")


def _seed(seed: object) -> int:
    if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed <= generator.MAX_SEED:
        raise ValueError(f"seed must be an integer from 0 to 2**53, not {seed!r}")
    return seed


def stream_digest(
    name: str, space: CommandSpace, seed: int, params: Mapping, waypoints: Waypoints | None = None
) -> str:
    """``"sha256:"`` + hex digest of the whole stream (module docstring); for be-hostile exactly
    ``generator.stream_sha256``."""
    if name == HOSTILE:
        return generator.stream_sha256(space, seed, params)
    p = normalize(name, params)
    proposals = stream(name, space, seed, p, waypoints)
    h = hashlib.sha256()
    header = (
        f"{name}/{VERSION}\n{canonical.canonicalize(p).decode('utf-8')}\n{seed}\n{space.digest}\n"
    )
    if name == WAYPOINTS:
        assert waypoints is not None
        header += f"{waypoints.digest}\n"
    h.update(header.encode("utf-8"))
    for c in proposals:
        h.update(f"{c.seq},{c.strategy},{encode_payload(c.payload)}\n".encode())
    return "sha256:" + h.hexdigest()


# --------------------------------------------------------------------------- checking commands.csv


@dataclass(frozen=True)
class StreamCheck:
    """The derived generator members, and the findings of the comparison. A member is None when
    it could not be derived (an error finding says why)."""

    findings: tuple[Finding, ...]
    stream_sha256: str | None
    commands_compared: int | None
    aborted: bool | None


def _clip(text: str, limit: int = 400) -> str:
    return text if len(text) <= limit else text[: limit - 3] + "..."


def check(
    name: object,
    version: object,
    space: CommandSpace,
    seed: object,
    params: object,
    rows: Iterable[tuple],
    waypoints: Waypoints | None = None,
) -> StreamCheck:
    """Regenerate the declared stream, digest it, and compare it with ``(seq, strategy,
    payload_json)`` rows. The rows may be a prefix of the stream (a run that stopped early: then
    ``aborted`` is True and a STREAM_PREFIX info finding says so). Never raises for bad
    declarations: they become findings."""
    if name not in GENERATORS or version != VERSION:
        detail = f"generator {name!r} version {version!r} is not one core can regenerate ({', '.join(g + '/1' for g in GENERATORS)})"
        return StreamCheck(
            (Finding("GENERATOR_UNSUPPORTED", "error", detail, "/generator"),), None, None, None
        )
    assert isinstance(name, str)
    try:
        _seed(seed)
        p = normalize(name, params)
        assert isinstance(seed, int)
    except ValueError as e:
        return StreamCheck(
            (Finding("STREAM_PARAMS_INVALID", "error", _clip(str(e)), "/generator"),),
            None,
            None,
            None,
        )
    bound = _expected_commands(name, p, waypoints)
    if bound > MAX_STREAM_COMMANDS:
        detail = f"the declared stream would hold up to {bound:.0f} commands; core regenerates at most {MAX_STREAM_COMMANDS}"
        return StreamCheck(
            (Finding("STREAM_TOO_LARGE", "error", detail, "/generator/params"),), None, None, None
        )
    try:
        digest = stream_digest(name, space, seed, p, waypoints)
        regenerated = stream(name, space, seed, p, waypoints)
    except (ValueError, KeyError) as e:
        text = e.args[0] if isinstance(e, KeyError) and e.args else str(e)
        return StreamCheck(
            (Finding("STREAM_PARAMS_INVALID", "error", _clip(str(text)), "/generator"),),
            None,
            None,
            None,
        )
    rows = list(rows)
    found = generator.compare_rows(regenerated, rows)
    if any(f.severity == "error" for f in found):
        return StreamCheck(tuple(found), digest, None, None)
    aborted = any(f.code == "STREAM_PREFIX" for f in found)
    return StreamCheck(tuple(found), digest, len(rows), aborted)
