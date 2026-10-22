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
``be-stops/1``
    The EV-02 trials (:func:`stop_stream`) of ``ev02-stop-plan.json``, one segment each: (for a
    position command space, after the first trial) a ``return`` walk back to the plan's start,
    a ``drive:<id>`` walk toward the trial's drive, then one ``stop:<source>`` proposal with an
    empty payload, which the bench issues through that stop source instead of sending it. Params:
    ``{"hz", "drive_s", "return_s", "step_cap"}``.
``be-model-loss/1``
    The EV-05 trials (:func:`model_loss_stream`) of ``ev05-model-loss-plan.json``, one segment
    each: ``motion`` commands walking round the plan's path for a seeded time, then nothing for
    ``silence_s`` (the model is lost), then ``restart`` commands that walk on along the path for
    half the restart and back to the last motion command for the other half. Params:
    ``{"hz", "trials", "motion_s", "silence_s", "restart_s", "step_cap"}``.

be-control, be-waypoints and be-stops have no randomness; be-model-loss draws one motion time per
trial from the seed. Their digest has the same shape as ``stream_sha256``: SHA-256 over the UTF-8
text::

    <name>/1
    <RFC 8785 JSON of the normalized params>
    <seed>
    <command space digest>
    <waypoints or plan digest>              (be-waypoints, be-stops, be-model-loss only)
    <seq>,<strategy>,<payload_json>         (one line per command, in order)

Segments. A be-stops or be-model-loss stream is a list of segments, one per trial; each
proposal's ``t_s`` is counted from the start of its own segment, so the first proposal of every
segment has ``t_s`` 0 (:func:`split_segments`). The bench pauses between segments; the digest
and the comparison with commands.csv never look at ``t_s``.

Finding codes: STREAM_PARAMS_INVALID, STREAM_TOO_LARGE, GENERATOR_UNSUPPORTED (errors), and the
generator's STREAM_SEQ_GAP, STREAM_TOO_LONG, STREAM_MISMATCH (errors) and STREAM_PREFIX (info).
"""

from __future__ import annotations

import hashlib
import math
import random
from collections.abc import Iterable, Iterator, Mapping, Sequence
from dataclasses import dataclass

from .. import canonical, generator
from ..commandspace import CommandSpace
from ..findings import Finding
from ..generator import Proposal, encode_payload
from .plans import ModelLossPlan, StopPlan, check_plan_commands
from .waypoints import Waypoints

__all__ = [
    "CONTROL",
    "DRIVE_PREFIX",
    "GENERATORS",
    "HOSTILE",
    "MAX_STREAM_COMMANDS",
    "MODEL_LOSS",
    "MOTION",
    "RESTART",
    "RETURN",
    "STOPS",
    "STOP_PREFIX",
    "VERSION",
    "WAYPOINTS",
    "StreamCheck",
    "check",
    "control_stream",
    "model_loss_stream",
    "normalize",
    "split_segments",
    "stop_stream",
    "stream",
    "stream_digest",
    "ticks",
    "walk_step",
    "waypoint_stream",
]

HOSTILE = generator.GENERATOR  # "be-hostile"
CONTROL = "be-control"
WAYPOINTS = "be-waypoints"
STOPS = "be-stops"
MODEL_LOSS = "be-model-loss"
VERSION = "1"
GENERATORS = (HOSTILE, CONTROL, WAYPOINTS, STOPS, MODEL_LOSS)

MAX_STREAM_COMMANDS = 2_000_000
"""A declared stream longer than this is refused (STREAM_TOO_LARGE) rather than regenerated: a
600 s EV-03 run at 50 Hz with 500 Hz bursts is under 100,000 commands."""

_CONTROL_KEYS = ("target", "seconds", "hz", "step_cap", "start")
_WAYPOINT_KEYS = ("cycles", "commands_per_visit", "hz")
_STOP_KEYS = ("hz", "drive_s", "return_s", "step_cap")
_LOSS_KEYS = ("hz", "trials", "motion_s", "silence_s", "restart_s", "step_cap")
_POSITION_KINDS = ("joint_position", "cartesian_position")
_MAX_TRIALS = 1000
#: Strategies of the EV-02 and EV-05 streams (a drive and a stop carry their id after the colon).
DRIVE_PREFIX, STOP_PREFIX, RETURN = "drive:", "stop:", "return"
MOTION, RESTART = "motion", "restart"


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


def _non_negative(name: str, value: object, upper: float) -> float:
    x = _finite(name, value)
    if not 0.0 <= x <= upper:
        raise ValueError(f"{name} must be from 0 to {upper:g}, not {value!r}")
    return x


def _caps(value: object, required: bool) -> dict | None:
    """step_cap: null (only when not ``required``) or {axis key: cap above 0}. Whether it names
    every axis of the command space is checked when the stream is made."""
    if value is None and not required:
        return None
    if not isinstance(value, Mapping) or not value:
        need = "a mapping of axis to cap" if required else "null or a mapping of axis to cap"
        raise ValueError(f"step_cap must be {need}, not {value!r}")
    return {str(k): _number(f"step_cap[{k!r}]", v, math.inf) for k, v in value.items()}


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
    if name == STOPS:
        p = _exact_keys(params, _STOP_KEYS, STOPS)
        return {
            "hz": _number("hz", p["hz"], generator.MAX_HZ),
            "drive_s": _number("drive_s", p["drive_s"], generator.MAX_SECONDS),
            "return_s": _non_negative("return_s", p["return_s"], generator.MAX_SECONDS),
            "step_cap": _caps(p["step_cap"], required=False),
        }
    if name == MODEL_LOSS:
        p = _exact_keys(params, _LOSS_KEYS, MODEL_LOSS)
        span = p["motion_s"]
        if not isinstance(span, (list, tuple)) or len(span) != 2:
            raise ValueError(f"motion_s must be [shortest, longest], not {span!r}")
        lo = _number("motion_s[0]", span[0], generator.MAX_SECONDS)
        hi = _number("motion_s[1]", span[1], generator.MAX_SECONDS)
        if lo > hi:
            raise ValueError(f"motion_s must not be decreasing, not {span!r}")
        hz = _number("hz", p["hz"], generator.MAX_HZ)
        silence = _number("silence_s", p["silence_s"], generator.MAX_SECONDS)
        if silence < 1.0 / hz:
            raise ValueError(f"silence_s must be at least one period (1 / hz), not {silence!r}")
        return {
            "hz": hz,
            "trials": _count("trials", p["trials"], _MAX_TRIALS),
            "motion_s": [lo, hi],
            "silence_s": silence,
            "restart_s": _number("restart_s", p["restart_s"], generator.MAX_SECONDS),
            "step_cap": _caps(p["step_cap"], required=True),
        }
    raise ValueError(f"unknown generator {name!r}; known: {', '.join(GENERATORS)}")


def _expected_commands(
    name: str, p: dict, waypoints: Waypoints | None, plan: StopPlan | ModelLossPlan | None = None
) -> float:
    """An upper bound on the number of commands the stream holds."""
    if name == HOSTILE:
        return p["seconds"] * max(p["base_hz"], p["burst_hz"]) + 1.0
    if name == CONTROL:
        return p["seconds"] * p["hz"] + 1.0
    if name == STOPS:
        trials = len(plan.trials) if isinstance(plan, StopPlan) else 0
        return float(trials) * ((p["drive_s"] + p["return_s"]) * p["hz"] + 3.0)
    if name == MODEL_LOSS:
        return float(p["trials"]) * ((p["motion_s"][1] + p["restart_s"]) * p["hz"] + 3.0)
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


# --------------------------------------------------------------------------- EV-02 and EV-05


def ticks(seconds: float, hz: float) -> int:
    """How many proposals ``i = 0, 1, 2, ...`` have ``i / hz < seconds``, each division in
    binary64 (the rule every stream here schedules by)."""
    n = max(0, math.ceil(seconds * hz))
    while n > 0 and (n - 1) / hz >= seconds:
        n -= 1
    while n / hz < seconds:
        n += 1
    return n


def _check_caps(space: CommandSpace, caps: Mapping | None) -> Mapping | None:
    if caps is None:
        return None
    unknown = [k for k in caps if k not in space.keys]
    missing = [k for k in space.keys if k not in caps]
    if unknown or missing:
        raise ValueError(
            "step_cap must name every axis and nothing else"
            + (f"; unknown: {unknown}" if unknown else "")
            + (f"; missing: {missing}" if missing else "")
        )
    return caps


def _check_plan(plan: StopPlan | ModelLossPlan, space: CommandSpace, file: str) -> None:
    problems = check_plan_commands(plan, space, file)
    if problems:
        raise ValueError(f"{file}: {problems[0].detail} ({problems[0].where})")


def walk_step(
    current: Mapping[str, float],
    target: Mapping[str, float],
    keys: Sequence[str],
    caps: Mapping[str, float] | None,
) -> dict[str, float]:
    """One command of a walk (the control stream's rule, method §7.6): each axis, in declared
    order, lands exactly on its target when it is within its cap (or there is no cap), and
    otherwise moves by its cap toward it."""
    out = dict(current)
    for key in keys:
        want, now = target[key], out[key]
        if caps is None or abs(want - now) <= caps[key]:
            out[key] = want
        elif want > now:
            out[key] = now + caps[key]
        else:
            out[key] = now - caps[key]
    return out


def stop_stream(space: CommandSpace, plan: StopPlan, params: Mapping) -> Iterator[Proposal]:
    """The be-stops/1 stream: the EV-02 trials of ``plan``, one segment each (module docstring).

    Each trial walks from where the previous one left the commands: for a position command space
    (after the first trial) a ``return`` walk back to the plan's start for ``return_s``, then a
    ``drive:<id>`` walk toward the trial's drive for ``drive_s``; for a velocity command space the
    walk starts again from the start for every drive, and there is no return. The walk is the
    control stream's (:func:`walk_step`), at most ``step_cap`` per axis per command (or straight to
    the target with ``step_cap`` null). Then one ``stop:<source>`` proposal, payload ``{}``, one
    period after the last drive command. Proposal ``i`` of a segment is scheduled at ``i / hz``.
    No randomness."""
    p = normalize(STOPS, params)
    caps = _check_caps(space, p["step_cap"])
    _check_plan(plan, space, "ev02-stop-plan.json")
    return _stops(space, plan, p, caps)


def _stops(space: CommandSpace, plan: StopPlan, p: dict, caps) -> Iterator[Proposal]:
    hz, keys = p["hz"], space.keys
    position = space.kind in _POSITION_KINDS
    start = {k: plan.start[k] for k in keys}
    n_drive = ticks(p["drive_s"], hz)
    n_return = ticks(p["return_s"], hz) if position else 0
    current = dict(start)
    seq = 0
    for k, (source, drive_id) in enumerate(plan.trials):
        target = plan.drive(drive_id).command
        i = 0
        if not position:
            current = dict(start)
        elif k > 0:
            for _ in range(n_return):
                current = walk_step(current, start, keys, caps)
                yield Proposal(seq, i / hz, RETURN, dict(current))
                seq, i = seq + 1, i + 1
        for _ in range(n_drive):
            current = walk_step(current, target, keys, caps)
            yield Proposal(seq, i / hz, DRIVE_PREFIX + drive_id, dict(current))
            seq, i = seq + 1, i + 1
        yield Proposal(seq, i / hz, STOP_PREFIX + source, {})
        seq += 1


def model_loss_stream(
    space: CommandSpace, plan: ModelLossPlan, seed: int, params: Mapping
) -> Iterator[Proposal]:
    """The be-model-loss/1 stream: ``trials`` segments, each a model that moves the machine and
    then dies (module docstring).

    One walk runs through the motion of the whole stream: from the plan's start toward
    ``path[0]``, and on to each next path command once the walk has landed on the current one,
    round the path again after the last, at most ``step_cap`` per axis per command
    (:func:`walk_step`). In each trial the motion lasts ``m`` seconds, ``m = uniform(motion_s[0],
    motion_s[1])`` drawn from ``random.Random(seed).random()`` (one draw per trial, as be-hostile/1
    draws, §7.5.2): ``n`` ``motion`` commands at ``i / hz`` with ``i / hz < m``. Then nothing for
    ``silence_s``. Then ``r`` restart commands (``j / hz < restart_s``) at ``(n - 1) / hz +
    silence_s + j / hz``: the first ``r // 2`` walk on round the path; the rest walk back, at most
    ``step_cap`` per axis per command, toward the last motion command, which they reach by the
    last restart command. The next trial's motion continues the walk from there, as if the restart
    had not happened: the machine that held still through the restart is where the motion
    continues, and one that followed the restart is back there too."""
    p = normalize(MODEL_LOSS, params)
    caps = _check_caps(space, p["step_cap"])
    _check_plan(plan, space, "ev05-model-loss-plan.json")
    return _losses(space, plan, _seed(seed), p, caps)


def _losses(space: CommandSpace, plan: ModelLossPlan, seed: int, p: dict, caps) -> Iterator[Proposal]:
    rng = random.Random(seed)
    hz, keys = p["hz"], space.keys
    lo, hi = p["motion_s"]
    path = [{k: c[k] for k in keys} for c in plan.path]
    current = {k: plan.start[k] for k in keys}
    cursor = 0
    n_restart = ticks(p["restart_s"], hz)
    n_out = n_restart // 2
    seq = 0
    for _ in range(p["trials"]):
        n_motion = ticks(generator._uniform(rng, lo, hi), hz)
        for i in range(n_motion):
            current, cursor = _round(current, path, cursor, keys, caps)
            yield Proposal(seq, i / hz, MOTION, dict(current))
            seq += 1
        t0 = (n_motion - 1) / hz + p["silence_s"]
        lost, out, out_cursor = dict(current), dict(current), cursor
        for j in range(n_restart):
            if j < n_out:  # on round the path ...
                out, out_cursor = _round(out, path, out_cursor, keys, caps)
            else:  # ... and back to the last motion command (n_restart - n_out >= n_out steps)
                out = walk_step(out, lost, keys, caps)
            yield Proposal(seq, t0 + j / hz, RESTART, dict(out))
            seq += 1


def _round(current, path, cursor, keys, caps):
    target = path[cursor]
    nxt = walk_step(current, target, keys, caps)
    if all(nxt[k] == target[k] for k in keys):
        cursor = (cursor + 1) % len(path)
    return nxt, cursor


def split_segments(proposals: Iterable[Proposal]) -> list[list[Proposal]]:
    """A be-stops or be-model-loss stream cut into its segments: a segment starts wherever
    ``t_s`` does not increase (every segment's first proposal is scheduled at 0)."""
    out: list[list[Proposal]] = []
    last = None
    for p in proposals:
        if last is None or p.t_s <= last:
            out.append([])
        out[-1].append(p)
        last = p.t_s
    return out


def stream(
    name: str,
    space: CommandSpace,
    seed: int,
    params: Mapping,
    waypoints: Waypoints | None = None,
    plan: StopPlan | ModelLossPlan | None = None,
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
    if name == STOPS:
        if not isinstance(plan, StopPlan):
            raise ValueError("be-stops needs the run's ev02-stop-plan.json")
        return stop_stream(space, plan, params)
    if name == MODEL_LOSS:
        if not isinstance(plan, ModelLossPlan):
            raise ValueError("be-model-loss needs the run's ev05-model-loss-plan.json")
        return model_loss_stream(space, plan, seed, params)
    raise ValueError(f"unknown generator {name!r}; known: {', '.join(GENERATORS)}")


def _seed(seed: object) -> int:
    if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed <= generator.MAX_SEED:
        raise ValueError(f"seed must be an integer from 0 to 2**53, not {seed!r}")
    return seed


def stream_digest(
    name: str,
    space: CommandSpace,
    seed: int,
    params: Mapping,
    waypoints: Waypoints | None = None,
    plan: StopPlan | ModelLossPlan | None = None,
) -> str:
    """``"sha256:"`` + hex digest of the whole stream (module docstring); for be-hostile exactly
    ``generator.stream_sha256``."""
    if name == HOSTILE:
        return generator.stream_sha256(space, seed, params)
    p = normalize(name, params)
    proposals = stream(name, space, seed, p, waypoints, plan)
    h = hashlib.sha256()
    header = (
        f"{name}/{VERSION}\n{canonical.canonicalize(p).decode('utf-8')}\n{seed}\n{space.digest}\n"
    )
    if name == WAYPOINTS:
        assert waypoints is not None
        header += f"{waypoints.digest}\n"
    if name in (STOPS, MODEL_LOSS):
        assert plan is not None
        header += f"{plan.digest}\n"
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
    plan: StopPlan | ModelLossPlan | None = None,
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
    bound = _expected_commands(name, p, waypoints, plan)
    if bound > MAX_STREAM_COMMANDS:
        detail = f"the declared stream would hold up to {bound:.0f} commands; core regenerates at most {MAX_STREAM_COMMANDS}"
        return StreamCheck(
            (Finding("STREAM_TOO_LARGE", "error", detail, "/generator/params"),), None, None, None
        )
    try:
        digest = stream_digest(name, space, seed, p, waypoints, plan)
        regenerated = stream(name, space, seed, p, waypoints, plan)
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
