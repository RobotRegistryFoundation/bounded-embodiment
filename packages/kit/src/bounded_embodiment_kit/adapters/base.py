# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""DRAFT — requires line-by-line human review before use on hardware.

The adapter contract: the one place where the kit touches the system under test.

A lab implements an adapter at the point where its model issues commands. Whatever sits behind
``send()`` (the lab's gate, a LeRobot wrapper Robot, a gateway) is the system under test; the kit
never looks inside it.

Who calls what, from which thread (the runner relies on this):

=================  =====================  ===================================================
method             called from            what it may do
=================  =====================  ===================================================
``open()``         main thread, once      connect safely; return the declared command space
``describe()``     any thread             pure data, no I/O
``send(cmd)``      command thread         hand ``cmd`` to the system under test; never raises
``read_state()``   command thread         read the robot's own joint state from the bus
``latch_stop()``   ANY thread             set a flag and return; never touches the bus, never
                                          blocks (the watchdog calls it)
``execute_stop()`` command thread         the stop itself, under the bus lock
``close()``        main thread, once      leave the robot safe (torque stays on unless the
                                          operator types OFF), release the bus
=================  =====================  ===================================================

``execute_stop()`` is not in the approved plan's six-method sketch. The plan says "latch_stop()
sets a latch; the command thread executes the stop under the bus lock"; the command thread needs a
named entry point for that, and overloading ``send()`` would be cleverness. ``send()`` still
refuses every command once the latch is set, as a backstop.

Stop rule (lesson 3 of the plan): the goals a stop holds are chosen ONCE per latch (an arm: the
present positions, read at the first stop, because a joint may be moving) and re-sent unchanged
on every later stop. A later stop must never re-read and re-anchor to the present (sagging)
position: on bob that ratcheted the arm down a little on every repeated stop.

EV-02 and EV-05 (``be run ev02|ev05``) use two OPTIONAL methods more, and two describe() keys.
An adapter without them can run EV-03 and EV-10 exactly as before; the runner refuses an EV-02 or
EV-05 plan that needs what an adapter does not declare.

=====================  =====================  ===================================================
method                 called from            what it may do
=====================  =====================  ===================================================
``trigger_stop(src)``  command thread         EV-02: issue the system under test's OWN stop
                                              through ``src`` (only "stop_message"; the external
                                              sources go through the lab's --estop-cmd). It is
                                              the system's stop, never the runner's: it does not
                                              latch, and it must return promptly.
``reset_stop(src)``    command thread         after the operator typed NEXT: let the system under
                                              test accept commands again after a stop or loss
                                              through ``src``. It must not move the machine.
=====================  =====================  ===================================================

``describe()["stop_triggers"]`` lists the sources ``trigger_stop`` issues, and
``describe()["stop_resets"]`` those ``reset_stop`` can reset (both read after ``open()``). The
runner looks at its latch just before each call, but the watchdog may trip it in between, so an
adapter MUST refuse both while latched: look at the latch before taking the bus lock and again
under it, as ``send`` does. After a ``hardwired_stop`` or ``power_loss`` a reset MUST bring the
machine back the way ``connect_safely`` connects it: every goal register set to the present
position and read back, and only then torque on, so that nothing moves when power returns.
``call_resume`` calls a wrapper's ``resume(source)`` when it takes the source (it should then
clear only that stop), else ``resume()``.
"""
from __future__ import annotations

import importlib
import importlib.util
import inspect
import math
import sys
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol, runtime_checkable

from bounded_embodiment.commandspace import CommandSpace

__all__ = [
    "ADAPTER_METHODS",
    "ADAPTER_STOP_SOURCES",
    "TRIAL_METHODS",
    "Adapter",
    "AdapterContext",
    "AdapterError",
    "SendResult",
    "State",
    "call_resume",
    "check_adapter",
    "clean_text",
    "load_callable",
]

#: Every method the runner calls. ``check_adapter`` refuses an object that lacks one.
ADAPTER_METHODS = ("open", "describe", "send", "read_state", "latch_stop", "execute_stop", "close")
#: The optional EV-02 / EV-05 methods (module docstring).
TRIAL_METHODS = ("trigger_stop", "reset_stop")
#: The stop sources an adapter may issue itself; the others are outside the system's software.
ADAPTER_STOP_SOURCES = ("stop_message",)


def call_resume(target, source: str) -> None:
    """``target.resume(source)`` when its ``resume`` takes an argument, else ``target.resume()``
    (review N5): a gate that knows which stop to clear is told, and a gate written for the plain
    ``resume()`` keeps working unchanged."""
    fn = target.resume
    try:
        params = [p for p in inspect.signature(fn).parameters.values()
                  if p.kind in (p.POSITIONAL_ONLY, p.POSITIONAL_OR_KEYWORD, p.VAR_POSITIONAL)]
    except (TypeError, ValueError):
        params = []
    if params:
        fn(source)
    else:
        fn()


class AdapterError(RuntimeError):
    """The adapter refuses to go on: the message says why and what to do. Raised before motion
    (open) or when the robot cannot be left safe (close)."""


@dataclass(frozen=True)
class SendResult:
    """What happened to one command.

    applied     the adapter handed the command to the system under test and the call returned.
                It says nothing about whether the robot moved: a gate may hold or clamp.
    decision    the system under test's own word for what it did ("allow", "clamp", ...), if it
                reports one. Logged in commands.csv, never scored.
    error_kind  why the command was not applied: "stop_latched", "exception:<Type>", ...
    """

    applied: bool
    decision: str | None = None
    error_kind: str | None = None


@dataclass(frozen=True)
class State:
    """The robot's own joint state, as its encoders report it.

    joints  joint name -> value in the robot's own units (LeRobot: degrees for the body joints,
            0..100 for the gripper).
    units   joint name -> "deg" | "rad" | "percent" | "m" | "m/s" | "rad/s". Forward kinematics
            uses only deg and rad joints.

    The runner stamps the time; the adapter does not.
    """

    joints: Mapping[str, float]
    units: Mapping[str, str] = field(default_factory=dict)

    def finite(self) -> bool:
        return all(isinstance(v, (int, float)) and math.isfinite(v) for v in self.joints.values())


@runtime_checkable
class Adapter(Protocol):
    def open(self) -> CommandSpace: ...  # connect safely; the command space comes from a declared file
    def describe(self) -> dict: ...  # self-description; describe()["config"] goes into the setup hash
    def send(self, cmd: dict) -> SendResult: ...  # never raises
    def read_state(self) -> State | None: ...  # None: this adapter has no joint state (no FK tripwire)
    def latch_stop(self) -> None: ...  # sets a flag; any thread; never touches the bus
    def execute_stop(self) -> None: ...  # command thread only; same goals every time per latch
    def close(self) -> None: ...


def _stderr(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


@dataclass
class AdapterContext:
    """What the runner hands an adapter's ``from_args``: the run's files and the operator's I/O.

    ``prompt`` asks a typed question (``input`` on a terminal; tests pass their own) and ``log``
    prints a line for the operator."""

    command_space_path: Path
    envelope_path: Path | None = None
    kit_config_path: Path | None = None
    kit_config: dict = field(default_factory=dict)
    urdf_path: Path | None = None
    mode: str = ""
    speed_factor: float | None = None
    prompt: Callable[[str], str] = input
    log: Callable[[str], None] = _stderr

    def wrapper_context(self) -> dict:
        """What a lab's wrapper factory receives as ``context``: plain data, no callables.
        ``speed_factor`` is 0.5 in a shakedown ("at half speed") and None otherwise ("use your
        own default"); the kit itself never slows or alters a command."""

        def text(p: Path | None) -> str | None:
            return None if p is None else str(p)

        return {"mode": self.mode, "speed_factor": self.speed_factor,
                "envelope_path": text(self.envelope_path),
                "command_space_path": text(self.command_space_path),
                "kit_config_path": text(self.kit_config_path), "urdf_path": text(self.urdf_path)}


def check_adapter(obj: object) -> None:
    """Refuse an object that does not implement every method the runner calls."""
    missing = [m for m in ADAPTER_METHODS if not callable(getattr(obj, m, None))]
    if missing:
        raise AdapterError(f"{type(obj).__name__} is not an adapter: it lacks {', '.join(missing)}")


def clean_text(value: object, limit: int = 64) -> str | None:
    """A short single-line rendering of a value the system under test reported (for the CSV)."""
    if value is None:
        return None
    text = " ".join(str(value).split())
    return text[:limit] if text else None


def load_callable(spec: str) -> Callable:
    """``module:name`` or ``path/to/file.py:name`` -> the callable it names.

    Used for the lab's own code (a wrapper factory, a python adapter class). It is imported at run
    time from a name the operator typed, so the kit never imports a gate statically (import-linter
    checks that)."""
    target, sep, name = spec.rpartition(":")
    if not sep or not target or not name:
        raise AdapterError(f"{spec!r} is not module:name or path/to/file.py:name")
    if target.endswith(".py") or "/" in target or "\\" in target:
        path = Path(target).expanduser().resolve()
        if not path.is_file():
            raise AdapterError(f"{path} does not exist")
        mod_name = f"be_lab_{path.stem}"
        loader_spec = importlib.util.spec_from_file_location(mod_name, path)
        if loader_spec is None or loader_spec.loader is None:
            raise AdapterError(f"cannot import {path}")
        module = importlib.util.module_from_spec(loader_spec)
        sys.modules[mod_name] = module
        loader_spec.loader.exec_module(module)
    else:
        module = importlib.import_module(target)
    obj = getattr(module, name, None)
    if not callable(obj):
        raise AdapterError(f"{spec!r}: {name!r} is not a callable in {target!r}")
    return obj
