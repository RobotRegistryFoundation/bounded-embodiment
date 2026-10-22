# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""DRAFT — requires line-by-line human review before use on hardware.

``sim-rover``: a simulated differential-drive rover commanded with ``{"linear": m/s, "angular":
rad/s}`` (``examples/command-spaces/rover.command-space.json``). Port of ev03-harness
``ev03/adapters.py`` SimRover, with its constants made parameters.

The drive's own command interface (``SimRover.send_action``) does what UngatedRover did in the
harness: a command with both keys as finite numbers is clamped to the drive's top speeds and
applied; anything else raises ValueError and changes nothing. A lab's gate can wrap it with
``--wrapper module:factory`` exactly as for an arm: ``factory(rover, context)``.

The stop is zero velocity, the same every time. The rover then brakes at its acceleration limit,
so from top speed it needs top_speed**2 / (2 * accel) metres (1.1 m with the defaults) and more
than the watchdog's 0.25 s acknowledgement window: a stop at speed escalates, by design.

There are no kinematics to check from the drive's state, so a run on it has no FK tripwire.

EV-02 and EV-05. The drive has its own stop request (``SimRover.stop()``: zero velocity, then it
refuses commands until ``resume()``); ``trigger_stop("stop_message")`` uses the lab wrapper's
``stop()`` when it has one, and the drive's otherwise. The simulated external stop
(``--sim-estop-file PATH``, shared with sim-so101) cuts the motors while the file exists: the rover
then brakes at its acceleration limit and ignores commands until the file is gone and
``reset_stop`` has run. The bare drive has no heartbeat: it keeps its last velocity when commands
stop, which is what EV-05 is meant to catch.
"""
from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable, Mapping
from pathlib import Path

from bounded_embodiment.commandspace import CommandSpace, load_command_space

from .base import (
    AdapterError,
    SendResult,
    State,
    call_resume,
    clean_text,
    load_callable,
)

__all__ = ["SimRover", "SimRoverAdapter", "add_arguments", "from_args"]

PHYSICS_PERIOD_S = 0.005


def _number(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"not a finite number: {value!r}")
    return float(value)


class SimRover:
    """Unicycle rover. Ground truth is (x, y, theta) in the envelope frame, metres and radians."""

    def __init__(self, top_speed: float = 1.5, max_turn: float = 1.5, accel: float = 1.0,
                 start: tuple[float, float, float] = (1.0, 1.0, 0.0)):
        self.x, self.y, self.th = (float(v) for v in start)
        self.v = self.w = 0.0
        self.cmd_v = self.cmd_w = 0.0
        self.top, self.max_turn, self.accel = top_speed, max_turn, accel
        self.stopped = False  # the drive's own stop request is in force (stop() until resume())
        self.cut = False      # the simulated external stop: no motor power
        self.lock = threading.Lock()

    def actuate(self, linear_mps: float, angular_radps: float) -> None:
        """What reaches the motors, clamped to what the drive can do."""
        with self.lock:
            self.cmd_v = max(-self.top, min(self.top, linear_mps))
            self.cmd_w = max(-self.max_turn, min(self.max_turn, angular_radps))

    def send_action(self, cmd: Mapping) -> dict:
        """The drive's own command interface: both keys, finite numbers, or ValueError. While its
        stop is in force, or its power is cut, it refuses every command (RuntimeError)."""
        if not isinstance(cmd, Mapping) or "linear" not in cmd or "angular" not in cmd:
            raise ValueError(f"a drive command needs linear and angular: {cmd!r}")
        v, w = _number(cmd["linear"]), _number(cmd["angular"])
        with self.lock:  # the check and the command in one step: a stop on another thread wins
            if self.stopped or self.cut:
                raise RuntimeError("the drive refuses commands: "
                                   + ("stopped" if self.stopped else "no motor power"))
            self.cmd_v = max(-self.top, min(self.top, v))
            self.cmd_w = max(-self.max_turn, min(self.max_turn, w))
            return {"linear": self.cmd_v, "angular": self.cmd_w}

    def stop(self) -> None:
        """The drive's own stop request: zero velocity, and no command until resume()."""
        with self.lock:
            self.stopped = True
            self.cmd_v = self.cmd_w = 0.0

    def resume(self) -> None:
        """Accept commands again. Nothing moves until a command comes."""
        with self.lock:
            self.stopped = False

    def cut_power(self) -> None:
        with self.lock:
            self.cut = True
            self.cmd_v = self.cmd_w = 0.0

    def restore_power(self) -> None:
        with self.lock:
            self.cut = False

    def tick(self, dt: float) -> None:
        with self.lock:
            if self.cut:
                self.cmd_v = self.cmd_w = 0.0
            dv = max(-self.accel * dt, min(self.accel * dt, self.cmd_v - self.v))
            self.v += dv
            self.w = self.cmd_w
            self.th += self.w * dt
            self.x += self.v * math.cos(self.th) * dt
            self.y += self.v * math.sin(self.th) * dt

    def pose(self) -> tuple[float, float, float]:
        with self.lock:
            return self.x, self.y, self.th

    def velocities(self) -> tuple[float, float]:
        with self.lock:
            return self.v, self.w


class SimRoverAdapter:
    """The adapter contract for the simulated rover."""

    name = "sim-rover"
    simulation = True

    def __init__(self, *, command_space_path: str | Path, top_speed: float = 1.5,
                 max_turn: float = 1.5, accel: float = 1.0,
                 start: tuple[float, float, float] = (1.0, 1.0, 0.0), wrapper: str | None = None,
                 wrapper_context: dict | None = None, realtime: bool = True,
                 estop_file: str | Path | None = None,
                 log: Callable[[str], None] = print, prompt: Callable[[str], str] = input):
        self.command_space_path = Path(command_space_path)
        self.params = {"top_speed": top_speed, "max_turn": max_turn, "accel": accel,
                       "start": list(start)}
        self.wrapper, self.wrapper_context = wrapper, dict(wrapper_context or {})
        self.realtime, self.log, self.prompt = realtime, log, prompt
        self.estop_file = Path(estop_file) if estop_file else None
        self._bus_lock = threading.RLock()  # the drive's "bus": one lock, as for an arm
        self._latched = threading.Event()
        self._stop_goals: dict[str, float] | None = None
        self.rover: SimRover | None = None
        self._target = None
        self._physics: threading.Thread | None = None
        self._physics_quit = threading.Event()

    def open(self) -> CommandSpace:
        space = load_command_space(self.command_space_path)
        if space.kind != "velocity" or set(space.keys) != {"linear", "angular"}:
            raise AdapterError("sim-rover takes a velocity command space with axes linear and angular")
        factory = load_callable(self.wrapper) if self.wrapper else None
        p = self.params
        self.rover = SimRover(p["top_speed"], p["max_turn"], p["accel"], tuple(p["start"]))
        self._target = factory(self.rover, dict(self.wrapper_context)) if factory else self.rover
        if not callable(getattr(self._target, "send_action", None)):
            raise AdapterError(f"{self.wrapper} returned an object without send_action")
        self._stop_goals = None
        self._latched.clear()
        if self.realtime:
            self._physics_quit.clear()
            self._physics = threading.Thread(target=self._run_physics, name="be-sim-physics",
                                             daemon=True)
            self._physics.start()
        return space

    def _estop(self) -> None:
        """The simulated external stop: while the e-stop file exists, the motors have no power."""
        if self.estop_file is not None and not self.rover.cut and self.estop_file.exists():
            self.rover.cut_power()

    def _run_physics(self) -> None:
        last = time.perf_counter()
        while not self._physics_quit.wait(PHYSICS_PERIOD_S):
            now = time.perf_counter()
            self._estop()
            self.rover.tick(now - last)
            last = now

    def describe(self) -> dict:
        resets = ["stop_message", "heartbeat_loss", "comms_loss"]
        if self.estop_file is not None:
            resets += ["hardwired_stop", "power_loss"]
        return {"adapter": self.name, "simulation": True,
                "config": {"adapter": self.name, "wrapper": self.wrapper, "sim": dict(self.params)},
                "stop": {"rule": "zero velocity; the rover brakes at its acceleration limit"},
                "joint_speed_trip_per_s": {},
                "stop_triggers": ["stop_message"], "stop_resets": resets,
                "sim_estop_file": str(self.estop_file) if self.estop_file else None}

    def _stoppable(self):
        """Where a stop message goes: the lab wrapper when it has stop() and resume(), else the
        drive's own."""
        target = self._target
        if (target is not None and target is not self.rover
                and callable(getattr(type(target), "stop", None))
                and callable(getattr(type(target), "resume", None))):
            return target
        return self.rover

    def trigger_stop(self, source: str) -> None:
        """Command thread only (EV-02): the system's own stop request. Never the runner's stop."""
        if source != "stop_message":
            raise AdapterError(f"sim-rover issues stop_message only, not {source}")
        if self._latched.is_set():
            raise AdapterError("the runner's stop is latched")
        with self._bus_lock:
            if self._latched.is_set():  # again under the lock: the watchdog may have tripped meanwhile
                raise AdapterError("the runner's stop is latched")
            self._stoppable().stop()

    def reset_stop(self, source: str) -> None:
        """Command thread only, after the operator's NEXT. Nothing moves until a command comes."""
        if self._latched.is_set():
            raise AdapterError("the runner's stop is latched")
        with self._bus_lock:
            if self._latched.is_set():
                raise AdapterError("the runner's stop is latched")
            if source in ("hardwired_stop", "power_loss"):
                if self.estop_file is None:
                    raise AdapterError("sim-rover has no simulated external stop (--sim-estop-file)")
                if self.estop_file.exists():
                    raise AdapterError(f"the simulated e-stop is still open: {self.estop_file} exists")
                self.rover.restore_power()
            elif source in ("stop_message", "heartbeat_loss", "comms_loss"):
                target = self._stoppable()
                call_resume(target, source)
                if target is not self.rover:
                    self.rover.resume()
            else:
                raise AdapterError(f"sim-rover cannot reset {source}")

    def send(self, cmd: dict) -> SendResult:
        if self._latched.is_set():
            return SendResult(False, None, "stop_latched")
        try:
            with self._bus_lock:
                if self._latched.is_set():
                    return SendResult(False, None, "stop_latched")
                self._target.send_action(cmd)
        except Exception as e:  # noqa: BLE001 - send() never raises; the error is the result
            return SendResult(False, None, f"exception:{type(e).__name__}")
        decision = getattr(self._target, "last_decision", None) if self._target is not self.rover else None
        return SendResult(True, clean_text(decision), None)

    def read_state(self) -> State | None:
        with self._bus_lock:
            v, w = self.rover.velocities()
        return State(joints={"linear": v, "angular": w}, units={"linear": "m/s", "angular": "rad/s"})

    def latch_stop(self) -> None:
        self._latched.set()

    def execute_stop(self) -> None:
        self._latched.set()
        with self._bus_lock:
            if self._stop_goals is None:
                self._stop_goals = {"linear": 0.0, "angular": 0.0}
            self.rover.actuate(self._stop_goals["linear"], self._stop_goals["angular"])

    def close(self) -> None:
        self._physics_quit.set()
        if self._physics is not None:
            self._physics.join(timeout=2.0)
            self._physics = None

    def step(self, dt: float, substep: float = 0.001) -> None:
        n = max(1, round(dt / substep))
        for _ in range(n):
            self._estop()
            self.rover.tick(dt / n)

    def truth_fn(self, fk=None, envelope=None) -> Callable[[], dict]:
        """For the simulated instrument: every envelope point at the rover's position, z = 0."""
        points = list(getattr(envelope, "points", {}) or {"base": None})

        def truth() -> dict:
            x, y, _ = self.rover.pose()
            return {pid: (x, y, 0.0) for pid in points}

        return truth


def add_arguments(p) -> None:
    g = p.add_argument_group("sim-rover adapter")
    g.add_argument("--sim-rover-start", default="1.0,1.0,0.0", metavar="X,Y,THETA",
                   help="start pose in the envelope frame (m, m, rad)")


def from_args(args, context) -> SimRoverAdapter:
    try:
        x, y, th = (float(v) for v in args.sim_rover_start.split(","))
    except ValueError:
        raise AdapterError(f"--sim-rover-start {args.sim_rover_start!r} is not X,Y,THETA") from None
    return SimRoverAdapter(command_space_path=context.command_space_path, start=(x, y, th),
                           wrapper=getattr(args, "wrapper", None),
                           wrapper_context=context.wrapper_context(),
                           estop_file=getattr(args, "sim_estop_file", None), log=context.log,
                           prompt=context.prompt)
