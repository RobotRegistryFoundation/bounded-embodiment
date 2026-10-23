# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""DRAFT — requires line-by-line human review before use on hardware.

The ``lerobot`` adapter: a LeRobot SO-100 / SO-101 follower arm on a Feetech bus.

How a lab's gate plugs in. The adapter builds LeRobot's follower and connects its bus ITSELF with
``feetech.connect_safely`` (goals synced to present before any torque). Only then does it hand the
already-connected robot to the lab's ``--wrapper module:factory``, called as
``factory(robot, context)``; the factory returns the gated wrapper (anything with
``send_action(action) -> dict``). From then on:

- every command goes through ``wrapper.send_action`` (the system under test);
- the adapter NEVER calls the wrapper's ``connect()``, ``configure()`` or ``disconnect()``, nor the
  inner robot's ``connect()`` or ``disconnect()``: LeRobot's ``connect()`` enables torque toward
  goal registers that read 0 at power-up, and its ``disconnect()`` drops torque by default
  (``SOFollowerConfig.disable_torque_on_disconnect=True``), which drops the arm;
- stops, state reads and the final release go straight to the inner robot's bus, under the bus
  lock, so a gate that misbehaves cannot block or alter a stop (it can only delay it by holding
  the bus inside send_action; the watchdog's acknowledgement timer covers that case).

``context`` is plain data: ``mode``, ``speed_factor`` (0.5 in a shakedown, else None: "use your own
default"), and the paths of the envelope, command space, kit config and URDF.

EV-02 and EV-05. ``trigger_stop("stop_message")`` calls the lab wrapper's own ``stop()`` and
``reset_stop`` its ``resume()``, under the bus lock; without a wrapper that has both, the adapter
declares no stop source (``describe()["stop_triggers"]``) and ``be run ev02`` refuses a plan with a
stop message. A hardwired stop or a loss of power is never reset here: an SO-101 whose servo power
was cut comes back with torque off and its goal registers at power-up values, and only
``connect_safely`` brings it back without a jump, which this adapter does not run mid-session. So
``be run ev02`` refuses a plan with those sources on this adapter (the lab writes a ``python``
adapter that can).

Other LeRobot robots (Dynamixel, Damiao, Robstride, Reachy 2 buses) are refused in v0.1: their
connect path and their stop differ. A lab with one writes a ``python`` adapter with its own connect
path, and runs with ``--estop-cmd``.

LeRobot is imported only when a real robot is built (``build_follower``), never at import time.
"""
from __future__ import annotations

import importlib
import sys
import threading
from collections.abc import Callable
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
from .feetech import (
    BODY_JOINT_SPEED_TRIP_DPS,
    check_bus_is_ours,
    choose_stop_goals,
    connect_safely,
    leave_safe,
    norm_name,
)

__all__ = ["SUPPORTED_ROBOT_TYPES", "LeRobotAdapter", "add_arguments", "build_follower", "from_args",
           "lerobot_version"]

SUPPORTED_ROBOT_TYPES = ("so101_follower", "so100_follower")

# LeRobot has moved the follower classes between releases (ev03-harness lerobot_compat.py).
_FOLLOWER_PATHS = [
    ("lerobot.robots.so_follower", "SO101Follower", "SO101FollowerConfig"),  # 0.6.x, main
    ("lerobot.robots.so101_follower", "SO101Follower", "SO101FollowerConfig"),  # 0.2 - 0.4
    ("lerobot.common.robots.so101_follower", "SO101Follower", "SO101FollowerConfig"),
]

#: How long close() waits for the bus lock before giving up (the command thread may be stuck
#: inside a gate's send_action, holding the bus).
CLOSE_LOCK_TIMEOUT_S = 5.0


def _log(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


def lerobot_version() -> str:
    try:
        from importlib.metadata import version

        return version("lerobot")
    except Exception:  # noqa: BLE001 - not installed, or broken metadata: say so, never fail
        return "not installed"


def build_follower(robot_type: str, port: str, robot_id: str,
                   max_relative_target: float | None = None) -> object:
    """Build (never connect) LeRobot's SO-10x follower, exactly as LeRobot's own scripts would."""
    if robot_type not in SUPPORTED_ROBOT_TYPES:
        raise AdapterError(f"robot type {robot_type!r} is refused in v0.1: only Feetech SO-10x "
                           f"followers {SUPPORTED_ROBOT_TYPES} have a reviewed connect path and stop. "
                           "Write a python adapter (be adapter new) with your own connect path, and "
                           "run with --estop-cmd.")
    errors = []
    for module, robot_cls, config_cls in _FOLLOWER_PATHS:
        try:
            m = importlib.import_module(module)
            robot_class, config_class = getattr(m, robot_cls), getattr(m, config_cls)
            break
        except (ImportError, AttributeError) as e:
            errors.append(f"{module}: {e}")
    else:
        raise AdapterError("LeRobot's SO-101 follower was not found (pip install "
                           "'bounded-embodiment-kit[lerobot]', Python >= 3.12). Tried:\n  "
                           + "\n  ".join(errors))
    kwargs = {"port": port, "id": robot_id, "max_relative_target": max_relative_target}
    try:
        cfg = config_class(use_degrees=True, **kwargs)
    except TypeError:  # older LeRobot had no use_degrees option (connect_safely then refuses)
        cfg = config_class(**kwargs)
    return robot_class(cfg)


class LeRobotAdapter:
    """The adapter contract (``adapters.base``) for a LeRobot SO-10x follower."""

    name = "lerobot"
    simulation = False

    def __init__(self, *, command_space_path: str | Path, port: str, robot_id: str,
                 robot_type: str = "so101_follower", wrapper: str | None = None,
                 wrapper_context: dict | None = None, max_relative_target: float | None = None,
                 refuse_while_active: tuple[str, ...] = (),
                 prompt: Callable[[str], str] = input, log: Callable[[str], None] = _log,
                 make_robot: Callable[[], object] | None = None):
        self.command_space_path = Path(command_space_path)
        self.port, self.robot_id, self.robot_type = port, robot_id, robot_type
        self.wrapper = wrapper
        self.wrapper_context = dict(wrapper_context or {})
        self.max_relative_target = max_relative_target
        self.refuse_while_active = tuple(refuse_while_active)
        self.prompt, self.log = prompt, log
        self._make_robot = make_robot or (lambda: build_follower(robot_type, port, robot_id,
                                                                 max_relative_target))
        # One bus lock. Every bus access takes it: send, read_state, execute_stop, close. The
        # watchdog never takes it; latch_stop() only sets an Event.
        self._bus_lock = threading.RLock()
        self._latched = threading.Event()
        self._stop_goals: dict[str, int] | None = None
        self._robot = None
        self._target = None
        self._connect_record: dict | None = None

    # ----- open --------------------------------------------------------------------------------
    def open(self) -> CommandSpace:
        # The command space comes from the lab's declared file (URDF limits), never from bus
        # calibration: bob's elbow and wrist are calibrated to the full encoder range.
        space = load_command_space(self.command_space_path)
        if space.kind != "joint_position":
            raise AdapterError(f"the lerobot adapter drives joint positions; {self.command_space_path} "
                               f"declares {space.kind!r}")
        # Import the lab's factory BEFORE any torque, so an import error cannot leave a powered arm.
        factory = load_callable(self.wrapper) if self.wrapper else None
        self._preflight()
        robot = self._make_robot()
        self._check_robot(robot, space)
        with self._bus_lock:
            self._connect_record = connect_safely(robot, log=self.log)
        self._robot = robot
        try:
            target = robot
            if factory is not None:
                target = factory(robot, dict(self.wrapper_context))
                if not callable(getattr(target, "send_action", None)):
                    raise AdapterError(f"{self.wrapper} returned {type(target).__name__}, which has "
                                       "no send_action(action)")
        except BaseException:
            self._release_without_prompt()
            raise
        self._target = target
        self._stop_goals = None
        self._latched.clear()  # a new latch for this connection
        return space

    def _preflight(self) -> None:
        """Refuse if anything else could command the arm (hw.py check_bus_is_ours)."""
        problems = check_bus_is_ours(self.port, self.refuse_while_active)
        if problems:
            raise AdapterError("Something else could command the arm:\n  " + "\n  ".join(problems))

    def _check_robot(self, robot: object, space: CommandSpace) -> None:
        bus = getattr(robot, "bus", None)
        motors = getattr(bus, "motors", None)
        if not motors:
            raise AdapterError(f"{type(robot).__name__} has no bus with motors")
        unknown = [a.key for a in space.axes
                   if not (a.key.endswith(".pos") and a.key[: -len(".pos")] in motors)]
        if unknown:
            raise AdapterError(f"command-space axes {unknown} are not '<motor>.pos' for this "
                               f"robot's motors {list(motors)}")

    def _release_without_prompt(self) -> None:
        robot, self._robot, self._target = self._robot, None, None
        if robot is None:
            return
        try:
            with self._bus_lock:
                if getattr(robot.bus, "is_connected", False):
                    robot.bus.disconnect(disable_torque=False)  # torque untouched: the arm holds
        except Exception as e:  # noqa: BLE001 - already failing; report and keep the first error
            self.log(f"Could not release the port ({type(e).__name__}: {e}).")

    # ----- describe ----------------------------------------------------------------------------
    def describe(self) -> dict:
        motors = list(getattr(getattr(self._robot, "bus", None), "motors", {}) or {})
        body = [m for m in motors if m != "gripper"]
        return {
            "adapter": self.name,
            "simulation": self.simulation,
            # describe()["config"] is what the setup hash covers: everything that changes what is
            # being tested or how it is driven, and nothing that changes from run to run.
            "config": {
                "adapter": self.name,
                "robot_type": self.robot_type,
                "port": self.port,
                "robot_id": self.robot_id,
                "wrapper": self.wrapper,
                "max_relative_target": self.max_relative_target,
            },
            "lerobot_version": lerobot_version(),
            "connect": self._connect_record,
            "stop": {"rule": "every joint's present position, read at the first stop of a latch "
                             "and re-sent unchanged by every later stop; raw ticks written to the "
                             "inner bus, not through the wrapper"},
            "joint_speed_trip_per_s": {m: BODY_JOINT_SPEED_TRIP_DPS for m in body},
            "refuse_while_active": list(self.refuse_while_active),
            "stop_triggers": ["stop_message"] if self._wrapper_has("stop", "resume") else [],
            "stop_resets": (["stop_message", "heartbeat_loss", "comms_loss"]
                            if self._wrapper_has("resume") else []),
        }

    def _wrapper_has(self, *names: str) -> bool:
        """Whether the lab's wrapper (never the bare robot) defines these methods on its class: a
        wrapper that forwards unknown attributes to the robot must not lend it the robot's."""
        target = self._target
        return (target is not None and target is not self._robot
                and all(callable(getattr(type(target), n, None)) for n in names))

    # ----- commands ----------------------------------------------------------------------------
    def send(self, cmd: dict) -> SendResult:
        """Hand one proposal to the system under test. Never raises: the hostile stream sends NaN,
        strings and missing keys on purpose, and LeRobot or a gate raising on them is a result."""
        if self._latched.is_set():
            return SendResult(False, None, "stop_latched")
        target = self._target
        if target is None:
            return SendResult(False, None, "not_open")
        try:
            with self._bus_lock:
                if self._latched.is_set():  # re-checked under the lock: the stop wins any race
                    return SendResult(False, None, "stop_latched")
                target.send_action(cmd)
        except Exception as e:  # noqa: BLE001 - send() never raises; the error is the result
            return SendResult(False, None, f"exception:{type(e).__name__}")
        decision = getattr(target, "last_decision", None) if target is not self._robot else None
        return SendResult(True, clean_text(decision), None)

    def read_state(self) -> State | None:
        robot = self._robot
        if robot is None:
            return None
        with self._bus_lock:
            values = robot.bus.sync_read("Present_Position")
        units = {m: ("deg" if norm_name(motor) == "DEGREES" else "percent")
                 for m, motor in robot.bus.motors.items()}
        return State(joints={m: float(v) for m, v in values.items()}, units=units)

    # ----- the stop ----------------------------------------------------------------------------
    def latch_stop(self) -> None:
        """Any thread. Sets the latch and returns at once: no bus access, no lock."""
        self._latched.set()

    def execute_stop(self) -> None:
        """Command thread only. The stop: the present positions read at the first stop of this
        latch (``feetech.choose_stop_goals``), written straight to the inner bus in raw ticks (no
        wrapper, no LeRobot clamp, no degree rounding). Every later call re-sends exactly those."""
        self._latched.set()
        robot = self._robot
        if robot is None:
            raise AdapterError("execute_stop: the robot is not open")
        with self._bus_lock:
            bus = robot.bus
            if self._stop_goals is None:
                present = bus.sync_read("Present_Position", normalize=False)
                self._stop_goals = choose_stop_goals(present)
                self.log("Stop: every joint held where it was; later stops re-send these goals.")
            bus.sync_write("Goal_Position", dict(self._stop_goals), normalize=False)

    # ----- EV-02 / EV-05 (adapters.base: optional) ----------------------------------------------
    def trigger_stop(self, source: str) -> None:
        """Command thread only. The system under test's own stop: the lab wrapper's ``stop()``,
        under the bus lock. Never the runner's stop (nothing is latched here)."""
        if source != "stop_message" or not self._wrapper_has("stop", "resume"):
            raise AdapterError(f"this adapter cannot issue {source}: it needs a --wrapper with "
                               "stop() and resume()")
        if self._latched.is_set():
            raise AdapterError("the runner's stop is latched")
        with self._bus_lock:
            if self._latched.is_set():  # again under the lock: the watchdog may have tripped meanwhile
                raise AdapterError("the runner's stop is latched")
            self._target.stop()

    def reset_stop(self, source: str) -> None:
        """Command thread only, after the operator's NEXT: the lab wrapper's ``resume(source=...)``
        (or ``resume()``, ``base.call_resume``), which must not move the arm. Hardwired stops and
        power loss are refused (module docstring)."""
        if source not in ("stop_message", "heartbeat_loss", "comms_loss") or not self._wrapper_has("resume"):
            raise AdapterError(f"this adapter cannot reset {source}")
        if self._latched.is_set():
            raise AdapterError("the runner's stop is latched")
        with self._bus_lock:
            if self._latched.is_set():
                raise AdapterError("the runner's stop is latched")
            call_resume(self._target, source)

    # ----- close -------------------------------------------------------------------------------
    def close(self) -> None:
        """Leave the arm holding (torque on) unless the operator types OFF; release the port. Never
        through LeRobot's robot.disconnect() or the wrapper's."""
        robot = self._robot
        if robot is None:
            return
        if not self._bus_lock.acquire(timeout=CLOSE_LOCK_TIMEOUT_S):
            self.log("*** The bus is still busy (a command is stuck?). Not touching it. "
                     "CUT SERVO POWER if the arm is not holding still.")
            return
        try:
            leave_safe(robot.bus, self.prompt, self.log)
        finally:
            self._bus_lock.release()
            self._robot, self._target = None, None


# ----- command line ----------------------------------------------------------------------------
def add_arguments(p) -> None:
    g = p.add_argument_group("lerobot adapter")
    g.add_argument("--port", help="the follower's serial port (lerobot)")
    g.add_argument("--id", dest="robot_id", help="the LeRobot robot id (its calibration file)")
    g.add_argument("--robot-type", default="so101_follower", choices=SUPPORTED_ROBOT_TYPES)
    g.add_argument("--wrapper", metavar="MODULE:FACTORY",
                   help="the lab's gate: factory(robot, context) -> wrapper with send_action "
                        "(module:name or path/to/file.py:name); omit for the bare robot")
    g.add_argument("--max-relative-target", type=float, default=None,
                   help="LeRobot's own per-command step clamp (part of the system under test)")
    g.add_argument("--refuse-while-active", action="append", default=[], metavar="UNIT",
                   help="a systemd unit that can also drive the arm; refuse while it is active")


def from_args(args, context) -> LeRobotAdapter:
    if not args.port or not args.robot_id:
        raise AdapterError("the lerobot adapter needs --port and --id")
    return LeRobotAdapter(command_space_path=context.command_space_path, port=args.port,
                          robot_id=args.robot_id, robot_type=args.robot_type,
                          wrapper=getattr(args, "wrapper", None),
                          wrapper_context=context.wrapper_context(),
                          max_relative_target=args.max_relative_target,
                          refuse_while_active=tuple(args.refuse_while_active),
                          prompt=context.prompt, log=context.log)
