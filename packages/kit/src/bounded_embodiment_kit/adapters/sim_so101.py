# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""DRAFT — requires line-by-line human review before use on hardware.

``sim-so101``: the real ``lerobot`` adapter over a simulated SO-101. Port of ev03-harness
``ev03/arm/sim.py``, with its constants made parameters.

sim.py built LeRobot's genuine SO101Follower and replaced only its serial layer. LeRobot is not a
dependency of the kit (and is not installed in CI), so here the follower and its bus are small
stand-ins that implement exactly the LeRobot calls the kit makes, with LeRobot's arithmetic
(degrees <-> ticks with ``int()`` truncation, the gripper's 0..100 range, 0.6.2's value checks)
and the servos' firmware position limits. The adapter on top is the ``lerobot`` adapter class
itself, so connect_safely, the stop, read_state and close run exactly the code a hardware run does.

Below the bus, six simulated STS3215 servos (``ServoBank``): 12-bit positions, a slew limit
(default 270 deg/s), an optional first-order response (``tau_s``) and optional gravity sag with
friction (``GravitySag``, fitted to bob; see its docstring). Added for the kit: per-servo torque,
and bob's power-up state (torque off, goal registers 0), so a connect that forgot to sync goals
would show up as a jump.

Not modelled: what an unpowered joint does (with torque off a servo here simply stays put; a real
raised arm falls), bus errors, packet loss, and momentum carrying a joint into its friction band.

EV-02's external stop, simulated (``--sim-estop-file PATH``): while the file exists the servos are
unpowered (torque off, torque writes ignored), as if the lab's relay had cut their power; the
lab's ``--power-loss-cmd`` (or ``--hardwired-stop-cmd``) is then a command that creates the file
(``touch PATH``), and the operator "resets" the stop by deleting it. ``reset_stop`` then powers
the servos back the way
``connect_safely`` connects them: goals set to the present positions first, torque after. An
unpowered servo here stays put, so the simulated external stop is instantaneous: it shows that
the runner and the scorer work, not how a real arm stops.
"""
from __future__ import annotations

import json
import math
import threading
import time
from collections.abc import Callable, Mapping
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType, SimpleNamespace

import numpy as np

from bounded_embodiment_kit.kinematics import JointMap, URDFModel

from .base import AdapterError
from .lerobot import LeRobotAdapter

__all__ = [
    "ARM_JOINTS",
    "MOTORS",
    "SIM_REST_POSE",
    "GravitySag",
    "MotorCal",
    "ServoBank",
    "SimFeetechBus",
    "SimSO101Adapter",
    "SimSO101Follower",
    "ToolHeight",
    "add_arguments",
    "from_args",
    "sim_calibration",
]

TICKS = 4096
MAX_RES = TICKS - 1
MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
ARM_JOINTS = MOTORS[:5]

# Symmetric calibrated ranges (degrees either side of the middle), so LeRobot's 0 deg = URDF 0 rad.
SIM_RANGE_DEG = {"shoulder_pan": 110.0, "shoulder_lift": 100.0, "elbow_flex": 96.8,
                 "wrist_flex": 95.0, "wrist_roll": 157.2}
GRIPPER_RANGE_TICKS = (2048 - 200, 2048 + 1250)  # 0..100 maps onto roughly -18..+110 deg
SAG_REFRESH_S = 0.01  # how often the (pose-dependent) sag is recomputed
#: A folded resting pose (hw.py's SIM_REST_POSE): the arm starts outside a tabletop envelope.
SIM_REST_POSE = {"shoulder_pan": 0.0, "shoulder_lift": -95.0, "elbow_flex": 92.0, "wrist_flex": 65.0,
                 "wrist_roll": 0.0, "gripper": 10.0}

# Registers LeRobot normalises (degrees / 0..100) when normalize=True.
_NORMALISED = ("Goal_Position", "Present_Position")
_WRITES = ("sync_write", "write", "enable_torque", "disable_torque", "configure_motors")


@dataclass(frozen=True)
class MotorCal:
    """LeRobot's MotorCalibration, as far as the simulator needs it."""

    id: int
    drive_mode: int
    homing_offset: int
    range_min: int
    range_max: int


def sim_calibration(path: str | Path | None = None) -> dict[str, MotorCal]:
    """Symmetric ranges (default), or a real arm's LeRobot calibration file (to rehearse that arm,
    e.g. ev03-harness examples/bob_calibration.json). Keys starting with "_" are comments."""
    if path:
        d = json.loads(Path(path).read_text(encoding="utf-8"))
        return {m: MotorCal(**{k: v for k, v in d[m].items() if not k.startswith("_")})
                for m in MOTORS}
    cal = {}
    for i, m in enumerate(MOTORS, start=1):
        if m == "gripper":
            lo, hi = GRIPPER_RANGE_TICKS
        else:
            half = round(SIM_RANGE_DEG[m] * MAX_RES / 360.0)
            lo, hi = 2048 - half, 2048 + half
        cal[m] = MotorCal(id=i, drive_mode=0, homing_offset=0, range_min=lo, range_max=hi)
    return cal


def _mid(c: MotorCal) -> float:
    return (c.range_min + c.range_max) / 2


class ToolHeight:
    """Tool-point height (metres, URDF root frame) for configurations in LeRobot degrees, through
    the arm's joint map. Batched: GravitySag needs six configurations every 10 ms."""

    def __init__(self, model: URDFModel, joint_map: JointMap | None = None,
                 link: str = "gripper_frame_link", offset_m=None):
        self.model, self.joint_map = model, joint_map or JointMap()
        self.link = link
        self.offset = np.zeros(3) if offset_m is None else np.asarray(offset_m, dtype=float)

    def __call__(self, configs: list[Mapping[str, float]]) -> np.ndarray:
        q = {j: [self.joint_map.to_urdf({j: c[j]})[j] for c in configs] for j in ARM_JOINTS}
        T = self.model.fk_batch(q)[self.link]
        p = T[:, :3, :3] @ self.offset + T[:, :3, 3]
        return p[:, 2]


class GravitySag:
    """Where each joint settles relative to its goal under gravity, in encoder ticks, by pose.

    A position servo is a spring around its goal, with gearbox friction. The gravity torque a joint
    carries is taken as proportional to how fast the tool point drops when that joint moves
    (-dz/dq, virtual work with the arm's weight lumped at the tool), so it settles past its goal by
    ticks_per_m[j] * (-dz/dq_j), give or take friction_ticks[j]: an arm lifted into place stops
    further from its goal than one that sank into place.

    The BOB_* values were fitted to Craig's SO-ARM101 "bob" (12 V STS3215, LeRobot's P gain of 16)
    at its start pose, from three observations on 2026-10-08 (ev03-harness sim.py has the details):
    lifted into place, the shoulder stopped 27 ticks past its goal, the elbow 19 and the wrist 12;
    with a 2.5-tick step limit the joints held still; with a 1 deg (11-tick) step limit the elbow
    and wrist did not move at all. One pose, one arm: an illustration, not a calibrated model.
    """

    BOB_TICKS_PER_M = MappingProxyType({"shoulder_lift": 66.8, "elbow_flex": 60.0, "wrist_flex": 40.9})
    BOB_FRICTION_TICKS = MappingProxyType({"shoulder_pan": 2.0, "shoulder_lift": 12.5,
                                           "elbow_flex": 8.5, "wrist_flex": 5.5, "wrist_roll": 2.0})

    def __init__(self, tool_height: Callable[[list], np.ndarray], ticks_per_m=None,
                 friction_ticks=None, scale: float = 1.0, friction_scale: float = 1.0):
        self.tool_height = tool_height
        self.k = {j: v * scale for j, v in (ticks_per_m or self.BOB_TICKS_PER_M).items()}
        self.friction_ticks = {j: v * friction_scale
                               for j, v in (friction_ticks or self.BOB_FRICTION_TICKS).items()}

    def __call__(self, deg: Mapping[str, float]) -> dict[str, float]:
        q = {j: deg[j] for j in ARM_JOINTS}
        h = 0.5
        z = self.tool_height([q] + [dict(q, **{j: q[j] + h}) for j in ARM_JOINTS])
        dz = (z[1:] - z[0]) / math.radians(h)  # metres of tool height per radian
        return {j: -self.k.get(j, 0.0) * float(dz[i]) for i, j in enumerate(ARM_JOINTS)}


class ServoBank:
    """Six simulated STS3215 servos. Thread-safe: the physics thread steps it while the command
    thread reads and writes registers."""

    def __init__(self, calibration: Mapping[str, MotorCal], slew_deg_s: float = 270.0,
                 tau_s: float = 0.0, sag: GravitySag | None = None):
        self.cal = dict(calibration)
        self.ids = {c.id: m for m, c in self.cal.items()}
        self.present = {m: _mid(c) for m, c in self.cal.items()}
        self.goal = dict(self.present)
        self.torque = dict.fromkeys(self.cal, 1)
        self.slew = slew_deg_s * TICKS / 360.0  # ticks per second
        self.tau = tau_s
        self.sag = sag
        self.friction = sag.friction_ticks if sag is not None else {}
        self._sag: dict[str, float] = {}
        self._sag_age = math.inf
        self.powered = True  # False while a simulated external stop has cut the servos' power
        self.lock = threading.Lock()

    def cut_power(self) -> None:
        """The simulated external stop: every servo unpowered (torque off) until restore_power()."""
        with self.lock:
            self.powered = False
            for m in self.torque:
                self.torque[m] = 0

    def restore_power(self) -> None:
        """Power back, torque still off (as a real servo comes back)."""
        with self.lock:
            self.powered = True

    def set_goal_ticks(self, motor: str, ticks: float) -> None:
        c = self.cal[motor]
        with self.lock:
            self.goal[motor] = min(c.range_max, max(c.range_min, ticks))  # firmware position limit

    def step(self, dt: float) -> None:
        with self.lock:
            if self.sag is not None:
                self._sag_age += dt
                if self._sag_age >= SAG_REFRESH_S:
                    self._sag, self._sag_age = self.sag(self._degrees()), 0.0
            lim = self.slew * dt
            k = 1.0 if self.tau <= 0 else -math.expm1(-dt / self.tau)  # first-order response
            for m in self.present:
                if not self.torque[m]:
                    continue  # unpowered: stays put (a real raised joint would fall; not modelled)
                e, f, p = self.goal[m] + self._sag.get(m, 0.0), self.friction.get(m, 0.0), self.present[m]
                if p < e - f:  # friction: the joint stops at the near edge of the band
                    target = e - f
                elif p > e + f:
                    target = e + f
                else:
                    continue
                self.present[m] = p + max(-lim, min(lim, (target - p) * k))

    def _degrees(self) -> dict[str, float]:
        return {m: (self.present[m] - _mid(c)) * 360.0 / MAX_RES for m, c in self.cal.items()}

    def degrees(self) -> dict[str, float]:
        """Joint angles in LeRobot degrees (DEGREES norm mode: (ticks - mid) * 360 / 4095),
        from the true positions: the simulation's ground truth."""
        with self.lock:
            return self._degrees()


@dataclass(frozen=True)
class BusCall:
    """One call into the simulated bus, for tests: which thread, which method, which register."""

    thread: str
    method: str
    register: str | None
    values: object = None

    @property
    def is_write(self) -> bool:
        return self.method in _WRITES or (self.method == "disconnect" and bool(self.values))


class SimFeetechBus:
    """Stands in for LeRobot's FeetechMotorsBus over a ServoBank: exactly the calls the kit makes.

    With ``record=True`` every call is appended to ``calls`` with the calling thread's name: the
    watchdog test uses it to prove the watchdog never touches the bus, the doctor test to prove
    ``be doctor`` writes nothing."""

    def __init__(self, servos: ServoBank, calibration: Mapping[str, MotorCal],
                 stored_calibration: Mapping[str, MotorCal] | None = None, record: bool = False):
        self.model_resolution_table = {"sts3215": TICKS}
        self.servos = servos
        self.calibration = dict(calibration)
        self._stored = dict(stored_calibration or calibration)  # what the servos' EEPROM holds
        self.motors = {m: SimpleNamespace(id=c.id, model="sts3215",
                                          norm_mode=SimpleNamespace(name="RANGE_0_100" if m == "gripper"
                                                                    else "DEGREES"))
                       for m, c in self.calibration.items()}
        self._connected = False
        self.record = record
        self.calls: list[BusCall] = []

    # ----- bookkeeping -------------------------------------------------------------------------
    def _note(self, method: str, register: str | None = None, values: object = None) -> None:
        if self.record:
            self.calls.append(BusCall(threading.current_thread().name, method, register, values))

    def _need_connection(self) -> None:
        if not self._connected:
            raise ConnectionError("the simulated bus is not connected")

    def _names(self, motors) -> list[str]:
        if motors is None:
            return list(self.motors)
        return [motors] if isinstance(motors, str) else list(motors)

    # ----- LeRobot's normalisation -------------------------------------------------------------
    def _check_value(self, register: str, motor: str, value: object, normalize: bool) -> None:
        """LeRobot 0.6.2's _validate_value: refuse what int() would mangle."""
        normalised = normalize and register in _NORMALISED
        if value is None or isinstance(value, (str, bytes)):
            raise TypeError(f"Value for '{register}' on motor '{motor}' must be a number, got {value!r}.")
        if normalised and isinstance(value, bool):
            raise TypeError(f"Value for '{register}' on motor '{motor}' is normalised and must be a "
                            f"number, got the boolean {value!r}.")
        try:
            as_float = float(value)
        except (TypeError, ValueError):
            raise TypeError(f"Value for '{register}' on motor '{motor}' must be a number, "
                            f"got {value!r}.") from None
        if not math.isfinite(as_float):
            raise ValueError(f"Value for '{register}' on motor '{motor}' must be finite, got {value!r}.")

    def _to_user(self, motor: str, ticks: int) -> float:
        c = self.calibration[motor]
        if self.motors[motor].norm_mode.name == "DEGREES":
            return (ticks - _mid(c)) * 360 / MAX_RES
        bounded = min(c.range_max, max(c.range_min, ticks))
        return (bounded - c.range_min) / (c.range_max - c.range_min) * 100

    def _to_ticks(self, motor: str, value: float) -> int:
        c = self.calibration[motor]
        if self.motors[motor].norm_mode.name == "DEGREES":
            return int((value * MAX_RES / 360) + _mid(c))  # LeRobot truncates with int()
        bounded = min(100.0, max(0.0, value))
        return int((bounded / 100) * (c.range_max - c.range_min) + c.range_min)

    def _raw(self, register: str, motor: str) -> int:
        s = self.servos
        with s.lock:
            if register == "Present_Position":
                return round(s.present[motor])
            if register == "Goal_Position":
                return int(s.goal[motor])
            if register == "Torque_Enable":
                return int(s.torque[motor])
        if register == "Present_Voltage":
            return 120  # 12.0 V
        if register == "Present_Temperature":
            return 30
        raise NotImplementedError(f"the simulated bus does not model reading {register}")

    def _apply(self, register: str, motor: str, raw: int) -> None:
        s = self.servos
        if register == "Goal_Position":
            s.set_goal_ticks(motor, raw)
        elif register == "Torque_Enable":
            with s.lock:
                s.torque[motor] = 1 if raw and s.powered else 0  # an unpowered servo stays off
        # Other registers (Lock, gains, acceleration, ...) are accepted and have no effect here.

    # ----- the LeRobot bus API the kit uses ----------------------------------------------------
    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self, handshake: bool = True) -> None:
        self._note("connect")
        if self._connected:
            raise ConnectionError("already connected")
        self._connected = True

    def disconnect(self, disable_torque: bool = True) -> None:
        self._note("disconnect", None, disable_torque)
        self._need_connection()
        if disable_torque:
            for m in self.motors:
                self._apply("Torque_Enable", m, 0)
        self._connected = False

    def read_calibration(self) -> dict[str, MotorCal]:
        self._note("read_calibration")
        self._need_connection()
        return dict(self._stored)

    @property
    def is_calibrated(self) -> bool:
        stored = self.read_calibration()
        return all(stored[m] == c for m, c in self.calibration.items())

    def sync_read(self, data_name: str, motors=None, *, normalize: bool = True,
                  num_retry: int = 0) -> dict:
        self._note("sync_read", data_name)
        self._need_connection()
        out = {}
        for m in self._names(motors):
            raw = self._raw(data_name, m)
            out[m] = self._to_user(m, raw) if normalize and data_name in _NORMALISED else raw
        return out

    def read(self, data_name: str, motor: str, *, normalize: bool = True, num_retry: int = 0):
        return self.sync_read(data_name, [motor], normalize=normalize)[motor]

    def sync_write(self, data_name: str, values, *, normalize: bool = True,
                   num_retry: int = 0) -> None:
        self._note("sync_write", data_name, dict(values) if isinstance(values, Mapping) else values)
        self._need_connection()
        if not isinstance(values, Mapping):
            values = dict.fromkeys(self.motors, values)
        if not values:
            raise ValueError(f"No values given to sync write '{data_name}'.")
        for m, v in values.items():
            self._check_value(data_name, m, v, normalize)
        raws = {m: (self._to_ticks(m, v) if normalize and data_name in _NORMALISED else int(v))
                for m, v in values.items()}
        for m, raw in raws.items():
            self._apply(data_name, m, raw)

    def write(self, data_name: str, motor: str, value, *, normalize: bool = True,
              num_retry: int = 0) -> None:
        self._note("write", data_name, {motor: value})
        self._need_connection()
        self._check_value(data_name, motor, value, normalize)
        raw = self._to_ticks(motor, value) if normalize and data_name in _NORMALISED else int(value)
        self._apply(data_name, motor, raw)

    def enable_torque(self, motors=None, num_retry: int = 0) -> None:
        self._note("enable_torque")
        for m in self._names(motors):
            self.write("Torque_Enable", m, 1)
            self.write("Lock", m, 1)

    def disable_torque(self, motors=None, num_retry: int = 0) -> None:
        self._note("disable_torque")
        for m in self._names(motors):
            self.write("Torque_Enable", m, 0)
            self.write("Lock", m, 0)

    @contextmanager
    def torque_disabled(self, motors=None):
        self.disable_torque(motors)
        try:
            yield
        finally:
            self.enable_torque(motors)

    def configure_motors(self, return_delay_time: int = 0, maximum_acceleration: int = 254,
                         acceleration: int = 254) -> None:
        self._note("configure_motors")
        for m in self.motors:
            self.write("Return_Delay_Time", m, return_delay_time)
            self.write("Maximum_Acceleration", m, maximum_acceleration)
            self.write("Acceleration", m, acceleration)


class SimSO101Follower:
    """Stands in for LeRobot's SOFollower over a SimFeetechBus: its configure(), send_action() and
    max_relative_target clamp follow LeRobot 0.6.x. Its connect() and disconnect() are LeRobot's
    too, which is exactly why the adapter must never call them; ``calls`` records robot-level calls
    so tests can check that."""

    name = "so101_follower"

    def __init__(self, bus: SimFeetechBus, calibration: Mapping[str, MotorCal],
                 max_relative_target: float | None = None):
        self.bus = bus
        self.calibration = dict(calibration)
        self.config = SimpleNamespace(max_relative_target=max_relative_target,
                                      disable_torque_on_disconnect=True,
                                      position_p_coefficient=16, position_i_coefficient=0,
                                      position_d_coefficient=32)
        self.calls: list[str] = []

    @property
    def action_features(self) -> dict:
        return {f"{m}.pos": float for m in self.bus.motors}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def connect(self, calibrate: bool = True) -> None:
        """LeRobot's connect(): enables torque toward whatever the goal registers hold."""
        self.calls.append("connect")
        self.bus.connect()
        self.configure()

    def configure(self) -> None:
        self.calls.append("configure")
        with self.bus.torque_disabled():
            self.bus.configure_motors()
            for m in self.bus.motors:
                self.bus.write("Operating_Mode", m, 0)
                self.bus.write("P_Coefficient", m, self.config.position_p_coefficient)
                self.bus.write("I_Coefficient", m, self.config.position_i_coefficient)
                self.bus.write("D_Coefficient", m, self.config.position_d_coefficient)

    def send_action(self, action: Mapping) -> dict:
        self.calls.append("send_action")
        goal = {k.removesuffix(".pos"): v for k, v in action.items() if k.endswith(".pos")}
        cap = self.config.max_relative_target
        if cap is not None:
            present = self.bus.sync_read("Present_Position")
            goal = {m: present[m] + max(-cap, min(cap, g - present[m])) for m, g in goal.items()}
        self.bus.sync_write("Goal_Position", goal)
        return {f"{m}.pos": v for m, v in goal.items()}

    def disconnect(self) -> None:
        """LeRobot's disconnect(): with the default config it turns torque off (the arm drops)."""
        self.calls.append("disconnect")
        self.bus.disconnect(self.config.disable_torque_on_disconnect)


class SimSO101Adapter(LeRobotAdapter):
    """The ``lerobot`` adapter over a simulated SO-101. Everything above the bus is the real
    adapter; ``_preflight`` is the only safety step skipped (there is no serial port to scan)."""

    name = "sim-so101"
    simulation = True

    def __init__(self, *, command_space_path, urdf_path, joint_map: JointMap | None = None,
                 start_pose: Mapping[str, float] | None = None, sag: float = 1.0,
                 friction: float = 1.0, tau_s: float = 0.1, slew_deg_s: float = 270.0,
                 calibration_path: str | Path | None = None, power_up_goals_zero: bool = True,
                 realtime: bool = True, record_bus_calls: bool = False,
                 estop_file: str | Path | None = None, **kwargs):
        super().__init__(command_space_path=command_space_path, port="sim://so101", robot_id="sim",
                         make_robot=self._build_sim, **kwargs)
        self.urdf_path = Path(urdf_path)
        self.joint_map = joint_map or JointMap()
        self.start_pose = dict(start_pose or {})
        self.sag_scale, self.friction_scale = float(sag), float(friction)
        self.tau_s, self.slew_deg_s = float(tau_s), float(slew_deg_s)
        self.calibration_path = calibration_path
        self.power_up_goals_zero = power_up_goals_zero
        self.realtime = realtime
        self.record_bus_calls = record_bus_calls
        self.estop_file = Path(estop_file) if estop_file else None
        self.servos: ServoBank | None = None
        self.sim_robot: SimSO101Follower | None = None
        self._physics: threading.Thread | None = None
        self._physics_quit = threading.Event()

    def _preflight(self) -> None:
        pass  # no serial port, and nothing else can reach a simulated bus

    def _build_sim(self) -> SimSO101Follower:
        cal = sim_calibration(self.calibration_path)
        sag = None
        if self.sag_scale > 0:
            model = URDFModel.from_file(self.urdf_path)
            sag = GravitySag(ToolHeight(model, self.joint_map), scale=self.sag_scale,
                             friction_scale=self.friction_scale)
        servos = ServoBank(cal, self.slew_deg_s, tau_s=self.tau_s, sag=sag)
        for m, value in self.start_pose.items():
            if m not in cal:
                continue
            c = cal[m]
            if m == "gripper":  # the gripper uses LeRobot's 0..100 range
                t = c.range_min + min(100.0, max(0.0, value)) / 100.0 * (c.range_max - c.range_min)
            else:  # body joints are degrees about the middle of the range
                t = value * MAX_RES / 360.0 + _mid(c)
            servos.present[m] = servos.goal[m] = t
        if sag is not None:  # start where the arm would have settled, past its goal
            for m, v in sag(servos.degrees()).items():
                servos.present[m] += v
        if self.power_up_goals_zero:  # bob at power-up: torque off, goal registers read 0
            for m in servos.goal:
                servos.goal[m] = 0
                servos.torque[m] = 0
        bus = SimFeetechBus(servos, cal, record=self.record_bus_calls)
        self.servos = servos
        self.sim_robot = SimSO101Follower(bus, cal, self.max_relative_target)
        return self.sim_robot

    def open(self):
        space = super().open()
        if self.realtime:
            self._physics_quit.clear()
            self._physics = threading.Thread(target=self._run_physics, name="be-sim-physics",
                                             daemon=True)
            self._physics.start()
        return space

    def _estop(self) -> None:
        """The simulated external stop: while the e-stop file exists, the servos are unpowered."""
        if self.estop_file is not None and self.servos.powered and self.estop_file.exists():
            self.servos.cut_power()

    def _run_physics(self) -> None:
        last = time.perf_counter()
        while not self._physics_quit.wait(0.001):
            now = time.perf_counter()
            self._estop()
            self.servos.step(now - last)
            last = now

    def step(self, dt: float, substep: float = 0.001) -> None:
        """Advance the simulated servos by dt (tests that run without the real-time thread)."""
        if self.servos is None:
            raise AdapterError("step: the simulated robot has not been built")
        n = max(1, round(dt / substep))
        for _ in range(n):
            self._estop()
            self.servos.step(dt / n)

    def reset_stop(self, source: str) -> None:
        """EV-02 after the operator's NEXT. The simulated external stop (``hardwired_stop`` or
        ``power_loss``): refused while the e-stop file exists; then power back, every goal set to
        the joint's present position, read back, and only then torque on (``connect_safely``'s
        order). Other sources: the lerobot adapter's reset (the wrapper's resume())."""
        if source not in ("hardwired_stop", "power_loss"):
            super().reset_stop(source)
            return
        if self.estop_file is None:
            raise AdapterError("sim-so101 has no simulated external stop (--sim-estop-file)")
        if self.estop_file.exists():
            raise AdapterError(f"the simulated e-stop is still open: {self.estop_file} exists")
        if self._latched.is_set():
            raise AdapterError("the runner's stop is latched")
        with self._bus_lock:
            if self._latched.is_set():
                raise AdapterError("the runner's stop is latched")
            bus = self.sim_robot.bus
            self.servos.restore_power()
            present = bus.sync_read("Present_Position", normalize=False)
            bus.sync_write("Goal_Position", dict(present), normalize=False)
            goals = bus.sync_read("Goal_Position", normalize=False)
            if any(abs(goals[m] - present[m]) > 1 for m in present):
                raise AdapterError(f"the goals did not take: {goals} against {present}")
            bus.enable_torque()

    def close(self) -> None:
        try:
            super().close()
        finally:
            self._physics_quit.set()
            if self._physics is not None:
                self._physics.join(timeout=2.0)
                self._physics = None

    def true_joints(self) -> dict[str, float]:
        """Ground truth: the simulated joints in LeRobot units (degrees; gripper 0..100)."""
        if self.servos is None or self.sim_robot is None:
            return {}
        deg = self.servos.degrees()
        g = self.sim_robot.bus._to_user("gripper", round(self.servos.present["gripper"]))
        return {**{m: deg[m] for m in ARM_JOINTS}, "gripper": g}

    def truth_fn(self, fk, envelope=None) -> Callable[[], dict]:
        """For the simulated instrument: the envelope points, from the true joints."""
        units = {**dict.fromkeys(ARM_JOINTS, "deg"), "gripper": "percent"}
        return lambda: fk.points(self.true_joints(), units)

    def describe(self) -> dict:
        d = super().describe()
        sim = {"urdf": str(self.urdf_path), "sag": self.sag_scale, "friction": self.friction_scale,
               "tau_s": self.tau_s, "slew_deg_s": self.slew_deg_s,
               "calibration": str(self.calibration_path) if self.calibration_path else "symmetric",
               "power_up_goals_zero": self.power_up_goals_zero}
        d["config"].update({"sim": sim})
        if self.estop_file is not None:
            d["sim_estop_file"] = str(self.estop_file)
            d["stop_resets"] = [*d["stop_resets"], "hardwired_stop", "power_loss"]
        return d


# ----- command line ----------------------------------------------------------------------------
def add_arguments(p) -> None:
    g = p.add_argument_group("sim-so101 adapter")
    g.add_argument("--sim-sag", type=float, default=1.0,
                   help="gravity sag as a multiple of the reference SO-101's measured sag (0 = ideal servos)")
    g.add_argument("--sim-friction", type=float, default=1.0, help="friction band, as a multiple of the reference SO-101's")
    g.add_argument("--sim-tau", type=float, default=0.1, help="servo response time constant, s")
    g.add_argument("--sim-slew", type=float, default=270.0, help="servo slew limit, deg/s")
    g.add_argument("--sim-calibration", help="a LeRobot calibration file, to simulate that arm")
    g.add_argument("--sim-start", choices=["start", "rest"], default="start",
                   help="begin at the kit config's start pose, or folded at rest")
    g.add_argument("--sim-estop-file", metavar="PATH",
                   help="EV-02 in simulation: the servos lose power while PATH exists (the lab's "
                        "--power-loss-cmd creates it; delete it to reset). Also read by sim-rover.")


def from_args(args, context) -> SimSO101Adapter:
    if context.urdf_path is None:
        raise AdapterError("sim-so101 needs --urdf (the gravity model and the simulated instrument "
                           "use the arm's kinematics)")
    start = SIM_REST_POSE if args.sim_start == "rest" else context.kit_config.get("start_pose_deg", {})
    return SimSO101Adapter(command_space_path=context.command_space_path, urdf_path=context.urdf_path,
                           joint_map=JointMap.from_dict(context.kit_config.get("joint_map")),
                           start_pose=start, sag=args.sim_sag, friction=args.sim_friction,
                           tau_s=args.sim_tau, slew_deg_s=args.sim_slew,
                           calibration_path=args.sim_calibration,
                           wrapper=getattr(args, "wrapper", None),
                           wrapper_context=context.wrapper_context(),
                           max_relative_target=getattr(args, "max_relative_target", None),
                           estop_file=getattr(args, "sim_estop_file", None),
                           prompt=context.prompt, log=context.log)
