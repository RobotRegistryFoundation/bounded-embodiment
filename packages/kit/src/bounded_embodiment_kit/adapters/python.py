# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""DRAFT — requires line-by-line human review before use on hardware.

The ``python`` adapter: a lab's own adapter class, loaded by name (``--adapter-class
path/to/my_adapter.py:Adapter`` or ``module:Class``) and constructed as ``Class(context)`` with
the run's ``AdapterContext``. ``be adapter new NAME`` writes a starting point (``TEMPLATE``).

The runner checks that the object has every method of the contract (``adapters.base``) before it
calls any of them.
"""
from __future__ import annotations

from pathlib import Path

from .base import AdapterError, check_adapter, load_callable

__all__ = ["TEMPLATE", "add_arguments", "from_args", "write_template"]


def add_arguments(p) -> None:
    g = p.add_argument_group("python adapter")
    g.add_argument("--adapter-class", metavar="FILE.py:CLASS",
                   help="the lab's adapter class (python adapter), constructed as Class(context)")


def from_args(args, context):
    if not getattr(args, "adapter_class", None):
        raise AdapterError("the python adapter needs --adapter-class FILE.py:CLASS (be adapter new "
                           "NAME writes a template)")
    cls = load_callable(args.adapter_class)
    adapter = cls(context)
    check_adapter(adapter)
    return adapter


def write_template(name: str, out_dir: str | Path = ".") -> Path:
    """Write ``<name>_adapter.py`` from TEMPLATE; refuse to overwrite."""
    if not name.isidentifier():
        raise AdapterError(f"{name!r} is not a Python identifier")
    path = Path(out_dir) / f"{name}_adapter.py"
    if path.exists():
        raise AdapterError(f"{path} already exists; not overwriting it")
    path.write_text(TEMPLATE.replace("{NAME}", name), encoding="utf-8")
    return path


TEMPLATE = '''"""{NAME}: a Bounded Embodiment adapter (written by `be adapter new {NAME}`).

DRAFT — requires line-by-line human review before use on hardware.

Fill in the four places marked TODO. Run it with:

    be run ev03 --mode shakedown --adapter python --adapter-class {NAME}_adapter.py:Adapter ...

Rules the runner relies on (bounded_embodiment_kit.adapters.base has the full contract):

- open() connects WITHOUT the robot jumping: set every position goal to where the joint already is
  before anything enables torque. It returns the lab's declared command space, loaded from the
  file, never built from what the robot reports.
- send() never raises. It hands the command to your system under test (your gate, your model's
  command interface) exactly as your model would, and reports what happened.
- latch_stop() is called from the WATCHDOG thread: it only sets a flag. No bus access, no lock.
- execute_stop() runs on the command thread, under the bus lock. It chooses the stop goals ONCE
  per latch and re-sends exactly the same goals on every later call. Never re-read the present
  position on a later stop: a sagging arm re-anchored on every stop ratchets downward.
- close() leaves the robot holding (torque on) unless the operator types OFF.
- EV-02 and EV-05 only (optional; without them `be run ev02|ev05` refuses what needs them):
  trigger_stop("stop_message") issues YOUR SYSTEM'S OWN stop request (never the runner's stop) and
  returns at once; reset_stop(source) lets your system accept commands again after that stop, after
  a hardwired stop or power loss the operator has reset, or after a model loss. It must not move
  the robot. List what they support in describe()["stop_triggers"] and ["stop_resets"].
"""
import threading

from bounded_embodiment.commandspace import load_command_space
from bounded_embodiment_kit.adapters.base import SendResult, State


class Adapter:
    name = "{NAME}"
    simulation = False

    def __init__(self, context):
        self.context = context              # AdapterContext: paths, mode, prompt, log
        self._bus_lock = threading.RLock()  # every bus access takes it
        self._latched = threading.Event()
        self._stop_goals = None
        self.robot = None

    def open(self):
        space = load_command_space(self.context.command_space_path)
        # TODO: connect to the robot WITHOUT enabling torque; set every goal to the present
        # position; read the goals back; only then enable torque. Then wrap it in your gate.
        raise NotImplementedError("{NAME}: open() is not written yet")
        self._latched.clear()
        self._stop_goals = None
        return space

    def describe(self):
        # "config" goes into the setup hash: everything that changes what is tested.
        return {"adapter": self.name, "simulation": False,
                "config": {"adapter": self.name},          # TODO: port, robot id, gate version...
                "joint_speed_trip_per_s": {},              # e.g. {"elbow": 150.0} (deg/s)
                "stop_triggers": [],                       # EV-02: e.g. ["stop_message"]
                "stop_resets": []}                         # e.g. ["stop_message", "power_loss"]

    def send(self, cmd):
        if self._latched.is_set():
            return SendResult(False, None, "stop_latched")
        try:
            with self._bus_lock:
                if self._latched.is_set():
                    return SendResult(False, None, "stop_latched")
                # TODO: hand cmd to your system under test, exactly as your model would.
                raise NotImplementedError
        except Exception as e:  # send() never raises
            return SendResult(False, None, f"exception:{type(e).__name__}")
        return SendResult(True, None, None)

    def read_state(self):
        # TODO: return State(joints={name: value}, units={name: "deg"}) from the encoders, or
        # None if the robot has no joint state (then a run needs --no-fk-tripwire-i-understand).
        return None

    def latch_stop(self):
        self._latched.set()                 # watchdog thread: a flag, nothing else

    def execute_stop(self):
        self._latched.set()
        with self._bus_lock:
            if self._stop_goals is None:
                # TODO: choose the goals ONCE: read every joint's present position (a joint may
                # be moving, so stop it where it is). Never read them again for a later stop.
                self._stop_goals = {}
            # TODO: send self._stop_goals to the robot, bypassing the gate.

    def trigger_stop(self, source):
        # EV-02, command thread: issue your system's own stop request through `source`
        # ("stop_message"). Do not latch the runner's stop. Raise if it cannot be issued.
        raise NotImplementedError("{NAME}: trigger_stop() is not written")

    def reset_stop(self, source):
        # EV-02 / EV-05, command thread, after the operator typed NEXT: let your system accept
        # commands again. Must not move the robot. Raise if it cannot.
        raise NotImplementedError("{NAME}: reset_stop() is not written")

    def close(self):
        answer = self.context.prompt("Type OFF and Enter to turn torque off, or Enter to leave it on: ")
        if answer.strip() == "OFF":
            pass                            # TODO: torque off
        # TODO: release the connection without touching torque.
'''
