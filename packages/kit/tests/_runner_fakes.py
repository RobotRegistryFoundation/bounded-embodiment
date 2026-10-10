# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""Stand-ins for the runner and adapter tests: the repo's example files, the simulated SO-101, a
feed the test drives by hand, a recording lab wrapper, a pure-Python adapter, and fake e-stop and
alarm. Nothing here touches hardware."""
from __future__ import annotations

import json
import math
import threading
import types
from pathlib import Path

from bounded_embodiment.commandspace import load_command_space
from bounded_embodiment_kit.adapters.base import SendResult, State
from bounded_embodiment_kit.adapters.sim_so101 import SimSO101Adapter
from bounded_embodiment_kit.kinematics import JointMap
from bounded_embodiment_kit.runner.feeds import FeedSample, _Store

REPO = Path(__file__).resolve().parents[3]
DATA = Path(__file__).parent / "data"
URDF = REPO / "examples" / "robots" / "so101" / "so101_new_calib.urdf"
SO101_SPACE = REPO / "examples" / "command-spaces" / "so101.command-space.json"
ROVER_SPACE = REPO / "examples" / "command-spaces" / "rover.command-space.json"
ENVELOPES = REPO / "examples" / "envelopes"
BOB_CAL = DATA / "bob_calibration.json"


def kit(name: str) -> dict:
    return json.loads((ENVELOPES / f"{name}.kit.json").read_text())


def envelope_path(name: str) -> Path:
    return ENVELOPES / f"{name}.envelope.json"


def sim_adapter(name: str = "so101", *, realtime: bool = False, record: bool = False,
                calibration: Path | None = None, wrapper: str | None = None, answers=("",),
                **kw) -> SimSO101Adapter:
    """The simulated SO-101 with the example's kit config (``name`` = so101 or bob)."""
    cfg = kit(name)
    it = iter(answers)
    return SimSO101Adapter(command_space_path=SO101_SPACE, urdf_path=URDF,
                           joint_map=JointMap.from_dict(cfg.get("joint_map")),
                           start_pose=cfg["start_pose_deg"], calibration_path=calibration,
                           realtime=realtime, record_bus_calls=record, wrapper=wrapper,
                           prompt=lambda q: next(it, ""), log=lambda m: None, **kw)


class ScriptedFeed:
    """An InstrumentFeed whose samples the test puts in by hand."""

    def __init__(self, clock):
        self.clock = clock
        self._store = _Store(clock())

    def start(self) -> None:
        pass

    def put(self, point: str, xyz, t: float | None = None, valid: bool = True) -> None:
        t = self.clock() if t is None else t
        self._store.put(FeedSample(t, point, tuple(xyz) if valid else None, valid, t), t, False)

    def set_camera_moved(self) -> None:
        self._store.put(None, self.clock(), True)

    def snapshot(self):
        return self._store.snapshot()

    def close(self) -> None:
        pass


class FeedPump(threading.Thread):
    """Puts a tool sample into a ScriptedFeed every 1/fps s from ``position(t)``."""

    def __init__(self, feed: ScriptedFeed, position, fps: float = 50.0, point: str = "tool"):
        super().__init__(name="test-feed-pump", daemon=True)
        self.feed, self.position, self.period, self.point = feed, position, 1.0 / fps, point
        self.quit = threading.Event()

    def run(self) -> None:
        while not self.quit.wait(self.period):
            t = self.feed.clock()
            self.feed.put(self.point, self.position(t), t)


# ----- a lab's gate, as the lerobot adapter's --wrapper loads it -------------------------------
WRAPPERS: list = []


class RecordingWrapper:
    """A gated wrapper Robot that forwards send_action and records every other attribute the
    adapter touches (connect, configure, disconnect would show up here)."""

    def __init__(self, robot, context):
        object.__setattr__(self, "_robot", robot)
        object.__setattr__(self, "context", context)
        object.__setattr__(self, "touched", [])
        object.__setattr__(self, "last_decision", "allow")

    def send_action(self, action):
        return self._robot.send_action(action)

    def __getattr__(self, name):
        self.touched.append(name)
        return getattr(self._robot, name)


def make_wrapper(robot, context):
    w = RecordingWrapper(robot, context)
    WRAPPERS.append(w)
    return w


def failing_factory(robot, context):
    raise RuntimeError("the lab's gate failed to build")


# ----- a pure-Python adapter -------------------------------------------------------------------
class FakeAdapter:
    """The adapter contract with no physics: records which thread called what, and when."""

    name = "fake"
    simulation = True

    def __init__(self, clock, *, block_send: threading.Event | None = None):
        self.clock = clock
        self.space = load_command_space(SO101_SPACE)
        self.sends: list[tuple[float, str]] = []
        self.stops: list[tuple[float, str]] = []
        self.states: list[str] = []
        self.latched = threading.Event()
        self.block_send = block_send
        self.joints = {"shoulder_pan": 0.0, "shoulder_lift": -40.0, "elbow_flex": 25.0,
                       "wrist_flex": 75.0, "wrist_roll": 0.0, "gripper": 30.0}

    def open(self):
        return self.space

    def describe(self):
        return {"adapter": self.name, "simulation": True, "config": {"adapter": self.name},
                "joint_speed_trip_per_s": {}}

    def send(self, cmd):
        if self.latched.is_set():
            return SendResult(False, None, "stop_latched")
        self.sends.append((self.clock(), threading.current_thread().name))
        if self.block_send is not None:
            self.block_send.wait()
        return SendResult(True, None, None)

    def read_state(self):
        self.states.append(threading.current_thread().name)
        units = {j: ("percent" if j == "gripper" else "deg") for j in self.joints}
        return State(dict(self.joints), units)

    def latch_stop(self):
        self.latched.set()

    def execute_stop(self):
        self.latched.set()
        self.stops.append((self.clock(), threading.current_thread().name))

    def close(self):
        pass


class FakeRun:
    """Stands in for subprocess.run: records the e-stop command it was asked to run."""

    def __init__(self):
        self.calls: list[list[str]] = []

    def __call__(self, argv, **kw):
        self.calls.append(list(argv))
        return types.SimpleNamespace(returncode=0)


class FakeAlarm:
    def __init__(self):
        self.sounded: list[str] = []
        self.stopped = False

    def sound(self, reason):
        self.sounded.append(reason)

    def stop(self):
        self.stopped = True


def dist(a, b) -> float:
    return math.dist(a, b)
