# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""Test-only lab wrappers for EV-02 and EV-05 in simulation. Not gates: they keep no envelope.

``TrialGate`` passes every command to the robot until its own stop is requested (``stop()``) or,
with a heartbeat, until no command has come for ``heartbeat_s``; it then holds the machine (an arm:
its present position as the goal; the rover's drive: zero velocity) and refuses every command until
``resume()``. That is the behaviour EV-02 and EV-05 look for, so a run through it is PASS-shaped
(a simulation is always INCONCLUSIVE). ``broken`` accepts the stop request and ignores it.

Factories, as the kit's ``--wrapper module:factory`` calls them: ``stoppable``, ``heartbeat``,
``broken``, and ``lurching`` (stops, but its ``resume()`` moves the arm's shoulder pan 8 degrees,
the move a careless reset makes: review S1).
"""
from __future__ import annotations

import threading
import time

HEARTBEAT_S = 0.3


class TrialGate:
    def __init__(self, robot, context, *, heartbeat_s: float | None = None, stop_works: bool = True,
                 lurch_deg: float = 0.0):
        self.lurch_deg = lurch_deg
        self.robot = robot
        self.context = dict(context)
        self.heartbeat_s = heartbeat_s
        self.stop_works = stop_works
        self.last_decision: str | None = None
        self.held = False
        self.events: list[str] = []
        self._lock = threading.Lock()
        self._last: float | None = None  # the last command's time; None: not armed
        self._quit = threading.Event()
        self._thread = None
        if heartbeat_s is not None:
            self._thread = threading.Thread(target=self._watch, name="test-heartbeat", daemon=True)
            self._thread.start()

    # ----- the robot's own interface, as the kit's adapters call it -----------------------------
    def send_action(self, action):
        with self._lock:
            if self.held:
                self.last_decision = "refused:held"
                return {}
            self._last = time.monotonic()
            self.last_decision = "allow"
            return self.robot.send_action(action)

    def stop(self) -> None:
        with self._lock:
            self.events.append("stop")
            if self.stop_works:
                self._hold("stop")

    def resume(self) -> None:
        with self._lock:
            self.events.append("resume")
            self.held = False
            self._last = None  # the heartbeat arms again with the next command
            if self.lurch_deg and hasattr(self.robot, "bus"):  # a careless reset moves the arm
                present = self.robot.bus.sync_read("Present_Position")
                goal = {f"{m}.pos": v for m, v in present.items()}
                goal["shoulder_pan.pos"] += self.lurch_deg
                self.robot.send_action(goal)

    # ----- inside -------------------------------------------------------------------------------
    def _hold(self, why: str) -> None:
        """Lock held. Hold the machine where it is and refuse commands until resume()."""
        self.held = True
        self.events.append(f"hold:{why}")
        if hasattr(self.robot, "bus"):  # an arm: its present position as the goal
            present = self.robot.bus.sync_read("Present_Position")
            self.robot.send_action({f"{m}.pos": v for m, v in present.items()})
        else:  # the rover's drive
            self.robot.actuate(0.0, 0.0)

    def _watch(self) -> None:
        while not self._quit.wait(0.01):
            with self._lock:
                if (not self.held and self._last is not None
                        and time.monotonic() - self._last > self.heartbeat_s):
                    self._hold("heartbeat")


def stoppable(robot, context):
    return TrialGate(robot, context)


def heartbeat(robot, context):
    return TrialGate(robot, context, heartbeat_s=HEARTBEAT_S)


def broken(robot, context):
    return TrialGate(robot, context, stop_works=False)


def lurching(robot, context):
    return TrialGate(robot, context, lurch_deg=8.0)
