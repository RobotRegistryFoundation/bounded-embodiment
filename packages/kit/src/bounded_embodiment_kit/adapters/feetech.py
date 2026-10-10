# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""DRAFT — requires line-by-line human review before use on hardware.

Feetech-bus (SO-100 / SO-101) safety code shared by the ``lerobot`` and ``sim-so101`` adapters.

Everything here talks to a LeRobot ``FeetechMotorsBus`` (or the simulator's stand-in for one)
through its public methods only: ``connect``, ``disconnect``, ``is_calibrated``, ``sync_read``,
``sync_write``, ``disable_torque``. Raw values (``normalize=False``) are encoder ticks.

Ported from ev03-harness ``ev03/arm/hw.py`` (reviewed): ``connect_safely``, ``check_bus_is_ours``
and the typed-OFF part of ``finish``. Changes from hw.py, each for a reason:

- connect_safely reads the goal registers back after syncing them: ``sync_write`` gets no status
  packet, so a lost packet would otherwise leave a stale goal (bob's read 0 at power-up) for
  ``configure()`` to drive toward when it enables torque.
- connect_safely releases the port without touching torque if anything fails after it opened it.
- check_bus_is_ours returns the problems instead of calling SystemExit, and says so plainly when
  it cannot scan (no /proc: not Linux) instead of crashing.
- The typed OFF must be exactly ``OFF`` (hw.py accepted any case). Anything else leaves torque on,
  which is the safe failure.
- choose_stop_goals is new. hw.py's ``hold()`` re-anchored EVERY stop to the present position;
  here the present position is read once, at the first stop of a latch, and every later stop
  re-sends it (so-arm101-actuator's PR #7 rule, which the plan cites).
"""
from __future__ import annotations

import os
import subprocess
from collections.abc import Callable, Mapping, Sequence

from .base import AdapterError

__all__ = [
    "BODY_JOINT_SPEED_TRIP_DPS",
    "HOLD_WITHIN_TICKS",
    "check_bus_is_ours",
    "choose_stop_goals",
    "connect_safely",
    "leave_safe",
    "norm_name",
]

#: connect_safely: a powered joint whose goal register is within this many encoder ticks of its
#: position is holding there, and its goal is left alone. 50 ticks is 4.4 deg on a 4096-tick
#: STS3215: above bob's measured gravity sag plus friction (27 ticks at the shoulder, lifted into
#: place), well below any real move. (hw.py's value.)
HOLD_WITHIN_TICKS = 50

#: The SO-101 joint-speed tripwire (hw.py's 150 deg/s). Servos slew at about 270 deg/s, so a body
#: joint above 150 deg/s over the 0.2 s window is a joint something drove hard, gate or not.
BODY_JOINT_SPEED_TRIP_DPS = 150.0


def norm_name(motor: object) -> str:
    """A LeRobot Motor's norm mode as text: "DEGREES", "RANGE_0_100" or "RANGE_M100_100"."""
    mode = getattr(motor, "norm_mode", None)
    return str(getattr(mode, "name", mode))


def _release(bus: object, log: Callable[[str], None]) -> None:
    """Close the port without touching torque. Never raises: it runs on the way out of an error."""
    try:
        if getattr(bus, "is_connected", False):
            bus.disconnect(disable_torque=False)  # closePort only; no register write
    except Exception as e:  # noqa: BLE001 - already failing; report and keep the first error
        log(f"Could not release the port cleanly ({type(e).__name__}: {e}).")


def connect_safely(robot: object, *, log: Callable[[str], None],
                   hold_within_ticks: int = HOLD_WITHIN_TICKS) -> dict:
    """Connect an SO-10x follower's bus without the arm jumping, then let LeRobot's own
    ``configure()`` set gains and acceleration and enable torque.

    Why not ``robot.connect()``: it runs ``configure()``, which enables torque toward whatever the
    goal registers hold, and bob's read 0 at power-up. So the bus is connected here, and every goal
    register is set to where its joint already is BEFORE torque comes on. If torque is already on
    and every goal is within ``hold_within_ticks`` of its joint, the arm is holding there and its
    goals are left alone: rewriting them to the measured (sagging) position would lower the arm by
    its sag.

    Returns what it found, for the adapter's describe(). Raises AdapterError (port released, torque
    untouched) if the calibration is missing or does not match the servos, or a goal did not take.
    """
    bus = robot.bus
    if not getattr(robot, "calibration", None):
        raise AdapterError("No LeRobot calibration file for this robot id. Calibrate with "
                           "lerobot-calibrate first (or save the servos' own with the harness probe).")
    body = [m for m in bus.motors if m != "gripper"]
    wrong = {m: norm_name(bus.motors[m]) for m in body if norm_name(bus.motors[m]) != "DEGREES"}
    if wrong:
        raise AdapterError(f"These joints are not in degrees: {wrong}. Build the robot with "
                           "use_degrees=True; the command space and the joint map are in degrees.")
    try:
        bus.connect()  # opens the port and pings the servos; no register write
        if not bus.is_calibrated:
            raise AdapterError("The servos' stored calibration differs from the calibration file. "
                               "Re-run lerobot-calibrate, or use the file that matches the servos.")
        present = bus.sync_read("Present_Position", normalize=False)
        goal = bus.sync_read("Goal_Position", normalize=False)
        torque = bus.sync_read("Torque_Enable", normalize=False)
        holding = all(torque.get(m) == 1 and abs(goal[m] - present[m]) <= hold_within_ticks
                      for m in present)
        if not holding:
            bus.sync_write("Goal_Position", dict(present), normalize=False)
            back = bus.sync_read("Goal_Position", normalize=False)
            off = {m: back.get(m) for m in present if back.get(m) is None or abs(back[m] - present[m]) > 1}
            if off:
                raise AdapterError(f"The goal registers did not take the synced positions {off}; "
                                   "not enabling torque. Check the bus wiring and power.")
        robot.configure()  # LeRobot's own: torque off, settings, torque on (now toward synced goals)
    except BaseException:
        _release(bus, log)
        raise
    log("Connected: " + ("the arm was already holding; goals left as they were." if holding
                         else "goal registers set to the present positions before torque."))
    return {"holding_at_connect": holding, "goals_synced_to_present": not holding,
            "torque_at_connect": dict(torque)}


def choose_stop_goals(present_raw: Mapping[str, int]) -> dict[str, int]:
    """The goals a stop holds: every joint's present position, read at the FIRST stop of a latch.
    The caller keeps them and re-sends exactly these on every later stop of the same latch.

    Why the present position, once: the joint may be moving (perhaps toward a hostile goal), so the
    first stop has to stop it where it is. A loaded joint then settles one sag below that goal.
    Why never again: reading the (sagged) position on every repeated stop re-anchors the goal lower
    each time, and the arm ratchets down (bob; so-arm101-actuator's estop before its PR #7).

    Rejected alternative: keeping a joint's goal register when it is within a few degrees. That
    removes even the single sag for an arm holding still, but a joint still travelling the last
    few degrees to its goal runs on after the stop; in the simulator that kept the tool moving past
    the 0.25 s acknowledgement window, which escalates. docs/seams/runner.md has the numbers.
    """
    return {m: int(p) for m, p in present_raw.items()}


def check_bus_is_ours(port: str, services: Sequence[str] = ()) -> list[str]:
    """Problems that mean something else could command the arm: a process holding the serial port
    open, or a named service that is running (a service may open the port only when it gets a
    request, so an idle one does not show up as holding it; name it with --refuse-while-active).

    Linux only (it reads /proc). Elsewhere it reports that it could not check, which the adapter
    treats as a refusal."""
    problems: list[str] = []
    if not os.path.isdir("/proc"):
        return [f"cannot check who holds {port}: no /proc on this system (the scan is Linux-only)"]
    real = os.path.realpath(port)
    me = os.getpid()
    for pid in filter(str.isdigit, os.listdir("/proc")):
        if int(pid) == me:
            continue
        try:
            fds = os.listdir(f"/proc/{pid}/fd")
        except OSError:
            continue
        for fd in fds:
            try:
                if os.path.realpath(f"/proc/{pid}/fd/{fd}") != real:
                    continue
                with open(f"/proc/{pid}/cmdline", "rb") as fh:
                    cmd = fh.read().replace(b"\0", b" ").decode(errors="replace")[:120]
                problems.append(f"process {pid} has {port} open: {cmd}")
                break
            except OSError:
                continue
    for unit in services:
        for scope in (["--user"], []):
            try:
                r = subprocess.run(["systemctl", *scope, "is-active", unit], capture_output=True,
                                   text=True, timeout=10, check=False)
            except (FileNotFoundError, subprocess.TimeoutExpired) as e:
                problems.append(f"cannot check {unit}: {type(e).__name__}")
                break
            if r.stdout.strip() == "active":
                where = "user" if scope else "system"
                stop = " ".join([*scope, "stop", unit])
                problems.append(f"{where} service {unit} is running (systemctl {stop})")
    return problems


def leave_safe(bus: object, prompt: Callable[[str], str], log: Callable[[str], None]) -> str:
    """Ask before turning torque off, then release the port without touching torque.

    Only a typed ``OFF`` turns torque off (the arm goes limp: support it first). Just Enter, any
    other answer, or end of input leaves torque on and the arm holding its last goal. LeRobot's
    own ``robot.disconnect()`` is never used: ``disable_torque_on_disconnect`` defaults to True and
    would drop the arm. Returns "torque_off" or "torque_on"."""
    try:
        answer = prompt("Support the arm, then type OFF and Enter to turn torque off (it goes limp), "
                        "or just Enter to leave torque on: ")
    except EOFError:
        answer = ""
    outcome = "torque_on"
    try:
        if answer.strip() == "OFF":
            bus.disable_torque()
            outcome = "torque_off"
            log("Torque off.")
        else:
            log("Torque left on; the arm holds its last goal.")
    finally:
        _release(bus, log)
    return outcome
