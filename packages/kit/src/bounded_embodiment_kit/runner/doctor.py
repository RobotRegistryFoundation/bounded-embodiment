# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""DRAFT — requires line-by-line human review before use on hardware.

``be doctor``: what is on this machine and what state the arm is in. Strictly read-only.

It never enables torque and never writes a register: anything that writes counts as motion and
belongs to ``be run`` or ``be goto-start``. On the bus it only connects (LeRobot's
``bus.connect()`` opens the port and pings the servos), reads registers, and disconnects with
``disable_torque=False`` (closes the port; torque stays exactly as it was found). It never calls
the robot's ``connect()`` or ``disconnect()``: the first enables torque, the second turns it off.

Ported from ev03-harness ``ev03/arm/probe.py`` (``list_environment``, ``read_arm``), plus an
instrument listener for the UDP feed.
"""
from __future__ import annotations

import glob
import os
import platform
import sys
import time
from collections.abc import Callable
from pathlib import Path

from bounded_embodiment import __version__ as core_version

from .. import __version__ as kit_version
from ..adapters.feetech import HOLD_WITHIN_TICKS, check_bus_is_ours, norm_name
from ..adapters.lerobot import lerobot_version
from .feeds import UDPFeed
from .watchdog import clearance

__all__ = ["calibration_dirs", "doctor_arm", "environment", "listen"]


def calibration_dirs() -> list[Path]:
    roots = []
    if os.environ.get("HF_LEROBOT_CALIBRATION"):
        roots.append(Path(os.environ["HF_LEROBOT_CALIBRATION"]))
    hf_home = Path(os.environ.get("HF_HOME", Path.home() / ".cache" / "huggingface"))
    roots.append(hf_home / "lerobot" / "calibration")
    return roots


def environment() -> dict:
    out = {"python": sys.version.split()[0], "platform": platform.platform(),
           "bounded_embodiment": core_version, "bounded_embodiment_kit": kit_version,
           "lerobot": lerobot_version(), "serial_ports": [], "calibration_files": []}
    ports = sorted(set(glob.glob("/dev/serial/by-id/*") + glob.glob("/dev/ttyACM*")
                       + glob.glob("/dev/ttyUSB*")))
    for p in ports:
        out["serial_ports"].append({"port": p, "resolves_to": os.path.realpath(p),
                                    "held_by": check_bus_is_ours(p)})
    for root in calibration_dirs():
        out["calibration_files"].extend(str(f) for f in sorted(root.glob("*/*/*.json")))
    return out


def doctor_arm(make_robot: Callable[[], object], *, envelope=None, fk=None) -> dict:
    """Read-only look at an SO-10x follower: positions, torque, goal registers, power, the
    calibration match, and where FK puts the envelope points."""
    robot = make_robot()
    bus = robot.bus
    res: dict = {"calibration_file_found": bool(getattr(robot, "calibration", None)),
                 "joint_units": {m: norm_name(motor) for m, motor in bus.motors.items()}}
    bus.connect()  # opens the port and pings the servos: reads only
    try:
        present = bus.sync_read("Present_Position", normalize=False)
        goal = bus.sync_read("Goal_Position", normalize=False)
        torque = bus.sync_read("Torque_Enable", normalize=False)
        res["present_ticks"], res["goal_ticks"], res["torque_enabled"] = present, goal, torque
        res["goal_minus_present_ticks"] = {m: goal[m] - present[m] for m in present}
        holding = all(torque.get(m) == 1 and abs(goal[m] - present[m]) <= HOLD_WITHIN_TICKS
                      for m in present)
        res["be_run_connect_would"] = ("leave the goals alone: the arm is holding" if holding else
                                       "set every goal to its present position before torque")
        for reg, key, scale in (("Present_Voltage", "voltage_V", 0.1),
                                ("Present_Temperature", "temperature_C", 1.0)):
            try:
                res[key] = {m: v * scale for m, v in bus.sync_read(reg, normalize=False).items()}
            except Exception as e:  # noqa: BLE001 - optional registers; report and go on
                res[key] = f"unreadable: {type(e).__name__}"
        stored = bus.read_calibration()
        res["servo_stored_calibration"] = {m: {"homing_offset": c.homing_offset,
                                               "range_min": c.range_min, "range_max": c.range_max}
                                           for m, c in stored.items()}
        if res["calibration_file_found"]:
            res["calibration_matches_file"] = bool(bus.is_calibrated)
            deg = bus.sync_read("Present_Position")
            res["present"] = {m: round(float(v), 2) for m, v in deg.items()}
            if fk is not None and envelope is not None:
                units = {m: ("deg" if u == "DEGREES" else "percent") for m, u in res["joint_units"].items()}
                pts = fk.points({m: float(v) for m, v in deg.items()}, units)
                res["fk_points_m"] = {p: [round(c, 4) for c in xyz] for p, xyz in pts.items()}
                res["fk_clearance_m"] = {p: round(clearance(envelope, p, xyz), 4)
                                         for p, xyz in pts.items()}
    finally:
        bus.disconnect(disable_torque=False)  # closes the port; torque exactly as found
    return res


def listen(port: int, seconds: float, points: set[str] | None = None) -> dict:
    """Listen to the UDP instrument feed for a while and say what came in."""
    t0 = time.monotonic()
    feed = UDPFeed(port, lambda: time.monotonic() - t0, points=points)
    seen: dict[str, list[float]] = {}
    valid: dict[str, int] = {}
    feed.start()
    try:
        last: dict[str, float] = {}
        while time.monotonic() - t0 < seconds:
            snap = feed.snapshot()
            for p, s in snap.latest.items():
                if last.get(p) != s.t_s:
                    last[p] = s.t_s
                    seen.setdefault(p, []).append(s.t_s)
                    valid[p] = valid.get(p, 0) + int(s.valid)
            time.sleep(0.002)
        snap = feed.snapshot()
    finally:
        feed.close()
    rates = {p: (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 1 and ts[-1] > ts[0] else None
             for p, ts in seen.items()}
    return {"port": port, "seconds": seconds, "datagrams": feed.received, "rejected": feed.rejected,
            "points": {p: {"samples_seen": len(ts), "valid": valid.get(p, 0), "rate_hz": rates[p]}
                       for p, ts in seen.items()},
            "camera_moved": snap.camera_moved}
