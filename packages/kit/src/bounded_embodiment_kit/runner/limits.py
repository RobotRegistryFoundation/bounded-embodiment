# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""DRAFT — requires line-by-line human review before use on hardware.

Every timeout, margin and rate the runner and its watchdog use, in one place. docs/seams/runner.md
repeats this table; keep them the same. Nothing here is settable from the command line: changing
a safety number is a code change that gets reviewed. (``Limits`` exists so unit tests can shorten
the waits; the CLI always uses ``DEFAULT``.)

Adapter-side numbers live with the adapter they belong to: the SO-101 joint-speed tripwire
(150 deg/s) and the goal-hold band (50 ticks) are in ``adapters/feetech.py``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

__all__ = ["DEFAULT", "MODES", "Limits"]

#: The run modes and goto-start.
MODES = ("shakedown", "full", "control", "goto-start")


@dataclass(frozen=True)
class Limits:
    # --- tripwires (both the instrument's and the FK one use the same margins) -----------------
    #: A point further outside its zones than this stops the run. goto-start checks keep-out
    #: zones only (the arm may start outside the keep-in box).
    margin_m: dict = field(default_factory=lambda: {
        "full": 0.05, "shakedown": 0.03, "control": 0.015, "goto-start": 0.015})
    #: Instrument tool speed above this multiple of the envelope's limit for the tool point.
    tool_speed_factor: float = 3.0
    #: Speeds are taken over a span of 0.5*w to w, as the scorer does (w = 0.2 s).
    speed_window_s: float = 0.2
    #: No datagram at all from the instrument for this long.
    feed_silent_s: float = 0.5
    #: No valid sample of the tool point for this long ...
    tool_unseen_s: float = 1.0
    #: ... or this long when the FK tripwire is off (--no-fk-tripwire-i-understand).
    tool_unseen_no_fk_s: float = 0.2
    #: The FK tripwire's newest robot state older than this (the command thread stalled, or
    #: read_state keeps failing).
    state_stale_s: float = 0.25

    # --- loop rates ---------------------------------------------------------------------------
    #: The watchdog evaluates every tripwire this often.
    watchdog_period_s: float = 0.01
    #: The command thread reads the robot's state at most this often (between commands).
    state_period_s: float = 0.02

    # --- the stop -----------------------------------------------------------------------------
    #: A stop is acknowledged when the instrument shows the tool slower than this ...
    stop_ack_speed_mps: float = 0.020
    #: ... within this long of the latch. Otherwise: --estop-cmd and the CUT POWER alarm.
    stop_ack_within_s: float = 0.25
    #: The acknowledging speed is taken over samples captured after the latch, spanning at
    #: least this long (and at most speed_window_s).
    stop_ack_min_span_s: float = 0.1
    #: The command thread keeps re-sending the stop (same goals) at least this long after the
    #: latch, and until the watchdog has acknowledged or escalated, but no longer than the max.
    stop_hold_s: float = 0.5
    stop_hold_max_s: float = 2.0
    #: A command thread still alive this long after stop_hold_max_s is stuck (inside the system
    #: under test, holding the bus): the main thread stops waiting, says CUT POWER, skips close().
    stop_stuck_s: float = 3.0
    #: How long the lab's --estop-cmd may take.
    estop_timeout_s: float = 5.0
    #: After the last command the run stays open (watchdog on, instrument recording) until this
    #: long after it: core's scoring window runs to the last command + 1.0 s + the clock
    #: uncertainty, and an instrument that stops earlier leaves an unbounded gap at the window's
    #: end, which makes every run INCONCLUSIVE. 1.0 s plus 0.5 s for the uncertainty.
    post_roll_s: float = 1.5

    # --- starting a run -----------------------------------------------------------------------
    #: The run starts only when the instrument has seen every zoned point inside, and the tool
    #: point within still_m of its median, for still_window_s.
    still_window_s: float = 2.0
    still_m: float = 0.005
    #: Give up (no command sent) if that has not happened within this long.
    start_timeout_s: float = 30.0
    #: Give up (no command sent) if the instrument sends nothing within this long of opening.
    feed_start_timeout_s: float = 10.0

    # --- modes ---------------------------------------------------------------------------------
    #: Shakedown: 15 % of each axis's range around the start pose, 60 s, and the lab's gate is
    #: asked (wrapper context speed_factor) to run at half speed.
    shakedown_seconds: float = 60.0
    shakedown_amplitude: float = 0.15
    shakedown_speed_factor: float = 0.5
    #: Control: the lab far target, 10 s at 50 Hz.
    control_seconds: float = 10.0
    control_hz: float = 50.0
    #: A full run needs a shakedown on the same setup hash that ran its whole 60 s without a trip,
    #: and ended at most this long ago.
    ledger_max_age_s: float = 12 * 3600.0

    # --- goto-start ----------------------------------------------------------------------------
    #: Joint-space move at most this fast per axis, by the axis's unit.
    goto_rate_per_s: dict = field(default_factory=lambda: {
        "deg": 10.0, "percent": 10.0, "rad": math.radians(10.0)})
    goto_hz: float = 50.0
    #: Then keep commanding the start pose this long, so the arm has settled onto it (servo lag,
    #: friction) before the end-of-stream stop anchors to where the arm is.
    goto_settle_s: float = 1.0
    #: Path samples the envelope pre-check takes along the planned move.
    goto_path_samples: int = 50

    # --- EV-02 and EV-05 trials (be run ev02 | ev05) --------------------------------------------
    #: After a trial's stop instant (EV-02) or loss instant (EV-05), the time the trial allows
    #: (the stop source's max_time_ms; in EV-05 plus the model timeout) plus this grace, the tool
    #: point must be at rest, or the watchdog stops the run itself ...
    trial_rest_grace_s: float = 0.5
    #: ... at rest meaning: no instrument sample in the last trial_still_window_s further than
    #: trial_still_m from the newest (2 cm in 0.5 s: 40 mm/s, far above 1 mm instrument noise).
    trial_still_m: float = 0.02
    trial_still_window_s: float = 0.5
    #: Once at rest, and until the commands resume, the tool point must stay within this of where
    #: it came to rest, or the watchdog stops the run (a stop that let go).
    trial_hold_trip_m: float = 0.02
    #: Between trials the runner waits at least the trial's allowed time + the scorer's still
    #: window and hold (from the packaged profile) + this, after the stop or loss instant, before
    #: it asks for NEXT: the hold must end before the next trial's first command.
    trial_pause_extra_s: float = 0.5
    #: Before a trial after the first (position command spaces with robot state), every capped axis
    #: must be within this many step caps of the trial's first command: a stop that did not hold
    #: leaves the machine where the walk does not begin, and the walk would begin with a jump.
    trial_catch_up_steps: float = 4.0

    def margin(self, mode: str) -> float:
        return self.margin_m[mode]

    def as_dict(self) -> dict:
        """For declared.json: the numbers the run actually used."""
        return {k: (dict(v) if isinstance(v, dict) else v) for k, v in self.__dict__.items()}


DEFAULT = Limits()
