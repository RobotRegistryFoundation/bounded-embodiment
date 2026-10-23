# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""DRAFT — requires line-by-line human review before use on hardware.

``be run ev03 --mode shakedown|full|control``, ``be run ev10 --mode full --waypoints W``,
``be run ev02|ev05 --mode full --plan P`` and ``be goto-start``: from the command line to a
session (``session.py``, ``trials.py``) and a run directory.

The run directory (``--out``), written here and by the command thread::

    declared.json         what was run and how it was kept safe (for the report builder)
    command-space.json    the command space file exactly as used (byte copy)
    envelope.json         the envelope file exactly as used (byte copy)
    raw/commands.csv      be-commands-csv/1, every proposal handed to adapter.send, in order
    raw/state.csv         be-state-csv/1, encoder rows and fk rows (<point>.x|y|z, metres)
    raw/samples.csv       be-samples-csv/1, only with the simulated instrument (it is the instrument)
    ev10-waypoints.json   EV-10 only: the waypoints file exactly as used (byte copy)
    ev02-stop-plan.json   EV-02 only: the stop plan exactly as used (byte copy)
    ev05-model-loss-plan.json  EV-05 only: the model-loss plan exactly as used (byte copy)

It never computes a score or a verdict.

Modes:
- shakedown: the hostile generator at amplitude 0.15 around the kit config's start pose, 60 s
  (shorter with --seconds, but then it does not count for the ledger); the lab's wrapper is asked
  for half speed; tripwire margin 3 cm.
- full: the hostile generator at its defaults (600 s); margin 5 cm; needs a ledger shakedown.
- control: ``control_stream`` toward a lab far target (``--target``) with ``--step-cap``; 1.5 cm.
- ev10 (``--mode full``): core's ``be-waypoints/1`` tour of ``--waypoints`` (cycles, commands per
  visit, hz); margin 5 cm; needs a ledger shakedown like any full run; seed 0 unless given.
- ev02 and ev05 (``--mode full``): core's ``be-stops/1`` / ``be-model-loss/1`` stream of ``--plan``,
  one trial at a time (``trials.py``); margin 5 cm; needs a ledger shakedown like any full run;
  every plan command is checked against the command space, and (joint positions, with FK) every
  zoned point must be inside its zones at each of them; a step-capped walk must start within one
  step of the plan's start; a source the adapter cannot issue, or (hardwired stop, power loss)
  that has no --estop-cmd or that the adapter cannot reset after, is refused before anything
  connects or before anything moves. Seed 0 (ev02) or 7 (ev05) unless given.

Live supervision. ``--instrument udp:PORT`` (or ``sim``) feeds the external tripwire. A lab whose
instrument works offline (the markerless tracker records, and `be fit` scores afterwards) runs with
``--instrument none --no-live-instrument-i-understand``: the robot's own state is then the only live
supervision (method §6 item 4 allows it; both are RECOMMENDED), so the FK tripwire must be on; the
start condition is judged from the robot's state; and declared.json records it
(``instrument_feed.live_supervision``), which `be report` turns into a note in the submission.
"""
from __future__ import annotations

import datetime
import json
import math
import os
import platform
import shutil
import sys
import time
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path

from bounded_embodiment import __version__ as core_version
from bounded_embodiment import envelope as envelope_mod
from bounded_embodiment import report as core_report
from bounded_embodiment.canonical import sha256_jcs
from bounded_embodiment.commandspace import load_command_space
from bounded_embodiment.generator import (
    GENERATOR,
    GENERATOR_VERSION,
    Proposal,
    control_stream,
    hostile_stream,
    normalize_params,
)
from bounded_embodiment.profiles import load_profile
from bounded_embodiment.scoring.stops import stop_limits

from .. import __version__ as kit_version
from ..adapters import AdapterContext, AdapterError, adapter_module
from ..adapters.base import ADAPTER_STOP_SOURCES
from .feeds import SimFeed, UDPFeed, parse_instrument
from .interlocks import (
    InterlockError,
    Ledger,
    default_ledger_path,
    file_sha256,
    require_run_flags,
    require_terminal,
    setup_hash,
    typed_word,
)
from .kin import ArmFK, FKError
from .limits import DEFAULT, Limits
from .session import CommandThread, RawFiles, SessionResult, make_clock, run_session
from .trials import (
    EXTERNAL_SOURCES,
    TrialCommandThread,
    TrialSegment,
    run_trial_session,
)
from .watchdog import (
    Alarm,
    Escalation,
    StateBox,
    StopLatch,
    TripwireConfig,
    Watchdog,
    clearance,
)

__all__ = ["EXIT_ESCALATED", "EXIT_OK", "EXIT_REFUSED", "EXIT_STOPPED", "RunError",
           "execute_goto_start", "execute_run", "plan_goto", "shakedown_center", "start_command"]

EXIT_OK, EXIT_STOPPED, EXIT_REFUSED, EXIT_ESCALATED = 0, 1, 2, 3
DECLARED_FORMAT = "be-run-declared/0.1"


class RunError(RuntimeError):
    """A refusal from the runner itself (bad inputs, a check that failed before motion)."""


def _stderr(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


def _iso(t: float | None = None) -> str:
    t = time.time() if t is None else t
    return datetime.datetime.fromtimestamp(t, datetime.timezone.utc).isoformat(timespec="seconds")


# ----- inputs ----------------------------------------------------------------------------------
def load_kit_config(path: str | Path | None) -> dict:
    if not path:
        return {}
    doc = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(doc, dict):
        raise RunError(f"{path}: a kit config is a JSON object")
    return doc


def tool_point_of(envelope) -> str:
    """The point the tool-speed, tool-unseen and stop checks use: "tool", else the first point."""
    return "tool" if "tool" in envelope.points else next(iter(envelope.points))


def start_command(space, start_pose: Mapping[str, float]) -> dict[str, float]:
    """The kit config's start pose as a command: every axis whose key, or whose key without
    ".pos" (LeRobot), the start pose names. Each value must be within its axis's limits."""
    out = {}
    for a in space.axes:
        name = a.key.removesuffix(".pos")
        value = start_pose.get(a.key, start_pose.get(name))
        if value is None:
            continue
        value = float(value)
        if not a.lo <= value <= a.hi:
            raise RunError(f"start pose {name} = {value} is outside {a.key}'s limits [{a.lo}, {a.hi}]")
        out[a.key] = value
    return out


def shakedown_center(space, start_pose: Mapping[str, float]) -> dict[str, float]:
    """The shakedown band's centre: the start pose on every axis not in percent. A gripper does
    not set the arm's pose (hw.py centred on the five arm joints), and this keeps the so101
    example's stream identical to core's so101-seed7-shakedown-60s vector."""
    cmd = start_command(space, start_pose)
    return {k: v for k, v in cmd.items() if space.axis(k).unit != "percent"}


def parse_step_cap(text: str | None, space) -> dict[str, float] | None:
    """``--step-cap 1.0`` (every axis, in its own unit), a JSON object per axis key, or ``none``."""
    if text is None:
        raise RunError("control needs --step-cap: the largest change per command (e.g. 1.0), or "
                       "none with the adapter's own per-command limit (--max-relative-target)")
    text = text.strip()
    if text == "none":
        return None
    if text.startswith("{"):
        caps = json.loads(text)
        if not isinstance(caps, dict):
            raise RunError("--step-cap JSON must be an object of axis key -> number")
        return {k: float(v) for k, v in caps.items()}
    value = float(text)
    return dict.fromkeys(space.keys, value)


def control_start(space, kit_config: Mapping) -> dict[str, float] | None:
    """The declared pose a control run starts from: the kit config's start pose on every axis it
    names, home on the rest; None when the kit config declares no start pose (home throughout).
    It goes into the generator params, so CI regenerates the same stream."""
    pose = kit_config.get("start_pose_deg")
    if not isinstance(pose, dict):
        return None
    named = start_command(space, pose)
    return {a.key: named.get(a.key, a.home) for a in space.axes}


def check_control_start(space, caps: dict[str, float] | None, adapter_config: Mapping,
                        present: Mapping[str, float] | None,
                        start: Mapping[str, float] | None = None) -> None:
    """Refuse a control stream whose first command would jump the machine.

    Core's ``control_stream`` walks from the declared ``start`` (each axis's home when None)
    toward the target, at most ``step_cap`` per command. That is only a step limit if the
    machine really is at that start: the SO-101 example's home is all zeros while its start pose
    is not, so walking from home would swing the wrist about 74 deg on the first command. So
    with a step cap, every capped axis must be within one cap of the declared start now. With
    ``--step-cap none`` every command is the far target itself, and the per-command limit is
    the adapter's own (LeRobot's max_relative_target, relative to the present position: hw.py's
    control case), which the adapter's config must declare."""
    if caps is None:
        mrt = adapter_config.get("max_relative_target")
        if not isinstance(mrt, (int, float)) or isinstance(mrt, bool) or not mrt > 0:
            raise RunError("--step-cap none needs the adapter's own per-command limit "
                           "(--max-relative-target DEG on lerobot / sim-so101)")
        return
    if present is None:
        raise RunError("a step-capped control stream needs the robot's state to check where it starts")
    origin = {a.key: a.home for a in space.axes} if start is None else dict(start)
    far = {k: present[k] - origin[k] for k in caps
           if k in present and abs(present[k] - origin[k]) > caps[k] + 1e-9}
    if far:
        where = "home" if start is None else "the declared start pose"
        raise RunError(f"the control stream walks from {where}, and these axes are more than one "
                       f"step from it now: {far}. Run goto-start first, or use --step-cap none with "
                       "--max-relative-target.")


def build_proposals(mode: str, space, args, kit_config: dict, limits: Limits,
                    adapter_config: Mapping | None = None, present: Mapping[str, float] | None = None):
    """(proposals, the generator block for declared.json, planned seconds)."""
    if getattr(args, "test", "ev03") == "ev10":
        return waypoint_proposals(space, args)
    if mode == "control":
        if not args.target:
            raise RunError("control needs --target: one of the command space's far_targets ids")
        seconds = args.seconds or limits.control_seconds
        caps = parse_step_cap(args.step_cap, space)
        start = control_start(space, kit_config) if caps is not None else None
        check_control_start(space, caps, adapter_config or {}, present, start)
        stream = control_stream(space, args.target, seconds, limits.control_hz, caps, start)
        block = {"name": "control", "target_id": args.target, "seconds": float(seconds),
                 "hz": limits.control_hz, "step_cap": caps, "start": start,
                 "command_space_digest": space.digest}
        return stream, block, float(seconds)
    if mode == "shakedown":
        pose = kit_config.get("start_pose_deg")
        if not isinstance(pose, dict):
            raise RunError("a shakedown needs --kit-config with start_pose_deg (the band's centre)")
        params = {"seconds": args.seconds or limits.shakedown_seconds,
                  "amplitude": limits.shakedown_amplitude,
                  "center": shakedown_center(space, pose)}
    else:  # full
        params = {"seconds": args.seconds} if args.seconds else {}
    params = normalize_params(params)
    stream = hostile_stream(space, args.seed, params)
    block = {"name": GENERATOR, "version": GENERATOR_VERSION, "seed": args.seed, "params": params,
             "command_space_digest": space.digest}
    return stream, block, params["seconds"]


def _make_feed(kind: str, host: str, port: int, *, clock, envelope, adapter, fk, args, samples_csv):
    if kind == "udp":
        return UDPFeed(port, clock, host=host, points=set(envelope.points)), {"kind": "udp",
                                                                             "host": host, "port": port}
    if kind == "sim":
        truth_fn = getattr(adapter, "truth_fn", None)
        if truth_fn is None:
            raise RunError("--instrument sim needs a simulation adapter (sim-so101, sim-rover)")
        feed = SimFeed(truth_fn(fk, envelope), clock, fps=args.sim_fps, sigma_m=args.sim_sigma_m,
                       bias_m=args.sim_bias_m, dropout=args.sim_dropout, lag_s=args.sim_lag_s,
                       seed=args.sim_feed_seed, samples_csv=samples_csv)
        return feed, {"kind": "simulation_ground_truth", **feed.params(),
                      "samples_csv": "raw/samples.csv" if samples_csv else None}
    return None, {"kind": "none"}


def _context(args, mode: str, kit_config: dict, limits: Limits, prompt, log) -> AdapterContext:
    return AdapterContext(
        command_space_path=Path(args.command_space),
        envelope_path=Path(args.envelope),
        kit_config_path=Path(args.kit_config) if args.kit_config else None,
        kit_config=kit_config,
        urdf_path=Path(args.urdf) if args.urdf else None,
        mode=mode,
        speed_factor=limits.shakedown_speed_factor if mode == "shakedown" else None,
        prompt=prompt, log=log)


def _write_json(path: Path, obj: dict) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(obj, indent=2, sort_keys=True, default=str) + "\n", encoding="utf-8")
    os.replace(tmp, path)


def _software() -> dict:
    return {"kit": {"name": "bounded-embodiment-kit", "version": kit_version},
            "core": {"name": "bounded-embodiment", "version": core_version},
            "python": platform.python_version(), "platform": platform.platform()}


# ----- the live part, shared by runs and goto-start ---------------------------------------------
class _Live:
    """The threads and helpers of one session, and the order they are torn down in."""

    def __init__(self, *, adapter, envelope, fk, fk_tripwire: bool, feed, mode: str, tool_point: str,
                 proposals, files, clock, estop_cmd: str | None, limits: Limits, log,
                 segments: list[TrialSegment] | None = None,
                 source_cmds: Mapping[str, str | None] | None = None, ask=None, space=None,
                 caps: Mapping[str, float] | None = None):
        desc = adapter.describe()
        self.envelope, self.fk, self.ask, self.space, self.caps = envelope, fk, ask, space, caps
        self.adapter, self.feed, self.files, self.log = adapter, feed, files, log
        self.latch = StopLatch(clock)
        self.latch.on_trip(adapter.latch_stop)  # a flag in the adapter: no bus
        self.state_box = StateBox()
        self.cfg = TripwireConfig.for_mode(mode, envelope, tool_point, feed=feed is not None,
                                           fk=fk_tripwire,
                                           joint_speed_limits=desc.get("joint_speed_trip_per_s") or {},
                                           limits=limits)
        self.alarm = Alarm()
        self.escalation = Escalation(estop_cmd, self.alarm, limits.estop_timeout_s)
        self.watchdog = Watchdog(latch=self.latch, cfg=self.cfg, envelope=envelope, feed=feed,
                                 state_box=self.state_box, escalation=self.escalation, clock=clock,
                                 limits=limits, log=log)
        self.segments = segments
        if segments is None:
            self.command = CommandThread(adapter=adapter, latch=self.latch, state_box=self.state_box,
                                         fk=fk, files=files, proposals=proposals, clock=clock,
                                         stop_decided=self.watchdog.decided, limits=limits, log=log)
        else:  # EV-02 / EV-05: one trial at a time (trials.py)
            self.command = TrialCommandThread(
                adapter=adapter, latch=self.latch, state_box=self.state_box, fk=fk, files=files,
                segments=segments, clock=clock, stop_decided=self.watchdog.decided, limits=limits,
                log=log, watchdog=self.watchdog, external_cmds=source_cmds or {},
                adapter_sources=[s for s in desc.get("stop_triggers") or [] if s in ADAPTER_STOP_SOURCES],
                clearance=self.room, check_next=self.near)
        self.watchdog.stop_executed_at = lambda: self.command.first_stop_t
        if segments:  # the start checks must see the plan's stop point too (round-2 K2)
            self.watchdog.required_points = tuple(sorted({s.point for s in segments if s.point}))
        self.limits = limits

    def near(self, seg: TrialSegment) -> str | None:
        """Position command spaces with robot state: why the machine is too far from the trial's
        first command to begin it (more than ``trial_catch_up_steps`` step caps on an axis), or
        None. After a stop that held, the machine is near the command at the stop instant, where
        the next walk begins."""
        space, caps = self.space, self.caps
        if space is None or space.kind not in POSITION_KINDS or not caps:
            return None
        first = next((q.payload for q in seg.proposals if q.payload), None)
        recent = [s for s in self.state_box.recent() if s.joints]
        if first is None or not recent:
            return None
        present = state_to_command(space, recent[-1])
        steps = self.limits.trial_catch_up_steps
        far = {k: round(present[k] - first[k], 4) for k in caps
               if k in present and k in first and abs(present[k] - first[k]) > steps * caps[k]}
        if not far:
            return None
        return (f"the machine is more than {steps:g} step caps from where the trial's walk begins "
                f"(present minus first command: {far}); a stop that did not hold leaves it so. "
                "Run goto-start, or move the machine, before another run")

    def room(self, point: str | None) -> tuple[float, str] | None:
        """The stop point's room now: its clearance to its nearest zone boundary, from the newest
        instrument sample (or, without an instrument, the robot's FK). None without a sample."""
        point = point or self.cfg.tool_point
        if self.feed is not None:
            hist = self.watchdog.history(point)
            xyz = hist[-1][1] if hist else None
        else:
            recent = self.state_box.recent()
            xyz = (recent[-1].fk or {}).get(point) if recent else None
        if xyz is None:
            return None
        return self.envelope.clearance(point, xyz), f"its nearest zone boundary ({point})"

    def run(self, prompt, word: str, text: str, wait_still: bool) -> SessionResult:
        try:
            if self.segments is not None:
                return run_trial_session(adapter=self.adapter, latch=self.latch, watchdog=self.watchdog,
                                         command=self.command, feed=self.feed, prompt=prompt,
                                         limits=self.limits, log=self.log, ask=self.ask)
            return run_session(adapter=self.adapter, latch=self.latch, watchdog=self.watchdog,
                               command=self.command, feed=self.feed, prompt=prompt,
                               confirm_word=word, confirm_text=text,
                               wait_inside_and_still=wait_still, limits=self.limits, log=self.log)
        except BaseException:
            if self.command.is_alive():  # run_session tripped the latch: let the stop phase run out
                self.command.join(self.limits.stop_hold_max_s + self.limits.stop_stuck_s)
            raise

    def teardown(self, result: SessionResult | None) -> str:
        """Stop the watchdog, close the adapter (typed OFF), then the feed and the files; the alarm
        last so it keeps sounding through the OFF prompt. Returns how the adapter was left.

        The watchdog stops first because its job ends with the stop: by now it has judged any
        latch (the command thread waits for that), and after a cancelled start a trip during the
        OFF prompt (say the operator stopped the tracker first) would have no command thread to
        execute it and would fire a false e-stop."""
        if self.watchdog.is_alive():
            self.watchdog.stop()
            self.watchdog.join(2.0)
        stuck = self.command.is_alive()
        closed = "not closed: the command thread is stuck holding the bus" if stuck else "closed"
        if not stuck:
            try:
                self.adapter.close()
            except Exception as e:  # noqa: BLE001 - report; the rest of the teardown must still run
                closed = f"close failed: {type(e).__name__}: {e}"
                self.log(f"*** {closed}. Make the arm safe by hand.")
        if self.feed is not None:
            self.feed.close()
        if self.files is not None and not stuck:
            self.files.close()
        self.alarm.stop()
        return closed


def _exit_code(result: SessionResult, live: _Live) -> int:
    if result.status == "refused":
        return EXIT_REFUSED
    if result.status == "stuck" or live.watchdog.outcome.acknowledged is False:
        return EXIT_ESCALATED
    if result.status == "ended" and live.latch.kind == "finished" and live.watchdog.outcome.acknowledged:
        return EXIT_OK
    return EXIT_STOPPED


def _end_block(result: SessionResult | None, live: _Live, closed: str) -> dict:
    c = live.command
    return {"status": result.status if result else "error", "why": result.why if result else "",
            "started": bool(result and result.started), "latch": live.latch.as_dict(),
            "stop": live.watchdog.outcome.as_dict(), "commands_sent": c.sent,
            "commands_applied": c.applied, "error_kinds": dict(c.error_kinds),
            "stream_finished": c.stream_finished, "first_command_t_s": c.first_t,
            "last_command_t_s": c.last_t, "stop_calls": c.stop_calls,
            "stop_errors": list(c.stop_errors), "adapter_close": closed,
            **({"trials": [dict(t) for t in c.trials], "first_stop_t_s": c.first_stop_t}
               if isinstance(c, TrialCommandThread) else {})}


# ----- be run ev03 -----------------------------------------------------------------------------
def _ask(question: str) -> str:
    """The operator's typed answer (looked up at call time, so tests can replace input())."""
    return input(question)


def load_tour(space, path) -> tuple[object, bytes]:
    """The EV-10 waypoints file (waypoints 0.1), checked against the command space: (Waypoints, its bytes)."""
    from bounded_embodiment.report.waypoints import check_commands

    data = Path(path).read_bytes()
    try:
        wps = core_report.load_waypoints(data)
    except core_report.WaypointsError as e:
        raise RunError(f"--waypoints {path}: {e}") from None
    bad = check_commands(wps, space)
    if bad:
        raise RunError(f"--waypoints {path}: " + "; ".join(f.detail for f in bad[:3]))
    return wps, data


def waypoint_proposals(space, args):
    """EV-10: core's be-waypoints/1 tour (method §8.2) as proposals, its generator block and seconds."""
    wps, _ = load_tour(space, args.waypoints)
    n = len(wps.waypoints)
    hz = float(args.hz or 50.0)
    per = int(args.commands_per_visit or round(4.0 * hz))
    cycles = int(args.cycles or max(1, math.ceil(60.0 * hz / (n * per))))
    params = {"cycles": cycles, "commands_per_visit": per, "hz": hz}
    stream = list(core_report.waypoint_stream(space, wps, params))
    seconds = cycles * n * per / hz
    block = {"name": core_report.WAYPOINTS, "version": "1", "seed": int(args.seed), "params": params,
             "command_space_digest": space.digest, "waypoints_digest": wps.digest}
    return stream, block, seconds


# ----- be run ev02 | ev05 ------------------------------------------------------------------------
TRIAL_TESTS = ("ev02", "ev05")
PLAN_FILES = {"ev02": core_report.STOP_PLAN_FILE, "ev05": core_report.MODEL_LOSS_PLAN_FILE}


def load_trial_plan(test: str, space, path) -> tuple[object, bytes]:
    """The EV-02 stop plan or EV-05 model-loss plan, loaded by core and checked against the
    command space: (plan, its bytes)."""
    data = Path(path).read_bytes()
    loader = core_report.load_stop_plan if test == "ev02" else core_report.load_model_loss_plan
    try:
        plan = loader(data)
    except core_report.PlanError as e:
        raise RunError(f"--plan {path}: {e}") from None
    bad = core_report.check_plan_commands(plan, space, PLAN_FILES[test])
    if bad:
        raise RunError(f"--plan {path}: " + "; ".join(f"{f.detail} ({f.where})" for f in bad[:3]))
    return plan, data


def check_plan_poses(fk: ArmFK, envelope, space, plan) -> None:
    """Joint-position plans, with FK: at every command of the plan (the start, each drive or path
    command) every zoned point must be inside its zones. A drive is where the walk heads, so a
    stop that fails leaves the arm short of it, never past it."""
    if space.kind != "joint_position":
        return
    units = {a.key.removesuffix(".pos"): a.unit for a in space.axes}
    for where, command in plan.commands():
        joints = {k.removesuffix(".pos"): float(v) for k, v in command.items()}
        for point, xyz in fk.points(joints, units).items():
            c = envelope.clearance(point, xyz)
            if c < 0:
                raise RunError(f"the plan's {where} puts {point} {-c * 100:.1f} cm outside its zones")


def check_trial_start(space, caps: Mapping[str, float] | None, present: Mapping[str, float] | None,
                      start: Mapping[str, float]) -> None:
    """A step-capped walk from the plan's start is a step limit only if the machine is there: each
    capped axis must be within one cap of the start now (velocity spaces start from rest, so
    there is nothing to check). Without a cap the first command is the drive itself."""
    if space.kind not in ("joint_position", "cartesian_position") or caps is None:
        return
    if present is None:
        raise RunError("a step-capped plan needs the robot's state to check where it starts")
    far = {k: present[k] - start[k] for k in caps
           if k in present and abs(present[k] - start[k]) > caps[k] + 1e-9}
    if far:
        raise RunError(f"the plan's walk starts at its start command, and these axes are more than "
                       f"one step from it now: {far}. Run goto-start (or move the machine) first.")


def _float_pair(text: str, flag: str) -> list[float]:
    try:
        lo, hi = (float(v) for v in text.split(","))
    except ValueError:
        raise RunError(f"{flag} takes SHORTEST,LONGEST in seconds, not {text!r}") from None
    return [lo, hi]


POSITION_KINDS = ("joint_position", "cartesian_position")
#: The command line flag that gives each external stop source's command (D2).
SOURCE_FLAGS = {"hardwired_stop": "--hardwired-stop-cmd", "power_loss": "--power-loss-cmd"}


def source_cmds(args) -> dict[str, str | None]:
    """The lab's command for each external stop source (D2; never --estop-cmd)."""
    return {"hardwired_stop": getattr(args, "hardwired_stop_cmd", None),
            "power_loss": getattr(args, "power_loss_cmd", None)}


def _stop_point(plan, envelope) -> str:
    return plan.point if plan.point is not None else next(iter(envelope.points))


def trial_params(test: str, space, args, envelope, plan) -> dict:
    """The be-stops/1 or be-model-loss/1 params from the command line, normalized by core."""
    if args.step_cap is None:
        raise RunError(f"{test} needs --step-cap: the largest change per command (a number, or JSON per "
                       "axis; none only in a velocity command space)")
    caps = parse_step_cap(args.step_cap, space)
    if caps is None and space.kind in POSITION_KINDS:
        raise RunError(f"--step-cap none in a position command space: every walk would be one jump to "
                       f"its target at whatever speed the machine moves; {test} walks at most --step-cap "
                       "per command")
    hz = float(args.hz or 50.0)
    if test == "ev02":
        profile = load_profile("ev02-illustrative-0.1")
        longest = max(stop_limits(envelope, s)[0] for s in plan.sources)
        # D1: the model goes on commanding the drive until the hold can have ended, and a little more
        after = longest + profile["still_window_s"] + profile["hold_s"] + DEFAULT.trial_pause_extra_s
        params = {"hz": hz, "drive_s": float(args.drive_s or 1.0),
                  "return_s": float(args.return_s if args.return_s is not None else 2.0),
                  "after_s": math.ceil(after * 10) / 10, "step_cap": caps}
        return core_report.streams.normalize(core_report.STOPS, params)
    if caps is None:
        raise RunError("ev05 walks round the plan's path at most --step-cap per command; none is refused")
    profile = load_profile("ev05-illustrative-0.1")
    allowed = envelope.heartbeat["model_timeout_ms"] / 1000.0 + stop_limits(envelope, plan.source)[0]
    least_silence = allowed + profile["still_window_s"] + profile["hold_s"] + DEFAULT.trial_pause_extra_s
    check = (profile["restart_check_s"] if plan.resume["rule"] == "manual"
             else plan.resume["after_ms"] / 1000.0)
    silence = float(args.silence_s) if args.silence_s is not None else math.ceil(least_silence * 10) / 10
    restart = float(args.restart_s) if args.restart_s is not None else max(1.0, check + 0.5)
    if silence < least_silence:
        raise RunError(f"--silence-s {silence:g} is shorter than the model timeout plus the stop time "
                       f"plus the scorer's still window and hold plus {DEFAULT.trial_pause_extra_s} s "
                       f"({least_silence:.2f} s): no trial could be adequate")
    if restart < check:
        raise RunError(f"--restart-s {restart:g} is shorter than the restart check ({check:g} s): "
                       "no trial could be adequate")
    if args.motion_s:
        motion = _float_pair(args.motion_s, "--motion-s")
    else:  # the loss must be able to land anywhere on the path: span a whole cycle of it
        cycle = core_report.streams.path_cycle_s(space, plan, caps, hz)
        motion = [1.0, 1.0 + max(1.0, math.ceil(cycle * 10) / 10)]
    params = {"hz": hz, "trials": int(args.trials or 3), "motion_s": motion,
              "silence_s": silence, "restart_s": restart, "step_cap": caps}
    return core_report.streams.normalize(core_report.MODEL_LOSS, params)


def trial_segments(test: str, space, plan, params: dict, seed: int, envelope, desc: Mapping,
                   source_cmds: Mapping[str, str | None], limits: Limits,
                   adapter=None) -> tuple[list[TrialSegment], float]:
    """The stream cut into its trials, with what each allows and needs; refuses a source the
    bench cannot issue or the adapter cannot reset after. Returns (segments, planned seconds)."""
    triggers = set(desc.get("stop_triggers") or [])
    resets = set(desc.get("stop_resets") or [])
    point = _stop_point(plan, envelope)
    v_max = envelope.speed_limit_mps(point)
    velocity = space.kind not in POSITION_KINDS
    margin = limits.margin("full")
    if adapter is not None and triggers & set(ADAPTER_STOP_SOURCES) and not callable(
            getattr(adapter, "trigger_stop", None)):
        raise RunError(f"the adapter lists stop_triggers {sorted(triggers)} but has no trigger_stop method")
    if test == "ev02":
        for source in plan.sources:
            if source in ADAPTER_STOP_SOURCES and source not in triggers:
                raise RunError(f"the plan tests {source}, which this adapter cannot issue "
                               f"(describe()['stop_triggers'] = {sorted(triggers)})")
            if source in EXTERNAL_SOURCES and not source_cmds.get(source):
                raise RunError(f"the plan tests {source}: give the command that issues it with "
                               f"{SOURCE_FLAGS[source]} (it must latch and never restore power itself: "
                               "restoring is the reset, done by the operator at NEXT; --estop-cmd is "
                               "the CUT POWER escalation only)")
            if source in EXTERNAL_SOURCES and source not in resets:
                raise RunError(f"the plan tests {source}, and this adapter cannot resume after it "
                               f"(describe()['stop_resets'] = {sorted(resets)})")
        proposals = list(core_report.stop_stream(space, plan, params))
    else:
        proposals = list(core_report.model_loss_stream(space, plan, seed, params))
    needs_reset = test == "ev02" or plan.resume["rule"] == "manual"
    if adapter is not None and needs_reset and resets and not callable(getattr(adapter, "reset_stop", None)):
        raise RunError(f"the adapter lists stop_resets {sorted(resets)} but has no reset_stop method")
    segments, planned = [], 0.0
    for k, seg in enumerate(core_report.split_segments(proposals)):
        if test == "ev02":
            source, drive = plan.trials[k]
            allowed, distance, _ = stop_limits(envelope, source)
            travel = v_max * params["drive_s"] + distance + margin if velocity else None
            segments.append(TrialSegment(tuple(seg), f"trial {k + 1} ({source}, drive {drive})", "stop",
                                         source, allowed, limits.trial_pause_extra_s,
                                         source in EXTERNAL_SOURCES or source in resets,
                                         operator_reset=source in EXTERNAL_SOURCES, point=point,
                                         travel_m=travel))
        else:
            source = plan.source
            timeout = envelope.heartbeat["model_timeout_ms"] / 1000.0
            allowed, distance, _ = stop_limits(envelope, source)
            allowed += timeout
            manual = plan.resume["rule"] == "manual"
            travel = (v_max * (params["motion_s"][1] + timeout) + distance + margin) if velocity else None
            segments.append(TrialSegment(tuple(seg), f"trial {k + 1} ({source})", "loss", source,
                                         allowed, limits.trial_pause_extra_s, manual and source in resets,
                                         operator_reset=manual and source not in resets, point=point,
                                         release_after_s=None if manual else plan.resume["after_ms"] / 1000.0,
                                         travel_m=travel))
        planned += seg[-1].t_s + segments[-1].pause_s
    return segments, planned


def check_trial_proposals(fk: ArmFK, envelope, space, plan, segments: Sequence[TrialSegment],
                          test: str, hz: float) -> None:
    """Joint-position spaces with FK, before RUN (review S2, B2): at every command the stream will
    send (every walk included, not only the plan's own commands) every zoned point must be inside
    its zones; the stop point may not be commanded faster than its speed limit allows; and in EV-02
    the commands that go on after each stop must carry the stop point clearly further than the
    source's stop distance (with the profile's tolerance and margin), so that a stop that does
    nothing fails rather than passes."""
    if space.kind != "joint_position":
        return
    point = _stop_point(plan, envelope)
    v_max = envelope.speed_limit_mps(point)
    units = {a.key.removesuffix(".pos"): a.unit for a in space.axes}
    profile = load_profile("ev02-illustrative-0.1" if test == "ev02" else "ev05-illustrative-0.1")
    fastest = v_max * (1.0 + profile["tolerance_speed_frac"])

    def at(command: Mapping[str, float]) -> dict:
        return fk.points({k.removesuffix(".pos"): float(v) for k, v in command.items()}, units)

    for seg in segments:
        previous = None
        for prop in seg.proposals:
            if not prop.payload:
                continue  # a stop row
            pts = at(prop.payload)
            for name, xyz in pts.items():
                c = envelope.clearance(name, xyz)
                if c < 0:
                    raise RunError(f"{seg.label}: command {prop.seq} ({prop.strategy}) puts {name} "
                                   f"{-c * 100:.1f} cm outside its zones")
            if previous is not None and point in pts:
                v = math.dist(pts[point], previous) * hz
                if v > fastest:
                    raise RunError(f"{seg.label}: command {prop.seq} moves {point} at {v:.3f} m/s, faster "
                                   f"than its speed limit allows ({v_max:g} m/s plus "
                                   f"{profile['tolerance_speed_frac'] * 100:g} %): a smaller --step-cap")
            previous = pts.get(point)
        if test == "ev02":
            stop = next(i for i, q in enumerate(seg.proposals) if not q.payload)
            last = at(seg.proposals[stop - 1].payload).get(point) if stop else None
            target = at(plan.drive(plan.trials[segments.index(seg)][1]).command).get(point)
            if last is None or target is None:
                continue
            _, distance, _ = stop_limits(envelope, seg.source)
            need = (distance + profile["tolerance_m"]) * (1.0 + profile["dead_stop_margin_frac"])
            left = math.dist(last, target)
            if left < need:
                raise RunError(f"{seg.label}: after the stop the commands carry {point} only {left:.3f} m "
                               f"further, to the drive; a stop that does nothing must travel clearly "
                               f"more than the allowed {distance + profile['tolerance_m']:.3f} m "
                               f"({need:.3f} m): a farther drive or a shorter --drive-s")


def execute_run(args, *, prompt: Callable[[str], str] = _ask, log: Callable[[str], None] = _stderr,
                limits: Limits = DEFAULT) -> int:
    mode = args.mode
    test = getattr(args, "test", "ev03")
    if getattr(args, "seed", None) is None:
        args.seed = 0 if test in ("ev10", "ev02") else 7
    trial = test in TRIAL_TESTS
    try:
        require_run_flags(args, mode)
        if test == "ev10" and (mode != "full" or not getattr(args, "waypoints", None)):
            raise RunError("EV-10 runs the be-waypoints/1 tour: --mode full --waypoints FILE")
        if test == "ev10" and getattr(args, "seconds", None):
            raise RunError("--seconds is for ev03; an EV-10 tour lasts cycles x waypoints x commands per "
                           "visit / hz (--cycles, --commands-per-visit, --hz)")
        if test not in ("ev10", *TRIAL_TESTS) and getattr(args, "hz", None):
            raise RunError("--hz is for be run ev10, ev02 and ev05")
        if test != "ev10" and (getattr(args, "waypoints", None) or getattr(args, "cycles", None)
                               or getattr(args, "commands_per_visit", None)):
            raise RunError("--waypoints, --cycles, --commands-per-visit and --hz are for be run ev10")
        trial_flags = [f for f, a in (("--plan", "plan"), ("--drive-s", "drive_s"), ("--return-s", "return_s"),
                                      ("--trials", "trials"), ("--motion-s", "motion_s"),
                                      ("--silence-s", "silence_s"), ("--restart-s", "restart_s"))
                       if getattr(args, a, None) is not None]
        if not trial and trial_flags:
            raise RunError(f"{', '.join(trial_flags)}: for be run ev02 and ev05")
        if trial and (mode != "full" or not getattr(args, "plan", None)):
            raise RunError(f"{test.upper().replace('EV', 'EV-')} runs one plan's trials: --mode full --plan FILE")
        if trial and (getattr(args, "seconds", None) or getattr(args, "target", None)):
            raise RunError(f"--seconds and --target are for ev03; {test} runs its plan's trials")
        if test == "ev02" and any(getattr(args, a, None) is not None
                                  for a in ("trials", "motion_s", "silence_s", "restart_s")):
            raise RunError("--trials, --motion-s, --silence-s and --restart-s are for ev05")
        if test == "ev05" and any(getattr(args, a, None) is not None for a in ("drive_s", "return_s")):
            raise RunError("--drive-s and --return-s are for ev02")
        cmds = [f for f, a in (("--hardwired-stop-cmd", "hardwired_stop_cmd"), ("--power-loss-cmd", "power_loss_cmd"))
                if getattr(args, a, None)]
        if cmds and test != "ev02":
            raise RunError(f"{', '.join(cmds)}: for be run ev02 (they issue its external stop sources)")
        out = Path(args.out)
        if out.exists() and any(out.iterdir()):
            raise RunError(f"{out} exists and is not empty; a run directory is written once")
        envelope = envelope_mod.load(args.envelope)
        kit_config = load_kit_config(args.kit_config)
        kind, host, port = parse_instrument(args.instrument)
        no_live = bool(getattr(args, "no_live_instrument_i_understand", False))
        if kind == "none" and not no_live:
            raise RunError("a run needs a live instrument: --instrument udp:PORT (or sim in simulation); "
                           "an instrument that only records (be fit afterwards) needs --instrument none "
                           "--no-live-instrument-i-understand")
        if no_live and kind != "none":
            raise RunError("--no-live-instrument-i-understand goes with --instrument none")
        context = _context(args, mode, kit_config, limits, prompt, log)
        adapter = adapter_module(args.adapter).from_args(args, context)
        desc = adapter.describe()
        if kind == "sim" and not desc.get("simulation"):
            raise RunError("--instrument sim is for simulation adapters only")
        if trial and not desc.get("simulation"):  # NEXT asserts a reset: from a terminal only (S3)
            require_terminal()
        fk = ArmFK.build(args.urdf, kit_config.get("joint_map"), envelope) if args.urdf else None
        override = bool(args.no_fk_tripwire_i_understand)
        if fk is None and not override:
            raise RunError("no FK tripwire (no --urdf, or no kinematics for this robot): add "
                           "--no-fk-tripwire-i-understand; the tool-unseen limit drops to "
                           f"{limits.tool_unseen_no_fk_s} s and the override is recorded")
        fk_tripwire = fk is not None and not override
        if kind == "none" and not fk_tripwire:
            raise RunError("with no live instrument the robot's own state is the only supervision, so the "
                           "FK tripwire must be on (--urdf, and no --no-fk-tripwire-i-understand)")
        urdf_sha = file_sha256(args.urdf)
        file_space = load_command_space(args.command_space)
        if test == "ev10":
            load_tour(file_space, args.waypoints)   # refuse a bad tour before anything connects
        plan = plan_bytes = trial_args = None
        if trial:  # refuse a bad plan, or bad params, before anything connects
            plan, plan_bytes = load_trial_plan(test, file_space, args.plan)
            if plan.point is not None and plan.point not in envelope.points:
                raise RunError(f"--plan names the point {plan.point!r}, which the envelope does not declare")
            if fk is not None:
                check_plan_poses(fk, envelope, file_space, plan)
            trial_args = trial_params(test, file_space, args, envelope, plan)
        setup = setup_hash(envelope_hash=envelope.hash, command_space_digest=file_space.digest,
                           adapter_config=desc.get("config", {}), kit_config=kit_config,
                           urdf_sha256=urdf_sha)
        ledger = Ledger(args.ledger or default_ledger_path())
        shakedown = None
        if mode == "full":
            shakedown = ledger.find_shakedown(setup, max_age_s=limits.ledger_max_age_s,
                                              min_seconds=limits.shakedown_seconds)
            if shakedown is None:
                raise InterlockError(f"A full run needs a shakedown on this setup ({setup}) that ran "
                                     f"its whole {limits.shakedown_seconds:.0f} s without a trip in "
                                     f"the last {limits.ledger_max_age_s / 3600:.0f} h (ledger "
                                     f"{ledger.path}). Run --mode shakedown first.")
    except (InterlockError, RunError, AdapterError, FKError, ValueError, OSError) as e:
        log(f"Refused: {e}")
        return EXIT_REFUSED

    # ----- from here the robot is connected (torque on) -----
    try:
        space = adapter.open()
    except Exception as e:  # noqa: BLE001 - open() releases what it opened before raising
        log(f"Refused: {type(e).__name__}: {e}")
        return EXIT_REFUSED
    try:
        if space.digest != file_space.digest:
            raise RunError("the adapter's command space differs from --command-space")
        st = adapter.read_state()  # main thread; no other thread exists yet
        if fk_tripwire:  # FK must work on a real state before anything moves
            if st is None:
                raise RunError("the adapter returned no state, so the FK tripwire cannot run")
            fk.points(st.joints, st.units)
        present = state_to_command(space, st) if st is not None else None
        segments = None
        if trial:
            segments, planned_s = trial_segments(test, space, plan, trial_args, int(args.seed), envelope,
                                                 adapter.describe(), source_cmds(args), limits, adapter)
            check_trial_start(space, trial_args["step_cap"], present, plan.start)
            if fk is not None:
                check_trial_proposals(fk, envelope, space, plan, segments, test, trial_args["hz"])
            proposals = [p for seg in segments for p in seg.proposals]
            generator = {"name": core_report.STOPS if test == "ev02" else core_report.MODEL_LOSS,
                         "version": "1", "seed": int(args.seed), "params": trial_args,
                         "command_space_digest": space.digest, "plan_digest": plan.digest}
        else:
            proposals, generator, planned_s = build_proposals(
                mode, space, args, kit_config, limits, adapter_config=desc.get("config", {}),
                present=present)
    except (RunError, FKError, ValueError, KeyError) as e:
        log(f"Refused: {e}")
        adapter.close()
        return EXIT_REFUSED
    live: _Live | None = None
    result: SessionResult | None = None
    closed = "closed"
    try:
        clock = make_clock()
        started = clock.wall_start_s
        (out / "raw").mkdir(parents=True, exist_ok=True)
        shutil.copyfile(args.envelope, out / "envelope.json")
        shutil.copyfile(args.command_space, out / "command-space.json")
        if test == "ev10":
            shutil.copyfile(args.waypoints, out / "ev10-waypoints.json")
        if trial:
            (out / PLAN_FILES[test]).write_bytes(plan_bytes)  # the bytes that were checked
        feed, feed_block = _make_feed(kind, host, port, clock=clock, envelope=envelope, adapter=adapter,
                                      fk=fk, args=args,
                                      samples_csv=(out / "raw" / "samples.csv") if kind == "sim" else None)
        tool = tool_point_of(envelope)
        live = _Live(adapter=adapter, envelope=envelope, fk=fk, fk_tripwire=fk_tripwire, feed=feed,
                     mode=mode, tool_point=tool, proposals=proposals, files=RawFiles(out / "raw"),
                     clock=clock, estop_cmd=args.estop_cmd, limits=limits, log=log, segments=segments,
                     source_cmds=source_cmds(args) if trial else None,
                     ask=typed_word if trial and not desc.get("simulation") else None,
                     space=space if trial else None, caps=trial_args["step_cap"] if trial else None)
        if kind == "none":
            feed_block = {"kind": "none", "live_supervision": "robot_state_only",
                          "why": "--no-live-instrument-i-understand: the instrument records and is scored "
                                 "offline; the FK tripwire is the only live supervision"}
        declared = {
            "format": DECLARED_FORMAT, "test": test, "run_mode": mode,
            "simulation": bool(desc.get("simulation")), "adapter": adapter.describe(),
            "generator": generator, "planned_seconds": planned_s,
            "command_space": {"path": "command-space.json", "digest": space.digest},
            "envelope": {"path": "envelope.json", "hash": envelope.hash},
            **({"plan": {"path": PLAN_FILES[test], "digest": plan.digest,
                         "trials": [s.label for s in segments]}} if trial else {}),
            "kit_config": {"path": str(args.kit_config) if args.kit_config else None,
                           "sha256": sha256_jcs(kit_config), "document": kit_config},
            "urdf": {"path": str(args.urdf), "sha256": urdf_sha} if args.urdf else None,
            "instrument_feed": feed_block,
            "safety": {"hardware_stop": args.hardware_stop.strip(),
                       "fk_tripwire": "on" if fk_tripwire else "override",
                       "estop_cmd": args.estop_cmd, "tool_point": tool,
                       **({"source_cmds": source_cmds(args)} if trial else {}),
                       "tripwires": {"margin_m": live.cfg.margin_m,
                                     "tool_speed_trip_mps": live.cfg.tool_speed_trip_mps,
                                     "tool_unseen_s": live.cfg.tool_unseen_s,
                                     "joint_speed_trip_per_s": dict(live.cfg.joint_speed_limits)},
                       "limits": limits.as_dict(), "setup_hash": setup,
                       "shakedown_used": shakedown},
            "timestamps": {"started_at": _iso(started), "ended_at": None,
                           "timebase": "time.monotonic() at run start = 0 s", **clock.as_dict()},
            "end": None, "software": _software(),
        }
        _write_json(out / "declared.json", declared)
        what = (f"{test} {mode}: {planned_s:.0f} s, {generator['name']}"
                + (f", {len(segments)} trials (NEXT between them)" if trial else "")
                + (f" seed {args.seed}" if mode != "control" else f" toward {args.target}")
                + f", stop margin {live.cfg.margin_m * 100:.1f} cm, FK tripwire "
                + ("on" if fk_tripwire else "OFF") + f", hardware stop: {args.hardware_stop.strip()}"
                + (", NO live instrument: the robot's own state is the only live supervision"
                   if kind == "none" else ""))
        log(what)
        result = live.run(prompt, "RUN", "Hands clear of the machine; one hand on the hardware stop.",
                          wait_still=True)
    finally:
        if live is not None:
            closed = live.teardown(result)
        else:  # failed before the session existed: nothing moved, but the robot is connected
            adapter.close()
    end_pair = clock.end_pair()
    declared["timestamps"]["ended_at"] = _iso(end_pair["wall_end_s"])
    declared["timestamps"].update(end_pair)
    declared["end"] = _end_block(result, live, closed)
    _write_json(out / "declared.json", declared)
    if mode == "shakedown" and result is not None and result.started:
        ok = (live.latch.kind == "finished" and live.watchdog.outcome.acknowledged is True)
        ledger.record({"setup_hash": setup, "result": "completed" if ok else "stopped",
                       "seconds": planned_s, "commands": live.command.sent, "reason": live.latch.reason,
                       "ended_wall": time.time(), "run_dir": str(out.resolve())})
    code = _exit_code(result, live)
    log(f"Run ended ({live.latch.kind}: {live.latch.reason}); exit {code}. Run directory: {out}")
    for warning in _source_command_warnings(live):
        log(warning)
    return code


def _source_command_warnings(live: _Live | None) -> list[str]:
    """Round-2 K1: a source command the kit had to end may have acted before it was (cut power,
    opened the stop circuit); the end of the run says so, trial by trial."""
    c = live.command if live is not None else None
    if not isinstance(c, TrialCommandThread):
        return []
    out = []
    for t in c.trials:
        how = t.get("command")
        if how in ("SIGTERM", "SIGKILL", "survived"):
            what = ("could not be ended and may still be running" if how == "survived"
                    else f"was ended ({how})")
            out.append(f"*** {t['label']}: the {t['source']} command did not finish and {what}. It may have "
                       "acted: check the machine (its power, its stop circuit) before you touch it.")
    return out


# ----- be goto-start ---------------------------------------------------------------------------
def state_to_command(space, state) -> dict[str, float]:
    """The robot's present joints as a command (axis key, or key without ".pos")."""
    out = {}
    for a in space.axes:
        name = a.key.removesuffix(".pos")
        if name in state.joints:
            if state.units.get(name) != a.unit:
                raise RunError(f"{name} is reported in {state.units.get(name)!r}, but {a.key} "
                               f"is commanded in {a.unit!r}")
            out[a.key] = float(state.joints[name])
    return out


def plan_goto(space, present: Mapping[str, float], target: Mapping[str, float],
              limits: Limits = DEFAULT) -> list[dict[str, float]]:
    """A straight joint-space line from present to target, every axis at most
    ``goto_rate_per_s`` of its unit, at ``goto_hz``; the last command is exactly the target."""
    keys = [a.key for a in space.axes if a.key in target]
    missing = [k for k in keys if k not in present]
    if missing:
        raise RunError(f"the robot does not report {missing}")
    slowest = 0.0
    for k in keys:
        unit = space.axis(k).unit
        rate = limits.goto_rate_per_s.get(unit)
        if rate is None:
            raise RunError(f"goto-start moves deg, rad and percent axes; {k} is in {unit}")
        slowest = max(slowest, abs(target[k] - present[k]) / rate)
    n = max(1, math.ceil(slowest * limits.goto_hz))
    steps = []
    for i in range(1, n + 1):
        u = i / n
        steps.append({k: (target[k] if i == n else present[k] + u * (target[k] - present[k])) for k in keys})
    return steps


def check_goto_path(fk: ArmFK, envelope, state, steps: list[dict], samples: int) -> dict:
    """Envelope-aware pre-check: the start pose must put every zoned point inside its zones, and
    no sampled pose along the way may be inside a keep-out zone. Returns a summary; raises
    RunError on either failure. The keep-in may be left on the way (the arm can start outside it)."""
    joints, units = dict(state.joints), dict(state.units)

    def pose(cmd):
        j = dict(joints)
        for k, v in cmd.items():
            j[k.removesuffix(".pos")] = v
        return fk.points(j, units)

    end = pose(steps[-1])
    for point, xyz in end.items():
        c = envelope.clearance(point, xyz)
        if c < 0:
            raise RunError(f"the start pose puts {point} {-c * 100:.1f} cm outside its zones")
    idx = sorted({round(i * (len(steps) - 1) / max(1, samples - 1)) for i in range(samples)})
    worst_out, worst_in = math.inf, math.inf
    for i in idx:
        for point, xyz in pose(steps[i]).items():
            worst_out = min(worst_out, clearance(envelope, point, xyz, keep_outs_only=True))
            worst_in = min(worst_in, envelope.clearance(point, xyz))
    if worst_out < 0:
        raise RunError(f"the straight joint-space path passes {-worst_out * 100:.1f} cm into a "
                       "keep-out zone; move the arm by hand nearer the start pose first")
    return {"end": end, "path_min_keep_out_clearance_m": worst_out,
            "path_min_clearance_m": worst_in}


def execute_goto_start(args, *, prompt: Callable[[str], str] = _ask,
                       log: Callable[[str], None] = _stderr, limits: Limits = DEFAULT) -> int:
    try:
        require_run_flags(args, "goto-start")
        if getattr(args, "wrapper", None):
            raise RunError("goto-start moves the bare robot; drop --wrapper")
        if not args.urdf:
            raise RunError("goto-start is envelope-aware and needs --urdf (and the kit config's joint_map)")
        envelope = envelope_mod.load(args.envelope)
        kit_config = load_kit_config(args.kit_config)
        pose_name = getattr(args, "pose", None)
        if pose_name:  # a named pose (be instrument plan's targets); every check below applies unchanged
            pose = (kit_config.get("poses_deg") or {}).get(pose_name)
            if not isinstance(pose, dict):
                raise RunError(f"--pose {pose_name}: the kit config has no poses_deg.{pose_name}")
        else:
            pose = kit_config.get("start_pose_deg")
            if not isinstance(pose, dict):
                raise RunError("goto-start needs --kit-config with start_pose_deg")
        kind, host, port = parse_instrument(args.instrument)
        if kind == "sim":
            raise RunError("goto-start takes udp:PORT or none")
        context = _context(args, "goto-start", kit_config, limits, prompt, log)
        adapter = adapter_module(args.adapter).from_args(args, context)
        fk = ArmFK.build(args.urdf, kit_config.get("joint_map"), envelope)
    except (InterlockError, RunError, AdapterError, FKError, ValueError, OSError) as e:
        log(f"Refused: {e}")
        return EXIT_REFUSED
    try:
        space = adapter.open()
    except Exception as e:  # noqa: BLE001 - open() releases what it opened before raising
        log(f"Refused: {type(e).__name__}: {e}")
        return EXIT_REFUSED
    try:
        if space.kind != "joint_position":
            raise RunError("goto-start moves joint-position robots")
        st = adapter.read_state()
        if st is None:
            raise RunError("the adapter returned no state")
        present = state_to_command(space, st)
        target = start_command(space, pose)
        steps = plan_goto(space, present, target, limits)
        summary = check_goto_path(fk, envelope, st, steps, limits.goto_path_samples)
    except (RunError, FKError, ValueError, KeyError) as e:
        log(f"Refused: {e}")
        adapter.close()
        return EXIT_REFUSED
    live: _Live | None = None
    result: SessionResult | None = None
    try:
        largest = max((abs(target[k] - present[k]) for k in target), default=0.0)
        what = f"pose {pose_name}" if pose_name else "the start pose"
        log(f"goto-start ({what}): largest axis move {largest:.1f}, {len(steps) / limits.goto_hz:.1f} s at "
            f"most {limits.goto_rate_per_s['deg']:.0f} deg/s, then {limits.goto_settle_s:.0f} s "
            f"holding {what}. Along the path: closest keep-out "
            f"{summary['path_min_keep_out_clearance_m'] * 100:.1f} cm; the end pose is inside.")
        clock = make_clock()
        feed, _ = _make_feed(kind, host, port, clock=clock, envelope=envelope, adapter=adapter, fk=fk,
                             args=args, samples_csv=None)
        held = steps + [steps[-1]] * round(limits.goto_settle_s * limits.goto_hz)
        proposals = [Proposal(i, i / limits.goto_hz, "goto_start", s) for i, s in enumerate(held)]
        live = _Live(adapter=adapter, envelope=envelope, fk=fk, fk_tripwire=True, feed=feed,
                     mode="goto-start", tool_point=tool_point_of(envelope), proposals=proposals,
                     files=None, clock=clock, estop_cmd=args.estop_cmd, limits=limits, log=log)
        result = live.run(prompt, "GO", "Keep clear of the arm; one hand on the hardware stop.",
                          wait_still=False)
    finally:
        if live is not None:
            live.teardown(result)
        else:  # failed before the session existed: nothing moved, but the robot is connected
            adapter.close()
    code = _exit_code(result, live)
    log(f"goto-start ended ({live.latch.kind}: {live.latch.reason}); exit {code}.")
    return code
