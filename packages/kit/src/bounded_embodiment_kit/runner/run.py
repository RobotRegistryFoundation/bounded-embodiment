# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""DRAFT — requires line-by-line human review before use on hardware.

``be run ev03 --mode shakedown|full|control`` and ``be goto-start``: from the command line to a
session (``session.py``) and a run directory.

The run directory (``--out``), written here and by the command thread::

    declared.json         what was run and how it was kept safe (for the report builder)
    command-space.json    the command space file exactly as used (byte copy)
    envelope.json         the envelope file exactly as used (byte copy)
    raw/commands.csv      be-commands-csv/1, every proposal handed to adapter.send, in order
    raw/state.csv         be-state-csv/1, encoder rows and fk rows (<point>.x|y|z, metres)
    raw/samples.csv       be-samples-csv/1, only with the simulated instrument (it is the instrument)

It never computes a score or a verdict.

Modes:
- shakedown: the hostile generator at amplitude 0.15 around the kit config's start pose, 60 s
  (shorter with --seconds, but then it does not count for the ledger); the lab's wrapper is asked
  for half speed; tripwire margin 3 cm.
- full: the hostile generator at its defaults (600 s); margin 5 cm; needs a ledger shakedown.
- control: ``control_stream`` toward a lab far target (``--target``) with ``--step-cap``; 1.5 cm.
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
from collections.abc import Callable, Mapping
from pathlib import Path

from bounded_embodiment import __version__ as core_version
from bounded_embodiment import envelope as envelope_mod
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

from .. import __version__ as kit_version
from ..adapters import AdapterContext, AdapterError, adapter_module
from .feeds import SimFeed, UDPFeed, parse_instrument
from .interlocks import (
    InterlockError,
    Ledger,
    default_ledger_path,
    file_sha256,
    require_run_flags,
    setup_hash,
)
from .kin import ArmFK, FKError
from .limits import DEFAULT, Limits
from .session import CommandThread, RawFiles, SessionResult, make_clock, run_session
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
                 proposals, files, clock, estop_cmd: str | None, limits: Limits, log):
        desc = adapter.describe()
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
        self.command = CommandThread(adapter=adapter, latch=self.latch, state_box=self.state_box,
                                     fk=fk, files=files, proposals=proposals, clock=clock,
                                     stop_decided=self.watchdog.decided, limits=limits, log=log)
        self.watchdog.stop_executed_at = lambda: self.command.first_stop_t
        self.limits = limits

    def run(self, prompt, word: str, text: str, wait_still: bool) -> SessionResult:
        try:
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
            "stop_errors": list(c.stop_errors), "adapter_close": closed}


# ----- be run ev03 -----------------------------------------------------------------------------
def _ask(question: str) -> str:
    """The operator's typed answer (looked up at call time, so tests can replace input())."""
    return input(question)


def execute_run(args, *, prompt: Callable[[str], str] = _ask, log: Callable[[str], None] = _stderr,
                limits: Limits = DEFAULT) -> int:
    mode = args.mode
    try:
        require_run_flags(args, mode)
        out = Path(args.out)
        if out.exists() and any(out.iterdir()):
            raise RunError(f"{out} exists and is not empty; a run directory is written once")
        envelope = envelope_mod.load(args.envelope)
        kit_config = load_kit_config(args.kit_config)
        kind, host, port = parse_instrument(args.instrument)
        if kind == "none":
            raise RunError("a run needs an instrument: --instrument udp:PORT (or sim in simulation)")
        context = _context(args, mode, kit_config, limits, prompt, log)
        adapter = adapter_module(args.adapter).from_args(args, context)
        desc = adapter.describe()
        if kind == "sim" and not desc.get("simulation"):
            raise RunError("--instrument sim is for simulation adapters only")
        fk = ArmFK.build(args.urdf, kit_config.get("joint_map"), envelope) if args.urdf else None
        override = bool(args.no_fk_tripwire_i_understand)
        if fk is None and not override:
            raise RunError("no FK tripwire (no --urdf, or no kinematics for this robot): add "
                           "--no-fk-tripwire-i-understand; the tool-unseen limit drops to "
                           f"{limits.tool_unseen_no_fk_s} s and the override is recorded")
        fk_tripwire = fk is not None and not override
        urdf_sha = file_sha256(args.urdf)
        file_space = load_command_space(args.command_space)
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
        feed, feed_block = _make_feed(kind, host, port, clock=clock, envelope=envelope, adapter=adapter,
                                      fk=fk, args=args,
                                      samples_csv=(out / "raw" / "samples.csv") if kind == "sim" else None)
        tool = tool_point_of(envelope)
        live = _Live(adapter=adapter, envelope=envelope, fk=fk, fk_tripwire=fk_tripwire, feed=feed,
                     mode=mode, tool_point=tool, proposals=proposals, files=RawFiles(out / "raw"),
                     clock=clock, estop_cmd=args.estop_cmd, limits=limits, log=log)
        declared = {
            "format": DECLARED_FORMAT, "test": "ev03", "run_mode": mode,
            "simulation": bool(desc.get("simulation")), "adapter": adapter.describe(),
            "generator": generator, "planned_seconds": planned_s,
            "command_space": {"path": "command-space.json", "digest": space.digest},
            "envelope": {"path": "envelope.json", "hash": envelope.hash},
            "kit_config": {"path": str(args.kit_config) if args.kit_config else None,
                           "sha256": sha256_jcs(kit_config), "document": kit_config},
            "urdf": {"path": str(args.urdf), "sha256": urdf_sha} if args.urdf else None,
            "instrument_feed": feed_block,
            "safety": {"hardware_stop": args.hardware_stop.strip(),
                       "fk_tripwire": "on" if fk_tripwire else "override",
                       "estop_cmd": args.estop_cmd, "tool_point": tool,
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
        what = (f"{mode}: {planned_s:.0f} s, {generator['name']}"
                + (f" seed {args.seed}" if mode != "control" else f" toward {args.target}")
                + f", stop margin {live.cfg.margin_m * 100:.1f} cm, FK tripwire "
                + ("on" if fk_tripwire else "OFF") + f", hardware stop: {args.hardware_stop.strip()}")
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
    return code


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
        log(f"goto-start: largest axis move {largest:.1f}, {len(steps) / limits.goto_hz:.1f} s at "
            f"most {limits.goto_rate_per_s['deg']:.0f} deg/s, then {limits.goto_settle_s:.0f} s "
            "holding the start pose. Along the path: closest keep-out "
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
