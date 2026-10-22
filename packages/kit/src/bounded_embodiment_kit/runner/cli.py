# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""DRAFT: requires line-by-line human review before use on hardware.

``be run``, ``be goto-start``, ``be doctor`` and ``be adapter``: the runner, its watchdog and the
adapters. Before anything moves, read the lab guide's safety section (docs/lab-guide.md, step 0).

    be doctor [--port P --id ID] [--envelope E --kit-config K --urdf U] [--instrument udp:PORT]
    be goto-start --adapter lerobot --port P --id ID --envelope E --command-space C \\
                  --kit-config K --urdf U --hardware-stop "hand on the servo power plug" [--pose T1]
    be run ev03 --mode shakedown --adapter lerobot --port P --id ID --wrapper gate.py:make_gate \\
                --envelope E --command-space C --kit-config K --urdf U --instrument udp:5005 \\
                --out runs/bob-shakedown --i-accept-the-risk --hardware-stop "..." [--estop-cmd CMD]
    be run ev10 --mode full --waypoints ev10-waypoints.json [--cycles N --commands-per-visit K --hz H] ...
    be run ev02 --mode full --plan ev02-stop-plan.json --step-cap 1.0 [--drive-s S --return-s S --hz H]
                [--hardwired-stop-cmd CMD] [--power-loss-cmd CMD] ...   (each issues that source)
    be run ev05 --mode full --plan ev05-model-loss-plan.json --step-cap 1.0 [--trials N
                --motion-s 1.0,2.0 --silence-s S --restart-s S --hz H] ...
    be run ev03 ... --instrument none --no-live-instrument-i-understand    # offline instrument (be fit)
    be adapter new NAME          write NAME_adapter.py, a python adapter template
    be adapter list

Exit codes for run and goto-start: 0 finished and the stop was acknowledged; 1 stopped early
(tripwire, Ctrl+C, error) or cancelled; 2 refused before anything moved; 3 the stop was not
acknowledged (CUT POWER was sounded) or a command got stuck.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from bounded_embodiment import envelope as envelope_mod

from ..adapters import ADAPTERS, AdapterError, adapter_module
from ..adapters.python import write_template
from .doctor import doctor_arm, environment, listen
from .feeds import parse_instrument
from .kin import ArmFK, FKError
from .limits import MODES

__all__ = ["register"]


def _adapter_args(p) -> None:
    p.add_argument("--adapter", required=True, choices=list(ADAPTERS))
    p.add_argument("--envelope", required=True, help="envelope 0.1 or 0.2 JSON")
    p.add_argument("--command-space", required=True, help="the lab's declared command space JSON")
    p.add_argument("--kit-config", help="kit config JSON: start_pose_deg, joint_map")
    p.add_argument("--urdf", help="the robot's URDF (FK tripwire, simulator)")
    p.add_argument("--hardware-stop", metavar="WHAT",
                   help='the physical stop at hand, recorded: e.g. "hand on the servo power plug"')
    p.add_argument("--estop-cmd", metavar="CMD",
                   help="run (no shell) when a stop is not acknowledged (the CUT POWER escalation only, "
                        "never an EV-02 trial source): the lab's relay or e-stop")
    g = p.add_argument_group("simulated instrument (--instrument sim)")
    g.add_argument("--sim-fps", type=float, default=30.0)
    g.add_argument("--sim-sigma-m", type=float, default=0.001)
    g.add_argument("--sim-bias-m", type=float, default=0.003)
    g.add_argument("--sim-dropout", type=float, default=0.02)
    g.add_argument("--sim-lag-s", type=float, default=0.05)
    g.add_argument("--sim-feed-seed", type=int, default=0)
    for name in ADAPTERS:
        adapter_module(name).add_arguments(p)


def _cmd_run(args) -> int:
    from .run import execute_run

    return execute_run(args)


def _cmd_goto(args) -> int:
    from .run import execute_goto_start

    return execute_goto_start(args)


def _cmd_doctor(args) -> int:
    out: dict = {"environment": environment()}
    envelope = fk = None
    try:
        if args.envelope:
            envelope = envelope_mod.load(args.envelope)
        if envelope is not None and args.urdf:
            kit = json.loads(Path(args.kit_config).read_text(encoding="utf-8")) if args.kit_config else {}
            fk = ArmFK.build(args.urdf, kit.get("joint_map"), envelope)
        if args.port and args.robot_id:
            from ..adapters.lerobot import build_follower

            out["arm"] = doctor_arm(lambda: build_follower(args.robot_type, args.port, args.robot_id),
                                    envelope=envelope, fk=fk)
        if args.instrument:
            kind, _, port = parse_instrument(args.instrument)
            if kind != "udp":
                raise ValueError("doctor listens to udp:PORT only")
            out["instrument"] = listen(port, args.listen, set(envelope.points) if envelope else None)
    except (AdapterError, FKError, ValueError, OSError) as e:
        out["error"] = f"{type(e).__name__}: {e}"
        print(json.dumps(out, indent=2, default=str))
        return 1
    print(json.dumps(out, indent=2, default=str))
    return 0


def _cmd_adapter(args) -> int:
    if args.adapter_command == "new":
        try:
            path = write_template(args.name, args.out)
        except AdapterError as e:
            print(f"be: {e}", file=sys.stderr)
            return 1
        print(f"Wrote {path}. Fill in its TODOs, then: be run ev03 --adapter python "
              f"--adapter-class {path}:Adapter ...")
        return 0
    for name, module in ADAPTERS.items():
        print(f"{name:10s} {module}")
    return 0


def register(subparsers) -> None:
    p = subparsers.add_parser("run", help="run a test against a machine through an adapter",
                              description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("test", choices=["ev03", "ev10", "ev02", "ev05"])
    p.add_argument("--mode", required=True, choices=[m for m in MODES if m != "goto-start"],
                   help="ev10, ev02 and ev05 take full")
    p.add_argument("--instrument", required=True,
                   help="udp:PORT, udp:HOST:PORT, sim, or none (with --no-live-instrument-i-understand)")
    p.add_argument("--out", required=True, help="the run directory (must not exist, or be empty)")
    p.add_argument("--seed", type=int, default=None,
                   help="default 7 (ev03, ev05), 0 (ev10, ev02: their streams have no randomness)")
    p.add_argument("--no-live-instrument-i-understand", action="store_true",
                   help="--instrument none: the instrument records for offline scoring and the FK tripwire "
                        "is the only live supervision (recorded in declared.json and the report's notes)")
    t = p.add_argument_group("EV-10 (be-waypoints/1)")
    t.add_argument("--waypoints", help="the lab's ev10-waypoints.json (waypoints 0.1)")
    t.add_argument("--cycles", type=int, help="rounds of the tour (default: enough for 60 s)")
    t.add_argument("--commands-per-visit", type=int, help="commands held at each waypoint (default 4 s worth)")
    t.add_argument("--hz", type=float, help="command rate of the ev10 tour or the ev02/ev05 trials (default 50)")
    s = p.add_argument_group("EV-02 and EV-05 (be-stops/1, be-model-loss/1): one trial at a time, NEXT between")
    s.add_argument("--plan", help="the lab's ev02-stop-plan.json (stop plan 0.1) or ev05-model-loss-plan.json")
    s.add_argument("--hardwired-stop-cmd", metavar="CMD",
                   help="ev02: the command (no shell) that opens the machine's hardwired stop circuit for a "
                        "hardwired_stop trial. It must latch, and never close the circuit again itself: "
                        "the operator resets it before typing NEXT")
    s.add_argument("--power-loss-cmd", metavar="CMD",
                   help="ev02: the command (no shell) that removes motive power for a power_loss trial. "
                        "It must latch, and never restore power itself: the operator restores it before "
                        "typing NEXT")
    s.add_argument("--drive-s", type=float, help="ev02: how long each drive walks before the stop (default 1.0)")
    s.add_argument("--return-s", type=float, help="ev02: how long the walk back to the start lasts (default 2.0)")
    s.add_argument("--trials", type=int, help="ev05: how many losses (default 3)")
    s.add_argument("--motion-s", help="ev05: SHORTEST,LONGEST motion before each loss, seeded (default 1.0,2.0)")
    s.add_argument("--silence-s", type=float,
                   help="ev05: how long no command comes (default and least: the model timeout + stop time + "
                        "the scorer's still window and hold + 0.5 s)")
    s.add_argument("--restart-s", type=float,
                   help="ev05: how long the commands come back for (default: the restart check + 0.5 s)")
    p.add_argument("--seconds", type=float, help="shorten the run (a short shakedown does not count)")
    p.add_argument("--target", help="control: the far target id")
    p.add_argument("--step-cap", help="control, ev02, ev05: largest change per command (number, or JSON per axis; "
                                      "none for no cap, refused by ev05)")
    p.add_argument("--i-accept-the-risk", action="store_true",
                   help="required for every run: the stream is meant to make a weak gate fail")
    p.add_argument("--no-fk-tripwire-i-understand", action="store_true",
                   help="run without the FK tripwire (recorded; tool-unseen limit 0.2 s)")
    p.add_argument("--ledger", help="the shakedown ledger (default: ~/.local/state/bounded-embodiment/)")
    _adapter_args(p)
    p.set_defaults(func=_cmd_run)

    g = subparsers.add_parser("goto-start", help="move slowly to the kit config's start pose",
                              description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g.add_argument("--instrument", default="none", help="udp:PORT, or none")
    g.add_argument("--pose", metavar="NAME",
                   help="go to the kit config's poses_deg[NAME] instead of start_pose_deg (be instrument plan)")
    _adapter_args(g)
    g.set_defaults(func=_cmd_goto)

    d = subparsers.add_parser("doctor", help="read-only check of this machine and the arm",
                              description="Read-only check: this computer's Python, packages, serial ports (and "
                              "who holds them) and LeRobot calibration files; with --port and --id, the "
                              "arm's servo registers, and with --envelope and --urdf also where its own "
                              "kinematics put the envelope's points; with --instrument, the instrument "
                              "feed. Nothing moves. Prints one JSON object; exit 1 if a check failed.")
    d.add_argument("--port", help="the follower's serial port (lerobot)")
    d.add_argument("--id", dest="robot_id", help="the LeRobot robot id (its calibration file)")
    d.add_argument("--robot-type", default="so101_follower", help="LeRobot robot type (default so101_follower)")
    d.add_argument("--envelope", help="envelope JSON (with --urdf: locate its points on the arm)")
    d.add_argument("--kit-config", help="kit config JSON: the joint map for --urdf")
    d.add_argument("--urdf", help="the robot's URDF, for forward kinematics (needs --envelope)")
    d.add_argument("--instrument", help="udp:PORT: listen to the instrument feed")
    d.add_argument("--listen", type=float, default=3.0, help="seconds to listen (default 3)")
    d.set_defaults(func=_cmd_doctor)

    a = subparsers.add_parser("adapter", help="write a python adapter template, or list adapters")
    asub = a.add_subparsers(dest="adapter_command", required=True)
    n = asub.add_parser("new", help="write NAME_adapter.py")
    n.add_argument("name", help="the adapter's name; the file is NAME_adapter.py")
    n.add_argument("--out", default=".", help="directory to write it in (default: the current one)")
    asub.add_parser("list", help="the built-in adapters")
    a.set_defaults(func=_cmd_adapter)
