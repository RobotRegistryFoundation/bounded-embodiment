# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""Reference workspace gate for an SO-101 on LeRobot, placed between the model and
``robot.send_action``. AN EXAMPLE, NOT A PRODUCT: it exists to show that the test discriminates.

Ported from ev03-harness ``ev03/arm/gate.py`` (``ReferenceArmGate``, the gate that held for 10 min
in simulation and on hardware). The decision logic is the harness's, rule for rule, and it makes the
same decisions (``tests/test_harness_parity.py`` drives both side by side). What changed is where
its arm model and its envelope come from:

- forward kinematics: the kit's ``URDFModel`` through the kit config's ``JointMap``
  (``ArmKinematics`` below), instead of the harness's ``SO101Kinematics``;
- the envelope: core's ``bounded_embodiment.envelope.load`` and ``Envelope.clearance(point, xyz)``,
  instead of the harness's ``ArmEnvelope``. The tool point is the envelope's ``tool`` (else its
  first point); every other envelope point with a link and a zone is checked the way the harness
  checked its link points (the elbow and the wrist) against ``link_keep_in`` and the keep-outs;
- the joint speed cap: the envelope's ``motion.joints[].max_speed_radps``, else
  ``motion.max_joint_speed_radps`` (the harness format had one ``max_joint_speed_dps``);
- the measurement tolerance in ``margin = tolerance_m / 2 + margin_m``: the kit config's
  ``tolerance_m`` (default 0.01 m), since an envelope 0.2 does not carry one.

Two additions, neither of which changes a decision on the repo's example envelopes: an envelope's
``motion.joints[].position_rad`` limits also bound each joint (the harness format could not declare
them), and rule 4(a) is evaluated fraction by fraction, stopping at the first that fails and skipping
a check that cannot fail, instead of all forty at once. A clearance is a signed distance, so it
changes by at most 1 m per m moved: a check whose clearance now, less how far its point can have
moved, is still above the margin passes there. The safe prefix is the same; it is cheaper, which
matters because the kit's runner runs in real time and a hostile ``fast`` burst asks for 500
commands a second.

The design, from the harness:

The gate keeps a reference: the joint goals it last sent, which is where it is holding the arm or
moving it from. It plans from that reference, not from the encoder reading. A position servo is a
spring around its goal, with friction: under gravity it settles past the goal (on an SO-101 at
LeRobot's P gain, 7 to 27 encoder ticks at the shoulder, depending on which way it arrived), and while
moving it lags behind. A layer that plans from the measured pose re-anchors the goal to the sagged
position every cycle, so the sag accumulates: the arm creeps downward while the layer believes it is
holding still. That is how the first version of this gate failed on a real SO-101 (out of the box in
under 6 s), after passing in simulation with ideal servos (``so101_reference_v1.py`` keeps it). The
encoders are still read every command, to measure the offset between the arm and its reference.

Every command:
  1. Validate: only known joint keys, finite numbers (booleans and strings rejected). Invalid -> hold:
     re-send the reference unchanged (the recovery table's "reject -> hold").
  2. Predict what will actually execute: clamp each joint to its calibrated range (the servo firmware
     will) and to 5 deg inside the arm's published joint limits; cap each joint's step at 3 deg and at
     the envelope's joint speed limit (if it declares one: the tool-point envelope says nothing about
     the elbow, so this keeps the links slow too); and quantise every step to whole encoder ticks,
     rounding toward the reference. (LeRobot converts degrees to ticks with int(), which floors, so a
     step of -0.3 ticks would otherwise become a full -1 tick; at 500 commands a second that alone
     moves a joint at 44 deg/s.) The gate checks the command the servo will receive.
  3. Rate-independent speed limit: a displacement budget fills at max_tool_speed * speed_factor metres
     per second of elapsed time (holding at most 20 ms worth) and each command spends what it moves
     the reference, so commanding faster does not move the arm faster, and commanding very fast
     (steps smaller than one encoder tick) does not freeze it either. Over any 0.2 s window the
     reference moves at most 0.8 * (0.2 + 0.02) / 0.2 = 0.88 of the speed limit. A servo following it
     with lag does not move faster than it.
  4. Predictive workspace check: sample the joint-space line from the reference toward the goal. The
     offset o = FK(measured) - FK(reference) says where the arm sits relative to its reference (sag,
     friction, lag). The rest offset r is the same offset measured the last time the arm was at rest
     (its reference unchanged for settle_s, 0.3 s): while the arm moves, its lag can cancel the sag,
     and the sag comes back when it stops. Keep the longest prefix on which every sample q is
       (a) inside the envelope by at least the margin, or no closer to the boundary than now, for
           the commanded point FK(q) and the predicted arm positions FK(q) + o and FK(q) + r; and
           the same for every other zoned point (the elbow and wrist), each with its own offsets;
       (b) within the displacement allowed for this command; and
       (c) no more than max_lead_m (4 cm) from the measured arm, or no further than now.
     If the whole step fits, allow (or clamp, if step 2 cut it); if only part fits, clamp to that part;
     if none fits, hold. Each joint is also kept within max_lead_deg (10 deg) of its measured angle
     (or no further than now), so a blocked joint cannot wind up and then spring free.

Rule 4(a)'s "no closer than now" lets a reference inside the margin band, or outside the envelope,
move back in rather than freezing; it can never move further out.

The first reference is the servos' own goal register where it is within 4 deg of the measured angle
(the arm is already holding there), otherwise the measured angle.

Known approximations: the gate checks the straight joint-space line, but servos do not move along it
(each joint slews toward its own goal and arrives at its own time), so mid-step the tool can wander a
few millimetres off the checked line; the offsets are measured now and assumed to persist (friction
can shift them by up to about a centimetre when a joint reverses); and the rest offset was measured
at another pose, while sag grows as the arm reaches out. The margin absorbs these. The external
measurement in EV-03 is what settles it.

Why the rest offset: without it, in simulation with the sag model fitted to bob, the gate brought a
moving arm's reference to the margin above a keep-out (13 mm) while lag hid the sag (offset a few mm);
when the arm stopped it sagged 13.5 mm, onto the keep-out's top face. That was within the 10 mm
measurement tolerance, so it scored as a pass, with none of the margin left.

Plugging it in: ``be run ev03 --adapter lerobot --wrapper reference_gates.so101_reference:factory``
(or ``sim-so101``). The kit calls ``factory(robot, context)`` with the already connected robot.
"""
from __future__ import annotations

import math
import time
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path

import numpy as np
from bounded_embodiment import envelope as envelope_mod
from bounded_embodiment_kit.kinematics import JointMap, URDFModel, envelope_points

from ._context import load_kit_config, tool_point

__all__ = ["ARM_JOINTS", "FRACTIONS", "ArmKinematics", "ReferenceArmGate", "factory"]

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# Fractions of the proposed step to test, log-spaced from 0.1 % to 100 % (about 19 % apart).
FRACTIONS = np.geomspace(1e-3, 1.0, 40)

#: The gate's speed factor when the kit says "use your default" (context speed_factor None): the
#: harness's full-run value. A shakedown asks for 0.5.
DEFAULT_SPEED_FACTOR = 0.8
#: The measurement tolerance when the kit config does not give one (the harness envelopes' value).
DEFAULT_TOLERANCE_M = 0.01
#: Room for rounding when a check is skipped because it cannot fail (far above any float error, far
#: below anything physical): the skipped check would have passed with the margin to spare.
_SLACK_M = 1e-9


class ArmKinematics:
    """The gate's arm model: LeRobot degrees, through the lab's joint map, to URDF joint values, to
    the envelope's points (``{point id: (link, offset_m)}``) in the envelope frame."""

    def __init__(self, model: URDFModel, joint_map: JointMap,
                 points: Mapping[str, tuple[str, Sequence[float] | None]]):
        self.model, self.joint_map = model, joint_map
        self.points = dict(points)

    @classmethod
    def from_files(cls, urdf_path: str | Path, joint_map: Mapping | None, envelope) -> ArmKinematics:
        model = URDFModel.from_file(urdf_path)
        if envelope.frame != model.root:
            raise ValueError(f"the envelope's frame is {envelope.frame!r} but the URDF's root link is "
                             f"{model.root!r}; the gate's points would be in the wrong frame")
        return cls(model, JointMap.from_dict(joint_map), envelope_points(envelope.normalized))

    def lerobot_limits_deg(self, margin_deg: float = 0.0) -> dict[str, tuple[float, float]]:
        """URDF joint limits expressed in this arm's LeRobot degrees, shrunk by margin_deg."""
        out = {}
        for name, (lo, hi) in self.model.limits().items():
            a = self.joint_map.from_urdf({name: lo})[name]
            b = self.joint_map.from_urdf({name: hi})[name]
            out[name] = (min(a, b) + margin_deg, max(a, b) - margin_deg)
        return out

    def to_lerobot_deg(self, name: str, value_rad: float) -> float:
        return self.joint_map.from_urdf({name: value_rad})[name]

    def link_points(self, configs: Sequence[Mapping[str, float]]) -> dict[str, np.ndarray]:
        """Batched forward kinematics: LeRobot-degree joint dicts -> ``{point id: (N, 3)}``."""
        n = len(configs)
        q = {}
        for j in ARM_JOINTS:
            sign, off = self.joint_map.entries.get(j, (1.0, 0.0))
            q[j] = np.radians(sign * np.array([c[j] for c in configs], dtype=float) + off)
        poses = self.model.fk_batch(q, n=n)
        out = {}
        for pid, (link, offset) in self.points.items():
            T = poses[link]
            o = np.zeros(3) if offset is None else np.asarray(offset, dtype=float)
            out[pid] = T[:, :3, :3] @ o + T[:, :3, 3]
        return out


class ReferenceArmGate:
    """The harness's ReferenceArmGate (module docstring). ``send_action`` is ``propose``: the kit's
    lerobot adapter hands it every command, and reads ``last_decision``."""

    name = "reference gate"

    def __init__(self, robot, envelope, kin: ArmKinematics, clock: Callable[[], float], *,
                 tolerance_m: float = DEFAULT_TOLERANCE_M, margin_m: float = 0.008,
                 max_joint_step_deg: float = 3.0, speed_factor: float = DEFAULT_SPEED_FACTOR,
                 max_lead_m: float = 0.04, max_lead_deg: float = 10.0,
                 adopt_goal_within_deg: float = 4.0, settle_s: float = 0.3):
        self.robot, self.env, self.kin, self.clock = robot, envelope, kin, clock
        self.tool = tool_point(envelope)
        if self.tool not in kin.points:
            raise ValueError(f"the envelope's point {self.tool!r} has no link, so the gate cannot "
                             "predict where it goes")
        # The other points the harness checked as "links" (elbow, wrist): every zoned point with a link.
        self.link_points = [p for p in envelope.points
                            if p != self.tool and p in kin.points and envelope.zones_for(p)]
        self.margin = tolerance_m * 0.5 + margin_m
        self.max_step = max_joint_step_deg
        self.speed_factor = speed_factor
        self.max_lead_m, self.max_lead_deg = max_lead_m, max_lead_deg
        self.adopt_goal_within_deg = adopt_goal_within_deg
        self.settle_s = settle_s
        self.max_tool_speed_mps = envelope.speed_limit_mps(self.tool)
        self.joint_speed_dps = _joint_speed_dps(envelope)
        self.rest_offsets = None  # {point: offset} measured when the arm last sat still on its reference
        self.ref_moved_t = None  # when the reference last moved
        self.last_t = None
        self.budget = self.max_tool_speed_mps * speed_factor * 0.02  # metres of tool travel banked
        self.ref = None  # joint goals last sent (whole ticks): the pose being held or moved from
        self.last_present = None  # encoder reading taken at the last command
        self.last_tracking_m = 0.0
        self.decisions = {"allow": 0, "clamp": 0, "reject_hold": 0, "invalid_hold": 0}
        self.last_decision = None
        bus = robot.bus
        urdf = kin.lerobot_limits_deg(margin_deg=5.0)
        declared = _declared_joint_limits_deg(envelope, kin)
        self.ticks = {}  # joint -> (mid, max_res, lo deg, hi deg)
        for j in ARM_JOINTS:
            c = bus.calibration[j]
            mid = (c.range_min + c.range_max) / 2
            res = bus.model_resolution_table[bus.motors[j].model] - 1
            lo, hi = (c.range_min - mid) * 360 / res, (c.range_max - mid) * 360 / res
            # Never past the servo's calibrated range, nor within 5 deg of the arm's published joint
            # limits: some calibrations leave a joint's range at a full turn, which would let a hostile
            # command drive it into its mechanical stop.
            if j in urdf:
                lo, hi = max(lo, urdf[j][0]), min(hi, urdf[j][1])
            if j in declared:  # the envelope's own joint limits, when it declares them
                lo, hi = max(lo, declared[j][0]), min(hi, declared[j][1])
            self.ticks[j] = (mid, res, lo, hi)

    def _present(self):
        obs = self.robot.bus.sync_read("Present_Position")
        return {m: float(v) for m, v in obs.items()}

    def _initial_ref(self, present):
        """Hold where the servos are already holding: their goal register, if it is close to the measured
        angle; otherwise the measured angle (a stale goal register must never become a target)."""
        start = dict(present)
        try:
            goal = self.robot.bus.sync_read("Goal_Position")
        except Exception:  # noqa: BLE001 - not every bus can read it back
            goal = {}
        for j in ARM_JOINTS:
            g = goal.get(j)
            if g is not None and math.isfinite(float(g)) and abs(float(g) - present[j]) <= self.adopt_goal_within_deg:
                start[j] = float(g)
        ref = self._executed(start, start)
        ref["gripper"] = min(100.0, max(0.0, present["gripper"]))
        return ref

    def _valid(self, action):
        if not isinstance(action, dict) or not action:
            return None
        out = {}
        for k, v in action.items():
            if not isinstance(k, str) or not k.endswith(".pos") or k[:-4] not in self.robot.bus.motors:
                return None
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                return None
            out[k[:-4]] = float(v)
        return out

    def _executed(self, start, goal):
        """The joint angles the servos will actually be sent: whole ticks from start, never past goal."""
        out = {}
        for j in ARM_JOINTS:
            mid, res, _, _ = self.ticks[j]
            n = round(start[j] * res / 360 + mid)  # start, in ticks
            k = math.trunc((goal[j] - start[j]) * res / 360)  # step in whole ticks, toward start
            out[j] = (n + k - mid) * 360 / res  # LeRobot's own normalisation formula
        return out

    def _send(self, joints):
        # LeRobot turns degrees into ticks with int(), which floors. Aim a quarter tick above the
        # intended tick so floating-point rounding can never land on the tick below.
        cmd = {}
        for m, v in joints.items():
            if m in self.ticks:
                v = v + 0.25 * 360 / self.ticks[m][1]
            cmd[f"{m}.pos"] = v
        return self.robot.send_action(cmd)

    def _ok(self, c, c0):
        """Rule 4(a) for one point: inside by the margin, or no closer to the boundary than now."""
        return c >= self.margin or c >= c0 - 1e-9

    def propose(self, action):
        now = self.clock()
        dt = 0.02 if self.last_t is None else min(max(now - self.last_t, 0.0), 0.04)
        self.last_t = now
        present = self._present()
        self.last_present = present
        if self.ref is None:  # start by holding wherever the arm is
            self.ref = self._initial_ref(present)
        ref = self.ref
        target = self._valid(action)
        if target is None:
            return self._decide("invalid_hold", ref)

        goal = dict(ref)
        for j, v in target.items():
            if j == "gripper":
                goal[j] = min(100.0, max(0.0, v))
                continue
            step = self.max_step
            if j in self.joint_speed_dps:  # keep every joint (and so every link) slow
                step = min(step, self.joint_speed_dps[j] * dt)
            _, _, lo, hi = self.ticks[j]
            v = min(hi, max(lo, v))  # firmware limit
            lead = max(self.max_lead_deg, abs(ref[j] - present[j]))  # no wind-up
            v = min(present[j] + lead, max(present[j] - lead, v))
            goal[j] = ref[j] + max(-step, min(step, v - ref[j]))
        capped = any(abs(goal[j] - v) > 1e-9 for j, v in target.items())

        # Points: the reference (row 0), the measured pose (row 1), then the tick-quantised
        # joint-space line reference -> goal.
        steps = [self._executed(ref, {j: ref[j] + u * (goal[j] - ref[j]) for j in ARM_JOINTS})
                 for u in FRACTIONS]
        L = self.kin.link_points([ref, present] + steps)
        offsets = {name: Q[1] - Q[0] for name, Q in L.items()}  # gravity sag, friction, servo lag, or a push
        if self.ref_moved_t is None:  # first command: the arm is holding, so at rest
            self.ref_moved_t = now - self.settle_s
        if self.rest_offsets is None or now - self.ref_moved_t >= self.settle_s:
            self.rest_offsets = offsets
        P = L[self.tool]
        p_ref, p_now, line = P[0], P[1], P[2:]
        offset, rest = offsets[self.tool], self.rest_offsets[self.tool]
        tracking = float(np.linalg.norm(offset))
        self.last_tracking_m = tracking
        disp = np.linalg.norm(line - p_ref, axis=1)
        lead = np.linalg.norm(line - p_now, axis=1)
        rate = self.max_tool_speed_mps * self.speed_factor
        self.budget = min(rate * 0.02, self.budget + rate * dt)
        cheap = (disp <= self.budget) & ((lead <= self.max_lead_m) | (lead <= tracking + 1e-9))

        # Rule 4(a), point by point: (point, "now" clearance, offset) for the commanded point and
        # for the predicted arm positions with the current and with the rest offset.
        clear = self.env.clearance
        checks = [(self.tool, clear(self.tool, p_ref), None),
                  (self.tool, clear(self.tool, p_now), offset),
                  (self.tool, clear(self.tool, p_ref + rest), rest)]
        for name in self.link_points:
            Q = L[name]
            for o in (offsets[name], self.rest_offsets[name]):
                checks.append((name, clear(name, Q[0] + o), o))
            checks.append((name, clear(name, Q[0]), None))
        # Plain floats from here: the same IEEE sums as numpy's, without numpy-scalar overhead.
        names = [self.tool, *self.link_points]
        rows = {name: L[name].tolist() for name in names}
        # How far each sample of the line is from the point's reference position. A clearance is a
        # signed distance, so it changes by at most 1 m per m moved: a check whose "now" clearance,
        # less that distance, is still above the margin cannot fail there and is not evaluated.
        moved = {name: np.linalg.norm(L[name][2:] - L[name][0], axis=1).tolist() for name in names}
        checks = [(name, c0, None if o is None else o.tolist()) for name, c0, o in checks]

        # The longest prefix of FRACTIONS on which every condition holds (the harness evaluated all
        # forty and took the first failure; stopping at that failure gives the same prefix).
        k = 0
        while k < len(FRACTIONS) and cheap[k]:
            if not all(c0 - moved[name][k] - _SLACK_M >= self.margin
                       or self._ok(clear(name, _shift(rows[name][2 + k], o)), c0)
                       for name, c0, o in checks):
                break
            k += 1

        if k == 0:
            return self._decide("reject_hold", ref)
        self.budget = max(0.0, self.budget - float(disp[k - 1]))
        q = dict(steps[k - 1])
        q["gripper"] = goal["gripper"]
        if any(abs(q[j] - ref[j]) > 1e-9 for j in ARM_JOINTS):
            self.ref_moved_t = now
        self.ref = q
        # "allow" = sent as asked, apart from rounding to whole encoder ticks
        exact = k == len(FRACTIONS) and not capped and all(
            abs(q[j] - goal[j]) <= 360 / self.ticks[j][1] + 1e-9 for j in ARM_JOINTS)
        if exact:
            return self._decide("allow", q)
        wanted_motion = any(abs(goal[j] - ref[j]) > 1e-9 for j in ARM_JOINTS)
        moved = any(abs(q[j] - ref[j]) > 1e-9 for j in ARM_JOINTS)
        # A clamp that rounds to no arm motion at all is, in effect, a hold.
        return self._decide("clamp" if moved or not wanted_motion else "reject_hold", q)

    send_action = propose

    def _decide(self, decision, joints):
        self.decisions[decision] += 1
        self.last_decision = decision
        return self._send(joints)


def _shift(p: list[float], o: list[float] | None) -> list[float]:
    return p if o is None else [p[0] + o[0], p[1] + o[1], p[2] + o[2]]


def _joint_speed_dps(envelope) -> dict[str, float]:
    """Per arm joint, the envelope's joint speed cap in deg/s: ``motion.joints[].max_speed_radps``,
    else ``motion.max_joint_speed_radps``; joints with neither are not capped."""
    motion = envelope.motion
    every = motion.get("max_joint_speed_radps")
    own = {j["name"]: j["max_speed_radps"] for j in motion.get("joints", []) if "max_speed_radps" in j}
    out = {}
    for j in ARM_JOINTS:
        radps = own.get(j, every)
        if radps is not None:
            out[j] = math.degrees(radps)
    return out


def _declared_joint_limits_deg(envelope, kin: ArmKinematics) -> dict[str, tuple[float, float]]:
    """The envelope's ``motion.joints[].position_rad`` (URDF radians) in LeRobot degrees."""
    out = {}
    for j in envelope.motion.get("joints", []):
        if j["name"] in ARM_JOINTS and "position_rad" in j:
            a = kin.to_lerobot_deg(j["name"], j["position_rad"][0])
            b = kin.to_lerobot_deg(j["name"], j["position_rad"][1])
            out[j["name"]] = (min(a, b), max(a, b))
    return out


def factory(robot, context: Mapping) -> ReferenceArmGate:
    """``--wrapper reference_gates.so101_reference:factory``: the gate around the connected robot.

    From the run's context: the envelope, the URDF and the kit config (joint map, tolerance_m), and
    ``speed_factor`` (0.5 in a shakedown; None means the gate's own default, 0.8)."""
    if not context.get("envelope_path") or not context.get("urdf_path"):
        raise ValueError("the reference gate needs the run's envelope and URDF (--envelope, --urdf)")
    envelope = envelope_mod.load(context["envelope_path"])
    kit = load_kit_config(context)
    kin = ArmKinematics.from_files(context["urdf_path"], kit.get("joint_map"), envelope)
    speed_factor = context.get("speed_factor")
    return ReferenceArmGate(robot, envelope, kin, time.monotonic,
                            tolerance_m=float(kit.get("tolerance_m", DEFAULT_TOLERANCE_M)),
                            speed_factor=DEFAULT_SPEED_FACTOR if speed_factor is None else float(speed_factor))
