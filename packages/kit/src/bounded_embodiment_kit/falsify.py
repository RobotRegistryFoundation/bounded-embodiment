# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""Falsification search, rule set ``be-falsify/1`` (DRAFT for method v0.2; simulation only).

The hostile stream ``be-hostile/1`` (method section 7.5) is random and does not react to the
machine. This module does the opposite, on purpose: it drives a gate in lockstep with one of the
kit's simulated machines, watches the gate's decisions and the simulated truth, and steers its
commands toward a violation of a declared property. It is a conformance search of the kind used
for cyber-physical systems: look for one trace that breaks the property, so the gate can be fixed.

What a result means (``docs/falsification-be-falsify-1-draft.md``): a trace that violates a
property is evidence of a defect, and it replays. Finding nothing within a budget establishes only
that nothing was found within that budget. It is never a PASS and never feeds a verdict; no scorer
reads anything this module writes.

Simulation only. Nothing here opens a port or talks to hardware: the machines are
``adapters.sim_rover.SimRover`` and ``adapters.sim_so101.SimSO101Adapter`` with
``realtime=False``, stepped by this module (no threads, simulated clock). Running a found trace on
a real machine would need the independent stop path of method section 6, and this module does not
offer to do it.

The kit never imports a gate: a gate is named as ``module:factory`` and loaded with
``adapters.base.load_callable``, as ``be run --wrapper`` does, and called as
``factory(machine, context)``.

The property (``position`` or ``speed``), as robustness (signed distance to violation; negative
is a violation):

- ``position``: the least ``Envelope.clearance`` of any zoned point of the simulated machine, after
  each command's physics period.
- ``speed``: the least ``limit - speed`` over the points with a speed limit, where speed is the
  point's displacement over the last ``speed_window_s`` (0.2 s, the illustrative profile's window)
  divided by the time it spanned. No tolerance is added: this is the declared limit in a simulation
  with perfect sensing.

The search (deterministic given the seed and the step budget; one step is one command handed to the
gate, and every step simulated in any rollout counts). It is a seeded random-restart hill climb on
the robustness that builds a sequence of ``segments`` piecewise-constant commands, each held for
``hold_steps`` command periods, one segment at a time, from the state the earlier segments left the
machine in (so it reacts to the machine and to the gate). For each segment:

a. Candidates: every combination of each axis's lower limit, home value and upper limit (or, with
   more than three axes, the home command, each axis alone at either limit and a few random
   corners), plus one uniform draw; each is held for one segment and scored by the robustness.
b. Bisection on the accept/reject boundary: between the best candidate the gate accepted throughout
   the segment and the best one it refused part of, bisect the straight line between the two
   commands for ``bisect_iters`` steps, scoring each point. This looks for the most aggressive
   command the gate is still willing to pass near a zone edge.
c. Coordinate (compass) search from the best so far: step each axis by plus and minus ``delta`` of
   its range, keep the first change that lowers the robustness, halve ``delta`` when none does,
   ``halvings`` times.
d. Keep the best and go on to the next segment; after the last one, restart.

The search stops at the first rollout whose robustness is below ``-VIOLATION_EPS``. A rollout
resumes from a snapshot of the simulation at the end of the prefix it shares with an earlier one
(simulation only); only the steps it simulates count.

``random_baseline`` drives the same machine with ``be-hostile/1`` streams, seed after seed, and
counts steps the same way, so the two can be compared.
"""
from __future__ import annotations

import argparse
import copy
import itertools
import json
import math
import random
import sys
import threading
import types
from collections import deque
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path

from bounded_embodiment import envelope as envelope_mod
from bounded_embodiment.commandspace import load_command_space

from .adapters.base import load_callable

__all__ = ["PROPERTIES", "RULE_SET", "VIOLATION_EPS", "ArmSystem", "RoverSystem", "SearchResult",
           "build_system", "falsify", "random_baseline", "replay", "rollout", "write_trace"]

RULE_SET = "be-falsify/1"
PROPERTIES = ("position", "speed")
#: The illustrative profile's speed window (``speed_window_s``), used by the ``speed`` property.
SPEED_WINDOW_S = 0.2
#: Words in a gate's ``last_decision`` that mean it did not pass the command on as asked or clamped.
REFUSALS = ("reject", "hold", "stop")
#: How far past the property's limit a step must be to count as a violation: a micrometre for
#: ``position`` (a micrometre per second for ``speed``), far above float rounding on a face, far
#: below anything physical.
VIOLATION_EPS = 1e-6


def refused(decision: object) -> bool:
    """Whether a gate decision is a refusal (reject, hold or stop), in either gate's vocabulary."""
    return isinstance(decision, str) and any(w in decision for w in REFUSALS)


# ----- the machines, in lockstep ------------------------------------------------------------------
class _System:
    """A gate around one of the kit's simulated machines, stepped by the caller. Subclasses set
    ``name`` and implement ``_build`` (a fresh machine in ``self.machine``; returns the gate),
    ``advance`` and ``points``."""

    name = "sim"
    period_s = 0.02

    def __init__(self, gate: str | None, context: Mapping):
        self.gate_spec, self.context = gate, dict(context)
        self.factory = load_callable(gate) if gate else None
        self.envelope = envelope_mod.load(self.context["envelope_path"])
        self.space = load_command_space(self.context["command_space_path"])
        self.zoned = [p for p in self.envelope.points if self.envelope.zones_for(p)]
        self.limits = {p: self.envelope.speed_limit_mps(p) for p in self.envelope.points
                       if self.envelope.speed_limit_mps(p) is not None}

    def reset(self) -> None:
        self.t = 0.0
        self.gate = self._build()
        self.history: deque = deque()

    def send(self, cmd: Mapping) -> object:
        """Hand one command to the gate (or straight to the machine without one); the decision."""
        target = self.gate if self.gate is not None else self.machine
        try:
            target.send_action(dict(cmd))
        except Exception as e:  # noqa: BLE001 - an ungated drive refuses by raising
            return f"exception:{type(e).__name__}"
        return getattr(self.gate, "last_decision", None) if self.gate is not None else None

    def robustness(self, prop: str) -> float:
        """The property's signed distance to violation now (negative: violated). Call once per
        step, after ``advance``: the speed window is kept here."""
        pts = self.points()
        if prop == "position":
            return min(self.envelope.clearance(p, pts[p]) for p in self.zoned)
        self.history.append((self.t, pts))
        while len(self.history) > 1 and self.history[1][0] <= self.t - SPEED_WINDOW_S + 1e-12:
            self.history.popleft()
        t0, p0 = self.history[0]
        if self.t - t0 < SPEED_WINDOW_S - 1e-9:
            return math.inf
        return min(lim - math.dist(pts[p], p0[p]) / (self.t - t0) for p, lim in self.limits.items())

    def state(self) -> dict:
        return {"t_s": self.t, "points": {p: list(v) for p, v in self.points().items()}}

    #: The attributes that hold the simulation's mutable state (copied by ``snapshot``).
    live = ("t", "machine", "gate", "history")
    #: The attributes that are read-only models, shared by every snapshot.
    shared = ("envelope", "space")

    def snapshot(self) -> dict:
        state = {k: getattr(self, k) for k in self.live}
        memo = {id(getattr(self, k)): getattr(self, k) for k in self.shared}
        _share_uncopyable(state, memo)
        return copy.deepcopy(state, memo)

    def restore(self, snap: dict) -> None:
        state = dict(snap)
        memo = {id(getattr(self, k)): getattr(self, k) for k in self.shared}
        _share_uncopyable(state, memo)
        for k, v in copy.deepcopy(state, memo).items():
            setattr(self, k, v)


_UNCOPYABLE = (type(threading.Lock()), type(threading.RLock()), threading.Event,
               threading.Condition, threading.Thread)


def _share_uncopyable(root: object, memo: dict) -> None:
    """Put every lock, event, condition and thread reachable from ``root``, and every read-only
    model (URDF, envelope, command space), into ``memo``, so that ``copy.deepcopy`` shares them
    instead of failing on them or copying them."""
    from bounded_embodiment.commandspace import CommandSpace

    from .kinematics import URDFModel

    shared = (*_UNCOPYABLE, URDFModel, envelope_mod.Envelope, CommandSpace)
    seen, todo = set(), [root]
    while todo:
        obj = todo.pop()
        if id(obj) in seen or id(obj) in memo:
            continue
        seen.add(id(obj))
        if isinstance(obj, shared):
            memo[id(obj)] = obj
        elif isinstance(obj, dict):
            todo.extend(obj.values())
        elif isinstance(obj, (list, tuple, set, frozenset, deque)):
            todo.extend(obj)
        elif isinstance(obj, types.MethodType):
            todo.append(obj.__self__)
        elif hasattr(obj, "__dict__") and not isinstance(obj, (type, types.FunctionType,
                                                                types.ModuleType)):
            todo.extend(vars(obj).values())


class RoverSystem(_System):
    """The kit's simulated rover (``SimRover``), physics at 5 ms as in the sim-rover adapter."""

    name = "sim-rover"
    physics_s = 0.005

    def __init__(self, gate, context, start=(1.0, 1.0, 0.0)):
        super().__init__(gate, context)
        self.start = tuple(start)

    def _build(self):
        from .adapters.sim_rover import SimRover

        self.machine = SimRover(start=self.start)
        return self.factory(self.machine, dict(self.context)) if self.factory else None

    def advance(self, dt: float) -> None:
        n = max(1, round(dt / self.physics_s))
        for _ in range(n):
            self.machine.tick(dt / n)
        self.t += dt

    def points(self) -> dict:
        x, y, _ = self.machine.pose()
        return {p: (x, y, 0.0) for p in self.envelope.points}

    def state(self) -> dict:
        x, y, th = self.machine.pose()
        v, w = self.machine.velocities()
        return {**super().state(), "pose": [x, y, th], "velocities": [v, w]}


class ArmSystem(_System):
    """The kit's simulated SO-101 at the kit config's start pose, powered and holding (as
    ev03-harness started it), connected by the kit's own safe connect; servos at 1 ms. A gate
    with a ``clock`` attribute is given the simulated clock."""

    name = "sim-so101"
    servo_s = 0.001
    live = ("t", "adapter", "machine", "gate", "history")

    def __init__(self, gate, context, *, servos: str = "sag"):
        super().__init__(gate, context)
        if servos not in ("ideal", "sag"):
            raise ValueError("servos is 'ideal' or 'sag'")
        self.servos = servos
        kit = json.loads(Path(self.context["kit_config_path"]).read_text(encoding="utf-8"))
        self.kit = kit
        from .runner.kin import ArmFK

        self.fk = ArmFK.build(self.context["urdf_path"], kit.get("joint_map"), self.envelope)

    def _build(self):
        from .adapters.sim_so101 import ARM_JOINTS, SimSO101Adapter
        from .kinematics import JointMap

        sag, tau = (0.0, 0.0) if self.servos == "ideal" else (1.0, 0.1)
        self.adapter = SimSO101Adapter(
            command_space_path=self.context["command_space_path"],
            urdf_path=self.context["urdf_path"],
            joint_map=JointMap.from_dict(self.kit.get("joint_map")),
            start_pose=self.kit["start_pose_deg"], power_up_goals_zero=False, realtime=False,
            prompt=lambda q: "", log=lambda m: None, sag=sag, tau_s=tau)
        self.adapter.open()
        self.machine = self.adapter.sim_robot
        self.units = {**dict.fromkeys(ARM_JOINTS, "deg"), "gripper": "percent"}
        gate = self.factory(self.machine, dict(self.context)) if self.factory else None
        if gate is not None and hasattr(gate, "clock"):
            gate.clock = lambda: self.t
        return gate

    def advance(self, dt: float) -> None:
        n = max(1, round(dt / self.servo_s))
        for _ in range(n):
            self.adapter.servos.step(dt / n)
        self.t += dt

    def points(self) -> dict:
        return self.fk.points(self.adapter.true_joints(), self.units)

    def state(self) -> dict:
        return {**super().state(), "joints": self.adapter.true_joints()}


# ----- rollouts ---------------------------------------------------------------------------------
def rollout(system: _System, prop: str, seq, hold: int, *, record: bool = False,
            max_steps: int | None = None, cache: _Snapshots | None = None) -> dict:
    """Run one command sequence from a fresh machine. Stops at the first violation (unless
    ``record``: a recorded trace runs to its end, so it shows how deep the violation goes), or
    after ``max_steps``. Returns the robustness, the steps simulated, which segments the gate refused
    part of, and (``record``) every step's command, decision and state.

    With ``cache``, a sequence that shares its first segments with one already run resumes from
    the simulation's snapshot at the end of the shared part, and only the steps simulated after it
    count. That is a simulation-only shortcut: it gives the same robustness as a fresh run."""
    keys = [a.key for a in system.space.axes]
    begin, r_min, refused_in = 0, math.inf, set()
    if cache is not None:
        begin, r_min, refused_in = cache.resume(system, seq)
    else:
        system.reset()
    steps, rows = 0, []
    for s in range(begin, len(seq)):
        cmd = dict(zip(keys, seq[s], strict=True))
        for j in range(hold):
            if max_steps is not None and steps >= max_steps:
                return {"robustness": r_min, "steps": steps, "refused_segments": refused_in,
                        "rows": rows}
            decision = system.send(cmd)
            system.advance(system.period_s)
            steps += 1
            r = system.robustness(prop)
            r_min = min(r_min, r)
            if refused(decision):
                refused_in.add(s)
            if record:
                rows.append({"step": s * hold + j, "command": dict(cmd), "decision": decision,
                             "robustness": None if math.isinf(r) else r, **system.state()})
            if r < -VIOLATION_EPS and not record:
                return {"robustness": r_min, "steps": steps, "refused_segments": refused_in,
                        "rows": rows}
        if cache is not None and s + 1 < len(seq):
            cache.save(system, seq[: s + 1], r_min, refused_in)
    return {"robustness": r_min, "steps": steps, "refused_segments": refused_in, "rows": rows}


class _Snapshots:
    """Simulation snapshots at segment boundaries, keyed by the command prefix (at most ``size``,
    least recently used dropped). A snapshot is a deep copy of the machine and the gate; objects
    that cannot be copied (locks, events) and the read-only models are shared, which is safe
    because the lockstep harness runs on one thread."""

    def __init__(self, size: int = 64):
        self.size = size
        self.items: dict = {}

    def save(self, system: _System, prefix, r_min: float, refused_in: set) -> None:
        key = tuple(tuple(v) for v in prefix)
        self.items.pop(key, None)
        self.items[key] = (system.snapshot(), r_min, set(refused_in))
        while len(self.items) > self.size:
            self.items.pop(next(iter(self.items)))

    def resume(self, system: _System, seq) -> tuple[int, float, set]:
        for n in range(len(seq) - 1, 0, -1):
            key = tuple(tuple(v) for v in seq[:n])
            hit = self.items.get(key)
            if hit is not None:
                self.items[key] = self.items.pop(key)
                snap, r_min, refused_in = hit
                system.restore(snap)
                return n, r_min, set(refused_in)
        system.reset()
        return 0, math.inf, set()


@dataclass
class SearchResult:
    """What one search did. ``found`` means a rollout violated the property; ``trace`` is that
    rollout (or the best one when nothing was found), replayable with :func:`replay`."""

    rule_set: str
    machine: str
    gate: str | None
    property: str
    seed: int
    budget_steps: int
    steps_used: int
    steps_to_violation: int | None
    found: bool
    robustness: float
    rollouts: int
    trace: dict = field(repr=False)

    def to_json(self) -> dict:
        return {"rule_set": self.rule_set, "draft": True, "simulation_only": True,
                "machine": self.machine, "gate": self.gate, "property": self.property,
                "seed": self.seed, "budget_steps": self.budget_steps,
                "steps_used": self.steps_used, "steps_to_violation": self.steps_to_violation,
                "found": self.found, "robustness": self.robustness, "rollouts": self.rollouts,
                "trace": self.trace}


class _Budget(Exception):
    pass


class _Found(Exception):
    pass


def candidates(rng: random.Random, axes) -> list[list[float]]:
    """One segment's candidate commands: every combination of each axis's lower limit, home value
    and upper limit when there are at most 27 of them, otherwise the home command, each axis alone
    at either limit and one random corner per axis; then one uniform draw. Shuffled."""
    if 3 ** len(axes) <= 27:
        out = [list(c) for c in itertools.product(*[(a.lo, a.home, a.hi) for a in axes])]
    else:
        home = [a.home for a in axes]
        out = [home]
        for k, a in enumerate(axes):
            for v in (a.lo, a.hi):
                c = list(home)
                c[k] = v
                out.append(c)
        out += [[a.hi if rng.random() < 0.5 else a.lo for a in axes] for _ in axes]
    out.append([rng.uniform(a.lo, a.hi) for a in axes])
    rng.shuffle(out)
    return out


def falsify(system: _System, prop: str = "position", *, seed: int = 0, budget: int = 5000,
            segments: int = 12, hold_steps: int = 15, bisect_iters: int = 3,
            delta: float = 0.25, halvings: int = 2) -> SearchResult:
    """Search for a command sequence that makes ``system`` violate ``prop`` (module docstring).
    ``budget`` is the total number of steps over every rollout."""
    if prop not in PROPERTIES:
        raise ValueError(f"property is one of {PROPERTIES}")
    rng = random.Random(seed)
    axes = system.space.axes
    cache = _Snapshots()
    st = {"used": 0, "count": 0, "best": (math.inf, [])}

    def evaluate(seq):
        left = budget - st["used"]
        if left <= 0:
            raise _Budget
        out = rollout(system, prop, seq, hold_steps, max_steps=left, cache=cache)
        st["used"] += out["steps"]
        st["count"] += 1
        if out["robustness"] < st["best"][0]:
            st["best"] = (out["robustness"], [list(v) for v in seq])
        if out["robustness"] < -VIOLATION_EPS:
            raise _Found
        return out

    def score(seq, c):
        out = evaluate([*seq, c])
        return out["robustness"], len(seq) in out["refused_segments"]

    def climb():
        seq: list[list[float]] = []
        for _ in range(segments):
            # (a) the candidates, each held for one segment from where the prefix left the machine
            scored = []
            for c in candidates(rng, axes):
                r, refused_here = score(seq, c)
                scored.append((r, refused_here, c))
            # (b) bisection on the accept/reject boundary, between the best command the gate
            # accepted throughout and the best one it refused part of
            ok = [x for x in scored if not x[1]]
            no = [x for x in scored if x[1]]
            if bisect_iters and ok and no:
                ca = min(ok, key=lambda x: x[0])[2]
                cr = min(no, key=lambda x: x[0])[2]
                lo, hi = 0.0, 1.0
                for _ in range(bisect_iters):
                    lam = (lo + hi) / 2
                    c = [x + lam * (y - x) for x, y in zip(ca, cr, strict=True)]
                    r, refused_here = score(seq, c)
                    scored.append((r, refused_here, c))
                    if refused_here:
                        hi = lam
                    else:
                        lo = lam
            r, _, c = min(scored, key=lambda x: x[0])
            # (c) coordinate search on this segment's command
            d = delta
            for _ in range(halvings):
                improved = False
                for k in rng.sample(range(len(axes)), len(axes)):
                    a = axes[k]
                    for sign in (1.0, -1.0):
                        v = min(a.hi, max(a.lo, c[k] + sign * d * (a.hi - a.lo)))
                        if v == c[k]:
                            continue
                        t = list(c)
                        t[k] = v
                        rt, _ = score(seq, t)
                        if rt < r:
                            r, c, improved = rt, t, True
                            break
                if not improved:
                    d /= 2
            seq.append(c)

    found = False
    try:
        while True:  # (d) restart
            climb()
    except _Budget:
        found = False
    except _Found:
        found = True
    r, seq = st["best"]
    trace = _trace(system, prop, seq, hold_steps) if seq else {}
    if found and not trace["robustness"] < -VIOLATION_EPS:  # a snapshot that was not faithful
        raise RuntimeError("the violation did not reproduce from a fresh machine")
    return SearchResult(RULE_SET, system.name, system.gate_spec, prop, seed, budget, st["used"],
                        st["used"] if found else None, found, r, st["count"], trace)


def _trace(system: _System, prop: str, seq, hold: int) -> dict:
    out = rollout(system, prop, seq, hold, record=True)
    return {"period_s": system.period_s, "hold_steps": hold, "segments": seq,
            "context": _portable(system.context), "machine_options": _options(system),
            "robustness": out["robustness"], "steps": out["rows"]}


def _options(system: _System) -> dict:
    if isinstance(system, RoverSystem):
        return {"start": list(system.start)}
    if isinstance(system, ArmSystem):
        return {"servos": system.servos}
    return {}


def _portable(context: Mapping) -> dict:
    """Context paths relative to the current directory when they are under it (for a trace file
    that replays from the repo root)."""
    out = {}
    for k, v in context.items():
        if isinstance(v, str) and k.endswith("_path"):
            p = Path(v).resolve()
            try:
                v = str(p.relative_to(Path.cwd().resolve()))
            except ValueError:
                v = str(p)
        out[k] = v
    return out


# ----- replay -----------------------------------------------------------------------------------
def build_system(machine: str, gate: str | None, context: Mapping, options: Mapping | None = None):
    options = dict(options or {})
    if machine == RoverSystem.name:
        return RoverSystem(gate, context, start=tuple(options.get("start", (1.0, 1.0, 0.0))))
    if machine == ArmSystem.name:
        return ArmSystem(gate, context, servos=options.get("servos", "sag"))
    raise ValueError(f"unknown machine {machine!r}")


def replay(doc: Mapping, *, root: str | Path = ".", atol: float = 0.0) -> dict:
    """Re-run a trace file's commands on a fresh machine and gate and compare every decision and
    state with the recorded ones: decisions and every other non-number exactly, numbers to within
    ``atol`` (0: bit for bit; a trace written on another platform may differ in the last bits of
    its trigonometry). Returns ``{"matches": bool, "first_mismatch": step or None,
    "robustness": float, "violated": bool}``. Context paths in the file are relative to
    ``root``."""
    tr = doc["trace"]
    ctx = {k: (str(Path(root) / v) if isinstance(v, str) and k.endswith("_path") else v)
           for k, v in tr["context"].items()}
    system = build_system(doc["machine"], doc["gate"], ctx, tr.get("machine_options"))
    out = rollout(system, doc["property"], tr["segments"], tr["hold_steps"], record=True)
    rows = json.loads(json.dumps(out["rows"]))
    first = None
    for i, (a, b) in enumerate(zip(rows, tr["steps"], strict=False)):
        if not _same(a, b, atol):
            first = i
            break
    if first is None and len(rows) != len(tr["steps"]):
        first = min(len(rows), len(tr["steps"]))
    return {"matches": first is None, "first_mismatch": first, "robustness": out["robustness"],
            "violated": out["robustness"] < -VIOLATION_EPS}


def _same(a: object, b: object, atol: float) -> bool:
    if isinstance(a, bool) or isinstance(b, bool) or a is None or b is None:
        return a == b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return a == b or abs(a - b) <= atol
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(_same(a[k], b[k], atol) for k in a)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(_same(x, y, atol) for x, y in zip(a, b, strict=True))
    return a == b


# ----- the random baseline ----------------------------------------------------------------------
def random_baseline(system: _System, prop: str = "position", seeds: Sequence[int] = (1,),
                    seconds: float = 600.0) -> dict:
    """``be-hostile/1`` streams at the default parameters, one seed after another, each from a
    fresh machine, driven as the gates' own lockstep tests drive them (advance the physics to the
    proposal's scheduled time, then send it). Counts steps (commands handed to the gate) until the
    first violation. ``{"found": bool, "seed": int or None, "steps": total steps,
    "steps_in_seed": int or None, "t_s": float or None}``."""
    from bounded_embodiment.generator import hostile_stream

    total = 0
    for seed in seeds:
        system.reset()
        prev = 0.0
        for n, p in enumerate(hostile_stream(system.space, seed, {"seconds": seconds}), 1):
            system.advance(p.t_s - prev)
            prev = p.t_s
            system.send(p.payload)
            total += 1
            if system.robustness(prop) < -VIOLATION_EPS:
                return {"found": True, "seed": seed, "steps": total, "steps_in_seed": n,
                        "t_s": p.t_s}
    return {"found": False, "seed": None, "steps": total, "steps_in_seed": None, "t_s": None}


# ----- command line -----------------------------------------------------------------------------
def main(argv: Sequence[str] | None = None) -> int:
    """``python -m bounded_embodiment_kit.falsify``: run one search and write its trace."""
    p = argparse.ArgumentParser(prog="python -m bounded_embodiment_kit.falsify",
                                description="be-falsify/1 (draft): a falsification search over a "
                                "gate in the kit's simulation. Simulation only.")
    p.add_argument("--machine", choices=[RoverSystem.name, ArmSystem.name], required=True)
    p.add_argument("--gate", help="module:factory (omit for the machine with no gate)")
    p.add_argument("--gate-path", action="append", default=[],
                   help="a directory to put on sys.path first, e.g. examples/reference-gates")
    p.add_argument("--envelope", required=True)
    p.add_argument("--command-space", required=True)
    p.add_argument("--kit-config")
    p.add_argument("--urdf")
    p.add_argument("--servos", choices=["ideal", "sag"], default="sag", help="sim-so101 only")
    p.add_argument("--property", choices=PROPERTIES, default="position")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--budget", type=int, default=5000, help="steps (commands sent), all rollouts")
    p.add_argument("--segments", type=int, default=12)
    p.add_argument("--hold-steps", type=int, default=15)
    p.add_argument("--out", help="the trace file to write (JSON); required for a search")
    p.add_argument("--random-seeds", metavar="A-B",
                   help="instead of searching, run the be-hostile/1 baseline over seeds A to B "
                   "(or one seed) and print the steps to the first violation")
    a = p.parse_args(argv)
    if not a.random_seeds and not a.out:
        p.error("--out is required for a search")
    for d in reversed(a.gate_path):
        sys.path.insert(0, str(Path(d).resolve()))
    ctx = {"mode": "full", "speed_factor": None, "envelope_path": a.envelope,
           "command_space_path": a.command_space, "kit_config_path": a.kit_config,
           "urdf_path": a.urdf}
    opts = {"servos": a.servos} if a.machine == ArmSystem.name else {}
    system = build_system(a.machine, a.gate, ctx, opts)
    if a.random_seeds:
        lo, _, hi = a.random_seeds.partition("-")
        seeds = range(int(lo), int(hi or lo) + 1)
        print(json.dumps({"baseline": "be-hostile/1", "property": a.property, "seeds":
                          [seeds.start, seeds.stop - 1],
                          **random_baseline(system, a.property, seeds)}))
        return 0
    res = falsify(system, a.property, seed=a.seed, budget=a.budget, segments=a.segments,
                  hold_steps=a.hold_steps)
    write_trace(res, a.out)
    print(json.dumps({k: v for k, v in res.to_json().items() if k != "trace"}))
    return 0


def write_trace(res: SearchResult, path: str | Path) -> None:
    """The result as JSON, keys sorted, one recorded step per line."""
    doc = res.to_json()
    doc["trace"] = dict(doc["trace"])
    steps = doc["trace"].pop("steps", [])
    text = json.dumps(doc, indent=1, sort_keys=True)
    lines = ",\n".join(json.dumps(r, sort_keys=True, separators=(",", ":")) for r in steps)
    marker = '"trace": {'
    i = text.index(marker) + len(marker)
    text = f'{text[:i]}\n  "steps": [\n{lines}\n  ],{text[i:]}'
    json.loads(text)  # still one JSON document
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(text + "\n", encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
