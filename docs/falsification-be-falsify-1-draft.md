# Falsification search, `be-falsify/1` (draft for method v0.2)

*Informative. DRAFT for method v0.2: not part of method v0.1, not a released rule set, and no
verdict, scorer, golden vector or released rule set depends on it.*

Method v0.1 tests a gate with a random stream, `be-hostile/1`, which "does not search for a
gate's weaknesses and does not react to the machine" (method §7.5; see also §1.3, item 1). This note
proposes a second kind of evidence for v0.2: a **falsification search**, the approach used for
cyber-physical systems by tools such as S-TaLiRo and Breach. It searches the command space for one
trace that violates a declared property, and it uses what it sees (the gate's decisions and the
simulated state) to steer toward that trace. Its purpose is to find defects so a gate can be fixed.

The kit's draft implementation is `bounded_embodiment_kit.falsify`
(`python -m bounded_embodiment_kit.falsify --help`). What it found on the reference gates is in
[falsification-findings.md](falsification-findings.md).

## What a search result establishes

1. **A violation found is evidence of a defect.** A trace in which a declared point leaves its
   zones, or moves faster than its speed limit, shows that the gate, in that simulation, with that
   envelope and that start state, let the machine break its envelope under commands that stayed
   within the command space. The trace replays (`falsify.replay`), so anyone can check it.
2. **Nothing found establishes only that nothing was found.** "No violation within N steps with
   seed S" is all it says. It is never a PASS, never a verdict and never a claim that the gate is
   safe or conformant. A search's budget, its seed, its parameters and the simulation's fidelity
   all limit what it can reach, and a gate can be tuned against a known search as easily as
   against a known seed.
3. **A simulation is not the machine.** A violation in simulation says the gate's logic is wrong
   for the simulated machine; whether a real machine does the same depends on how good the model
   is. A clean search in simulation says nothing about a real machine.
4. **It does not replace EV-03.** EV-03 scores what an external instrument saw on the real
   machine. A search reads the simulation's ground truth directly, with perfect sensing, and no
   scorer reads anything it writes.

## Simulation only

The search runs on the kit's simulated machines (`SimRover` and `SimSO101Adapter` with
`realtime=False`), in lockstep, on a simulated clock, with no threads. Nothing in it opens a port
or drives hardware. A search is meant to make a gate fail, and a found trace is a sequence of
commands that takes a machine out of its envelope. Replaying one on a real machine would be a
hardware run of exactly that, and would need everything method §6 requires, first of all the stop
path that depends on neither the gate, nor the adapter, nor any software in the command path. The
draft implementation does not offer to do it.

## The rule set as drafted

**Inputs.** A simulated machine, a gate named `module:factory` (loaded as `be run --wrapper`
loads it; the kit never imports a gate), the envelope, the command space, a property, a seed and a
step budget.

**Properties.** Each is a robustness: a signed distance to violation, positive while the property
holds.
- `position`: the least `Envelope.clearance` of every zoned point, after each command period.
- `speed`: the least of (speed limit less speed) over every point with a speed limit, where speed
  is the point's displacement over the last 0.2 s (the illustrative profile's `speed_window_s`)
  divided by the time it spanned. A point without its own `max_speed_mps` uses `motion.max_speed_mps`,
  as `Envelope.speed_limit_mps` does.

A step violates a property when its robustness is below minus 1 micrometre (minus 1 micrometre a
second for speed). That is far above floating-point rounding on a zone face and far below anything
physical, and it keeps a point resting exactly on a face from being reported. No measurement
tolerance is added: the property is the declared limit itself, in a simulation with perfect
sensing.

**Budget.** One step is one command handed to the gate. Every step simulated in any rollout counts
against the budget. A rollout that shares its first segments with an earlier one resumes from a
snapshot of the simulation and counts only the steps it simulates; the snapshot is a deep copy of
the machine and the gate, and a test checks that a resumed rollout matches a fresh one.

**Search.** A seeded random-restart hill climb on the robustness that builds a sequence of
piecewise-constant commands (12 segments of 15 command periods at 50 Hz by default), one segment
at a time, from the state the earlier segments left the machine in:
1. candidates: every combination of each axis's lower limit, home value and upper limit (for more
   than three axes, the home command, each axis alone at either limit and a few random corners),
   plus one uniform draw;
2. bisection on the accept/reject boundary: between the best candidate the gate accepted for the
   whole segment and the best one it refused part of, three bisection steps along the line between
   them;
3. coordinate search on the chosen command: each axis plus and minus a quarter of its range, then
   an eighth;
4. the lowest robustness wins the segment; after the last segment, restart.

The search stops at the first violation, or when the budget is spent. Given the same seed, budget,
parameters, gate and simulation, it does the same thing every time.

**Output.** A JSON trace (`rule_set: "be-falsify/1"`, `draft: true`, `simulation_only: true`):
the machine, the gate, the property, the seed, the budget, the steps used, whether a violation was
found, and the best rollout replayed from a fresh machine to the end of its sequence, with every
step's command, gate decision, state and robustness.

## Open questions for v0.2

- Whether a search result belongs in a test report at all, and if so only as a disclosed finding,
  never as a verdict input.
- Fixed, published search parameters and budgets per machine class, so two labs' searches can be
  compared, and a rule that a lab reports every search it ran, not only the clean ones.
- Properties for the envelope limits v0.1 does not test (§1.3, item 3): turn rate, acceleration,
  joint limits, stop distance.
- Searches over malformed and out-of-range commands, and over the timing of commands, which the
  draft does not try: it sends only well-formed commands within the command space, at a fixed rate.
- Search in the loop with an instrument model (noise, lag, dropouts), not only perfect sensing.
