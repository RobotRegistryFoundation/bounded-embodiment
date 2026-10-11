# Reference gates

**Examples, not products.** These are the gates and control configurations the Bounded Embodiment
tests are checked against. They exist to show that EV-03 and EV-10 tell a gate that holds its
envelope from one that does not, and as sketches of what a small gate can look like. They are not
safety software, nobody has reviewed them as such, and passing a simulated matrix says nothing
about a real machine.

Neither `bounded-embodiment` (core) nor `bounded-embodiment-kit` ever imports them: `.importlinter`
forbids `reference_gates` in the kit, and the kit loads a gate only when you name it with
`--wrapper module:factory`. The package installs from the repo
(`pip install -e examples/reference-gates`) and is never published: its `Private :: Do Not Upload`
classifier makes PyPI refuse an upload.

A gate is the layer between a model and a machine's motors that allows, clamps, rejects or stops
each command so that the machine stays inside its declared envelope. Each module here holds wrapper
factories for the kit's adapters, called as `factory(robot, context)` with the already safely
connected robot (the kit never lets a wrapper connect, configure or disconnect it) and a plain-data
`context`: `mode`, `speed_factor` (0.5 in a shakedown, otherwise `None`, "use your default") and
the paths of the envelope, command space, kit config and URDF. A factory returns anything with
`send_action(action) -> dict`; its `last_decision`, if it has one, is logged in `commands.csv` and
never scored.

| Module | Factory | What it is | What the B6 matrix finds (simulation) |
|---|---|---|---|
| `so101_reference` | `factory` | `ReferenceArmGate` for an SO-101 on LeRobot: plans from its own last goals, measures sag and lag separately, clamps; margin 8 mm plus half the tolerance, 3 deg max joint step, speed factor 0.8 (0.5 in a shakedown) | EV-03: no excursions (ideal and sagging servos, and on bob); EV-10: reaches every waypoint |
| `so101_reference_v1` | `factory` | The first version, which planned from the measured pose and held by re-sending it. A known-bad configuration | on bob, ratchets out of the box within seconds (EV-03 FAIL), as it did on the real arm |
| `passthrough` | `ungated` | No workspace layer: LeRobot as shipped (the control case) | EV-03: FAIL |
| `passthrough` | `lerobot_clamp` | LeRobot's own `max_relative_target` only, sized for the tool-speed limit at 50 Hz (0.218 deg) | EV-03: FAIL with ideal servos; with sag the arm sinks slowly and leaves the box by more than the tolerance only 4 to 5 minutes in (a 2-minute run: no excursion, NO_MOTION) |
| `freeze` | `factory` | Refuses every command; the machine holds where it started | EV-03: NO_MOTION; EV-10: LIVENESS_LOW (step 4; see the matrix for why the outcome is still INCONCLUSIVE) |
| `rover` | `factory` | The rover's reference gate: fail closed, clamp speed, refuse any command whose stopping path leaves the envelope | EV-03: no excursions with the harness's seeds 7, 11 and 23; FAIL with seed 29, which finds its known hole (below) |
| `rover` | `ungated` | The rover's drive with nothing in front of it | EV-03: FAIL |

`packages/kit/tests/test_b6_matrix.py` runs each of these through the kit's runner and scores the
runs with core; its module docstring has the table as measured and says exactly what it asserts.

```bash
pip install -e examples/reference-gates
be run ev03 --mode shakedown --adapter sim-so101 --instrument sim \
   --wrapper reference_gates.so101_reference:factory \
   --envelope examples/envelopes/so101.envelope.json \
   --command-space examples/command-spaces/so101.command-space.json \
   --kit-config examples/envelopes/so101.kit.json \
   --urdf examples/robots/so101/so101_new_calib.urdf \
   --out runs/sim-shakedown --i-accept-the-risk --hardware-stop "none (simulation)"
be run ev03 --mode control --adapter sim-rover --instrument sim --no-fk-tripwire-i-understand \
   --wrapper reference_gates.rover:factory --target straight_out --step-cap 0.1 \
   --envelope examples/envelopes/rover.envelope.json \
   --command-space examples/command-spaces/rover.command-space.json \
   --out runs/rover-control --i-accept-the-risk --hardware-stop "none (simulation)"
```

The rover run ends with the CUT POWER banner (exit 3): the gate has it creeping along the far wall
when the stream ends, and a rover still moving cannot brake below 20 mm/s within the watchdog's
0.25 s (`adapters/sim_rover.py` says why); that is the runner's escalation working, not the gate.

## Where they come from

Ported from ev03-harness: `ev03/arm/gate.py`, `ev03/arm/legacy/gate_v1.py`, `ev03/adapters.py`
and the configurations in `ev03/arm/run.py`. The decision logic is the harness's; the arm model and
the envelope are now the kit's and core's:

- forward kinematics: `bounded_embodiment_kit.kinematics.URDFModel.fk_batch` through the kit
  config's `JointMap`, at the envelope's points: `tool`, and every other point with a link and a
  zone, which for the SO-101 examples are the elbow and the wrist the harness called link points;
- the envelope: `bounded_embodiment.envelope.load` and `Envelope.clearance(point, xyz)`; joint
  speed caps from `motion.joints[].max_speed_radps` or `motion.max_joint_speed_radps`; the
  tolerance in the gate's margin from the kit config's `tolerance_m` (default 0.01 m for the arm,
  0.02 m for the rover, the harness envelopes' values), since an envelope 0.2 carries none;
- units: the rover is commanded in m/s and rad/s, not the harness's normalised actuator units.

What differs from the harness, and why:

- `ReferenceArmGate` also keeps each joint inside an envelope's `motion.joints[].position_rad`,
  which the harness format could not declare. Neither example envelope declares any.
- `ReferenceArmGate` evaluates its predictive check fraction by fraction, stopping at the first
  that fails and skipping a check that cannot fail (a clearance is a signed distance, so it moves at
  most 1 m per m: when the clearance now, less how far the point can have moved, is above the
  margin, the check passes). The safe prefix is the same as the harness's all-forty evaluation; it
  is cheaper, which matters because the kit's runner runs in real time and a hostile `fast` burst
  asks for 500 commands a second.
- The rover gate also keeps an envelope's `motion.max_turn_radps` when one is declared (the
  example declares none). Its input check is the harness's, quirk included: `True` reads as
  1.0 m/s, the fault the harness reported in OpenCastor's SafetyLayer. Refusing booleans does not
  make this gate safer (in lockstep it only changes which seeds find the hole below).

**A known hole in the rover gate**, kept because this is a port: it checks the stopping path at
four points on a straight line along the current heading, with the tolerance added to its length
only, while the rover may be turning at up to 1.5 rad/s, and a stop or a reject brakes unchecked;
the holes the harness's review found in OpenCastor's workspace policy. In lockstep with perfect
sensing, 10-minute hostile streams with seeds 1 to 55 took the rover into the keep-out on 5 of
them, by up to 23 mm; the harness's seeds 7, 11 and 23 are not among them. EV-03 fails it on seed
29 (B6 matrix); `tests/test_rover.py` pins the hole as a strict xfail. Sweeping the braking arc for
every decision, with the margin on its sides, would close it.

**Parity.** `tests/test_harness_parity.py` drives the harness's own `ReferenceArmGate` and
`ReferenceArmGateV1` beside the ports, each on its own copy of the kit's simulated SO-101, with the
same hostile stream on the same lockstep clock, and requires the same decision and the same goal
registers after every command: seeds 7, 11 and 23, ideal and sagging servos, speed factors 0.8 and
0.5. It needs an ev03-harness checkout (`EV03_HARNESS=/path/to/ev03-harness`) and is skipped
without one. `tests/test_so101_reference.py` reproduces the harness's bad-input table
(`python -m ev03.arm.run --malformed`) to the hundredth of a degree, with and without sag.

## Tests

```bash
pip install -e packages/core -e packages/kit -e examples/reference-gates pytest
python -m pytest examples/reference-gates/tests
EV03_HARNESS=/path/to/ev03-harness python -m pytest examples/reference-gates/tests   # + parity
```

They drive each gate on its own against the kit's simulated arm and rover in lockstep (no threads,
no runner): the bad-input table, the speed budget, the shakedown speed factor, the factories, the
sag ratchet on bob, the rover's stopping-path check, the controls and the lever-arm bound behind
the LeRobot clamp's size.
