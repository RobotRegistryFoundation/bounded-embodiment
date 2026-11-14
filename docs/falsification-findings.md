# Falsification findings: the reference gates

*Informative. Results of the draft rule set `be-falsify/1`
([falsification-be-falsify-1-draft.md](falsification-be-falsify-1-draft.md)) against the gates in
`examples/reference-gates/`, measured on 24 Oct 2026. Simulation only: nothing here ran on
hardware, and a hardware run of any trace below would need the independent stop path of method §6.
No released rule set, scorer, golden vector or verdict changed, and no gate changed.*

A found violation is evidence of a defect in the gate, in that simulation. "Not found" means only
that the search found nothing within that budget with that seed. It is never a PASS.

## Results

Every trace replays: `packages/kit/tests/test_falsify.py` replays each file below and checks that
the ones marked found still violate their property. Paths are relative to
`packages/kit/tests/fixtures/falsification/`. Budgets and steps count commands handed to the gate,
over every rollout. "Closest" is the best robustness the search reached: for `position` the least
clearance of any zoned point (negative: outside), for `speed` the least margin under the speed
limit (negative: over it). For a found violation, the depth is how far the found trace goes past
the limit when it is played to the end of its sequence.

| Gate | Machine | Property | Seed | Budget | Found | Steps | Closest / depth | Trace |
|---|---|---|---|---|---|---|---|---|
| `rover:factory` | sim-rover | position | 0 | 20000 | **yes** | 5499 | 21.5 mm inside the keep-out | `rover-reference-position-seed0.json` |
| `rover:factory` | sim-rover | speed | 0 | 20000 | no | 20000 | 0.0 m/s under the limit (at it, never over) | `rover-reference-speed-seed0.json` |
| `so101_reference:factory`, sag servos | sim-so101 | position | 0 | 20000 | no | 20000 | 8.2 mm clearance (tool above the box floor) | `arm-reference-sag-position-seed0.json` |
| `so101_reference:factory`, sag servos | sim-so101 | speed | 0 | 20000 | no | 20000 | 0.023 m/s under the limit (wrist, 0.227 m/s) | `arm-reference-sag-speed-seed0.json` |
| `so101_reference:factory`, ideal servos | sim-so101 | position | 0 | 20000 | no | 20000 | 13.0 mm clearance (the gate's margin) | `arm-reference-ideal-position-seed0.json` |
| `rover:ungated` (control) | sim-rover | position | 0 | 2000 | yes | 1033 | 37 mm outside | `rover-ungated-position-seed0.json` |
| `rover:ungated` (control) | sim-rover | speed | 0 | 2000 | yes | 300 | over the limit (the trace stops there) | `rover-ungated-speed-seed0.json` |
| `passthrough:ungated` (control) | sim-so101, sag | position | 0 | 2000 | yes | 9 | 158 mm outside | `arm-ungated-sag-position-seed0.json` |
| `passthrough:ungated` (control) | sim-so101, sag | speed | 0 | 2000 | yes | 11 | 1.14 m/s over the limit | `arm-ungated-sag-speed-seed0.json` |

Gates are named as `module:factory` under `reference_gates.`. The sim-so101 machine starts at the
kit config's start pose, powered and holding; "sag" is the simulator's default servo model (bob's
fitted gravity sag and friction, 0.1 s lag), "ideal" has neither. The sim-rover machine starts at
(1, 1) heading +x. Each control case is found within a few hundred steps at most, which shows the
search and the properties can see a violation when there is one.

## The rover sanity check: the documented hole, rediscovered

The rover gate's known hole (`examples/reference-gates/README.md`): it checks the stopping path at
four points on a straight line along the current heading while the rover may be turning at up to
1.5 rad/s, and a stop or a reject brakes unchecked, with the turn kept.

**What the search found** (seed 0): drive straight ahead from (1, 1) at the speed limit along the
line of the keep-out's lower face (y = 1), and 2.1 s in, near the keep-out's corner (2, 1), command
full speed and a 1.5 rad/s left turn. The gate passes the first turning command (clamped to
0.5 m/s); one command later the heading has swung toward the keep-out, its straight-line check
fails and it rejects, which sets the linear speed to 0 and keeps the turn. The rover brakes from
0.5 m/s along an arc that nothing checks and enters the keep-out at 2.28 s, 21.5 mm deep at
worst, with perfect sensing.
That is the documented mechanism (the harness's seed 29 reached 23 mm by backing into the same
corner). The search's commands stay within the command space; the gate clamps the linear speed to
0.5 m/s as it should.

**Search against random, in steps** (commands handed to the gate):

| Method | Steps to the first violation |
|---|---|
| `be-falsify/1`, seed 0 | 5499 |
| `be-falsify/1`, seeds 0 to 9 | all 10 found; median 5492, mean 5471, range 1945 to 8994 |
| `be-hostile/1`, seed 29 alone (the seed already known to find it) | 9737 (68.29 s into the stream) |
| `be-hostile/1`, seeds 1, 2, 3, ... in turn, 600 s each (no seed known in advance) | 1,211,530 (first found in seed 19, 46,117 steps into it) |

Every one of the ten searches is below the 9737 steps of seed 29, the seed the README names because
it was already known to find the hole; against random search that does not know the seed in
advance the gap is a factor of about 200. `test_falsify.py` pins both: seed 0 within 9736 steps,
and the hostile baseline at exactly 9737 for seed 29.

Honest caveats:
- All ten searches find the same corner by the same route. The rover's start pose lies on the line
  of the keep-out's lower face, and the search's candidate commands include "angular exactly 0",
  which keeps it on that line. A different start pose would make the search work harder; how much
  harder is not measured here.
- In 9 of the 10 the gate was refusing (braking unchecked) at the first step outside; in seed 5 it
  was passing a clamped command while the rover turned, the other half of the same hole (the
  straight-line check missing the arc).
- The bisection step did not help on this gate: without it (`bisect_iters=0`) the same ten seeds
  need a median of 4734 steps (range 1810 to 7554). It is kept because it is cheap and aims at the
  gate's decision boundary, which is where a gate with a different weakness may be thin; whether it
  pays for itself on other gates is an open question.

## New findings against the reference gates

None. Within these budgets the search found no violation against `so101_reference:factory` (with
ideal or sagging servos) and no speed violation against `rover:factory`. The only violation it
found against a reference gate is the rover's documented hole above, which the README already
records with its proposed fix: sweep the braking arc for every decision, including a stop and a
reject, with the margin added on its sides as well as its length. That fix is not made here.

Observations, not findings (no property was violated):
- The arm with sagging servos came to 8.2 mm above the keep-in box's floor, against a gate margin
  of 13 mm (half the 10 mm tolerance plus 8 mm): sag used 4.8 mm of the margin. With ideal servos
  the closest approach is exactly the margin. This is the margin doing its job, as the gate's
  docstring says it should.
- The fastest point in the arm's speed search was the wrist at 0.227 m/s over 0.2 s, 91 % of
  `motion.max_speed_mps` (0.25 m/s), which applies to the wrist and the elbow because they declare
  no limit of their own. The gate budgets only the tool's displacement; the wrist is held back only
  by the 90 deg/s joint speed cap. A longer or differently shaped search is the place to look next.

## Reproducing

From the repo root, one run at a time (each arm run takes about a minute on a Raspberry Pi 5):

```bash
G="python -m bounded_embodiment_kit.falsify --gate-path examples/reference-gates"
ROVER="--machine sim-rover --envelope examples/envelopes/rover.envelope.json \
  --command-space examples/command-spaces/rover.command-space.json"
ARM="--machine sim-so101 --envelope examples/envelopes/so101.envelope.json \
  --command-space examples/command-spaces/so101.command-space.json \
  --kit-config examples/envelopes/so101.kit.json --urdf examples/robots/so101/so101_new_calib.urdf"
OUT=packages/kit/tests/fixtures/falsification
$G $ROVER --gate reference_gates.rover:factory --property position --budget 20000 \
  --out $OUT/rover-reference-position-seed0.json
$G $ARM --gate reference_gates.so101_reference:factory --servos sag --property speed \
  --budget 20000 --out $OUT/arm-reference-sag-speed-seed0.json
# the hostile baseline: seed 29 alone, then seeds 1 to 19 in turn
$G $ROVER --gate reference_gates.rover:factory --random-seeds 29
$G $ROVER --gate reference_gates.rover:factory --random-seeds 1-19
```

The other rows change `--gate`, `--property`, `--servos` and `--budget` as the table shows.
