# Loophole search: where the letter passes and the intent breaks

*Informative. Not part of the normative method text. Written 24 October 2026 against method
v0.1, envelope 0.2, `be-hostile/1`, `be-control/1`, `be-waypoints/1`, `ev03-tool-point/1` and
`ev10-waypoints/1`, core 0.1.1.*

This page records a search for places where an envelope, a run or a report meets every rule of
the method as written, and the result still breaks what the rule is for. Each finding is either
a **regression vector** (the current rules give the intended answer, and a vector now pins it)
or a **defect** (they do not). Released rule sets are never changed to fix a defect (method
§1.5, §11): each defect below proposes a fix for the next version of the rule set, and its
vector keeps pinning what the released version does today.

Two searches were made.

1. **Envelope semantic search.** `tools/envelope_loopholes.py` generates seeded envelopes (0.1
   and 0.2) that pass the JSON schema, each with one problem class applied, and checks that the
   loader rejects it or that the new `be envelope lint` reports it. One vector per class and
   version is published in `vectors/envelope/lint/`.
2. **Scorer and verifier search.** `tools/make_loophole_runs.py` builds signed synthetic run
   directories in `vectors/loopholes/runs/` and pins what `be verify` finds for each in
   `vectors/loopholes/runs/expect.json`. Each entry says whether it is a regression or a defect;
   a defect entry also gives the intended verdict and the defect id used below.

Both are TEST-ONLY synthetic data in the formats of the golden runs (`vectors/runs/`) and use
the golden runs' public test keys. The run vectors use a TEST-ONLY profile whose
`min_duration_s` (3 s) equals the length of their 3 s stream, as the packaged profile's 600 s
equals the full run's, so they carry `NONSTANDARD_PARAMS` and nothing else differs. Each run is
a few seconds; the numbers that matter (sample rates, spacings, speeds and clearances) are those
a 600 s run would have.

## Severity

| Severity | Meaning |
|---|---|
| **Unsafe PASS** | A run in which a point may have left its zones, or moved faster than its limit, gets PASS. |
| **Unsound PASS** | A PASS that the evidence does not support (a test not fully made, a measurement not shown to be external, a vacuous limit), without a shown excursion. |
| **No PASS** | The verdict is right or no PASS results, but a reason, report or check is missing or wrong. |

## Summary

| Id | Defect | Severity | Repro |
|---|---|---|---|
| L1 | Spacings at or below the gap threshold are never bounded | Unsafe PASS | `defect-hidden-in-short-spacing` |
| L2 | Speed is taken only over the longest span within the window | Unsafe PASS | `defect-speed-burst-under-window` |
| L3 | σ_jitter has no upper limit and is never compared with the run's own scatter | Unsafe PASS | `defect-jitter-inflates-allowance` |
| L4 | v_phys is never compared with the speeds the run measured | Unsafe PASS | `defect-vphys-contradicted` |
| L5 | Samples outside the scoring window are dropped without a word | Unsafe PASS | `defect-excursion-after-window`, `defect-excursion-before-window` |
| L6 | An aborted run sent late can cover the minimum duration without the end of its stream | Unsound PASS | `defect-prefix-sent-slowly` |
| L7 | The robot-state copy check is evaded by 0.2 mm or 6 ms, and is opt-in | Unsound PASS | `defect-copied-track-offset`, `defect-copied-track-shifted` |
| L8 | A keep_out zone no deeper than tolerance_m can be crossed with no excursion | Unsafe PASS | `defect-through-thin-keep-out` |
| L9 | The envelope's integrator signature is never checked | No PASS (provenance) | `defect-envelope-signature-unchecked` |
| L10 | Token motion meets the EV-03 motion rule; nothing checks the paired EV-10 run | Unsound PASS | `defect-token-motion` |
| L11 | An aborted EV-10 tour leaves declared waypoints unvisited and unmissed | Unsound PASS | `defect-ev10-aborted-unvisited-waypoint` |
| L12 | Repeated timestamps make the median spacing 0 and switch off `INSTRUMENT_RATE_LOW` | No PASS | `defect-duplicate-timestamps` |
| L13 | The loader accepts an envelope whose allowed region is empty | No PASS | `vectors/envelope/lint/keep-out-covers-keep-in-*`, `keep-in-disjoint-v02` |
| L14 | The loader accepts speed limits that no run can test | Unsound PASS | `vectors/envelope/lint/speed-*` |
| L15 | The loader accepts contradictory limits that v0.1 does not test | No PASS (today) | `vectors/envelope/lint/proximity-*`, `stop-*`, `joint-*`, `keep-in-unbounded-z-*` |

Run vectors are under `vectors/loopholes/runs/`; envelope vectors under `vectors/envelope/lint/`.

## Defects in the scorers and the verifier

### L1 Spacings at or below the gap threshold are never bounded

**Problem.** The gap rule (§7.8.6) asks for a bound only on a *gap*, a spacing above
max(`gap_min_s`, 3 × median spacing). Every shorter spacing is accepted with no bound. With a
median spacing of 0.08 s (12.5 Hz, above the 10 Hz minimum) the threshold is 0.24 s, so one
invalid frame (a 0.16 s spacing) is not a gap. At v_phys = 0.5 m/s a point can travel
v_phys × s / 2 = 0.04 m out and back in 0.16 s. Even the regular 0.08 s spacing allows 0.02 m,
more than the 0.015 m (clearance plus tolerance) that the gap rule would demand. The note under
§7.8.6 says a bounded gap cannot hide an excursion; the rule never asks whether a spacing that
is not a gap can.

**Repro.** `vectors/loopholes/runs/defect-hidden-in-short-spacing`: the tool dwells 5 mm inside
the x face for 1.7 s, the instrument runs at 12.5 Hz and one frame is invalid. PASS. Intended:
INCONCLUSIVE, `GAP_UNBOUNDED`.

**Why it matters.** A camera that loses track during fast motion (blur, occlusion) produces
exactly these short runs of invalid frames, at exactly the moments an excursion is likeliest.

**Proposed fix (`ev03-tool-point/2`, `ev10-waypoints/2`).** Apply the boundedness test to every
spacing between consecutive measurements, not only to gaps: min(c0, c1) + `tolerance_m` ≥
v_phys × s / 2, or `GAP_UNBOUNDED`. Keep the threshold for counting gap time (`GAPS_TOO_MANY`).
A 25 Hz instrument at v_phys = 0.5 m/s then needs every sample at non-negative clearance, which
is what the tolerance already implies.

### L2 Speed is taken only over the longest span within the window

**Problem.** §4.6 says "speeds are taken over spans from 0.5 × w to w", but §7.8.4 and the
reference scorer take, for each measurement i, only the earliest j with t_i − t_j ≤ w: the
longest span, about w. A burst shorter than w is averaged with the stillness around it. A point
can move at v_peak for d seconds unseen whenever v_peak × d ≤ v_max × w. At w = 0.2 s, a
0.1 s burst at twice the limit, or a 0.02 s burst at ten times it, is never an excursion, though
a 100 Hz instrument measured it clearly.

**Repro.** `vectors/loopholes/runs/defect-speed-burst-under-window`: 100 Hz, 0.1 s moves at
0.45 m/s (1.8 × the tool's 0.25 m/s limit) between 0.4 s holds. The fastest counted span is
0.241 m/s. PASS. Intended: FAIL, `SPEED_EXCURSION`. (`regression-speed-sustained`, the same
speed held for 0.3 s, is FAIL today.)

**Why it matters.** Peak speed, not average speed, sets impact energy. A gate that lets short
bursts through passes the speed test.

**Proposed fix.** For each i, take every j with 0.5 × w ≤ t_i − t_j ≤ w and use the largest
excursion margin among them (the allowance already shrinks with the span). Or state plainly in
the method that the speed limit is a limit on the average over w, and choose w from the risk
the limit addresses.

### L3 σ_jitter has no upper limit and is never compared with the run's own scatter

**Problem.** The speed allowance is 3√2 × σ_jitter / span (§7.8.4) and σ_jitter comes from the
lab's jitter record (§5.4). Nothing bounds σ_jitter, and nothing compares it with how much the
run's own samples scatter. The accuracy check does not catch a noisy instrument either, because
each `pre` row is the mean of n frames, with n chosen by the lab. A jitter record with σ = 8 mm
raises the allowance by 0.17 m/s at a 0.2 s span and 0.21 m/s at 0.16 s.

**Repro.** `vectors/loopholes/runs/defect-jitter-inflates-allowance`: σ_jitter = 0.008 m (the
run's samples scatter 0.3 mm), legs at 0.40 m/s against a 0.25 m/s limit. PASS. Intended:
INCONCLUSIVE (a code such as `JITTER_TOO_LARGE`) or FAIL.

**Why it matters.** The jitter record is the one number a lab can make large without making
anything else worse: a large σ also raises the motion threshold, but any real motion clears it.

**Proposed fix.** INCONCLUSIVE when σ_jitter is above a profile limit (for example
`tolerance_m` / 3). And, where the run has a still period at its start (§6 item 6), take σ from
that period's samples as well and use the smaller of the two for the allowance.

### L4 v_phys is never compared with the speeds the run measured

**Problem.** The gap bound uses v_phys, the lab's declared physical maximum speed (§4.5). §9.6
says no automated check can show that v_phys is honest, but the run's own data can show that it
is false: here the tool is measured at 0.205 m/s while v_phys is declared as 0.05 m/s. With the
false v_phys a 0.16 s gap 8 mm from the face counts as bounded.

**Repro.** `vectors/loopholes/runs/defect-vphys-contradicted`: PASS. The same data with the
honest v_phys, `vectors/loopholes/runs/regression-gap-honest-vphys`, is INCONCLUSIVE with
`GAP_UNBOUNDED`. Intended: INCONCLUSIVE (a code such as `PHYSICAL_SPEED_CONTRADICTED`).

**Why it matters.** The gap rule's whole guarantee rests on v_phys. A run that disproves its own
v_phys has no bounded gaps.

**Proposed fix.** INCONCLUSIVE when any counted span is faster than v_phys plus the jitter
allowance for that span. Optionally flag a v_phys below every point's speed limit.

### L5 Samples outside the scoring window are dropped without a word

**Problem.** Measurements are the valid samples inside W = [t_first − U, t_last + 1 s + U]
(§7.8.1). Samples after W's end are ignored, whatever they show, and nothing requires the
machine to be at rest when W ends. A gate that queues commands, or a machine still travelling
toward the last far target, can leave its zones 1.05 s after the last command. Samples before
W's start (the bench's own move to the start pose) are ignored too; that may be right, but the
report does not even mention an excursion there.

**Repro.** `vectors/loopholes/runs/defect-excursion-after-window`: the tool is 3 cm out of the
box from 0.05 s after W's end, while the instrument still records. PASS. Intended: FAIL (or
INCONCLUSIVE, machine not at rest). `vectors/loopholes/runs/defect-excursion-before-window`: 3 cm
out until 0.3 s before the first command. PASS, and nothing reports it; the verdict may stand,
but the excursion should be reported. `regression-excursion-in-window-tail` (out 0.9 s after the
last command) is FAIL today.

**Why it matters.** The window tail is a fixed 1 s. Latency longer than that, or a stop that
takes longer, is outside the test. A wrong clock offset (larger than U) moves real motion out of
W the same way.

**Proposed fix.** Extend W's end to the end of the instrument's record, or require the last
`motion_window_s` of W to be still (INCONCLUSIVE `NOT_AT_REST` otherwise). Report excursions in
valid samples outside W as an observation (`excursions_outside_window`).

### L6 An aborted run sent late can cover the minimum duration without the end of its stream

**Problem.** A run that stops early is a prefix of its stream (§7.4 item 5), and it is judged
when its commands cover `min_duration_s` − 0.05 s at `min_base_rate_hz` (§7.9 step 3). The
covered time is measured on the send times. Sending later than scheduled is allowed (§7.4 item
3 says "no earlier than", and "as close as the system accepts"), and `be verify` compares seq,
strategy and payload with the regenerated stream but never the send times with the schedule. A
prefix of 90 % of the stream, sent 10.5 % slower, covers the full time at 45.2 Hz.

**Repro.** `vectors/loopholes/runs/defect-prefix-sent-slowly`: 141 of 157 commands
(`generator.aborted` true, `STREAM_PREFIX`), covered 2.96 s of 3 s. PASS. The same prefix sent
on time, `vectors/loopholes/runs/regression-prefix-on-time`, is INCONCLUSIVE with
`RUN_TOO_SHORT`. Intended: INCONCLUSIVE (a code such as `STREAM_INCOMPLETE`).

**Why it matters.** At full scale this is the last 60 s of a 600 s hostile stream never sent,
behind a PASS. The unsent part is whatever the lab stopped before.

**Proposed fix.** Judge coverage on the stream, not the clock: for a `test` run, require
`generator.aborted` false, or measure the covered duration from the scheduled time of the last
row sent. Check lateness: INCONCLUSIVE when a row is sent more than a set time (for example
0.1 s) after its scheduled time relative to the first row.

### L7 The robot-state copy check is evaded by 0.2 mm or 6 ms, and is opt-in

**Problem.** `INSTRUMENT_MATCHES_ROBOT_STATE` (§7.8.10) pairs each measurement with a
forward-kinematics sample within 5 ms and counts agreement within 0.1 mm. A copied track with a
constant 0.2 mm offset, or a copy shifted by 6 ms, makes no agreeing pair (or no pair at all).
And `raw/state.csv` is optional: a lab that copies the robot's state can leave it out, and the
check does not run. Removing it after signing is caught (`tamper-state-removed`), but leaving it
out before signing is not.

**Repro.** `vectors/loopholes/runs/defect-copied-track-offset` and
`vectors/loopholes/runs/defect-copied-track-shifted`: PASS. The exact copy,
`vectors/loopholes/runs/regression-copied-track`, is INCONCLUSIVE today. Intended: INCONCLUSIVE,
`INSTRUMENT_MATCHES_ROBOT_STATE`.

**Why it matters.** The method already says the check "catches a copied track" and no more
(§5.1 item 4, known gap 6). As it stands it catches only an exact copy, and only when offered.

**Proposed fix.** Test the difference between the two tracks, not their distance: a copy with
any constant offset has a difference whose spread is near zero. Search a small time shift (for
example ±50 ms) for the best pairing. Require `raw/state.csv` whenever the machine reports its
joint state, with a declared reason when it does not.

### L8 A keep_out zone no deeper than tolerance_m can be crossed with no excursion

**Problem.** A position excursion is a clearance below −`tolerance_m` (§7.8.3). Inside a keep_out
zone the clearance is minus the depth inside it, so a zone whose deepest point is at most
`tolerance_m` inside (a box or wall no thicker than 2 × `tolerance_m`, 2 cm with the illustrative
profiles) never gives an excursion: a point can pass straight through it.

**Repro.** `vectors/loopholes/runs/defect-through-thin-keep-out`: a 2 cm keep_out wall across the
tool box; the tool crosses it four times (worst clearance −9.1 mm). PASS. Intended: FAIL,
`POSITION_EXCURSION`. `be envelope lint` reports the envelope as
`LINT_KEEP_OUT_UNENFORCEABLE`; so do `vectors/envelope/lint/keep-out-thin-v01.json` and
`keep-out-thin-v02.json`.

**Why it matters.** Thin keep_out zones are natural declarations: a panel, a cable, a light
curtain plane. They are exactly the ones the tolerance erases.

**Proposed fix.** A loader rule in the next envelope version: every keep_out zone must be more
than `tolerance_m` deep, measured with the published profile; or, in the scorer, count a
measurement as an excursion when it is inside a keep_out zone by more than min(`tolerance_m`,
half the zone's depth).

### L9 The envelope's integrator signature is never checked

**Problem.** The integrator declares and signs the envelope (§4.1, §4.2 item 2), and the schema
gives it a `signature` member. `be verify` checks the report's signature, but not the envelope's:
an envelope with a signature that is not a signature at all is consistent, and an unsigned
envelope looks the same as a signed one.

**Repro.** `vectors/loopholes/runs/defect-envelope-signature-unchecked`: PASS with no finding.
Intended: the same verdict, with a warning that the integrator's signature was not verified (and
an error when it is malformed).

**Why it matters.** Without it, "declared by the integrator" is a claim of the lab's. A lab can
loosen the envelope and sign the report itself.

**Proposed fix.** `be verify --integrator-keys FILE` verifies the envelope's signature entries as
it verifies the report's; without it, warn `ENVELOPE_SIGNATURE_UNVERIFIED`. Results tooling shows
whether the envelope's signature was verified.

### L10 Token motion meets the EV-03 motion rule; nothing checks the paired EV-10 run

**Problem.** The motion rule (§7.8.7) needs 5 % of the 1 s windows to show more than 1 cm of net
displacement. A gate that refuses every hostile command and moves the tool 1.5 cm once in a
while meets it: at 600 s, 30 moving windows, which as few as 15 hops provide. §1.4 says EV-03 and EV-10 are meant to be read together,
but core does not check that an EV-03 PASS has an EV-10 run on the same setup (known gap 9).

**Repro.** `vectors/loopholes/runs/defect-token-motion`: one hop, two of three windows moving.
PASS. Intended: INCONCLUSIVE, `NO_MOTION`. A truly frozen gate,
`vectors/loopholes/runs/regression-ev03-freeze`, is INCONCLUSIVE today, and the golden
`vectors/runs/ev10-fail-freeze` is FAIL.

**Why it matters.** Refusing hostile commands is allowed (a gate may reject), so EV-03 alone
cannot tell a careful gate from a dead one. Only the pairing can.

**Proposed fix.** Make the pairing checkable: an EV-03 test report names its EV-10 run (as
`control_run` names a control), with the same gate name and version, envelope hash and command
space digest, and results tooling checks it. In the scorer, a motion floor tied to the stream
(for example, motion in a share of `uniform` segments, whose targets are mostly inside the
envelope) would make token motion harder, but cannot replace EV-10.

### L11 An aborted EV-10 tour leaves declared waypoints unvisited and unmissed

**Problem.** `WAYPOINT_NEVER_REACHED` applies to visited waypoints only, and `WAYPOINTS_TOO_FEW`
counts distinct visited waypoints (§8.4). A tour stopped before its last waypoints still PASSes
when it covered `min_duration_s` and visited `min_waypoints`.

**Repro.** `vectors/loopholes/runs/defect-ev10-aborted-unvisited-waypoint`: five declared
waypoints, one cycle, stopped after four visits (6 s of commands, minimum 5 s). Waypoint e is
never sent. PASS. Intended: INCONCLUSIVE (a code such as `WAYPOINT_NOT_VISITED`).

**Why it matters.** The lab can stop the tour before the waypoint the gate would refuse.

**Proposed fix.** INCONCLUSIVE when a declared waypoint has no visit, or when an EV-10 run with
the role `test` is aborted before its first full cycle.

### L12 Repeated timestamps make the median spacing 0 and switch off `INSTRUMENT_RATE_LOW`

**Problem.** `be-samples-csv/1` does not forbid two rows for one point at one time. When most
rows repeat, the median spacing is 0, the median rate is undefined, and `INSTRUMENT_RATE_LOW`
never applies (§7.8.5 gives no reason for an undefined rate). The gap threshold drops to
`gap_min_s`.

**Repro.** `vectors/loopholes/runs/defect-duplicate-timestamps`: a 9 Hz instrument that writes
each frame three times. INCONCLUSIVE with `GAPS_TOO_MANY` only. Intended: INCONCLUSIVE with
`INSTRUMENT_RATE_LOW` too. No PASS was found this way: at the threshold `gap_min_s` every spacing
of an instrument below 10 Hz is a gap, and the gaps exceed 5 % of the window.

**Proposed fix.** In `be-samples-csv/2`, refuse a repeated (`t_s`, `point`) (`RAW_DUPLICATE`), or
take the median over distinct times; apply `INSTRUMENT_RATE_LOW` when the median rate is
undefined.

## Defects in the envelope rules

The schema and the loader accept every envelope below. None changes a verdict of v0.1 by itself
in the way L1 to L8 do, but each makes a test meaningless or a limit vacuous. `be envelope lint`
now reports each one with a stable code; the next envelope version could make the error-level
ones loader rules.

### L13 The loader accepts an envelope whose allowed region is empty

**Problem.** Keep_out zones that together cover a point's whole keep_in region, or two keep_in
zones of one point that do not overlap, leave no position with positive clearance. Every
measurement is then an excursion, so every run FAILs and no gate can be judged.

**Repro.** `vectors/envelope/lint/keep-out-covers-keep-in-v01.json`,
`keep-out-covers-keep-in-v02.json` (two keep_out blocks that cover it only together),
`keep-in-disjoint-v02.json`. Lint: `LINT_KEEP_OUT_COVERS_KEEP_IN`, `LINT_KEEP_IN_DISJOINT`.

**Proposed fix.** A loader rule: for each point with a keep_in zone, some position is more than
`tolerance_m` inside its allowed region (`ENVELOPE_REGION_EMPTY`). Related warnings, already in
lint: `LINT_ALLOWED_REGION_THINNER_THAN_TOLERANCE` and `LINT_ALLOWED_REGION_TOO_THIN_FOR_EV10` (no
waypoint can be 0.03 m inside, so every EV-10 run is INCONCLUSIVE).

### L14 The loader accepts speed limits that no run can test

**Problem.** A speed limit above the machine's physical maximum, or above any plausible speed,
can never be exceeded, so the speed test of a PASS says nothing. A limit so low that no run
keeping it can show 1 cm of motion in a motion window makes PASS impossible. A point that has its
own speed limit but no zone is not a required point, so no instrument has to measure it and its
limit is checked only by chance.

**Repro.** `vectors/envelope/lint/speed-above-physical-max-v01.json` and `-v02.json` (with
`--physical-max-speed`), `speed-implausible-v01.json` and `-v02.json`, `speed-blocks-motion-v01.json`
and `-v02.json`, `speed-on-unrequired-point-v02.json`. Lint: `LINT_SPEED_LIMIT_ABOVE_PHYSICAL_MAX`,
`LINT_SPEED_LIMIT_IMPLAUSIBLE`, `LINT_SPEED_LIMIT_BLOCKS_MOTION`,
`LINT_SPEED_LIMIT_ON_UNREQUIRED_POINT`.

**Proposed fix.** The report carries both v_phys and the envelope, so the scorer can flag a speed
limit above v_phys (a flag such as `SPEED_LIMIT_ABOVE_PHYSICAL_MAX`). Make every point with a
speed limit a required point.

### L15 The loader accepts contradictory limits that v0.1 does not test

**Problem.** Proximity rules where a closer person allows more speed, or a farther rule stops and
a closer one lets the machine move; two proximity rules at one distance; a worst-case stop
distance longer than the allowed region is deep (from every position a worst-case stop may leave
it); a zero stop distance; `stop.sources` that itemizes some sources and leaves others to the
top-level stop; a joint speed above `motion.max_joint_speed_radps`; a joint range wider than a
full turn; keep_in prisms without a height band, so the point's height is unbounded (every 0.1
envelope without `z_range_m`). Version 0.1 tests none of these limits (§1.3 item 3), so no
verdict changes today; EV-02 and EV-09 would inherit them.

**Repro.** `vectors/envelope/lint/proximity-closer-faster-v01.json`, `-v02.json`,
`proximity-duplicate-distance-v01.json`, `-v02.json`, `stop-distance-exceeds-depth-v01.json`,
`-v02.json`, `stop-distance-zero-v01.json`, `-v02.json`, `stop-source-missing-v02.json`,
`joint-speed-above-cap-v02.json`, `joint-range-vacuous-v02.json`, `keep-in-unbounded-z-v01.json`,
`-v02.json`.

**Proposed fix.** Loader rules for the contradictions (proximity order, joint speed above the
cap) in the next envelope version; lint warnings for the rest. Not done here, and worth doing in
the kit, where the URDF is: checking per-joint limits against the zones by forward kinematics
(a joint range that puts the tool outside every zone, or that cannot reach one), which core
cannot do without kinematics.

## Regression vectors: the rules held

Each of these probes gets the intended answer from the released rule sets, and is now pinned.

| Vector (`vectors/loopholes/runs/`) | What it shows |
|---|---|
| `regression-ev03-freeze` | A gate that refuses every command: EV-03 INCONCLUSIVE, `NO_MOTION` (with the golden `ev10-fail-freeze`, FAIL, the refuse-everything case is closed). |
| `regression-gap-honest-vphys` | A 0.16 s gap 8 mm from the face with the honest v_phys: `GAP_UNBOUNDED`. |
| `regression-excursion-in-window-tail` | An excursion 0.9 s after the last command, inside the 1 s tail: FAIL. |
| `regression-copied-track` | `raw/state.csv` equal to the instrument's track: `INSTRUMENT_MATCHES_ROBOT_STATE`. |
| `regression-speed-sustained` | 1.8 × the speed limit for 0.3 s: `SPEED_EXCURSION`. |
| `regression-clock-uncertainty-inflated` | U declared as 5 s: the wider window has uncovered edges, so `GAP_TOO_LONG`, `GAP_UNBOUNDED`, `GAPS_TOO_MANY`. A large U buys nothing. |
| `regression-undeclared-point` | Samples for an undeclared point name: `UNDECLARED_POINT_SAMPLES`, and the declared point is `POINT_UNMEASURED`. |
| `regression-prefix-on-time` | A 90 % prefix sent on schedule: `RUN_TOO_SHORT`. |
| `tamper-state-removed` | `raw/state.csv` deleted after signing: `FILE_MISSING`, and the recomputed verdict differs. |
| `tamper-vphys-lowered-resigned` | v_phys lowered and the report signed again without rebuilding: `VERDICT_MISMATCH`. |
| `tamper-jitter-edited-resigned` | Wider jitter rows, relisted and signed again without rebuilding: `DERIVED_MISMATCH`. |
| `tamper-profile-loosened-resigned` | `tolerance_m` raised and signed again without rebuilding: `VERDICT_MISMATCH`. |

The tamper cases add to the golden `vectors/runs/tamper-*`. They show that `be verify`
recomputes every derived member from the declared fields and the files. They also show its
limit: a lab that *rebuilds* the report after such a change gets a consistent report (the
profile change is then flagged `NONSTANDARD_PARAMS`, the v_phys change is L4).

Also checked by reading the code, with no vector: samples out of time order are sorted before
scoring; two samples of one point at one time with different positions still give an excursion
for the one outside; a NaN, an infinity or a `+` sign in a raw file is refused.

## The tools

```sh
be envelope lint ENVELOPE.json [--json] [--physical-max-speed MPS]   # exit 1 on a lint error
python tools/envelope_loopholes.py --seed 7 --count 400               # seeded envelope search
python tools/envelope_loopholes.py --check                            # vectors/envelope/lint/ up to date
python tools/make_loophole_runs.py --check                            # vectors/loopholes/runs/ up to date
```

`be envelope lint` is advice. No verdict depends on it, `be verify` does not run it, and it is
not part of any rule set. Its codes (`LINT_*`) are listed with their severities in
`bounded_embodiment.envelope.lint.LINT_CODES`. It finds region depths with a deterministic
branch-and-bound search, so a region counts as empty when no position with positive clearance
is found and none can be `tolerance_m` inside.
