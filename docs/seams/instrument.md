# Seam: the instrument's accuracy check (`be instrument check`, `be instrument plan`)

Built 11 Oct 2026 in `packages/kit/src/bounded_embodiment_kit/instruments/{accuracy,plan}.py` and
`kinematics/ik.py`; tested in `packages/kit/tests/test_instrument_{check,plan}.py` on synthetic data
(and, marked slow, on a rendered recording fitted by `be fit`). It closes known gap 1 of the method
draft ("no accuracy-check tool") and part of gap 2.

## What the method asks (§5.3 to §5.5)

- **Pre-run check**: 6 or more surveyed points, each with its own label, spread through the
  measured point's keep-in near both extremes of each axis; reference positions measured
  independently of the instrument and of the machine's state; the instrument exactly as in the run;
  at most 12 h before the run start and not after it.
- **Jitter**: raw samples of a measured point with the machine still in its run-start state, one
  label per still period, SHOULD be 10 s.
- **Post-run check**: after each run, at least one surveyed point again, same label, same procedure.

The kit cannot survey for the lab. It does the rest: it finds the still periods in a recording,
averages the instrument's valid samples, writes `raw/accuracy.csv`, computes what core's scorer will
compute (with core's own functions), and checks the parts of the procedure core cannot.

## `be instrument check`

```
be instrument check --envelope ENV --targets TARGETS.txt --session SESSION [--run RUN_DIR]
    [--samples CSV ...] [--registration REG] [--background BG] [--urdf URDF] [--point ID]
    [--test EV-03|EV-10] [--out DIR] [--settle-s 0.5] [--hold-s 3] [--jitter-s 15]
    [--still-max-mm 2] [--min-frames 5]
be instrument check --envelope ENV --targets TARGETS.txt --holds HOLDS.csv --samples CSV ... [...]
```

**Holds from a `be record` session.** Events typed during the recording (`events.csv`):

| Event | Meaning |
|---|---|
| `pre:T1` (typed `pre T1` is stored as `pre_T1`, also accepted) | the measured point is at surveyed point T1, the machine still |
| `post:T1` | the same, after the run |
| `jitter` or `jitter:LABEL` | the machine still in its run-start state (label `start_pose` by default) |
| anything else (`moving`) | ends the hold before it |

A hold starts `--settle-s` after its event and ends at the next event or after `--hold-s` (pre,
post) or `--jitter-s` (jitter), whichever is first. An event that looks like a hold but is not
`PHASE:LABEL` is noted and ignored.

**Holds from a file** (`be-holds-csv/0.1`, any instrument): `phase,label,t_start_s,t_end_s,t_wall_start_s`
on the samples' timebase; `t_wall_start_s` (Unix seconds at `t_start_s`) may be empty, but then the
check's time and its order against the run are unknown. Windows are used as given.

**Samples.** `--samples` takes the instrument's `be-samples-csv/1` (read by core's strict reader).
With `--session` and no `--samples`, `be fit` runs on the chosen hold windows only, with the
session's `fit/registration.json` and `fit/background.npz` (both required: the check must use the
instrument as the run does); each window's output goes to `OUT/fit/<phase>_<label>_<k>/`.

**Targets** (`be-targets/0.1`, text, `#` starts a comment, strict):

```
format = be-targets/0.1
frame = F            # be fit register's table frame (needs the registration), or the envelope's frame
units = mm           # mm or m
method = Plumb line and steel rule from the tool-point dot; tape and square to frame F's axis lines
surveyed_at = 2026-10-13T16:20:00Z    # optional
T1 = 196.0, -80.5, 283.0
```

Frame F is converted with the registration's `steps.T_bl_F`, so the targets and the registration
share one table frame. `method` is required: it becomes `accuracy_check.method`.

**Choosing holds against a run** (`--run RUN_DIR`, its `declared.json` timestamps): for each label
the last `pre` hold that ended before the run started; the last jitter hold before it; the first
`post` hold per label that started after the run ended. Several runs can share one pre-run check.
The order is judged on the monotonic clock when the session's events were stamped on the runner's
host (their wall − monotonic offsets agree within 0.25 s), else on the wall clock. Without `--run`
the order is not checked (noted), the last pre hold per label and the last jitter hold are used,
and two post holds at one label are refused.

**Per hold**: valid samples in the window, their mean (`math.fsum`), RMS deviation, and the
distance between the means of the window's two halves. A hold whose halves differ by more than
`--still-max-mm` (the machine moved) or with fewer than `--min-frames` valid samples is a problem.

**Outputs** (`--out`, default `RUN_DIR/accuracy` with `--run`, else `SESSION/fit/check`):

- `accuracy.csv` (`be-accuracy-csv/1`): one `pre` row per surveyed point (reference, mean, n), the
  `post` rows (reference, mean, n), and every raw `jitter` sample (n = 1, empty references). Numbers
  are the float's shortest exact decimal, no exponent; core's reader re-reads the file before it is
  written.
- `accuracy-check.json` (`be-accuracy-check/0.1`): inputs and their digests, settings, every hold
  (used or why not), the targets in both frames, core's `scoring.ev03.accuracy` block, the spread
  check, the order, problems, notes, and `lab_config`: the `accuracy_check` (`at` = the wall time
  the last pre hold ended, `points` = the pre rows, `method`) and `post_run_check` blocks to copy into
  the lab config.

**Problems** (exit 1, files written): core's step-1 accuracy reasons (`ACCURACY_CHECK_TOO_FEW_POINTS`,
`ACCURACY_CHECK_FAILED`, `ACCURACY_CHECK_STALE`, `JITTER_MISSING`, `POST_CHECK_MISSING`,
`POST_CHECK_DRIFT`; evaluated by `scoring.verdict.validity_reasons`); a hold that moved or has too few
valid samples; a held label with no surveyed position; points that do not reach within a quarter of
the keep-in's extent of both faces on every axis (only points inside the allowed region, or within
`tolerance_m` of it, count); a pre-run check that ended after the run started; no `method` line.
**Notes** (exit 0): a jitter record shorter than 10 s, points outside the allowed region, surveyed
points with no hold, superseded holds, the order unchecked. Exit 2: unusable input, nothing written.

Nothing it opens holds robot state or commands (tested by recording every file opened).

## `be instrument plan`

```
be instrument plan --envelope ENV --urdf URDF --kit-config KIT [--command-space CS] [--point tool]
    [--inset-mm 20] [--margin-mm 10] [--level LINK:AXIS] [--out PLAN.json] [--waypoints-out W.json]
```

Eight targets at the corners of the measured point's keep-in box (the intersection of its keep-in
zones), `--inset-mm` inside each face; a corner within `--margin-mm` of leaving the allowed region (a
keep-out) has one coordinate moved to the box's centre, whichever leaves the most clearance. For
each, position inverse kinematics (`kinematics.ik.solve_point`: Levenberg-Marquardt, forward-difference
Jacobian from `fk_batch`, clamped to the URDF limits less 3°) from the kit config's start pose and
two seeds with the last joint turned ±90°; it keeps the solution nearest the start pose with every
envelope point `--margin-mm` inside its zones and every value inside the command space's limits.
An unreachable target moves 1 cm toward the box's centre, up to three times. `--level
gripper_link:x` keeps that axis horizontal (on the SO-101: the fixed jaw's inner face vertical).

It prints, and with `--out` writes (`be-target-plan/0.1`), the planned positions and a `poses_deg`
block for the kit config, in the robot's own degrees through the kit config's joint map. The
planned positions are never reference positions; the targets file holds what the lab surveys.
`be goto-start --pose T1` drives to one.

`--waypoints-out` writes the same targets as an EV-10 waypoints file (waypoints 0.1): positions by
inverse kinematics (§8.1 item 4), commands on every axis of `--command-space`. It refuses a waypoint
less than 3 cm inside its zones or two less than 5 cm apart (they would make an EV-10 run
INCONCLUSIVE); use `--inset-mm 35 --margin-mm 30`.

## What is still the lab's (method known gap 2, narrowed)

- That the reference positions were measured independently: the tool cannot see how.
- That the instrument was not moved, re-registered or recalibrated between the checks and the run.
- A frame-origin error shared by the registration and the survey (both use frame F) moves both
  alike, and the check cannot see it. Mark F's origin carefully.
