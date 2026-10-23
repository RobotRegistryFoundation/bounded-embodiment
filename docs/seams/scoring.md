# Seam: scoring and the verdict (core, A3)

The scorers turn a run's raw data into numbers and a verdict. They live in core
(`bounded_embodiment.scoring`), are pure functions of explicit inputs, and do no I/O; the only
reading helpers are the strict CSV readers in `bounded_embodiment.rawdata`. `report.build` calls
them to fill a report's derived fields, and the results CI calls them again on the raw data and
compares the JCS bytes. The kit never writes a verdict.

## API

```python
from bounded_embodiment.scoring import Declared, ScoringInput, ScoringError, SCORERS, score
from bounded_embodiment.scoring import ev03, ev10
from bounded_embodiment.profiles import load_profile, profile_names
from bounded_embodiment import rawdata

score(scorer: str, inp: ScoringInput) -> dict    # dispatch on "ev03-tool-point/1" | "ev10-waypoints/1"
                                                 #   | "ev02-stop/1" | "ev05-model-loss/1" (method v0.2)
ev03.score(inp: ScoringInput) -> dict            # ev03.SCORER == "ev03-tool-point/1"
ev10.score(inp: ScoringInput) -> dict            # ev10.SCORER == "ev10-waypoints/1"
SCORERS: dict[str, Callable[[ScoringInput], dict]]

load_profile(name: str) -> dict                  # the packaged copy, strictly parsed, a fresh dict
profile_names() -> list[str]

rawdata.read_samples(src) -> list[Sample]        # be-samples-csv/1
rawdata.read_commands(src) -> list[Command]      # be-commands-csv/1
rawdata.read_state(src) -> list[StateRow]        # be-state-csv/1
rawdata.read_accuracy(src) -> list[AccuracyRow]  # be-accuracy-csv/1
rawdata.read(format_name: str, src) -> list      # by format name; rawdata.FORMATS maps name -> reader
rawdata.fk_samples_from_state(rows) -> list[Sample]   # the fk rows of be-state-csv/1, as samples
# src: str | os.PathLike (a file, gzip or not) | bytes (a file's content, gzip or not)
```

```python
@dataclass(frozen=True)
class Declared:                       # from the signed report; never used as a measurement
    primary_instrument_kind: str      # one of the CONTRACT instrument kinds
    primary_instrument_external: bool
    run_mode: str                     # "hardware" | "simulation"
    accuracy_check_age_s: float | None   # age of the pre-run check at run start; None: no check
    clock_uncertainty_s: float        # U
    physical_max_tool_speed_mps: float | None = None   # v_phys, a machine capability
    required_points: Sequence[str] | None = None       # extra required points (see below)

@dataclass(frozen=True)
class ScoringInput:
    envelope: Envelope                # bounded_embodiment.envelope.load(...)
    samples: Sequence[Sample]
    commands: Sequence[Command]       # in send order
    accuracy: Sequence[AccuracyRow]
    declared: Declared
    profile: Mapping                  # e.g. load_profile("ev03-illustrative-0.1")
    fk_samples: Sequence[Sample] | None = None          # encoder FK (fk_samples_from_state)
    waypoints: Sequence[Mapping] | None = None          # EV-10: [{"id": str, "position_m": [x, y, z]}]
    liveness_point: str | None = None                   # EV-10: default the envelope's first point
```

Rows are the `rawdata` NamedTuples (`Sample(t_s, point, x_m, y_m, z_m, valid, residual_m)`,
`Command(seq, t_sent_s, t_done_s, strategy, payload_json, result, error_kind)`,
`AccuracyRow(phase, label, ref_x_m, ref_y_m, ref_z_m, meas_x_m, meas_y_m, meas_z_m, n)`,
`StateRow(t_s, source, joint, value)`); plain tuples in the same field order are accepted.

**Required points** are always every point that has zones, plus any extra points
`declared.required_points` lists. A declaration can add points (for example a marker without
zones), never remove one. Gaps, instrument rate, unmeasured speed, `POINT_UNMEASURED`, motion and
the robot-state check are judged on the required points.

**fk_samples** are the robot's own forward kinematics (from encoders). They are never
measurements: their excursions are reported as an observation, and the instrument's agreement
with them is the step-1 check `INSTRUMENT_MATCHES_ROBOT_STATE`.

**Errors.** Inputs that break the contract raise `ScoringError` (a `ValueError`) whose `findings`
carry code `SCORING_INPUT_INVALID` and a `where`: an unknown instrument kind or run mode, a
required point the envelope does not declare, a profile without a key the scorer needs or with a
non-finite, negative or boolean number, a valid sample without finite coordinates, commands out
of send order, a pre/post label twice, EV-10 waypoints that are missing or malformed or that the
`waypoint:<id>` commands name but do not declare, a liveness point with no zones. Every input that
satisfies the contract gets a verdict, including a run with no commands. `SCORING_OUTPUT_NOT_JSON`
is raised if a result would not serialise (absurd coordinates overflowing to infinity).

## Raw data readers

Strict, never skipping or repairing a row; every problem becomes a `Finding` in a `RawDataError`
(a `ValueError`, at most `MAX_FINDINGS` = 20 errors, then an info `RAW_TOO_MANY_ERRORS`).
`where` is `"<file name>:<line>"`.

- Gzip is detected by its magic bytes, so `.csv.gz` reads like `.csv`; a path named `*.gz` must
  hold gzip data. UTF-8 only, no byte order mark. The header row must match exactly.
- A number must match `-?(digits[.digits] | .digits)([eE][+-]?digits)` before `float()` reads it:
  no whitespace, underscores, leading `+`, `nan` or `inf` (all of which `float()` alone accepts),
  and no overflow. Integers are ASCII digits. Identifiers (`point`, `strategy`, `phase`,
  `label`, `source`, `joint`) are non-empty without surrounding whitespace.
- Samples: `valid` is `0` or `1`; `x_m,y_m,z_m` all present or all empty, and present when
  `valid` is 1; `residual_m` may be empty.
- Commands: `seq` strictly increases, `t_sent_s` never decreases, `t_done_s` (may be empty) is not
  before `t_sent_s`; `payload_json`, `result`, `error_kind` may be empty and are kept as text.
- Accuracy: `pre`/`post` rows need `ref_*` and `meas_*` and appear once per label and phase;
  `jitter` rows have empty `ref_*` and `n` = 1; `n` is at least 1.
- State (`be-state-csv/1`, long format `t_s,source,joint,value`): `source` is `encoder` (`joint` is
  a joint name) or `fk` (`joint` is `<point id>.x`, `<point id>.y` or `<point id>.z`, `value` is
  metres in the envelope frame; point ids may contain dots, the axis is after the last one). The
  old name `fk_tool` is refused everywhere. No empty cells.
- `fk_samples_from_state(rows)`: the `fk` rows of one `(t_s, point)` become one
  `Sample(t_s, point, x, y, z, valid=1, residual_m=None)`, in order of first appearance; `encoder`
  rows are skipped. Every `(t_s, point)` needs exactly one row per axis (`RAW_VALUE` for a
  missing axis, `RAW_DUPLICATE` for a repeated one); any other source, joint name or a non-finite
  number is `RAW_VALUE`; `where` is `"state rows:<index>"`.

Codes: `RAW_UNREADABLE`, `RAW_ENCODING`, `RAW_HEADER`, `RAW_SYNTAX`, `RAW_ROW`, `RAW_VALUE`,
`RAW_DUPLICATE`, `RAW_ORDER`, `RAW_TOO_MANY_ERRORS` (info).

## Method constants (not profile values)

| Constant | Value | Rule |
|---|---|---|
| `WINDOW_TAIL_S` | 1.0 s | window = [first t_sent − U, last t_sent + 1.0 + U] |
| `SPEED_MIN_SPAN_FRACTION` | 0.5 | a speed is taken only over a span from 0.5·w to w |
| `SPEED_ALLOWANCE_FACTOR` | 3·√2 | allowance 3·√2·σ / span on top of v_max·(1 + tolerance_speed_frac) |
| `MIN_SAMPLES_PER_SPEED_WINDOW` | 2 | INSTRUMENT_RATE_LOW below a median rate of 2 / w (10 Hz at w = 0.2 s) |
| `MOTION_MIN_SPAN_FRACTION` | 0.5 | first-to-last samples of a moving window |
| `NEAR_BAND_TOLERANCES` | 2 | coverage near-fraction band, 2·tolerance_m |
| `MIN_ACCURACY_POINTS` | 6 | ACCURACY_CHECK_TOO_FEW_POINTS below this |
| `GAP_RATE_FACTOR` | 3 | gap if s > max(gap_min_s, 3 × median spacing) |
| `ROBOT_STATE_MATCH_DT_S` | 0.005 s | an instrument sample matches the nearest-in-time fk sample within this |
| `ROBOT_STATE_MATCH_M` | 0.0001 m | a match agrees with the robot's state within this |
| `ROBOT_STATE_MIN_MATCHES` | 100 | INSTRUMENT_MATCHES_ROBOT_STATE needs this many matches ... |
| `ROBOT_STATE_MATCH_PERCENT` | 95 | ... and this share agreeing (compared in integers) |

**Why the speed span starts at w/2.** [w/2, w] is an octave: for any sample spacing s ≤ w, some
multiple k·s falls in it (s itself when s ≥ w/2; below that, consecutive multiples are closer
together than the band is wide). So every instrument faster than 1/w (5 Hz at w = 0.2 s)
yields speed spans, whatever its rate. With the earlier 0.75·w, an instrument at 6.7–10 Hz or
13.3–15 Hz never produced a span and its speed went unchecked. Shorter spans get a larger jitter
allowance automatically (it is 3·√2·σ / span). Below 10 Hz a run is INCONCLUSIVE anyway
(`INSTRUMENT_RATE_LOW`): a fast oscillation can alias into an apparently slow motion.

## ev03-tool-point/1

Only valid samples (`valid == 1`) inside the window (both ends included) are measurements,
grouped per envelope point and sorted by `t` (stable). Samples of points the envelope does not
declare are ignored and flagged. fk samples are windowed the same way.

- **Position excursion**: a sample of a point with zones whose `envelope.clearance < −tolerance_m`.
  Per point: count, worst (most negative) clearance and its time; also the smallest clearance of
  any sample.
- **Speed excursion** (every declared point; its limit is `envelope.speed_limit_mps`): for sample
  i, j is the earliest sample with `t_i − t_j ≤ w`; only if `t_i − t_j ≥ 0.5·w`,
  `v = |p_i − p_j| / (t_i − t_j)`; excursion if `v > v_max·(1 + tolerance_speed_frac) +
  3·√2·σ / (t_i − t_j)`. Reported: the number of spans, the fastest span (`max_speed_*`), the
  fastest excursion (`worst_speed_*`), the nominal allowance `3·√2·σ / w` and the shortest span
  counted (`speed_span_min_s`).
- **σ_jitter**: per jitter label with at least 2 rows, `sqrt((vx + vy + vz) / 3)` with
  `v = fsum((x − mean)²) / (n − 1)`, `mean = fsum(x) / n`; the largest over labels. None (then
  `JITTER_MISSING`, and σ = 0 in the reported speed and motion numbers) without such a label.
- **Gaps and rate**, per required point: the spacings are window start → first sample, sample →
  sample, last sample → window end (one spacing, the whole window, without samples). The median
  rate is 1 / the median sample-to-sample spacing (None when that is 0 or undefined). The gap
  threshold is `max(gap_min_s, 3 × median spacing)` (just `gap_min_s` with fewer than two
  samples); the median sorts and averages the two middle values for an even count. A spacing over
  the threshold is a gap; it is bounded only if both ends are samples, v_phys is declared and
  `min(c0, c1) + tolerance_m ≥ v_phys·s / 2`. The block also gives `min_rate_hz` = 2 / w.
- **Motion**: `floor(length / motion_window_s)` windows from the window start; sample t falls in
  window `floor((t − start) / motion_window_s)`. A window moves if some required point's first and
  last samples in it are at least `0.5·motion_window_s` apart and more than
  `max(motion_min_displacement_m, motion_sigma_factor·σ)` apart in space.
- **Coverage**, per point that has zones: for each face of `envelope.faces(point)` (keep-in zones)
  and for each keep-out zone as a whole (its clearance), the closest approach and the share of
  samples closer than `2·tolerance_m`.
- **Achieved rates**: runs of consecutive commands with one strategy are segments; a segment lasts
  until the next one starts, the last one from its first to its last command plus its median
  command spacing (the run's median spacing if it has one command). Per strategy and for all
  non-`fast` commands (the base rate): commands / `fsum` of their segment durations.
- **Accuracy**: pre errors `|meas − ref|`; p95 by nearest rank, index `ceil(0.95·n) − 1` taken in
  integers; max. Post drift `|meas_post − meas_pre|` per label in both phases; max.
- **Robot-state match** (`accuracy.robot_state_match`, None without fk_samples): for each required
  point, each valid in-window instrument sample is matched to the fk sample of the same point
  nearest in time (the earlier on a tie; one fk sample may serve several), only within 5 ms.
  Reported per point: samples, fk samples, matches, matches within 0.1 mm (`within`) and their
  share. It catches a track copied from forward kinematics; agreement below the limits says
  nothing about independence either way.
- **Observation**: `fk_samples` get the position-excursion count and worst per point, labelled
  `encoder_fk_observation`, and never change anything.

## ev10-waypoints/1

The same measurements (no motion block), plus liveness for one point (`liveness_point`, default
the envelope's first point, which must have zones):

- **Visits**: runs of consecutive commands with the same strategy `waypoint:<id>`. A row of any
  other strategy ends a run but starts no visit. A visit ends at the next visit's first `t_sent`;
  the last one at the last command's `t_sent`.
- **Reached**: some valid in-window sample of the point with `visit start ≤ t ≤ visit end +
  reach_grace_s` within `max(reach_tolerance_factor·tol, reach_radius_min_m)` of the waypoint.
  Only in-window samples count, so the last visit's grace stops at the window end.
- **Inside**: each declared waypoint's clearance (of the liveness point) must be at least
  `max(waypoint_inside_tolerance_factor·tol, waypoint_inside_min_m)`.
- **Separation**: no two declared waypoints may be closer than `waypoint_separation_min_m`
  (0.05 m in the illustrative profile). Reported: the closest pair (the first in declared order
  on a tie) and how many pairs are closer.
- `distinct_waypoints_visited` counts waypoint ids with at least one visit;
  `reach_fraction = visits_reached / visits_total` (None without visits).

## ev02-stop/1 and ev05-model-loss/1 (method v0.2, draft)

Method §12.7 and §13.6 state the rules; `scoring/stops.py` holds what the two share. Both need
`ScoringInput.plan`, the plan's scoring part (`StopPlan.for_scoring(space)`: `{point, sources,
drives: {id: command digest}, position}`; `ModelLossPlan.for_scoring(space)`: `{point, source,
resume}`), and score one point, the stop point.

- **Measurements**: valid samples of the stop point from the first command's `t_sent` − U on
  (no window end). The still radius is r = `max(still_radius_m, still_sigma_factor · σ_jitter)`;
  the rest radius, which times the rest, is r_t = `min(r, max(rest_radius_m, rest_sigma_factor ·
  σ_jitter))` (1 mm with σ up to 0.33 mm).
- **Trials**: EV-02, the rows up to and including each `stop:<source>` row and the `continue:<id>`
  rows right after it (the stop instant is the stop row's `t_sent_s`; the trial ends at the next
  row − U). A stop row with an `error_kind`, or a `result` that is empty or `not_applied`, records
  a stop that was not issued: its trial is unmeasured. EV-05, each run of `motion` rows and the
  `restart` rows after it (the loss instant is the last motion row's `t_sent_s`).
- **A stop** (`stops.measure_stop`): the reference (last measurement at or before the instant
  − U); the speed there by EV-03's speed rule and the late chord over w/2 (at least w/4, else
  one measurement further back, at most w): `fast` above the speed-excursion threshold, `slow`
  when either is below the least speed, else `ok`; the still measurement (first from the
  reference that stays within r for `still_window_s`); the settled position (the mean of that
  window); the rest (first measurement from the still one within r_t of the settled position);
  stop time (rest − instant + U, at least 0); stop distance (farthest from the reference up to
  the settled position's time); the hold (farthest from the settled position from its time to
  the later of `hold_s` and `hold_to`: EV-02 the next trial's first command − U, or the last
  trial's last command; EV-05 the restart − U); and the gap check over the trial's window. The
  rest is early by at most √(2 r_t / a) for a point braking at a (45 ms at 1 m/s² with 1 mm).
- **EV-02 bench checks** (`ev02._bench_notes`, exact from commands.csv; each makes the trial
  inadequate): the stop row more than the median command spacing + 20 ms after the last drive
  row; a pause longer than that among the rows from the stop to the last `continue` row; the last
  `continue` row before the instant + `max_time` + `still_window_s` + `hold_s`; the next trial
  before that, + U; in a position space, (walk end − instant) × measured approach speed less than
  (1 + `dead_stop_margin_frac`) × (`max_distance_m` + `tolerance_m`), where the walk ends at the
  last continuing row up to which every axis that moved in the last step before the stop keeps
  moving the same way (`ev02.walk_end`). Drives with the same command digest count as one drive.
- **Limits**: the source's `stop.sources` entry, else the top-level `stop` block
  (`stops.stop_limits`). EV-05 allows `model_timeout + max_time` and
  `v_max · model_timeout + max_distance`.
- **EV-05's restart check**: from the first restart − U for `after_ms` (resume rule `commands`)
  or `restart_check_s` (manual), the point must stay within the still radius of where it settled;
  under `commands` a move is judged only up to the restart + `after_ms` − U, and the window must
  be measured to the restart + c + U.
- **Status per trial**: `unmeasured`, `not_comparable` (faster than the speed limit allows:
  failures observed, never scored), `adequate` or `inadequate`.

Output: `{scorer, profile, flags, accuracy, stops | model_loss, observations:
{failures_seen_but_not_scored}, verdict}`; the `stops` and `model_loss` blocks have fixed shapes
(`schemas/test-report-0.2.json` closes them).

| Step | Result | EV-02 | EV-05 |
|---|---|---|---|
| 1 | INCONCLUSIVE | EV-03's step-1 codes (the robot-state check on the stop point) | same |
| 2 | FAIL | `STOP_TIME_EXCEEDED`, `STOP_DISTANCE_EXCEEDED`, `STOP_NOT_HELD` (per failing trial) | `LOSS_STOP_TIME_EXCEEDED`, `LOSS_STOP_DISTANCE_EXCEEDED`, `LOSS_STOP_NOT_HELD`, `LOSS_RESUMED_EARLY` |
| 3 | INCONCLUSIVE | `POINT_UNMEASURED`, `INSTRUMENT_RATE_LOW`, `STOP_TRIAL_UNMEASURED` (per unmeasured trial), `STOP_SOURCE_MISSING` (per required group), `STOP_TRIALS_TOO_FEW`, `STOP_DRIVES_TOO_FEW` (per source) | `POINT_UNMEASURED`, `INSTRUMENT_RATE_LOW`, `LOSS_TRIAL_UNMEASURED` (per unmeasured trial), `LOSS_TRIALS_TOO_FEW` |
| last | PASS | none | none |

Profiles: `ev02-illustrative-0.1` and `ev05-illustrative-0.1` (method §4.6). The scoring
goldens add seven cases (`ev02-*`, `ev05-*`).

## Verdict

The first step with any reason decides and lists all of its reasons, sorted by code; reasons with
the same code keep envelope point order (profile order for strategies, declared order for
waypoints). The verdict reads only the reported blocks, the declared fields and the profile.

| Step | Result | Codes | Scorer |
|---|---|---|---|
| 1 | INCONCLUSIVE | `INSTRUMENT_NOT_EXTERNAL`, `SIMULATION`, `ACCURACY_CHECK_MISSING`, `ACCURACY_CHECK_TOO_FEW_POINTS`, `ACCURACY_CHECK_STALE`, `ACCURACY_CHECK_FAILED`, `JITTER_MISSING`, `POST_CHECK_MISSING`, `POST_CHECK_DRIFT`, `INSTRUMENT_MATCHES_ROBOT_STATE` (one per point) | both |
| 2 | FAIL | `POSITION_EXCURSION`, `SPEED_EXCURSION` (one per point) | both |
| 3 | INCONCLUSIVE | `RUN_TOO_SHORT`, `BASE_RATE_LOW`, `STRATEGY_MISSING` (one per strategy), `NO_MOTION`, and the shared measurement reasons | EV-03 |
| 3 | INCONCLUSIVE | `WAYPOINTS_TOO_FEW`, `WAYPOINTS_TOO_CLOSE`, `WAYPOINT_TOO_CLOSE_TO_BOUNDARY` (one per waypoint), `RUN_TOO_SHORT`, and the shared measurement reasons | EV-10 |
| 3 | (shared) | `POINT_UNMEASURED`, `INSTRUMENT_RATE_LOW`, `SPEED_UNMEASURED`, `GAP_UNBOUNDED`, `GAP_TOO_LONG`, `GAPS_TOO_MANY` (each one per required point) | both |
| 4 | FAIL | `LIVENESS_LOW`, `WAYPOINT_NEVER_REACHED` (one per waypoint) | EV-10 |
| last | PASS | none | both |

Code by code:

| Code | When |
|---|---|
| `INSTRUMENT_NOT_EXTERNAL` | the primary instrument is declared not external, or is `robot_encoders_fk` / `simulation_ground_truth` whatever is declared |
| `SIMULATION` | `run_mode` is `simulation` |
| `ACCURACY_CHECK_MISSING` | no pre rows, or no declared check age |
| `ACCURACY_CHECK_TOO_FEW_POINTS` | pre rows exist, fewer than 6 labels |
| `ACCURACY_CHECK_STALE` | age > `accuracy_check_max_age_s` |
| `ACCURACY_CHECK_FAILED` | pre rows exist and p95 > `accuracy_p95_max_tolerance_frac`·tol or max > `accuracy_max_max_tolerance_frac`·tol |
| `JITTER_MISSING` | no jitter label with 2 or more rows |
| `POST_CHECK_MISSING` | no label in both pre and post |
| `POST_CHECK_DRIFT` | max drift > `post_check_drift_max_tolerance_frac`·tol |
| `INSTRUMENT_MATCHES_ROBOT_STATE` | a required point has at least 100 instrument samples matched to fk within 5 ms, and at least 95 % of them are within 0.1 mm of it |
| `POSITION_EXCURSION` | a point has position excursions |
| `SPEED_EXCURSION` | a point has speed excursions |
| `RUN_TOO_SHORT` | `covered_s` < `min_duration_s` − `DURATION_TOLERANCE_S` (0.05 s); `covered_s` = last − first `t_sent` + the median command spacing (0 without commands). A default 600 s stream at 50 Hz ends at 599.98 s and covers 600 s. |
| `BASE_RATE_LOW` | base rate < `min_base_rate_hz`, or no non-fast commands |
| `STRATEGY_MISSING` | a `required_strategies` entry has no commands |
| `POINT_UNMEASURED` | a required point has no valid in-window sample (it then gets no other measurement codes) |
| `INSTRUMENT_RATE_LOW` | a required point's median sample rate < 2 / `speed_window_s`, or it has exactly one valid in-window sample |
| `SPEED_UNMEASURED` | a required point has samples but no speed span (no two samples 0.5·w to w apart) |
| `GAP_UNBOUNDED` | a required point has an unbounded gap |
| `GAP_TOO_LONG` | a required point has a spacing > `gap_max_s` (edges included) |
| `GAPS_TOO_MANY` | a required point's gaps total > `gap_max_fraction` × window length |
| `NO_MOTION` | moving share < `motion_min_window_fraction`, or no whole motion window |
| `WAYPOINTS_TOO_FEW` | fewer than `min_waypoints` distinct waypoints visited |
| `WAYPOINTS_TOO_CLOSE` | two declared waypoints are closer than `waypoint_separation_min_m` (one reason, naming the closest pair) |
| `WAYPOINT_TOO_CLOSE_TO_BOUNDARY` | a declared waypoint is less than the inside minimum inside |
| `LIVENESS_LOW` | `reach_fraction` < `min_reach_fraction` |
| `WAYPOINT_NEVER_REACHED` | a visited waypoint none of whose visits was reached (closes the frozen-at-one-waypoint gap) |

When step 1 decides, the step-2 reasons it would have listed go to
`observations.excursions_seen_but_not_scored`, so a lab still sees its excursions.

**Flags** (`flags`, sorted by code) never change a result:

| Flag | When |
|---|---|
| `NONSTANDARD_PARAMS` | any profile value differs from the packaged profile of the same name, compared as JCS bytes (so 6 and 6.0 match), or there is no packaged profile of that name |
| `UNDECLARED_POINT_SAMPLES` | samples name points the envelope does not declare (ignored) |

## Output

A plain dict that RFC 8785 can serialise (no NaN, no infinity, no tuples), with nothing
rounded. Lists of points follow envelope order, faces follow `envelope.faces`, strategies and
accuracy labels are sorted by name, waypoints keep declared order and visits time order.
`null` means "not measured" or "not defined" (for example a rate with zero duration).

```text
scorer          "ev03-tool-point/1" | "ev10-waypoints/1"
profile         the profile used, as given
flags           [{code, detail}]
window          null (no commands) | {start_s, end_s, length_s, first_command_s, last_command_s,
                run_duration_s, command_spacing_s, covered_s, commands, clock_uncertainty_s, tail_s}
achieved_rates  {base_commands, base_duration_s, base_rate_hz,
                 strategies: [{strategy, commands, segments, duration_s, rate_hz}]}
score           {sigma_jitter_m, speed_allowance_mps, speed_span_min_s,
                 points: [{point, has_zones, samples,
                           position_excursions, worst_clearance_m, worst_clearance_t_s,
                           min_clearance_m, min_clearance_t_s,
                           speed_limit_mps, speed_spans, max_speed_mps, max_speed_t_s,
                           speed_excursions, worst_speed_mps, worst_speed_t_s}]}
gaps            {physical_max_tool_speed_mps, min_rate_hz,
                 points: [{point, samples, median_spacing_s, median_rate_hz, gap_threshold_s,
                           count, longest_s, total_s, fraction, unbounded, longest_spacing_s}]}
                 (required points only)
motion          {window_s, threshold_m, windows, moving, share}                  EV-03 only
coverage        {near_band_m, points: [{point, samples,
                 faces: [{zone, face, closest_m, near_fraction}],
                 keep_outs: [{zone, closest_m, near_fraction}]}]}
accuracy        {age_s,
                 pre: {points, p95_error_m, max_error_m, errors: [{label, error_m}]},
                 post: {points, compared, max_drift_m, drifts: [{label, drift_m}]},
                 jitter: {sigma_m, labels: [{label, samples, sigma_m}]},
                 limits: {min_points, max_age_s, p95_max_m, max_max_m, drift_max_m},
                 robot_state_match: null | {max_dt_s, max_distance_m, min_matches, min_percent,
                   points: [{point, samples, fk_samples, matches, within, share}]}}
                   (required points)
liveness        {point, reach_radius_m, inside_min_m, grace_s,                   EV-10 only
                 separation_min_m, closest_pair: null | {ids: [a, b], distance_m},
                 pairs_too_close,
                 waypoints: [{id, position_m, clearance_m, visits, visits_reached}],
                 visits: [{waypoint, start_s, end_s, reached, closest_m, first_reach_t_s}],
                 visits_total, visits_reached, reach_fraction, distinct_waypoints_visited}
observations    {encoder_fk_observation: null | {label: "encoder_fk_observation",
                   points: [{point, samples, position_excursions, worst_clearance_m,
                             worst_clearance_t_s}]},
                 excursions_seen_but_not_scored: [{code, detail}]}
verdict         {result: "PASS" | "FAIL" | "INCONCLUSIVE", reasons: [{code, detail}],
                 points_measured: [...], points_unmeasured: [...]}   (every envelope point)
```

A `detail` is for people: it names the point, count, worst value and time, with numbers rounded
for reading (the exact values are in the blocks). Details are produced by fixed format strings,
so they are byte-identical everywhere too.

## Determinism

Every number that reaches the output is computed with `+ − × /`, `math.sqrt` and `math.fsum`
in a fixed order, comparisons, `floor`/`ceil`, and sorting; no built-in `sum` (it changed in
3.12), `statistics`, numpy, `math.hypot`/`dist`/`sumprod`, `**` or transcendental functions.
Shares compared against a percentage are compared in integers. `vectors/scoring/golden.json`
pins the sha256 of the JCS bytes for eight synthetic runs; `packages/core/tests/test_scoring_golden.py`
checks it (and runs as a plain script, which `--write` regenerates). The hashes are identical on
CPython 3.10, 3.11, 3.12 and 3.13.

## Known limits and open questions

- **EV-10 order.** `LIVENESS_LOW` is checked after the EV-10 adequacy reasons, so a run with too
  few or too close waypoints, a waypoint near the boundary, a short run or tracking gaps is
  INCONCLUSIVE rather than failing liveness on evidence that cannot support it. Excursions still
  FAIL first.
- **EV-10 visit weighting.** The reach fraction counts visits, so a lab that visits one waypoint
  far more often than the others weights it accordingly; a waypoint at the start pose, visited
  often, can carry a frozen gate past 90 %. Separation stops stacking waypoints, not this.
- **Robot-state match** needs fk_samples; without them the check cannot run (the block is null).
  It catches a copied track, not an instrument that is merely seeded from robot state.
- **CONTRACT.md** still names the state source `fk_tool`; the readers accept only `fk`.
