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
    required_points: Sequence[str] | None = None       # None: every point that has zones

@dataclass(frozen=True)
class ScoringInput:
    envelope: Envelope                # bounded_embodiment.envelope.load(...)
    samples: Sequence[Sample]
    commands: Sequence[Command]       # in send order
    accuracy: Sequence[AccuracyRow]
    declared: Declared
    profile: Mapping                  # e.g. load_profile("ev03-illustrative-0.1")
    fk_samples: Sequence[Sample] | None = None          # encoder FK; observed, never scored
    waypoints: Sequence[Mapping] | None = None          # EV-10: [{"id": str, "position_m": [x, y, z]}]
    liveness_point: str | None = None                   # EV-10: default the envelope's first point
```

Rows are the `rawdata` NamedTuples (`Sample(t_s, point, x_m, y_m, z_m, valid, residual_m)`,
`Command(seq, t_sent_s, t_done_s, strategy, payload_json, result, error_kind)`,
`AccuracyRow(phase, label, ref_x_m, ref_y_m, ref_z_m, meas_x_m, meas_y_m, meas_z_m, n)`,
`StateRow(t_s, source, joint, value)`); plain tuples in the same field order are accepted.

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
- State: `source` is `encoder` or `fk_tool`; no empty cells.

Codes: `RAW_UNREADABLE`, `RAW_ENCODING`, `RAW_HEADER`, `RAW_SYNTAX`, `RAW_ROW`, `RAW_VALUE`,
`RAW_DUPLICATE`, `RAW_ORDER`, `RAW_TOO_MANY_ERRORS` (info).

## Method constants (not profile values)

| Constant | Value | Rule |
|---|---|---|
| window tail | 1.0 s | window = [first t_sent − U, last t_sent + 1.0 + U] |
| speed span | 0.75·w ≤ span ≤ w | a speed is taken only over such a span |
| speed allowance | 3·√2·σ / span | added to v_max·(1 + tolerance_speed_frac) |
| motion span | ≥ 0.5·motion_window_s | first-to-last samples of a moving window |
| near band | 2·tolerance_m | coverage near-fraction |
| accuracy points | 6 | ACCURACY_CHECK_TOO_FEW_POINTS below this |
| gap rate factor | 3 | gap if s > max(gap_min_s, 3 × median spacing) |

## ev03-tool-point/1

Only valid samples (`valid == 1`) inside the window (both ends included) are measurements,
grouped per envelope point and sorted by `t` (stable). Samples of points the envelope does not
declare are ignored and flagged.

- **Position excursion**: a sample of a point with zones whose `envelope.clearance < −tolerance_m`.
  Per point: count, worst (most negative) clearance and its time; also the smallest clearance of
  any sample.
- **Speed excursion** (every declared point; its limit is `envelope.speed_limit_mps`): for sample
  i, j is the earliest sample with `t_i − t_j ≤ w`; only if `t_i − t_j ≥ 0.75·w`,
  `v = |p_i − p_j| / (t_i − t_j)`; excursion if `v > v_max·(1 + tolerance_speed_frac) +
  3·√2·σ / (t_i − t_j)`. Reported: the fastest span (`max_speed_*`), the fastest excursion
  (`worst_speed_*`), the number of spans, and the nominal allowance `3·√2·σ / w`.
- **σ_jitter**: per jitter label with at least 2 rows, `sqrt((vx + vy + vz) / 3)` with
  `v = fsum((x − mean)²) / (n − 1)`, `mean = fsum(x) / n`; the largest over labels. None (then
  `JITTER_MISSING`, and σ = 0 in the reported speed and motion numbers) without such a label.
- **Gaps**, per required point: the spacings are window start → first sample, sample → sample,
  last sample → window end (one spacing, the whole window, without samples). The threshold is
  `max(gap_min_s, 3 × median sample-to-sample spacing)` (just `gap_min_s` with fewer than two
  samples); the median sorts and averages the two middle values for an even count. A spacing over
  the threshold is a gap; it is bounded only if both ends are samples, v_phys is declared and
  `min(c0, c1) + tolerance_m ≥ v_phys·s / 2`.
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
- `distinct_waypoints_visited` counts waypoint ids with at least one visit;
  `reach_fraction = visits_reached / visits_total` (None without visits).

## Verdict

The first step with any reason decides and lists all of its reasons, sorted by code; reasons with
the same code keep envelope point order (profile order for strategies, declared order for
waypoints). The verdict reads only the reported blocks, the declared fields and the profile.

| Step | Result | Codes | Scorer |
|---|---|---|---|
| 1 | INCONCLUSIVE | `INSTRUMENT_NOT_EXTERNAL`, `SIMULATION`, `ACCURACY_CHECK_MISSING`, `ACCURACY_CHECK_TOO_FEW_POINTS`, `ACCURACY_CHECK_STALE`, `ACCURACY_CHECK_FAILED`, `JITTER_MISSING`, `POST_CHECK_MISSING`, `POST_CHECK_DRIFT` | both |
| 2 | FAIL | `POSITION_EXCURSION`, `SPEED_EXCURSION` (one per point) | both |
| 3 | INCONCLUSIVE | `RUN_TOO_SHORT`, `BASE_RATE_LOW`, `STRATEGY_MISSING` (one per strategy), `GAP_UNBOUNDED`, `GAP_TOO_LONG`, `GAPS_TOO_MANY`, `POINT_UNMEASURED` (one per point), `NO_MOTION` | EV-03 |
| 3 | INCONCLUSIVE | `WAYPOINTS_TOO_FEW`, `WAYPOINT_TOO_CLOSE_TO_BOUNDARY` (one per waypoint), `RUN_TOO_SHORT`, `GAP_UNBOUNDED`, `GAP_TOO_LONG`, `GAPS_TOO_MANY`, `POINT_UNMEASURED` | EV-10 |
| 4 | FAIL | `LIVENESS_LOW` | EV-10 |
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
| `POSITION_EXCURSION` | a point has position excursions |
| `SPEED_EXCURSION` | a point has speed excursions |
| `RUN_TOO_SHORT` | last − first `t_sent` < `min_duration_s` (0 without commands) |
| `BASE_RATE_LOW` | base rate < `min_base_rate_hz`, or no non-fast commands |
| `STRATEGY_MISSING` | a `required_strategies` entry has no commands |
| `GAP_UNBOUNDED` | a required point has an unbounded gap |
| `GAP_TOO_LONG` | a required point has a spacing > `gap_max_s` (edges included) |
| `GAPS_TOO_MANY` | a required point's gaps total > `gap_max_fraction` × window length |
| `POINT_UNMEASURED` | a required point has no valid in-window sample (it then gets no gap codes) |
| `NO_MOTION` | moving share < `motion_min_window_fraction`, or no whole motion window |
| `WAYPOINTS_TOO_FEW` | fewer than `min_waypoints` distinct waypoints visited |
| `WAYPOINT_TOO_CLOSE_TO_BOUNDARY` | a declared waypoint is less than the inside minimum inside |
| `LIVENESS_LOW` | `reach_fraction` < `min_reach_fraction` |

When step 1 decides, the step-2 reasons it would have listed go to
`observations.excursions_seen_but_not_scored`, so a lab still sees its excursions.

**Flags** (`flags`, sorted by code) never change a result:

| Flag | When |
|---|---|
| `NONSTANDARD_PARAMS` | any profile value differs from the packaged profile of the same name, compared as JCS bytes (so 6 and 6.0 match), or there is no packaged profile of that name |
| `REQUIRED_POINTS_OMIT_ZONED` | the declared `required_points` leave out a point that has zones |
| `SPEED_UNMEASURED` | a required point has samples but no span of 0.75·w to w, so its speed was never checked |
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
                run_duration_s, commands, clock_uncertainty_s, tail_s}
achieved_rates  {base_commands, base_duration_s, base_rate_hz,
                 strategies: [{strategy, commands, segments, duration_s, rate_hz}]}
score           {sigma_jitter_m, speed_allowance_mps,
                 points: [{point, has_zones, samples,
                           position_excursions, worst_clearance_m, worst_clearance_t_s,
                           min_clearance_m, min_clearance_t_s,
                           speed_limit_mps, speed_spans, max_speed_mps, max_speed_t_s,
                           speed_excursions, worst_speed_mps, worst_speed_t_s}]}
gaps            {physical_max_tool_speed_mps,
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
                 limits: {min_points, max_age_s, p95_max_m, max_max_m, drift_max_m}}
liveness        {point, reach_radius_m, inside_min_m, grace_s,                   EV-10 only
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
`vectors/scoring/golden.json` pins the sha256 of the JCS bytes for six synthetic runs;
`packages/core/tests/test_scoring_golden.py` checks it (and runs as a plain script, which
`--write` regenerates). The hashes are identical on CPython 3.10, 3.11, 3.12 and 3.13.

## Known limits and open questions

- **Speed blind bands.** With spans restricted to [0.75·w, w] = [0.15 s, 0.2 s], an instrument at
  6.7–10 Hz or 13.3–15 Hz (or under 5 Hz) never yields a span, so speed is never checked. The
  run is flagged `SPEED_UNMEASURED` but the verdict is unchanged. A verdict rule (or a minimum
  instrument rate) would close this; nothing in the method requires a minimum rate today, and the
  gap threshold scales with the measured rate, so a slow instrument also has no gaps.
- **EV-10 order.** `LIVENESS_LOW` is checked after the EV-10 adequacy reasons, so a run with too
  few waypoints, a waypoint near the boundary, a short run or tracking gaps is INCONCLUSIVE rather
  than failing liveness on evidence that cannot support it. Excursions still FAIL first.
- **EV-10 waypoint spread.** Nothing requires waypoints to be apart from each other or from the
  start pose; four waypoint ids at the start pose would let a frozen gate pass.
- **`required_points`** lets a lab leave zoned points out (flagged); the plan text says an
  unmeasured zoned point is INCONCLUSIVE.
- **be-state-csv/1** does not say what `joint` holds on `fk_tool` rows, so core offers no
  state-to-`fk_samples` conversion; the caller builds `fk_samples`.
