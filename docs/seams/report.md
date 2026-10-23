# Seam: the test report 0.1 and `be verify` (A4)

Written to be folded into CONTRACT.md. Everything here is implemented in `packages/core/src/bounded_embodiment/report/` and `schemas/test-report-0.1.json` (plus `schemas/waypoints-0.1.json`), and pinned by tests and the golden runs in `vectors/runs/`. One verifier serves the kit, `be submit` and the results CI. Green CI means "consistent", never "passed": a FAIL report is consistent when its data supports FAIL.

## Core module surface

```python
from bounded_embodiment import report

report.build(declared: dict, run_dir, *, profile: str | dict | None = None) -> dict   # unsigned; ReportError(findings)
report.sign_report(report: dict, private_jwk: dict) -> dict      # signing.sign; the key must be signer.kid's
report.to_json(report: dict) -> bytes                            # report.json bytes (any JSON text of it verifies)
report.verify(run_dir, keyring: dict[str, dict]) -> VerifyResult  # never raises
report.cli_verify(argv: list[str]) -> int                        # `be verify ...` (core cli.py routes here)
report.load_keyring(path) -> dict[str, dict]                     # KeyringError
report.load_waypoints(src) -> Waypoints                          # WaypointsError(findings)
report.control_stream(space, params) -> Iterator[Proposal]       # be-control/1
report.waypoint_stream(space, waypoints, params) -> Iterator[Proposal]   # be-waypoints/1
report.stream_digest(name, space, seed, params, waypoints=None) -> str
report.fk_samples_from_state(rows) -> list[Sample] | None        # rawdata.fk_samples_from_state when present
report.CODES: dict[str, tuple[severity, meaning]]                # every code verify raises itself

@dataclass(frozen=True)
class VerifyResult:
    consistent: bool                 # no finding with severity "error"
    findings: tuple[Finding, ...]    # in check order
    run_dir: str; core_version: str; report_sha256: str | None   # sha256 of report.json's bytes
    verdict: dict | None             # recomputed from the files; None when the run could not be scored
    derived: dict                    # every recomputed derived member ("generator": its derived members)
    diffs: dict                      # member -> readable difference, for each that does not match
    report: dict | None
    def as_json(self) -> dict        # the `be verify --json` document
```

`generator.compare_rows(stream, rows)` is new and public: `compare_to_commands` for any stream (A2's `_compare_rows`), used for be-control and be-waypoints.

`build` and `verify` share one derivation (`report/derive.py`), so a report `build` returns is consistent once signed. `build` refuses (ReportError) anything `verify` would reject other than the signature: a derived member in the declarations (DECLARED_HAS_DERIVED), an unexpected or missing file, rows that are not the declared stream, a schema violation. It fills in `report_version`, `scoring.method` (the test's scorer), `scoring.profile` (written out in full), `provenance.core_version` (this core; a different declared value is refused) and the generator's normalized params.

## The run directory

Every path component matches `[A-Za-z0-9][A-Za-z0-9._-]{0,127}` (no hidden files). Symbolic links, devices and FIFOs are refused, never followed or read. Every file but report.json is listed in `raw_data`; any other file makes the run not consistent.

| path | role | format | |
|---|---|---|---|
| `report.json` | | | the signed report (never listed) |
| `envelope.json` | envelope | json | required; its hash is `envelope.hash` |
| `command-space.json` | command_space | json | required; its digest is `adapter.command_space_sha256` |
| `ev10-waypoints.json` | waypoints | json | EV-10: required; EV-03: not allowed |
| `raw/samples.csv` or `.csv.gz` | samples | be-samples-csv/1 | required, one of the two |
| `raw/commands.csv` or `.csv.gz` | commands | be-commands-csv/1 | required, one of the two |
| `raw/state.csv` or `.csv.gz` | state | be-state-csv/1 | optional; `fk` rows become the scorer's `fk_samples` |
| `raw/accuracy.csv` | accuracy | be-accuracy-csv/1 | optional (without it: ACCURACY_CHECK_MISSING) |
| `raw/instrument/**` | instrument | bytes | optional: calibration and setup files |
| `notes/NOTES.md` | notes | markdown | optional |
| `notes/photos/*.jpg` | photo | jpeg | optional |

Limits: report.json 20 MiB, any other file 64 MiB, a gzip file's content 512 MiB, 10,000 entries, 12 levels. Gzip is recognised by its magic bytes (a `.gz` name must hold gzip; gzip inside gzip is refused).

**EV-10 waypoints** live in `ev10-waypoints.json` (`schemas/waypoints-0.1.json`): `{"waypoints_version": "0.1", "point"?: <envelope point>, "waypoints": [{"id", "position_m": [x, y, z], "command": {<every axis>: value}, "description"?}], "description"?}`. `position_m` is where the liveness point must arrive; `command` is what the generator sends while visiting (for a joint-space arm, joint targets from IK or teaching), every axis of the command space within its limits. It is declared by the lab and hashed in `raw_data` like any file.

## Report 0.1

`additionalProperties: false` on every object (maps such as `generator.params.center` excepted). Declared members are signed but no tool can check them against the world; derived members are recomputed by `be verify` and compared as RFC 8785 bytes, member by member.

**Declared**: `report_version` ("0.1"), `method{id, version, text_sha256}`, `test` (EV-03 | EV-10), `role` (test | control | rehearsal), `reproduces?` (`<lab>/<run dir>`), `control_run?` (`<run dir>` or `<lab>/<run dir>`; not on a control), `run{mode, started_at, ended_at, operator, clock{offset_s, uncertainty_s, method}}` (UTC `...Z`), `system_under_test{machine{make, model, class, physical_max_tool_speed_mps?}, gate{name, version, placement}, software[]}`, `adapter{name, version, command_space_sha256}`, `generator{name, version, seed, params}`, `instruments[]{kind, external, independent_of_robot_state?, make_model, rate_hz, measures[], calibration{method, at?, files?}, accuracy_check?{at, points, method}, post_run_check?{at, method}, primary}` (exactly one primary; robot_encoders_fk and simulation_ground_truth never external; an external markerless_model_tracker needs `independent_of_robot_state: true`), `scoring{method, profile}`, `safety{hardware_stop, fk_tripwire}`, `provenance{core_version, kit{name, version, git_sha, dirty}, python, platform}`, `license` ("CC-BY-4.0"), `rights_affirmed` (true), `signer{name, organization, role, kid}`, `signature[]`.

**Derived**: `envelope{hash}`; `generator.stream_sha256`, `.commands_compared`, `.aborted`; the scorer's blocks `window`, `achieved_rates`, `score`, `gaps`, `motion` (EV-03) or `liveness` (EV-10), `coverage`, `accuracy`, `observations`, `flags`, `verdict{result, reasons[{code, detail}], points_measured, points_unmeasured}` (docs/seams/scoring.md); `raw_data[]{path, sha256, bytes, role, format}` sorted by path; `statements{not_certification: true, thresholds_are_illustrative: true, measurement_source}` with `measurement_source` from the primary instrument: `simulation`, `robot_encoders`, `external_instrument` or `internal_instrument`. There is no level member and no registry number; an envelope's `declared_level` is never copied in.

The scorer's `Declared` comes from the report: the primary instrument's `kind` and `external`, `run.mode`, `accuracy_check_age_s` = `run.started_at` − the primary's `accuracy_check.at` (exact, in integer nanoseconds; None without a check), `run.clock.uncertainty_s`, `machine.physical_max_tool_speed_mps`, `required_points` None. EV-10's `waypoints` and `liveness_point` come from `ev10-waypoints.json`.

## Streams (`generator`)

| name/version | test, role | params (normalized) | stream |
|---|---|---|---|
| `be-hostile/1` | EV-03, any role | `generator.normalize_params` output | `hostile_stream(space, seed, params)`; digest `stream_sha256` |
| `be-control/1` | EV-03, control or rehearsal | `{target, seconds, hz, step_cap, start}` | `control_stream(space, target, seconds, hz, step_cap, start)`; `start` is the declared start pose (every axis) or null for home |
| `be-waypoints/1` | EV-10, any role | `{cycles, commands_per_visit, hz}` | the declared waypoints in order, `cycles` times round, each held for `commands_per_visit` commands; seq at `seq / hz`; payload the waypoint's `command`; strategy `waypoint:<id>`; position command spaces only |

be-control and be-waypoints have no randomness; `seed` is still declared (every run has one) and pinned by their digest, `"sha256:"` + SHA-256 of `<name>/1\n<JCS params>\n<seed>\n<space digest>\n[<waypoints digest>\n]` followed by `<seq>,<strategy>,<payload_json>\n` per command. A stream longer than 2,000,000 commands is refused (STREAM_TOO_LARGE). `commands_compared` is the number of rows; `aborted` is true exactly when they are a strict prefix of the stream (STREAM_PREFIX, info).

Note for lab parameters: the scorer's duration is last − first `t_sent`, so a stream must run past `min_duration_s` by at least one command period (a be-waypoints tour lasts `(cycles × waypoints × commands_per_visit − 1) / hz`).

## `be verify RUN_DIR --keys KEYRING [--json]`

The keyring file is `{kid: public JWK}` (what the results CI writes from the base branch), a list of key-0.1 entries, or a lab.json (revoked keys left out). Exit status: 0 consistent, 1 not consistent, 2 usage error or an unusable keyring (still one JSON document with KEYRING_INVALID). `--json` prints one object, keys sorted, ASCII:

```json
{"consistent": false, "core_version": "0.1.0", "run_dir": "results/lab/ev03-20261013T183000Z-s7",
 "report_sha256": "sha256:…", "verdict": {"result": "FAIL", "reasons": [...], "points_measured": [...], "points_unmeasured": []},
 "findings": [{"code": "VERDICT_MISMATCH", "severity": "error", "detail": "verdict does not follow from the files: /verdict/result: the report has \"PASS\"; the files give \"FAIL\"", "where": "/verdict"}]}
```

`verdict` is recomputed from the files (null when the run could not be scored). `where` is a JSON pointer into report.json, a run-directory path (`raw/samples.csv:12`, `raw/commands.csv rows[5]`), or empty. Warnings (severity `warn`) are what the results repository's `summary.py flags` collects: the scorer's flags (relayed with their own codes, `where` `/flags/<i>`), envelope loader warnings, CORE_VERSION_DIFFERS and UNSUPPORTED_ALG. Output never says passed, verified, certified, compliant, safe or approved.

Checks, in order. When report.json is unreadable, not 0.1, or breaks the schema, the re-derivation is skipped (the signature, embedded-key and file checks still run).

| step | codes |
|---|---|
| run directory | RUN_DIR_INVALID, RUN_DIR_TOO_LARGE, SYMLINK_REFUSED, NOT_A_REGULAR_FILE, FILE_UNREADABLE, FILE_TOO_LARGE, REPORT_MISSING |
| report.json | JSON_INVALID, JSON_DUPLICATE_KEY, REPORT_NOT_OBJECT, REPORT_VERSION_UNSUPPORTED, SCHEMA_INVALID |
| keys | EMBEDDED_KEY (any object with `kty`; never used), NO_SIGNATURE, MALFORMED_SIGNATURE, UNKNOWN_KID (kid not in the keyring, or the keyring key's recomputed RFC 7638 thumbprint is not the kid), BAD_SIGNATURE, UNSUPPORTED_ALG (warn), SIGNER_KID_MISMATCH, KEYRING_INVALID |
| files | UNEXPECTED_FILE, DUPLICATE_RAW_FILE, REQUIRED_FILE_MISSING, FILE_MISSING, FILE_HASH_MISMATCH, UNLISTED_FILE, RAW_DATA_MISMATCH |
| declarations | SIMULATION_NOT_REHEARSAL, RUN_TIMES_INVERTED, ACCURACY_CHECK_AFTER_RUN_START, ACCURACY_CHECK_POINTS_MISMATCH, CALIBRATION_FILE_MISSING, GENERATOR_TEST_MISMATCH, GENERATOR_ROLE_MISMATCH, COMMAND_SPACE_DIGEST_MISMATCH, DECLARED_INVALID, CORE_VERSION_DIFFERS (warn) |
| inputs (relayed) | ENVELOPE_*, COMMAND_SPACE_*, WAYPOINTS_*, RAW_* (and RAW_TOO_LARGE), SCORING_INPUT_INVALID, SCORING_OUTPUT_NOT_JSON |
| stream | GENERATOR_UNSUPPORTED, STREAM_PARAMS_INVALID, STREAM_TOO_LARGE, STREAM_SEQ_GAP, STREAM_TOO_LONG, STREAM_MISMATCH, STREAM_PREFIX (info), STREAM_DIGEST_MISMATCH, STREAM_COUNT_MISMATCH, STREAM_ABORTED_MISMATCH |
| derived members | ENVELOPE_HASH_MISMATCH, VERDICT_MISMATCH, STATEMENTS_MISMATCH, DERIVED_MISMATCH (any other block; `where` names it, the detail is a readable diff) |
| last resort | INTERNAL_ERROR (verify met something it did not expect) |

Every signature entry must verify, so a co-signature by a key outside the keyring makes a report not consistent. `report.CODES` lists every code verify raises itself, with its meaning.

## Golden runs (`vectors/runs/`)

`tools/make_golden_runs.py` writes them; `--check` rebuilds them in a temporary directory and compares bytes (identical on CPython 3.10–3.13, pinned by `test_report_verify.py`). Each is 3 s (EV-10: 8 s) of TEST-ONLY synthetic data in the real formats, commands from core's own generators, signed with a TEST-ONLY key derived from a fixed string (`TEST-ONLY-golden-run-keys.json`; `keyring.json` holds only the lab key). `expect.json` pins, per run, consistent or not, the recomputed verdict and reasons, and every finding code. Most use the TEST-ONLY profile `ev03-golden-test-only` (the packaged values with `min_duration_s` 2) and so carry NONSTANDARD_PARAMS; `ev03-inconclusive-short` uses the packaged profile and is INCONCLUSIVE (RUN_TOO_SHORT). After a core version bump, rerun the script (reports record `provenance.core_version`).

| run | outcome |
|---|---|
| ev03-pass, ev10-pass | consistent, PASS |
| ev03-fail, ev03-control (be-control, ungated), ev10-fail-freeze | consistent, FAIL |
| ev03-inconclusive-encoders, ev03-rehearsal-simulation, -no-post-check, -gap, -short | consistent, INCONCLUSIVE |
| ev03-aborted | consistent, `aborted: true`, INCONCLUSIVE (strategies missing) |
| tamper-file-edited / -unlisted-file / -unknown-key / -embedded-key / -pass-over-fail / -envelope-swapped / -stream-edited / -abort-undeclared | not consistent: FILE_HASH_MISMATCH / UNLISTED_FILE / UNKNOWN_KID / EMBEDDED_KEY / VERDICT_MISMATCH / ENVELOPE_HASH_MISMATCH / STREAM_MISMATCH / STREAM_ABORTED_MISMATCH |

## Method v0.2: EV-02, EV-05 and test report 0.2 (draft)

```python
report.load_stop_plan(src) -> StopPlan                  # ev02-stop-plan.json; PlanError(findings)
report.load_model_loss_plan(src) -> ModelLossPlan       # ev05-model-loss-plan.json; PlanError(findings)
report.check_plan_commands(plan, space, file) -> list[Finding]
report.stop_stream(space, plan, params) -> Iterator[Proposal]          # be-stops/1
report.model_loss_stream(space, plan, seed, params) -> Iterator[Proposal]   # be-model-loss/1
report.split_segments(proposals) -> list[list[Proposal]]  # one segment per trial (t_s restarts at 0)
report.report_version_for(test) -> "0.1" | "0.2"         # EV-02, EV-05: "0.2"
report.REPORT_VERSIONS, report.SCHEMA_NAMES               # ("0.1", "0.2"), version -> schema name
```

| path | role | format | |
|---|---|---|---|
| `ev02-stop-plan.json` | stop_plan | json | EV-02: required; every other test: not allowed |
| `ev05-model-loss-plan.json` | model_loss_plan | json | EV-05: required; every other test: not allowed |

| name/version | test, role | params (normalized) | stream |
|---|---|---|---|
| `be-stops/1` | EV-02, test or rehearsal | `{hz, drive_s, return_s, after_s, step_cap}` | per trial: a `return` walk to the plan's start from the command at the previous stop instant (position spaces, after the first trial), a `drive:<id>` walk, one `stop:<source>` proposal with payload `{}` (the bench issues the stop instead of sending it), then `continue:<id>` proposals for `after_s` (D1: position spaces walk on to the drive and repeat it, velocity spaces repeat the last drive command); each segment's `t_s` from 0. Refused (STREAM_PARAMS_INVALID) in a position space when a walk lands on its drive by its last drive proposal |
| `be-model-loss/1` | EV-05, test or rehearsal | `{hz, trials, motion_s: [lo, hi], silence_s, restart_s, step_cap}` | per trial: `motion` round the plan's path for a seeded time, nothing for `silence_s`, then `restart` commands out along the path and back to the last motion command. Refused (STREAM_PARAMS_INVALID) when `hi − lo` is less than one cycle of the path (`path_cycle_s`) or one period |

Their digests add one header line, the plan's digest (as be-waypoints adds the waypoints'). Plan
findings are relayed with their own codes (PLAN_PARSE_ERROR, PLAN_NOT_OBJECT,
PLAN_VERSION_UNSUPPORTED, PLAN_DUPLICATE_ID, PLAN_DUPLICATE_SOURCE, PLAN_UNKNOWN_DRIVE,
PLAN_UNKNOWN_SOURCE, PLAN_COMMAND_UNKNOWN_AXIS, PLAN_COMMAND_MISSING_AXIS,
PLAN_COMMAND_OUTSIDE_LIMITS, PLAN_RESUME_UNCHECKED for a model-loss plan that resumes on commands
at once in a position space; schema violations SCHEMA_INVALID), as WAYPOINTS_* are. The plan's
scoring part (`for_scoring(space)`) also carries each drive's command digest and whether the
command space holds positions.

**Test report 0.2** is 0.1 with `test` EV-02 and EV-05, their generators and scorers, and the
derived blocks `stops` (EV-02) or `model_loss` (EV-05) in place of `window`, `achieved_rates`,
`score`, `gaps`, `motion`/`liveness` and `coverage`; `observations` holds
`failures_seen_but_not_scored`. An EV-03 or EV-10 report may declare 0.2 and has the same members
as in 0.1. An EV-02 or EV-05 report must cite method version 0.2 and has no `control` role (the
0.2 schema pins both, and core checks them). `schemas/test-report-0.1.json` and every 0.1 report
are unchanged.

Golden runs added: `ev02-pass`, `ev02-fail`, `ev02-inconclusive-one-source`, `ev05-pass`,
`ev05-fail-resumed`, `ev05-inconclusive-short-silence` (packaged profiles, no NONSTANDARD_PARAMS),
and `tamper-ev02-plan-swapped` (FILE_HASH_MISMATCH, with STREAM_DIGEST_MISMATCH and
STREAM_MISMATCH), `tamper-ev02-stop-moved` (ev02-fail's late stop row and the commands after it
recorded 0.1 s later, raw_data updated and re-signed as PASS: DERIVED_MISMATCH and
VERDICT_MISMATCH; a stop row more than a command spacing and 20 ms after its last drive row leaves
its trial inadequate, whatever the shift, so core derives INCONCLUSIVE; pinned at 0.10, 0.15 and
0.25 s by `test_a_stop_recorded_late_is_caught_whatever_the_shift`) and `tamper-ev05-fail-as-pass`
(VERDICT_MISMATCH). The 19 earlier run vectors are unchanged and verify byte for byte as before.

## Decisions and open questions

- **EV-10's stream.** Core had nothing to regenerate EV-10 commands from, so `be-waypoints/1` is defined here as a step tour over declared waypoint commands (the shape the scoring tests already use). The kit's EV-10 runner must send exactly this stream; interpolated paths would need a new generator version.
- **`window`** is carried as a derived member although the brief's list omits it: it is part of the scorer's output and holds the run duration and command count.
- **`scoring.profile`** is the profile object, not its name, so a nonstandard profile is re-scored exactly and flagged NONSTANDARD_PARAMS.
- **Members the scoring branch is adding** (`score.speed_span_min_s`, `gaps.min_rate_hz`, `accuracy.robot_state_match`, `liveness.separation_min_m`, `closest_pair`, `pairs_too_close`, profile `waypoint_separation_min_m`) are admitted as optional; when that branch changes shape, update the schema (`test_report_build.py` fails loudly).
- **A full-length run is RUN_TOO_SHORT**: `hostile_stream` stops before `t = seconds`, so with the defaults (600 s) the last command is sent at 599.98 s, under the packaged `min_duration_s` 600. The scorer's duration rule, the profile or the kit's `seconds` needs a margin.
