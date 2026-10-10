# Seam: `be report` and `be submit` (kit)

From a run directory to a signed submission, and from a submission into a clone of the results
repository. Everything here is in `packages/kit/src/bounded_embodiment_kit/submission/`, tested
without hardware in `packages/kit/tests/test_submission_*.py`. Core is untouched: the kit packages
files and declares fields; core's `report.build` hashes the files and derives every derived
member (the verdict among them), and core's `report.verify` checks. The kit never writes a score
or a verdict.

| Module | What it does |
|---|---|
| `report.py` | `be report`: inputs, packaging, declared fields, then core's build, sign and verify |
| `submit.py` | `be submit`: core's verify with the lab's keys, the repository's rules, the copy, its tools |
| `timebase.py` | instrument clock to run timebase (`--clock monotonic|wall|run`), exact decimal conversion |
| `scrub.py` | hostnames, usernames, home paths and IP addresses out of text, JSON and CSV |
| `files.py` | deterministic gzip, JPEG metadata removal, private-key and Git LFS checks |
| `labconfig.py`, `lab-config-0.1.json` | the lab config (`be-lab-config/0.1`), validated strictly |
| `ids.py` | run directory names (`<test>-<UTC>-s<seed>`) and result ids |
| `cli.py` | registers `report` and `submit` (in `bounded_embodiment_kit.cli.MODULES`) |

## `be report`

```bash
be report RUN_DIR --lab-config LAB.json --key KEY.json --out SUBMISSION_DIR \
    [--samples PATH ...] [--clock monotonic|wall|run] [--clock-csv PATH] [--wall-clock-agreement-s S] \
    [--accuracy PATH] [--instrument-file PATH ...] [--notes NOTES.md] [--photo JPG ...] \
    [--test EV-03|EV-10] [--role test|control|rehearsal] [--reproduces RESULT] [--control-run RUN] \
    [--redact TEXT ...]
```

Put RUN_DIR first: `--samples`, `--photo` and `--instrument-file` take several paths. Exit 0 written
and consistent, 1 refused (nothing is left behind), 2 usage error.

**Order.** Everything is packaged first, into a staging directory beside SUBMISSION_DIR
(`.NAME.partial-*`); then `report.build(declared, staging)`, `report.sign_report(report, key)`,
report.json (`report.to_json`), and `report.verify(staging, {kid: public half})`. Only a
consistent submission is renamed to SUBMISSION_DIR, which must not exist or be empty.

| Submission file | From | How |
|---|---|---|
| `envelope.json`, `command-space.json`, `ev10-waypoints.json` (EV-10) | RUN_DIR | byte copies; refused if their hash/digest differs from declared.json's; scanned, not scrubbed (below) |
| `raw/commands.csv.gz`, `raw/state.csv.gz` | RUN_DIR/raw (`.csv` or `.csv.gz`) | scrubbed CSV, gzipped |
| `raw/samples.csv.gz` | `--samples` (several are joined in order), else the runner's simulated instrument's RUN_DIR/raw/samples.csv | converted to the run timebase, scrubbed, gzipped |
| `raw/accuracy.csv` | `--accuracy` | checked with core's reader, scrubbed, **not** gzipped (core's layout has no `raw/accuracy.csv.gz`) |
| `raw/instrument/**` | RUN_DIR/raw/instrument/** and `--instrument-file` | scrubbed by kind (JSON, CSV, text; JPEG metadata stripped; binary copied with a warning) |
| `raw/instrument/recorder-clock.csv` | the clock.csv a clock method used | as evidence for the conversion |
| `raw/instrument/timebase.json` | be report | `be-timebase/0.1`: method, offset, U and each of its terms, the runner's clock readings, the source files' sha256 and row counts |
| `notes/NOTES.md` | `--notes` | scrubbed |
| `notes/photos/*.jpg` | `--photo` | metadata stripped; named `<stem>.jpg`, or `photo-NN.jpg` when the stem is not a valid name or holds a scrubbed word |
| `report.json` | core | built, signed, verified |

Gzip is written by the kit itself: a fixed header (no name, mtime 0, XFL 2, OS 255), one raw
deflate stream at level 9, CRC-32 and size; the bytes depend only on the content (and on the zlib
that deflates it). Two runs of `be report` on the same inputs with the same key give byte-identical
submissions (tested): Ed25519 is deterministic and nothing depends on the time.

**Defaults.** `--test` comes from declared.json (`ev03` is EV-03) and is only checked when given.
`--role` defaults to rehearsal for a simulation or a shakedown, control for a control run, else
test; a simulation must be a rehearsal, the control stream makes a control or rehearsal run, and a
control run takes no `--control-run`. `--reproduces` is a result id `<lab>/<run>` or a path ending
in `results/<lab>/<run>`; `--control-run` also takes a bare run name or a local submission
directory (named from its report.json).

## The lab config (`be-lab-config/0.1`)

The declared fields the runner cannot know. `lab-config-0.1.json` takes each definition from the
test report 0.1 schema by reference (`$ref` into core's packaged schema), so a lab config that
validates gives members the report schema accepts; unknown members are refused at every level;
the file is parsed strictly (no duplicate keys, NaN or Infinity).

```json
{
  "format": "be-lab-config/0.1",
  "machine": {"make": "Hugging Face", "model": "withheld", "class": "arm", "physical_max_tool_speed_mps": 1.0},
  "gate": {"name": "example-gate", "version": "0.3.1", "placement": "separate_process"},
  "software": [{"name": "my-stack", "version": "2.1", "role": "model runtime"}],
  "instruments": [{
    "kind": "markerless_model_tracker", "external": true, "independent_of_robot_state": true,
    "make_model": "Luxonis OAK-D, be fit 0.1", "rate_hz": 15, "measures": ["tool", "elbow", "wrist"],
    "calibration": {"method": "ChArUco board registration", "files": ["raw/instrument/registration.json"]},
    "accuracy_check": {"at": "2026-10-13T17:40:00Z", "points": 8, "method": "tape-surveyed points"},
    "post_run_check": {"at": "2026-10-13T18:45:00Z", "method": "static re-survey"},
    "primary": true}],
  "signer": {"name": "A. Person", "organization": "Example Lab", "role": "lab lead"},
  "operator": "lab technician",
  "method": {"id": "bounded-embodiment", "version": "0.1", "text_sha256": "sha256:<64 hex>"}
}
```

`software` is optional. `accuracy_check.at` and `post_run_check.at` change from session to session.
Beyond the schema, be report refuses instruments that measure a point the envelope does not
declare, and calibration files the submission does not hold.

## Declared fields

| Report member | From |
|---|---|
| `method`, `system_under_test.machine`, `.gate`, `instruments`, `run.operator` | the lab config |
| `signer` | the lab config, plus the key's kid |
| `test`, `role`, `reproduces`, `control_run` | declared.json and the flags (above) |
| `run.mode` | declared.json `simulation` |
| `run.started_at`, `run.ended_at` | declared.json `timestamps` (`wall_start_s`/`wall_end_s` to the microsecond; an older runner's whole-second `started_at`/`ended_at`) |
| `run.clock` | the clock method (below) |
| `system_under_test.software` | the runner's kit and core versions (declared.json `software`), LeRobot's when the adapter reports it, then the lab config's `software` |
| `adapter` | declared.json `adapter.adapter`, the runner's kit version, the digest of command-space.json |
| `generator` | declared.json `generator`; the runner's `control` block becomes `be-control/1` with seed 0 (no randomness, and the runner records no seed for it: the golden control run does the same) |
| `safety` | declared.json `safety.hardware_stop`, `.fk_tripwire` |
| `provenance.kit` | this kit's name and version, and its git commit and dirty state when it runs from a checkout of this repository (null from a wheel); `python`, `platform`; core fills `core_version` |
| `license`, `rights_affirmed` | `CC-BY-4.0`, `true`: the signer affirms them by signing |
| `scoring` | left to core: the test's scorer and its packaged illustrative profile |

## The timebase (`--clock`)

The run timebase is the runner's `time.monotonic()` with run start = 0. The runner writes
commands.csv and state.csv on it; `be fit` writes samples on the recording host's
CLOCK_MONOTONIC. `be report` adds one offset to every sample's `t_s` (run time = instrument time +
`offset_s`) and declares `run.clock{offset_s, uncertainty_s, method}`. The offset is rounded to the
microsecond and added in decimal, so a converted cell is exact (at least six decimals, more if it
had more); U is rounded up to the microsecond. Nothing else in a samples file changes.

| Method | When | offset | U |
|---|---|---|---|
| `monotonic` | the instrument ran on the runner's host (be fit on the robot's computer) | −`monotonic_start_s` | 1 µs: the offset's rounding plus the clock's resolution |
| `wall` | another host; the recorder's clock.csv pairs `time.time()` with `time.monotonic()` every 5 s | c − W0: c the midrange of wall − monotonic over the pairs within 10 s of the run, W0 the runner's wall start | the hosts' wall-clock agreement as the lab declares it (`--wall-clock-agreement-s`, 0 for one host) + half the range of c over those pairs + the largest pair-reading uncertainty + 1 µs (clock.csv's six decimals) + W0's uncertainty + the runner's wall-minus-monotonic drift from start to end + the offset's rounding |
| `run` | the samples are on the run timebase already (the default, and the only choice, for the runner's simulated instrument) | 0 | 0 |

`monotonic` is cross-checked when a clock.csv is at hand: the recorder's wall − monotonic at the
pair nearest the run start must equal the runner's within 50 ms, or the two are not one clock
(another host, a reboot) and the method is refused. `--clock-csv` defaults to `SESSION/clock.csv`
when `--samples` is `SESSION/fit/samples.csv`. With an older runner's whole-second `started_at`,
W0 is the middle of that second and its uncertainty 0.5 s. After conversion at least one valid
sample must fall in the scoring window [first command − U, last command + 1 s + U]; none means the
clock method or its inputs are wrong, and be report refuses. Samples from two `--samples` files at
the same (t_s, point) are refused as overlapping instrument output.

**The runner change.** declared.json's `timestamps` now also holds `monotonic_start_s` (the
`time.monotonic()` value that is t = 0), `wall_start_s` and `wall_start_uncertainty_s` (`time.time()`
read between two monotonic readings, the tightest of five; half the bracket), `monotonic_clock` and
`monotonic_resolution_s` (`time.get_clock_info`), and at the end `monotonic_end_s`, `wall_end_s` and
`wall_end_uncertainty_s`. `runner/session.py`'s `make_clock()` returns a `RunClock`, still a callable
giving seconds since t = 0, which keeps these; `runner/run.py` writes them (it, not session.py, writes
declared.json). `started_at` and `ended_at` keep their whole-second form.

## Scrubbing

`scrub.py`'s docstring has the rules; in short, in this order: home-directory paths become `~` and
their last component (`/home/ana/runs/a/notes.md` -> `~/.../notes.md`; Linux, macOS, Windows, WSL,
`/root`, `~user`, and this machine's own home wherever it is); IPv6 and IPv4 addresses become `[ip]`;
`*.local`, `*.lan`, `*.localdomain`, `*.internal`, `*.home.arpa` and this machine's hostname (and its
first label, any case) become `[host]`; this machine's login names (matching case, so a signer named
"Ana" survives the login "ana") become `[user]`; `--redact` words (any case) become `[redacted]`.
Generic names (`pi`, `root`, `ubuntu`, `runner`, `raspberrypi`, `robot`, `localhost` ...) and names
under three characters are not looked for. Whole word means not next to a letter or digit.

What it touches: every text file in the submission and every declared field, except ids, digests and
the generator's inputs (`generator`, `scoring`, `signer`, `measures`, `calibration.files`, the
digests, `reproduces`, `control_run`). JSON is parsed and its strings (keys too) scrubbed, then
written again only if something changed (escaped forms like `\/home\/ana` are caught). In CSV only
text cells change: never the header, never a number, never `strategy` or `payload_json` (core
regenerates and compares them), point and joint ids, or `source` and `phase`. A file nothing was
found in keeps its bytes. be report prints what it replaced per file, by category and count, never
the values.

envelope.json, command-space.json and ev10-waypoints.json are copied byte for byte: their hash
identifies what the run used (the gate loaded those bytes), so scrubbing them would make the report
describe a different setup. What the scrubber would replace there is printed as a warning, to fix
in the file before the next run.

## `be submit`

```bash
be submit SUBMISSION_DIR --results-repo PATH --lab LAB [--allow-dev-core]
```

It only checks and copies; it never commits, pushes or opens a pull request. Exit 0 copied, 1
refused (a copy already made is removed), 2 usage error.

1. `labs/LAB/lab.json` in the clone gives the keys (core's `load_keyring`; revoked keys left out).
   CI reads that file from `main`, so the lab must be merged first.
2. Core's `report.verify` with those keys: refused unless consistent.
3. A simulation must be a rehearsal: `run.mode` simulation, a `simulation_ground_truth` primary
   instrument, or `measurement_source` simulation, with any other role, is refused.
4. The core the report names must be a release (`X.Y.Z`, `aN`/`bN`/`rcN`, `.postN`, as
   `tools/pin_core.py`) listed in the clone's `kits.lock`. An unreleased core (`.dev`, local) is
   refused unless `--allow-dev-core`, which is for dry runs and only under a `test-*` lab; CI refuses
   such a run in any case.
5. The clone's limits: `MAX_FILE_BYTES` and `MAX_RUN_BYTES` read (never run) from
   `tools/path_guard.py`, else POLICY.md's "No file over N MB, and no run over M MB" (MB = 10^6, the
   stricter reading), else 20 MiB and 50 MiB. No file may look like a private key or a Git LFS pointer.
6. The copy, to `results/LAB/<test>-<UTC yyyymmddThhmmssZ>-s<seed>/`, which must not exist; it is
   verified again.
7. The clone's own `tools/path_guard.py --paths-from -` (where the files go) and
   `tools/claim_lint.py` on the copy, and `tools/pin_core.py --run-dir` for a released core; their
   output is shown. If one rejects the run, the copy is removed.

Then it prints, without running them: `cd`, `git switch -c results/LAB/RUN`, `git add`,
`git commit`, `git push -u origin`, and `gh pr create --base main --head ... --title ... --body ...`
(the title and body use no claim words).

## Refusals

be report refuses: an unusable lab config, key file (a `kid` that is not its thumbprint, `x` not
`d`'s public key) or run directory (not `be-run-declared/0.1`, no `end` block, never started, no
commands); `--test` against the run; a role the run cannot have; setup files whose hash differs from
declared.json's; `--samples` without `--clock`; a clock method without its inputs (`monotonic`
without `monotonic_start_s`, `wall` without clock.csv or `--wall-clock-agreement-s`, flags that do
not go with the method); a monotonic cross-check that fails; no clock.csv pair within 10 s of the
run; samples that core's reader refuses, that overlap, or none of which fall in the run; an accuracy
file core's reader refuses; instrument file names a run directory does not allow, that hold a
scrubbed word, or that be report writes itself; a non-UTF-8 NOTES.md; a photo that is not a JPEG;
any file that looks like a private key or an LFS pointer; anything core's `build` refuses (its
findings are printed); and a signed report that `verify` does not find consistent.

## Decisions and open points

- **Setup files are not scrubbed** (above): the brief asks for every text file, but a scrubbed
  envelope is not the envelope the run used.
- **The runner change is in `run.py` as well as `session.py`**, because run.py writes declared.json.
- **Control runs declare seed 0**: the runner's control block records no seed.
- **`--samples` replaces the simulated instrument's raw/samples.csv** when both exist (the
  end-to-end test does this); the run's own file is then not in the submission.
- **The results repository's policy lists `raw/state.csv.gz`, `raw/accuracy.csv` and
  `raw/instrument/` as required**; core treats them as optional (no accuracy file gives
  ACCURACY_CHECK_MISSING), and be submit leaves that to core and the reviewer.
- **`kits.lock` is empty**, so until core 0.1.0 is released every submission is a dry run.
- **CI does not clone the results repository**, so the end-to-end test's `be submit` step (the real
  path guard in both modes, identity, claim lint) skips there; set `BE_RESULTS_REPO` to a clone to
  run it. The `be submit` refusal tests use stand-in tools and always run.
- **The scrubber errs toward replacing**: a four-part version string with parts up to 255
  (`4.10.0.84`) is taken for an IPv4 address; a file named `x.local` for a hostname. The
  identity files' warnings and the per-file counts show what happened.
