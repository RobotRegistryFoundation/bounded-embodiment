# Changelog

Changes to the two packages, `bounded-embodiment` (core) and `bounded-embodiment-kit` (the kit).
The method text has its own version and SHA-256 (see `method/README.md`); nothing here changes it.
Generators, scorers and profiles are versioned by name (`be-hostile/1`, `ev03-tool-point/1`,
`ev03-illustrative-0.1`), and no release below changes any of them.

## Unreleased (method v0.2 draft: EV-02 and EV-05)

Nothing here changes any released generator, scorer, profile or format: EV-03 and EV-10, their
streams, scorers, profiles, test report 0.1 and every existing vector are byte for byte as they
were. The method text v0.2 is a new file (`method/README.md`); v0.1 is unchanged.

Core:

- New formats: stop plan 0.1 and model-loss plan 0.1 (`ev02-stop-plan.json`,
  `ev05-model-loss-plan.json` in a run directory), and test report 0.2 (test report 0.1 plus
  EV-02 and EV-05; an EV-02 or EV-05 report is always 0.2). `be verify` reads 0.1 and 0.2.
- New streams `be-stops/1` and `be-model-loss/1`, new scorers `ev02-stop/1` and
  `ev05-model-loss/1`, new profiles `ev02-illustrative-0.1` and `ev05-illustrative-0.1`
  (illustrative thresholds).
- Golden runs (PASS, FAIL and INCONCLUSIVE of each, three tamper cases) and scoring goldens.
- After review: `be-stops/1` keeps commanding the drive through each stop (`after_s`,
  `continue:<id>` rows) and refuses a walk that lands before the stop; `be-model-loss/1` refuses a
  `motion_s` narrower than one cycle of the path; a model-loss plan that resumes on commands at
  once in a position space is refused (`PLAN_RESUME_UNCHECKED`). The scorers time the rest with a
  noise-scaled rest radius, judge the approach at its end too (the late chord), hold until the next
  trial's commands (EV-02) or the restart (EV-05), judge a `commands` resume rule only where it
  applies, leave a trial whose stop was not issued unmeasured, check from `commands.csv` that the
  commands could show a stop that does not act, count drives by command, and make a run with an
  unmeasured trial INCONCLUSIVE (`STOP_TRIAL_UNMEASURED`, `LOSS_TRIAL_UNMEASURED`). Test report
  0.2 pins an EV-02/EV-05 report's method version and refuses the `control` role.

Kit (draft; line-by-line human review before any hardware use):

- `be run ev02` and `be run ev05`: one trial at a time, the operator's NEXT between trials, a
  new watchdog tripwire for a stop that does not stop or let go; every EV-03 interlock applies.
- Adapters: optional `trigger_stop` and `reset_stop` (the system's own stop request, and resuming
  after a stop or a lost model), which refuse while the runner's stop is latched, under the bus
  lock too. `sim-rover` and `sim-so101` simulate an external stop (`--sim-estop-file`).
- After review: `--hardwired-stop-cmd` and `--power-loss-cmd` issue the external sources, one
  command each, started off the command thread in a session of their own with every stream closed,
  so a command that hangs never holds up the runner's own stop (`--estop-cmd` is the escalation
  only); the trial watch is armed before a stop is issued and stays armed through the reset; the
  start check after a reset uses only what came after it; every command of the stream is checked
  against the zones and the speed limit before RUN; a hardware trial run takes NEXT from a terminal
  only (`interlocks.typed_word`); a velocity-space trial needs room for its travel.
- `be report` packages EV-02 and EV-05 runs.

## bounded-embodiment 0.1.1 and bounded-embodiment-kit 0.1.2 (2026-10-16)

Core and kit:

- Package metadata: Python version classifiers (core: 3.10 to 3.13; kit: 3.12 and 3.13, the
  versions CI tests), project links (method text, requirements, lab guide, issues, changelog,
  results table), keywords.
- Kit: requires Python 3.12 or later (was 3.10), matching what CI tests and what LeRobot needs.
- The wheels carry their licence texts. Core's licence is now declared as
  `Apache-2.0 AND CC-BY-4.0`, because it ships the schemas and profiles, which are CC BY 4.0.
- The PyPI pages (the package READMEs) say how to install, what each command does, and where the
  documentation is.

Core:

- `be --help` describes the whole command set and links the documentation; `be verify` and the
  `envelope` options have plain help text.
- `be keygen` explains a bad `--lab`, `--contact` or `--added` once, in words, instead of printing
  the schema's pattern.
- A kit command (`be run`, `be record`, ...) without the kit installed says how to install it,
  instead of "invalid choice".

Kit:

- Help text for `be doctor`, `be adapter new`, `be record mark`, `be record board`, `be fit`,
  `be instrument check` and `be instrument plan` options that had none.
- `be record` without DepthAI names the `oakd` extra and `--no-oak`.

## bounded-embodiment-kit 0.1.1 (2026-10-12)

- The `oakd` extra pins DepthAI v2 (`depthai>=2.24,<3`). The recorder is written for the v2 API
  and refuses v3, but 0.1.0's extra let pip install DepthAI 3.x.

## bounded-embodiment 0.1.0 and bounded-embodiment-kit 0.1.0 (2026-10-12)

- First release, for method v0.1 (draft for comment): core's loaders, the streams `be-hostile/1`,
  `be-control/1` and `be-waypoints/1`, the scorers `ev03-tool-point/1` and `ev10-waypoints/1`, the
  test report 0.1 and `be verify`; the kit's adapters, runner and watchdog (a draft that needs
  line-by-line human review before any hardware use), `be record`, `be fit`, `be instrument`,
  `be report` and `be submit`.
