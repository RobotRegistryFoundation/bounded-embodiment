# Changelog

Changes to the two packages, `bounded-embodiment` (core) and `bounded-embodiment-kit` (the kit).
The method text has its own version and SHA-256 (see `method/README.md`); nothing here changes it.
Generators, scorers and profiles are versioned by name (`be-hostile/1`, `ev03-tool-point/1`,
`ev03-illustrative-0.1`), and no release below changes any of them.

## Unreleased

Core:

- `be envelope lint FILE [--json] [--physical-max-speed MPS]` (new, `bounded_embodiment.envelope.lint`):
  reports envelope limits that the schema and the loader accept but that are unsafe, contradictory
  or vacuous, each with a stable `LINT_*` code: keep_out zones that cover the keep_in region or
  are too thin to enforce, stop distances longer than the region is deep, proximity rules that
  allow more speed closer in, speed limits no run can test, and more. Exit status 1 on a lint
  error. It is advice: no verdict depends on it and `be verify` does not run it.
- No rule set, profile, format or reason code changes. The loophole search that motivated the lint
  is in `docs/loopholes.md`, with new vectors in `vectors/envelope/lint/` (made by
  `tools/envelope_loopholes.py`) and `vectors/loopholes/runs/` (made by
  `tools/make_loophole_runs.py`). The defects it found are for the next rule-set versions.

Core and kit:

- Package metadata: the Homepage link is https://robotregistryfoundation.org/bounded-embodiment/,
  the project's one homepage (it was https://craigmerry.com/bounded-embodiment/, the talk). Both
  package READMEs link the homepage, and core's README names the craigmerry.com page as the talk.

Kit:

- `bounded_embodiment_kit.falsify` (`python -m bounded_embodiment_kit.falsify`), a draft
  falsification search, rule set `be-falsify/1` (draft for method v0.2): it drives a gate in
  lockstep with the simulated rover or SO-101 and steers its commands toward a position or speed
  violation, writing a replayable JSON trace. Simulation only. A violation it finds is evidence of
  a defect; finding none establishes only that, never a PASS. Results on the reference gates are in
  `docs/falsification-findings.md`. No released rule set, scorer, golden vector or verdict changes.

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
