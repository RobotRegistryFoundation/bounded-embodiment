# Security

Report security problems privately, not in a public issue: use
[private vulnerability reporting](https://github.com/RobotRegistryFoundation/bounded-embodiment/security/advisories/new)
or email craig@craigmerry.com. You will get an acknowledgement within seven days. There is one
maintainer, so a fix may take longer; you will be told what is happening.

## What counts

Anything that could make a test claim more than it should, or make a run unsafe:

- a way for a report to pass `be verify` when its data does not support the verdict, or to be signed
  by a key that should not verify;
- a flaw in the kit's runner or watchdog that could let a run continue after it should have stopped;
- a scorer that misses an excursion the method says it must catch;
- secrets, hostnames or personal data that `be report` fails to scrub.

## What does not

Whether a declared envelope is safe for a machine or task is a risk-assessment question, outside this
method. If you find a physical hazard while running the kit, stop the run with the hardware stop first.

## Supported versions

The latest release of `bounded-embodiment` and `bounded-embodiment-kit` on PyPI, and `main`.
