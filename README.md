# Bounded Embodiment

A public test method, and its reference implementation, for one question: does a machine stay within
its declared physical limits when its commands come from a source that cannot be trusted, such as an
AI model?

Version 0.1 defines two tests. **EV-03** sends a seeded stream of hostile commands for 10 minutes
while an external instrument measures the machine. **EV-10** checks that the same system still
reaches waypoints the lab declares. Each run produces a signed report whose verdict (PASS, FAIL or
INCONCLUSIVE) anyone can recompute from the raw data with `be verify`.

**Status.** Method v0.1 is a draft for comment, and its thresholds are illustrative. A verdict is a
finding about one run, never a certification. The kit's adapters, runner and watchdog are a draft
that needs line-by-line human review before any hardware use.

## Take part

This method was proposed at the ITU Workshop on "Embodied AI for Multimedia Technologies" (Hangzhou,
China, 16 October 2026), held with the ITU-T Focus Group on Embodied AI. That is a presentation, not an
endorsement: neither ITU nor any other body has reviewed or adopted it. It asked for three things.

| | One action |
|---|---|
| **Feedback** on the five requirements and the tests: what's missing, what's wrong? | Read [docs/requirements.md](docs/requirements.md), then open a [Feedback on a requirement or test](https://github.com/RobotRegistryFoundation/bounded-embodiment/issues/new?template=feedback-requirement.yml) issue |
| **Collaborators** to shape it into a written contribution to FG-EAI | Start a thread in [Discussions](https://github.com/RobotRegistryFoundation/bounded-embodiment/discussions) under "Standards contribution", or email craig@craigmerry.com |
| **Testbeds** willing to run the tests on real hardware and show where it breaks | Open an [Offer a testbed](https://github.com/RobotRegistryFoundation/bounded-embodiment/issues/new?template=offer-testbed.yml) issue |

[CONTRIBUTING.md](CONTRIBUTING.md) has a path through the code that fits in 30 minutes, and the terms
for contributions that may end up in a standards contribution (DCO sign-off, licences, patents).

There is one maintainer and no third-party verification yet. The method is vendor-neutral: running a
test needs no particular protocol, vendor or model. One open implementation, RCAN (rcan.dev), is
maintained by the same person; the method does not depend on it.

| | |
|---|---|
| [`method/`](method/) | the normative text (CC BY 4.0) and how to cite it |
| [`packages/core`](packages/core) | `bounded-embodiment`: loaders, generators, scorers, the report and `be verify` (pure Python) |
| [`packages/kit`](packages/kit) | `bounded-embodiment-kit`: adapters, the runner and watchdog, `be record`, `be fit`, `be instrument`, `be report`, `be submit` |
| [`schemas/`](schemas/), [`profiles/`](profiles/) | the formats and the illustrative thresholds (shipped inside core) |
| [`vectors/`](vectors/) | golden streams, scoring cases and signed golden runs |
| [`examples/`](examples/) | envelopes, command spaces, the SO-101 URDF, reference gates |
| [`docs/requirements.md`](docs/requirements.md) | the five requirements, the full test plan, recovery after a gate decision and the assurance levels (scope draft, informative) |
| [`docs/lab-guide.md`](docs/lab-guide.md) | a lab's session, from setup to a signed submission |
| [`docs/accuracy-survey.md`](docs/accuracy-survey.md) | surveying the accuracy check's points with tape, rule and plumb line |
| [`CONTRACT.md`](CONTRACT.md), [`docs/seams/`](docs/seams/) | the interfaces between the parts |

Results go to [bounded-embodiment-results](https://github.com/RobotRegistryFoundation/bounded-embodiment-results),
whose CI checks every submission with `be verify`. The merged results are listed in a table at
<https://robotregistryfoundation.org/bounded-embodiment/results/>. The talk and where the work goes
next are at <https://craigmerry.com/bounded-embodiment/>. Changes to the packages are in
[CHANGELOG.md](CHANGELOG.md).

## Try it in simulation

```sh
python3 -m venv .venv && . .venv/bin/activate
python -m pip install -e "packages/core[test]" -e "packages/kit[fit,test]"
be --help
python -m pytest -q packages/core/tests
python -m pytest -q packages/kit/tests -m "not slow"
```

The kit's tests run the whole bench against simulated machines, with and without reference gates
(`examples/reference-gates`), and score the runs with core's scorers. A simulation is always a
rehearsal: its verdict is INCONCLUSIVE by rule.

## License

Code: Apache-2.0. The method text, the requirements page, schemas, profiles and vectors: CC BY 4.0. See `REUSE.toml` and
`LICENSES/`.
