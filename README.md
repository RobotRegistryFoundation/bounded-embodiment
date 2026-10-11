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

| | |
|---|---|
| [`method/`](method/) | the normative text (CC BY 4.0) and how to cite it |
| [`packages/core`](packages/core) | `bounded-embodiment`: loaders, generators, scorers, the report and `be verify` (pure Python) |
| [`packages/kit`](packages/kit) | `bounded-embodiment-kit`: adapters, the runner and watchdog, `be record`, `be fit`, `be instrument`, `be report`, `be submit` |
| [`schemas/`](schemas/), [`profiles/`](profiles/) | the formats and the illustrative thresholds (shipped inside core) |
| [`vectors/`](vectors/) | golden streams, scoring cases and signed golden runs |
| [`examples/`](examples/) | envelopes, command spaces, the SO-101 URDF, reference gates |
| [`docs/lab-guide.md`](docs/lab-guide.md) | a lab's session, from setup to a signed submission |
| [`docs/accuracy-survey.md`](docs/accuracy-survey.md) | surveying the accuracy check's points with tape, rule and plumb line |
| [`CONTRACT.md`](CONTRACT.md), [`docs/seams/`](docs/seams/) | the interfaces between the parts |

Results go to [bounded-embodiment-results](https://github.com/RobotRegistryFoundation/bounded-embodiment-results),
whose CI checks every submission with `be verify`.

## Try it in simulation

```sh
python -m pip install -e packages/core -e packages/kit
be --help
python -m pytest -q packages/core/tests packages/kit/tests -m "not slow"
```

The kit's tests run the whole bench against simulated machines, with and without reference gates
(`examples/reference-gates`), and score the runs with core's scorers. A simulation is always a
rehearsal: its verdict is INCONCLUSIVE by rule.

## License

Code: Apache-2.0. The method text, schemas, profiles and vectors: CC BY 4.0. See `REUSE.toml` and
`LICENSES/`.
