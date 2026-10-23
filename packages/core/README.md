# bounded-embodiment (core)

The core of the reference implementation of the Bounded Embodiment test method. The method asks one
question: does a machine stay within its declared physical limits when its commands come from a
source that cannot be trusted, such as an AI model?

Core holds the loaders for envelopes, command spaces and waypoints; the command streams
`be-hostile/1`, `be-control/1` and `be-waypoints/1`; the scorers `ev03-tool-point/1` and
`ev10-waypoints/1`; the signed test report; and `be verify`, which recomputes a report's verdict
from its raw data. Pure Python: `jsonschema` and `cryptography`, nothing else.

**Status.** Method v0.1 is a draft for comment with illustrative thresholds. A verdict is a finding
about one run, never a certification. There is one maintainer and no third-party verification yet.

## Install

```sh
python -m pip install bounded-embodiment
be --help
```

Python 3.10 or later. The command is `be`, also installed as `bounded-embodiment`.

## Use

```sh
be keygen --out lab.key.json --lab my-lab --contact lab@example.org   # a lab's signing key
be envelope validate envelope.json                                    # check an envelope, print its hash
be verify SUBMISSION_DIR --keys keyring.json                          # recompute a report's verdict
```

`be verify` checks consistency: that a signed report agrees with its files, and that its verdict
follows from them. Consistent is not passed: a FAIL report is consistent when its data supports FAIL.

To run the tests on a machine you also need the lab bench, the
[`bounded-embodiment-kit`](https://pypi.org/project/bounded-embodiment-kit/) package, which adds
`be run`, `be record`, `be fit`, `be instrument`, `be report` and `be submit`.

## Links

- Homepage: https://robotregistryfoundation.org/bounded-embodiment/
- Method text (normative, CC BY 4.0): https://github.com/RobotRegistryFoundation/bounded-embodiment/blob/main/method/bounded-embodiment-method-v0.1.md
- Requirements and test plan (informative): https://github.com/RobotRegistryFoundation/bounded-embodiment/blob/main/docs/requirements.md
- Lab guide: https://github.com/RobotRegistryFoundation/bounded-embodiment/blob/main/docs/lab-guide.md
- Source and issues: https://github.com/RobotRegistryFoundation/bounded-embodiment
- Results table: https://robotregistryfoundation.org/bounded-embodiment/results/
- The talk: https://craigmerry.com/bounded-embodiment/

Code: Apache-2.0. The packaged schemas and profiles: CC BY 4.0.
