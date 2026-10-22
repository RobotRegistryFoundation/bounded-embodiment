# Bounded Embodiment test method

This directory holds the normative text of the Bounded Embodiment test method. This README is informative.

## What it is

The Bounded Embodiment test method is a public test method for one question: does a machine stay within its declared physical limits when its commands come from a source that cannot be trusted, such as an AI model? Version 0.1 defines two tests: EV-03 sends a seeded stream of hostile commands for 10 minutes while an external instrument measures the machine, and EV-10 checks that the same system still reaches waypoints the lab declares. The integrator declares the machine's envelope and the lab declares its command interface and instrument setup, while the method fixes the command streams, the scoring rules and the thresholds, which are illustrative in version 0.1. Each run produces a signed report whose verdict (PASS, FAIL or INCONCLUSIVE) anyone can recompute from the raw data with `be verify`, which checks consistency, not the truth of the data. A verdict is a finding about one run, never a certification, and only a different lab can reproduce it.

Version 0.2, a draft for comment, adds two tests and changes nothing about EV-03 and EV-10: EV-02 measures how the machine stops when a stop is issued through each stop source the lab tests (its own stop message and a stop path outside its software), and EV-05 what it does when its model dies or its connection is lost while it moves, including whether it stays still when commands return. Its thresholds are illustrative too.

## Files

| File | Status |
|---|---|
| [`bounded-embodiment-method-v0.1.md`](bounded-embodiment-method-v0.1.md) | version 0.1, draft for comment; normative; EV-03 and EV-10. Its SHA-256 is pinned by a test, and version 0.2 carries its sections 7 and 8 word for word |
| [`bounded-embodiment-method-v0.2.md`](bounded-embodiment-method-v0.2.md) | version 0.2, draft for comment; normative; EV-02, EV-03, EV-05 and EV-10 |

## How to cite

> Bounded Embodiment test method, version 0.1 (draft for comment), 10 October 2026. Method id `bounded-embodiment`. SHA-256 of the text: `sha256:ff5b502e542bf14456a2a23c4c1b3fe9f68ccada0162d1052857bab53254c4a5`.

> Bounded Embodiment test method, version 0.2 (draft for comment), 22 October 2026. Method id `bounded-embodiment`. SHA-256 of the text: `sha256:4d95f94c784dd06c25a852ec9fb82a44b686212dd20b43d1ce990bfb41e1b9f2`.

A test report cites the version it followed in its `method` member, for example:

```json
{"id": "bounded-embodiment", "version": "0.1", "text_sha256": "sha256:ff5b502e542bf14456a2a23c4c1b3fe9f68ccada0162d1052857bab53254c4a5"}
```

An EV-02 or EV-05 report cites version 0.2, the first that defines them.

The SHA-256 of a draft is a placeholder: it is what `tools/method_hash.py` printed when this README was last updated, and every edit to the text changes it (`packages/core/tests/test_method_text.py` checks that this README is current). Before citing the text, or before any run under it, run the script and use its output. The hash covers the file's bytes exactly as stored (UTF-8, LF line endings), and `.gitattributes` stops Git from converting line endings under `method/`.

```sh
python tools/method_hash.py                    # prints version 0.1's SHA-256
python tools/method_hash.py --text method/bounded-embodiment-method-v0.2.md   # version 0.2's
python tools/method_hash.py --check RUN_DIR    # does RUN_DIR/report.json cite version 0.1? (add --text for 0.2)
```

The text is frozen, and its hash published, before any physical run is made under it.

## The reference implementation

| Part | Where | Package name |
|---|---|---|
| Core: loaders, the generators `be-hostile/1`, `be-control/1` and `be-waypoints/1` (and, for version 0.2, `be-stops/1` and `be-model-loss/1`), the scorers `ev03-tool-point/1` and `ev10-waypoints/1` (and `ev02-stop/1`, `ev05-model-loss/1`), the report builder and `be verify` | `packages/core` | `bounded-embodiment` |
| Kit: adapters, runner and watchdog (a draft that needs line-by-line human review before any hardware use), instruments | `packages/kit` | `bounded-embodiment-kit` |
| Schemas and profiles | `schemas/`, `profiles/` | shipped inside core |
| Vectors: golden streams, scoring goldens and golden runs | `vectors/` | |

The repository is <https://github.com/RobotRegistryFoundation/bounded-embodiment>. Annex A of the text maps each rule to the code and test that implement it, and the text's closing section lists what the reference implementation does not do yet.

## Proposing changes

- Open an issue or a pull request in the repository. Name the section and the rule, and say what should change and why.
- Until version 0.1 is frozen, edits change the text and its SHA-256. Once it is frozen, the version 0.1 text never changes, and a correction goes into a new version.
- A change to a generator or scorer rule gets a new rule name, for example `ev03-tool-point/2`. Core keeps the old one, so reports made under it can still be checked.
- A disagreement between the text and the reference implementation is a defect. Report it the same way.

## Licence

The method text and this README are licensed under CC BY 4.0 (see `REUSE.toml` and `LICENSES/CC-BY-4.0.txt`). The code that implements the method is licensed under Apache-2.0.
