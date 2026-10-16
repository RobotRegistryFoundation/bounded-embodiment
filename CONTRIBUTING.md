# Contributing

Thank you for looking. This method is a draft for comment, and comment is the contribution it needs
most. There are three ways in; each starts with one action.

| You want to | Do this |
|---|---|
| Say a requirement or test is missing, wrong or untestable | Open a [Feedback on a requirement or test](https://github.com/RobotRegistryFoundation/bounded-embodiment/issues/new?template=feedback-requirement.yml) issue |
| Help shape this into a written contribution to a standards body | Start a thread in [Discussions](https://github.com/RobotRegistryFoundation/bounded-embodiment/discussions) under "Standards contribution", or email craig@craigmerry.com |
| Run the tests on your hardware | Open an [Offer a testbed](https://github.com/RobotRegistryFoundation/bounded-embodiment/issues/new?template=offer-testbed.yml) issue |
| Ask about a result you ran or plan to submit | Open a [Submit results](https://github.com/RobotRegistryFoundation/bounded-embodiment/issues/new?template=submit-results.yml) issue. Signed results themselves go to [bounded-embodiment-results](https://github.com/RobotRegistryFoundation/bounded-embodiment-results) by pull request |

You do not need to install anything to comment. The requirements are in
[docs/requirements.md](docs/requirements.md) and the normative text is in
[method/](method/).

## Your first 30 minutes with the code

This path ends with the tests passing on your machine and, if you like, a draft pull request.

1. **Clone and make a virtual environment** (Python 3.10 or later):
   ```sh
   git clone https://github.com/RobotRegistryFoundation/bounded-embodiment
   cd bounded-embodiment
   python3 -m venv .venv && . .venv/bin/activate
   ```
2. **Install both packages with their test extras.** `fit` brings OpenCV, which some kit tests need;
   without it they skip.
   ```sh
   python -m pip install -e "packages/core[test]" -e "packages/kit[fit,test]"
   be --help
   ```
3. **Run the fast tests**, one package at a time, as CI does. Each package has its own pytest
   settings, so run them separately:
   ```sh
   python -m pytest -q packages/core/tests
   python -m pytest -q packages/kit/tests -m "not slow"
   ```
   The kit suite runs the whole bench against simulated machines and takes a few minutes. A
   simulated run is always a rehearsal: its verdict is INCONCLUSIVE by rule. A few kit tests need
   the SO-101 arm's meshes and skip without them; CI fetches them with
   `python tools/fetch_robot_assets.py examples/robots/so101`.
4. **Make a change on a branch and commit with sign-off** (see below):
   ```sh
   git switch -c my-change
   git commit -s -m "Short description of the change"
   ```
5. **Open a draft pull request** from your fork. Say what you changed and why, and which section of
   the method or which test it touches.

The kit's adapters, runner and watchdog are a draft. Do not point them at real hardware without
reading [docs/lab-guide.md](docs/lab-guide.md) first and keeping a hardware stop within reach.

## How decisions are made

There is one maintainer, Craig Merry, and no third-party verification of anything here yet. The
maintainer decides what is merged and records the reason on the issue or pull request. Changes to
the normative text under `method/` make a new method version (method §11); a frozen version's bytes
never change.

If the work moves to a neutral steward or into a standards body, that body's process replaces this
one, and this file will say so.

## Sign-off (DCO)

Every commit must carry a `Signed-off-by:` line, added by `git commit -s`. It certifies the
[Developer Certificate of Origin 1.1](https://developercertificate.org/): that you wrote the change,
or otherwise have the right to submit it under this repository's licences. Pull requests without
sign-off on every commit are not merged.

## Licences

Your contribution is licensed the same way as the files it changes, as recorded per path in
[REUSE.toml](REUSE.toml):

| What | Licence |
|---|---|
| Method text, requirements and scope docs, schemas, profiles, test vectors | CC BY 4.0 |
| Code, tools, tests and other documentation | Apache-2.0 |
| Results and lab registrations (in bounded-embodiment-results) | CC BY 4.0 |

## Standards bodies and intellectual property

This work is intended to feed written contributions to standards bodies, starting with the ITU-T
Focus Group on Embodied AI (FG-EAI). By contributing you agree that text you contribute under
CC BY 4.0 may be included, with attribution, in such contributions, and that the receiving body may
then reproduce and adapt it under its own rules. ITU-T contributions fall under the
[ITU-T/ITU-R/ISO/IEC Common Patent Policy](https://www.itu.int/en/ITU-T/ipr/Pages/policy.aspx).

If you know of a patent or patent application, yours or anyone's, that a contribution might need,
say so on the issue or pull request before it is merged. Nothing in this repository is an ITU
document, and nothing here has been reviewed or endorsed by ITU or any other body.

## Conduct and security

Everyone taking part follows the [Code of Conduct](CODE_OF_CONDUCT.md). Report security problems
privately as described in [SECURITY.md](SECURITY.md), not in a public issue.
