# bounded-embodiment-kit

The lab bench for the Bounded Embodiment tests. It adds these commands to core's `be`:

| Command | What it does |
|---|---|
| `be doctor` | read-only check of this computer, the arm and the instrument feed |
| `be goto-start`, `be run` | the runner and its watchdog: move to the start pose; run EV-03 or EV-10 through an adapter |
| `be adapter` | list the adapters (LeRobot SO-10x, simulators), or write a Python adapter template |
| `be record` | record a Luxonis OAK-D (and a wrist camera) for the markerless instrument |
| `be fit` | the markerless tracker: register the camera, fit the robot's URDF to depth |
| `be instrument` | plan the accuracy check's poses; compute the accuracy check, jitter record and post-run check |
| `be report`, `be submit` | package a run into a signed submission; check it and copy it into the results repository |

**Status.** The adapters, runner and watchdog are a draft that needs line-by-line human review before
they drive hardware. The markerless tracker is validated on synthetic recordings only. Method v0.1 is
a draft for comment with illustrative thresholds; a verdict is never a certification.

## Install

```sh
python -m pip install "bounded-embodiment-kit[fit,oakd,lerobot]"
```

| Extra | Adds |
|---|---|
| `fit` | the markerless tracker (OpenCV, SciPy) |
| `oakd` | the OAK-D recorder (DepthAI v2, OpenCV) |
| `camera` | OpenCV only, for a wrist camera |
| `lerobot` | the SO-10x adapter (LeRobot) |

The kit needs Python 3.12 or later and is tested on 3.12 and 3.13. A lab session also needs the example envelopes, the
SO-101 URDF and the tools in the repository, so start from a clone and the lab guide.

## Links

- Lab guide, start here: https://github.com/RobotRegistryFoundation/bounded-embodiment/blob/main/docs/lab-guide.md
- Homepage: https://robotregistryfoundation.org/bounded-embodiment/
- Method text (normative, CC BY 4.0): https://github.com/RobotRegistryFoundation/bounded-embodiment/blob/main/method/bounded-embodiment-method-v0.1.md
- Requirements and test plan (informative): https://github.com/RobotRegistryFoundation/bounded-embodiment/blob/main/docs/requirements.md
- Source and issues: https://github.com/RobotRegistryFoundation/bounded-embodiment
- Results repository: https://github.com/RobotRegistryFoundation/bounded-embodiment-results

Apache-2.0.
