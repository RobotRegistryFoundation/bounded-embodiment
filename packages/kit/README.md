# bounded-embodiment-kit

The lab bench for the Bounded Embodiment tests: adapters (LeRobot SO-10x, a Python template,
simulators), the runner and its watchdog (`be run`, `be goto-start`, `be doctor`), the OAK-D recorder
(`be record`), the markerless tracker (`be fit`), the accuracy check (`be instrument`), and packaging
(`be report`, `be submit`). It adds these commands to core's `be`.

**The adapters, runner and watchdog are a draft that needs line-by-line human review before they
drive hardware.** The markerless tracker is validated on synthetic recordings only. Method v0.1 is a
draft for comment; a verdict is never a certification.

Extras: `fit` (the tracker: OpenCV, SciPy), `oakd` (DepthAI), `camera` (OpenCV), `lerobot`.

Start with the lab guide: https://github.com/RobotRegistryFoundation/bounded-embodiment/blob/main/docs/lab-guide.md
