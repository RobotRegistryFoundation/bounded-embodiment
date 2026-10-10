# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""The markerless model tracker (instrument kind ``markerless_model_tracker``).

It measures where an arm's points are by fitting the arm's own URDF meshes to depth from a fixed camera.
Nothing is attached to the arm, and nothing is read from the robot: the fitter takes a recording, the
URDF, the envelope and the camera registration, and nothing else.

Modules: :mod:`.mesh` (STL, surface samples), :mod:`.model` (per-link lookup fields, bounds, ROI),
:mod:`.register` (camera -> base_link from a ChArUco board and tape-measured corners), :mod:`.background`
(static depth background), :mod:`.fit` (the per-frame articulated fit and the tracker), :mod:`.synth`
(a synthetic renderer that writes sessions in the recorder's own format), :mod:`.cli` (``be fit``).

It needs numpy, SciPy and OpenCV (the kit's ``fit`` extra) and the URDF's meshes. Heavy imports stay
inside functions so that ``be`` starts fast. See docs/seams/markerless.md.
"""
