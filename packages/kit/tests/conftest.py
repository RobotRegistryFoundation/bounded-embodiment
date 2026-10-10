"""Shared fixtures for the markerless instrument (`be fit`).

The end-to-end tests render synthetic recordings of the SO-101 (in the recorder's own file format, with
known truth) and fit them. They need OpenCV, SciPy and the SO-101 meshes, which are not in git; without
them they skip and say how to fetch the meshes. They are marked ``slow`` (a few minutes together):
``-m "not slow"`` leaves them out. Each recording is rendered once per test session and shared.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest
from _markerless_support import (
    ENVELOPE,
    MOTION,
    URDF,
    need_fit_stack,
    plant_robot_state,
    recording_opens,
)


@pytest.fixture(scope="session")
def fit_model():
    """The fitter's model of the SO-101 (built once: about 2 s)."""
    need_fit_stack()
    from bounded_embodiment_kit.instruments.markerless.model import ArmModel

    return ArmModel(URDF)


@pytest.fixture(scope="session")
def full_run(tmp_path_factory, fit_model):
    """The whole 10 Oct procedure on a synthetic recording: the board on the table, background, a sweep to
    READY and 10 s of the pick-and-place at 30 fps; then `be fit register`, `be fit background` and
    `be fit run --from nominal_1`, with harness logs and probe files planted in the session and every file
    opened during the three steps logged."""
    from bounded_embodiment_kit.instruments.markerless import (
        background,
        cli,
        fit,
        register,
    )

    root = tmp_path_factory.mktemp("markerless-full") / "session"
    truth = cli.build_synthetic_session(root, URDF, nominal_s=10.0, board_s=6.0, board_fps=2.0, background_s=1.0,
                                        sweep_s=1.0, seed=1, say=lambda s: None)
    planted = plant_robot_state(root)
    with recording_opens() as log:
        reg = register.register_session(root, URDF, say=lambda s: None)
        bg = background.build_background(root, say=lambda s: None)
        meta = fit.fit_session(root, URDF, ENVELOPE, model=fit_model, t_from="nominal_1", say=lambda s: None)
    return SimpleNamespace(root=root, truth=truth, reg=reg, background=bg, meta=meta, opened=log,
                           planted=planted, score=cli.score(root / "fit" / "samples.csv", truth))


class Motion:
    """Short motion recordings (no board; the registration is the truth), rendered and fitted on demand,
    each once per test session. Every variant uses the same settings (MOTION) and seed, so two of them
    differ only by what the variant adds."""

    def __init__(self, tmp_path_factory, model):
        self.tmp, self.model = tmp_path_factory, model
        self.sessions, self.fits = {}, {}

    def session(self, name: str, **variant):
        if name not in self.sessions:
            from bounded_embodiment_kit.instruments.markerless import background, synth
            from bounded_embodiment_kit.kinematics import URDFModel

            root = self.tmp.mktemp(f"motion-{name}") / name
            truth = synth.write_motion_session(root, URDFModel.from_file(URDF), **{**MOTION, **variant})
            background.build_background(root, say=lambda s: None)
            self.sessions[name] = (root, truth)
        return self.sessions[name]

    def fit(self, name: str, *, out_name: str = "fit", registration=None, **variant):
        key = (name, out_name)
        if key not in self.fits:
            from bounded_embodiment_kit.instruments.markerless import cli, fit

            root, truth = self.session(name, **variant)
            out = root / out_name
            meta = fit.fit_session(root, URDF, ENVELOPE, registration, model=self.model, t_from="nominal_1",
                                   out=out, say=lambda s: None)
            self.fits[key] = SimpleNamespace(root=root, out=out, truth=truth, meta=meta,
                                             score=cli.score(out / "samples.csv", truth))
        return self.fits[key]


@pytest.fixture(scope="session")
def motion(tmp_path_factory, fit_model):
    return Motion(tmp_path_factory, fit_model)
