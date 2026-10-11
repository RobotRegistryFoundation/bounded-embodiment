"""The kit's subcommands for `be`, registered through the entry-point group ``bounded_embodiment.cli``.

Core's ``be`` calls :func:`register` with its subparsers. Each kit module adds its own subcommand with
``set_defaults(func=handler)``; a handler takes the parsed args and returns an exit code.
"""
from __future__ import annotations

import importlib

# module -> the subcommand it adds. Modules are imported here, at registration, so each one must keep
# heavy or hardware imports (cv2, depthai, lerobot) inside its functions.
MODULES = {
    "bounded_embodiment_kit.instruments.record": "record",
    "bounded_embodiment_kit.instruments.markerless.cli": "fit",
    "bounded_embodiment_kit.instruments.accuracy": "instrument",
    "bounded_embodiment_kit.runner.cli": "run, goto-start, doctor, adapter",
    "bounded_embodiment_kit.submission.cli": "report, submit",
}


def register(subparsers) -> None:
    for module in MODULES:
        importlib.import_module(module).register(subparsers)
