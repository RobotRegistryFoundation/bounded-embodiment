# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""Adapters: where the kit hands commands to the system under test (``base`` has the contract).

Each adapter module has ``add_arguments(parser)`` for its own flags and ``from_args(args,
context)`` to build it. They are imported by name, so LeRobot is never imported unless asked for.
"""
from __future__ import annotations

import importlib

from .base import AdapterContext, AdapterError, SendResult, State, check_adapter

__all__ = ["ADAPTERS", "AdapterContext", "AdapterError", "SendResult", "State", "adapter_module",
           "check_adapter"]

ADAPTERS = {
    "lerobot": "bounded_embodiment_kit.adapters.lerobot",
    "python": "bounded_embodiment_kit.adapters.python",
    "sim-so101": "bounded_embodiment_kit.adapters.sim_so101",
    "sim-rover": "bounded_embodiment_kit.adapters.sim_rover",
}


def adapter_module(name: str):
    try:
        return importlib.import_module(ADAPTERS[name])
    except KeyError:
        raise AdapterError(f"unknown adapter {name!r}; known: {', '.join(ADAPTERS)}") from None
