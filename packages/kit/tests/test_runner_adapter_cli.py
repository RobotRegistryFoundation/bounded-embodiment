# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""``be adapter new``: the python adapter template implements the contract, cannot run until it
is filled in, and is never written over an existing file."""
from __future__ import annotations

import argparse

import pytest
from bounded_embodiment_kit import cli
from bounded_embodiment_kit.adapters.base import (
    AdapterContext,
    AdapterError,
    check_adapter,
    load_callable,
)
from bounded_embodiment_kit.adapters.python import write_template


def test_the_template_implements_the_contract_and_refuses_to_open_unfilled(tmp_path):
    path = write_template("labarm", tmp_path)
    cls = load_callable(f"{path}:Adapter")
    adapter = cls(AdapterContext(command_space_path=tmp_path / "missing.json"))
    check_adapter(adapter)
    assert adapter.describe()["config"] == {"adapter": "labarm"}
    with pytest.raises((NotImplementedError, ValueError)):
        adapter.open()
    with pytest.raises(AdapterError, match="already exists"):
        write_template("labarm", tmp_path)
    with pytest.raises(AdapterError):
        write_template("not a name", tmp_path)


def test_the_templates_trial_methods_refuse_while_latched_before_and_under_the_lock(tmp_path):
    """Review S7: the runner looks at its latch just before each call, but the watchdog can trip
    it in between, so an adapter written from the template must refuse on its own, before taking
    the bus lock and again under it."""
    cls = load_callable(f"{write_template('labarm', tmp_path)}:Adapter")
    adapter = cls(AdapterContext(command_space_path=tmp_path / "missing.json"))
    for name in ("trigger_stop", "reset_stop"):
        with pytest.raises(NotImplementedError):  # not latched: the stub says it is not written
            getattr(adapter, name)("stop_message")
    adapter.latch_stop()
    for name in ("trigger_stop", "reset_stop"):
        with pytest.raises(RuntimeError, match="latched"):
            getattr(adapter, name)("stop_message")

    class TripsUnderTheLock:  # the latch trips while the method waits for the bus
        def __init__(self):
            self.inner = adapter._bus_lock

        def __enter__(self):
            adapter._latched.set()
            return self.inner.__enter__()

        def __exit__(self, *exc):
            return self.inner.__exit__(*exc)

    for name in ("trigger_stop", "reset_stop"):
        adapter._latched.clear()
        adapter._bus_lock = TripsUnderTheLock()
        with pytest.raises(RuntimeError, match="latched"):
            getattr(adapter, name)("stop_message")
        adapter._bus_lock = adapter._bus_lock.inner


def test_be_adapter_new_and_list(tmp_path, capsys):
    p = argparse.ArgumentParser(prog="be")
    cli.register(p.add_subparsers(dest="command"))
    args = p.parse_args(["adapter", "new", "rig", "--out", str(tmp_path)])
    assert args.func(args) == 0 and (tmp_path / "rig_adapter.py").exists()
    args = p.parse_args(["adapter", "list"])
    assert args.func(args) == 0
    assert {"lerobot", "python", "sim-so101", "sim-rover"} <= set(capsys.readouterr().out.split())


def test_check_adapter_names_what_is_missing():
    class Half:
        def open(self):
            pass

    with pytest.raises(AdapterError, match="execute_stop"):
        check_adapter(Half())
