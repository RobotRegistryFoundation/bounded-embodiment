# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""`be report` on EV-02 and EV-05 runs (method v0.2): the plan file goes in byte for byte and must
be the one the run used, and the lab config must cite method version 0.2."""
from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import _submission_support as sup
from bounded_embodiment import report as core_report


@pytest.fixture(autouse=True)
def _fixed_identity(monkeypatch):
    from bounded_embodiment_kit.submission.scrub import Identity

    monkeypatch.setattr(Identity, "this_machine", classmethod(
        lambda cls, redact=(): Identity(("benchhost",), ("ana",), ("/home/ana",), tuple(redact))))


PLAN = {"ev02-pass": "ev02-stop-plan.json", "ev05-pass": "ev05-model-loss-plan.json"}


def _run(tmp_path: Path, name: str) -> tuple[Path, Path]:
    rep = sup.golden_report(name)
    plan_bytes = (sup.RUNS / name / PLAN[name]).read_bytes()
    loader = core_report.load_stop_plan if name.startswith("ev02") else core_report.load_model_loss_plan
    gen = rep["generator"]
    generator = {"name": gen["name"], "version": gen["version"], "seed": gen["seed"],
                 "params": gen["params"], "command_space_digest": rep["adapter"]["command_space_sha256"],
                 "plan_digest": loader(plan_bytes).digest}
    run = sup.runner_run(tmp_path, name, generator=generator)
    shutil.copyfile(sup.RUNS / name / PLAN[name], run / PLAN[name])
    key, _ = sup.make_key(tmp_path / "lab.key.json")
    return run, key


@pytest.mark.parametrize("name", sorted(PLAN))
def test_an_ev02_or_ev05_run_becomes_a_0_2_report(tmp_path, name):
    run, key = _run(tmp_path, name)
    args = sup.report_args(tmp_path, run, key, lab=sup.lab_config(name))
    assert args.func(args) == 0
    out = tmp_path / "submission"
    report = json.loads((out / "report.json").read_text())
    assert report["report_version"] == "0.2" and report["method"]["version"] == "0.2"
    assert (out / PLAN[name]).read_bytes() == (sup.RUNS / name / PLAN[name]).read_bytes()
    assert report["verdict"]["result"] == "PASS"
    assert any(e["path"] == PLAN[name] and e["role"] in ("stop_plan", "model_loss_plan")
               for e in report["raw_data"])


def test_the_plan_must_be_the_one_the_run_used(tmp_path):
    run, key = _run(tmp_path, "ev02-pass")
    doc = json.loads((run / "ev02-stop-plan.json").read_text())
    doc["description"] = "edited after the run"
    (run / "ev02-stop-plan.json").write_text(json.dumps(doc))
    args = sup.report_args(tmp_path, run, key, lab=sup.lab_config("ev02-pass"))
    assert args.func(args) == 1


def test_ev02_needs_a_lab_config_that_cites_method_0_2(tmp_path):
    run, key = _run(tmp_path, "ev02-pass")
    lab = sup.lab_config("ev02-pass")
    lab["method"] = dict(lab["method"], version="0.1")
    args = sup.report_args(tmp_path, run, key, lab=lab)
    assert args.func(args) == 1
