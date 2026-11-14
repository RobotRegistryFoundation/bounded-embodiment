"""envelope.lint and `be envelope lint`: the published lint vectors, the seeded loophole search
(tools/envelope_loopholes.py), the region search and the command line."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest

from bounded_embodiment import cli
from bounded_embodiment.envelope import EnvelopeError, load
from bounded_embodiment.envelope.lint import LINT_CODES, lint, region_depth, zone_depth

REPO = Path(__file__).resolve().parents[3]
SCRIPT = REPO / "tools" / "envelope_loopholes.py"
VECTORS = REPO / "vectors" / "envelope" / "lint"
CASES = sorted(p for p in VECTORS.glob("*.json") if not p.name.endswith(".expect.json")) if VECTORS.is_dir() else []

in_repo = pytest.mark.skipif(not SCRIPT.is_file() or not VECTORS.is_dir(), reason="not in a repository checkout")


def loopholes():
    name = "envelope_loopholes"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    previous, sys.dont_write_bytecode = sys.dont_write_bytecode, True
    try:
        sys.modules[name] = module
        spec.loader.exec_module(module)
    finally:
        sys.dont_write_bytecode = previous
    return module


def box_envelope(zones: list[dict], **extra) -> dict:
    doc = {
        "envelope_version": "0.2",
        "machine": {"id": "lint-test", "class": "arm"},
        "frame": "base_link",
        "points": [{"id": "tool", "description": "test tool point"}],
        "workspace": {"zones": zones},
        "motion": {"max_speed_mps": 0.25},
        "stop": {"category": 1, "max_time_ms": 300, "max_distance_m": 0.02},
        "heartbeat": {"model_timeout_ms": 500, "gate_timeout_ms": 200, "on_loss": "stop"},
    }
    doc.update(extra)
    return doc


def box(zid: str, kind: str, lo: list, hi: list, applies=("tool",)) -> dict:
    return {"id": zid, "kind": kind, "applies_to": list(applies), "box": {"min_m": lo, "max_m": hi}}


UNIT = box("cube", "keep_in", [0.0, 0.0, 0.0], [1.0, 1.0, 1.0])


def codes(doc: dict, **kw) -> list[str]:
    return sorted(f.code for f in lint(load(doc), **kw))


# --------------------------------------------------------------------------- published vectors


@in_repo
def test_lint_vectors_cover_every_class() -> None:
    assert len(CASES) >= 40
    classes = {json.loads(p.with_name(p.name[:-5] + ".expect.json").read_text("utf-8"))["class"] for p in CASES}
    assert classes == set(loopholes().CLASSES)


@in_repo
@pytest.mark.parametrize("path", CASES, ids=lambda p: p.stem)
def test_lint_vector(path: Path) -> None:
    expect = json.loads(path.with_name(path.name[:-5] + ".expect.json").read_text("utf-8"))
    if expect["load_error_codes"] is not None:
        with pytest.raises(EnvelopeError) as e:
            load(path)
        assert sorted(f.code for f in e.value.findings if f.severity == "error") == expect["load_error_codes"]
        return
    found = lint(load(path), **expect["options"])
    assert sorted(f.code for f in found) == expect["lint_codes"]
    if expect["class"] == "clean":
        assert not [f for f in found if f.severity in ("error", "warn")]


@in_repo
def test_lint_vectors_regenerate_byte_identically() -> None:
    want = loopholes().vector_files()
    have = {p.name: p.read_bytes() for p in VECTORS.glob("*.json")}
    assert want == have


@in_repo
@pytest.mark.parametrize("seed", [0, 1, 2])
def test_seeded_search_finds_every_class(seed: int) -> None:
    lh = loopholes()
    cases, problems = lh.search(seed, len(lh.ALL) * 2)
    assert problems == []
    assert {c["class"] for c in cases} == set(lh.CLASSES)


@in_repo
def test_the_search_is_deterministic() -> None:
    lh = loopholes()
    a = [json.dumps(lh.make_case(5, k)["doc"]) for k in range(len(lh.ALL))]
    b = [json.dumps(lh.make_case(5, k)["doc"]) for k in range(len(lh.ALL))]
    assert a == b


# --------------------------------------------------------------------------- the rules


def test_codes_have_known_severities() -> None:
    for code, (severity, summary) in LINT_CODES.items():
        assert code.startswith("LINT_") and code.upper() == code
        assert severity in ("error", "warn", "info") and summary


PUBLISHED = [
    p
    for p in sorted((REPO / "vectors" / "envelope" / "valid").glob("*.json"))
    + sorted((REPO / "examples" / "envelopes").glob("*.envelope.json"))
    if not p.name.endswith(".expect.json")
]


@pytest.mark.parametrize("path", PUBLISHED, ids=lambda p: p.stem)
def test_published_envelopes_have_no_lint_error(path: Path) -> None:
    assert not [f for f in lint(load(path)) if f.severity == "error"]


def test_region_depth_of_a_cube_and_a_hole() -> None:
    env = load(box_envelope([UNIT]))
    d = region_depth(env, "tool", (0.4999, 0.5001))
    assert d.exact and d.best <= 0.5 + 1e-12 and d.bound >= 0.5 - 1e-12 and d.best >= 0.4999
    env = load(box_envelope([UNIT, box("all", "keep_out", [-0.1] * 3, [1.1] * 3)]))
    d = region_depth(env, "tool", (0.01,))
    assert d.best < 0.0 and d.bound < 0.01


def test_exact_cover_is_reported() -> None:
    doc = box_envelope([UNIT, box("same", "keep_out", [0.0] * 3, [1.0] * 3)])
    assert "LINT_KEEP_OUT_COVERS_KEEP_IN" in codes(doc)


def test_two_keep_outs_that_cover_only_together() -> None:
    doc = box_envelope([
        UNIT,
        box("left", "keep_out", [-0.1, -0.1, -0.1], [0.55, 1.1, 1.1]),
        box("right", "keep_out", [0.5, -0.1, -0.1], [1.1, 1.1, 1.1]),
    ])
    assert "LINT_KEEP_OUT_COVERS_KEEP_IN" in codes(doc)
    doc["workspace"]["zones"][1]["box"]["max_m"][0] = 0.4  # now a 0.1 m slot is free
    assert "LINT_KEEP_OUT_COVERS_KEEP_IN" not in codes(doc)


def test_keep_out_thickness_edge() -> None:
    wall = lambda t: box("wall", "keep_out", [0.5 - t / 2, -1.0, -1.0], [0.5 + t / 2, 2.0, 2.0])  # noqa: E731
    assert "LINT_KEEP_OUT_UNENFORCEABLE" in codes(box_envelope([UNIT, wall(0.02)]))
    assert "LINT_KEEP_OUT_UNENFORCEABLE" not in codes(box_envelope([UNIT, wall(0.022)]))
    assert zone_depth(load(box_envelope([UNIT, wall(0.02)])).zones[1], (0.0101,)).bound == pytest.approx(0.01)


def test_stop_distance_against_depth() -> None:
    thin = box("slab", "keep_in", [0.0, 0.0, 0.0], [1.0, 1.0, 0.03])
    assert "LINT_STOP_DISTANCE_EXCEEDS_DEPTH" in codes(box_envelope([thin]))  # 0.02 > 0.015
    doc = box_envelope([thin])
    doc["stop"]["max_distance_m"] = 0.01
    assert "LINT_STOP_DISTANCE_EXCEEDS_DEPTH" not in codes(doc)


def test_proximity_order() -> None:
    def rules(*pairs):
        return [{"when": {"human_within_m": d}, **({"action": "stop"} if v is None else {"max_speed_mps": v})}
                for d, v in pairs]

    assert "LINT_PROXIMITY_CLOSER_FASTER" in codes(box_envelope([UNIT], proximity=rules((2.0, 0.1), (0.5, 0.2))))
    assert "LINT_PROXIMITY_CLOSER_FASTER" in codes(box_envelope([UNIT], proximity=rules((2.0, None), (0.5, 0.05))))
    assert "LINT_PROXIMITY_CLOSER_FASTER" not in codes(box_envelope([UNIT], proximity=rules((2.0, 0.1), (0.5, None))))
    assert "LINT_PROXIMITY_SPEED_NOT_A_LIMIT" in codes(box_envelope([UNIT], proximity=rules((2.0, 0.3))))


def test_speed_limits() -> None:
    assert "LINT_SPEED_LIMIT_ABOVE_PHYSICAL_MAX" in codes(box_envelope([UNIT]), physical_max_speed_mps=0.2)
    assert "LINT_SPEED_LIMIT_ABOVE_PHYSICAL_MAX" not in codes(box_envelope([UNIT]), physical_max_speed_mps=0.25)
    slow = box_envelope([UNIT], motion={"max_speed_mps": 0.009})
    assert "LINT_SPEED_LIMIT_BLOCKS_MOTION" in codes(slow)
    fine = box_envelope([UNIT], motion={"max_speed_mps": 0.0096})  # 0.0096 x 1.05 x 1 s > 0.01 m
    assert "LINT_SPEED_LIMIT_BLOCKS_MOTION" not in codes(fine)


def test_a_0_1_envelope_is_linted_after_its_upgrade() -> None:
    doc = json.loads((REPO / "vectors" / "envelope" / "valid" / "v01-tabletop-arm.json").read_text("utf-8"))
    doc["workspace"]["keep_out"] = [[[0.0, -0.3], [0.5, -0.3], [0.5, 0.3], [0.0, 0.3]]]
    found = lint(load(doc))
    assert [f.code for f in found if f.severity == "error"] == ["LINT_KEEP_OUT_COVERS_KEEP_IN"]


# --------------------------------------------------------------------------- command line


def test_cli_exit_codes_and_json(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    clean = tmp_path / "clean.json"
    clean.write_text(json.dumps(box_envelope([UNIT])), "utf-8")
    assert cli.main(["envelope", "lint", str(clean)]) == 0
    assert "0 lint error(s)" in capsys.readouterr().out
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps(box_envelope([UNIT, box("same", "keep_out", [0.0] * 3, [1.0] * 3)])), "utf-8")
    assert cli.main(["envelope", "lint", "--json", str(bad)]) == 1
    out = json.loads(capsys.readouterr().out)
    assert out["lint_errors"] == 1 and out["lint"][0]["code"] == "LINT_KEEP_OUT_COVERS_KEEP_IN"
    assert cli.main(["envelope", "lint", "--physical-max-speed", "0.1", str(clean)]) == 0
    assert "LINT_SPEED_LIMIT_ABOVE_PHYSICAL_MAX" in capsys.readouterr().out
    assert cli.main(["envelope", "lint", "--physical-max-speed", "-1", str(clean)]) == 2
    invalid = REPO / "vectors" / "envelope" / "invalid" / "polygon-bowtie.json"
    if invalid.is_file():
        assert cli.main(["envelope", "lint", str(invalid)]) == 1
