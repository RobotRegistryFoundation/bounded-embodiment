"""The hostile command generator: golden digests, determinism, each strategy, the shakedown band,
parameters, the control case, payload text and the commands.csv comparison."""

from __future__ import annotations

import csv
import hashlib
import io
import itertools
import json
import math
import os
import random
import subprocess
import sys
from collections import Counter
from pathlib import Path

import pytest

from bounded_embodiment import canonical
from bounded_embodiment import generator as gen
from bounded_embodiment.commandspace import MALFORMED_KINDS, load_command_space
from bounded_embodiment.findings import errors
from bounded_embodiment.generator import (
    SHAKEDOWN_STRATEGIES,
    STRATEGIES,
    compare_to_commands,
    control_stream,
    decode_payload,
    default_params,
    encode_payload,
    far_target_pool,
    hostile_stream,
    normalize_params,
    stream_sha256,
)

REPO = Path(__file__).resolve().parents[3]
SRC = REPO / "packages" / "core" / "src"
GOLDEN = json.loads((REPO / "vectors" / "streams" / "golden.json").read_text("utf-8"))
CASES = {c["id"]: c for c in GOLDEN["cases"]}
SO101_PATH = REPO / "examples" / "command-spaces" / "so101.command-space.json"
ROVER_PATH = REPO / "examples" / "command-spaces" / "rover.command-space.json"
CARTESIAN_PATH = REPO / "vectors" / "command-space" / "valid" / "cartesian-arm.json"
START_POSE = CASES["so101-seed7-shakedown-60s"]["params"]["center"]


@pytest.fixture(scope="module")
def so101():
    return load_command_space(SO101_PATH)


@pytest.fixture(scope="module")
def cartesian():
    return load_command_space(CARTESIAN_PATH)


@pytest.fixture(scope="module")
def stream(so101):
    """The full default stream for so101, seed 7 (about 50,000 commands), built once."""
    return list(hostile_stream(so101, 7))


def rows_of(proposals):
    return [(p.seq, p.strategy, encode_payload(p.payload)) for p in proposals]


def by_strategy(proposals, name):
    return [p for p in proposals if p.strategy == name]


# ----- golden digests --------------------------------------------------------------------------
def test_golden_file_names_this_generator():
    assert GOLDEN["generator"] == f"{gen.GENERATOR}/{gen.GENERATOR_VERSION}" == "be-hostile/1"
    defaults = {c["id"] for c in GOLDEN["cases"] if c["params"] == default_params()}
    assert {"so101-seed7", "so101-seed11", "so101-seed23", "rover-seed7"} <= defaults


@pytest.mark.parametrize("case", GOLDEN["cases"], ids=lambda c: c["id"])
def test_golden_stream(case):
    space = load_command_space(REPO / case["space"])
    assert space.digest == case["space_digest"], "the command space file changed"
    assert normalize_params(case["params"]) == case["params"]
    assert stream_sha256(space, case["seed"], case["params"]) == case["stream_sha256"]
    counts = Counter(p.strategy for p in hostile_stream(space, case["seed"], case["params"]))
    assert {s: n for s, n in counts.items()} == case["per_strategy"]
    total = 0
    for n in counts.values():
        total += n
    assert total == case["commands"]


def test_default_params_are_the_default(so101):
    case = CASES["so101-seed7-5s"]
    short = {"seconds": 5}
    assert stream_sha256(so101, 7, short) == case["stream_sha256"]
    # however the same parameters are spelled
    respelled = {"seconds": 5.0, "segment_s": (0.5, 4), "strategies": list(reversed(STRATEGIES))}
    assert stream_sha256(so101, 7, respelled) == case["stream_sha256"]
    assert normalize_params(None) == normalize_params({}) == default_params()


def test_digest_text_is_as_documented(so101):
    """Rebuild the hashed text by hand from the documented format."""
    case = CASES["so101-seed7-5s"]
    params = normalize_params(case["params"])
    text = "be-hostile/1\n" + canonical.canonicalize(params).decode() + "\n7\n"
    text += so101.digest + "\n"
    lines = [
        f"{p.seq},{p.strategy},{encode_payload(p.payload)}\n"
        for p in hostile_stream(so101, 7, params)
    ]
    assert "".join(line + "\n" for line in case["head"]) == text + lines[0] + lines[1]
    digest = hashlib.sha256((text + "".join(lines)).encode("utf-8")).hexdigest()
    assert "sha256:" + digest == case["stream_sha256"]


def test_digest_covers_seed_params_space_and_version(so101, cartesian, monkeypatch):
    base = stream_sha256(so101, 7, {"seconds": 1})
    assert stream_sha256(so101, 8, {"seconds": 1}) != base
    assert stream_sha256(so101, 7, {"seconds": 1, "burst_hz": 400}) != base
    assert stream_sha256(cartesian, 7, {"seconds": 1}) != base
    monkeypatch.setattr(gen, "GENERATOR_VERSION", "2")
    assert stream_sha256(so101, 7, {"seconds": 1}) != base


# ----- determinism -----------------------------------------------------------------------------
class OnlyRandom(random.Random):
    """A Random that refuses every method but random() (and the seeding __init__ needs)."""

    calls = 0

    def random(self):
        OnlyRandom.calls += 1
        return super().random()

    def __getattribute__(self, name):
        if not name.startswith("__") and name not in ("random", "seed", "gauss_next"):
            value = super().__getattribute__(name)
            if callable(value):
                raise AssertionError(f"the generator called Random.{name}")
            return value
        return super().__getattribute__(name)


def test_only_random_is_used(so101, cartesian, monkeypatch):
    monkeypatch.setattr(random, "Random", OnlyRandom)

    def refuse(*args, **kwargs):
        raise AssertionError("the generator used the random module's shared generator")

    for name in ("random", "uniform", "choice", "choices", "randint", "randrange", "shuffle",
                 "sample", "gauss", "getrandbits", "seed"):
        monkeypatch.setattr(random, name, refuse)
    OnlyRandom.calls = 0
    # a full stream (every strategy and malformed kind) and the cartesian case, through the patch
    assert stream_sha256(so101, 7) == CASES["so101-seed7"]["stream_sha256"]
    case = CASES["cartesian-arm-seed7-120s"]
    assert stream_sha256(cartesian, 7, case["params"]) == case["stream_sha256"]
    assert len(far_target_pool(so101, 7)) == 27
    assert OnlyRandom.calls > 50_000  # the patched class really was the one used


def test_the_stream_is_the_same_under_any_string_hash_seed():
    """Nothing may depend on set or dict-of-set ordering, which PYTHONHASHSEED changes."""
    case = CASES["so101-seed7-5s"]
    code = (
        "import json, sys\n"
        "from bounded_embodiment.commandspace import load_command_space\n"
        "from bounded_embodiment.generator import stream_sha256\n"
        "space = load_command_space(sys.argv[1])\n"
        "print(stream_sha256(space, 7, json.loads(sys.argv[2])))\n"
    )
    for hash_seed in ("0", "4242"):
        env = dict(os.environ, PYTHONHASHSEED=hash_seed, PYTHONPATH=str(SRC))
        out = subprocess.run(
            [sys.executable, "-c", code, str(SO101_PATH), json.dumps(case["params"])],
            capture_output=True, text=True, env=env, check=True,
        ).stdout.strip()
        assert out == case["stream_sha256"], hash_seed


def test_same_digest_same_stream():
    """Spellings RFC 8785 does not tell apart (0 and 0.0 and -0.0, 30 and 30.0) give one digest,
    so they must give one stream."""
    doc = json.loads(CARTESIAN_PATH.read_text("utf-8"))
    assert doc["axes"][1]["home"] == 0 and type(doc["axes"][3]["hi"]) is int
    respelled = json.loads(json.dumps(doc))
    respelled["axes"][1]["home"] = -0.0
    respelled["axes"][3].update(lo=0.0, hi=100.0, home=30.0)
    a, b = load_command_space(doc), load_command_space(respelled)
    assert a.digest == b.digest
    assert stream_sha256(a, 3, {"seconds": 20}) == stream_sha256(b, 3, {"seconds": 20})


def test_stream_is_lazy_and_repeatable(so101):
    first = rows_of(hostile_stream(so101, 11, {"seconds": 3}))
    again = rows_of(hostile_stream(so101, 11, {"seconds": 3}))
    assert first == again
    assert rows_of(hostile_stream(so101, 12, {"seconds": 3})) != first


# ----- the stream's shape ----------------------------------------------------------------------
def test_every_strategy_appears_and_seq_counts_from_zero(stream):
    assert {p.strategy for p in stream} == set(STRATEGIES)
    assert [p.seq for p in stream] == list(range(len(stream)))
    assert len(stream) == CASES["so101-seed7"]["commands"]


def test_rates_and_timing(stream):
    """fast commands go out at burst_hz, every other one at base_hz, from t = 0 until 600 s."""
    assert stream[0].t_s == 0.0
    for p, q in itertools.pairwise(stream):
        want = 1 / 500 if p.strategy == "fast" else 1 / 50
        assert q.t_s - p.t_s == pytest.approx(want, rel=1e-6)
    last = stream[-1]
    assert last.t_s < 600.0 <= last.t_s + (1 / 500 if last.strategy == "fast" else 1 / 50)
    for name in STRATEGIES:
        ps = by_strategy(stream, name)
        dt = 1 / 500 if name == "fast" else 1 / 50
        seconds_in_strategy = math.fsum(dt for _ in ps)
        assert len(ps) / seconds_in_strategy == pytest.approx(1 / dt)
        # each strategy gets a fair share of the 600 s (expected 1/8 each)
        assert 0.05 < seconds_in_strategy / 600.0 < 0.25, name


def test_uniform_covers_150_percent_of_the_range(so101, stream):
    outside = 0
    for p in by_strategy(stream, "uniform"):
        assert list(p.payload) == list(so101.keys)
        for a in so101.axes:
            v, quarter = p.payload[a.key], 0.25 * (a.hi - a.lo)
            assert type(v) is float and a.lo - quarter <= v <= a.hi + quarter
            outside += not a.lo <= v <= a.hi
    assert outside > 1000


def test_boundary_is_additive_with_one_k_per_command(so101, stream):
    seen_k = set()
    for p in by_strategy(stream, "boundary"):
        ks = set()
        for a in so101.axes:
            v, half = p.payload[a.key], 0.5 * (a.hi - a.lo)
            matches = {k for k in gen.BOUNDARY_K if v in (a.hi + k * half, a.lo - k * half)}
            assert matches, (a.key, v)
            ks.add(frozenset(matches))
        common = frozenset.intersection(*ks)
        assert len(common) == 1, p
        seen_k |= common
    assert seen_k == set(gen.BOUNDARY_K)


def test_max_uses_one_magnitude_per_command(stream):
    seen = set()
    for p in by_strategy(stream, "max"):
        mags = {abs(v) for v in p.payload.values()}
        assert len(mags) == 1 and mags <= {1e3, 1e6}
        seen |= mags
        assert all(type(v) is float for v in p.payload.values())
    assert seen == {1e3, 1e6}


def test_sign_flip_alternates_every_command_starting_at_hi(so101, stream):
    hi = {a.key: a.hi for a in so101.axes}
    lo = {a.key: a.lo for a in so101.axes}
    flips = by_strategy(stream, "sign_flip")
    assert flips
    for n, p in enumerate(flips):
        assert p.payload == (hi if n % 2 == 0 else lo)


def test_drive_out_and_fast_hold_targets_from_the_pool(so101, stream):
    pool = [values for _, values in far_target_pool(so101, 7)]
    held = set()
    for p in by_strategy(stream, "drive_out") + by_strategy(stream, "fast"):
        assert p.payload in pool
        held.add(pool.index(p.payload))
    assert len(held) > 10


def test_partial_names_one_axis_inside_its_limits(so101, stream):
    keys = set()
    for p in by_strategy(stream, "partial"):
        ((key, value),) = p.payload.items()
        a = so101.axis(key)
        assert type(value) is float and a.lo <= value <= a.hi
        keys.add(key)
    assert keys == set(so101.keys)


def classify(payload: dict, keys: tuple) -> str:
    if payload == {}:
        return "empty"
    if gen.UNKNOWN_AXIS_KEY in payload:
        assert len(payload) == 2 and payload[gen.UNKNOWN_AXIS_KEY] == 99.0
        return "unknown_key"
    if len(payload) == len(keys) - 1:
        return "missing_key"
    odd = [v for v in payload.values() if type(v) is not float or not math.isfinite(v)]
    assert len(odd) == 1 and len(payload) == len(keys), payload
    v = odd[0]
    if v is None:
        return "null"
    if v is True:
        return "bool"
    if isinstance(v, str):
        float(v)  # the value as text
        return "string"
    return "nan" if math.isnan(v) else "inf" if v > 0 else "neg_inf"


def test_malformed_bends_the_far_target_in_every_declared_way(so101, stream):
    kinds = Counter(classify(p.payload, so101.keys) for p in by_strategy(stream, "malformed"))
    assert set(kinds) == set(MALFORMED_KINDS)


def test_malformed_is_limited_to_what_the_transport_carries(cartesian):
    case = CASES["cartesian-arm-seed7-120s"]
    proposals = list(hostile_stream(cartesian, 7, case["params"]))
    kinds = {classify(p.payload, cartesian.keys) for p in by_strategy(proposals, "malformed")}
    assert kinds == set(cartesian.malformed)
    for p in proposals:
        text = encode_payload(p.payload)
        assert "NaN" not in text and "Infinity" not in text


def test_payload_values_are_floats(stream):
    for p in stream:
        if p.strategy != "malformed":
            assert all(type(v) is float for v in p.payload.values()), p


# ----- the far-target pool ---------------------------------------------------------------------
def test_far_target_pool_order_and_values(so101):
    pool = far_target_pool(so101, 7)
    ids = [i for i, _ in pool]
    axes_part = [f"{side}:{k}" for k in so101.keys for side in ("hi", "lo")]
    assert ids == axes_part + [f"corner:{n}" for n in range(8)] + [
        f"lab:{ft.id}" for ft in so101.far_targets
    ]
    home = {a.key: a.home for a in so101.axes}
    for target_id, values in pool[:12]:
        side, key = target_id.split(":", 1)
        a = so101.axis(key)
        limit = a.hi if side == "hi" else a.lo
        assert values == {**home, key: a.home + 0.95 * (limit - a.home)}
    for _, values in pool[20:]:
        assert values in [ft.values for ft in so101.far_targets]


def test_corners_are_the_streams_first_draws(so101):
    """Corner by corner, axis by axis, one draw each: below 0.5 is the lo side."""
    r = random.Random(23)
    expected = []
    for _ in range(8):
        corner = {}
        for a in so101.axes:
            limit = a.hi if r.random() < 0.5 else a.lo
            corner[a.key] = a.home + 0.95 * (limit - a.home)
        expected.append(corner)
    assert [v for i, v in far_target_pool(so101, 23) if i.startswith("corner:")] == expected
    assert far_target_pool(so101, 23) != far_target_pool(so101, 24)


def test_cartesian_pool_goes_past_each_metric_face(cartesian):
    pool = dict(far_target_pool(cartesian, 7))
    home = {a.key: a.home for a in cartesian.axes}
    faces = [i for i in pool if i.startswith("face_")]
    assert faces == [f"face_{side}:{k}" for k in ("x", "y", "z") for side in ("lo", "hi")]
    assert pool["face_lo:z"] == {**home, "z": 0.02 - 0.2}
    assert pool["face_hi:x"] == {**home, "x": 0.4 + 0.2}
    assert list(pool)[-7] == "lab:below_table"  # the faces come last; the gripper has none


# ----- amplitude < 1: the hardware shakedown ---------------------------------------------------
def band(axis, center, amplitude):
    c = center.get(axis.key, axis.home)
    return c + (axis.lo - c) * amplitude, c + (axis.hi - c) * amplitude


def test_shakedown_never_leaves_its_band(so101):
    params = {"amplitude": 0.15, "center": START_POSE}
    proposals = list(hostile_stream(so101, 7, params))
    assert {p.strategy for p in proposals} == set(SHAKEDOWN_STRATEGIES)
    for p in proposals:
        for key, value in p.payload.items():
            lo, hi = band(so101.axis(key), START_POSE, 0.15)
            assert lo <= value <= hi, (p, key)
    for p in by_strategy(proposals, "boundary"):  # k is 0: exactly on the band's limits
        for key, value in p.payload.items():
            assert value in band(so101.axis(key), START_POSE, 0.15)


def test_shakedown_pool_and_safety(so101):
    assert normalize_params({"amplitude": 0.5})["strategies"] == list(SHAKEDOWN_STRATEGIES)
    assert normalize_params({"amplitude": 0.5, "strategies": ["partial", "uniform"]})[
        "strategies"
    ] == ["uniform", "partial"]
    for unsafe in (["max"], ["uniform", "drive_out"], list(STRATEGIES)):
        with pytest.raises(ValueError, match="shakedown"):
            normalize_params({"amplitude": 0.5, "strategies": unsafe})


def test_center_must_name_axes_within_their_limits(so101):
    with pytest.raises(ValueError, match="not axes"):
        hostile_stream(so101, 7, {"amplitude": 0.5, "center": {"elbow": 0.0}})
    with pytest.raises(ValueError, match="outside"):
        hostile_stream(so101, 7, {"amplitude": 0.5, "center": {"gripper.pos": 120.0}})


# ----- parameters and seeds --------------------------------------------------------------------
def test_default_params():
    assert default_params() == {
        "seconds": 600.0,
        "base_hz": 50.0,
        "burst_hz": 500.0,
        "segment_s": [0.5, 4.0],
        "amplitude": 1.0,
        "center": None,
        "strategies": list(STRATEGIES),
    }
    assert default_params() is not default_params()


def test_normalize_params_spells_everything_one_way():
    p = normalize_params(
        {"seconds": 60, "segment_s": (1, 2), "center": {"x": -0.0}, "strategies": ("max", "fast")}
    )
    assert p["seconds"] == 60.0 and type(p["seconds"]) is float
    assert p["segment_s"] == [1.0, 2.0]
    assert p["center"] == {"x": 0.0} and math.copysign(1.0, p["center"]["x"]) == 1.0
    assert p["strategies"] == ["max", "fast"]
    assert normalize_params(p) == p
    canonical.canonicalize(p)  # JCS-safe


@pytest.mark.parametrize(
    "params",
    [
        {"secs": 600},
        {"seconds": 0},
        {"seconds": -1},
        {"seconds": math.nan},
        {"seconds": math.inf},
        {"seconds": "600"},
        {"seconds": True},
        {"seconds": 10**400},
        {"seconds": 86400.5},
        {"base_hz": 0},
        {"burst_hz": 20000},
        {"segment_s": [4, 0.5]},
        {"segment_s": [0.5]},
        {"segment_s": [0, 1]},
        {"segment_s": "0.5,4"},
        {"amplitude": 0},
        {"amplitude": 1.5},
        {"center": "home"},
        {"center": {"x": "1"}},
        {"center": {1: 2.0}},
        {"strategies": []},
        {"strategies": "uniform"},
        {"strategies": ["uniform", "uniform"]},
        {"strategies": ["teleport"]},
        {"strategies": None},
        ["seconds", 600],
    ],
)
def test_bad_params_raise_value_error(params):
    with pytest.raises(ValueError):
        normalize_params(params)


@pytest.mark.parametrize("seed", [-1, True, 1.5, "7", None, 2**53 + 1])
def test_bad_seeds_raise_value_error(so101, seed):
    with pytest.raises(ValueError, match="seed"):
        hostile_stream(so101, seed)
    with pytest.raises(ValueError, match="seed"):
        far_target_pool(so101, seed)


def test_extreme_good_seeds(so101):
    for seed in (0, 2**53):
        assert next(hostile_stream(so101, seed)).seq == 0


# ----- the control case ------------------------------------------------------------------------
def test_control_stream_asks_for_one_target(so101):
    target = so101.far_target("full_reach_forward").values
    proposals = list(control_stream(so101, "full_reach_forward", 10.0))
    assert len(proposals) == 500
    for n, p in enumerate(proposals):
        assert (p.seq, p.t_s, p.strategy, p.payload) == (n, n / 50, "drive_out", target)
    assert len(list(control_stream(so101, "overhead", 1.0, hz=200))) == 200


def test_control_stream_walks_from_home_under_a_step_cap(so101):
    target = so101.far_target("into_table").values
    cap = {k: 2.0 for k in so101.keys}
    proposals = list(control_stream(so101, "into_table", 2.0, step_cap=cap))
    previous = {a.key: a.home for a in so101.axes}
    for p in proposals:
        for key, value in p.payload.items():
            assert abs(value - previous[key]) <= 2.0
        previous = p.payload
    # the farthest axis is wrist_flex, 90 degrees from home: 45 steps of 2, then hold
    assert proposals[0].payload["wrist_flex.pos"] == 2.0
    assert proposals[43].payload != target and proposals[44].payload == target
    assert all(p.payload == target for p in proposals[44:])
    assert {p.strategy for p in proposals} == {"drive_out"}


def test_control_stream_checks_its_arguments(so101):
    with pytest.raises(KeyError, match="unknown far target"):
        control_stream(so101, "nowhere", 10.0)
    with pytest.raises(ValueError, match="missing"):
        control_stream(so101, "overhead", 10.0, step_cap={"gripper.pos": 1.0})
    with pytest.raises(ValueError, match="unknown"):
        control_stream(so101, "overhead", 10.0, step_cap={**{k: 1.0 for k in so101.keys}, "x": 1})
    with pytest.raises(ValueError):
        control_stream(so101, "overhead", 10.0, step_cap={k: 0.0 for k in so101.keys})
    with pytest.raises(ValueError):
        control_stream(so101, "overhead", 0.0)
    with pytest.raises(ValueError):
        control_stream(so101, "overhead", 10.0, hz=-50)


# ----- payload text ----------------------------------------------------------------------------
def test_encode_payload_text():
    text = encode_payload(
        {"b": math.nan, "a": math.inf, "c": -math.inf, "d": None, "e": True, "f": "45.0", "g": 1.5}
    )
    assert text == (
        '{"a":Infinity,"b":NaN,"c":-Infinity,"d":null,"e":true,"f":"45.0","g":1.5}'
    )
    assert encode_payload({}) == "{}"
    assert encode_payload({"é": 0.1 + 0.2}) == '{"\\u00e9":0.30000000000000004}'


@pytest.mark.parametrize(
    "payload",
    [
        {"x": math.nan},
        {"x": math.inf, "y": -math.inf},
        {"x": None, "y": True, "z": False},
        {"x": "104.5", "unknown_axis": 99.0},
        {"x": 1e-07, "y": -1e22, "z": 123456789.125},
        {},
    ],
)
def test_encode_decode_round_trip(payload):
    text = encode_payload(payload)
    back = decode_payload(text)
    assert encode_payload(back) == text
    for key, value in payload.items():
        if isinstance(value, float) and math.isnan(value):
            assert math.isnan(back[key])
        else:
            assert back[key] == value and type(back[key]) is type(value)


def test_decode_payload_wants_an_object():
    with pytest.raises(ValueError):
        decode_payload("[1, 2]")


def test_payload_json_survives_a_csv_file(so101):
    """commands.csv quotes payload_json (it has commas and quotes); csv reads it back intact."""
    proposals = list(hostile_stream(so101, 23, {"seconds": 30}))
    buffer = io.StringIO()
    writer = csv.writer(buffer)
    writer.writerow(["seq", "t_sent_s", "t_done_s", "strategy", "payload_json", "result",
                     "error_kind"])
    for p in proposals:
        payload_json = encode_payload(p.payload)
        writer.writerow([p.seq, f"{p.t_s:.4f}", "", p.strategy, payload_json, "ok", ""])
    buffer.seek(0)
    reader = csv.DictReader(buffer)
    rows = [(r["seq"], r["strategy"], r["payload_json"]) for r in reader]
    assert [r[2] for r in rows] == [encode_payload(p.payload) for p in proposals]
    assert compare_to_commands(so101, 23, {"seconds": 30}, rows) == []


# ----- compare_to_commands ---------------------------------------------------------------------
SHORT = {"seconds": 5}


@pytest.fixture(scope="module")
def short_rows(so101):
    return rows_of(hostile_stream(so101, 7, SHORT))


def only(findings):
    assert len(findings) == 1, findings
    f = findings[0]
    return f.code, f.severity, f.where


def test_compare_the_whole_stream(so101, short_rows):
    assert len(short_rows) == CASES["so101-seed7-5s"]["commands"]
    assert compare_to_commands(so101, 7, SHORT, short_rows) == []
    assert compare_to_commands(so101, 7, SHORT, iter(short_rows)) == []


def test_compare_a_prefix_is_fine(so101, short_rows, stream):
    findings = compare_to_commands(so101, 7, SHORT, short_rows[:100])
    assert only(findings) == ("STREAM_PREFIX", "info", "")
    assert "first 100 commands" in findings[0].detail and errors(findings) == []
    assert only(compare_to_commands(so101, 7, SHORT, []))[0] == "STREAM_PREFIX"
    # an aborted 600 s run
    assert only(compare_to_commands(so101, 7, None, rows_of(stream[:2000])))[0] == "STREAM_PREFIX"


def test_compare_catches_a_changed_payload(so101, short_rows):
    rows = list(short_rows)
    seq, strategy, payload = rows[400]
    rows[400] = (seq, strategy, payload.replace(":", ": ", 1))  # same command, other spelling
    assert only(compare_to_commands(so101, 7, SHORT, rows)) == (
        "STREAM_MISMATCH",
        "error",
        "rows[400]",
    )
    rows[400] = (seq, "drive_out" if strategy != "drive_out" else "fast", payload)
    assert only(compare_to_commands(so101, 7, SHORT, rows))[0] == "STREAM_MISMATCH"


def test_compare_catches_reordered_rows(so101, short_rows):
    rows = list(short_rows)
    i = next(k for k in range(len(rows) - 1) if rows[k][2] != rows[k + 1][2])
    rows[i], rows[i + 1] = rows[i + 1], rows[i]
    assert only(compare_to_commands(so101, 7, SHORT, rows)) == (
        "STREAM_SEQ_GAP",
        "error",
        f"rows[{i}]",
    )
    # the same swap with the seq column renumbered: the content no longer matches
    renumbered = [(n, s, p) for n, (_, s, p) in enumerate(rows)]
    assert only(compare_to_commands(so101, 7, SHORT, renumbered)) == (
        "STREAM_MISMATCH",
        "error",
        f"rows[{i}]",
    )


def test_compare_catches_a_seq_gap(so101, short_rows):
    rows = short_rows[:300] + short_rows[301:]
    code, severity, where = only(compare_to_commands(so101, 7, SHORT, rows))
    assert (code, severity, where) == ("STREAM_SEQ_GAP", "error", "rows[300]")


def test_compare_catches_an_over_long_stream(so101, short_rows):
    n = len(short_rows)
    rows = short_rows + [(n, "uniform", "{}")]
    findings = compare_to_commands(so101, 7, SHORT, rows)
    assert only(findings) == ("STREAM_TOO_LONG", "error", f"rows[{n}]")
    assert f"has {n} commands" in findings[0].detail


def test_compare_reads_seq_as_text(so101, short_rows):
    as_text = [(str(n), s, p) for n, s, p in short_rows]
    assert compare_to_commands(so101, 7, SHORT, as_text) == []
    for bad in ("01", "x", "-0", " 1", True, 1.0):
        rows = [as_text[0], (bad, *as_text[1][1:])]
        assert only(compare_to_commands(so101, 7, SHORT, rows))[0] == "STREAM_SEQ_GAP", bad


def test_compare_reports_bad_parameters_as_a_finding(so101, short_rows):
    for seed, params in ((-1, SHORT), (7, {"seconds": -5}), (7, {"amplitude": 0.5,
                                                                  "strategies": ["max"]})):
        code, severity, _ = only(compare_to_commands(so101, seed, params, short_rows))
        assert (code, severity) == ("STREAM_PARAMS_INVALID", "error")


def test_compare_stops_at_the_first_problem(so101, short_rows):
    garbage = [(n, "uniform", "{}") for n in range(len(short_rows))]
    assert len(compare_to_commands(so101, 7, SHORT, garbage)) == 1


def test_the_same_comparison_checks_a_control_run(so101):
    rows = rows_of(control_stream(so101, "overhead", 2.0))
    assert gen._compare_rows(control_stream(so101, "overhead", 2.0), rows) == []
    other = control_stream(so101, "behind_base", 2.0)
    assert only(gen._compare_rows(other, rows))[0] == "STREAM_MISMATCH"
