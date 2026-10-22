"""be-stops/1 (EV-02) and be-model-loss/1 (EV-05): the streams, their segments, their digests and
the check of raw/commands.csv against them."""

from __future__ import annotations

import copy
import math

import pytest
from _report_helpers import golden, in_repo
from bounded_embodiment import canonical, generator
from bounded_embodiment.commandspace import load_command_space
from bounded_embodiment.report import streams
from bounded_embodiment.report.plans import load_model_loss_plan, load_stop_plan

pytestmark = in_repo


def space():
    return load_command_space(golden().COMMAND_SPACE)


def velocity_space():
    doc = copy.deepcopy(golden().COMMAND_SPACE)
    doc["kind"] = "velocity"
    doc["axes"] = [
        {"key": "x", "unit": "m/s", "lo": -0.25, "hi": 0.25, "home": 0.0},
        {"key": "y", "unit": "m/s", "lo": -0.25, "hi": 0.25, "home": 0.0},
        {"key": "z", "unit": "m/s", "lo": -0.25, "hi": 0.25, "home": 0.0},
    ]
    del doc["far_targets"]
    return load_command_space(doc)


def stop_plan(trials=None):
    return load_stop_plan(golden().stop_plan_doc(list(trials or golden().EV02_TRIALS)))


def loss_plan():
    return load_model_loss_plan(golden().loss_plan_doc())


def rows_of(proposals) -> list[tuple]:
    return [(p.seq, p.strategy, generator.encode_payload(p.payload)) for p in proposals]


# --------------------------------------------------------------------------- shared rules


@pytest.mark.parametrize("hz", [1.0, 7.0, 30.0, 50.0, 100.0, 333.0, 1000.0])
@pytest.mark.parametrize("seconds", [0.001, 0.02, 0.1, 0.3, 0.7, 1.0, 1.1, 2.5, 3.3])
def test_ticks_counts_i_over_hz_below_seconds(seconds: float, hz: float) -> None:
    n = streams.ticks(seconds, hz)
    assert all(i / hz < seconds for i in range(n))
    assert n / hz >= seconds
    assert n == len([i for i in range(math.ceil(seconds * hz) + 2) if i / hz < seconds])


def test_walk_step() -> None:
    keys = ("x", "y")
    caps = {"x": 0.01, "y": 0.5}
    assert streams.walk_step({"x": 0.0, "y": 0.0}, {"x": 0.1, "y": -0.2}, keys, caps) == {
        "x": 0.01,
        "y": -0.2,
    }
    assert streams.walk_step({"x": 0.1, "y": 0.0}, {"x": 0.0, "y": 0.0}, keys, caps) == {
        "x": 0.1 - 0.01,  # one cap toward the target, in binary64
        "y": 0.0,
    }
    assert streams.walk_step({"x": 0.0, "y": 0.0}, {"x": 9.0, "y": -9.0}, keys, None) == {
        "x": 9.0,
        "y": -9.0,
    }


def test_split_segments() -> None:
    P = generator.Proposal
    props = [P(0, 0.0, "a", {}), P(1, 0.5, "a", {}), P(2, 0.0, "b", {}), P(3, 0.0, "c", {})]
    assert [[p.seq for p in s] for s in streams.split_segments(props)] == [[0, 1], [2], [3]]
    assert streams.split_segments([]) == []


# --------------------------------------------------------------------------- be-stops


def test_stop_stream_segments() -> None:
    g = golden()
    plan = stop_plan()
    proposals = list(streams.stop_stream(space(), plan, g.STOP_PARAMS))
    segments = streams.split_segments(proposals)
    assert len(segments) == len(plan.trials) == 6
    assert [p.seq for p in proposals] == list(range(len(proposals)))
    n_drive, n_return = streams.ticks(0.3, 50.0), streams.ticks(0.5, 50.0)
    assert (n_drive, n_return) == (15, 25)
    for k, (seg, (source, drive)) in enumerate(zip(segments, plan.trials)):
        kinds = [p.strategy for p in seg]
        want = ["return"] * (n_return if k else 0) + [f"drive:{drive}"] * n_drive + [f"stop:{source}"]
        assert kinds == want
        assert [p.t_s for p in seg] == [i / 50.0 for i in range(len(seg))]
        assert seg[-1].payload == {}
        if k:  # the return walk ends at the start (0.075 m away at 0.005 m a command)
            back = [p for p in seg if p.strategy == "return"][-1].payload
            assert back == {"x": 0.3, "y": 0.0, "z": 0.175}
        last = [p for p in seg if p.strategy.startswith("drive:")][-1].payload
        sign = 1.0 if drive == "x_plus" else -1.0
        assert last["x"] == pytest.approx(0.3 + sign * 15 * 0.005)
        assert (last["y"], last["z"]) == (0.0, 0.175)


def test_stop_stream_without_a_cap_jumps_and_velocity_spaces_restart() -> None:
    g = golden()
    params = {**g.STOP_PARAMS, "step_cap": None}
    seg = streams.split_segments(streams.stop_stream(space(), stop_plan(), params))[1]
    assert seg[0].payload == {"x": 0.3, "y": 0.0, "z": 0.175}  # back in one step
    assert seg[25].payload == {"x": 0.2, "y": 0.0, "z": 0.175}  # at the drive in one step
    doc = g.stop_plan_doc(list(g.EV02_TRIALS))
    doc["start"] = {"x": 0.0, "y": 0.0, "z": 0.0}
    doc["drives"] = [
        {"id": "x_plus", "command": {"x": 0.2, "y": 0.0, "z": 0.0}},
        {"id": "x_minus", "command": {"x": -0.2, "y": 0.0, "z": 0.0}},
    ]
    vplan = load_stop_plan(doc)
    vparams = {**g.STOP_PARAMS, "step_cap": {"x": 0.05, "y": 0.05, "z": 0.05}}
    segs = streams.split_segments(streams.stop_stream(velocity_space(), vplan, vparams))
    for seg in segs:
        assert not any(p.strategy == "return" for p in seg)
        assert seg[0].payload["x"] in (0.05, -0.05)  # every drive ramps up from the start
        assert seg[3].payload["x"] in (0.2, -0.2)


def test_stop_stream_refuses_what_it_cannot_make() -> None:
    g = golden()
    plan = stop_plan()
    with pytest.raises(ValueError, match="step_cap must name every axis"):
        list(streams.stop_stream(space(), plan, {**g.STOP_PARAMS, "step_cap": {"x": 0.01}}))
    doc = g.stop_plan_doc(list(g.EV02_TRIALS))
    doc["drives"][0]["command"]["x"] = 0.9
    with pytest.raises(ValueError, match="ev02-stop-plan.json"):
        list(streams.stop_stream(space(), load_stop_plan(doc), g.STOP_PARAMS))
    with pytest.raises(ValueError, match="needs the run's ev02-stop-plan.json"):
        streams.stream(streams.STOPS, space(), 0, g.STOP_PARAMS)
    with pytest.raises(ValueError, match="needs the run's ev02-stop-plan.json"):
        streams.stream(streams.STOPS, space(), 0, g.STOP_PARAMS, plan=loss_plan())


@pytest.mark.parametrize(
    "params",
    [
        {"hz": 50.0, "drive_s": 0.3, "return_s": 0.5},
        {"hz": 50.0, "drive_s": 0.0, "return_s": 0.5, "step_cap": None},
        {"hz": 50.0, "drive_s": 0.3, "return_s": -1.0, "step_cap": None},
        {"hz": 50.0, "drive_s": 0.3, "return_s": 0.5, "step_cap": {}},
        {"hz": 50.0, "drive_s": 0.3, "return_s": 0.5, "step_cap": {"x": 0.0}},
        {"hz": 50.0, "drive_s": 0.3, "return_s": 0.5, "step_cap": None, "seconds": 1.0},
    ],
)
def test_stop_params_are_exact(params: dict) -> None:
    with pytest.raises(ValueError):
        streams.normalize(streams.STOPS, params)


def test_stop_params_normalize() -> None:
    p = streams.normalize(streams.STOPS, {"hz": 50, "drive_s": 1, "return_s": 0, "step_cap": None})
    assert p == {"hz": 50.0, "drive_s": 1.0, "return_s": 0.0, "step_cap": None}
    assert all(type(v) is float for k, v in p.items() if k != "step_cap")
    assert streams.normalize(streams.STOPS, p) == p


# --------------------------------------------------------------------------- be-model-loss


def test_model_loss_stream_segments() -> None:
    g = golden()
    params = streams.normalize(streams.MODEL_LOSS, g.LOSS_PARAMS)
    proposals = list(streams.model_loss_stream(space(), loss_plan(), g.LOSS_SEED, params))
    segments = streams.split_segments(proposals)
    assert len(segments) == params["trials"] == 3
    n_restart = streams.ticks(params["restart_s"], params["hz"])
    shortest, longest = (streams.ticks(s, 50.0) for s in params["motion_s"])
    for seg in segments:
        motion = [p for p in seg if p.strategy == "motion"]
        restart = [p for p in seg if p.strategy == "restart"]
        assert [p.strategy for p in seg] == ["motion"] * len(motion) + ["restart"] * n_restart
        assert shortest <= len(motion) <= longest
        assert [p.t_s for p in motion] == [i / 50.0 for i in range(len(motion))]
        t0 = (len(motion) - 1) / 50.0 + params["silence_s"]
        assert [p.t_s for p in restart] == [t0 + j / 50.0 for j in range(n_restart)]
    # one walk through the whole stream: no step is larger than the cap on any axis
    previous = {"x": 0.3, "y": 0.1, "z": 0.175}
    for p in proposals:
        for axis in ("x", "y", "z"):
            assert abs(p.payload[axis] - previous[axis]) <= 0.005 + 1e-12
        previous = p.payload
    ys = [p.payload["y"] for p in proposals]
    assert min(ys) == -0.1 and max(ys) == 0.1  # it went round the path


def test_model_loss_stream_is_seeded() -> None:
    g = golden()
    a = rows_of(streams.model_loss_stream(space(), loss_plan(), 5, g.LOSS_PARAMS))
    b = rows_of(streams.model_loss_stream(space(), loss_plan(), 5, g.LOSS_PARAMS))
    c = rows_of(streams.model_loss_stream(space(), loss_plan(), 6, g.LOSS_PARAMS))
    assert a == b and a != c


@pytest.mark.parametrize(
    ("change", "match"),
    [
        ({"motion_s": [2.0, 1.0]}, "not be decreasing"),
        ({"motion_s": [1.0]}, r"\[shortest, longest\]"),
        ({"silence_s": 0.01}, "at least one period"),
        ({"step_cap": None}, "step_cap must be a mapping"),
        ({"trials": 0}, "trials must be a whole number"),
        ({"trials": 1001}, "trials must be a whole number"),
        ({"restart_s": 0}, "restart_s must be above 0"),
    ],
)
def test_model_loss_params_refused(change: dict, match: str) -> None:
    with pytest.raises(ValueError, match=match):
        streams.normalize(streams.MODEL_LOSS, {**golden().LOSS_PARAMS, **change})


# --------------------------------------------------------------------------- digests and checks


def _digest_by_hand(name: str, seed: int, params: dict, plan) -> str:
    import hashlib

    p = streams.normalize(name, params)
    header = (
        f"{name}/1\n{canonical.canonicalize(p).decode('utf-8')}\n{seed}\n{space().digest}\n"
        f"{plan.digest}\n"
    )
    h = hashlib.sha256(header.encode("utf-8"))
    for c in streams.stream(name, space(), seed, p, plan=plan):
        h.update(f"{c.seq},{c.strategy},{generator.encode_payload(c.payload)}\n".encode())
    return "sha256:" + h.hexdigest()


def test_digests_pin_the_plan() -> None:
    g = golden()
    plan = stop_plan()
    d = streams.stream_digest(streams.STOPS, space(), 0, g.STOP_PARAMS, plan=plan)
    assert d == _digest_by_hand(streams.STOPS, 0, g.STOP_PARAMS, plan)
    doc = g.stop_plan_doc(list(g.EV02_TRIALS))
    doc["description"] = "another description, the same commands"
    other = streams.stream_digest(streams.STOPS, space(), 0, g.STOP_PARAMS, plan=load_stop_plan(doc))
    assert other != d
    lp = loss_plan()
    d5 = streams.stream_digest(streams.MODEL_LOSS, space(), 5, g.LOSS_PARAMS, plan=lp)
    assert d5 == _digest_by_hand(streams.MODEL_LOSS, 5, g.LOSS_PARAMS, lp)


def test_digests_match_the_golden_reports() -> None:
    from _report_helpers import read_report

    g = golden()
    for name, plan, seed, params in (
        ("ev02-pass", stop_plan(), 0, g.STOP_PARAMS),
        ("ev05-pass", loss_plan(), g.LOSS_SEED, g.LOSS_PARAMS),
    ):
        report = read_report(g.OUT / name)
        gen = report["generator"]
        assert gen["stream_sha256"] == streams.stream_digest(gen["name"], space(), seed, params, plan=plan)
        assert gen["seed"] == seed


def test_check_against_commands() -> None:
    g = golden()
    plan = stop_plan()
    rows = rows_of(streams.stop_stream(space(), plan, g.STOP_PARAMS))
    whole = streams.check("be-stops", "1", space(), 0, g.STOP_PARAMS, rows, plan=plan)
    assert whole.findings == () and whole.commands_compared == len(rows) and whole.aborted is False
    prefix = streams.check("be-stops", "1", space(), 0, g.STOP_PARAMS, rows[:20], plan=plan)
    assert [f.code for f in prefix.findings] == ["STREAM_PREFIX"] and prefix.aborted is True
    bad = list(rows)
    bad[15] = (15, "stop:power_loss", "{}")
    assert [f.code for f in streams.check("be-stops", "1", space(), 0, g.STOP_PARAMS, bad, plan=plan).findings] == [
        "STREAM_MISMATCH"
    ]
    missing = streams.check("be-stops", "1", space(), 0, g.STOP_PARAMS, rows)
    assert [f.code for f in missing.findings] == ["STREAM_PARAMS_INVALID"]
    lp = loss_plan()
    lrows = rows_of(streams.model_loss_stream(space(), lp, 5, g.LOSS_PARAMS))
    ok = streams.check("be-model-loss", "1", space(), 5, g.LOSS_PARAMS, lrows, plan=lp)
    assert ok.findings == () and ok.commands_compared == len(lrows)
    reseeded = streams.check("be-model-loss", "1", space(), 6, g.LOSS_PARAMS, lrows, plan=lp)
    assert [f.code for f in reseeded.findings] == ["STREAM_MISMATCH"]
    huge = {**g.LOSS_PARAMS, "trials": 1000, "motion_s": [80000.0, 80000.0]}
    assert [f.code for f in streams.check("be-model-loss", "1", space(), 5, huge, [], plan=lp).findings] == [
        "STREAM_TOO_LARGE"
    ]
