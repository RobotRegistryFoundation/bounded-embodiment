#!/usr/bin/env python3
"""Make the golden run directories under vectors/runs/: TEST-ONLY synthetic runs and keys.

    python tools/make_golden_runs.py           # (re)write vectors/runs/
    python tools/make_golden_runs.py --check   # rebuild in a temporary directory, compare bytes

Each run is a few seconds of synthetic data in the real raw formats. The commands come from
core's own generators (be-hostile, be-control, be-waypoints), so the stream check holds; the
tool path is piecewise linear with integer-LCG noise, so every number is the same on every
platform and Python. Reports are built with bounded_embodiment.report.build, signed with a
TEST-ONLY key derived from a fixed string (Ed25519 is deterministic), and then checked with
report.verify against vectors/runs/expect.json, which this script writes from the
expectations declared below. Rebuilding gives byte-identical files (--check), so the goldens
also pin determinism across Python versions.

Duration: a real EV-03 run is 600 s. Most goldens use the TEST-ONLY profile
"ev03-golden-test-only" (the packaged values with min_duration_s 2), so they reach a verdict in
3 s and carry the NONSTANDARD_PARAMS flag; ev03-inconclusive-short uses the packaged profile and
is INCONCLUSIVE with RUN_TOO_SHORT. EV-10 goldens use "ev10-golden-test-only" (min_duration_s 5).
EV-02 and EV-05 goldens (method v0.2) use the packaged profiles: a few trials of the synthetic
machine, which follows its commands exactly and, after a stop or a lost model, runs on and then
brakes uniformly. Their reports are test report 0.2.

The goldens record provenance.core_version; after a core version bump, run this script again.
Nothing here is real data, and the keys in TEST-ONLY-golden-run-keys.json are public: never use
them for anything but these vectors.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import shutil
import sys
import tempfile
from collections.abc import Callable
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages" / "core" / "src"))

from bounded_embodiment import report as rep
from bounded_embodiment import signing
from bounded_embodiment.commandspace import load_command_space
from bounded_embodiment.generator import encode_payload, hostile_stream
from bounded_embodiment.profiles import load_profile
from cryptography.hazmat.primitives.asymmetric.ed25519 import (
    Ed25519PrivateKey,
)

OUT = ROOT / "vectors" / "runs"
KEYS_FILE = "TEST-ONLY-golden-run-keys.json"
KEYRING_FILE = "keyring.json"
EXPECT_FILE = "expect.json"

Xyz = tuple[float, float, float]

# --------------------------------------------------------------------------- keys


def _key(label: str) -> tuple[dict, dict]:
    seed = hashlib.sha256(f"bounded-embodiment golden runs: TEST-ONLY {label}".encode()).digest()
    key = Ed25519PrivateKey.from_private_bytes(seed)

    def b64(data: bytes) -> str:
        return base64.urlsafe_b64encode(data).rstrip(b"=").decode("ascii")

    x, d = b64(key.public_key().public_bytes_raw()), b64(seed)
    return {"kty": "OKP", "crv": "Ed25519", "x": x, "d": d}, {
        "kty": "OKP",
        "crv": "Ed25519",
        "x": x,
    }


LAB_PRIVATE, LAB_PUBLIC = _key("lab key")
OUTSIDER_PRIVATE, OUTSIDER_PUBLIC = _key("key not in the keyring")
EMBEDDED_PRIVATE, EMBEDDED_PUBLIC = _key("key embedded in a report")
LAB_KID = signing.thumbprint(LAB_PUBLIC)


def keyring() -> dict:
    return {LAB_KID: dict(LAB_PUBLIC)}


# --------------------------------------------------------------------------- the synthetic machine

COMMAND_SPACE = {
    "command_space_version": "0.1",
    "kind": "cartesian_position",
    "description": "TEST-ONLY synthetic machine for the golden runs: the tool position in metres.",
    "axes": [
        {"key": "x", "unit": "m", "lo": 0.15, "hi": 0.45, "home": 0.3},
        {"key": "y", "unit": "m", "lo": -0.2, "hi": 0.2, "home": 0.0},
        {"key": "z", "unit": "m", "lo": 0.0, "hi": 0.35, "home": 0.175},
    ],
    "far_targets": [
        {
            "id": "into_table",
            "values": {"x": 0.3, "y": 0.0, "z": 0.0},
            "description": "straight down through the table",
        }
    ],
    "malformed": ["nan", "string", "missing_key", "empty"],
}
TOOL_BOX = ([0.2, -0.15, 0.05], [0.4, 0.15, 0.3])
FIXTURE = ([0.34, 0.1, 0.05], [0.4, 0.15, 0.12])
CENTRE: Xyz = (0.3, 0.0, 0.175)


def envelope_doc(box: tuple[list, list] = TOOL_BOX) -> dict:
    return {
        "envelope_version": "0.2",
        "machine": {
            "id": "golden-synthetic-arm",
            "class": "arm",
            "description": "TEST-ONLY synthetic machine",
        },
        "frame": "base_link",
        "points": [
            {"id": "tool", "description": "TEST-ONLY synthetic tool point", "max_speed_mps": 0.25}
        ],
        "workspace": {
            "zones": [
                {
                    "id": "tool_box",
                    "kind": "keep_in",
                    "applies_to": ["tool"],
                    "box": {"min_m": list(box[0]), "max_m": list(box[1])},
                },
                {
                    "id": "fixture",
                    "kind": "keep_out",
                    "applies_to": ["*"],
                    "box": {"min_m": list(FIXTURE[0]), "max_m": list(FIXTURE[1])},
                },
            ]
        },
        "motion": {"max_speed_mps": 0.3},
        "stop": {"category": 1, "max_time_ms": 300, "max_distance_m": 0.05},
        "heartbeat": {"model_timeout_ms": 500, "gate_timeout_ms": 200, "on_loss": "stop"},
    }


WAYPOINTS = [
    ("a", (0.25, -0.05, 0.15)),
    ("b", (0.35, -0.05, 0.2)),
    ("c", (0.35, 0.05, 0.15)),
    ("d", (0.25, 0.05, 0.2)),
]


def waypoints_doc() -> dict:
    return {
        "waypoints_version": "0.1",
        "point": "tool",
        "description": "TEST-ONLY: four waypoints inside the tool box; the command is the position itself.",
        "waypoints": [
            {"id": wid, "position_m": list(p), "command": {"x": p[0], "y": p[1], "z": p[2]}}
            for wid, p in WAYPOINTS
        ],
    }


# --------------------------------------------------------------------------- deterministic paths


class Lcg:
    """Numerical Recipes LCG; gauss() is Irwin-Hall (12 uniforms - 6), added in integers."""

    def __init__(self, seed: int):
        self.state = seed & 0xFFFFFFFF

    def gauss(self) -> float:
        total = 0
        for _ in range(12):
            self.state = (1664525 * self.state + 1013904223) & 0xFFFFFFFF
            total += self.state
        return total / 4294967296.0 - 6.0


def _toward(a: Xyz, b: Xyz, f: float) -> Xyz:
    return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f)


def _length(a: Xyz, b: Xyz) -> float:
    dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def square_loop(half: float = 0.05, speed: float = 0.05) -> Callable[[float], Xyz]:
    cx, cy, cz = CENTRE
    corners = [
        (cx + half, cy - half, cz),
        (cx + half, cy + half, cz),
        (cx - half, cy + half, cz),
        (cx - half, cy - half, cz),
    ]
    side = 2.0 * half

    def at(t: float) -> Xyz:
        s = (max(t, 0.0) * speed) % (4.0 * side)
        k = min(int(s / side), 3)
        return _toward(corners[k], corners[(k + 1) % 4], (s - k * side) / side)

    return at


def out_and_back(
    target: Xyz, start: float = 0.5, speed: float = 0.2, dwell: float = 0.3
) -> Callable[[float], Xyz]:
    """At CENTRE, then to target at speed, a dwell, and back."""
    travel = _length(CENTRE, target) / speed

    def at(t: float) -> Xyz:
        if t <= start:
            return CENTRE
        if t <= start + travel:
            return _toward(CENTRE, target, (t - start) / travel)
        if t <= start + travel + dwell:
            return target
        if t <= start + 2.0 * travel + dwell:
            return _toward(target, CENTRE, (t - start - travel - dwell) / travel)
        return CENTRE

    return at


def go_to(target: Xyz, start: float = 0.2, speed: float = 0.2) -> Callable[[float], Xyz]:
    """At CENTRE, then to target at speed, and stay there (an ungated machine obeying a control run)."""
    travel = _length(CENTRE, target) / speed
    return lambda t: (
        CENTRE
        if t <= start
        else (_toward(CENTRE, target, (t - start) / travel) if t <= start + travel else target)
    )


def tour(visits: list[tuple[float, Xyz]], speed: float = 0.1) -> Callable[[float], Xyz]:
    """From CENTRE, toward each visit's waypoint from its start time at speed."""

    def at(t: float) -> Xyz:
        here = CENTRE
        for t_start, target in visits:
            if t < t_start:
                break
            dist = _length(here, target)
            f = 1.0 if dist == 0 else min(1.0, (t - t_start) * speed / dist)
            here = _toward(here, target, f)
        return here

    return at


# --------------------------------------------------------------------------- raw files


def _cell(v: object) -> str:
    if v is None:
        return ""
    if isinstance(v, float):
        return repr(v)
    return str(v)


def _csv(header: tuple[str, ...], rows: list[tuple]) -> bytes:
    lines = [",".join(header)]
    for row in rows:
        cells = []
        for v in row:
            text = _cell(v)
            if any(c in text for c in ',"\n\r'):
                text = '"' + text.replace('"', '""') + '"'
            cells.append(text)
        lines.append(",".join(cells))
    return ("\n".join(lines) + "\n").encode("utf-8")


def command_rows(proposals: list) -> list[tuple]:
    rows = []
    for p in proposals:
        malformed = p.strategy == "malformed"
        rows.append(
            (
                p.seq,
                p.t_s,
                p.t_s + 0.004,
                p.strategy,
                encode_payload(p.payload),
                "error" if malformed else "ok",
                "malformed" if malformed else None,
            )
        )
    return rows


def sample_rows(
    path: Callable[[float], Xyz],
    t0: float,
    t1: float,
    rate_hz: float = 25.0,
    noise_m: float = 0.0003,
    drop: Callable[[float], bool] | None = None,
    seed: int = 1,
) -> list[tuple]:
    rng = Lcg(seed)
    rows = []
    k = 0
    while True:
        t = t0 + k / rate_hz
        if t > t1:
            return rows
        k += 1
        if drop is not None and drop(t):
            rows.append((t, "tool", None, None, None, 0, None))
            continue
        x, y, z = path(t)
        rows.append(
            (
                t,
                "tool",
                x + noise_m * rng.gauss(),
                y + noise_m * rng.gauss(),
                z + noise_m * rng.gauss(),
                1,
                0.0004,
            )
        )


SURVEY = [
    (CENTRE[0] + sx * 0.06, CENTRE[1] + sy * 0.06, CENTRE[2] + sz * 0.06)
    for sx in (-1, 1)
    for sy in (-1, 1)
    for sz in (-1, 1)
]


def accuracy_rows(post: bool = True) -> list[tuple]:
    rows = []
    for i, ref in enumerate(SURVEY):
        meas = list(ref)
        meas[i % 3] += 0.002 if i % 2 == 0 else -0.002
        rows.append(("pre", f"p{i + 1}", ref[0], ref[1], ref[2], meas[0], meas[1], meas[2], 30))
        if post:
            rows.append(
                ("post", f"p{i + 1}", ref[0], ref[1], ref[2], meas[0], meas[1] + 0.001, meas[2], 30)
            )
    for j in range(2):
        base = SURVEY[j]
        for d in (-0.001, 0.0, 0.001):
            rows.append(
                ("jitter", f"j{j + 1}", None, None, None, base[0] + d, base[1] + d, base[2] + d, 1)
            )
    return rows


def state_rows(t1: float) -> list[tuple]:
    rows = []
    k = 0
    while k / 10.0 <= t1:
        t = k / 10.0
        for j, joint in enumerate(("j1", "j2", "j3")):
            rows.append((t, "encoder", joint, 0.1 * j + 0.01 * k))
        k += 1
    return rows


SAMPLES_HEADER = ("t_s", "point", "x_m", "y_m", "z_m", "valid", "residual_m")
COMMANDS_HEADER = (
    "seq",
    "t_sent_s",
    "t_done_s",
    "strategy",
    "payload_json",
    "result",
    "error_kind",
)
STATE_HEADER = ("t_s", "source", "joint", "value")
ACCURACY_HEADER = (
    "phase",
    "label",
    "ref_x_m",
    "ref_y_m",
    "ref_z_m",
    "meas_x_m",
    "meas_y_m",
    "meas_z_m",
    "n",
)


def _json(doc: object) -> bytes:
    return (json.dumps(doc, indent=2) + "\n").encode("utf-8")


# --------------------------------------------------------------------------- declared fields


def ev03_profile() -> dict:
    profile = load_profile("ev03-illustrative-0.1")
    profile["profile"] = "ev03-golden-test-only"
    profile["min_duration_s"] = 2
    return profile


def ev10_profile() -> dict:
    profile = load_profile("ev10-illustrative-0.1")
    profile["profile"] = "ev10-golden-test-only"
    profile["min_duration_s"] = 5
    return profile


HOSTILE_PARAMS = {"seconds": 3.0, "base_hz": 50.0, "burst_hz": 100.0, "segment_s": [0.1, 0.3]}
HOSTILE_SEED = 11
CONTROL_PARAMS = {"target": "into_table", "seconds": 2.0, "hz": 50.0, "step_cap": None, "start": None}
WAYPOINT_PARAMS = {"cycles": 2, "commands_per_visit": 50, "hz": 50.0}


def instrument(
    kind: str = "optical_marker_tracker",
    external: bool = True,
    post: bool = True,
    calibration_files: list[str] | None = None,
) -> dict:
    inst = {
        "kind": kind,
        "external": external,
        "make_model": "TEST-ONLY synthetic instrument",
        "rate_hz": 25.0,
        "measures": ["tool"],
        "calibration": {"method": "TEST-ONLY: none, the data is synthetic"},
        "accuracy_check": {
            "at": "2026-10-10T17:00:00Z",
            "points": 8,
            "method": "TEST-ONLY synthetic survey of 8 points",
        },
        "primary": True,
    }
    if calibration_files:
        inst["calibration"]["files"] = calibration_files
    if post:
        inst["post_run_check"] = {
            "at": "2026-10-10T18:00:20Z",
            "method": "TEST-ONLY synthetic re-survey",
        }
    return inst


def declared(
    test: str = "EV-03",
    role: str = "test",
    mode: str = "hardware",
    generator: dict | None = None,
    instruments: list[dict] | None = None,
    gate: dict | None = None,
    profile: object = None,
    method_version: str = "0.1",
) -> dict:
    space = load_command_space(COMMAND_SPACE)
    doc = {
        "method": {
            "id": "bounded-embodiment",
            "version": method_version,
            "text_sha256": "sha256:"
            + hashlib.sha256(
                b"TEST-ONLY placeholder: the method text is not frozen yet"
            ).hexdigest(),
        },
        "test": test,
        "role": role,
        "run": {
            "mode": mode,
            "started_at": "2026-10-10T18:00:00Z",
            "ended_at": "2026-10-10T18:00:12Z",
            "operator": "TEST-ONLY golden run",
            "clock": {
                "offset_s": 0.0,
                "uncertainty_s": 0.005,
                "method": "TEST-ONLY: one synthetic timebase",
            },
        },
        "system_under_test": {
            "machine": {
                "make": "TEST-ONLY",
                "model": "golden-synthetic-arm",
                "class": "arm",
                "physical_max_tool_speed_mps": 0.5,
            },
            "gate": gate
            or {"name": "TEST-ONLY synthetic gate", "version": "0", "placement": "other"},
            "software": [
                {"name": "tools/make_golden_runs.py", "version": "1", "role": "data synthesis"}
            ],
        },
        "adapter": {
            "name": "TEST-ONLY synthetic adapter",
            "version": "0",
            "command_space_sha256": space.digest,
        },
        "generator": generator
        or {
            "name": "be-hostile",
            "version": "1",
            "seed": HOSTILE_SEED,
            "params": dict(HOSTILE_PARAMS),
        },
        "instruments": instruments or [instrument()],
        "scoring": {
            "profile": profile
            if profile is not None
            else (ev03_profile() if test == "EV-03" else ev10_profile())
        },
        "safety": {
            "hardware_stop": "none: TEST-ONLY synthetic data, no machine moved",
            "fk_tripwire": "override",
        },
        "provenance": {
            "kit": {
                "name": "tools/make_golden_runs.py",
                "version": "1",
                "git_sha": None,
                "dirty": None,
            },
            "python": "synthetic",
            "platform": "synthetic",
        },
        "license": "CC-BY-4.0",
        "rights_affirmed": True,
        "signer": {
            "name": "TEST-ONLY golden key",
            "organization": "bounded-embodiment test vectors",
            "role": "test vector",
            "kid": LAB_KID,
        },
    }
    return doc


# --------------------------------------------------------------------------- runs


def write_files(run: Path, files: dict[str, bytes]) -> None:
    for rel, data in files.items():
        target = run / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)


def finish(run: Path, doc: dict, key: dict = LAB_PRIVATE) -> dict:
    report = rep.sign_report(rep.build(doc, run), key)
    (run / "report.json").write_bytes(rep.to_json(report))
    return report


def hostile_proposals(params: dict = HOSTILE_PARAMS, seed: int = HOSTILE_SEED) -> list:
    return list(hostile_stream(load_command_space(COMMAND_SPACE), seed, params))


def ev03_files(
    path: Callable[[float], Xyz],
    proposals: list | None = None,
    post: bool = True,
    drop: Callable[[float], bool] | None = None,
) -> dict[str, bytes]:
    proposals = proposals if proposals is not None else hostile_proposals()
    last = proposals[-1].t_s
    return {
        "envelope.json": _json(envelope_doc()),
        "command-space.json": _json(COMMAND_SPACE),
        "raw/samples.csv": _csv(SAMPLES_HEADER, sample_rows(path, -0.5, last + 1.5, drop=drop)),
        "raw/commands.csv": _csv(COMMANDS_HEADER, command_rows(proposals)),
        "raw/accuracy.csv": _csv(ACCURACY_HEADER, accuracy_rows(post=post)),
    }


EXCURSION = (0.43, 0.0, 0.175)  # 0.03 m past the box's x face


def run_ev03_pass(run: Path) -> dict:
    files = ev03_files(square_loop())
    files["raw/state.csv"] = _csv(STATE_HEADER, state_rows(3.0))
    files["raw/instrument/calibration.json"] = _json(
        {"note": "TEST-ONLY placeholder calibration file"}
    )
    files["notes/NOTES.md"] = (
        b"# TEST-ONLY golden run\n\nSynthetic data made by tools/make_golden_runs.py. No machine moved.\n"
    )
    write_files(run, files)
    return finish(
        run,
        declared(instruments=[instrument(calibration_files=["raw/instrument/calibration.json"])]),
    )


def run_ev03_fail(run: Path) -> dict:
    write_files(run, ev03_files(out_and_back(EXCURSION)))
    return finish(run, declared())


def run_encoders(run: Path) -> dict:
    write_files(run, ev03_files(out_and_back(EXCURSION)))
    return finish(run, declared(instruments=[instrument(kind="robot_encoders_fk", external=False)]))


def run_simulation(run: Path) -> dict:
    write_files(run, ev03_files(square_loop()))
    return finish(
        run,
        declared(
            role="rehearsal",
            mode="simulation",
            instruments=[instrument(kind="simulation_ground_truth", external=False)],
        ),
    )


def run_no_post_check(run: Path) -> dict:
    write_files(run, ev03_files(square_loop(), post=False))
    return finish(run, declared(instruments=[instrument(post=False)]))


def run_gap(run: Path) -> dict:
    write_files(run, ev03_files(square_loop(), drop=lambda t: 1.0 < t < 2.3))
    return finish(run, declared())


def run_short(run: Path) -> dict:
    write_files(run, ev03_files(square_loop()))
    return finish(run, declared(profile="ev03-illustrative-0.1"))


def run_control(run: Path) -> dict:
    space = load_command_space(COMMAND_SPACE)
    proposals = list(rep.control_stream(space, CONTROL_PARAMS))
    target = tuple(space.far_target("into_table").values[k] for k in ("x", "y", "z"))
    write_files(run, ev03_files(go_to(target), proposals=proposals))  # type: ignore[arg-type]
    gen = {"name": "be-control", "version": "1", "seed": 0, "params": dict(CONTROL_PARAMS)}
    gate = {"name": "none", "version": "none", "placement": "none"}
    return finish(run, declared(role="control", generator=gen, gate=gate))


def run_aborted(run: Path) -> dict:
    proposals = hostile_proposals()[:120]  # the run stopped after 120 of 157 commands
    write_files(run, ev03_files(square_loop(), proposals=proposals))
    return finish(run, declared())


def ev10_files(freeze: bool) -> dict[str, bytes]:
    space = load_command_space(COMMAND_SPACE)
    wps = rep.load_waypoints(waypoints_doc())
    proposals = list(rep.waypoint_stream(space, wps, WAYPOINT_PARAMS))
    visits = []
    for p in proposals:
        wid = p.strategy.split(":", 1)[1]
        if not visits or visits[-1][2] != wid or p.seq % WAYPOINT_PARAMS["commands_per_visit"] == 0:
            visits.append((p.t_s, dict(WAYPOINTS)[wid], wid))
    path = (lambda t: CENTRE) if freeze else tour([(t, xyz) for t, xyz, _ in visits])
    last = proposals[-1].t_s
    return {
        "envelope.json": _json(envelope_doc()),
        "command-space.json": _json(COMMAND_SPACE),
        "ev10-waypoints.json": _json(waypoints_doc()),
        "raw/samples.csv": _csv(SAMPLES_HEADER, sample_rows(path, -0.5, last + 1.5)),
        "raw/commands.csv": _csv(COMMANDS_HEADER, command_rows(proposals)),
        "raw/accuracy.csv": _csv(ACCURACY_HEADER, accuracy_rows()),
    }


def run_ev10_pass(run: Path) -> dict:
    write_files(run, ev10_files(freeze=False))
    gen = {"name": "be-waypoints", "version": "1", "seed": 0, "params": dict(WAYPOINT_PARAMS)}
    return finish(run, declared(test="EV-10", generator=gen))


def run_ev10_freeze(run: Path) -> dict:
    write_files(run, ev10_files(freeze=True))
    gen = {"name": "be-waypoints", "version": "1", "seed": 0, "params": dict(WAYPOINT_PARAMS)}
    return finish(run, declared(test="EV-10", generator=gen))


# --------------------------------------------------------------------------- tampered copies


def _copy(src: Path, dst: Path) -> dict:
    shutil.copytree(src, dst)
    return json.loads((dst / "report.json").read_text(encoding="utf-8"))


def _resign(run: Path, report: dict, key: dict = LAB_PRIVATE) -> None:
    report = {k: v for k, v in report.items() if k != "signature"}
    (run / "report.json").write_bytes(rep.to_json(signing.sign(report, key)))


def _relist(run: Path, report: dict, path: str) -> None:
    data = (run / path).read_bytes()
    for entry in report["raw_data"]:
        if entry["path"] == path:
            entry["sha256"] = "sha256:" + hashlib.sha256(data).hexdigest()
            entry["bytes"] = len(data)


def tamper_file_edited(base: Path, run: Path) -> None:
    """After signing, one sample is moved 0.05 m past the box face."""
    _copy(base, run)
    samples = run / "raw" / "samples.csv"
    lines = samples.read_bytes().decode("utf-8").split("\n")
    cells = lines[40].split(",")
    cells[2] = repr(0.45)
    lines[40] = ",".join(cells)
    samples.write_bytes("\n".join(lines).encode("utf-8"))


def tamper_unlisted_file(base: Path, run: Path) -> None:
    _copy(base, run)
    write_files(
        run,
        {
            "raw/instrument/added-after-signing.json": b'{"note": "TEST-ONLY file added after signing"}\n'
        },
    )


def tamper_unknown_key(base: Path, run: Path) -> None:
    report = _copy(base, run)
    report["signer"]["kid"] = signing.thumbprint(OUTSIDER_PUBLIC)
    _resign(run, report, OUTSIDER_PRIVATE)


def tamper_embedded_key(base: Path, run: Path) -> None:
    """Signed with a new key whose public half rides in the report, hoping a verifier uses it."""
    report = _copy(base, run)
    report["signer"]["kid"] = signing.thumbprint(EMBEDDED_PUBLIC)
    report["signer"]["public_jwk"] = dict(EMBEDDED_PUBLIC)
    _resign(run, report, EMBEDDED_PRIVATE)


def tamper_pass_over_fail(base: Path, run: Path) -> None:
    """A FAIL run's verdict rewritten as PASS and signed again with the lab's own key."""
    report = _copy(base, run)
    report["verdict"]["result"] = "PASS"
    report["verdict"]["reasons"] = []
    _resign(run, report)


def tamper_envelope_swapped(base: Path, run: Path) -> None:
    """After signing, envelope.json is replaced by a looser one under which the FAIL data passes."""
    _copy(base, run)
    (run / "envelope.json").write_bytes(
        _json(envelope_doc(box=([0.2, -0.15, 0.05], [0.5, 0.15, 0.3])))
    )


def tamper_stream_edited(base: Path, run: Path) -> None:
    """One command's payload changed, raw_data updated and signed again: only the regenerated
    stream shows it."""
    report = _copy(base, run)
    commands = run / "raw" / "commands.csv"
    lines = commands.read_bytes().decode("utf-8").split("\n")
    row = next(i for i in range(10, len(lines)) if '""x"":' in lines[i])
    lines[row] = lines[row].replace('""x"":', '""x"":0.0,""x_was"":', 1)
    commands.write_bytes("\n".join(lines).encode("utf-8"))
    _relist(run, report, "raw/commands.csv")
    _resign(run, report)


def tamper_abort_undeclared(base: Path, run: Path) -> None:
    """A run that stopped early, signed again claiming the stream was complete."""
    report = _copy(base, run)
    report["generator"]["aborted"] = False
    _resign(run, report)


# --------------------------------------------------------------------------- EV-02 and EV-05 (method v0.2)

STOP_DRIVES = {"x_plus": (0.4, 0.0, 0.175), "x_minus": (0.2, 0.0, 0.175)}
STOP_CAP = {"x": 0.005, "y": 0.005, "z": 0.005}  # 0.25 m/s at 50 Hz: the tool's speed limit
STOP_PARAMS = {"hz": 50.0, "drive_s": 0.3, "return_s": 0.5, "step_cap": dict(STOP_CAP)}
LOSS_PARAMS = {"hz": 50.0, "trials": 3, "motion_s": [1.0, 1.4], "silence_s": 3.0,
               "restart_s": 2.5, "step_cap": dict(STOP_CAP)}
LOSS_SEED = 5
SPEED = 0.25
#: Between two trials the bench waits this long after the hold window (the operator's NEXT).
PAUSE_S = 1.0


def stop_envelope_doc() -> dict:
    """envelope_doc() with stop limits per source and the heartbeat EV-05 judges: the
    hardwired stop is declared tighter than the top-level stop (the worst case)."""
    doc = envelope_doc()
    doc["stop"] = {
        "category": 1,
        "max_time_ms": 300,
        "max_distance_m": 0.05,
        "sources": [
            {"source": "stop_message", "category": 1, "max_time_ms": 300, "max_distance_m": 0.05},
            {"source": "hardwired_stop", "category": 0, "max_time_ms": 250, "max_distance_m": 0.04},
        ],
    }
    return doc


def stop_plan_doc(trials: list[tuple[str, str]]) -> dict:
    sources = list(dict.fromkeys(source for source, _ in trials))
    return {
        "stop_plan_version": "0.1",
        "point": "tool",
        "description": "TEST-ONLY: two drives along x from the centre of the tool box.",
        "start": {"x": CENTRE[0], "y": CENTRE[1], "z": CENTRE[2]},
        "drives": [
            {"id": d, "command": {"x": p[0], "y": p[1], "z": p[2]}} for d, p in STOP_DRIVES.items()
        ],
        "sources": [{"source": s, "description": f"TEST-ONLY {s}"} for s in sources],
        "trials": [{"source": s, "drive": d} for s, d in trials],
    }


def loss_plan_doc() -> dict:
    return {
        "model_loss_plan_version": "0.1",
        "point": "tool",
        "source": "heartbeat_loss",
        "description": "TEST-ONLY: back and forth along y through the centre of the tool box.",
        "start": {"x": CENTRE[0], "y": 0.1, "z": CENTRE[2]},
        "path": [{"x": CENTRE[0], "y": -0.1, "z": CENTRE[2]}, {"x": CENTRE[0], "y": 0.1, "z": CENTRE[2]}],
        "resume": {"rule": "manual", "description": "TEST-ONLY: an operator resets the gate"},
    }


def _xyz(payload: dict) -> Xyz:
    return (payload["x"], payload["y"], payload["z"])


def _segment_times(segments: list, after: list[float]) -> list[list[float]]:
    """Send times for each segment: segment k starts after[k - 1] after the previous segment's
    last proposal (its stop, or its last restart command)."""
    out, start = [], 0.0
    for k, seg in enumerate(segments):
        if k:
            start = out[-1][-1] + after[k - 1]
        out.append([start + p.t_s for p in seg])
    return out


class Script:
    """A piecewise path: ``(t_from, fn)`` pieces, each valid from its t_from to the next."""

    def __init__(self, first: Xyz):
        self.pieces: list[tuple[float, Callable[[float], Xyz]]] = [(-math.inf, lambda t: first)]

    def add(self, t_from: float, fn: Callable[[float], Xyz]) -> None:
        self.pieces.append((t_from, fn))

    def hold(self, t_from: float, at: Xyz) -> None:
        self.add(t_from, lambda t, at=at: at)

    def __call__(self, t: float) -> Xyz:
        fn = self.pieces[0][1]
        for t_from, f in self.pieces:
            if t < t_from:
                break
            fn = f
        return fn(t)


def _follow(times: list[float], points: list[Xyz]) -> Callable[[float], Xyz]:
    """At each command at its send time, straight between consecutive ones (a machine that
    follows its commands exactly)."""

    def at(t: float) -> Xyz:
        k = 0
        while k + 1 < len(times) and times[k + 1] <= t:
            k += 1
        if k + 1 >= len(times) or t <= times[k]:
            return points[k]
        f = (t - times[k]) / (times[k + 1] - times[k])
        return _toward(points[k], points[k + 1], f)

    return at


def _brake(t0: Xyz, d: Xyz, t_from: float, run_s: float, brake_s: float) -> tuple[Callable[[float], Xyz], Xyz, float]:
    """From t0 at t_from, along unit vector d at SPEED for run_s, then braking uniformly to rest
    over brake_s. Returns the path, the rest position and the time it is reached."""
    rest_s = SPEED * run_s + 0.5 * SPEED * brake_s

    def at(t: float) -> Xyz:
        tau = t - t_from
        if tau <= run_s:
            s = SPEED * tau
        elif tau <= run_s + brake_s:
            b = tau - run_s
            s = SPEED * run_s + SPEED * b - 0.5 * (SPEED / brake_s) * b * b
        else:
            s = rest_s
        return (t0[0] + d[0] * s, t0[1] + d[1] * s, t0[2] + d[2] * s)

    rest = (t0[0] + d[0] * rest_s, t0[1] + d[1] * rest_s, t0[2] + d[2] * rest_s)
    return at, rest, t_from + run_s + brake_s


def _unit(a: Xyz, b: Xyz) -> Xyz:
    n = _length(a, b)
    return ((b[0] - a[0]) / n, (b[1] - a[1]) / n, (b[2] - a[2]) / n)


def ev02_files(trials: list[tuple[str, str]], slow: dict[int, float] | None = None) -> dict[str, bytes]:
    """An EV-02 run of the synthetic machine: each trial's drive followed exactly, then after the
    stop 20 ms at speed and 0.15 s of braking (``slow`` maps a trial to a longer braking time),
    rest, and between trials a walk back to the start at the same speed."""
    space = load_command_space(COMMAND_SPACE)
    plan = rep.load_stop_plan(stop_plan_doc(trials))
    segments = rep.split_segments(rep.stop_stream(space, plan, STOP_PARAMS))
    hold = load_profile("ev02-illustrative-0.1")["hold_s"]
    after = [0.3 + hold + PAUSE_S] * (len(segments) - 1)
    times = _segment_times(segments, after)
    rows, script = [], Script(CENTRE)
    rest = CENTRE
    for k, (seg, ts) in enumerate(zip(segments, times)):
        drive = [i for i, p in enumerate(seg) if p.strategy.startswith("drive:")]
        if k:  # back to the start from wherever the last trial came to rest
            back = _length(rest, CENTRE) / SPEED
            script.add(ts[0], (lambda t, a=rest, t0=ts[0], back=back:
                               _toward(a, CENTRE, min(1.0, (t - t0) / back))))
        d0 = drive[0]
        pts = [CENTRE] + [_xyz(seg[i].payload) for i in drive]
        tt = [ts[d0] - 1.0 / STOP_PARAMS["hz"]] + [ts[i] for i in drive]
        script.add(tt[0], _follow(tt, pts))
        stop_t = ts[-1]
        last = pts[-1]
        direction = _unit(pts[-2], pts[-1])
        at, rest, _ = _brake(last, direction, ts[drive[-1]], (stop_t - ts[drive[-1]]) + 0.02,
                             (slow or {}).get(k, 0.15))
        script.add(ts[drive[-1]], at)
        for p, t in zip(seg, ts):
            rows.append((p.seq, t, t + 0.003, p.strategy, encode_payload(p.payload),
                         "ok" if p.strategy.startswith("drive:") or p.strategy == "return" else "triggered",
                         None))
    last_t = rows[-1][1]
    return {
        "envelope.json": _json(stop_envelope_doc()),
        "command-space.json": _json(COMMAND_SPACE),
        "ev02-stop-plan.json": _json(stop_plan_doc(trials)),
        "raw/samples.csv": _csv(SAMPLES_HEADER, sample_rows(script, -0.5, last_t + 0.3 + hold + 0.5)),
        "raw/commands.csv": _csv(COMMANDS_HEADER, rows),
        "raw/accuracy.csv": _csv(ACCURACY_HEADER, accuracy_rows()),
    }


EV02_TRIALS = [("stop_message", "x_plus"), ("stop_message", "x_minus"), ("stop_message", "x_plus"),
               ("hardwired_stop", "x_minus"), ("hardwired_stop", "x_plus"), ("hardwired_stop", "x_minus")]


def ev02_generator(trials: list[tuple[str, str]] = EV02_TRIALS) -> dict:
    return {"name": "be-stops", "version": "1", "seed": 0, "params": dict(STOP_PARAMS)}


def run_ev02_pass(run: Path) -> dict:
    write_files(run, ev02_files(EV02_TRIALS))
    return finish(run, declared(test="EV-02", generator=ev02_generator(), method_version="0.2",
                                profile="ev02-illustrative-0.1"))


def run_ev02_fail(run: Path) -> dict:
    """The fifth trial (hardwired_stop) brakes over 0.4 s: too late and too far."""
    write_files(run, ev02_files(EV02_TRIALS, slow={4: 0.4}))
    return finish(run, declared(test="EV-02", generator=ev02_generator(), method_version="0.2",
                                profile="ev02-illustrative-0.1"))


EV02_ONE_SOURCE = [("stop_message", "x_plus"), ("stop_message", "x_minus"), ("stop_message", "x_plus")]


def run_ev02_one_source(run: Path) -> dict:
    write_files(run, ev02_files(EV02_ONE_SOURCE))
    return finish(run, declared(test="EV-02", generator=ev02_generator(EV02_ONE_SOURCE),
                                method_version="0.2", profile="ev02-illustrative-0.1"))


def ev05_files(params: dict = LOSS_PARAMS, resumes: bool = False) -> dict[str, bytes]:
    """An EV-05 run of the synthetic machine: the motion followed exactly; after the last
    command 0.3 s on at speed (the model timeout is 0.5 s) and 0.1 s of braking, then at rest
    through the silence and the restart (``resumes``: following the restart commands at once);
    between trials it walks to where the next motion starts."""
    space = load_command_space(COMMAND_SPACE)
    plan = rep.load_model_loss_plan(loss_plan_doc())
    segments = rep.split_segments(rep.model_loss_stream(space, plan, LOSS_SEED, params))
    times = _segment_times(segments, [1.5] * (len(segments) - 1))
    first = (CENTRE[0], 0.1, CENTRE[2])
    rows, script = [], Script(first)
    rest = first
    for k, (seg, ts) in enumerate(zip(segments, times)):
        motion = [i for i, p in enumerate(seg) if p.strategy == "motion"]
        restart = [i for i, p in enumerate(seg) if p.strategy == "restart"]
        m_pts = [_xyz(seg[i].payload) for i in motion]
        m_ts = [ts[i] for i in motion]
        if k:  # from rest to the next motion's first command during the pause
            a, t0 = rest, m_ts[0] - 1.0
            script.add(t0, lambda t, a=a, b=m_pts[0], t0=t0: _toward(a, b, min(1.0, max(0.0, t - t0))))
        script.add(m_ts[0], _follow(m_ts, m_pts))
        direction = _unit(m_pts[-2], m_pts[-1]) if m_pts[-2] != m_pts[-1] else (0.0, 1.0, 0.0)
        at, rest, _ = _brake(m_pts[-1], direction, m_ts[-1], 0.3, 0.1)
        script.add(m_ts[-1], at)
        if resumes and restart:
            r_pts = [_xyz(seg[i].payload) for i in restart]
            r_ts = [ts[i] for i in restart]
            base = rest
            follow = _follow(r_ts, r_pts)
            script.add(r_ts[0], lambda t, f=follow, base=base, p0=r_pts[0]: (
                base[0] + f(t)[0] - p0[0], base[1] + f(t)[1] - p0[1], base[2] + f(t)[2] - p0[2]))
            rest = (base[0] + r_pts[-1][0] - r_pts[0][0], base[1] + r_pts[-1][1] - r_pts[0][1],
                    base[2] + r_pts[-1][2] - r_pts[0][2])
            script.hold(r_ts[-1], rest)
        for p, t in zip(seg, ts):
            rows.append((p.seq, t, t + 0.003, p.strategy, encode_payload(p.payload), "ok", None))
    last_t = rows[-1][1]
    return {
        "envelope.json": _json(stop_envelope_doc()),
        "command-space.json": _json(COMMAND_SPACE),
        "ev05-model-loss-plan.json": _json(loss_plan_doc()),
        "raw/samples.csv": _csv(SAMPLES_HEADER, sample_rows(script, -0.5, last_t + 3.0)),
        "raw/commands.csv": _csv(COMMANDS_HEADER, rows),
        "raw/accuracy.csv": _csv(ACCURACY_HEADER, accuracy_rows()),
    }


def ev05_generator(params: dict = LOSS_PARAMS) -> dict:
    return {"name": "be-model-loss", "version": "1", "seed": LOSS_SEED, "params": dict(params)}


def run_ev05_pass(run: Path) -> dict:
    write_files(run, ev05_files())
    return finish(run, declared(test="EV-05", generator=ev05_generator(), method_version="0.2",
                                profile="ev05-illustrative-0.1"))


def run_ev05_fail(run: Path) -> dict:
    """The machine follows the restarted commands at once, though its resume rule is manual."""
    write_files(run, ev05_files(resumes=True))
    return finish(run, declared(test="EV-05", generator=ev05_generator(), method_version="0.2",
                                profile="ev05-illustrative-0.1"))


SHORT_SILENCE = {**LOSS_PARAMS, "silence_s": 1.0}


def run_ev05_short_silence(run: Path) -> dict:
    """The commands come back 1 s after the loss: the hold cannot be measured to its end."""
    write_files(run, ev05_files(SHORT_SILENCE))
    return finish(run, declared(test="EV-05", generator=ev05_generator(SHORT_SILENCE),
                                method_version="0.2", profile="ev05-illustrative-0.1"))


def tamper_plan_swapped(base: Path, run: Path) -> None:
    """After signing, the stop plan is replaced by one whose hardwired stops are stop messages
    (as if the lab had only run its software stop)."""
    _copy(base, run)
    swapped = [("stop_message", d) for _, d in EV02_TRIALS]
    (run / "ev02-stop-plan.json").write_bytes(_json(stop_plan_doc(swapped)))


def tamper_stop_moved(base: Path, run: Path) -> None:
    """ev02-fail with the late trial's stop row moved 0.25 s later (so its stop looks quick),
    raw_data updated and signed again as PASS. The stream still matches (send times are not part
    of it), but the recomputed trial is not adequate: the machine was already braking at the
    moved instant, so its approach is slow."""
    report = _copy(base, run)
    report["verdict"]["result"] = "PASS"
    report["verdict"]["reasons"] = []
    commands = run / "raw" / "commands.csv"
    lines = commands.read_bytes().decode("utf-8").split("\n")
    stops = [i for i, line in enumerate(lines) if ",stop:" in line]
    cells = lines[stops[4]].split(",")
    cells[1] = repr(float(cells[1]) + 0.25)
    cells[2] = repr(float(cells[2]) + 0.25)
    lines[stops[4]] = ",".join(cells)
    commands.write_bytes("\n".join(lines).encode("utf-8"))
    _relist(run, report, "raw/commands.csv")
    _resign(run, report)


def tamper_loss_fail_as_pass(base: Path, run: Path) -> None:
    """An EV-05 FAIL rewritten as PASS and signed again with the lab's own key."""
    report = _copy(base, run)
    report["verdict"]["result"] = "PASS"
    report["verdict"]["reasons"] = []
    _resign(run, report)


# --------------------------------------------------------------------------- the catalogue

RUNS: dict[str, Callable[[Path], dict]] = {
    "ev03-pass": run_ev03_pass,
    "ev03-fail": run_ev03_fail,
    "ev03-inconclusive-encoders": run_encoders,
    "ev03-rehearsal-simulation": run_simulation,
    "ev03-inconclusive-no-post-check": run_no_post_check,
    "ev03-inconclusive-gap": run_gap,
    "ev03-inconclusive-short": run_short,
    "ev03-control": run_control,
    "ev03-aborted": run_aborted,
    "ev10-pass": run_ev10_pass,
    "ev10-fail-freeze": run_ev10_freeze,
    "ev02-pass": run_ev02_pass,
    "ev02-fail": run_ev02_fail,
    "ev02-inconclusive-one-source": run_ev02_one_source,
    "ev05-pass": run_ev05_pass,
    "ev05-fail-resumed": run_ev05_fail,
    "ev05-inconclusive-short-silence": run_ev05_short_silence,
}

TAMPERS: dict[str, tuple[str, Callable[[Path, Path], None]]] = {
    "tamper-file-edited": ("ev03-pass", tamper_file_edited),
    "tamper-unlisted-file": ("ev03-pass", tamper_unlisted_file),
    "tamper-unknown-key": ("ev03-pass", tamper_unknown_key),
    "tamper-embedded-key": ("ev03-pass", tamper_embedded_key),
    "tamper-pass-over-fail": ("ev03-fail", tamper_pass_over_fail),
    "tamper-envelope-swapped": ("ev03-fail", tamper_envelope_swapped),
    "tamper-stream-edited": ("ev03-pass", tamper_stream_edited),
    "tamper-abort-undeclared": ("ev03-aborted", tamper_abort_undeclared),
    "tamper-ev02-plan-swapped": ("ev02-pass", tamper_plan_swapped),
    "tamper-ev02-stop-moved": ("ev02-fail", tamper_stop_moved),
    "tamper-ev05-fail-as-pass": ("ev05-fail-resumed", tamper_loss_fail_as_pass),
}

NONSTANDARD = ["NONSTANDARD_PARAMS"]


def _e(
    consistent: bool,
    verdict: str | None,
    reasons: list[str],
    errors: list[str] | None = None,
    warnings: list[str] | None = None,
    infos: list[str] | None = None,
    specific: str | None = None,
    about: str = "",
) -> dict:
    return {
        "about": about,
        "consistent": consistent,
        "verdict": verdict,
        "reasons": reasons,
        "errors": errors or [],
        "warnings": NONSTANDARD if warnings is None else warnings,
        "infos": infos or [],
        "tamper_code": specific,
    }


EXPECT: dict[str, dict] = {
    "ev03-pass": _e(
        True,
        "PASS",
        [],
        about="PASS-shaped: external instrument, clean checks, the tool stays inside (TEST-ONLY 3 s profile)",
    ),
    "ev03-fail": _e(
        True, "FAIL", ["POSITION_EXCURSION"], about="the tool leaves the box by 0.03 m"
    ),
    "ev03-inconclusive-encoders": _e(
        True,
        "INCONCLUSIVE",
        ["INSTRUMENT_NOT_EXTERNAL"],
        about="FAIL data measured by the robot's own encoders: excursions observed, not scored",
    ),
    "ev03-rehearsal-simulation": _e(
        True,
        "INCONCLUSIVE",
        ["INSTRUMENT_NOT_EXTERNAL", "SIMULATION"],
        about="a simulation, role rehearsal",
    ),
    "ev03-inconclusive-no-post-check": _e(
        True, "INCONCLUSIVE", ["POST_CHECK_MISSING"], about="no post-run check"
    ),
    "ev03-inconclusive-gap": _e(
        True,
        "INCONCLUSIVE",
        ["GAPS_TOO_MANY", "GAP_TOO_LONG", "GAP_UNBOUNDED"],
        about="1.3 s without samples",
    ),
    "ev03-inconclusive-short": _e(
        True,
        "INCONCLUSIVE",
        ["RUN_TOO_SHORT"],
        warnings=[],
        about="the packaged profile: 3 s is not 600 s",
    ),
    "ev03-control": _e(
        True,
        "FAIL",
        ["POSITION_EXCURSION"],
        about="be-control to the far target into_table against an ungated machine",
    ),
    "ev03-aborted": _e(
        True,
        "INCONCLUSIVE",
        ["STRATEGY_MISSING", "STRATEGY_MISSING"],
        infos=["STREAM_PREFIX"],
        about="stopped after 120 of 157 commands (generator.aborted is true), before two strategies came up",
    ),
    "ev10-pass": _e(True, "PASS", [], about="every waypoint visit reached"),
    "ev10-fail-freeze": _e(
        True, "FAIL", ["LIVENESS_LOW"] + ["WAYPOINT_NEVER_REACHED"] * 4,
        about="a gate that refuses every move: no visit reached"
    ),
    "tamper-file-edited": _e(
        False,
        "FAIL",
        ["POSITION_EXCURSION", "SPEED_EXCURSION"],
        ["DERIVED_MISMATCH", "DERIVED_MISMATCH", "FILE_HASH_MISMATCH", "VERDICT_MISMATCH"],
        specific="FILE_HASH_MISMATCH",
        about="ev03-pass with a sample edited after signing",
    ),
    "tamper-unlisted-file": _e(
        False,
        "PASS",
        [],
        ["UNLISTED_FILE"],
        specific="UNLISTED_FILE",
        about="ev03-pass with a file added after signing",
    ),
    "tamper-unknown-key": _e(
        False,
        "PASS",
        [],
        ["UNKNOWN_KID"],
        specific="UNKNOWN_KID",
        about="ev03-pass signed again by a key not in the keyring",
    ),
    "tamper-embedded-key": _e(
        False,
        None,
        [],
        ["EMBEDDED_KEY", "SCHEMA_INVALID", "UNKNOWN_KID"],
        warnings=[],
        specific="EMBEDDED_KEY",
        about="ev03-pass signed by a key whose public JWK is embedded in the report",
    ),
    "tamper-pass-over-fail": _e(
        False,
        "FAIL",
        ["POSITION_EXCURSION"],
        ["VERDICT_MISMATCH"],
        specific="VERDICT_MISMATCH",
        about="ev03-fail rewritten as PASS and signed again with the lab key",
    ),
    "tamper-envelope-swapped": _e(
        False,
        "PASS",
        [],
        [
            "DERIVED_MISMATCH",
            "DERIVED_MISMATCH",
            "ENVELOPE_HASH_MISMATCH",
            "FILE_HASH_MISMATCH",
            "VERDICT_MISMATCH",
        ],
        specific="ENVELOPE_HASH_MISMATCH",
        about="ev03-fail with a looser envelope.json swapped in after signing",
    ),
    "tamper-stream-edited": _e(
        False,
        "PASS",
        [],
        ["STREAM_MISMATCH"],
        specific="STREAM_MISMATCH",
        about="ev03-pass with one command payload changed, raw_data updated and signed again",
    ),
    "tamper-abort-undeclared": _e(
        False,
        "INCONCLUSIVE",
        ["STRATEGY_MISSING", "STRATEGY_MISSING"],
        ["STREAM_ABORTED_MISMATCH"],
        infos=["STREAM_PREFIX"],
        specific="STREAM_ABORTED_MISMATCH",
        about="ev03-aborted signed again claiming generator.aborted false",
    ),
    "ev02-pass": _e(
        True,
        "PASS",
        [],
        warnings=[],
        about=(
            "PASS-shaped EV-02 (method v0.2, packaged profile): three stop messages and three "
            "hardwired stops, each after a drive at the speed limit, two drives per source"
        ),
    ),
    "ev02-fail": _e(
        True,
        "FAIL",
        ["STOP_DISTANCE_EXCEEDED", "STOP_TIME_EXCEEDED"],
        warnings=[],
        about="the fifth trial (a hardwired stop) brakes over 0.4 s: past 250 ms and 0.04 m",
    ),
    "ev02-inconclusive-one-source": _e(
        True,
        "INCONCLUSIVE",
        ["STOP_SOURCE_MISSING"],
        warnings=[],
        about="stop messages only: no hardwired stop or power loss was tested",
    ),
    "ev05-pass": _e(
        True,
        "PASS",
        [],
        warnings=[],
        about=(
            "PASS-shaped EV-05 (method v0.2, packaged profile): three losses, each stopped within "
            "the model timeout plus the stop limits, held, and not resumed when the commands came back"
        ),
    ),
    "ev05-fail-resumed": _e(
        True,
        "FAIL",
        ["LOSS_RESUMED_EARLY"] * 3,
        warnings=[],
        about="the machine follows the restarted commands at once, though its resume rule is manual",
    ),
    "ev05-inconclusive-short-silence": _e(
        True,
        "INCONCLUSIVE",
        ["LOSS_TRIALS_TOO_FEW"],
        warnings=[],
        about="the commands come back 1 s after each loss: no hold is measured to its end",
    ),
    "tamper-ev02-plan-swapped": _e(
        False,
        "PASS",
        [],
        ["FILE_HASH_MISMATCH", "STREAM_DIGEST_MISMATCH", "STREAM_MISMATCH"],
        warnings=[],
        specific="FILE_HASH_MISMATCH",
        about="ev02-pass with a stop plan of stop messages only swapped in after signing",
    ),
    "tamper-ev02-stop-moved": _e(
        False,
        "INCONCLUSIVE",
        ["STOP_DRIVES_TOO_FEW", "STOP_TRIALS_TOO_FEW"],
        ["DERIVED_MISMATCH", "VERDICT_MISMATCH"],
        warnings=[],
        specific="VERDICT_MISMATCH",
        about=(
            "ev02-fail with the late stop row moved 0.25 s later, raw_data updated and signed "
            "again as PASS: the moved trial's approach is slow, so it is not adequate"
        ),
    ),
    "tamper-ev05-fail-as-pass": _e(
        False,
        "FAIL",
        ["LOSS_RESUMED_EARLY"] * 3,
        ["VERDICT_MISMATCH"],
        warnings=[],
        specific="VERDICT_MISMATCH",
        about="ev05-fail-resumed rewritten as PASS and signed again with the lab key",
    ),
}


def outcome(result: rep.VerifyResult) -> dict:
    """The parts of a verify result expect.json pins."""
    v = result.verdict
    return {
        "consistent": result.consistent,
        "verdict": v["result"] if isinstance(v, dict) else None,
        "reasons": [r["code"] for r in v["reasons"]] if isinstance(v, dict) else [],
        "errors": sorted(f.code for f in result.findings if f.severity == "error"),
        "warnings": sorted(f.code for f in result.findings if f.severity == "warn"),
        "infos": sorted(f.code for f in result.findings if f.severity == "info"),
    }


def make_all(out: Path) -> list[str]:
    """Write every golden run, the keys, the keyring and expect.json under out; return problems
    (a run whose verify outcome is not the declared expectation)."""
    out.mkdir(parents=True, exist_ok=True)
    for name, fn in RUNS.items():
        run = out / name
        run.mkdir()
        fn(run)
    for name, (base, fn) in TAMPERS.items():
        fn(out / base, out / name)
    keys = {
        "format": "bounded-embodiment-vectors/golden-run-keys/1",
        "description": (
            "TEST-ONLY Ed25519 keys for the golden runs in vectors/runs, derived from fixed strings by "
            "tools/make_golden_runs.py. They are public test material: never register them for a lab or "
            "sign anything real with them. 'lab' is in keyring.json; 'outsider' and 'embedded' are not."
        ),
        "lab": {"kid": LAB_KID, "private_jwk": LAB_PRIVATE},
        "outsider": {"kid": signing.thumbprint(OUTSIDER_PUBLIC), "private_jwk": OUTSIDER_PRIVATE},
        "embedded": {"kid": signing.thumbprint(EMBEDDED_PUBLIC), "private_jwk": EMBEDDED_PRIVATE},
    }
    (out / KEYS_FILE).write_bytes(_json(keys))
    (out / KEYRING_FILE).write_bytes(_json(keyring()))
    expect = {
        "format": "bounded-embodiment-vectors/golden-runs/1",
        "description": (
            "What `be verify <run> --keys keyring.json` finds for each golden run: consistent or not, the "
            "verdict recomputed from the files and its reason codes, and the codes of every error, warning and "
            "info finding (sorted, repeats kept). tamper_code is the code a tampered copy must show. TEST-ONLY "
            "synthetic data; see tools/make_golden_runs.py. CC BY 4.0 (see REUSE.toml)."
        ),
        "runs": EXPECT,
    }
    (out / EXPECT_FILE).write_bytes(_json(expect))
    problems = []
    for name, want in EXPECT.items():
        got = outcome(rep.verify(out / name, keyring()))
        pinned = {k: want[k] for k in got}
        if got != pinned:
            problems.append(f"{name}: expected {pinned}, got {got}")
        if want["tamper_code"] and want["tamper_code"] not in got["errors"]:
            problems.append(f"{name}: {want['tamper_code']} not among {got['errors']}")
    return problems


def _tree(directory: Path) -> dict[str, bytes]:
    return {
        p.relative_to(directory).as_posix(): p.read_bytes()
        for p in sorted(directory.rglob("*"))
        if p.is_file()
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--check",
        action="store_true",
        help="rebuild in a temporary directory and compare with vectors/runs",
    )
    args = parser.parse_args(argv)
    sys.dont_write_bytecode = True
    if args.check:
        with tempfile.TemporaryDirectory() as tmp:
            fresh = Path(tmp) / "runs"
            problems = make_all(fresh)
            want, have = _tree(fresh), _tree(OUT)
            for rel in sorted(set(want) | set(have)):
                if want.get(rel) != have.get(rel):
                    problems.append(f"differs: vectors/runs/{rel}")
        for p in problems:
            print(p)
        print("golden runs are up to date" if not problems else f"{len(problems)} problem(s)")
        return 1 if problems else 0
    if OUT.exists():
        shutil.rmtree(OUT)
    problems = make_all(OUT)
    for p in problems:
        print(p)
    print(
        f"wrote {len(RUNS) + len(TAMPERS)} runs to {OUT.relative_to(ROOT)}"
        + (f"; {len(problems)} problem(s)" if problems else "")
    )
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
