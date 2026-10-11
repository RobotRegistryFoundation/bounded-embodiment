# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""`be report` on run directories made from core's golden runs (no simulation, a few seconds):
what goes in, what is converted and scrubbed, what core derives, and every refusal."""
from __future__ import annotations

import gzip
import json
import sys
from decimal import Decimal
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import _submission_support as sup
from bounded_embodiment import rawdata
from bounded_embodiment import report as core_report
from bounded_embodiment_kit.submission import report as kitreport
from bounded_embodiment_kit.submission.scrub import Identity

FIXED = Identity(hostnames=("benchhost",), usernames=("ana",), homes=("/home/ana",))


@pytest.fixture(autouse=True)
def _fixed_identity(monkeypatch):
    """Scrub for a fixed machine (host "benchhost", user "ana"), whatever runs the tests."""
    monkeypatch.setattr(Identity, "this_machine",
                        classmethod(lambda cls, redact=(): Identity(
                            FIXED.hostnames, FIXED.usernames, FIXED.homes, tuple(redact))))


@pytest.fixture
def key(tmp_path):
    return sup.make_key(tmp_path / "lab.key.json")


def _report(args, logs=None):
    return kitreport.execute_report(args, say=(logs.append if logs is not None else lambda m: None))


def _verify(out, entry):
    return core_report.verify(out, {entry["kid"]: entry["public_jwk"]})


def test_a_hardware_run_is_packaged_converted_signed_and_consistent(tmp_path, key):
    key_path, entry = key
    run = sup.runner_run(tmp_path)
    (tmp_path / "NOTES.md").write_text("Recorded by ana on benchhost (192.168.1.20), files in "
                                       "/home/ana/runs/bob/notes.md.\n")
    logs: list[str] = []
    args = sup.report_args(tmp_path, run, key_path, extra=["--notes", tmp_path / "NOTES.md"])
    assert _report(args, logs) == kitreport.EXIT_OK, logs
    out = tmp_path / "submission"
    result = _verify(out, entry)
    assert result.consistent, result.findings
    rep = json.loads((out / "report.json").read_text())

    # The layout: CSVs gzipped but accuracy, the instrument's files, the timebase record.
    files = sorted(p.relative_to(out).as_posix() for p in out.rglob("*") if p.is_file())
    assert files == ["command-space.json", "envelope.json", "notes/NOTES.md", "raw/accuracy.csv",
                     "raw/commands.csv.gz", "raw/instrument/calibration.json",
                     "raw/instrument/timebase.json", "raw/samples.csv.gz", "raw/state.csv.gz",
                     "report.json"]
    assert not list(tmp_path.glob(".submission.partial-*"))
    # Setup files go in byte for byte; the robot's CSVs are the run's, compressed.
    assert (out / "envelope.json").read_bytes() == (run / "envelope.json").read_bytes()
    assert gzip.decompress((out / "raw/commands.csv.gz").read_bytes()) == \
        (run / "raw/commands.csv").read_bytes()

    # The clock: same-host CLOCK_MONOTONIC, so the offset is minus the runner's monotonic start.
    assert rep["run"]["clock"]["offset_s"] == -sup.MONO_START
    assert rep["run"]["clock"]["uncertainty_s"] == 0.000001
    assert rep["run"]["clock"]["method"].startswith("monotonic:")
    assert rep["run"]["started_at"] == "2026-10-10T18:00:00.000000Z"
    golden = rawdata.read_samples(sup.RUNS / "ev03-pass" / "raw" / "samples.csv")
    converted = rawdata.read_samples(out / "raw" / "samples.csv.gz")
    assert [(s.t_s, s.point, s.x_m, s.y_m, s.z_m, s.valid) for s in converted] == \
        [(s.t_s, s.point, s.x_m, s.y_m, s.z_m, s.valid) for s in golden]

    # Derived members are core's: the same as core's own report from the same files.
    assert rep["verdict"] == result.verdict
    assert rep["statements"]["measurement_source"] == "external_instrument"
    # Declared fields from the run and the lab config.
    assert (rep["role"], rep["test"], rep["run"]["mode"]) == ("test", "EV-03", "hardware")
    assert rep["generator"]["seed"] == 11 and rep["generator"]["name"] == "be-hostile"
    assert rep["signer"]["kid"] == entry["kid"] and rep["signer"]["name"] == "Test Signer"
    assert rep["safety"] == {"hardware_stop": "a relay on servo power, switch by the bench",
                             "fk_tripwire": "override"}
    timebase = json.loads((out / "raw/instrument/timebase.json").read_text())
    assert timebase["method"] == "monotonic" and timebase["samples"]["sources"][0]["rows"] == len(golden)

    # Scrubbed: the notes, not the signer.
    notes = (out / "notes/NOTES.md").read_text()
    assert notes == "Recorded by [user] on [host] ([ip]), files in ~/.../notes.md.\n"
    assert any("notes/NOTES.md: 1 home path, 1 hostname, 1 ip, 1 username" in m for m in logs)


def test_the_same_inputs_give_the_same_bytes(tmp_path, key):
    key_path, _ = key
    run = sup.runner_run(tmp_path)
    for out in ("a", "b"):
        assert _report(sup.report_args(tmp_path, run, key_path, tmp_path / out)) == 0
    files = sorted(p.relative_to(tmp_path / "a") for p in (tmp_path / "a").rglob("*") if p.is_file())
    assert files
    for rel in files:
        assert (tmp_path / "a" / rel).read_bytes() == (tmp_path / "b" / rel).read_bytes(), rel


def test_declared_fields_are_scrubbed_but_ids_and_the_signer_are_not(tmp_path, key):
    key_path, _ = key
    doc = sup.declared_json("ev03-pass")
    doc["safety"]["hardware_stop"] = "relay on benchhost at 10.0.0.7, script /home/ana/estop.sh"
    run = sup.runner_run(tmp_path, declared=doc)
    lab = sup.lab_config(operator="ana", signer={"name": "Ana Smith", "organization": "ana lab",
                                                  "role": "PI"})
    assert _report(sup.report_args(tmp_path, run, key_path, lab=lab)) == 0
    rep = json.loads((tmp_path / "submission/report.json").read_text())
    assert rep["safety"]["hardware_stop"] == "relay on [host] at [ip], script ~/estop.sh"
    assert rep["run"]["operator"] == "[user]"
    assert rep["signer"] == {"name": "Ana Smith", "organization": "ana lab", "role": "PI",
                             "kid": rep["signer"]["kid"]}
    assert rep["instruments"][0]["calibration"]["files"] == ["raw/instrument/calibration.json"]


def test_instrument_files_are_scrubbed_and_photos_lose_their_metadata(tmp_path, key):
    key_path, _ = key
    run = sup.runner_run(tmp_path)
    meta = tmp_path / "fit_meta.json"
    meta.write_text(json.dumps({"session": {"path": "/home/ana/sessions/s1"}, "n": 3}))
    (run / "raw" / "instrument").mkdir()
    (run / "raw" / "instrument" / "notes.txt").write_text("camera on benchhost.local\n")
    exif = b"\xff\xe1" + (len(b"Exif\x00\x00GPS 37.7N") + 2).to_bytes(2, "big") + b"Exif\x00\x00GPS 37.7N"
    jfif = b"\xff\xe0\x00\x10JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00"
    photo = tmp_path / "IMG_0001.JPG"
    photo.write_bytes(b"\xff\xd8" + jfif + exif + b"\xff\xda\x00\x02" + b"\x01\x02" + b"\xff\xd9")
    args = sup.report_args(tmp_path, run, key_path,
                           extra=["--instrument-file", meta, "--photo", photo])
    assert _report(args) == 0
    out = tmp_path / "submission"
    assert json.loads((out / "raw/instrument/fit_meta.json").read_text()) == \
        {"session": {"path": "~/.../s1"}, "n": 3}
    assert (out / "raw/instrument/notes.txt").read_text() == "camera on [host]\n"
    stripped = (out / "notes/photos/IMG_0001.jpg").read_bytes()
    assert b"GPS" not in stripped and b"JFIF" in stripped and stripped.endswith(b"\xff\xd9")


def test_an_ev10_run_and_a_control_run(tmp_path, key):
    key_path, entry = key
    ev10 = sup.runner_run(tmp_path / "ev10", "ev10-pass")
    args = sup.report_args(tmp_path / "ev10", ev10, key_path, lab=sup.lab_config("ev10-pass"))
    assert _report(args) == 0
    rep = json.loads((tmp_path / "ev10/submission/report.json").read_text())
    assert rep["test"] == "EV-10" and rep["generator"]["name"] == "be-waypoints"
    assert (tmp_path / "ev10/submission/ev10-waypoints.json").is_file()
    assert _verify(tmp_path / "ev10/submission", entry).consistent

    # The runner's control block becomes be-control/1, seed 0, role control.
    gen = sup.golden_report("ev03-control")["generator"]["params"]
    control = {"name": "control", "target_id": gen["target"], "seconds": gen["seconds"],
               "hz": gen["hz"], "step_cap": gen["step_cap"], "start": gen["start"],
               "command_space_digest": "sha256:" + "0" * 64}
    run = sup.runner_run(tmp_path / "control", "ev03-control", run_mode="control", generator=control)
    args = sup.report_args(tmp_path / "control", run, key_path, lab=sup.lab_config("ev03-control"))
    assert _report(args) == 0
    rep = json.loads((tmp_path / "control/submission/report.json").read_text())
    assert rep["role"] == "control" and rep["generator"]["name"] == "be-control"
    assert rep["generator"]["seed"] == 0 and rep["verdict"]["result"] == "FAIL"


def test_wall_clock_pairs_bring_another_hosts_samples_onto_the_run(tmp_path, key):
    """The instrument's clock reads 1234.5 s at run start; the recorder's clock.csv pairs its
    clock with a wall clock that agrees with the runner's to 2 ms."""
    key_path, entry = key
    run = sup.runner_run(tmp_path, samples_offset="1234.5")
    w0 = sup.epoch(sup.GOLDEN_STARTED)
    rows = ["kind,t_wall_s,t_mono_s,uncertainty_s"]
    for k, tau in enumerate(range(-15, 30, 5)):
        rows.append(f"{'start' if k == 0 else 'periodic'},{w0 + tau + (k % 3 - 1) * 4e-5:.6f},"
                    f"{1234.5 + tau:.6f},0.000003000")
    (tmp_path / "clock.csv").write_text("\n".join(rows) + "\n")
    args = sup.report_args(tmp_path, run, key_path, clock="wall",
                           extra=["--clock-csv", tmp_path / "clock.csv",
                                  "--wall-clock-agreement-s", "0.002"])
    assert _report(args) == 0
    out = tmp_path / "submission"
    assert _verify(out, entry).consistent
    clock = json.loads((out / "report.json").read_text())["run"]["clock"]
    assert clock["offset_s"] == -1234.5
    # U = 2 ms agreement + 40 us half-range + 3 us reading + 1 us csv + 0.2 us start reading.
    assert clock["uncertainty_s"] == 0.002045
    assert (out / "raw/instrument/recorder-clock.csv").read_bytes() == (tmp_path / "clock.csv").read_bytes()


def test_the_runners_simulated_instrument_needs_no_clock(tmp_path, key):
    key_path, entry = key
    run = sup.runner_run(tmp_path, simulation=True, run_mode="shakedown",
                         instrument_feed={"kind": "simulation_ground_truth",
                                          "samples_csv": "raw/samples.csv"})
    (run / "raw" / "samples.csv").write_bytes((sup.RUNS / "ev03-pass/raw/samples.csv").read_bytes())
    config = sup.write_json(tmp_path / "lab.json", sup.lab_config())
    args = sup.be("report", run, "--lab-config", config, "--key", key_path, "--out",
                  tmp_path / "submission", "--instrument-file", tmp_path / "instrument/calibration.json",
                  "--accuracy", tmp_path / "instrument/accuracy.csv")
    assert _report(args) == 0
    rep = json.loads((tmp_path / "submission/report.json").read_text())
    assert rep["run"]["clock"]["offset_s"] == 0.0 and rep["run"]["clock"]["uncertainty_s"] == 0.0
    assert rep["run"]["clock"]["method"].startswith("run: the samples are on the run timebase")
    assert gzip.decompress((tmp_path / "submission/raw/samples.csv.gz").read_bytes()).splitlines()[1] \
        .startswith(b"-0.500000,tool,")
    assert rep["role"] == "rehearsal" and rep["run"]["mode"] == "simulation"
    assert rep["verdict"]["result"] == "INCONCLUSIVE"
    assert _verify(tmp_path / "submission", entry).consistent


# ----- refusals ------------------------------------------------------------------------------------
def _refused(args, needle: str, tmp_path: Path) -> None:
    """Refused with a message containing ``needle``, and --out exactly as it was before."""
    out = Path(args.out)
    before = sorted(out.rglob("*")) if out.exists() else None
    logs: list[str] = []
    assert _report(args, logs) == kitreport.EXIT_REFUSED, logs
    assert any(needle in m for m in logs), logs
    assert (sorted(out.rglob("*")) if out.exists() else None) == before
    assert not list(out.parent.glob(f".{out.name}.partial-*")), "staging left behind"


def test_refusals_before_anything_is_written(tmp_path, key):
    key_path, _ = key
    run = sup.runner_run(tmp_path)
    (tmp_path / "taken").mkdir()
    (tmp_path / "taken" / "x").write_text("x")
    _refused(sup.report_args(tmp_path, run, key_path, tmp_path / "taken"), "is not an empty directory",
             tmp_path)
    _refused(sup.report_args(tmp_path, run, key_path, clock=None), "--samples needs --clock", tmp_path)
    _refused(sup.report_args(tmp_path, run, key_path, extra=["--test", "EV-10"]),
             "--test EV-10, but the run directory holds a EV-03 run", tmp_path)
    _refused(sup.report_args(tmp_path, run, key_path, lab={**sup.lab_config(), "colour": "red"}),
             "Additional properties are not allowed", tmp_path)
    bad_key = tmp_path / "bad.key.json"
    bad_key.write_text(json.dumps({**json.loads(key_path.read_text()), "kid": "A" * 43}))
    _refused(sup.report_args(tmp_path, run, bad_key), "is not the key's thumbprint", tmp_path)
    _refused(sup.report_args(tmp_path, run, key_path, extra=[
        "--role", "control", "--control-run", "ev03-20261010T170000Z-s0"]),
        "a control run is not paired with another control", tmp_path)
    _refused(sup.report_args(tmp_path, run, key_path, extra=["--reproduces", "not-a-result"]),
             "expected a result id", tmp_path)


def test_refusals_about_the_run(tmp_path, key):
    key_path, _ = key
    never = sup.runner_run(tmp_path / "never", end={"status": "cancelled", "why": "operator",
                                                    "started": False})
    _refused(sup.report_args(tmp_path / "never", never, key_path), "the run never started", tmp_path)

    edited = sup.runner_run(tmp_path / "edited")
    env = json.loads((edited / "envelope.json").read_text())
    env["motion"]["max_speed_mps"] = 9.0
    (edited / "envelope.json").write_text(json.dumps(env))
    _refused(sup.report_args(tmp_path / "edited", edited, key_path),
             "envelope.json is not the envelope the run used", tmp_path)

    sim = sup.runner_run(tmp_path / "sim", simulation=True)
    _refused(sup.report_args(tmp_path / "sim", sim, key_path, extra=["--role", "test"]),
             "a simulation is a rehearsal", tmp_path)

    old = sup.declared_json("ev03-pass")
    old["timestamps"] = {"started_at": "2026-10-10T18:00:00+00:00", "ended_at": "2026-10-10T18:00:12+00:00"}
    older = sup.runner_run(tmp_path / "old", declared=old)
    _refused(sup.report_args(tmp_path / "old", older, key_path),
             "has no timestamps.monotonic_start_s", tmp_path)


def test_refusals_about_the_instrument(tmp_path, key):
    key_path, _ = key
    run = sup.runner_run(tmp_path, samples_offset="777.0")  # not this host's monotonic clock
    _refused(sup.report_args(tmp_path, run, key_path), "no valid instrument sample falls in the run",
             tmp_path)

    run2 = sup.runner_run(tmp_path / "two")
    lab = sup.lab_config()
    lab["instruments"][0]["calibration"]["files"].append("raw/instrument/registration.json")
    _refused(sup.report_args(tmp_path / "two", run2, key_path, lab=lab),
             "raw/instrument/registration.json (add them with --instrument-file)", tmp_path)

    lab = sup.lab_config()
    lab["instruments"][0]["measures"] = ["tool", "gripper_tip"]
    _refused(sup.report_args(tmp_path / "two", run2, key_path, lab=lab),
             "measure ['gripper_tip'], which the envelope does not declare", tmp_path)

    leaked = tmp_path / "two" / "leak.json"
    leaked.write_text(key_path.read_text())
    _refused(sup.report_args(tmp_path / "two", run2, key_path, extra=["--instrument-file", leaked]),
             "looks like a private key", tmp_path)

    _refused(sup.report_args(tmp_path / "two", run2, key_path, clock="wall",
                             extra=["--wall-clock-agreement-s", "0"]),
             "--clock wall needs the recorder's clock.csv", tmp_path)


def test_a_samples_file_split_in_two_is_joined_and_overlap_is_refused(tmp_path, key):
    key_path, _ = key
    run = sup.runner_run(tmp_path)
    lines = (tmp_path / "instrument/samples.csv").read_text().splitlines()
    half = len(lines) // 2
    (tmp_path / "a.csv").write_text("\n".join(lines[:half]) + "\n")
    (tmp_path / "b.csv").write_text("\n".join([lines[0], *lines[half:]]) + "\n")
    config = sup.write_json(tmp_path / "lab.json", sup.lab_config())
    base = ["report", run, "--lab-config", config, "--key", key_path, "--clock", "monotonic",
            "--accuracy", tmp_path / "instrument/accuracy.csv",
            "--instrument-file", tmp_path / "instrument/calibration.json"]
    joined = sup.be(*base, "--out", tmp_path / "joined", "--samples", tmp_path / "a.csv", tmp_path / "b.csv")
    assert _report(joined) == 0
    assert len(rawdata.read_samples(tmp_path / "joined/raw/samples.csv.gz")) == len(lines) - 1
    overlap = sup.be(*base, "--out", tmp_path / "overlap", "--samples", tmp_path / "a.csv",
                     tmp_path / "instrument/samples.csv")
    _refused(overlap, "overlapping instrument output", tmp_path)


def test_converted_times_are_exact_decimals(tmp_path):
    from bounded_embodiment_kit.submission.timebase import convert_samples

    data = (b"t_s,point,x_m,y_m,z_m,valid,residual_m\n5000.25,tool,0.1,0.2,0.3,1,\n"
            b"5000.123456789,tool,,,,0,0.001\n4.5e3,tool,0.1,0.2,0.3,1,\n")
    out = convert_samples([("s.csv", data)], Decimal("-5000.250000"))
    assert out.data.decode().splitlines()[1:] == [
        "0.000000,tool,0.1,0.2,0.3,1,", "-0.126543211,tool,,,,0,0.001",
        "-500.250000,tool,0.1,0.2,0.3,1,"]


def test_a_run_without_a_live_instrument_says_so_in_the_signed_notes(tmp_path, key):
    """`be run --instrument none --no-live-instrument-i-understand` records it in declared.json, and
    `be report` carries it into notes/NOTES.md (hashed and signed with the rest), after the lab's own."""
    key_path, entry = key
    doc = sup.declared_json("ev03-pass")
    doc["instrument_feed"] = {"kind": "none", "live_supervision": "robot_state_only", "why": "offline fit"}
    run = sup.runner_run(tmp_path, declared=doc)
    (tmp_path / "NOTES.md").write_text("Bob, Tuesday.\n")
    args = sup.report_args(tmp_path, run, key_path, extra=["--notes", tmp_path / "NOTES.md"])
    assert _report(args) == kitreport.EXIT_OK
    out = tmp_path / "submission"
    assert _verify(out, entry).consistent
    notes = (out / "notes" / "NOTES.md").read_text()
    assert notes.startswith("Bob, Tuesday.\n\n## Live supervision")
    assert "no live external instrument feed" in notes
    listed = {f["path"] for f in json.loads((out / "report.json").read_text())["raw_data"]}
    assert "notes/NOTES.md" in listed
    # Without --notes the statement is the whole file; with a live instrument there is none.
    assert kitreport.supervision_note({"instrument_feed": {"kind": "udp"}}) is None
    assert kitreport.supervision_note({}) is None
