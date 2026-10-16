"""The ``be`` command line: keygen, canon, the lazy verify/envelope commands, extensions."""

from __future__ import annotations

import datetime
import io
import json
import os
import stat
import sys
import types
from pathlib import Path

import pytest
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey

from bounded_embodiment import cli, signing
from bounded_embodiment.canonical import canonicalize, sha256_jcs
from bounded_embodiment.findings import Finding

KEYGEN = ["--lab", "example-lab", "--contact", "lab@example.org"]


def _keygen(tmp_path: Path, name: str = "lab.key.json", *extra: str) -> tuple[int, Path]:
    path = tmp_path / name
    return cli.main(["keygen", "--out", str(path), *KEYGEN, *extra]), path


# --------------------------------------------------------------------------- keygen


def test_keygen_writes_a_private_key_and_prints_the_entry(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    code, path = _keygen(tmp_path, "lab.key.json", "--added", "2026-10-10")
    out, err = capsys.readouterr()
    assert code == 0
    entry = json.loads(out)
    assert signing.check_key_entry(entry) == []
    assert entry["lab"] == "example-lab" and entry["contact"] == "lab@example.org" and entry["added"] == "2026-10-10"
    key = json.loads(path.read_text(encoding="utf-8"))
    assert key["kty"] == "OKP" and key["crv"] == "Ed25519" and key["kid"] == entry["kid"]
    assert key["x"] == entry["public_jwk"]["x"]
    d = Ed25519PrivateKey.from_private_bytes(signing._unb64url(key["d"], "d"))
    assert signing._b64url(d.public_key().public_bytes_raw()) == key["x"]
    assert set(entry["public_jwk"]) == {"kty", "crv", "x"}  # no private material in the entry
    assert str(path) in err and "never commit" in err
    if os.name == "posix":
        assert stat.S_IMODE(path.stat().st_mode) == 0o600
    # the key signs, and the printed entry verifies it
    doc = signing.sign({"report": 1}, key)
    assert signing.verify(doc, signing.keyring_from_entries([entry])) == []


def test_keygen_defaults_added_to_today_utc(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    before = datetime.datetime.now(datetime.timezone.utc).date().isoformat()
    code, _ = _keygen(tmp_path)
    after = datetime.datetime.now(datetime.timezone.utc).date().isoformat()
    assert code == 0
    assert json.loads(capsys.readouterr().out)["added"] in (before, after)


def test_keygen_refuses_to_overwrite(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    code, path = _keygen(tmp_path)
    assert code == 0
    original = path.read_bytes()
    capsys.readouterr()
    code, _ = _keygen(tmp_path)
    out, err = capsys.readouterr()
    assert code == 1 and out == "" and "refusing to overwrite" in err
    assert path.read_bytes() == original


@pytest.mark.skipif(os.name != "posix", reason="symlinks")
def test_keygen_refuses_a_symlink(tmp_path: Path) -> None:
    target = tmp_path / "elsewhere.json"
    (tmp_path / "link.json").symlink_to(target)
    code, _ = _keygen(tmp_path, "link.json")
    assert code == 1 and not target.exists()


@pytest.mark.parametrize(
    "extra",
    [["--lab", "Bad Lab"], ["--contact", "nobody"], ["--added", "2026-02-30"]],
)
def test_keygen_rejects_bad_entry_fields_before_writing(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], extra: list[str]
) -> None:
    path = tmp_path / "k.json"
    code = cli.main(["keygen", "--out", str(path), *KEYGEN, *extra])
    assert code == 2 and not path.exists()
    assert extra[0] in capsys.readouterr().err


def test_keygen_explains_a_bad_lab_id_once_without_the_pattern(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    code = cli.main(["keygen", "--out", str(tmp_path / "k.json"), "--lab", "My Lab", "--contact", "nope"])
    err = capsys.readouterr().err
    assert code == 2 and not (tmp_path / "k.json").exists()
    assert err.count("--lab") == 1 and err.count("--contact") == 1
    assert "lower-case letters, digits and hyphens" in err and "email address" in err
    assert "^[" not in err


def test_keygen_unwritable_directory(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    code = cli.main(["keygen", "--out", str(tmp_path / "missing" / "k.json"), *KEYGEN])
    assert code == 1 and "cannot write" in capsys.readouterr().err


# --------------------------------------------------------------------------- canon


def test_canon_prints_canonical_bytes(tmp_path: Path, capsysbinary: pytest.CaptureFixture[bytes]) -> None:
    src = tmp_path / "doc.json"
    src.write_text('{ "b": [1.0, 5e-05, 1E21],\n  "a": "\\u20ac", "signature": [] }', encoding="utf-8")
    assert cli.main(["canon", str(src)]) == 0
    assert capsysbinary.readouterr().out == '{"a":"\u20ac","b":[1,0.00005,1e+21],"signature":[]}'.encode()
    assert cli.main(["canon", "--unsigned", str(src)]) == 0
    assert capsysbinary.readouterr().out == '{"a":"\u20ac","b":[1,0.00005,1e+21]}'.encode()
    assert cli.main(["canon", "--sha256", str(src)]) == 0
    expected = sha256_jcs({"a": "\u20ac", "b": [1, 5e-05, 1e21], "signature": []})
    assert capsysbinary.readouterr().out == (expected + "\n").encode()


def test_canon_reads_standard_input(
    monkeypatch: pytest.MonkeyPatch, capsysbinary: pytest.CaptureFixture[bytes]
) -> None:
    fake_stdin = types.SimpleNamespace(buffer=io.BytesIO(b'{"z":0,"y":-0.0}'))
    monkeypatch.setattr(sys, "stdin", fake_stdin)
    assert cli.main(["canon", "-"]) == 0
    assert capsysbinary.readouterr().out == b'{"y":0,"z":0}'


@pytest.mark.parametrize("text", ['{"a":1,"a":2}', "[NaN]", '"\\ud800"', "[9007199254740993]", "{"])
def test_canon_rejects_what_cannot_be_canonical(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], text: str
) -> None:
    src = tmp_path / "bad.json"
    src.write_text(text, encoding="utf-8")
    assert cli.main(["canon", str(src)]) == 1
    assert "bad.json" in capsys.readouterr().err


def test_canon_missing_file(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    assert cli.main(["canon", str(tmp_path / "nope.json")]) == 1
    assert "cannot read" in capsys.readouterr().err


# --------------------------------------------------------------------------- verify (pass-through)


def test_verify_without_report_module(monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]) -> None:
    monkeypatch.setitem(sys.modules, "bounded_embodiment.report", None)  # import raises ImportError
    assert cli.main(["verify", "run-dir", "--json"]) == 2
    assert "not available yet" in capsys.readouterr().err


def test_verify_without_hook(monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]) -> None:
    monkeypatch.setitem(sys.modules, "bounded_embodiment.report", types.ModuleType("bounded_embodiment.report"))
    assert cli.main(["verify", "run-dir"]) == 2
    assert "cli_verify" in capsys.readouterr().err


def test_verify_hands_its_arguments_to_the_report_module(monkeypatch: pytest.MonkeyPatch) -> None:
    seen: list[list[str]] = []
    report = types.ModuleType("bounded_embodiment.report")
    report.cli_verify = lambda argv: seen.append(argv) or 3  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "bounded_embodiment.report", report)
    assert cli.main(["verify", "runs/x", "--keys", "lab.json", "--json"]) == 3
    assert cli.main(["verify", "--help"]) == 3
    assert seen == [["runs/x", "--keys", "lab.json", "--json"], ["--help"]]


# --------------------------------------------------------------------------- envelope


class _FakeEnvelopeError(ValueError):
    def __init__(self, findings: list[Finding]):
        super().__init__("invalid envelope")
        self.findings = findings


def _fake_envelope(monkeypatch: pytest.MonkeyPatch, load=None, convert=None) -> None:
    env = types.ModuleType("bounded_embodiment.envelope")
    env.EnvelopeError = _FakeEnvelopeError  # type: ignore[attr-defined]
    if load is not None:
        env.load = load  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "bounded_embodiment.envelope", env)
    conv = types.ModuleType("bounded_embodiment.envelope.convert")
    if convert is not None:
        conv.from_harness_arm = convert  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "bounded_embodiment.envelope.convert", conv)


def test_envelope_commands_without_the_module(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setitem(sys.modules, "bounded_embodiment.envelope", None)
    assert cli.main(["envelope", "validate", "e.json"]) == 2
    assert "not available yet" in capsys.readouterr().err
    _fake_envelope(monkeypatch)  # a module without load()
    assert cli.main(["envelope", "hash", "e.json"]) == 2
    assert "not available yet" in capsys.readouterr().err


def test_envelope_validate_and_hash(monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]) -> None:
    loaded = types.SimpleNamespace(version="0.2", hash="sha256:" + "1" * 64,
                                   findings=(Finding("NOTE", "warn", "just so you know", "/notes"),))
    calls: list[object] = []
    _fake_envelope(monkeypatch, load=lambda src: calls.append(src) or loaded)
    assert cli.main(["envelope", "validate", "e.json"]) == 0
    out = capsys.readouterr().out
    assert "valid envelope 0.2 sha256:111" in out and "NOTE" in out
    assert calls == [Path("e.json")]
    assert cli.main(["envelope", "validate", "--json", "e.json"]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["valid"] is True and report["hash"] == loaded.hash and report["findings"][0]["code"] == "NOTE"
    assert cli.main(["envelope", "hash", "e.json"]) == 0
    assert capsys.readouterr().out == loaded.hash + "\n"


def test_envelope_validate_reports_findings(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    def load(src: object) -> None:
        raise _FakeEnvelopeError([Finding("SCHEMA_INVALID", "error", "'frame' is a required property", "")])

    _fake_envelope(monkeypatch, load=load)
    assert cli.main(["envelope", "validate", "e.json"]) == 1
    assert "SCHEMA_INVALID" in capsys.readouterr().err
    assert cli.main(["envelope", "validate", "--json", "e.json"]) == 1
    assert json.loads(capsys.readouterr().out)["findings"][0]["code"] == "SCHEMA_INVALID"
    assert cli.main(["envelope"]) == 2


def test_envelope_convert(tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]) -> None:
    calls: list[tuple[object, dict]] = []

    def from_harness_arm(doc: dict, **kwargs: object) -> tuple[dict, dict]:
        calls.append((doc, kwargs))
        return {"envelope_version": "0.2", "machine": {"id": kwargs["machine_id"]}}, {"tolerance_m": 0.01}

    _fake_envelope(monkeypatch, load=lambda src: None, convert=from_harness_arm)
    src = tmp_path / "harness.json"
    src.write_text('{"keep_in": {"min_m": [0, 0, 0], "max_m": [1, 1, 1]}}', encoding="utf-8")
    stop_file = tmp_path / "stop.json"
    stop_file.write_text('{"category": 1, "max_time_ms": 300, "max_distance_m": 0.05}', encoding="utf-8")
    heartbeat = '{"model_timeout_ms": 500, "gate_timeout_ms": 200, "on_loss": "stop"}'
    base = ["envelope", "convert", "--from", "harness-arm", str(src), "--machine-id", "bob",
            "--stop", str(stop_file), "--heartbeat", heartbeat]

    assert cli.main(base) == 0
    out, err = capsys.readouterr()
    assert json.loads(out)["machine"]["id"] == "bob" and "--kit-config" in err
    doc, kwargs = calls[0]
    assert doc == {"keep_in": {"min_m": [0, 0, 0], "max_m": [1, 1, 1]}}
    assert kwargs == {"machine_id": "bob", "stop": {"category": 1, "max_time_ms": 300, "max_distance_m": 0.05},
                      "heartbeat": {"model_timeout_ms": 500, "gate_timeout_ms": 200, "on_loss": "stop"}}

    out_path, kit_path = tmp_path / "env.json", tmp_path / "kit.json"
    args = [*base, "--frame", "world", "--class", "arm", "--out", str(out_path), "--kit-config", str(kit_path)]
    assert cli.main(args) == 0
    assert json.loads(out_path.read_text())["envelope_version"] == "0.2"
    assert json.loads(kit_path.read_text()) == {"tolerance_m": 0.01}
    assert calls[-1][1]["frame"] == "world" and calls[-1][1]["machine_class"] == "arm"
    capsys.readouterr()
    assert cli.main(args) == 1 and "--force" in capsys.readouterr().err
    assert cli.main([*args, "--force"]) == 0

    bad = [*base[:-2], "--heartbeat", "{not json"]
    assert cli.main(bad) == 1 and "--heartbeat" in capsys.readouterr().err


def test_envelope_commands_against_the_real_module(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    envelope = pytest.importorskip("bounded_embodiment.envelope")
    if not callable(getattr(envelope, "load", None)):
        pytest.skip("bounded_embodiment.envelope has no load() yet")
    doc = {
        "envelope_version": "0.1",
        "machine": {"id": "cli-test", "class": "tabletop_arm"},
        "level": "BE-1",
        "workspace": {"frame": "base_link", "keep_in": [[0, 0], [1, 0], [1, 1], [0, 1]]},
        "motion": {"max_speed_mps": 0.25},
        "stop": {"category": 1, "max_time_ms": 300, "max_distance_m": 0.05},
        "heartbeat": {"model_timeout_ms": 500, "gate_timeout_ms": 200, "on_loss": "stop"},
    }
    src = tmp_path / "envelope.json"
    src.write_text(json.dumps(doc), encoding="utf-8")
    assert cli.main(["envelope", "hash", str(src)]) == 0
    assert capsys.readouterr().out.strip() == sha256_jcs(doc)
    assert cli.main(["envelope", "validate", str(src)]) == 0
    assert sha256_jcs(doc) in capsys.readouterr().out


# --------------------------------------------------------------------------- extensions and general


class _FakeEntryPoint:
    def __init__(self, name: str, loader) -> None:
        self.name, self.value, self._loader = name, f"fake_module:{name}", loader

    def load(self):
        return self._loader()


def test_extensions_add_commands(monkeypatch: pytest.MonkeyPatch) -> None:
    def register(subparsers) -> None:
        p = subparsers.add_parser("hello", help="say hello")
        p.add_argument("--times", type=int, default=1)
        p.set_defaults(func=lambda args: 40 + args.times)

    monkeypatch.setattr(cli, "_entry_points", lambda: [_FakeEntryPoint("kit", lambda: register)])
    assert cli.main(["hello", "--times", "2"]) == 42


def test_extensions_cannot_replace_core_commands_or_break_the_cli(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    def hijack(subparsers) -> None:
        subparsers.add_parser("keygen").set_defaults(func=lambda args: 99)

    def hijack_alias(subparsers) -> None:
        subparsers.add_parser("fine", aliases=["canon"]).set_defaults(func=lambda args: 99)

    def broken():
        raise ImportError("numpy is not installed")

    monkeypatch.setattr(cli, "_entry_points", lambda: [
        _FakeEntryPoint("a", lambda: hijack),
        _FakeEntryPoint("b", lambda: hijack_alias),
        _FakeEntryPoint("c", broken),
    ])
    src = tmp_path / "x.json"
    src.write_text("[1]", encoding="utf-8")
    assert cli.main(["canon", str(src)]) == 0
    out, err = capsys.readouterr()
    assert out == "[1]"
    assert err.count("warning: CLI extension") == 3 and "already registered" in err and "numpy" in err


@pytest.mark.parametrize("command", sorted(cli.KIT_COMMANDS))
def test_a_kit_command_without_the_kit_says_how_to_install_it(
    command: str, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(cli, "_entry_points", lambda: [])
    assert cli.main([command, "--help"]) == 2
    assert "pip install bounded-embodiment-kit" in capsys.readouterr().err


def test_a_kit_command_with_the_kit_is_not_intercepted(monkeypatch: pytest.MonkeyPatch) -> None:
    def register(subparsers) -> None:
        subparsers.add_parser("run").set_defaults(func=lambda args: 41)

    monkeypatch.setattr(cli, "_entry_points", lambda: [_FakeEntryPoint("kit", lambda: register)])
    assert cli.main(["run"]) == 41


def test_real_entry_point_lookup_works() -> None:
    assert isinstance(cli._entry_points(), list)


def test_no_command_prints_help(capsys: pytest.CaptureFixture[str]) -> None:
    assert cli.main([]) == 2
    assert "keygen" in capsys.readouterr().err


def test_version(capsys: pytest.CaptureFixture[str]) -> None:
    from bounded_embodiment import __version__

    with pytest.raises(SystemExit) as excinfo:
        cli.main(["--version"])
    assert excinfo.value.code == 0
    assert __version__ in capsys.readouterr().out


def test_help_lists_core_commands(capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(SystemExit):
        cli.main(["--help"])
    out = capsys.readouterr().out
    for command in ("keygen", "canon", "envelope", "verify"):
        assert command in out


def test_canonical_output_of_keygen_entry_is_stable(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    code, _ = _keygen(tmp_path, "k.json", "--added", "2026-10-10")
    entry = json.loads(capsys.readouterr().out)
    assert code == 0 and canonicalize(entry) == canonicalize(json.loads(json.dumps(entry)))
