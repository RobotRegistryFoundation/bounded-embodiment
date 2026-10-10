"""Ed25519 JWKs, RFC 7638 thumbprints, sign() and verify()."""

from __future__ import annotations

import base64
import copy
import json
import shutil
import subprocess
from pathlib import Path

import pytest
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey, Ed25519PublicKey

from bounded_embodiment import signing
from bounded_embodiment.canonical import CanonicalError, canonicalize, loads_strict, sha256_jcs
from bounded_embodiment.findings import Finding
from bounded_embodiment.signing import generate_key, sign, thumbprint, verify

REPO_ROOT = Path(__file__).resolve().parents[3]
VECTORS = REPO_ROOT / "vectors" / "signing"
needs_vectors = pytest.mark.skipif(not VECTORS.is_dir(), reason="repository vectors/ not available")

RFC8037_PRIVATE = {
    "kty": "OKP",
    "crv": "Ed25519",
    "d": "nWGxne_9WmC6hEr0kuwsxERJxWl7MmkZcDusAxyuf2A",
    "x": "11qYAYKxCrfVS_7TyWQHOg7hcvPapiMlrwIaaPcHURo",
}
RFC8037_PUBLIC = {k: v for k, v in RFC8037_PRIVATE.items() if k != "d"}
RFC8037_THUMBPRINT = "kPrK_qmxVWaYVA9wwBF6Iuo3vVzz7TxHCTwXBygrS4k"


def _unb64(s: str) -> bytes:
    return base64.urlsafe_b64decode(s + "=" * (-len(s) % 4))


def _b64(b: bytes) -> str:
    return base64.urlsafe_b64encode(b).rstrip(b"=").decode()


def _codes(findings: list[Finding]) -> list[tuple[str, str, str]]:
    return [(f.code, f.severity, f.where) for f in findings]


def _load(name: str) -> dict:
    return loads_strict((VECTORS / name).read_bytes())


# --------------------------------------------------------------------------- RFC vectors


def test_rfc8037_key_d_derives_x() -> None:
    key = Ed25519PrivateKey.from_private_bytes(_unb64(RFC8037_PRIVATE["d"]))
    assert _b64(key.public_key().public_bytes_raw()) == RFC8037_PRIVATE["x"]


def test_rfc8037_thumbprint() -> None:
    assert thumbprint(RFC8037_PUBLIC) == RFC8037_THUMBPRINT
    assert thumbprint(RFC8037_PRIVATE) == RFC8037_THUMBPRINT  # only required members count
    assert thumbprint({**RFC8037_PUBLIC, "kid": "x", "use": "sig", "alg": "EdDSA"}) == RFC8037_THUMBPRINT


def test_rfc8032_test1_empty_message() -> None:
    secret = bytes.fromhex("9d61b19deffd5a60ba844af492ec2cc44449c5697b326919703bac031cae7f60")
    expected_sig = bytes.fromhex(
        "e5564300c360ac729086e2cc806e828a84877f1eb8e5d974d873e065224901555fb8821590a33bac"
        "c61e39701cf9b46bd25bf5f0595bbe24655141438e7a100b"
    )
    assert secret == _unb64(RFC8037_PRIVATE["d"])  # RFC 8037 Appendix A is RFC 8032 TEST 1
    key = Ed25519PrivateKey.from_private_bytes(secret)
    assert key.sign(b"") == expected_sig
    Ed25519PublicKey.from_public_bytes(_unb64(RFC8037_PUBLIC["x"])).verify(expected_sig, b"")


@needs_vectors
def test_key_vectors() -> None:
    doc = _load("keys.json")
    keys = {k["name"]: k for k in doc["keys"]}
    for k in doc["keys"]:
        priv = Ed25519PrivateKey.from_private_bytes(_unb64(k["private_jwk"]["d"]))
        assert priv.private_bytes_raw().hex() == k["secret_key_hex"]
        assert priv.public_key().public_bytes_raw().hex() == k["public_key_hex"]
        assert _b64(priv.public_key().public_bytes_raw()) == k["public_jwk"]["x"]
        assert canonicalize({m: k["public_jwk"][m] for m in ("crv", "kty", "x")}).decode() == k["thumbprint_input"]
        assert thumbprint(k["public_jwk"]) == k["thumbprint"]
        assert thumbprint(k["private_jwk"]) == k["thumbprint"]
    assert keys["rfc8037"]["thumbprint"] == RFC8037_THUMBPRINT
    assert keys["rfc8037"]["private_jwk"] == RFC8037_PRIVATE
    for s in doc["signatures"]:
        k = keys[s["key"]]
        priv = Ed25519PrivateKey.from_private_bytes(bytes.fromhex(k["secret_key_hex"]))
        message = bytes.fromhex(s["message_hex"])
        assert priv.sign(message).hex() == s["signature_hex"]  # Ed25519 is deterministic
        priv.public_key().verify(bytes.fromhex(s["signature_hex"]), message)


# --------------------------------------------------------------------------- keys


def test_generate_key_shape_and_consistency() -> None:
    private, public = generate_key()
    assert set(private) == {"kty", "crv", "x", "d"} and set(public) == {"kty", "crv", "x"}
    assert private["kty"] == public["kty"] == "OKP" and private["crv"] == public["crv"] == "Ed25519"
    assert private["x"] == public["x"] and "=" not in private["x"] + private["d"]
    assert len(_unb64(private["d"])) == 32 and len(_unb64(public["x"])) == 32
    derived = Ed25519PrivateKey.from_private_bytes(_unb64(private["d"])).public_key()
    assert _b64(derived.public_bytes_raw()) == public["x"]
    assert thumbprint(private) == thumbprint(public)
    assert generate_key()[1]["x"] != public["x"]


@pytest.mark.parametrize(
    "jwk",
    [
        {"kty": "OKP", "crv": "Ed25519", "x": RFC8037_PUBLIC["x"] + "="},  # padding
        {"kty": "OKP", "crv": "Ed25519", "x": RFC8037_PUBLIC["x"][:-1] + "p"},  # non-zero trailing bits
        {"kty": "OKP", "crv": "Ed25519", "x": RFC8037_PUBLIC["x"][:-2]},  # 31 bytes
        {"kty": "OKP", "crv": "Ed25519", "x": base64.b64encode(_unb64(RFC8037_PUBLIC["x"])).decode()},
        {"kty": "OKP", "crv": "Ed448", "x": RFC8037_PUBLIC["x"]},
        {"kty": "EC", "crv": "Ed25519", "x": RFC8037_PUBLIC["x"]},
        {"crv": "Ed25519", "x": RFC8037_PUBLIC["x"]},
        {"kty": "OKP", "crv": "Ed25519"},
        {"kty": "OKP", "crv": "Ed25519", "x": 12},
        {"kty": "AKP", "alg": "ML-DSA-65", "pub": "AAAA"},
        "not a jwk",
    ],
)
def test_thumbprint_rejects_malformed_keys(jwk: object) -> None:
    with pytest.raises(ValueError):
        thumbprint(jwk)  # type: ignore[arg-type]


def test_generate_key_rejects_unknown_alg() -> None:
    with pytest.raises(ValueError):
        generate_key("RS256")


# --------------------------------------------------------------------------- sign


def test_sign_appends_and_does_not_mutate() -> None:
    doc = {"b": [1, 2.5], "a": "x"}
    before = copy.deepcopy(doc)
    signed = sign(doc, RFC8037_PRIVATE)
    assert doc == before
    assert [e["alg"] for e in signed["signature"]] == ["Ed25519"]
    entry = signed["signature"][0]
    assert set(entry) == {"alg", "kid", "sig"} and entry["kid"] == RFC8037_THUMBPRINT
    payload = canonicalize({"a": "x", "b": [1, 2.5]})
    Ed25519PublicKey.from_public_bytes(_unb64(RFC8037_PUBLIC["x"])).verify(_unb64(entry["sig"]), payload)
    # deterministic, and signing again appends rather than replaces
    again = sign(signed, RFC8037_PRIVATE)
    assert again["signature"] == [entry, entry]
    assert verify(again, {RFC8037_THUMBPRINT: RFC8037_PUBLIC}) == []


def test_signatures_cover_the_document_without_signature_only() -> None:
    private2, public2 = generate_key()
    ring = {RFC8037_THUMBPRINT: RFC8037_PUBLIC, thumbprint(public2): public2}
    doc = sign(sign({"k": 1}, RFC8037_PRIVATE), private2)
    assert len(doc["signature"]) == 2
    assert verify(doc, ring) == []
    doc["signature"].reverse()
    assert verify(doc, ring) == []


def test_sign_refuses_bad_inputs() -> None:
    with pytest.raises(ValueError, match="legacy"):
        sign({"signature": "ed25519:abc"}, RFC8037_PRIVATE)
    with pytest.raises(TypeError):
        sign([1, 2], RFC8037_PRIVATE)  # type: ignore[arg-type]
    with pytest.raises(ValueError):
        sign({"a": 1}, RFC8037_PUBLIC)  # no d
    _, other_public = generate_key()
    with pytest.raises(ValueError, match="not the public key"):
        sign({"a": 1}, {**RFC8037_PRIVATE, "x": other_public["x"]})
    with pytest.raises(CanonicalError):
        sign({"a": float("nan")}, RFC8037_PRIVATE)
    with pytest.raises(CanonicalError):
        sign({"a": 2**60}, RFC8037_PRIVATE)


# --------------------------------------------------------------------------- verify


def test_verify_round_trip_with_generated_key() -> None:
    private, public = generate_key()
    doc = sign({"envelope": {"hash": "sha256:" + "0" * 64}, "values": [0.1, 1e21]}, private)
    assert verify(doc, {thumbprint(public): public}) == []
    # a JSON round trip changes nothing
    assert verify(loads_strict(json.dumps(doc)), {thumbprint(public): public}) == []


def test_verify_never_uses_an_embedded_key() -> None:
    private, public = generate_key()
    doc = sign({"claims": "anything", "public_jwk": public, "keyring": {thumbprint(public): public}}, private)
    assert _codes(verify(doc, {})) == [("UNKNOWN_KID", "error", "/signature/0")]
    assert _codes(verify(doc, {RFC8037_THUMBPRINT: RFC8037_PUBLIC})) == [("UNKNOWN_KID", "error", "/signature/0")]


def test_verify_non_object_and_uncanonicalizable_documents() -> None:
    assert _codes(verify([1], {})) == [("NO_SIGNATURE", "error", "")]  # type: ignore[arg-type]
    doc = sign({"a": 1}, RFC8037_PRIVATE)
    doc["a"] = float("inf")
    assert _codes(verify(doc, {RFC8037_THUMBPRINT: RFC8037_PUBLIC})) == [("BAD_SIGNATURE", "error", "")]


@needs_vectors
def test_signed_sample_vector() -> None:
    sample = _load("signed-sample.json")
    keys = {k["thumbprint"]: k for k in _load("keys.json")["keys"]}
    assert canonicalize(sample["unsigned"]).decode("utf-8") == sample["canonical_payload"]
    assert sha256_jcs(sample["unsigned"]) == sample["payload_sha256"]
    first_kid = sample["signed"]["signature"][0]["kid"]
    # a conforming signer reproduces the published signatures byte for byte
    resigned = sign(sample["unsigned"], keys[first_kid]["private_jwk"])
    assert resigned == sample["signed"]
    second_kid = sample["signed_twice"]["signature"][1]["kid"]
    assert sign(resigned, keys[second_kid]["private_jwk"]) == sample["signed_twice"]
    assert verify(sample["signed"], sample["keyring"]) == []
    assert verify(sample["signed_twice"], sample["keyring"]) == []


def _tamper_cases() -> list[dict]:
    if not VECTORS.is_dir():
        return []
    doc = _load("tamper-cases.json")
    return [dict(c, keyring=c.get("keyring", doc["keyring"])) for c in doc["cases"]]


@needs_vectors
@pytest.mark.parametrize("case", _tamper_cases(), ids=lambda c: c["name"])
def test_tamper_vectors(case: dict) -> None:
    findings = verify(case["document"], case["keyring"])
    got = [{"code": f.code, "severity": f.severity, "where": f.where} for f in findings]
    assert got == case["expected"]


def _mldsa_cases() -> list[dict]:
    return _load("mldsa65.json")["cases"] if VECTORS.is_dir() else []


@needs_vectors
@pytest.mark.parametrize("case", _mldsa_cases(), ids=lambda c: c["name"])
def test_mldsa65_vectors_when_supported(case: dict) -> None:
    if not signing.mldsa65_supported():
        pytest.skip("this cryptography build has no ML-DSA")
    ring = _load("mldsa65.json")["keyring"]
    got = [{"code": f.code, "severity": f.severity, "where": f.where} for f in verify(case["document"], ring)]
    assert got == case["expected_if_supported"]


@needs_vectors
@pytest.mark.parametrize("case", _mldsa_cases(), ids=lambda c: c["name"])
def test_mldsa65_vectors_when_unsupported(case: dict, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(signing, "_mldsa65", lambda: None)
    assert not signing.mldsa65_supported()
    ring = _load("mldsa65.json")["keyring"]
    got = [{"code": f.code, "severity": f.severity, "where": f.where} for f in verify(case["document"], ring)]
    assert got == case["expected_if_unsupported"]


@needs_vectors
def test_mldsa65_key_vector() -> None:
    key = _load("mldsa65.json")["key"]
    assert thumbprint(key["public_jwk"]) == key["thumbprint"]
    assert thumbprint(key["private_jwk"]) == key["thumbprint"]
    assert canonicalize({m: key["public_jwk"][m] for m in ("alg", "kty", "pub")}).decode() == key["thumbprint_input"]
    if signing.mldsa65_supported():
        from cryptography.hazmat.primitives.asymmetric import mldsa

        derived = mldsa.MLDSA65PrivateKey.from_seed_bytes(_unb64(key["private_jwk"]["priv"]))
        assert _b64(derived.public_key().public_bytes_raw()) == key["public_jwk"]["pub"]


def test_mldsa65_generate_sign_verify() -> None:
    if not signing.mldsa65_supported():
        with pytest.raises(ValueError):
            generate_key("ML-DSA-65")
        return
    pq_private, pq_public = generate_key("ML-DSA-65")
    assert set(pq_public) == {"kty", "alg", "pub"} and pq_public["kty"] == "AKP"
    ring = {RFC8037_THUMBPRINT: RFC8037_PUBLIC, thumbprint(pq_public): pq_public}
    doc = sign(sign({"x": [1, 2]}, RFC8037_PRIVATE), pq_private)
    assert [e["alg"] for e in doc["signature"]] == ["Ed25519", "ML-DSA-65"]
    assert verify(doc, ring) == []
    doc["x"].append(3)
    assert _codes(verify(doc, ring)) == [
        ("BAD_SIGNATURE", "error", "/signature/0"),
        ("BAD_SIGNATURE", "error", "/signature/1"),
    ]


# --------------------------------------------------------------------------- node cross-check

NODE_VERIFY = r"""
'use strict';
const crypto = require('crypto');
const fs = require('fs');
function jcs(v) {
  if (v === null || typeof v !== 'object') return JSON.stringify(v);
  if (Array.isArray(v)) return '[' + v.map(jcs).join(',') + ']';
  return '{' + Object.keys(v).sort().map((k) => JSON.stringify(k) + ':' + jcs(v[k])).join(',') + '}';
}
const sample = JSON.parse(fs.readFileSync(process.argv[2], 'utf8'));
const results = [];
for (const doc of [sample.signed, sample.signed_twice]) {
  const body = Object.assign({}, doc);
  delete body.signature;
  const payload = Buffer.from(jcs(body), 'utf8');
  for (const entry of doc.signature) {
    const jwk = sample.keyring[entry.kid];
    const key = crypto.createPublicKey({ key: jwk, format: 'jwk' });
    results.push(crypto.verify(null, payload, key, Buffer.from(entry.sig, 'base64url')));
  }
  results.push(jcs(body) === sample.canonical_payload);
}
process.stdout.write(JSON.stringify(results));
"""


@needs_vectors
@pytest.mark.skipif(shutil.which("node") is None, reason="node is not installed")
def test_signed_sample_verifies_in_node(tmp_path: Path) -> None:
    script = tmp_path / "verify.js"
    script.write_text(NODE_VERIFY, encoding="utf-8")
    out = subprocess.run(
        [shutil.which("node"), str(script), str(VECTORS / "signed-sample.json")],
        check=True,
        capture_output=True,
        timeout=60,
    )
    assert json.loads(out.stdout) == [True, True, True, True, True]
