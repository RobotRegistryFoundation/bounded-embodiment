"""Signatures over RFC 8785 canonical JSON, with keys held as JWKs.

Keys
    Ed25519, required: ``{"kty": "OKP", "crv": "Ed25519", "x": ..., "d": ...}``
    (RFC 8037). ``x`` and ``d`` are base64url without padding.

    ML-DSA-65, optional: ``{"kty": "AKP", "alg": "ML-DSA-65", "pub": ..., "priv": ...}``
    (draft-ietf-cose-dilithium; ``priv`` is the 32-byte seed). Used only when the
    installed ``cryptography`` build supports ML-DSA.

Key ids
    ``kid`` is the RFC 7638 thumbprint of the public key: SHA-256 over the
    canonical JSON of the required members (``crv``, ``kty``, ``x`` for OKP;
    ``alg``, ``kty``, ``pub`` for AKP), base64url without padding.

Signatures
    ``doc["signature"]`` is a list of ``{"alg", "kid", "sig"}`` entries. Each one
    covers ``canonicalize(doc without its top-level "signature" member)``, so
    signatures can be added in any order and each verifies on its own.

``verify`` only ever uses keys from the keyring it is given, looked up by
``kid`` and re-checked against their own thumbprint. Nothing in the document,
including any key it carries, is trusted. At least one valid Ed25519 signature
is required; ML-DSA-65 signatures are checked in addition when supported.
"""

from __future__ import annotations

import base64
import copy
import functools
import hashlib
import os
import re
from collections.abc import Iterable, Mapping
from types import ModuleType

from cryptography.exceptions import InvalidSignature, UnsupportedAlgorithm
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey, Ed25519PublicKey

from .canonical import CanonicalError, canonicalize
from .findings import Finding, errors

__all__ = [
    "ED25519",
    "ML_DSA_65",
    "check_key_entry",
    "generate_key",
    "key_entry",
    "keyring_from_entries",
    "mldsa65_supported",
    "sign",
    "thumbprint",
    "verify",
]

ED25519 = "Ed25519"
ML_DSA_65 = "ML-DSA-65"

_ED25519_KEY_BYTES = 32
_ED25519_SIG_BYTES = 64
_MLDSA65_PUB_BYTES = 1952
_MLDSA65_SEED_BYTES = 32
_MLDSA65_SIG_BYTES = 3309

_B64URL = re.compile(r"[A-Za-z0-9_-]*")
_KID = re.compile(r"[A-Za-z0-9_-]{43}")


# --------------------------------------------------------------------------- base64url


def _b64url(data: bytes) -> str:
    return base64.urlsafe_b64encode(data).rstrip(b"=").decode("ascii")


def _unb64url(text: object, what: str) -> bytes:
    """Decode canonical base64url without padding; anything else is a ValueError."""
    if not isinstance(text, str):
        raise ValueError(f"{what} must be a base64url string")
    if not _B64URL.fullmatch(text) or len(text) % 4 == 1:
        raise ValueError(f"{what} is not base64url without padding")
    data = base64.urlsafe_b64decode(text + "=" * (-len(text) % 4))
    if _b64url(data) != text:
        raise ValueError(f"{what} is not canonical base64url (non-zero trailing bits)")
    return data


# --------------------------------------------------------------------------- ML-DSA support


@functools.cache
def _mldsa65() -> ModuleType | None:
    """The cryptography ML-DSA module if this build can do ML-DSA-65, else None."""
    try:
        from cryptography.hazmat.primitives.asymmetric import mldsa
    except ImportError:
        return None
    try:
        mldsa.MLDSA65PrivateKey.from_seed_bytes(bytes(_MLDSA65_SEED_BYTES))
    except (AttributeError, UnsupportedAlgorithm, TypeError, ValueError):
        return None
    return mldsa


def mldsa65_supported() -> bool:
    """True when the installed cryptography build can sign and verify ML-DSA-65."""
    return _mldsa65() is not None


# --------------------------------------------------------------------------- keys


def _okp(jwk: object, *, private: bool) -> tuple[bytes, bytes | None]:
    if not isinstance(jwk, Mapping):
        raise ValueError("a JWK must be a JSON object")
    if jwk.get("kty") != "OKP":
        raise ValueError(f"kty must be 'OKP' for an Ed25519 key, not {jwk.get('kty')!r}")
    if jwk.get("crv") != ED25519:
        raise ValueError(f"crv must be 'Ed25519', not {jwk.get('crv')!r}")
    x = _unb64url(jwk.get("x"), "x")
    if len(x) != _ED25519_KEY_BYTES:
        raise ValueError(f"x must decode to {_ED25519_KEY_BYTES} bytes, not {len(x)}")
    if not private:
        return x, None
    d = _unb64url(jwk.get("d"), "d")
    if len(d) != _ED25519_KEY_BYTES:
        raise ValueError(f"d must decode to {_ED25519_KEY_BYTES} bytes, not {len(d)}")
    return x, d


def _akp(jwk: object, *, private: bool) -> tuple[bytes, bytes | None]:
    if not isinstance(jwk, Mapping):
        raise ValueError("a JWK must be a JSON object")
    if jwk.get("kty") != "AKP":
        raise ValueError(f"kty must be 'AKP' for an ML-DSA key, not {jwk.get('kty')!r}")
    if jwk.get("alg") != ML_DSA_65:
        raise ValueError(f"alg must be 'ML-DSA-65', not {jwk.get('alg')!r}")
    pub = _unb64url(jwk.get("pub"), "pub")
    if len(pub) != _MLDSA65_PUB_BYTES:
        raise ValueError(f"pub must decode to {_MLDSA65_PUB_BYTES} bytes, not {len(pub)}")
    if not private:
        return pub, None
    seed = _unb64url(jwk.get("priv"), "priv")
    if len(seed) != _MLDSA65_SEED_BYTES:
        raise ValueError(f"priv must be the {_MLDSA65_SEED_BYTES}-byte seed, not {len(seed)} bytes")
    return pub, seed


def generate_key(alg: str = ED25519) -> tuple[dict, dict]:
    """Return ``(private_jwk, public_jwk)`` for a new key (Ed25519 unless asked)."""
    if alg == ED25519:
        key = Ed25519PrivateKey.generate()
        x = _b64url(key.public_key().public_bytes_raw())
        d = _b64url(key.private_bytes_raw())
        return {"kty": "OKP", "crv": ED25519, "x": x, "d": d}, {"kty": "OKP", "crv": ED25519, "x": x}
    if alg == ML_DSA_65:
        mldsa = _mldsa65()
        if mldsa is None:
            raise ValueError("ML-DSA-65 is not supported by the installed cryptography build")
        seed = os.urandom(_MLDSA65_SEED_BYTES)
        pub = _b64url(mldsa.MLDSA65PrivateKey.from_seed_bytes(seed).public_key().public_bytes_raw())
        return (
            {"kty": "AKP", "alg": ML_DSA_65, "pub": pub, "priv": _b64url(seed)},
            {"kty": "AKP", "alg": ML_DSA_65, "pub": pub},
        )
    raise ValueError(f"unsupported alg {alg!r}")


def thumbprint(public_jwk: dict) -> str:
    """RFC 7638 JWK thumbprint (SHA-256, base64url without padding).

    Only the required members are hashed, so a private JWK gives the same
    thumbprint as its public half.
    """
    kty = public_jwk.get("kty") if isinstance(public_jwk, Mapping) else None
    if kty == "OKP":
        _okp(public_jwk, private=False)
        members = {"crv": public_jwk["crv"], "kty": "OKP", "x": public_jwk["x"]}
    elif kty == "AKP":
        _akp(public_jwk, private=False)
        members = {"alg": public_jwk["alg"], "kty": "AKP", "pub": public_jwk["pub"]}
    else:
        raise ValueError(f"unsupported key type {kty!r}; expected 'OKP' (Ed25519) or 'AKP' (ML-DSA-65)")
    return _b64url(hashlib.sha256(canonicalize(members)).digest())


# --------------------------------------------------------------------------- signing


def _unsigned(doc: Mapping) -> dict:
    return {name: value for name, value in doc.items() if name != "signature"}


def _sign_payload(payload: bytes, private_jwk: dict) -> dict:
    kty = private_jwk.get("kty") if isinstance(private_jwk, Mapping) else None
    if kty == "OKP":
        x, d = _okp(private_jwk, private=True)
        key = Ed25519PrivateKey.from_private_bytes(d)
        if key.public_key().public_bytes_raw() != x:
            raise ValueError("the JWK's x is not the public key of its d")
        return {"alg": ED25519, "kid": thumbprint(private_jwk), "sig": _b64url(key.sign(payload))}
    if kty == "AKP":
        mldsa = _mldsa65()
        if mldsa is None:
            raise ValueError("ML-DSA-65 is not supported by the installed cryptography build")
        pub, seed = _akp(private_jwk, private=True)
        key = mldsa.MLDSA65PrivateKey.from_seed_bytes(seed)
        if key.public_key().public_bytes_raw() != pub:
            raise ValueError("the JWK's pub is not the public key of its priv seed")
        return {"alg": ML_DSA_65, "kid": thumbprint(private_jwk), "sig": _b64url(key.sign(payload))}
    raise ValueError(f"unsupported private key type {kty!r}")


def sign(doc: dict, private_jwk: dict) -> dict:
    """Return a copy of ``doc`` with one more entry in its ``signature`` list.

    The new entry is ``{"alg": "Ed25519", "kid": thumbprint, "sig": base64url}``
    (``"ML-DSA-65"`` for an AKP key) and covers the canonical JSON of ``doc``
    without its top-level ``signature`` member. Existing entries are kept.
    """
    if not isinstance(doc, Mapping):
        raise TypeError("sign() expects a JSON object")
    existing = doc.get("signature", [])
    if not isinstance(existing, list):
        raise ValueError(
            "the document's signature member is not a list of signature entries "
            "(a legacy string signature must be removed before signing)"
        )
    entry = _sign_payload(canonicalize(_unsigned(doc)), private_jwk)
    signed = copy.deepcopy(dict(doc))
    signed["signature"] = copy.deepcopy(existing) + [entry]
    return signed


# --------------------------------------------------------------------------- verification


def _keyring_key(
    keyring: Mapping, kid: str, alg: str, where: str
) -> tuple[object | None, Finding | None]:
    """Resolve kid to a public key object, re-deriving the thumbprint."""
    jwk = keyring.get(kid)
    if jwk is None:
        return None, Finding("UNKNOWN_KID", "error", f"kid {kid} is not in the keyring", where)
    try:
        actual = thumbprint(jwk)
    except ValueError as exc:
        return None, Finding(
            "UNKNOWN_KID", "error", f"the keyring entry for kid {kid} is not a usable public key: {exc}", where
        )
    if actual != kid:
        return None, Finding(
            "UNKNOWN_KID",
            "error",
            f"the keyring entry for kid {kid} holds a different key (its thumbprint is {actual})",
            where,
        )
    try:
        if alg == ED25519:
            x, _ = _okp(jwk, private=False)
            return Ed25519PublicKey.from_public_bytes(x), None
        mldsa = _mldsa65()
        pub, _ = _akp(jwk, private=False)
        return mldsa.MLDSA65PublicKey.from_public_bytes(pub), None  # type: ignore[union-attr]
    except (ValueError, UnsupportedAlgorithm) as exc:
        return None, Finding(
            "UNKNOWN_KID", "error", f"the keyring entry for kid {kid} is not a usable {alg} key: {exc}", where
        )


def verify(doc: dict, keyring: dict[str, dict]) -> list[Finding]:
    """Check every entry of ``doc["signature"]`` against ``keyring`` (kid -> public JWK).

    Returns an empty list when every entry verifies and at least one of them is
    a valid Ed25519 signature. Codes: NO_SIGNATURE, MALFORMED_SIGNATURE,
    UNKNOWN_KID, BAD_SIGNATURE (errors) and UNSUPPORTED_ALG (warning).
    """
    keyring = keyring or {}
    if not isinstance(doc, Mapping):
        return [Finding("NO_SIGNATURE", "error", "the document is not a JSON object")]
    if "signature" not in doc:
        return [Finding("NO_SIGNATURE", "error", "the document has no signature member", "/signature")]
    entries = doc["signature"]
    if isinstance(entries, str):
        return [
            Finding(
                "MALFORMED_SIGNATURE",
                "error",
                "signature is a legacy <alg>:<value> string with no kid; it cannot be checked against a keyring",
                "/signature",
            )
        ]
    if not isinstance(entries, list):
        return [Finding("MALFORMED_SIGNATURE", "error", "signature must be a list of entries", "/signature")]
    if not entries:
        return [Finding("NO_SIGNATURE", "error", "the signature list is empty", "/signature")]
    try:
        payload = canonicalize(_unsigned(doc))
    except CanonicalError as exc:
        return [
            Finding(
                "BAD_SIGNATURE",
                "error",
                f"the document cannot be canonicalized (RFC 8785), so no signature over it can be valid: {exc}",
                "",
            )
        ]

    findings: list[Finding] = []
    ed25519_ok = 0
    for i, entry in enumerate(entries):
        where = f"/signature/{i}"
        if not isinstance(entry, Mapping) or set(entry) != {"alg", "kid", "sig"}:
            findings.append(
                Finding(
                    "MALFORMED_SIGNATURE",
                    "error",
                    "a signature entry must be an object with exactly the members alg, kid and sig",
                    where,
                )
            )
            continue
        alg, kid, sig_text = entry["alg"], entry["kid"], entry["sig"]
        if not all(isinstance(v, str) for v in (alg, kid, sig_text)):
            findings.append(Finding("MALFORMED_SIGNATURE", "error", "alg, kid and sig must be strings", where))
            continue
        if alg not in (ED25519, ML_DSA_65):
            detail = f"signature algorithm {alg!r} is not supported; entry skipped"
            findings.append(Finding("UNSUPPORTED_ALG", "warn", detail, where))
            continue
        if alg == ML_DSA_65 and _mldsa65() is None:
            detail = "ML-DSA-65 is not supported by the installed cryptography build; entry skipped"
            findings.append(Finding("UNSUPPORTED_ALG", "warn", detail, where))
            continue
        if not _KID.fullmatch(kid):
            detail = "kid must be an RFC 7638 SHA-256 thumbprint (43 base64url characters)"
            findings.append(Finding("MALFORMED_SIGNATURE", "error", detail, where))
            continue
        try:
            sig = _unb64url(sig_text, "sig")
        except ValueError as exc:
            findings.append(Finding("MALFORMED_SIGNATURE", "error", str(exc), where))
            continue
        expected_len = _ED25519_SIG_BYTES if alg == ED25519 else _MLDSA65_SIG_BYTES
        if len(sig) != expected_len:
            detail = f"an {alg} signature is {expected_len} bytes, not {len(sig)}"
            findings.append(Finding("MALFORMED_SIGNATURE", "error", detail, where))
            continue
        key, problem = _keyring_key(keyring, kid, alg, where)
        if problem is not None:
            findings.append(problem)
            continue
        try:
            key.verify(sig, payload)  # type: ignore[union-attr]
        except InvalidSignature:
            detail = f"the {alg} signature by kid {kid} does not match the document"
            findings.append(Finding("BAD_SIGNATURE", "error", detail, where))
            continue
        if alg == ED25519:
            ed25519_ok += 1

    if not errors(findings) and ed25519_ok == 0:
        findings.append(
            Finding(
                "NO_SIGNATURE",
                "error",
                "no valid Ed25519 signature; Ed25519 is required and other algorithms only add to it",
                "/signature",
            )
        )
    return findings


# --------------------------------------------------------------------------- key-0.1 entries


def key_entry(public_jwk: dict, lab: str, contact: str, added: str) -> dict:
    """Build a key-0.1 entry for an Ed25519 public key (``added`` is YYYY-MM-DD)."""
    x, _ = _okp(public_jwk, private=False)
    return {
        "key_version": "0.1",
        "kid": thumbprint(public_jwk),
        "alg": ED25519,
        "public_jwk": {"kty": "OKP", "crv": ED25519, "x": _b64url(x)},
        "lab": lab,
        "contact": contact,
        "added": added,
    }


def check_key_entry(entry: object) -> list[Finding]:
    """Validate a key-0.1 entry, including the rule the schema cannot express:
    ``kid`` must be the RFC 7638 thumbprint of ``public_jwk`` (KID_MISMATCH)."""
    from .schema import validate

    findings = validate(entry, "key-0.1")
    if findings:
        return findings
    assert isinstance(entry, Mapping)
    try:
        actual = thumbprint(entry["public_jwk"])
    except ValueError as exc:
        return [Finding("SCHEMA_INVALID", "error", f"public_jwk is not a usable Ed25519 key: {exc}", "/public_jwk")]
    if actual != entry["kid"]:
        findings.append(
            Finding("KID_MISMATCH", "error", f"kid is not the RFC 7638 thumbprint of public_jwk ({actual})", "/kid")
        )
    if "revoked" in entry and entry["revoked"] < entry["added"]:
        findings.append(Finding("KEY_DATES_INVALID", "error", "revoked is earlier than added", "/revoked"))
    return findings


def keyring_from_entries(entries: Iterable[object], *, include_revoked: bool = False) -> dict[str, dict]:
    """Build a ``verify`` keyring from key-0.1 entries.

    Raises ValueError on the first invalid entry. Revoked keys are left out
    unless ``include_revoked`` is true.
    """
    ring: dict[str, dict] = {}
    for i, entry in enumerate(entries):
        problems = errors(check_key_entry(entry))
        if problems:
            first = problems[0]
            raise ValueError(f"key entry {i} is invalid: {first.code} at {first.where or '/'}: {first.detail}")
        assert isinstance(entry, Mapping)
        if "revoked" in entry and not include_revoked:
            continue
        ring[entry["kid"]] = dict(entry["public_jwk"])
    return ring
