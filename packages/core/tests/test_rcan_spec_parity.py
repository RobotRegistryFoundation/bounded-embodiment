"""Parity with the hashes already published in rcan-spec.

The rcan-spec fixtures are not copied into this repository (their licence is
still to be decided upstream). Point RCAN_SPEC_DIR at a checkout of
RobotRegistryFoundation/rcan-spec to run these checks; otherwise they skip.
"""

from __future__ import annotations

import base64
import os
from pathlib import Path

import pytest

from bounded_embodiment.canonical import canonicalize, loads_strict, sha256_jcs

ROOT = Path(os.environ.get("RCAN_SPEC_DIR", "/nonexistent-rcan-spec"))
pytestmark = pytest.mark.skipif(
    not (ROOT / "fixtures").is_dir(), reason="set RCAN_SPEC_DIR to an rcan-spec checkout"
)

ROVER_ENVELOPE_HASH = "sha256:b36eaa8823b50ff231532b23444514e03fb9594b6bb3c8a0f9efea1bfd7ddb5e"
ROVER_CHAIN_HEAD = "sha256:b580f2833bcecc528cdce9f68e90706ede2c6d690e686b625ff4f35bc20b7032"


def _read(rel: str) -> object:
    return loads_strict((ROOT / rel).read_bytes())


def test_rover_envelope_hash() -> None:
    env = _read("fixtures/envelope/rover.valid.json")
    body = {k: v for k, v in env.items() if k != "signature"}
    assert sha256_jcs(body) == ROVER_ENVELOPE_HASH


def test_rover_chain_records_and_head() -> None:
    chain = _read("fixtures/gate-decision/rover-chain.valid.json")
    for record in chain:
        assert sha256_jcs({k: v for k, v in record.items() if k != "hash"}) == record["hash"]
        assert record["envelope"] == ROVER_ENVELOPE_HASH
    assert chain[-1]["hash"] == ROVER_CHAIN_HEAD


def test_canonical_json_v1_cases() -> None:
    suite = _read("fixtures/canonical-json-v1.json")
    assert len(suite["cases"]) >= 12
    for case in suite["cases"]:
        got = base64.b64encode(canonicalize(case["input"])).decode("ascii")
        assert got == case["expected_bytes_base64"], case["name"]
