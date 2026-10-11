"""Cross-check canonicalize() against an independent JavaScript JCS run by Node.js.

JavaScript's default sort compares UTF-16 code units and JSON.stringify of a
number is Number::toString, so a few lines of JS are a reference RFC 8785
implementation. Skipped when ``node`` is not on PATH.

The values travel to Node as tagged arrays of numbers (strings as UTF-16 code
units), never as JSON objects, because JSON.parse itself is not trustworthy for
this: on Node 24 and 26 (found 10 Oct 2026; Node 20 and 22 are fine), after

    JSON.parse('{"a":1,"\\\\":2}')

the call JSON.parse('{"a":1,"\\u00e9":2}') returns the keys ["a", "\\"]
instead of ["a", "é"]. A JCS cross-check built on JSON.parse fails on those
versions for reasons that have nothing to do with canonicalization; so does
any verifier that parses signed JSON with escaped keys on them.
"""

from __future__ import annotations

import json
import math
import random
import shutil
import struct
import subprocess
from pathlib import Path

import pytest

from bounded_embodiment.canonical import MAX_EXACT_INT, canonicalize, loads_strict

NODE = shutil.which("node")
pytestmark = pytest.mark.skipif(NODE is None, reason="node is not installed")

VECTORS = Path(__file__).resolve().parents[3] / "vectors" / "canonical"

JS_JCS = r"""
'use strict';
function jcs(v) {
  if (v === null || typeof v !== 'object') {
    if (typeof v === 'number' && !Number.isFinite(v)) throw new Error('non-finite number');
    return JSON.stringify(v);
  }
  if (Array.isArray(v)) return '[' + v.map(jcs).join(',') + ']';
  return '{' + Object.keys(v).sort().map((k) => JSON.stringify(k) + ':' + jcs(v[k])).join(',') + '}';
}
const fs = require('fs');
const req = JSON.parse(fs.readFileSync(process.argv[2], 'utf8'));
const dv = new DataView(new ArrayBuffer(8));
function f64(h) { dv.setBigUint64(0, BigInt('0x' + h)); return dv.getFloat64(0); }
function str(units) { let s = ''; for (const u of units) s += String.fromCharCode(u); return s; }
// Rebuild a value from its tagged form; keys become own data properties ("__proto__" included).
function dec(e) {
  switch (e[0]) {
    case 'n': return null;
    case 't': return true;
    case 'f': return false;
    case 'i': return Number(e[1]);
    case 'd': return f64(e[1]);
    case 's': return str(e[1]);
    case 'a': return e[1].map(dec);
    case 'o': {
      const o = {};
      for (const [k, v] of e[1]) {
        Object.defineProperty(o, str(k), { value: dec(v), enumerable: true, writable: true, configurable: true });
      }
      return o;
    }
  }
  throw new Error('bad tag ' + e[0]);
}
const out = {
  numbers: req.numbers.map((h) => jcs(f64(h))),
  texts: req.values.map((e) => jcs(dec(e))),
};
fs.writeFileSync(process.argv[3], JSON.stringify(out), 'utf8');
"""

KEY_POOL = [
    "", "a", "b", "A", "Z", "aa", "ab", "a b", "_", "0", "10", "9", "~", "\u007f", "\u0080",
    "\u00e9", "e\u0301", "\u00f6", "\u20ac", "\ufb33", "\ue000", "\uffff", "\ud7ff",
    "\U0001F600", "\U0001F4A9", "\U00010000", "\U0010FFFF", "a\U0001F600", "a\uffff",
    "\r", "\n", "\u0000", "\u001f", '"', "\\", "/", "\u2028", "__proto__", "constructor",
    "\u05d3\u05bc", "\u4e2d", "key", "Key", "KEY",
]


def _random_float(rng: random.Random) -> float:
    pick = rng.random()
    if pick < 0.45:  # decimal-looking values across 1e-30 .. 1e30
        mantissa = rng.randint(1, 10 ** rng.randint(1, 17))
        x = float(f"{mantissa}e{rng.randint(-30 - 17, 30)}")
    elif pick < 0.7:  # uniform over the exponent range 1e-30 .. 1e30
        x = 10.0 ** rng.uniform(-30.0, 30.0)
    elif pick < 0.9:  # any finite double, including subnormals and huge values
        while True:
            x = struct.unpack("<d", rng.getrandbits(64).to_bytes(8, "little"))[0]
            if math.isfinite(x):
                break
    else:  # neighbours of the fixed/exponent switch points
        base = rng.choice([1e-6, 1e-7, 1e21, 1e20, 2.0**53, 5e-324, 1.7976931348623157e308])
        bits = struct.unpack("<Q", struct.pack("<d", base))[0] + rng.randint(-3, 3)
        x = struct.unpack("<d", struct.pack("<Q", bits % 2**64))[0]
        if not math.isfinite(x):
            x = base
    return -x if rng.random() < 0.5 else x


def _random_string(rng: random.Random) -> str:
    chars = []
    for _ in range(rng.randint(0, 12)):
        r = rng.random()
        if r < 0.4:
            chars.append(chr(rng.randint(0x20, 0x7E)))
        elif r < 0.55:
            chars.append(chr(rng.randint(0x00, 0x1F)))
        elif r < 0.75:
            cp = rng.randint(0x80, 0xFFFF)
            chars.append(chr(cp) if not 0xD800 <= cp <= 0xDFFF else "\ufffd")
        elif r < 0.9:
            chars.append(chr(rng.randint(0x10000, 0x10FFFF)))
        else:
            chars.append(rng.choice(['"', "\\", "/", "\u2028", "\u2029", "\x7f"]))
    return "".join(chars)


def _random_value(rng: random.Random, depth: int) -> object:
    r = rng.random()
    if depth <= 0 or r < 0.35:
        kind = rng.randrange(6)
        if kind == 0:
            return _random_float(rng)
        if kind == 1:
            return rng.randint(-MAX_EXACT_INT, MAX_EXACT_INT)
        if kind == 2:
            return _random_string(rng)
        if kind == 3:
            return rng.choice([True, False, None])
        if kind == 4:
            return rng.randint(-1000, 1000)
        return rng.choice([0.0, -0.0, 1.0, 0.5, 2.0**53])
    if r < 0.65:
        return [_random_value(rng, depth - 1) for _ in range(rng.randint(0, 5))]
    obj = {}
    for _ in range(rng.randint(0, 6)):
        key = rng.choice(KEY_POOL) if rng.random() < 0.7 else _random_string(rng)
        obj[key] = _random_value(rng, depth - 1)
    return obj


def _units(text: str) -> list[int]:
    """The UTF-16 code units of a string, as JavaScript holds it."""
    data = text.encode("utf-16-le", "surrogatepass")
    return [int.from_bytes(data[i : i + 2], "little") for i in range(0, len(data), 2)]


def _tagged(v: object) -> list:
    """A value as tagged arrays of numbers and plain tags, so JSON.parse never sees its strings."""
    if v is None:
        return ["n"]
    if v is True:
        return ["t"]
    if v is False:
        return ["f"]
    if isinstance(v, int):
        return ["i", str(v)]
    if isinstance(v, float):
        return ["d", struct.pack(">d", v).hex()]
    if isinstance(v, str):
        return ["s", _units(v)]
    if isinstance(v, list):
        return ["a", [_tagged(x) for x in v]]
    if isinstance(v, dict):
        return ["o", [[_units(k), _tagged(x)] for k, x in v.items()]]
    raise TypeError(type(v).__name__)


def _run_node(tmp_path: Path, numbers: list[float], values: list[object]) -> dict:
    req = {
        "numbers": [struct.pack(">d", x).hex() for x in numbers],
        "values": [_tagged(v) for v in values],
    }
    (tmp_path / "jcs.js").write_text(JS_JCS, encoding="utf-8")
    (tmp_path / "req.json").write_text(json.dumps(req), encoding="utf-8")
    subprocess.run(
        [NODE, str(tmp_path / "jcs.js"), str(tmp_path / "req.json"), str(tmp_path / "out.json")],
        check=True,
        timeout=120,
    )
    return json.loads((tmp_path / "out.json").read_text(encoding="utf-8"))


def test_random_values_match_node(tmp_path: Path) -> None:
    rng = random.Random(20261010)
    numbers = [_random_float(rng) for _ in range(1200)]
    numbers += [2.0**k for k in range(-1074, 1024)]  # every power of two: asymmetric rounding gaps
    values = [_random_value(rng, depth=4) for _ in range(800)]
    out = _run_node(tmp_path, numbers, values)
    mismatches = []
    for x, js in zip(numbers, out["numbers"], strict=True):
        py = canonicalize(x).decode("utf-8")
        if py != js:
            mismatches.append((repr(x), py, js))
    for v, js in zip(values, out["texts"], strict=True):
        py = canonicalize(v).decode("utf-8")
        if py != js:
            mismatches.append((repr(v)[:200], py[:200], js[:200]))
    assert not mismatches, mismatches[:10]
    assert len(out["numbers"]) == len(numbers) and len(out["texts"]) == len(values)


@pytest.mark.skipif(not VECTORS.is_dir(), reason="repository vectors/ not available")
def test_vectors_match_node(tmp_path: Path) -> None:
    numbers_cases = loads_strict((VECTORS / "numbers.json").read_bytes())["cases"]
    structure_cases = loads_strict((VECTORS / "structures.json").read_bytes())["cases"]
    numbers = [struct.unpack(">d", bytes.fromhex(c["ieee754"]))[0] for c in numbers_cases]
    values = [c["input"] if "input" in c else loads_strict(c["input_json"]) for c in structure_cases]
    out = _run_node(tmp_path, numbers, values)
    assert out["numbers"] == [c["expected"] for c in numbers_cases]
    assert out["texts"] == [c["expected"] for c in structure_cases]
