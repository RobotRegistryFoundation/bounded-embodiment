"""RFC 8785 canonicalization and strict loading."""

from __future__ import annotations

import base64
import hashlib
import json
import math
import random
import re
import struct
from collections import OrderedDict
from decimal import Decimal
from pathlib import Path

import pytest

from bounded_embodiment import canonical
from bounded_embodiment.canonical import (
    MAX_EXACT_INT,
    CanonicalError,
    canonicalize,
    loads_strict,
    sha256_jcs,
)

REPO_ROOT = Path(__file__).resolve().parents[3]
VECTORS = REPO_ROOT / "vectors" / "canonical"

needs_vectors = pytest.mark.skipif(not VECTORS.is_dir(), reason="repository vectors/ not available")


def _load_vectors(name: str) -> list[dict]:
    return loads_strict((VECTORS / name).read_bytes())["cases"]


def _js_semantics_loads(text: str) -> object:
    """Parse as JavaScript would: integers beyond 2**53 become doubles."""

    def parse_int(s: str) -> object:
        n = int(s)
        return n if abs(n) <= MAX_EXACT_INT else float(s)

    return json.loads(text, parse_int=parse_int)


def _ids(case: dict) -> str:
    return case["name"]


# --------------------------------------------------------------------------- vectors


@needs_vectors
@pytest.mark.parametrize("case", _load_vectors("numbers.json") if VECTORS.is_dir() else [], ids=_ids)
def test_number_vectors(case: dict) -> None:
    bits = struct.unpack(">d", bytes.fromhex(case["ieee754"]))[0]
    value = case["input"]
    # the JSON input is the same double as the authoritative bit pattern (sign of zero included)
    assert struct.pack(">d", float(value)) == struct.pack(">d", bits)
    assert canonicalize(bits).decode("utf-8") == case["expected"]
    assert canonicalize(value).decode("utf-8") == case["expected"]
    assert sha256_jcs(value) == case["sha256"]
    assert case["sha256"] == "sha256:" + hashlib.sha256(case["expected"].encode("utf-8")).hexdigest()


@needs_vectors
@pytest.mark.parametrize("case", _load_vectors("structures.json") if VECTORS.is_dir() else [], ids=_ids)
def test_structure_vectors(case: dict) -> None:
    value = case["input"] if "input" in case else loads_strict(case["input_json"])
    assert canonicalize(value).decode("utf-8") == case["expected"]
    assert sha256_jcs(value) == case["sha256"]
    assert case["sha256"] == "sha256:" + hashlib.sha256(case["expected"].encode("utf-8")).hexdigest()
    # canonical text is a fixed point (read with JavaScript number semantics)
    again = _js_semantics_loads(case["expected"])
    assert canonicalize(again).decode("utf-8") == case["expected"]


@needs_vectors
@pytest.mark.parametrize("case", _load_vectors("invalid.json") if VECTORS.is_dir() else [], ids=_ids)
def test_invalid_vectors(case: dict) -> None:
    data = base64.b64decode(case["input_base64"]) if "input_base64" in case else case["input_json"]
    if case["stage"] == "parse":
        with pytest.raises(CanonicalError):
            loads_strict(data)
    else:
        assert case["stage"] == "canonicalize"
        value = loads_strict(data)
        with pytest.raises(CanonicalError):
            canonicalize(value)


@needs_vectors
def test_vector_files_cover_what_the_method_requires() -> None:
    numbers = {c["expected"] for c in _load_vectors("numbers.json")}
    for required in ("0.00005", "1e-7", "1e+21", "123456789012345680000", "0.1",
                     "0.3333333333333333", "9007199254740992", "-1.7976931348623157e+308",
                     "-5e-324", "-1e+300"):
        assert required in numbers, required
    names = {c["name"] for c in _load_vectors("structures.json")}
    assert {"rfc8785_3_2_3_sorting", "key_order_utf16_not_code_points", "nested_arrays",
            "rfc8785_3_2_2_example"} <= names


# --------------------------------------------------------------------------- numbers


@pytest.mark.parametrize(
    ("value", "expected"),
    [
        (5e-05, "0.00005"),
        (1e-7, "1e-7"),
        (1.5e-7, "1.5e-7"),
        (1e-6, "0.000001"),
        (1e21, "1e+21"),
        (1e20, "100000000000000000000"),
        (123456789012345680000.0, "123456789012345680000"),
        (0.1, "0.1"),
        (1 / 3, "0.3333333333333333"),
        (-0.0, "0"),
        (0.0, "0"),
        (0, "0"),
        (1.0, "1"),
        (-50.0, "-50"),
        (2**53, "9007199254740992"),
        (-(2**53), "-9007199254740992"),
        (float(2**53), "9007199254740992"),
        (5e-324, "5e-324"),
        (-1.7976931348623157e308, "-1.7976931348623157e+308"),
        (-1e-300, "-1e-300"),
        (333333333.33333329, "333333333.3333333"),
        (1e30, "1e+30"),
        (4.5, "4.5"),
        (2e-3, "0.002"),
        (1e-27, "1e-27"),
    ],
)
def test_numbers(value: object, expected: str) -> None:
    assert canonicalize(value) == expected.encode("ascii")


@pytest.mark.parametrize("value", [2**53 + 1, -(2**53) - 1, 2**64, -(10**30)])
def test_integers_beyond_2_pow_53_are_refused(value: int) -> None:
    with pytest.raises(CanonicalError, match="2\\*\\*53"):
        canonicalize(value)
    with pytest.raises(CanonicalError):
        canonicalize({"nested": [value]})


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf, float("nan")])
def test_non_finite_numbers_are_refused(value: float) -> None:
    with pytest.raises(CanonicalError):
        canonicalize(value)
    with pytest.raises(CanonicalError):
        canonicalize({"x": [value]})


def test_booleans_are_not_integers() -> None:
    assert canonicalize([True, False, 1, 0, None]) == b"[true,false,1,0,null]"
    assert canonicalize({"a": True, "b": 1}) == b'{"a":true,"b":1}'


def test_int_and_float_subclasses() -> None:
    import enum

    class Level(enum.IntEnum):
        ONE = 1

    class MyFloat(float):
        def __repr__(self) -> str:  # must not leak into the output
            return "nonsense"

    assert canonicalize([Level.ONE, MyFloat(0.5)]) == b"[1,0.5]"


def test_decimal_fallback_matches_repr() -> None:
    rng = random.Random(8785)
    values = [2.0**k for k in range(-1074, 1024)]
    values += [abs(struct.unpack("<d", rng.getrandbits(64).to_bytes(8, "little"))[0]) for _ in range(3000)]
    values += [float(f"{rng.randint(1, 10**17)}e{rng.randint(-30, 30)}") for _ in range(1000)]
    values += [1424953923781206.25, 9.999999999999997e22, 5e-324, 1.7976931348623157e308]
    for x in values:
        if not math.isfinite(x) or x == 0:
            continue
        assert canonical._shortest_digits_decimal(x) == canonical._shortest_digits_repr(x), repr(x)


def test_shortest_digits_round_trip() -> None:
    for x in (0.1, 1e21, 5e-324, 123.456, 1e-7):
        s, n = canonical._shortest_digits_repr(x)
        assert s and s[0] != "0" and s[-1] != "0"
        assert float(Decimal(int(s)).scaleb(n - len(s))) == x


# --------------------------------------------------------------------------- strings and keys


def test_string_escaping_matches_json_stringify() -> None:
    controls = "".join(chr(c) for c in range(0x20))
    expected = (
        '"\\u0000\\u0001\\u0002\\u0003\\u0004\\u0005\\u0006\\u0007\\b\\t\\n\\u000b\\f\\r'
        '\\u000e\\u000f\\u0010\\u0011\\u0012\\u0013\\u0014\\u0015\\u0016\\u0017\\u0018'
        '\\u0019\\u001a\\u001b\\u001c\\u001d\\u001e\\u001f"'
    )
    assert canonicalize(controls).decode("ascii") == expected
    assert canonicalize('"\\/') == b'"\\"\\\\/"'
    # DEL, U+2028, U+2029, non-BMP: literal UTF-8, never escaped
    assert canonicalize("\x7f\u2028\u2029\U0001F600") == '"\x7f\u2028\u2029\U0001F600"'.encode()
    assert canonicalize("Caf\u00e9 \u2615") == '"Caf\u00e9 \u2615"'.encode("utf-8")


@pytest.mark.parametrize("bad", ["\ud800", "x\udc00", chr(0xD83D) + chr(0xDE00), "\udfff"])
def test_lone_surrogates_are_refused(bad: str) -> None:
    # chr(0xD83D) + chr(0xDE00) is two separate code points in a Python str, not U+1F600
    with pytest.raises(CanonicalError, match="surrogate"):
        canonicalize(bad)
    with pytest.raises(CanonicalError, match="surrogate"):
        canonicalize({bad: 1})
    with pytest.raises(CanonicalError, match="surrogate"):
        canonicalize({"k": [bad]})


def test_keys_sort_by_utf16_code_units() -> None:
    doc = {"\ufb33": 3, "\U0001F600": 2, "\u20ac": 1, "\uffff": 5, "\ue000": 4}
    out = canonicalize(doc).decode("utf-8")
    assert out == '{"\u20ac":1,"\U0001F600":2,"\ue000":4,"\ufb33":3,"\uffff":5}'
    # code point order would differ
    assert sorted(doc) != [k for k in json.loads(out)]


def test_key_order_basic_and_nested() -> None:
    assert canonicalize({"b": 1, "a": {"d": 1, "c": 2}, "": 0}) == b'{"":0,"a":{"c":2,"d":1},"b":1}'
    assert canonicalize({"aa": 1, "a": 2, "ab": 3, "B": 4}) == b'{"B":4,"a":2,"aa":1,"ab":3}'
    assert canonicalize(OrderedDict([("z", 1), ("y", 2)])) == b'{"y":2,"z":1}'


@pytest.mark.parametrize("key", [1, 1.5, None, True, ("a",)])
def test_non_string_keys_are_refused(key: object) -> None:
    with pytest.raises(CanonicalError, match="not a string"):
        canonicalize({key: 1})


@pytest.mark.parametrize("value", [{1, 2}, b"bytes", Decimal("1.5"), object(), 1j, range(3)])
def test_non_json_types_are_refused(value: object) -> None:
    with pytest.raises(CanonicalError, match="not a JSON type"):
        canonicalize(value)


def test_tuples_are_arrays_and_no_whitespace() -> None:
    assert canonicalize((1, [2, (3,)], {"a": ()})) == b'[1,[2,[3]],{"a":[]}]'


def test_cycles_and_deep_nesting_raise_canonical_error() -> None:
    cyclic: list = []
    cyclic.append(cyclic)
    with pytest.raises(CanonicalError):
        canonicalize(cyclic)


def test_canonical_error_is_a_value_error() -> None:
    assert issubclass(CanonicalError, ValueError)


def test_sha256_jcs_format_and_value() -> None:
    h = sha256_jcs({"b": 1, "a": 2})
    assert re.fullmatch(r"sha256:[0-9a-f]{64}", h)
    assert h == "sha256:" + hashlib.sha256(b'{"a":2,"b":1}').hexdigest()
    assert sha256_jcs({"a": 2.0, "b": 1}) == h  # 2.0 and 2 are the same JSON number


# --------------------------------------------------------------------------- loads_strict


def test_loads_strict_returns_ints_and_floats() -> None:
    doc = loads_strict('{"i": 3, "f": 3.5, "e": 1e2, "z": -0, "neg": -0.0, "big": 9007199254740993}')
    assert type(doc["i"]) is int and doc["i"] == 3
    assert type(doc["f"]) is float and doc["f"] == 3.5
    assert type(doc["e"]) is float and doc["e"] == 100.0
    assert type(doc["z"]) is int and doc["z"] == 0
    assert type(doc["neg"]) is float and math.copysign(1.0, doc["neg"]) == -1.0
    assert doc["big"] == 9007199254740993  # exact; canonicalize refuses it
    with pytest.raises(CanonicalError):
        canonicalize(doc)


@pytest.mark.parametrize(
    "text",
    [
        '{"a":1,"a":2}',
        '{"a":1,"a":1}',
        '[{"x":{"b":null,"b":null}}]',
        '{"a":1,"\\u0061":2}',
        '{"outer":{"inner":[1,{"k":1,"k":2}]}}',
    ],
)
def test_loads_strict_rejects_duplicate_keys(text: str) -> None:
    with pytest.raises(CanonicalError, match="duplicate"):
        loads_strict(text)


@pytest.mark.parametrize("text", ["NaN", "[NaN]", "Infinity", '{"x":-Infinity}', "1e400", "[-1e309]"])
def test_loads_strict_rejects_non_finite(text: str) -> None:
    with pytest.raises(CanonicalError):
        loads_strict(text)


@pytest.mark.parametrize("text", ['"\\ud800"', '["x\\udc00"]', '{"\\ud83d":1}', '"\\ude00\\ud83d"', '"\ud800"'])
def test_loads_strict_rejects_lone_surrogates(text: str) -> None:
    with pytest.raises(CanonicalError, match="surrogate"):
        loads_strict(text)


def test_loads_strict_accepts_surrogate_pairs() -> None:
    assert loads_strict('"\\ud83d\\ude00"') == "\U0001F600"
    assert loads_strict('{"\\ud83d\\ude00":1}') == {"\U0001F600": 1}


def test_loads_strict_bytes_handling() -> None:
    assert loads_strict(b'{"a":[1,2]}') == {"a": [1, 2]}
    assert loads_strict(bytearray(b"[true]")) == [True]
    assert loads_strict(memoryview(b"null")) is None
    assert loads_strict('"caf\u00e9"'.encode("utf-8")) == "caf\u00e9"
    with pytest.raises(CanonicalError, match="UTF-8"):
        loads_strict(b'"\xff"')
    with pytest.raises(CanonicalError, match="UTF-8"):
        loads_strict('"x"'.encode("utf-16"))
    with pytest.raises(CanonicalError, match="byte order mark"):
        loads_strict(b"\xef\xbb\xbf{}")
    with pytest.raises(CanonicalError, match="byte order mark"):
        loads_strict("\ufeff{}")
    with pytest.raises(TypeError):
        loads_strict(123)  # type: ignore[arg-type]


@pytest.mark.parametrize("text", ["", "{", "[1,]", "{'a':1}", "{} x", "[1] [2]", '{"a" 1}', "01", '"\t"'])
def test_loads_strict_rejects_malformed_json(text: str) -> None:
    with pytest.raises(CanonicalError):
        loads_strict(text)


def test_loads_strict_rejects_absurd_nesting() -> None:
    with pytest.raises(CanonicalError):
        loads_strict("[" * 200_000 + "]" * 200_000)


def test_round_trip_through_loads_strict() -> None:
    doc = {"z": [1, 2.5, -0.0, "\u20ac", {"\U0001F600": None}], "a": True}
    text = canonicalize(doc)
    assert canonicalize(loads_strict(text)) == text
    assert loads_strict(text) == {"a": True, "z": [1, 2.5, 0.0, "\u20ac", {"\U0001F600": None}]}
