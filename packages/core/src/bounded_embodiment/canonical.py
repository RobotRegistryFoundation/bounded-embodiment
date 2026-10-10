"""RFC 8785 JSON Canonicalization Scheme (JCS), and a strict JSON loader.

``canonicalize(obj)`` returns the canonical UTF-8 bytes of a JSON value:

* no whitespace;
* object members sorted by the UTF-16 code units of their names;
* strings escaped exactly as ECMAScript ``JSON.stringify`` escapes them
  (``\\" \\\\ \\b \\f \\n \\r \\t``, ``\\u00xx`` in lower-case hex for the
  other C0 controls, everything else as literal UTF-8);
* numbers serialized as ECMAScript ``Number.prototype.toString`` does
  (RFC 8785 section 3.2.2.3), so ``5e-05`` becomes ``0.00005``, ``1e21``
  becomes ``1e+21`` and ``-0.0`` becomes ``0``.

Python ``int`` values are exact, so an integer above 2**53 in magnitude cannot
be carried by an IEEE 754 double without possibly changing it; it raises
``CanonicalError`` instead of being rounded. A ``float`` is already a double and
is serialized whatever its magnitude. NaN, the infinities, lone surrogates and
non-string object keys all raise ``CanonicalError``.

``loads_strict(data)`` parses JSON text and rejects what would make two
readers disagree about a document: duplicate object member names at any
depth, the non-JSON literals ``NaN``/``Infinity``/``-Infinity``, numbers that
overflow a double, lone surrogates, invalid UTF-8 and a leading byte order mark.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
import sys
from collections.abc import Mapping
from decimal import ROUND_CEILING, ROUND_FLOOR, Decimal, localcontext

__all__ = ["MAX_EXACT_INT", "CanonicalError", "canonicalize", "loads_strict", "sha256_jcs"]

MAX_EXACT_INT = 2**53
"""Largest integer magnitude ``canonicalize`` accepts for a Python ``int``."""


class CanonicalError(ValueError):
    """A value cannot be canonicalized, or JSON text is not acceptable."""


# --------------------------------------------------------------------------- strings

_SURROGATE = re.compile("[\ud800-\udfff]")
_NEEDS_ESCAPE = re.compile('[\\x00-\\x1f"\\\\]')
_ESCAPES = {
    '"': '\\"',
    "\\": "\\\\",
    "\b": "\\b",
    "\f": "\\f",
    "\n": "\\n",
    "\r": "\\r",
    "\t": "\\t",
}
for _code in range(0x20):
    _ESCAPES.setdefault(chr(_code), f"\\u{_code:04x}")
del _code


def _escape_match(match: re.Match[str]) -> str:
    return _ESCAPES[match.group()]


def _string(s: str) -> str:
    found = _SURROGATE.search(s)
    if found is not None:
        raise CanonicalError(
            f"string contains a lone surrogate U+{ord(found.group()):04X} at index {found.start()}"
        )
    return '"' + _NEEDS_ESCAPE.sub(_escape_match, s) + '"'


def _utf16_sort_key(name: str) -> bytes:
    # Big-endian UTF-16 bytes compare exactly as sequences of UTF-16 code units.
    return name.encode("utf-16-be")


# --------------------------------------------------------------------------- numbers


def _shortest_digits_repr(x: float) -> tuple[str, int]:
    """Shortest round-trip digits of a positive finite double, via CPython's repr.

    Returns ``(s, n)`` with ``s`` the decimal digits (no leading or trailing
    zeros) and ``x == 0.s * 10**n``.
    """
    text = float.__repr__(x)
    mantissa, _, exp = text.partition("e")
    int_part, _, frac_part = mantissa.partition(".")
    digits = (int_part + frac_part).lstrip("0")
    s = digits.rstrip("0")
    point = (int(exp) if exp else 0) - len(frac_part) + (len(digits) - len(s))
    return s, point + len(s)


def _shortest_digits_decimal(x: float) -> tuple[str, int]:
    """The same as ``_shortest_digits_repr`` without relying on repr.

    Used only where ``sys.float_repr_style`` is not ``"short"``. For each
    precision k from 1 to 17 it takes the two k-digit decimals around x, keeps
    those that read back as x, and picks the closer one (the even one on a
    tie), which is the choice ECMAScript Number::toString specifies.
    """
    exact = Decimal(x)
    for k in range(1, 18):
        candidates = []
        for rounding in (ROUND_FLOOR, ROUND_CEILING):
            with localcontext() as ctx:
                ctx.prec = k
                ctx.rounding = rounding
                c = +exact
            if float(c) == x and c not in candidates:
                candidates.append(c)
        if not candidates:
            continue

        def rank(c: Decimal, k: int = k) -> tuple[Decimal, int]:
            with localcontext() as ctx:
                ctx.prec = 2000  # exact: a double has at most 767 significant digits
                distance = abs(c - exact)
                significand = int(c.scaleb(k - 1 - c.adjusted()))
            return (distance, significand % 2)

        best = min(candidates, key=rank)
        sign, digit_tuple, exponent = best.as_tuple()
        s = "".join(map(str, digit_tuple)).lstrip("0")
        stripped = s.rstrip("0")
        point = int(exponent) + (len(s) - len(stripped))
        return stripped, point + len(stripped)
    raise AssertionError(f"no round-trip representation found for {x!r}")  # pragma: no cover


_shortest_digits = (
    _shortest_digits_repr if sys.float_repr_style == "short" else _shortest_digits_decimal
)


def _format_positive(x: float) -> str:
    s, n = _shortest_digits(x)
    k = len(s)
    if k <= n <= 21:
        return s + "0" * (n - k)
    if 0 < n <= 21:
        return s[:n] + "." + s[n:]
    if -6 < n <= 0:
        return "0." + "0" * (-n) + s
    e = n - 1
    sign = "+" if e >= 0 else "-"
    if k == 1:
        return f"{s}e{sign}{abs(e)}"
    return f"{s[0]}.{s[1:]}e{sign}{abs(e)}"


def _number(x: float) -> str:
    """ECMAScript Number::toString(x) for a finite double."""
    if math.isnan(x) or math.isinf(x):
        raise CanonicalError(f"{x!r} is not a JSON number")
    if x == 0:
        return "0"  # also -0.0
    if x < 0:
        return "-" + _format_positive(-x)
    return _format_positive(x)


def _integer(n: int) -> str:
    if n > MAX_EXACT_INT or n < -MAX_EXACT_INT:
        raise CanonicalError(
            f"integer {n} is larger than 2**53 in magnitude and cannot be represented "
            "exactly as an IEEE 754 double"
        )
    return str(int(n))


# --------------------------------------------------------------------------- values


def _serialize(value: object, out: list[str]) -> None:
    if value is None:
        out.append("null")
    elif value is True:
        out.append("true")
    elif value is False:
        out.append("false")
    elif isinstance(value, str):
        out.append(_string(value))
    elif isinstance(value, int):  # bool cannot be subclassed, so True/False never get here
        out.append(_integer(value))
    elif isinstance(value, float):
        out.append(_number(float(value)))
    elif isinstance(value, Mapping):
        names = list(value.keys())
        for name in names:
            if not isinstance(name, str):
                raise CanonicalError(f"object member name {name!r} is not a string")
        try:
            names.sort(key=_utf16_sort_key)
        except UnicodeEncodeError:
            bad = next(n for n in names if _SURROGATE.search(n))
            raise CanonicalError(f"object member name {bad!r} contains a lone surrogate") from None
        out.append("{")
        for i, name in enumerate(names):
            if i:
                out.append(",")
            out.append(_string(name))
            out.append(":")
            _serialize(value[name], out)
        out.append("}")
    elif isinstance(value, (list, tuple)):
        out.append("[")
        for i, item in enumerate(value):
            if i:
                out.append(",")
            _serialize(item, out)
        out.append("]")
    else:
        raise CanonicalError(f"{type(value).__name__} is not a JSON type")


def canonicalize(obj: object) -> bytes:
    """Return the RFC 8785 canonical UTF-8 bytes of a JSON-compatible value."""
    out: list[str] = []
    try:
        _serialize(obj, out)
    except RecursionError:
        raise CanonicalError("value is nested too deeply, or contains a cycle") from None
    return "".join(out).encode("utf-8")


def sha256_jcs(obj: object) -> str:
    """``"sha256:"`` followed by the lower-hex SHA-256 of ``canonicalize(obj)``."""
    return "sha256:" + hashlib.sha256(canonicalize(obj)).hexdigest()


# --------------------------------------------------------------------------- strict loading


def _reject_constant(name: str) -> object:
    raise CanonicalError(f"{name} is not valid JSON")


def _parse_float(text: str) -> float:
    value = float(text)
    if math.isinf(value):
        raise CanonicalError(f"number {text} is out of range for an IEEE 754 double")
    return value


def _no_duplicates(pairs: list[tuple[str, object]]) -> dict:
    obj: dict = {}
    for name, value in pairs:
        if name in obj:
            raise CanonicalError(f"duplicate object member name {name!r}")
        obj[name] = value
    return obj


def _check_strings(root: object) -> None:
    stack = [root]
    while stack:
        value = stack.pop()
        if isinstance(value, str):
            if _SURROGATE.search(value):
                raise CanonicalError(f"string {value!r} contains a lone surrogate")
        elif isinstance(value, dict):
            for name, item in value.items():
                if _SURROGATE.search(name):
                    raise CanonicalError(f"object member name {name!r} contains a lone surrogate")
                stack.append(item)
        elif isinstance(value, list):
            stack.extend(value)


def loads_strict(data: bytes | str) -> object:
    """Parse JSON text strictly.

    Bytes must be UTF-8. Rejects, with ``CanonicalError``: duplicate member
    names at any depth (compared after unescaping), ``NaN``, ``Infinity``,
    ``-Infinity``, numbers that overflow a double, lone surrogates (escaped or
    raw), a leading byte order mark, and anything that is not JSON. Integers
    come back as ``int`` and other numbers as ``float``.
    """
    if isinstance(data, (bytes, bytearray, memoryview)):
        try:
            text = bytes(data).decode("utf-8")
        except UnicodeDecodeError as exc:
            raise CanonicalError(f"JSON text is not valid UTF-8: {exc}") from None
    elif isinstance(data, str):
        text = data
    else:
        raise TypeError(f"loads_strict expects bytes or str, not {type(data).__name__}")
    if text.startswith("\ufeff"):
        raise CanonicalError("JSON text starts with a byte order mark (U+FEFF); remove it")
    try:
        value = json.loads(
            text,
            object_pairs_hook=_no_duplicates,
            parse_constant=_reject_constant,
            parse_float=_parse_float,
        )
    except CanonicalError:
        raise
    except RecursionError:
        raise CanonicalError("JSON text is nested too deeply") from None
    except ValueError as exc:
        raise CanonicalError(f"invalid JSON: {exc}") from None
    _check_strings(value)
    return value
