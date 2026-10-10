"""Core arithmetic must give the same bits on every supported Python.

Built-in ``sum`` of floats changed in Python 3.12 (it now compensates for
rounding), so ``sum([0.1] * 10)`` is ``0.9999999999999999`` on 3.11 and ``1.0``
on 3.12+. ``statistics`` builds on it, ``math.hypot``/``math.dist`` changed
algorithm between versions, ``math.sumprod`` only exists from 3.12, and numpy
is not a core dependency. Core uses ``math.fsum`` (correctly rounded) or
explicit loops in a fixed order, and ``math.sqrt`` on explicit sums of squares.

This scans the source with ``ast`` and fails on:

* any use of the name ``sum`` (calling it, or passing it around), or ``builtins.sum``;
* importing ``statistics`` or ``numpy`` (directly, ``from`` imports, ``__import__``
  or ``importlib.import_module`` with a literal name);
* ``math.hypot``, ``math.dist`` or ``math.sumprod``, however ``math`` was imported.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parents[1] / "src" / "bounded_embodiment"

BANNED_MODULES = {"statistics", "numpy"}
BANNED_MATH = {"hypot", "dist", "sumprod"}


def violations(source: str, filename: str = "<string>") -> list[str]:
    tree = ast.parse(source, filename=filename)
    math_names = {"math"}
    found: list[str] = []

    def bad(node: ast.AST, what: str) -> None:
        found.append(f"{filename}:{getattr(node, 'lineno', '?')}: {what}")

    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                top = alias.name.split(".")[0]
                if top in BANNED_MODULES:
                    bad(node, f"import {alias.name}")
                if alias.name == "math":
                    math_names.add(alias.asname or "math")
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            top = node.module.split(".")[0]
            if top in BANNED_MODULES:
                bad(node, f"from {node.module} import ...")
            for alias in node.names:
                if node.module == "math" and alias.name in BANNED_MATH | {"*"}:
                    bad(node, f"from math import {alias.name}")
                if node.module == "builtins" and alias.name in ("sum", "*"):
                    bad(node, f"from builtins import {alias.name}")
    for node in ast.walk(tree):
        if isinstance(node, ast.Name) and node.id == "sum" and isinstance(node.ctx, ast.Load):
            bad(node, "built-in sum (use math.fsum or an explicit loop)")
        elif isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name):
            if node.value.id == "builtins" and node.attr == "sum":
                bad(node, "builtins.sum")
            if node.value.id in math_names and node.attr in BANNED_MATH:
                bad(node, f"math.{node.attr} (use math.sqrt over an explicit sum of squares)")
        elif isinstance(node, ast.Call) and node.args and isinstance(node.args[0], ast.Constant):
            target = node.args[0].value
            func = node.func
            dynamic = (isinstance(func, ast.Name) and func.id in ("__import__", "import_module")) or (
                isinstance(func, ast.Attribute) and func.attr == "import_module"
            )
            if dynamic and isinstance(target, str) and target.split(".")[0] in BANNED_MODULES:
                bad(node, f"dynamic import of {target}")
    return found


def _sources() -> list[Path]:
    return sorted(SRC.rglob("*.py"))


def test_core_sources_exist() -> None:
    names = {p.name for p in _sources()}
    assert {"canonical.py", "signing.py", "schema.py", "cli.py"} <= names


def test_core_uses_no_float_sum_statistics_or_numpy() -> None:
    found: list[str] = []
    for path in _sources():
        found += violations(path.read_text(encoding="utf-8"), str(path.relative_to(SRC.parent)))
    assert not found, "\n".join(found)


@pytest.mark.parametrize(
    "snippet",
    [
        "total = sum(values)",
        "f = sum\nf(values)",
        "x = list(map(sum, rows))",
        "import builtins\nbuiltins.sum(v)",
        "from builtins import sum",
        "import statistics",
        "import statistics as st",
        "from statistics import mean",
        "import numpy as np",
        "import numpy.linalg",
        "from numpy import array",
        "import math\nmath.hypot(1, 2)",
        "import math as m\nm.dist(a, b)",
        "from math import hypot",
        "from math import *",
        "import math\nmath.sumprod(a, b)",
        "np = __import__('numpy')",
        "import importlib\nimportlib.import_module('statistics')",
    ],
)
def test_the_scanner_catches(snippet: str) -> None:
    assert violations(snippet), snippet


@pytest.mark.parametrize(
    "snippet",
    [
        "import math\ntotal = math.fsum(values)",
        "total = 0.0\nfor v in values:\n    total += v",
        "import math\nr = math.sqrt(x * x + y * y)",
        "summary = 'sum'",
        "obj.sum_of_squares = 1",
        "def fsum_like(values):\n    return values",
    ],
)
def test_the_scanner_allows(snippet: str) -> None:
    assert violations(snippet) == [], snippet
