"""The method texts: version 0.1 is frozen (its SHA-256 is pinned), version 0.2 keeps its sections
7 and 8 (EV-03 and EV-10) word for word, every test Annex A names exists, and the method README
cites each text's current SHA-256."""

from __future__ import annotations

import hashlib
import re

import pytest
from _report_helpers import REPO_ROOT, in_repo

pytestmark = in_repo

METHOD = REPO_ROOT / "method"
V01 = METHOD / "bounded-embodiment-method-v0.1.md"
V02 = METHOD / "bounded-embodiment-method-v0.2.md"
V01_SHA256 = "sha256:ff5b502e542bf14456a2a23c4c1b3fe9f68ccada0162d1052857bab53254c4a5"


def _sha(path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def _section(text: str, start: str, end: str) -> str:
    i = text.index(start)
    return text[i : text.index(end, i)]


def test_version_0_1_is_frozen() -> None:
    assert _sha(V01) == V01_SHA256


@pytest.mark.parametrize(
    ("start", "end"),
    [("\n## 7 EV-03", "\n## 8 EV-10"), ("\n## 8 EV-10", "\n## 9 Reporting")],
)
def test_version_0_2_keeps_ev03_and_ev10_word_for_word(start: str, end: str) -> None:
    a, b = V01.read_text(encoding="utf-8"), V02.read_text(encoding="utf-8")
    assert _section(a, start, end) == _section(b, start, end)


def test_version_0_2_says_what_it_is() -> None:
    text = V02.read_text(encoding="utf-8")
    assert "- **Version:** 0.2\n- **Status:** draft for comment\n" in text
    assert "The thresholds of version 0.2 are illustrative." in text
    for heading in ("## 12 EV-02: stop performance", "## 13 EV-05: model loss",
                    "### 1.7 What EV-02 can and cannot establish",
                    "### 1.8 What EV-05 can and cannot establish"):
        assert f"\n{heading}\n" in text
    assert b"\r" not in V02.read_bytes()


def _tests_named(text: str) -> set[tuple[str, str]]:
    annex = _section(text, "## Annex A", "## Annex B")
    return set(re.findall(r"`([A-Za-z0-9_]+\.py)::([A-Za-z0-9_]+)`", annex))


@pytest.mark.parametrize("path", [V01, V02], ids=["v0.1", "v0.2"])
def test_every_test_annex_a_names_exists(path) -> None:
    roots = [REPO_ROOT / "packages" / "core" / "tests", REPO_ROOT / "packages" / "kit" / "tests"]
    missing = []
    for file, name in sorted(_tests_named(path.read_text(encoding="utf-8"))):
        sources = [r / file for r in roots if (r / file).is_file()]
        if not any(re.search(rf"^def {name}\(", s.read_text(encoding="utf-8"), re.MULTILINE) for s in sources):
            missing.append(f"{file}::{name}")
    assert missing == []


def test_the_readme_cites_each_texts_sha256() -> None:
    readme = (METHOD / "README.md").read_text(encoding="utf-8")
    for path in (V01, V02):
        assert _sha(path) in readme, path.name
