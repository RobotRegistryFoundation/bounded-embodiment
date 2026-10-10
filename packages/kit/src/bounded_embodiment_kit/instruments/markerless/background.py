# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""The static depth background (`be fit background`): what the camera sees when nothing moves.

Per pixel, the median of the valid depth (0 = none) over a still window: by default from the last
``background_start`` to the next ``background_end`` event; without those events, the first 10 s of depth
(reported as a fallback). A pixel valid in fewer than half the frames has no background (0), and the fitter
then treats any depth there as possible foreground.

Several still windows can be given (``windows``): their medians combine as the farthest one per pixel, and a
pixel unknown in any window stays unknown. That is for a background recorded with the arm in view (as in
the 10 Oct session, where it rests folded during background_start..end): adding a window where the arm holds
elsewhere (``hold_1:hold_1_end``) uncovers what is behind its rest pose, while pixels where the arm is in
both windows keep the arm (they are near its base). The fitter's foreground test is "nearer than the
background", so an arm that later passes through where it rested is not lost there.

Output: ``<session>/fit/background.npz`` (``background_mm`` uint16 HxW, ``iqr_mm`` uint16 spread of each
pixel's valid depth, ``valid_frac`` float32, ``windows`` (k, 2) t_mono_s, ``frames`` (k,)) and
``background.json`` (the report).
"""
from __future__ import annotations

import json
from collections.abc import Callable, Sequence
from pathlib import Path

import numpy as np

from ..session import Session

__all__ = ["background_windows", "build_background", "median_nonzero"]

FALLBACK_S = 10.0


def median_nonzero(stack: np.ndarray, min_valid_frac: float = 0.5) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Per-pixel (median, interquartile range, valid fraction) of the non-zero values of an (n, h, w) uint16
    stack; median 0 where fewer than ``min_valid_frac`` of the frames are non-zero."""
    n = stack.shape[0]
    s = np.sort(stack, axis=0)                       # zeros first
    nz = (stack > 0).sum(axis=0)
    first = n - nz                                   # index of the first non-zero value
    take = lambda idx: np.take_along_axis(s, np.clip(idx, 0, n - 1)[None], axis=0)[0]
    med = take(first + (nz - 1) // 2)
    iqr = take(first + (3 * nz) // 4).astype(np.int64) - take(first + nz // 4).astype(np.int64)
    ok = nz >= max(1, int(np.ceil(min_valid_frac * n)))
    return (np.where(ok, med, 0).astype(np.uint16), np.where(ok, np.maximum(iqr, 0), 0).astype(np.uint16),
            (nz / max(n, 1)).astype(np.float32))


def background_windows(sess: Session, specs: Sequence[str] | None = None) -> tuple[list[tuple[float, float]], str]:
    """The windows to use and where they came from. ``specs`` are ``START:END`` (labels, LABEL+S, or seconds
    since the recording started); ``END`` defaults to the first event after START."""
    if specs:
        out = []
        for spec in specs:
            a, _, b = str(spec).partition(":")
            t0 = sess.resolve_time(a)
            if b:
                t1 = sess.resolve_time(b)
            else:
                nxt = sess.next_event_after(t0)
                if nxt is None:
                    raise ValueError(f"window {spec!r}: no event after {a} to end it; give START:END")
                t1 = nxt[0]
            if t1 <= t0:
                raise ValueError(f"window {spec!r} ends before it starts")
            out.append((t0, t1))
        return out, "given: " + ", ".join(specs)
    t0 = sess.event_time("background_start")
    t1 = sess.event_time("background_end", after=t0) if t0 is not None else None
    if t0 is not None and t1 is not None:
        return [(t0, t1)], "events background_start..background_end"
    t = sess.depth["t"]
    if not t.size:
        raise ValueError("the session has no depth frames")
    return [(float(t[0]), float(t[0]) + FALLBACK_S)], (
        f"fallback: no background_start/background_end events, so the first {FALLBACK_S:g} s of depth")


def build_background(session, *, windows: Sequence[str] | None = None, out=None, max_frames: int = 150,
                     min_valid_frac: float = 0.5, say: Callable[[str], None] = print) -> dict:
    sess = session if isinstance(session, Session) else Session(session)
    wins, source = background_windows(sess, windows)
    meds, iqrs, fracs, used = [], [], [], []
    for t0, t1 in wins:
        idx = sess.depth_indices(t0, t1)
        if not idx.size:
            raise ValueError(f"no depth frames between {t0 - sess.t0:.2f} s and {t1 - sess.t0:.2f} s")
        pick = idx[np.linspace(0, idx.size - 1, min(max_frames, idx.size)).astype(int)]
        stack = np.stack([sess.depth_frame(int(i)) for i in pick])
        m, q, f = median_nonzero(stack, min_valid_frac)
        meds.append(m)
        iqrs.append(q)
        fracs.append(f)
        used.append(int(pick.size))
    known = np.all([m > 0 for m in meds], axis=0)
    bg = np.where(known, np.max(meds, axis=0), 0).astype(np.uint16)
    iqr = np.where(known, np.max(iqrs, axis=0), 0).astype(np.uint16)
    vf = np.min(fracs, axis=0).astype(np.float32)
    out = Path(out) if out else sess.path / "fit"
    out.mkdir(parents=True, exist_ok=True)
    npz = out / "background.npz"
    np.savez_compressed(npz, background_mm=bg, iqr_mm=iqr, valid_frac=vf,
                        windows=np.array(wins, dtype=np.float64), frames=np.array(used, dtype=np.int64))
    report = {
        "format": "be-fit-background/0.1",
        "session": str(sess.path),
        "source": source,
        "windows_s_since_start": [[a - sess.t0, b - sess.t0] for a, b in wins],
        "windows_t_mono_s": [list(w) for w in wins],
        "frames_used": used,
        "min_valid_frac": min_valid_frac,
        "combine": "single window" if len(wins) == 1 else "farthest per pixel; unknown if unknown in any window",
        "pixels_with_background": float(known.mean()),
        "median_iqr_mm": float(np.median(iqr[known])) if known.any() else None,
        "p95_iqr_mm": float(np.percentile(iqr[known], 95)) if known.any() else None,
        "file": str(npz),
    }
    (out / "background.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    say(f"background: {source}; {sum(used)} frames; {report['pixels_with_background'] * 100:.0f} % of pixels have a "
        f"background (median spread {report['median_iqr_mm'] or 0:.0f} mm) -> {npz}")
    return report
