# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""Deterministic gzip, JPEG metadata removal, and the private-key and LFS checks."""
from __future__ import annotations

import gzip
import json
import struct
import zlib

import pytest
from bounded_embodiment import rawdata, signing
from bounded_embodiment import report as core_report
from bounded_embodiment_kit.submission.files import (
    GZIP_HEADER,
    JpegError,
    gzip_bytes,
    looks_like_lfs_pointer,
    looks_like_private_key,
    strip_jpeg_metadata,
)

CSV = b"t_s,point,x_m,y_m,z_m,valid,residual_m\n" + b"".join(
    f"{i / 30:.6f},tool,0.2,0.0,0.1,1,\n".encode() for i in range(300))


def test_gzip_depends_only_on_the_content():
    once, again = gzip_bytes(CSV), gzip_bytes(CSV)
    assert once == again and once != gzip_bytes(CSV + b"0.1,tool,0.2,0.0,0.1,1,\n")
    # ID1 ID2, deflate, no flags (so no file name), mtime 0, XFL 2, OS 255.
    assert once[:10] == GZIP_HEADER == b"\x1f\x8b\x08\x00\x00\x00\x00\x00\x02\xff"
    assert struct.unpack("<II", once[-8:]) == (zlib.crc32(CSV), len(CSV))
    assert gzip.decompress(once) == CSV
    assert gzip_bytes(b"") and gzip.decompress(gzip_bytes(b"")) == b""


def test_core_reads_what_be_report_gzips_and_never_takes_accuracy_gzipped(tmp_path):
    (tmp_path / "samples.csv.gz").write_bytes(gzip_bytes(CSV))
    assert len(rawdata.read_samples(tmp_path / "samples.csv.gz")) == 300
    assert core_report.classify("raw/samples.csv.gz") == ("samples", "be-samples-csv/1")
    assert core_report.classify("raw/accuracy.csv.gz") is None  # so be report leaves it plain
    assert core_report.classify("raw/accuracy.csv") == ("accuracy", "be-accuracy-csv/1")


def _segment(marker: int, payload: bytes) -> bytes:
    return bytes((0xFF, marker)) + (len(payload) + 2).to_bytes(2, "big") + payload


def _jpeg(*, trailer: bytes = b"") -> bytes:
    """A JPEG's marker structure: metadata of every kind, then two scans with stuffing and a
    restart marker. The parser follows markers; it does not decode pixels."""
    return b"".join([
        b"\xff\xd8",
        _segment(0xE0, b"JFIF\x00\x01\x02\x00\x00\x01\x00\x01\x00\x00"),
        _segment(0xE0, b"JFXX\x00\x10thumbnail"),
        _segment(0xE1, b"Exif\x00\x00MM\x00*GPSLatitude 37.77N"),
        _segment(0xE1, b"http://ns.adobe.com/xap/1.0/\x00<x:xmpmeta>owner Ana</x:xmpmeta>"),
        _segment(0xE2, b"ICC_PROFILE\x00\x01\x01colour"),
        _segment(0xE2, b"MPF\x00second image"),
        _segment(0xED, b"Photoshop 3.0\x008BIM city"),
        _segment(0xFE, b"taken at 12 Main St"),
        _segment(0xEE, b"Adobe\x00\x64\x00\x00\x00\x00\x01"),
        _segment(0xDB, b"\x00" + bytes(64)),
        _segment(0xC0, b"\x08\x00\x10\x00\x10\x01\x01\x11\x00"),
        _segment(0xC4, b"\x00" + bytes(16) + b"\x00"),
        _segment(0xDA, b"\x01\x01\x00\x00\x3f\x00"),
        b"\x12\x34\xff\x00\x56\xff\xd0\x78\x9a",
        _segment(0xDA, b"\x01\x01\x00\x00\x3f\x00"),
        b"\xbc\xde\xff\xff\xd9",
        trailer,
    ])


def test_jpeg_metadata_goes_and_the_image_stays():
    src = _jpeg(trailer=b"MotionPhoto video bytes")
    out = strip_jpeg_metadata(src)
    for gone in (b"Exif", b"GPSLatitude", b"xmpmeta", b"MPF", b"8BIM", b"12 Main St", b"JFXX",
                 b"MotionPhoto"):
        assert gone not in out
    for kept in (b"JFIF", b"ICC_PROFILE", b"Adobe", b"\x12\x34\xff\x00\x56\xff\xd0\x78\x9a",
                 b"\xbc\xde"):
        assert kept in out
    assert out.startswith(b"\xff\xd8") and out.endswith(b"\xff\xd9")
    assert strip_jpeg_metadata(out) == out


@pytest.mark.parametrize("bad", [b"", b"\x89PNG\r\n", b"\xff\xd8\xff\xe1\x00\xff",
                                 _jpeg()[:-3], b"\xff\xd8" + _segment(0xDA, b"\x00") + b"\x01\x02"])
def test_jpeg_not_followable_is_refused(bad):
    with pytest.raises(JpegError):
        strip_jpeg_metadata(bad)


def test_a_real_jpeg_decodes_to_the_same_pixels():
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    img = (np.arange(48 * 64 * 3, dtype=np.uint32) % 251).astype(np.uint8).reshape(48, 64, 3)
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 90])
    assert ok
    data = buf.tobytes()
    tagged = data[:2] + _segment(0xE1, b"Exif\x00\x00GPS 37.7N") + data[2:]
    clean = strip_jpeg_metadata(tagged)
    assert b"GPS" not in clean
    before = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    after = cv2.imdecode(np.frombuffer(clean, np.uint8), cv2.IMREAD_COLOR)
    assert after is not None and (before == after).all()


def test_private_keys_and_lfs_pointers_are_recognised(tmp_path):
    private, public = signing.generate_key()
    keygen_file = json.dumps({"kid": signing.thumbprint(public), **private}, indent=2).encode()
    assert looks_like_private_key(keygen_file)
    assert looks_like_private_key(b"-----BEGIN OPENSSH PRIVATE KEY-----\nabc\n")
    assert not looks_like_private_key(json.dumps(public).encode())
    assert not looks_like_private_key(b'{"d": "not a key, no kty", "x": 1}')
    assert looks_like_lfs_pointer(b"version https://git-lfs.github.com/spec/v1\noid sha256:ab\n")
    assert not looks_like_lfs_pointer(b"t_s,point\n")
