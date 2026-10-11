# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""File handling for submissions: deterministic gzip, JPEG metadata removal, and the checks for
private keys and Git LFS pointers.

``gzip_bytes`` writes the gzip container itself: a fixed 10-byte header (no file name, no
comment, mtime 0, XFL 2, OS 255 "unknown"), one raw deflate stream at level 9, then CRC-32 and
size. So the bytes depend only on the content and on the zlib that deflates it (zlib and zlib-ng
can choose different, equally valid, streams); ``gzip.compress`` would add the platform's OS byte
on some Python versions.

``strip_jpeg_metadata`` keeps a JPEG's image and drops what can carry location or identity:
APP1 (Exif, with GPS; XMP), APP2 except ICC_PROFILE (MPF secondary images, FlashPix), APP3-APP13
and APP15 (vendor data, Photoshop IPTC), APP0 other than JFIF, COM comments, and anything after
the end-of-image marker (motion-photo video). APP14 (Adobe colour transform) and the ICC
profile stay, so colours decode the same. The pixels are not re-encoded. Exif orientation goes
with Exif, so a photo may show turned.
"""
from __future__ import annotations

import re
import struct
import zlib

__all__ = ["JpegError", "gzip_bytes", "looks_like_lfs_pointer",
           "looks_like_private_key", "strip_jpeg_metadata"]

GZIP_HEADER = b"\x1f\x8b\x08\x00\x00\x00\x00\x00\x02\xff"
"""ID1 ID2, CM deflate, FLG none, MTIME 0, XFL 2 (level 9), OS 255 (unknown)."""

# The results repository's path guard looks for the same things (tools/path_guard.py).
_PEM_PRIVATE_KEY = re.compile(rb"-----BEGIN [A-Z0-9 ]*PRIVATE KEY-----")
_JWK_PRIVATE_D = re.compile(rb'"d"\s*:\s*"[A-Za-z0-9_-]{20,}"')
_JWK_PRIVATE_SEED = re.compile(rb'"priv"\s*:\s*"[A-Za-z0-9_-]{20,}"')
_JWK_KTY = re.compile(rb'"kty"\s*:')
_LFS_POINTER = b"version https://git-lfs.github.com/spec/"
SCAN_BYTES = 1024 * 1024


class JpegError(ValueError):
    """Not a JPEG this module can read."""


def gzip_bytes(data: bytes) -> bytes:
    """``data`` as a gzip file that depends on nothing else (see the module docstring)."""
    deflate = zlib.compressobj(9, zlib.DEFLATED, -15, 8, zlib.Z_DEFAULT_STRATEGY)
    body = deflate.compress(data) + deflate.flush()
    trailer = struct.pack("<II", zlib.crc32(data) & 0xFFFFFFFF, len(data) & 0xFFFFFFFF)
    return GZIP_HEADER + body + trailer


def looks_like_private_key(data: bytes) -> bool:
    """A PEM private key, or a JWK with a private member (``d``, or ``priv`` for ML-DSA), in the
    first MiB."""
    head = data[:SCAN_BYTES]
    if _PEM_PRIVATE_KEY.search(head):
        return True
    return bool(_JWK_KTY.search(head) and (_JWK_PRIVATE_D.search(head) or _JWK_PRIVATE_SEED.search(head)))


def looks_like_lfs_pointer(data: bytes) -> bool:
    return data.startswith(_LFS_POINTER)


def _keep(marker: int, payload: bytes) -> bool:
    if marker == 0xFE:  # COM
        return False
    if marker == 0xE0:  # APP0: JFIF only (JFXX thumbnails, AVI1 and the rest go)
        return payload.startswith(b"JFIF\x00")
    if marker == 0xE2:  # APP2: the ICC colour profile only
        return payload.startswith(b"ICC_PROFILE\x00")
    if marker == 0xEE:  # APP14 Adobe: colour transform, needed to decode
        return True
    return not 0xE1 <= marker <= 0xEF  # APP1, APP3-APP13, APP15


def strip_jpeg_metadata(data: bytes) -> bytes:
    """The JPEG with its metadata segments removed (module docstring); JpegError if ``data``
    is not a JPEG whose markers can be followed to the end of the image."""
    if not data.startswith(b"\xff\xd8"):
        raise JpegError("not a JPEG (it does not start with the SOI marker FF D8)")
    out = bytearray(b"\xff\xd8")
    i, n = 2, len(data)
    while True:
        if i >= n:
            raise JpegError("the file ends before the end-of-image marker")
        if data[i] != 0xFF:
            raise JpegError(f"expected a marker at byte {i}")
        while i < n and data[i] == 0xFF:  # fill bytes
            i += 1
        if i >= n:
            raise JpegError("the file ends inside a marker")
        marker = data[i]
        i += 1
        if marker == 0xD9:  # EOI: anything after it is dropped
            out += b"\xff\xd9"
            return bytes(out)
        if 0xD0 <= marker <= 0xD7 or marker == 0x01:  # RSTn, TEM: no length
            out += bytes((0xFF, marker))
            continue
        if i + 2 > n:
            raise JpegError(f"the segment FF {marker:02X} has no length")
        length = int.from_bytes(data[i:i + 2], "big")
        if length < 2 or i + length > n:
            raise JpegError(f"the segment FF {marker:02X} at byte {i - 2} runs past the file")
        segment = data[i:i + length]
        i += length
        if _keep(marker, segment[2:]):
            out += bytes((0xFF, marker)) + segment
        if marker != 0xDA:  # not SOS
            continue
        # Entropy-coded data runs to the next marker that is not a stuffed FF 00 or an RSTn.
        j = i
        while True:
            k = data.find(b"\xff", j)
            if k < 0 or k + 1 >= n:
                raise JpegError("the file ends inside the image data")
            nxt = data[k + 1]
            if nxt == 0x00 or 0xD0 <= nxt <= 0xD7:
                j = k + 2
                continue
            if nxt == 0xFF:  # fill byte before a marker
                j = k + 1
                continue
            break
        out += data[i:k]
        i = k
