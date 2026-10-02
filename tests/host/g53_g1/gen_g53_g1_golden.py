#!/usr/bin/env python3
"""TASK-EVD-TQ-06-G1: golden digests for src/g53_g1_limiter.c.

The expected behaviour is NOT re-derived here. It is the Python transcription of the original
G5300 machine code in integration/evidence/evd-tq/TQ-02B-N4/tools/tq02b_n4.py, which was
differentially tested against Unicorn execution of the original image (8 functions x 100 000
vectors, 0 mismatches; independent blind check by the reviewer; TASK-EVD-TQ-02B-N4 = PASS).

Both this script and tests/host/g53_g1_limiter_host.c draw the SAME vectors from the same
xorshift32 stream (the spec below is the contract between them). This script runs the pinned
models on each vector and records an FNV-1a-64 digest per block of BLOCK vectors; the harness
recomputes the digests with the C module. A digest per block localizes any mismatch.

Usage (dev-time only; needs the integration repo next to this one, capstone + unicorn):
    python tests/host/g53_g1/gen_g53_g1_golden.py > tests/host/g53_g1/g53_g1_golden.h
"""
import hashlib
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
N4_TOOLS = os.path.normpath(os.path.join(HERE, "..", "..", "..", "..", "integration", "evidence",
                                         "evd-tq", "TQ-02B-N4", "tools"))
TQ02B_TOOLS = os.path.normpath(os.path.join(N4_TOOLS, "..", "..", "TQ-02B", "tools"))
sys.path.insert(0, N4_TOOLS)
sys.path.insert(0, TQ02B_TOOLS)
import tq02b_n4 as N4                     # noqa: E402
from tq02b_d7ec import Buf                # noqa: E402

PINNED_MODEL_SHA256 = "2242c3e19f0a348895f0d59d3dc9c7612cbeffc96d47cbe4fc5086b55ca1f8a3"
SEED = 0x1C5BA6B4
VECTORS = 100000
BLOCK = 1000
NB = 0x60
M32 = 0xFFFFFFFF
SPEC = [0, 1, 0x1000, 0x7FFF, 0x8000, 0xFFFF, 100, 500, 1900]


class Rng:
    def __init__(self, seed):
        self.x = seed & M32

    def nxt(self):
        x = self.x
        x ^= (x << 13) & M32
        x ^= x >> 17
        x ^= (x << 5) & M32
        self.x = x
        return x

    def rnd(self, n):
        return self.nxt() % n

    def r16(self):
        k = self.rnd(12)
        if k == 0:
            return self.nxt() & 0xFFFF
        if k == 1:
            return self.rnd(5000)
        if k == 2:
            return (self.rnd(400) - 200) & 0xFFFF
        return SPEC[k - 3]


def put16(img, off, v):
    img[off] = v & 0xFF
    img[off + 1] = (v >> 8) & 0xFF


def put32(img, off, v):
    for i in range(4):
        img[off + i] = (v >> (8 * i)) & 0xFF


def fnv(h, data):
    for b in data:
        h ^= b
        h = (h * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return h


def new_mem(img):
    R = Buf(bytes(N4.WIN_SZ), N4.WIN_LO)
    for i, b in enumerate(img):
        R.w8(N4.N + i, b)
    return R


def image(R):
    return bytes(R.u8(N4.N + i) for i in range(NB))


def vec_cfg1(rng):
    img = bytearray(rng.nxt() & 0xFF for _ in range(NB))
    f = [rng.r16() for _ in range(12)]
    m314 = rng.r16()
    R = new_mem(img)
    for off, v in zip((0xF6, 0xF8, 0xFA, 0xFC, 0xFE, 0x100, 0x102, 0x104, 0x106, 0x108, 0x1EE, 0x288), f):
        R.w16(N4.M + off, v)
    R.w16(N4.M + 0x314, m314)
    N4.cfg1_model(R)
    return image(R) + R.u16(N4.M + 0x314).to_bytes(2, "little")


def vec_lim(rng):
    img = bytearray(rng.nxt() & 0xFF for _ in range(NB))
    if rng.rnd(2) == 0:
        put16(img, 0x0C, rng.rnd(4000))
        put16(img, 0x0E, rng.rnd(4000))
        put16(img, 0x10, rng.rnd(3000))
        put16(img, 0x12, rng.rnd(3000))
    k = rng.rnd(4)
    q50 = [6, 7, 4][k] if k < 3 else rng.rnd(256)
    pcts = []
    for _ in range(2):
        k = rng.rnd(5)
        pcts.append([0, 15, 40, 100][k] if k < 4 else rng.nxt() & 0xFFFF)
    k = rng.rnd(4)
    sel = [0, 1, 2][k] if k < 3 else rng.rnd(256)
    if rng.rnd(2) == 0:
        soc, fa, fb = rng.rnd(0x1001), rng.rnd(0x1001), rng.rnd(0x1001)
    else:
        soc, fa, fb = rng.r16(), rng.r16(), rng.r16()
    R = new_mem(img)
    R.w8(N4.Q + 0x50, q50)
    R.w16(N4.H + 0x5E, pcts[0])
    R.w16(N4.H + 0x62, pcts[1])
    R.w8(N4.M + 0x2ED, sel)
    R.w16(N4.E + 0x36, soc)
    R.w16(N4.M + 0x2C0, fa)
    R.w16(N4.M + 0x2C2, fb)
    N4.lim_model(R)
    return image(R) + R.u16(N4.M + 0x54).to_bytes(2, "little")


def vec_pi1(rng):
    img = bytearray(rng.nxt() & 0xFF for _ in range(NB))
    if rng.rnd(2) == 0:
        put16(img, 0x34, [2048, 64][rng.rnd(2)])
        put16(img, 0x36, [200, 96][rng.rnd(2)])
        put16(img, 0x38, [500, 0][rng.rnd(2)])
        put32(img, 0x3C, 4096 << 12)
        put32(img, 0x40, (-(4096 << 12)) & M32)
        put16(img, 0x44, 4096)
        put16(img, 0x46, (-4096) & 0xFFFF)
        put32(img, 0x50, (rng.rnd(1 << 25) - (1 << 24)) & M32)
        put16(img, 0x1C, rng.rnd(3000))
    if rng.rnd(4) == 0:                    # review F-03: floor == limit (0x0800C6FE movle edge)
        img[0x18] = img[0x1C]
        img[0x19] = img[0x1D]
    k = rng.rnd(3)
    if k == 0:
        taper = 0x1000
    elif k == 1:
        taper = rng.nxt() & 0xFFFF
    else:
        taper = rng.rnd(0x1001)
    fb = rng.r16()
    R = new_mem(img)
    R.w16(N4.M + 0x2A4, taper)
    R.w16(N4.M + 0x1C, fb)
    N4.pi1_model(R)
    return image(R)


def main():
    model = os.path.join(N4_TOOLS, "tq02b_n4.py")
    sha = hashlib.sha256(open(model, "rb").read()).hexdigest()
    if sha != PINNED_MODEL_SHA256:
        raise SystemExit("pinned model changed: %s" % sha)
    out = ["/* GENERATED by tests/host/g53_g1/gen_g53_g1_golden.py - do not edit.",
           " * Model: integration/evidence/evd-tq/TQ-02B-N4/tools/tq02b_n4.py sha256 %s */" % sha,
           "#ifndef G53_G1_GOLDEN_H", "#define G53_G1_GOLDEN_H", "#include <stdint.h>",
           "#define G53_G1_GOLDEN_SEED 0x%08Xu" % SEED,
           "#define G53_G1_GOLDEN_VECTORS %d" % VECTORS,
           "#define G53_G1_GOLDEN_BLOCK %d" % BLOCK]
    for name, fn in (("CFG1", vec_cfg1), ("LIM", vec_lim), ("PI1", vec_pi1)):
        rng = Rng(SEED ^ {"CFG1": 0x11111111, "LIM": 0x22222222, "PI1": 0x33333333}[name])
        digests = []
        h = 0xCBF29CE484222325
        for i in range(VECTORS):
            h = fnv(h, fn(rng))
            if (i + 1) % BLOCK == 0:
                digests.append(h)
                h = 0xCBF29CE484222325
        out.append("static const uint64_t g53_g1_golden_%s[%d] = {" % (name.lower(), len(digests)))
        for j in range(0, len(digests), 4):
            out.append("    " + ", ".join("0x%016XULL" % d for d in digests[j:j + 4]) + ",")
        out.append("};")
    out.append("#endif")
    print("\n".join(out))


if __name__ == "__main__":
    main()
