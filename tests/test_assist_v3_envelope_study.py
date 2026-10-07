#!/usr/bin/env python3
"""Self-tests for tools/assist_v3_envelope_study.py: every check passes on the candidate design and FAILS on an
injected defect (RULE 70: a check that cannot fail is not evidence)."""
from __future__ import annotations

import subprocess
import sys
import unittest
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import assist_v3_envelope_study as s  # noqa: E402

CSV = ROOT / ".build/envelope_study/legacy_surface.csv"
DUMPER = ROOT / "tests/host/tools/assist_v3_legacy_surface.c"


def ensure_surface() -> None:
    """Rebuild the production legacy surface (host C, production modules) when it is missing."""
    if CSV.exists():
        return
    CSV.parent.mkdir(parents=True, exist_ok=True)
    exe = CSV.parent / "legacy_surface.exe"
    subprocess.run(["gcc", "-std=c11", "-O2", "-Iinc", "-Itests/host/common/host_stubs", "-Itests/host/common",
                    "-o", str(exe), str(DUMPER), "src/g53_port_chain.c", "src/assist_v3_intent.c"],
                   cwd=ROOT, check=True)
    with open(CSV, "w") as f:
        subprocess.run([str(exe)], cwd=ROOT, check=True, stdout=f)


class EnvelopeStudy(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        ensure_surface()
        cls.rows = s.load_surface(CSV)
        cls.profs = s.profiles()

    def test_candidate_design_passes(self):
        res = s.run(CSV, ROOT / ".build/envelope_study/test_out")
        self.assertTrue(s.verdict(res), res)

    def test_replica_rejects_wrong_model(self):
        self.assertTrue(s.check_replica(self.rows)["pass"])
        bad = lambda level, env, cad: s.legacy_iq(level, env, cad) * 1.01
        self.assertFalse(s.check_replica(self.rows, model=bad)["pass"])

    def test_parity_rejects_shifted_default(self):
        profs = dict(self.profs)
        p = profs["SPORT"]
        profs["SPORT"] = replace(p, base=s.Curve(p.base.lo * 1.1, p.base.hi * 1.1))
        res = s.check_default_parity(self.rows, profs)
        self.assertTrue(res["ECO"]["pass"])
        self.assertFalse(res["SPORT"]["pass"])

    def test_monotone_rejects_dip(self):
        def dipping(p, a, env, cad, *args, **kw):
            raw, val, b = s.v3_iq(p, a, env, cad)
            if a == 60:
                raw *= 0.9; val *= 0.9
            return raw, val, b
        self.assertFalse(s.check_monotone_assist(self.rows, {"ECO": self.profs["ECO"]}, model=dipping)["pass"])

    def test_envelope_regions_reject_leaky_power_cap(self):
        orig = s.v3_iq
        def leaky(p, assist, env, cad, max_torque=None, max_power=None, hc_bias=None, overrides=None):
            raw, val, b = orig(p, assist, env, cad, max_torque, None, hc_bias, overrides)
            if max_power is not None:
                val *= 0.97   # touches points far below the power cap
            return raw, val, b
        s.v3_iq = leaky
        try:
            res = s.check_envelope_regions(self.rows, {"SPORT": self.profs["SPORT"]})
        finally:
            s.v3_iq = orig
        self.assertFalse(res["SPORT"]["pass"])

    def test_independence_rejects_duplicate_caps(self):
        orig = s.iq_power_cap
        # A "Max Power" that is really a torque cap: same region, same effect as Max Torque.
        s.iq_power_cap = lambda w, cad: min(s.IQ_MAX, w / 720.0 * s.IQ_MAX)
        try:
            p = replace(self.profs["SPORT"], max_power_w=720.0)
            res = s.check_basic_independence(self.rows, {"SPORT": p})
        finally:
            s.iq_power_cap = orig
        self.assertFalse(res["SPORT"]["pass"], res)


if __name__ == "__main__":
    unittest.main(verbosity=1)
