#!/usr/bin/env python3
"""Self-tests for tools/assist_v3_acceptance.py: a good synthetic candidate is accepted, and each kind of
regression is rejected (RULE 70: a gate that cannot fail is not evidence)."""
from __future__ import annotations

import csv
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import assist_v3_acceptance as A  # noqa: E402

GOOD = [
    ("true_release", 60, "release_latency_50_ms", 1950, 83),
    ("steady", 60, "phase_dip_fp_rate", 0, 0),
    ("steady", 60, "ripple_rev_median", 0.15, 0.04),
    ("steady", 20, "ripple_rev_median", 0.77, 0.04),
    ("climb", 120, "ripple_rev_median", "REFUSED", "REFUSED"),
    ("attack", 60, "attack_t63_ms", 490, 480),
    ("coast", 25, "safety_ref0_ms", 85, 86),
    ("brake", 60, "safety_ref0_ms", 196, 199),
    ("attack_stop_restart", 30, "restart_dip_frac", 0.01, 0.01),
    ("attack_stop_restart", 30, "restart_first_iq_ms", 81, 73),
    ("steady", 60, "level_mean_iq", 177.5, 177.2),
    ("coast", 60, "carry_activations", "N/A", 0),
    ("steady", 60, "carry_active_during_cut_rows", "N/A", 0),
]


def write(rows) -> Path:
    f = tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False, newline="")
    w = csv.writer(f)
    w.writerow(["scenario", "profile", "cadence", "metric", "baseline", "candidate", "delta", "baseline_check",
                "candidate_check"])
    for prof, cad, metric, b, c in rows:
        w.writerow([f"{prof}_{cad}", prof, cad, metric, b, c, "", "", ""])
    f.close()
    return Path(f.name)


def run(rows) -> bool:
    return A.verdict(A.criteria(A.load(write(rows))))


def mutate(index, value):
    rows = list(GOOD)
    prof, cad, metric, b, _ = rows[index]
    rows[index] = (prof, cad, metric, b, value)
    return rows


class Acceptance(unittest.TestCase):
    def test_good_candidate_accepted(self):
        self.assertTrue(run(GOOD))

    def test_rejects_slow_release(self):
        self.assertFalse(run(mutate(0, 400)))

    def test_rejects_phase_dip(self):
        self.assertFalse(run(mutate(1, 0.05)))

    def test_rejects_ripple_regression(self):
        self.assertFalse(run(mutate(2, 0.30)))

    def test_rejects_low_cadence_ripple_target(self):
        self.assertFalse(run(mutate(3, 0.26)))

    def test_rejects_missing_value(self):
        self.assertFalse(run(mutate(2, "REFUSED")))

    def test_rejects_attack_change(self):
        self.assertFalse(run(mutate(5, 700)))

    def test_rejects_slower_stop(self):
        self.assertFalse(run(mutate(6, 90)))

    def test_rejects_restart_dip(self):
        self.assertFalse(run(mutate(8, 0.10)))

    def test_rejects_level_shortfall(self):
        self.assertFalse(run(mutate(10, 160.0)))

    def test_rejects_wrong_carry(self):
        self.assertFalse(run(mutate(11, 1)))

    def test_rejects_carry_during_cut(self):
        self.assertFalse(run(mutate(12, 3)))


if __name__ == "__main__":
    unittest.main(verbosity=1)
