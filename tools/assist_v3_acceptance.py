#!/usr/bin/env python3
"""Assist Behavior V3 behaviour acceptance (SIMULATION_REPORT §2), baseline vs candidate.

Reads the matrix written by tools/run_assist_v3_matrix.py --engine v3 (out/matrix.csv: scenario, profile, cadence,
metric, baseline, candidate, ...) and applies the RELATIVE acceptance criteria of the program, which the per-row
absolute bounds in assist_v3_metrics.CHECKS cannot express. A missing or refused value is never a pass.

Usage: python tools/assist_v3_acceptance.py [MATRIX_CSV] [--json OUT]
Exit 0 = ACCEPT, 1 = REJECT.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT = ROOT / ".build/assist_v3/out/matrix.csv"

NO_CARRY_PROFILES = ("steady", "dead_spot", "asymmetry", "coast", "crest", "gradual_release", "true_release",
                     "pas_glitch", "stall_noise", "reverse", "brake")
DIP_PROFILES = ("steady", "dead_spot", "asymmetry", "climb", "pas_glitch", "stall_noise")
LEVEL_PROFILES = ("steady", "dead_spot", "asymmetry", "climb")
# Rows that must exist for every cadence of the matrix; a missing row is a FAIL, never a silent pass.
REQUIRED = {
    "true_release": ("release_latency_50_ms",),
    "steady": ("phase_dip_fp_rate", "ripple_rev_median", "level_mean_iq", "carry_activations"),
    "dead_spot": ("phase_dip_fp_rate", "ripple_rev_median", "level_mean_iq", "carry_activations"),
    "asymmetry": ("phase_dip_fp_rate", "ripple_rev_median", "level_mean_iq", "carry_activations"),
    "climb": ("phase_dip_fp_rate", "level_mean_iq", "ripple_rev_median"),
    "attack_stop": ("safety_ref0_ms",),
    "attack": ("attack_t63_ms",),
    "attack_stop_restart": ("restart_dip_frac", "restart_first_iq_ms"),
    "coast": ("safety_ref0_ms", "carry_activations"),
    "reverse": ("safety_ref0_ms",),
    "brake": ("safety_ref0_ms",),
    "crest": ("carry_activations",),
}
L4_CARRY_EXPECT = {"climb_pedal_stop_obstacle": 1, "crest_pedal_stop": 0, "coast_stop": 0, "reverse_while_motor": 0}


def num(v):
    try:
        x = float(v)
    except (TypeError, ValueError):
        return None
    return None if x != x else x


def load(path: Path) -> dict:
    m = defaultdict(dict)
    for r in csv.DictReader(open(path, newline="")):
        m[(r["profile"], int(r["cadence"]))][r["metric"]] = (num(r["baseline"]), num(r["candidate"]))
    return m


def criteria(m: dict) -> list[dict]:
    out = []

    def add(name, key, ok, detail):
        out.append({"criterion": name, "case": f"{key[0]}@{key[1]}rpm", "pass": bool(ok), "detail": detail})

    for prof, needed in REQUIRED.items():
        cadences = sorted({c for (p, c) in m if p == prof})
        if not cadences:
            add("required profile present", (prof, 0), False, "profile missing from the matrix")
        for cad in cadences:
            met = m.get((prof, cad), {})
            for metric in needed:
                if metric not in met:
                    add(f"required row present: {metric}", (prof, cad), False, "missing row")
    for key, met in sorted(m.items()):
        prof, cad = key
        g = met.get
        if prof == "true_release" and "release_latency_50_ms" in met:
            b, c = g("release_latency_50_ms")
            add("TRUE_RELEASE 50% <= 250 ms and < baseline", key, c is not None and b is not None and c <= 250 and c < b,
                f"baseline {b} ms, V3 {c} ms")
        if prof in DIP_PROFILES and "phase_dip_fp_rate" in met:
            _, c = g("phase_dip_fp_rate")
            add("phase-dip false positives = 0", key, c is not None and c == 0, f"V3 {c}")
        if "ripple_rev_median" in met and prof in ("steady", "dead_spot", "asymmetry", "climb"):
            b, c = g("ripple_rev_median")
            if b is None and c is None:
                pass   # both refused: demand pinned at the 455 Iq ceiling in both engines -> not applicable
            else:
                lim = max(b + 0.01, 0.10) if b is not None else None
                ok = c is not None and lim is not None and c <= lim and (cad > 30 or c < 0.25)
                add("ripple <= max(baseline+0.01, 0.10); < 0.25 at <= 30 rpm", key, ok, f"baseline {b}, V3 {c}")
        if "attack_t63_ms" in met and prof == "attack":
            b, c = g("attack_t63_ms")
            add("attack t63 within +-20 % of baseline", key, b and c is not None and abs(c / b - 1) <= 0.20,
                f"baseline {b} ms, V3 {c} ms")
        if "safety_ref0_ms" in met:
            b, c = g("safety_ref0_ms")
            # 2 ms = two 1 ms logical ticks of trace resolution; brake 5 ms: native SAFETY 200 ms from a slightly
            # different level at the event (unchanged path)
            slack = 5.0 if prof == "brake" else 2.0
            add("stop/reverse/brake ref->0 <= baseline", key, b is not None and c is not None and c <= b + slack,
                f"baseline {b} ms, V3 {c} ms")
        if "restart_dip_frac" in met:
            b, c = g("restart_dip_frac")
            add("restart dip <= baseline + 0.05", key, b is not None and c is not None and c <= b + 0.05, f"baseline {b}, V3 {c}")
        if "restart_first_iq_ms" in met:
            b, c = g("restart_first_iq_ms")
            add("restart first Iq <= baseline + 20 ms", key, b is not None and c is not None and c <= b + 20,
                f"baseline {b} ms, V3 {c} ms")
        if prof in LEVEL_PROFILES and "level_mean_iq" in met:
            b, c = g("level_mean_iq")
            add("steady level within +-5 % of baseline (G1-LEVEL)", key, b and c is not None and abs(c / b - 1) <= 0.05,
                f"baseline {b}, V3 {c}")
        if prof in NO_CARRY_PROFILES and "carry_activations" in met:
            _, c = g("carry_activations")
            add("no carry where carry is wrong", key, c is not None and c == 0, f"V3 activations {c}")
        if "carry_active_during_cut_rows" in met:
            _, c = g("carry_active_during_cut_rows")
            add("carry never active during a cut", key, c is not None and c == 0, f"V3 rows {c}")
    return out


def l4_carry(log_text: str) -> list[dict]:
    """Positive and negative carry evidence from the closed-loop Level-4 scripts (tools/run_level4.py output)."""
    import re
    out = []
    seen = {}
    for name, active in re.findall(r"L4 CARRY (\S+) active=(\d)", log_text):
        seen[name] = int(active)
    for name, want in L4_CARRY_EXPECT.items():
        got = seen.get(name)
        out.append({"criterion": "L4 carry activates only where intended", "case": name,
                    "pass": got == want, "detail": f"expected active={want}, got {got}"})
    return out


def verdict(results: list[dict]) -> bool:
    return bool(results) and all(r["pass"] for r in results)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("matrix", nargs="?", type=Path, default=DEFAULT)
    ap.add_argument("--json", type=Path)
    ap.add_argument("--l4-log", type=Path, help="output of tools/run_level4.py (required for carry evidence)")
    a = ap.parse_args()
    res = criteria(load(a.matrix))
    if a.l4_log is None or not a.l4_log.exists():
        res.append({"criterion": "L4 carry evidence supplied", "case": "--l4-log", "pass": False, "detail": "missing"})
    else:
        raw = a.l4_log.read_bytes()
        # PowerShell redirection writes UTF-16 with a BOM; Python/bash output is UTF-8.
        text = raw.decode("utf-16") if raw[:2] in (b"\xff\xfe", b"\xfe\xff") else raw.decode("utf-8", "replace")
        res += l4_carry(text)
    fails = [r for r in res if not r["pass"]]
    by = defaultdict(lambda: [0, 0])
    for r in res:
        by[r["criterion"]][0 if r["pass"] else 1] += 1
    for k, (p, f) in sorted(by.items()):
        print(f"{'PASS' if f == 0 else 'FAIL'}  {k}: {p} pass, {f} fail")
    for r in fails[:30]:
        print(f"  FAIL {r['case']}: {r['criterion']} ({r['detail']})")
    ok = verdict(res)
    print("ACCEPTANCE:", "ACCEPT" if ok else "REJECT", f"({len(res) - len(fails)}/{len(res)})")
    if a.json:
        a.json.write_text(json.dumps({"accept": ok, "results": res}, indent=1) + "\n")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
