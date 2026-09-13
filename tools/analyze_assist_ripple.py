#!/usr/bin/env python3
"""THE most important assist property, measured rather than asserted.

Pedal torque pulsates twice per crank revolution. The whole point of the demand model is that
the motor current must NOT pulsate with it: Iq has to represent the assist the rider intended,
not every individual leg push.

This reports, for each scenario and over the STEADY part of the trace only (the start transient
is skipped, because a rise from zero is not ripple), the peak-to-peak swing of each signal as a
fraction of its own mean. The number that matters is the last column:

    attenuation = iq ripple / torque ripple

Below 1.0 the motor is smoother than the pedal. Around 1.0 the motor is copying the pedal, which
is the defect this pipeline exists to remove.

Usage: analyze_assist_ripple.py [regression-dir] [--skip-s N] [--max-attenuation X]
"""
from __future__ import annotations
import argparse
import csv
import statistics
import sys
from pathlib import Path

R = Path(__file__).resolve().parents[1]
SCENARIOS = ['CRUISE_60', 'CRUISE_80', 'CRUISE_100']
TICK_HZ = 4000.0


def ripple(vals):
    vals = [v for v in vals if v == v]
    if not vals:
        return None
    mean = sum(vals) / len(vals)
    if abs(mean) < 1e-9:
        return None
    return dict(mean=mean, p2p=max(vals) - min(vals),
                ripple=(max(vals) - min(vals)) / abs(mean),
                std=statistics.pstdev(vals))


def col(rows, name):
    out = []
    for r in rows:
        try:
            out.append(float(r[name]))
        except (KeyError, TypeError, ValueError):
            out.append(float('nan'))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dir', nargs='?', default=str(R / '.build/regression-linux'))
    ap.add_argument('--skip-s', type=float, default=3.0,
                    help='seconds of start transient to exclude')
    ap.add_argument('--max-attenuation', type=float, default=0.60,
                    help='fail if iq ripple / torque ripple exceeds this in any scenario')
    a = ap.parse_args()
    d = Path(a.dir)
    skip = int(a.skip_s * TICK_HZ)

    print('Steady-state ripple (peak-to-peak / mean), start transient excluded')
    print(f'{"scenario":<12}{"torque":>9}{"demand":>9}{"base":>9}{"dynamic":>9}'
          f'{"iq":>9}{"attenuation":>13}')
    worst = 0.0
    worst_scenario = ''
    rows_seen = 0
    for sc in SCENARIOS:
        path = d / f'{sc}_assist.csv'
        if not path.exists():
            continue
        rows = list(csv.DictReader(open(path, newline='')))[skip:]
        if not rows:
            continue
        rows_seen += 1
        tq = ripple(col(rows, 'torque_fast'))
        dem = ripple(col(rows, 'rider_demand'))
        base = ripple(col(rows, 'assist_base'))
        dyn = ripple(col(rows, 'assist_dynamic'))
        iq = ripple(col(rows, 'iq_final'))
        if not (tq and iq):
            continue
        att = iq['ripple'] / tq['ripple'] if tq['ripple'] else float('inf')
        # A response pinned at its ceiling has no ripple because it is CLIPPED, which would make
        # this whole measurement pass for the wrong reason. Refuse to report it as a result.
        resp = col(rows, 'assist_response')
        clipped = sum(1 for v in resp if v >= 999) / float(len(resp))
        if clipped > 0.10:
            print(f'{sc:<12}  SATURATED: assist_response is at its ceiling for '
                  f'{clipped*100:.0f} % of the trace - ripple here would be clipping, '
                  f'not attenuation', file=sys.stderr)
            return 2
        if att > worst:
            worst, worst_scenario = att, sc
        print(f'{sc:<12}{tq["ripple"]:>9.3f}'
              f'{(dem["ripple"] if dem else float("nan")):>9.3f}'
              f'{(base["ripple"] if base else float("nan")):>9.3f}'
              f'{(dyn["ripple"] if dyn else float("nan")):>9.3f}'
              f'{iq["ripple"]:>9.3f}{att:>13.3f}')

    if rows_seen == 0:
        print('no assist traces found - run tools/run_regression.py first', file=sys.stderr)
        return 2

    print()
    print(f'worst attenuation: {worst:.3f} ({worst_scenario}), limit {a.max_attenuation:.3f}')
    if worst > a.max_attenuation:
        print('FAIL: the motor is reproducing the pedal ripple instead of the rider intent',
              file=sys.stderr)
        return 1
    print('PASS: Iq ripple stays well below pedal ripple in every steady scenario')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
