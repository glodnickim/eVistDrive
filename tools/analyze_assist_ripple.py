#!/usr/bin/env python3
"""THE most important assist property, measured rather than asserted.

Pedal torque pulsates twice per crank revolution. The whole point of the demand model is that
the motor current must NOT pulsate with it: Iq has to represent the assist the rider intended,
not every individual leg push.

This reports, for each scenario and over the STEADY part of the trace only, the peak-to-peak
swing of each signal as a fraction of its own mean, and the ratio that matters:

    attenuation = iq ripple / torque ripple

Below the limit the motor is smoother than the pedal. Around 1.0 the motor is copying the pedal,
which is the defect this pipeline exists to remove.

WHAT THIS TOOL REFUSES TO CALL A PASS
-------------------------------------
A quality measurement that can be satisfied by bad data is not a measurement. Every one of these
is a FAILURE, not a skip:

  * a scenario is missing, or its trace has no steady part;
  * any signal it needs is absent or non-finite;
  * the response is saturated at its ceiling - a clipped signal is smooth because it is
    clipped, and reporting that as attenuation would pass for entirely the wrong reason;
  * the response is zero, or the final current is zero - there is no assist to be smooth;
  * a limiter was binding for a large part of the trace - then the current is being shaped by a
    protection rather than by the demand model, and the number says nothing about the model.

Usage: analyze_assist_ripple.py [regression-dir] [--skip-s N] [--max-attenuation X]
"""
from __future__ import annotations
import argparse
import csv
import statistics
import sys
from pathlib import Path

R = Path(__file__).resolve().parents[1]

# Every scenario is REQUIRED. A missing one is a failure, never a silent skip: the specification
# is the whole cadence range and every profile, not whichever traces happened to be produced.
SCENARIOS = [
    'CRUISE_20_ECO', 'CRUISE_20_TRAIL', 'CRUISE_20_SPORT', 'CRUISE_20_SPORTPLUS',
    'CRUISE_40_ECO', 'CRUISE_40_TRAIL', 'CRUISE_40_SPORT', 'CRUISE_40_SPORTPLUS',
    'CRUISE_60_ECO', 'CRUISE_60_TRAIL', 'CRUISE_60_SPORT', 'CRUISE_60_SPORTPLUS',
    'CRUISE_80_SPORT', 'CRUISE_100_SPORT',
]
TICK_HZ = 4000.0

# Above this share of samples the trace is describing a limiter or a ceiling, not the model.
MAX_SATURATED_SHARE = 0.10
MAX_LIMITED_SHARE = 0.10

REQUIRED_COLUMNS = ('torque_fast', 'rider_demand', 'assist_base', 'assist_dynamic',
                    'assist_response', 'iq_request', 'iq_final')


class Reject(Exception):
    """The evidence is not usable. Never downgraded to a warning."""


def column(rows, name):
    if not rows or name not in rows[0]:
        raise Reject(f'column {name!r} is missing from the trace')
    out = []
    for r in rows:
        try:
            v = float(r[name])
        except (TypeError, ValueError):
            raise Reject(f'column {name!r} contains a non-numeric value {r[name]!r}')
        if v != v or v in (float('inf'), float('-inf')):
            raise Reject(f'column {name!r} contains a non-finite value')
        out.append(v)
    return out


def ripple(vals, name):
    mean = sum(vals) / len(vals)
    if abs(mean) < 1e-9:
        raise Reject(f'{name} is zero throughout the steady window - there is nothing to measure')
    return dict(mean=mean, p2p=max(vals) - min(vals),
                ripple=(max(vals) - min(vals)) / abs(mean),
                std=statistics.pstdev(vals))


def share_at_or_above(vals, threshold):
    return sum(1 for v in vals if v >= threshold) / float(len(vals))


def evaluate(path: Path, skip: int):
    rows = list(csv.DictReader(open(path, newline='')))
    if not rows:
        raise Reject('the trace is empty')
    rows = rows[skip:]
    if len(rows) < 1000:
        raise Reject(f'only {len(rows)} steady samples after the start transient')

    for col in REQUIRED_COLUMNS:
        column(rows, col)

    resp = column(rows, 'assist_response')
    saturated = share_at_or_above(resp, 999)
    if saturated > MAX_SATURATED_SHARE:
        raise Reject(f'assist_response is at its ceiling for {saturated * 100:.0f} % of the '
                     f'trace - ripple here would be clipping, not attenuation')

    iq = column(rows, 'iq_final')
    if max(iq) <= 0:
        raise Reject('iq_final is zero throughout - no assist was produced to be smooth')

    req = column(rows, 'iq_request')
    # A request that the chain cut down is a limiter result, not a demand-model result.
    limited = sum(1 for a, b in zip(req, iq) if a - b > max(1.0, 0.02 * a)) / float(len(req))
    if limited > MAX_LIMITED_SHARE:
        raise Reject(f'a limiter was binding for {limited * 100:.0f} % of the trace - the '
                     f'current is being shaped by a protection, not by the demand model')

    tq = ripple(column(rows, 'torque_fast'), 'torque_fast')
    dem = ripple(column(rows, 'rider_demand'), 'rider_demand')
    base = ripple(column(rows, 'assist_base'), 'assist_base')
    dyn_vals = column(rows, 'assist_dynamic')
    dyn_mean = sum(dyn_vals) / len(dyn_vals)
    iqr = ripple(iq, 'iq_final')
    return dict(torque=tq['ripple'], demand=dem['ripple'], base=base['ripple'],
                dynamic_mean=dyn_mean, iq=iqr['ripple'],
                attenuation=iqr['ripple'] / tq['ripple'],
                saturated=saturated, limited=limited)


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
    print(f'{"scenario":<22}{"torque":>9}{"demand":>9}{"base":>9}{"iq":>9}'
          f'{"attenuation":>13}{"sat%":>7}{"lim%":>7}')

    rejects = []
    worst = 0.0
    worst_scenario = ''
    for sc in SCENARIOS:
        path = d / f'{sc}_assist.csv'
        if not path.exists():
            rejects.append((sc, f'trace {path.name} was not produced'))
            print(f'{sc:<22}  REJECTED: trace not produced')
            continue
        try:
            m = evaluate(path, skip)
        except Reject as e:
            rejects.append((sc, str(e)))
            print(f'{sc:<22}  REJECTED: {e}')
            continue
        if m['attenuation'] > worst:
            worst, worst_scenario = m['attenuation'], sc
        print(f'{sc:<22}{m["torque"]:>9.3f}{m["demand"]:>9.3f}{m["base"]:>9.3f}'
              f'{m["iq"]:>9.3f}{m["attenuation"]:>13.3f}'
              f'{m["saturated"] * 100:>7.0f}{m["limited"] * 100:>7.0f}')

    print()
    if rejects:
        print(f'FAIL: {len(rejects)} of {len(SCENARIOS)} scenarios produced no usable evidence',
              file=sys.stderr)
        for sc, why in rejects:
            print(f'  {sc}: {why}', file=sys.stderr)
        return 1

    print(f'worst attenuation: {worst:.3f} ({worst_scenario}), limit {a.max_attenuation:.3f}')
    if worst > a.max_attenuation:
        print('FAIL: the motor is reproducing the pedal ripple instead of the rider intent',
              file=sys.stderr)
        return 1
    print(f'PASS: all {len(SCENARIOS)} scenarios measured; Iq ripple stays well below pedal '
          f'ripple in every one')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
