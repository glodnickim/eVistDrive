#!/usr/bin/env python3
"""Quantitative behaviour criteria for a replayed ride.

WHY THIS IS SEPARATE FROM "THE REPLAY RAN".

A replay that completes proves the production chain can consume a real sensor history without
crashing. That is worth having and it is not a statement about the assist being any good. The
runner used to print CASE PASS for exactly that, which reads like a quality verdict and is not
one - audit finding 8.

So the two are reported separately and named differently:

    REPLAY_EXECUTED    the chain consumed the capture and produced a trace
    BEHAVIOR_ACCEPTED  the trace satisfies stated, quantitative criteria

A case with no criteria gets REPLAY_EXECUTED and an explicit BEHAVIOR_NOT_ASSESSED. It is never
called a pass.

WHAT IS NOT DONE HERE. None of these criteria says the assist FEELS right; that needs the bike.
They say the chain responded to the rider in the direction and on the timescale it claims to -
which is the part a recording can actually settle.
"""
from __future__ import annotations
import csv
import math
from pathlib import Path


class BehaviorResult:
    def __init__(self):
        self.checks: list[tuple[str, bool, str]] = []

    def add(self, name: str, ok: bool, detail: str):
        self.checks.append((name, ok, detail))

    @property
    def ok(self) -> bool:
        return all(c[1] for c in self.checks)

    @property
    def assessed(self) -> bool:
        return bool(self.checks)


def _col(rows, name):
    out = []
    for r in rows:
        try:
            v = float(r[name])
        except (KeyError, TypeError, ValueError):
            v = math.nan
        out.append(v)
    return out


def _finite(vals):
    return [v for v in vals if isinstance(v, float) and v == v]


def _mean(vals):
    f = _finite(vals)
    return sum(f) / len(f) if f else math.nan


def _ripple(vals):
    f = _finite(vals)
    if not f:
        return math.nan
    m = sum(f) / len(f)
    if abs(m) < 1e-9:
        return math.nan
    return (max(f) - min(f)) / abs(m)


def evaluate(replayed_csv: Path, spec: dict) -> BehaviorResult:
    res = BehaviorResult()
    if not spec:
        return res
    rows = list(csv.DictReader(open(replayed_csv, newline='')))
    if not rows:
        res.add('trace', False, 'the replayed trace is empty')
        return res

    iq = _col(rows, 'iq_ref_new')
    req = _col(rows, 'iq_request_new')
    torque = _col(rows, 'torque_ckg')
    cadence = _col(rows, 'cadence_rpm')

    if 'produces_assist' in spec:
        want = bool(spec['produces_assist'])
        got = any(v > 0 for v in _finite(iq))
        res.add('produces_assist', got == want,
                f'current present in the trace: {got}, expected {want}')

    if 'responds_to_load' in spec:
        # The rider's effort rises through the fragment; the assist must follow it. Compared as
        # halves rather than sample by sample: the model deliberately does NOT track each stroke.
        half = len(rows) // 2
        t1, t2 = _mean(torque[:half]), _mean(torque[half:])
        i1, i2 = _mean(iq[:half]), _mean(iq[half:])
        rising = spec['responds_to_load'] == 'rising'
        load_moved = (t2 > t1) if rising else (t2 < t1)
        iq_followed = (i2 > i1) if rising else (i2 < i1)
        res.add('responds_to_load', load_moved and iq_followed,
                f'torque {t1:.0f}->{t2:.0f}, iq {i1:.0f}->{i2:.0f}, expected {spec["responds_to_load"]}')

    if 'pause_releases' in spec:
        # Somewhere in the fragment the rider stops. The current must actually reach zero, and
        # stay there for a meaningful stretch - not merely dip.
        idle = [i for i, c in enumerate(cadence) if c == c and c <= 0.0]
        zero_during_idle = sum(1 for i in idle if iq[i] == iq[i] and iq[i] <= 0.0)
        frac = zero_during_idle / len(idle) if idle else 0.0
        need = float(spec['pause_releases'])
        res.add('pause_releases', bool(idle) and frac >= need,
                f'{len(idle)} idle samples, {frac * 100:.0f} % at zero current, need {need * 100:.0f} %')

    if 'restart_recovers' in spec:
        # After the pause the rider pedals again and the assist must come back.
        idle = [i for i, c in enumerate(cadence) if c == c and c <= 0.0]
        if idle:
            after = idle[-1] + 1
            tail = _finite(iq[after:])
            res.add('restart_recovers', bool(tail) and max(tail) > 0,
                    f'{len(tail)} samples after the pause, peak current {max(tail) if tail else 0:.0f}')
        else:
            res.add('restart_recovers', False, 'no pause found in the fragment')

    if 'max_attenuation' in spec:
        # Only meaningful where the request is not pinned at the ceiling: a clipped signal is
        # smooth because it is clipped. Refuse rather than report a flattering number.
        rf = _finite(req)
        ceiling = max(rf) if rf else 0.0
        pinned = sum(1 for v in rf if v >= ceiling - 1e-9) / len(rf) if rf else 1.0
        if pinned > 0.10:
            res.add('max_attenuation', False,
                    f'the request is pinned at its ceiling for {pinned * 100:.0f} % of the '
                    f'fragment - attenuation here would be clipping, not the demand model')
        else:
            tq_r, iq_r = _ripple(torque), _ripple(iq)
            att = iq_r / tq_r if tq_r and tq_r == tq_r and tq_r > 0 else math.nan
            limit = float(spec['max_attenuation'])
            res.add('max_attenuation', att == att and att <= limit,
                    f'torque ripple {tq_r:.3f}, iq ripple {iq_r:.3f}, attenuation {att:.3f}, limit {limit}')
    return res
