#!/usr/bin/env python3
"""Every replay behaviour criterion must be able to FAIL.

A criterion that has only ever been seen to pass is indistinguishable from one that always
passes. `tools/run_replay_regression.py` now prints BEHAVIOR_ACCEPTED for the six registered W1
rides; this file shows what it takes to make each of those criteria say no, by handing
`replay_behavior.evaluate()` a trace with exactly the defect the criterion exists to catch.

The `max_attenuation` group is the important one. It must refuse a trace whose request is clipped,
because a clipped signal is smooth for a reason that has nothing to do with the demand model -
the same trap audit finding 8 found in the ripple analyzer, checked here rather than assumed not
to have been repeated.
"""
from __future__ import annotations
import csv
import sys
import tempfile
from pathlib import Path

R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R / 'tools'))
from replay_behavior import evaluate  # noqa: E402

COLUMNS = ['time_s', 'cadence_rpm', 'torque_ckg', 'wheel_speed_kph', 'battery_v', 'battery_a',
           'iq_request_new', 'iq_ref_new', 'iq_request_recorded', 'iq_ref_recorded',
           'delta_request', 'delta_ref', 'debug_flags']

failures: list[str] = []


def trace(rows):
    d = Path(tempfile.mkdtemp())
    p = d / 'trace.csv'
    with p.open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS)
        w.writeheader()
        for r in rows:
            full = {c: 0 for c in COLUMNS}
            full.update(r)
            w.writerow(full)
    return p


def row(t, cad, tq, iq, req=None):
    return {'time_s': t, 'cadence_rpm': cad, 'torque_ckg': tq,
            'iq_ref_new': iq, 'iq_request_new': req if req is not None else iq}


def ride(n=200, cad=60, tq=lambda i: 400, iq=lambda i: 300, req=None):
    return [row(i * 0.03, cad, tq(i), iq(i), None if req is None else req(i)) for i in range(n)]


def check(name, rows, spec, want_ok, want_detail=None):
    res = evaluate(trace(rows), spec)
    if res.ok != want_ok:
        failures.append(f'{name}: expected ok={want_ok}, got ok={res.ok} :: '
                        + ' | '.join(f'{c}={o} {d}' for c, o, d in res.checks))
        print(f'FAIL [{name}]: expected ok={want_ok}, got {res.ok}')
        for c, o, d in res.checks:
            print(f'        {c} {"ok" if o else "FAIL"} {d}')
        return
    if want_detail:
        blob = ' '.join(d for _, _, d in res.checks)
        if want_detail not in blob:
            failures.append(f'{name}: right verdict, wrong reason (wanted {want_detail!r})')
            print(f'FAIL [{name}]: right verdict, wrong reason; got: {blob}')
            return
    print(f'PASS [{name}]')


def main() -> int:
    # --- produces_assist ------------------------------------------------------------------
    check('produces_assist accepts a ride with current',
          ride(), {'produces_assist': True}, True)
    check('produces_assist rejects a dead chain',
          ride(iq=lambda i: 0), {'produces_assist': True}, False)

    # --- responds_to_load -----------------------------------------------------------------
    check('responds_to_load accepts assist following the pedal up',
          ride(tq=lambda i: 200 + i * 8, iq=lambda i: 100 + i * 3),
          {'responds_to_load': 'rising'}, True)
    check('responds_to_load rejects assist that ignores a rising pedal',
          ride(tq=lambda i: 200 + i * 8, iq=lambda i: 300),
          {'responds_to_load': 'rising'}, False)
    check('responds_to_load rejects assist moving the WRONG way',
          ride(tq=lambda i: 200 + i * 8, iq=lambda i: 800 - i * 3),
          {'responds_to_load': 'rising'}, False)
    check('responds_to_load accepts assist following the pedal down',
          ride(tq=lambda i: 1800 - i * 8, iq=lambda i: 700 - i * 3),
          {'responds_to_load': 'falling'}, True)

    # --- pause_releases -------------------------------------------------------------------
    pause = lambda iq_idle: [row(i * 0.03, 60 if i < 60 or i >= 160 else 0, 400,
                                 300 if i < 60 or i >= 160 else iq_idle) for i in range(200)]
    check('pause_releases accepts current that actually goes to zero',
          pause(0), {'pause_releases': 0.95}, True)
    check('pause_releases rejects a sustained term that keeps pulling',
          pause(120), {'pause_releases': 0.95}, False)
    check('pause_releases rejects a fragment with no pause in it at all',
          ride(), {'pause_releases': 0.95}, False, 'idle samples')

    # --- restart_recovers -----------------------------------------------------------------
    check('restart_recovers accepts assist coming back after the pause',
          pause(0), {'restart_recovers': True}, True)
    check('restart_recovers rejects a chain that stays dead after the pause',
          [row(i * 0.03, 60 if i < 60 or i >= 160 else 0, 400, 300 if i < 60 else 0)
           for i in range(200)],
          {'restart_recovers': True}, False)

    # --- max_attenuation ------------------------------------------------------------------
    # The request must have real headroom in these fixtures. `req` therefore walks over a wide
    # range and revisits its own maximum rarely; a coarse fixture (say `500 + i % 7`) puts 14 % of
    # the samples on one plateau value and the criterion correctly refuses it, which would be the
    # fixture's defect and not the chain's.
    open_req = lambda i: 400 + (i % 40) * 5

    # An honest trace: the pedal swings hard, the current swings much less.
    check('max_attenuation accepts a current smoother than the pedal',
          ride(n=400, tq=lambda i: 200 + 600 * (i % 40) / 40.0,
               iq=lambda i: 300 + 20 * (i % 40) / 40.0, req=open_req),
          {'max_attenuation': 0.60}, True)
    # The defect the whole pipeline exists to remove: the motor copying every leg push.
    check('max_attenuation rejects a current that copies the pedal',
          ride(n=400, tq=lambda i: 200 + 600 * (i % 40) / 40.0,
               iq=lambda i: 100 + 300 * (i % 40) / 40.0, req=open_req),
          {'max_attenuation': 0.60}, False, 'attenuation')
    # Smooth because it is clipped. This must NOT be reported as attenuation.
    check('max_attenuation refuses a clipped request instead of flattering it',
          ride(n=400, tq=lambda i: 200 + 600 * (i % 40) / 40.0,
               iq=lambda i: 300, req=lambda i: 700),
          {'max_attenuation': 0.60}, False, 'pinned at its ceiling')
    # The bias is deliberate: a request that spends a lot of the fragment on ONE value cannot
    # express the pedal swing, whatever put it there, so the criterion refuses rather than reports
    # a flattering number. Recorded here so the sensitivity is a decision and not a surprise.
    check('max_attenuation refuses a request stuck on a plateau, even a smooth one',
          ride(n=400, tq=lambda i: 200 + 600 * (i % 40) / 40.0,
               iq=lambda i: 300 + 20 * (i % 40) / 40.0, req=lambda i: 500 + (i % 7)),
          {'max_attenuation': 0.60}, False, 'pinned at its ceiling')

    # --- the "no criteria" contract -------------------------------------------------------
    res = evaluate(trace(ride()), {})
    if res.assessed:
        failures.append('an empty spec reported itself as assessed')
        print('FAIL [empty spec]: reported itself as assessed')
    else:
        print('PASS [empty spec is NOT_ASSESSED, never a pass]')

    print()
    if failures:
        print(f'FAIL: {len(failures)} behaviour criteria did not behave as specified',
              file=sys.stderr)
        for f in failures:
            print(f'  {f}', file=sys.stderr)
        return 1
    print('PASS: every replay behaviour criterion can both accept and reject')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
