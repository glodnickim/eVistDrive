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
from replay_behavior import evaluate, MissingControlDomainSignal  # noqa: E402

COLUMNS = ['time_s', 'cadence_rpm', 'torque_ckg', 'torque_ctrl', 'wheel_speed_kph', 'battery_v',
           'battery_a', 'iq_request_new', 'iq_ref_new', 'iq_request_recorded', 'iq_ref_recorded',
           'delta_request', 'delta_ref', 'debug_flags']

# Columns written when a trace deliberately has NO torque_ctrl, to prove the criterion refuses
# to fall back to torque_ckg rather than silently reading it.
COLUMNS_NO_CTRL = [c for c in COLUMNS if c != 'torque_ctrl']

failures: list[str] = []


def trace(rows, columns=COLUMNS):
    d = Path(tempfile.mkdtemp())
    p = d / 'trace.csv'
    with p.open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=columns)
        w.writeheader()
        for r in rows:
            full = {c: 0 for c in columns}
            full.update({k: v for k, v in r.items() if k in full})
            w.writerow(full)
    return p


def row(t, cad, tq, iq, req=None, tq_ctrl=None):
    # tq_ctrl defaults to tq: most fixtures don't care about the kg-vs-CLU distinction, only the
    # tests under "torque_ctrl vs torque_ckg" below deliberately pull them apart.
    return {'time_s': t, 'cadence_rpm': cad, 'torque_ckg': tq,
            'torque_ctrl': tq if tq_ctrl is None else tq_ctrl,
            'iq_ref_new': iq, 'iq_request_new': req if req is not None else iq}


def ride(n=200, cad=60, tq=lambda i: 400, iq=lambda i: 300, req=None, tq_ctrl=None):
    return [row(i * 0.03, cad, tq(i), iq(i), None if req is None else req(i),
                None if tq_ctrl is None else tq_ctrl(i))
            for i in range(n)]


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

    # --- max_attenuation must be invariant to the physical kg representation --------------
    # docs/AUDIT_REPLAY_ATTENUATION_DIVERGENCE_2026-09-20.md: the 2026-09-14 -> 2026-09-20
    # divergence (0.318/0.372/0.302 -> 0.618/0.785/0.588) was caused entirely by a firmware
    # commit re-measuring the torque_ckg (display kg) curve while iq_ref_new stayed byte-
    # identical. torque_ctrl is the frozen control-domain signal that curve is NOT allowed to
    # move. This is that regression, reproduced synthetically: two traces share the exact same
    # torque_ctrl and iq_ref_new, but one has a torque_ckg column re-scaled 2.1x, as if its kg
    # curve had just been re-measured. If max_attenuation reads torque_ckg, this fails (the
    # traces disagree); reading torque_ctrl, they must not.
    same_ctrl = lambda i: 200 + 600 * (i % 40) / 40.0
    same_iq = lambda i: 300 + 20 * (i % 40) / 40.0
    rows_old_kg_curve = ride(n=400, tq=lambda i: same_ctrl(i) * 1.00, tq_ctrl=same_ctrl,
                              iq=same_iq, req=open_req)
    rows_new_kg_curve = ride(n=400, tq=lambda i: same_ctrl(i) * 2.10, tq_ctrl=same_ctrl,
                              iq=same_iq, req=open_req)
    spec = {'max_attenuation': 0.60}
    res_old = evaluate(trace(rows_old_kg_curve), spec)
    res_new = evaluate(trace(rows_new_kg_curve), spec)
    att_old = next(d for c, _, d in res_old.checks if c == 'max_attenuation')
    att_new = next(d for c, _, d in res_new.checks if c == 'max_attenuation')
    name = 'max_attenuation is invariant to a torque_ckg-only rescale (torque_ctrl unchanged)'
    if res_old.ok != res_new.ok or att_old != att_new:
        failures.append(f'{name}: old-curve trace gave {att_old!r} ({res_old.ok}), '
                         f'new-curve trace gave {att_new!r} ({res_new.ok}) - '
                         f'changing only torque_ckg changed the result')
        print(f'FAIL [{name}]')
        print(f'        old torque_ckg curve: {att_old}')
        print(f'        new torque_ckg curve: {att_new}')
    else:
        print(f'PASS [{name}]: {att_old}')

    # --- max_attenuation must refuse to fall back to torque_ckg --------------------------
    rows_no_ctrl = [{k: v for k, v in r.items() if k != 'torque_ctrl'}
                     for r in ride(n=400, tq=lambda i: 200 + 600 * (i % 40) / 40.0,
                                    iq=lambda i: 300 + 20 * (i % 40) / 40.0, req=open_req)]
    name = 'max_attenuation raises rather than silently falling back to torque_ckg'
    try:
        evaluate(trace(rows_no_ctrl, columns=COLUMNS_NO_CTRL), {'max_attenuation': 0.60})
        failures.append(f'{name}: no exception was raised')
        print(f'FAIL [{name}]: no exception was raised')
    except MissingControlDomainSignal:
        print(f'PASS [{name}]')

    # --- max_attenuation: the ratio itself, pinned to known numbers -----------------------
    # The two tests above prove max_attenuation reads the right COLUMN. They do not prove it
    # computes the right NUMBER: both would still pass if _ripple() were off by a factor of two
    # (half-cycle amplitude instead of full peak-to-peak), or if the ratio were inverted, because
    # each compares two runs against each other rather than against arithmetic. So here the input
    # ripples are constructed to be exactly known and the reported attenuation is required to be
    # exactly their quotient - through the real tools/replay_behavior.py, never a local copy of
    # the formula, which would only pin the copy.
    #
    #   torque_ctrl swings 200..600 about a mean of 400  -> ripple (600-200)/400 = 1.000
    #   iq          swings 225..375 about a mean of 300  -> ripple (375-225)/300 = 0.500
    #   attenuation                                      -> 0.500 / 1.000       = 0.500
    #
    #   iq          swings 262.5..337.5 about 300        -> ripple  (75)/300    = 0.250
    #   attenuation                                      -> 0.250 / 1.000       = 0.250
    #
    # A x2 error anywhere in that chain reports 0.250 or 1.000 instead, and this test says so.
    span = lambda lo, hi: (lambda i: lo + (hi - lo) * (i % 40) / 39.0)
    ctrl_ripple_one = span(200.0, 600.0)

    for iq_lo, iq_hi, want in ((225.0, 375.0, '0.500'), (262.5, 337.5, '0.250')):
        rows_known = ride(n=400, tq=ctrl_ripple_one, tq_ctrl=ctrl_ripple_one,
                          iq=span(iq_lo, iq_hi), req=open_req)
        res = evaluate(trace(rows_known), {'max_attenuation': 0.60})
        detail = next(d for c, _, d in res.checks if c == 'max_attenuation')
        name = f'max_attenuation computes exactly {want} for a known ripple pair'
        want_detail = f'torque_ctrl ripple 1.000, iq ripple {want}, attenuation {want}'
        if not res.ok or want_detail not in detail:
            failures.append(f'{name}: got {detail!r} (ok={res.ok}), wanted {want_detail!r}')
            print(f'FAIL [{name}]: got {detail}')
        else:
            print(f'PASS [{name}]: {detail}')

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
