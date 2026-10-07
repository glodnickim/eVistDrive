#!/usr/bin/env python3
"""Every Assist V3 metric must be able to say NO.

For each metric in tools/assist_v3_metrics.py: a synthetic trace with known behaviour is
measured correctly, a trace showing the defect the metric exists to catch FAILS its check, and
evidence that cannot support the measurement is REFUSED (MetricRefused), never passed.

Run: python tests/test_assist_v3_metrics.py   (or python -m unittest tests.test_assist_v3_metrics)
"""
from __future__ import annotations

import math
import sys
import tempfile
import unittest
from pathlib import Path

R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R / 'tools'))
import assist_v3_metrics as M  # noqa: E402

HZ = 1000.0


def shape(angle_deg: float) -> float:
    """Two dips per revolution at 90/270 deg (synthetic, mean 1)."""
    return 1.0 + 0.9 * math.cos(2.0 * math.radians(angle_deg))


def synth(segs, demand, rpm=60.0, extra=None) -> M.Trace:
    """segs: [(kind, dur_s, mean0, mean1, rpm or None)]. demand(ctx) -> Iq."""
    cols = {k: [] for k in ('t_s', 'crank_cum_deg', 'cadence_true', 'gt_seg', 'gt_kind',
                            'gt_label', 'gt_intent_ckg', 'gt_mean_ckg', 'speed_x100', 'iq_ref',
                            'iq_actual', 'final_request', 'brake', 'elapsed', 'iq_ceiling',
                            'm2aa')}
    t = 0.0
    cum = 0.0
    for si, (kind, dur, m0, m1, seg_rpm) in enumerate(segs):
        n = int(round(dur * HZ))
        r = rpm if seg_rpm is None else seg_rpm
        seg_cum0 = cum
        for k in range(n):
            frac = k / n
            mean = m0 + (m1 - m0) * frac
            cum += r * 6.0 / HZ
            if kind in ('brake', 'reverse', 'stop', 'coast', 'release'):
                label = {'brake': 'BRAKE', 'reverse': 'REVERSE', 'stop': 'PEDAL_STOP',
                         'coast': 'COAST', 'release': 'TRUE_RELEASE'}[kind]
            elif kind == 'attack' and abs(cum - seg_cum0) < 360.0:
                label = 'ATTACK'
            elif mean > 0 and shape(cum) < 0.5:
                label = 'PHASE_DIP'
            else:
                label = 'NORMAL'
            ctx = {'t': t, 'cum': cum, 'label': label, 'seg': si, 'kind': kind,
                   't_seg': k / HZ, 'mean': mean}
            iq = float(demand(ctx))
            for key, v in (('t_s', t), ('crank_cum_deg', cum), ('cadence_true', r),
                           ('gt_seg', float(si)), ('gt_kind', kind), ('gt_label', label),
                           ('gt_intent_ckg', mean), ('gt_mean_ckg', mean),
                           ('speed_x100', 1500.0), ('iq_ref', iq), ('iq_actual', iq),
                           ('final_request', iq), ('brake', 1.0 if kind == 'brake' else 0.0),
                           ('elapsed', 1.0), ('iq_ceiling', 700.0), ('m2aa', iq * 6500 / 455)):
                cols[key].append(v)
            t += 1.0 / HZ
    if extra:
        for k, fn in extra.items():
            cols[k] = [fn(i, cols) for i in range(len(cols['t_s']))]
    return M.validate(M.Trace(cols, 'synthetic'))


def smooth(level):
    return lambda c: level


class Loader(unittest.TestCase):
    def test_refuses_nan_missing_and_time_reversal(self):
        tr = synth([('steady', 1.0, 1000, 1000, None)], smooth(300))
        tr.cols['iq_ref'][10] = float('nan')
        with self.assertRaises(M.MetricRefused):
            M.validate(tr)
        tr = synth([('steady', 1.0, 1000, 1000, None)], smooth(300))
        tr.cols['t_s'][5] = tr.cols['t_s'][4]
        with self.assertRaises(M.MetricRefused):
            M.validate(tr)
        tr = synth([('steady', 1.0, 1000, 1000, None)], smooth(300))
        del tr.cols['iq_ref']
        with self.assertRaises(M.MetricRefused):
            M.release_latency(tr)

    def test_load_trace_roundtrip_and_empty_file(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'x.csv'
            p.write_text('t_s,crank_cum_deg,gt_label,iq_ref\n0,0,NORMAL,1\n0.001,1,NORMAL,2\n')
            tr = M.load_trace(p)
            self.assertEqual(tr.cols['iq_ref'], [1.0, 2.0])
            e = Path(d) / 'e.csv'
            e.write_text('')
            with self.assertRaises(M.MetricRefused):
                M.load_trace(e)


class ReleaseLatency(unittest.TestCase):
    SEGS = [('steady', 3.0, 1000, 1000, None), ('release', 3.0, 0, 0, None)]

    def test_fast_release_passes(self):
        tr = synth(self.SEGS, lambda c: 300 if c['kind'] == 'steady'
                   else max(0.0, 300 - 300 * c['t_seg'] / 0.05))
        r = M.release_latency(tr)
        self.assertLess(r['ms_50'], 30)
        self.assertLess(r['deg_50'], 10)
        self.assertTrue(M.check('release_latency_50_deg', r['deg_50']))

    def test_slow_release_fails(self):
        tr = synth(self.SEGS, lambda c: 300 if c['kind'] == 'steady'
                   else max(0.0, 300 - 100 * c['t_seg']))
        r = M.release_latency(tr)
        self.assertAlmostEqual(r['ms_50'], 1500, delta=5)
        self.assertFalse(M.check('release_latency_50_deg', r['deg_50']))

    def test_never_released_is_inf_and_fails(self):
        tr = synth(self.SEGS, smooth(300))
        r = M.release_latency(tr)
        self.assertTrue(math.isinf(r['ms_50']))
        self.assertFalse(M.check('release_latency_50_deg', r['deg_50']))

    def test_refusals(self):
        with self.assertRaises(M.MetricRefused):          # no release event
            M.release_latency(synth([('steady', 3.0, 1000, 1000, None)], smooth(300)))
        with self.assertRaises(M.MetricRefused):          # nothing to release
            M.release_latency(synth(self.SEGS, smooth(0)))
        with self.assertRaises(M.MetricRefused):          # < 1 revolution of history
            M.release_latency(synth([('steady', 0.3, 1000, 1000, None),
                                     ('release', 1.0, 0, 0, None)], smooth(300)))


class PhaseDip(unittest.TestCase):
    SEGS = [('steady', 8.0, 1000, 1000, None)]

    def test_smooth_demand_passes(self):
        r = M.phase_dip_fp(synth(self.SEGS, smooth(300)))
        self.assertGreater(r['passages'], 8)
        self.assertEqual(r['rate'], 0.0)
        self.assertTrue(M.check('phase_dip_fp_rate', r['rate']))

    def test_pulsing_demand_fails(self):
        # Demand that follows the pedal stroke into every dead spot: exactly what V3 must stop.
        r = M.phase_dip_fp(synth(self.SEGS, lambda c: 300 * (0.5 + 0.5 * min(1.0, shape(c['cum'])))))
        self.assertEqual(r['rate'], 1.0)
        self.assertFalse(M.check('phase_dip_fp_rate', r['rate']))

    def test_refusals(self):
        with self.assertRaises(M.MetricRefused):      # no dip passages at all
            M.phase_dip_fp(synth([('release', 3.0, 0, 0, None)], smooth(300)))
        with self.assertRaises(M.MetricRefused):      # demand below the measurable floor
            M.phase_dip_fp(synth(self.SEGS, smooth(5)))


class Ripple(unittest.TestCase):
    SEGS = [('steady', 8.0, 1000, 1000, None)]

    def test_small_ripple_passes_large_fails(self):
        ok = M.per_rev_ripple(synth(self.SEGS, lambda c: 300 + 6 * math.sin(math.radians(2 * c['cum']))))
        self.assertAlmostEqual(ok['median'], 12 / 300, delta=0.003)
        self.assertTrue(M.check('ripple_rev_median', ok['median']))
        bad = M.per_rev_ripple(synth(self.SEGS, lambda c: 300 + 60 * math.sin(math.radians(2 * c['cum']))))
        self.assertFalse(M.check('ripple_rev_median', bad['median']))

    def test_smooth_uncapped_is_a_result_not_a_refusal(self):
        self.assertEqual(M.per_rev_ripple(synth(self.SEGS, smooth(300)))['median'], 0.0)

    def test_pinned_at_ceiling_is_refused(self):
        with self.assertRaises(M.MetricRefused):
            M.per_rev_ripple(synth(self.SEGS, smooth(455)))

    def test_too_short_is_refused(self):
        with self.assertRaises(M.MetricRefused):
            M.per_rev_ripple(synth([('steady', 3.5, 1000, 1000, None)], smooth(300)))


class LowerDemand(unittest.TestCase):
    SEGS = [('steady', 3.0, 1000, 1000, None), ('ramp', 2.0, 1000, 400, None),
            ('steady', 4.0, 400, 400, None)]

    @staticmethod
    def follow(lag_s):
        def f(c):
            if c['kind'] == 'steady' and c['seg'] == 0:
                return 300
            x = (c['t'] - 3.0 - lag_s) / 2.0
            return 300 - 180 * min(1.0, max(0.0, x))
        return f

    def test_prompt_follow_passes(self):
        r = M.lower_demand(synth(self.SEGS, self.follow(0.0)))
        self.assertLess(r['lag_ms'], 50)
        self.assertTrue(M.check('lower_demand_lag_ms', r['lag_ms']))

    def test_late_follow_fails(self):
        r = M.lower_demand(synth(self.SEGS, self.follow(1.5)))
        self.assertGreater(r['lag_ms'], 1000)
        self.assertFalse(M.check('lower_demand_lag_ms', r['lag_ms']))

    def test_refusals(self):
        with self.assertRaises(M.MetricRefused):      # no falling ramp
            M.lower_demand(synth([('steady', 5.0, 1000, 1000, None)], smooth(300)))
        with self.assertRaises(M.MetricRefused):      # demand never changed
            M.lower_demand(synth(self.SEGS, smooth(300)))


class Attack(unittest.TestCase):
    SEGS = [('steady', 3.0, 500, 500, None), ('attack', 3.0, 1500, 1500, None)]

    @staticmethod
    def rise(tau):
        return lambda c: 150 if c['seg'] == 0 else 150 + 250 * (1 - math.exp(-c['t_seg'] / tau))

    def test_fast_passes_slow_fails(self):
        fast = M.attack_response(synth(self.SEGS, self.rise(0.1)))
        self.assertAlmostEqual(fast['ms'], 100, delta=10)
        self.assertTrue(M.check('attack_t63_ms', fast['ms']))
        slow = M.attack_response(synth(self.SEGS, self.rise(0.6)))
        self.assertFalse(M.check('attack_t63_ms', slow['ms']))

    def test_refusals(self):
        with self.assertRaises(M.MetricRefused):
            M.attack_response(synth([('steady', 3.0, 500, 500, None)], smooth(150)))
        with self.assertRaises(M.MetricRefused):      # no measurable demand step
            M.attack_response(synth(self.SEGS, smooth(150)))


class Restart(unittest.TestCase):
    SEGS = [('steady', 3.0, 1000, 1000, None), ('stop', 2.0, 0, 0, 0.0),
            ('steady', 4.0, 1000, 1000, None)]

    def test_monotone_restart_passes(self):
        r = M.restart_continuity(synth(self.SEGS, lambda c: 0 if c['seg'] == 1 else
                                       (300 if c['seg'] == 0 else min(300, 1500 * c['t_seg']))))
        self.assertEqual(r['dip_frac'], 0.0)
        self.assertTrue(M.check('restart_dip_frac', r['dip_frac']))

    def test_restart_that_falls_back_fails(self):
        def f(c):
            if c['seg'] == 0:
                return 300
            if c['seg'] == 1:
                return 0
            x = c['t_seg']
            if x < 0.1:
                return 3000 * x                      # 0 -> 300 in 0.1 s
            if x < 0.7:
                return 300
            if x < 1.3:
                return 60                            # falls back to 20 % for 216 deg
            return 300
        r = M.restart_continuity(synth(self.SEGS, f))
        self.assertAlmostEqual(r['dip_frac'], 0.8, delta=0.02)
        self.assertFalse(M.check('restart_dip_frac', r['dip_frac']))

    def test_stroke_ripple_is_not_a_restart_dip(self):
        # Regression (2026-10-07): the first version scored ordinary 2/rev pedal-stroke ripple as
        # a restart give-back (restart_dip_frac == steady ripple at every cadence).
        def f(c):
            if c['seg'] == 0:
                return 300
            if c['seg'] == 1:
                return 0
            ramp = min(1.0, c['t_seg'] / 0.1)
            return ramp * 300 * (1 + 0.3 * math.cos(math.radians(2 * c['cum'])))
        r = M.restart_continuity(synth(self.SEGS, f))
        self.assertLess(r['dip_frac'], 0.05)
        self.assertTrue(M.check('restart_dip_frac', r['dip_frac']))

    def test_refusals(self):
        with self.assertRaises(M.MetricRefused):      # no stop -> restart
            M.restart_continuity(synth([('steady', 4.0, 1000, 1000, None)], smooth(300)))
        with self.assertRaises(M.MetricRefused):      # no measurable post-restart demand
            M.restart_continuity(synth(self.SEGS, smooth(0)))


class SafetyZero(unittest.TestCase):
    SEGS = [('steady', 3.0, 1000, 1000, None), ('brake', 2.0, 1000, 1000, None)]

    def test_prompt_zero_passes(self):
        r = M.safety_zero(synth(self.SEGS, lambda c: 300 if c['seg'] == 0 else
                                max(0.0, 300 - 300 * c['t_seg'] / 0.1)), 'BRAKE')
        self.assertAlmostEqual(r['ref0_ms'], 100, delta=3)
        self.assertTrue(M.check('safety_ref0_ms', r['ref0_ms']))
        self.assertTrue(M.check('safety_overshoot', r['overshoot']))

    def test_never_zero_or_kick_fails(self):
        r = M.safety_zero(synth(self.SEGS, smooth(300)), 'BRAKE')
        self.assertTrue(math.isinf(r['ref0_ms']))
        self.assertFalse(M.check('safety_ref0_ms', r['ref0_ms']))
        kick = M.safety_zero(synth(self.SEGS, lambda c: 300 if c['seg'] == 0 else
                                   (400 if c['t_seg'] < 0.05 else 0)), 'BRAKE')
        self.assertFalse(M.check('safety_overshoot', kick['overshoot']))

    def test_refusals(self):
        with self.assertRaises(M.MetricRefused):      # no such event
            M.safety_zero(synth(self.SEGS, smooth(300)), 'REVERSE')
        with self.assertRaises(M.MetricRefused):      # nothing to cut
            M.safety_zero(synth(self.SEGS, smooth(0)), 'BRAKE')


class Energy(unittest.TestCase):
    def test_integral(self):
        r = M.energy(synth([('steady', 2.0, 1000, 1000, None)], smooth(100)))
        self.assertAlmostEqual(r['iq_s'], 100 * 1.999, delta=0.01)

    def test_refuses_time_reversal(self):
        tr = synth([('steady', 1.0, 1000, 1000, None)], smooth(100))
        tr.cols['t_s'][3] = tr.cols['t_s'][1]
        with self.assertRaises(M.MetricRefused):
            M.energy(tr)


class Carry(unittest.TestCase):
    SEGS = [('steady', 2.0, 1000, 1000, None), ('stop', 1.0, 0, 0, 0.0),
            ('brake', 1.0, 0, 0, 0.0)]

    def test_absent_columns_are_na_not_pass(self):
        tr = synth(self.SEGS, smooth(100))
        self.assertEqual(M.carry(tr)['status'], 'N/A')
        out = M.scenario_metrics(tr, 'steady')
        self.assertEqual(out['carry_activations'], 'N/A')
        self.assertFalse(M.check('carry_active_during_cut_rows', None))

    def test_carry_during_brake_fails(self):
        tr = synth(self.SEGS, smooth(100), extra={
            'v3_carry_state': lambda i, c: 1.0 if c['gt_kind'][i] in ('stop', 'brake') else 0.0})
        r = M.carry(tr)
        self.assertEqual(r['activations'], 1)
        self.assertAlmostEqual(r['duration_s'], 2.0, delta=0.01)
        self.assertAlmostEqual(r['distance_m'], 2.0 * 15 / 3.6, delta=0.05)
        self.assertGreater(r['active_during_cut_rows'], 0)
        self.assertFalse(M.check('carry_active_during_cut_rows', r['active_during_cut_rows']))


class CadenceInvariance(unittest.TestCase):
    def test_invariant_passes_varying_fails(self):
        ok = M.cadence_invariance({20: 40.0, 60: 42.0, 130: 41.0})
        self.assertTrue(M.check('cadence_rel_spread', ok['rel_spread']))
        bad = M.cadence_invariance({20: 40.0, 60: 120.0, 130: 400.0})
        self.assertFalse(M.check('cadence_rel_spread', bad['rel_spread']))

    def test_refusals_and_missing_points(self):
        with self.assertRaises(M.MetricRefused):
            M.cadence_invariance({20: 40.0, 60: float('inf'), 130: None})
        r = M.cadence_invariance({20: 40.0, 60: float('inf'), 130: 45.0})
        self.assertEqual(r['missing'], [60])


class CheckNeverPassesMissingEvidence(unittest.TestCase):
    def test_none_and_nan_fail_every_check(self):
        for k in M.CHECKS:
            self.assertFalse(M.check(k, None), k)
            self.assertFalse(M.check(k, float('nan')), k)


if __name__ == '__main__':
    unittest.main(verbosity=1)
