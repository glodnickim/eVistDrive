#!/usr/bin/env python3
"""Assist Behavior V3 metrics over SIL --script traces (pure functions, stdlib only).

Input: the CSV written by `evist_sil --script` (columns documented in sim/evist_sil.c,
SIL_SCRIPT_CSV_HEADER) - ground truth from tests/host/common/rider_script.[ch] plus the
production pipeline outputs. Every metric either returns numbers or raises MetricRefused when
the evidence cannot support a measurement (no event, nothing to measure, NaN, missing column,
too short, pinned at a ceiling). A refusal is never converted into a pass.

Demand signal: `iq_ref` by default = MS.i_q_setpoint, the published Iq after the 16 kHz
fast_iq_slew (what the current loop is asked for). `final_request` (pipeline final request,
before the slew) can be selected with signal=.

Definitions (all angles are crank degrees taken from the ground-truth crank_cum_deg):
  release latency     t0 = first TRUE_RELEASE tick. pre = mean demand over the one crank
                      revolution before t0. Latency(frac) = first t >= t0 with demand <= frac*pre.
                      Reported for frac = 0.5 (decision visible) and 0.1 (released). inf = never.
  phase-dip FP rate   a passage = a maximal run of PHASE_DIP labels, at least SETTLE_REVS after
                      its segment started, whose preceding revolution is all NORMAL/PHASE_DIP.
                      ref = mean demand over that preceding revolution. The passage is a false
                      positive when min demand over [passage start, +180 deg] < (1-drop)*ref.
                      rate = FP / passages, drop = 0.2 by default.
  per-rev ripple      complete crank revolutions inside steady segments, after SETTLE_S, all
                      labels NORMAL/PHASE_DIP: (max-min)/mean per revolution; median and max.
  lower demand        first falling `ramp` segment (gradual release / crest): pre = mean over the
                      revolution before it, new = mean over the last revolution of the trace.
                      Time/angle from ramp start until demand <= new + tol*(pre-new) and stays
                      <= new + 2*tol*(pre-new) for one revolution; lag = same from ramp END.
  attack response     t0 = first ATTACK tick; pre = mean over the revolution before; post = mean
                      over the last revolution of that attack segment; t63 = first t with
                      demand >= pre + 0.63*(post-pre).
  restart continuity  restart = first forward segment after a `stop` segment. steady = mean over
                      its last revolution. t50 = first demand >= 0.5*steady; dip = largest
                      give-back from an already reached level of the 180-deg trailing mean
                      (cancels the 2/rev stroke ripple) over the 2 revolutions after t50,
                      divided by steady; first_iq = first demand > 0.
  safety zero         t0 = first tick of label (BRAKE, REVERSE, PEDAL_STOP, COAST). iq_at_event =
                      demand on the row before. ref0 = first iq_ref <= 0; drive0 = first plant
                      iq_actual <= 2 counts (AXIS_CURRENT_ZERO_COUNTS in the SIL); overshoot =
                      max demand after t0 minus the max demand of the revolution before t0
                      (must be <= 0).
  energy              the SIL models no battery current and a fixed 42 V bus, so energy is the
                      proxy E = integral(iq_actual dt) over t >= 0 in Iq-count*seconds (x 42 V for
                      a W*s-shaped number in arbitrary current units). Comparable baseline vs
                      candidate only.
  carry               computed from v3_carry_state / speed_x100 when those columns exist
                      (activations, false positives = activations in scenarios without obstacle
                      ground truth - none of the current scripts has one - duration, distance,
                      active during BRAKE/REVERSE); otherwise status N/A.
  cadence invariance  spread (max-min) and relative spread of one metric across cadences.
"""
from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable, Sequence

INF = float('inf')
MIN_DEMAND = 20.0          # Iq counts: below this there is nothing meaningful to measure
SETTLE_S = 3.0             # start transient excluded from steady-state metrics
SETTLE_REVS = 2.0
CURRENT_ZERO_COUNTS = 2.0
STEADY_LABELS = ('NORMAL', 'PHASE_DIP')

NUMERIC = ('t_s', 'crank_cum_deg', 'cadence_true', 'gt_seg', 'gt_intent_ckg', 'gt_mean_ckg',
           'speed_x100', 'iq_ref', 'iq_actual', 'final_request', 'brake', 'elapsed',
           'iq_ceiling', 'm2aa')
TEXT = ('gt_label', 'gt_kind')


class MetricRefused(Exception):
    """The evidence cannot support this measurement. Never a pass."""


@dataclass
class Trace:
    cols: dict[str, list] = field(default_factory=dict)
    name: str = ''

    def __len__(self) -> int:
        return len(self.cols.get('t_s', ()))

    def col(self, name: str) -> list:
        if name not in self.cols:
            raise MetricRefused(f'missing column {name!r}')
        return self.cols[name]


# --------------------------------------------------------------------------------- loading

def validate(tr: Trace) -> Trace:
    n = len(tr)
    if n < 2:
        raise MetricRefused('trace has fewer than 2 rows')
    for k, v in tr.cols.items():
        if len(v) != n:
            raise MetricRefused(f'column {k!r} length {len(v)} != {n}')
        if v and isinstance(v[0], float):
            for x in v:
                if math.isnan(x) or math.isinf(x):
                    raise MetricRefused(f'non-finite value in column {k!r}')
    t = tr.col('t_s')
    for a, b in zip(t, t[1:]):
        if not b > a:
            raise MetricRefused('time is not strictly increasing')
    return tr


def load_trace(path: str | Path, extra: Iterable[str] = ()) -> Trace:
    """Read only the columns the metrics use (plus any V3/carry columns present)."""
    path = Path(path)
    with path.open(newline='') as f:
        rd = csv.reader(f)
        try:
            header = next(rd)
        except StopIteration:
            raise MetricRefused(f'{path.name}: empty file') from None
        want_num = [c for c in header if c in NUMERIC or c.startswith('v3_') or c in extra]
        want_txt = [c for c in header if c in TEXT]
        idx_num = [(c, header.index(c)) for c in want_num]
        idx_txt = [(c, header.index(c)) for c in want_txt]
        cols: dict[str, list] = {c: [] for c in want_num + want_txt}
        for row in rd:
            if len(row) != len(header):
                raise MetricRefused(f'{path.name}: ragged row')
            for c, i in idx_num:
                cols[c].append(float(row[i]))
            for c, i in idx_txt:
                cols[c].append(row[i])
    return validate(Trace(cols, path.stem))


# --------------------------------------------------------------------------------- helpers

def _first(seq: Sequence, pred: Callable, start: int = 0) -> int | None:
    for i in range(start, len(seq)):
        if pred(seq[i]):
            return i
    return None


def onsets(tr: Trace, label: str) -> list[int]:
    lab = tr.col('gt_label')
    return [i for i in range(len(lab)) if lab[i] == label and (i == 0 or lab[i - 1] != label)]


def segments(tr: Trace) -> list[tuple[int, int, str]]:
    """[(start_index, end_index_exclusive, kind)] in trace order."""
    seg = tr.col('gt_seg')
    kind = tr.col('gt_kind')
    out = []
    s = 0
    for i in range(1, len(seg) + 1):
        if i == len(seg) or seg[i] != seg[s]:
            out.append((s, i, kind[s]))
            s = i
    return out


def rev_before(tr: Trace, i: int, revs: float = 1.0) -> tuple[int, int]:
    """Index range [j, i) covering `revs` crank revolutions before row i."""
    cum = tr.col('crank_cum_deg')
    span = 360.0 * revs
    j = i
    while j > 0 and abs(cum[i] - cum[j - 1]) < span:
        j -= 1
    if j == 0 and abs(cum[i] - cum[0]) < span:
        raise MetricRefused('less than one crank revolution of history before the event')
    return j, i


def rev_after(tr: Trace, i: int, revs: float = 1.0) -> tuple[int, int]:
    cum = tr.col('crank_cum_deg')
    span = 360.0 * revs
    k = i
    while k < len(cum) and abs(cum[k] - cum[i]) < span:
        k += 1
    return i, k


def _mean(v: Sequence[float]) -> float:
    if not v:
        raise MetricRefused('empty window')
    return sum(v) / len(v)


M2AA_CAP = 6500.0          # BDE8 Q5A cap (ARCHITECTURE_V3 §5): demand saturated at 0.65*P


def _at_ceiling(tr: Trace, i: int, value: float) -> bool:
    """A flat demand is only 'pinned' when it sits on a real ceiling: the BDE8 m2aa cap, or
    0.65 * the phase-current ceiling. A perfectly smooth demand below the cap is a result."""
    if 'm2aa' in tr.cols and tr.cols['m2aa'][i] >= M2AA_CAP:
        return True
    if 'iq_ceiling' in tr.cols and value >= 0.65 * tr.cols['iq_ceiling'][i] - 1.0:
        return True
    return False


def _dt_ms(tr: Trace, a: int, b: int) -> float:
    t = tr.col('t_s')
    return 1000.0 * (t[b] - t[a])


def _ddeg(tr: Trace, a: int, b: int) -> float:
    c = tr.col('crank_cum_deg')
    return abs(c[b] - c[a])


# --------------------------------------------------------------------------------- metrics

def release_latency(tr: Trace, signal: str = 'iq_ref', fracs: Sequence[float] = (0.5, 0.1)) -> dict:
    on = onsets(tr, 'TRUE_RELEASE')
    if not on:
        raise MetricRefused('no TRUE_RELEASE event in the trace')
    i0 = on[0]
    sig = tr.col(signal)
    j, _ = rev_before(tr, i0)
    pre = _mean(sig[j:i0])
    if pre < MIN_DEMAND:
        raise MetricRefused(f'pre-release demand {pre:.1f} < {MIN_DEMAND}: nothing to release')
    out = {'pre': pre}
    for fr in fracs:
        k = _first(sig, lambda x: x <= fr * pre, i0)
        tag = f'{int(round(fr * 100))}'
        out[f'ms_{tag}'] = INF if k is None else _dt_ms(tr, i0, k)
        out[f'deg_{tag}'] = INF if k is None else _ddeg(tr, i0, k)
    return out


def phase_dip_fp(tr: Trace, signal: str = 'iq_ref', drop: float = 0.2,
                 settle_revs: float = SETTLE_REVS) -> dict:
    lab = tr.col('gt_label')
    sig = tr.col(signal)
    cum = tr.col('crank_cum_deg')
    seg_start = {}
    for s, e, _k in segments(tr):
        for i in range(s, e):
            seg_start[i] = s
    passages = fp = refused = 0
    worst = 0.0
    for i0 in onsets(tr, 'PHASE_DIP'):
        if abs(cum[i0] - cum[seg_start[i0]]) < 360.0 * settle_revs:
            continue
        try:
            j, _ = rev_before(tr, i0)
        except MetricRefused:
            continue
        if any(lab[x] not in STEADY_LABELS for x in range(j, i0)):
            continue
        ref = _mean(sig[j:i0])
        a, b = rev_after(tr, i0, 0.5)
        if b - a < 2 or any(lab[x] not in STEADY_LABELS for x in range(a, b)):
            continue
        if ref < MIN_DEMAND:
            refused += 1
            continue
        passages += 1
        depth = 1.0 - min(sig[a:b]) / ref
        worst = max(worst, depth)
        if depth > drop:
            fp += 1
    if passages == 0:
        raise MetricRefused('no usable PHASE_DIP passage (or demand below the measurable floor)')
    return {'rate': fp / passages, 'passages': passages, 'fp': fp, 'worst_drop': worst,
            'refused_low_demand': refused}


def per_rev_ripple(tr: Trace, signal: str = 'iq_ref', settle_s: float = SETTLE_S) -> dict:
    lab = tr.col('gt_label')
    kind = tr.col('gt_kind')
    sig = tr.col(signal)
    cum = tr.col('crank_cum_deg')
    t = tr.col('t_s')
    seg = tr.col('gt_seg')
    revs: list[float] = []
    pinned = 0
    if t[-1] < settle_s:
        raise MetricRefused('trace shorter than the settle time')
    n = len(sig)

    def eligible(x: int) -> bool:
        return t[x] >= settle_s and kind[x] == 'steady' and lab[x] in STEADY_LABELS

    i = 0
    while i < n:
        if not eligible(i):
            i += 1
            continue
        run_end = i
        while run_end < n and eligible(run_end) and seg[run_end] == seg[i]:
            run_end += 1
        a = i
        while True:
            _, b = rev_after(tr, a, 1.0)
            if b >= run_end:
                break
            w = sig[a:b]
            m = _mean(w)
            if m >= MIN_DEMAND:
                hi = max(w)
                revs.append((hi - min(w)) / m)
                flat = sum(1 for x in w if x == hi) > 0.9 * len(w)
                if flat and _at_ceiling(tr, a + w.index(hi), hi):
                    pinned += 1
            a = b
        i = run_end
    if len(revs) < 2:
        raise MetricRefused(f'only {len(revs)} complete steady revolution(s) with measurable demand')
    if pinned > len(revs) // 2:
        raise MetricRefused('demand pinned at a ceiling in most revolutions: ripple not measurable')
    srt = sorted(revs)
    return {'median': srt[len(srt) // 2], 'max': srt[-1], 'revs': len(revs)}


def lower_demand(tr: Trace, signal: str = 'iq_ref', tol: float = 0.1) -> dict:
    mean_gt = tr.col('gt_mean_ckg')
    sig = tr.col(signal)
    ramp = None
    for s, e, k in segments(tr):
        if k == 'ramp' and mean_gt[e - 1] < mean_gt[s] - 1e-6:
            ramp = (s, e)
            break
    if ramp is None:
        raise MetricRefused('no falling ramp segment (gradual release) in the trace')
    s, e = ramp
    j, _ = rev_before(tr, s)
    pre = _mean(sig[j:s])
    n = len(sig)
    k0, _ = rev_before(tr, n - 1)
    new = _mean(sig[k0:n])
    if pre - new < MIN_DEMAND:
        raise MetricRefused(f'demand did not change measurably (pre {pre:.1f}, new {new:.1f})')
    target = new + tol * (pre - new)
    band = new + 2.0 * tol * (pre - new)
    hit = None
    i = s
    while i < n:
        if sig[i] > target:
            i += 1
            continue
        a, b = rev_after(tr, i, 1.0)
        if b >= n:
            break
        bad = _first(sig[a:b], lambda x: x > band)
        if bad is None:
            hit = i
            break
        i = a + bad + 1
    return {'pre': pre, 'new': new,
            'ms': INF if hit is None else _dt_ms(tr, s, hit),
            'deg': INF if hit is None else _ddeg(tr, s, hit),
            'lag_ms': INF if hit is None else _dt_ms(tr, e - 1, hit)}


def attack_response(tr: Trace, signal: str = 'iq_ref') -> dict:
    on = onsets(tr, 'ATTACK')
    if not on:
        raise MetricRefused('no ATTACK event in the trace')
    i0 = on[0]
    sig = tr.col(signal)
    seg = tr.col('gt_seg')
    e = i0
    while e < len(seg) and seg[e] == seg[i0]:
        e += 1
    j, _ = rev_before(tr, i0)
    pre = _mean(sig[j:i0])
    k, _ = rev_before(tr, e - 1)
    post = _mean(sig[k:e])
    if post - pre < MIN_DEMAND:
        raise MetricRefused(f'no measurable demand step (pre {pre:.1f}, post {post:.1f})')
    lvl = pre + 0.63 * (post - pre)
    hit = _first(sig, lambda x: x >= lvl, i0)
    if hit is not None and hit >= e:
        hit = None
    return {'pre': pre, 'post': post,
            'ms': INF if hit is None else _dt_ms(tr, i0, hit),
            'deg': INF if hit is None else _ddeg(tr, i0, hit)}


def restart_continuity(tr: Trace, signal: str = 'iq_ref') -> dict:
    segs = segments(tr)
    restart = None
    for (s0, e0, k0), (s1, e1, k1) in zip(segs, segs[1:]):
        if k0 == 'stop' and k1 in ('steady', 'ramp', 'attack'):
            restart = (s1, e1)
            break
    if restart is None:
        raise MetricRefused('no stop -> restart sequence in the trace')
    s, e = restart
    sig = tr.col(signal)
    k, _ = rev_before(tr, e - 1)
    steady = _mean(sig[k:e])
    if steady < MIN_DEMAND:
        raise MetricRefused(f'post-restart demand {steady:.1f} below the measurable floor')
    first = _first(sig, lambda x: x > 0, s)
    t50 = _first(sig, lambda x: x >= 0.5 * steady, s)
    if t50 is None or t50 >= e:
        return {'steady': steady, 'first_iq_ms': INF if first is None else _dt_ms(tr, s, first),
                't50_ms': INF, 'dip_frac': 1.0}
    # Continuity = the largest give-back from a level already reached, over the two revolutions
    # after the demand first crossed 50 % of its post-restart steady value. A monotone rise
    # scores 0; rising, falling back and rising again scores the fall-back. It is evaluated on
    # a 180-degree trailing mean, which spans exactly one period of the 2/rev pedal-stroke
    # ripple, so ordinary stroke pulsation (measured by per_rev_ripple) is not counted again.
    cum = tr.col('crank_cum_deg')
    a, b = rev_after(tr, t50, 2.0)
    b = min(b, e)
    lo = s
    acc = sum(sig[s:a])
    peak = None
    worst = 0.0
    for k in range(a, b):
        acc += sig[k]
        while abs(cum[k] - cum[lo]) > 180.0 and lo < k:
            acc -= sig[lo]
            lo += 1
        m = acc / (k - lo + 1)
        peak = m if peak is None else max(peak, m)
        worst = max(worst, peak - m)
    dip = worst / steady
    return {'steady': steady, 'first_iq_ms': _dt_ms(tr, s, first),
            't50_ms': _dt_ms(tr, s, t50), 'dip_frac': dip}


def safety_zero(tr: Trace, label: str, signal: str = 'iq_ref') -> dict:
    on = onsets(tr, label)
    if not on:
        raise MetricRefused(f'no {label} event in the trace')
    i0 = on[0]
    if i0 == 0:
        raise MetricRefused(f'{label} at the first row: no demand before the event')
    sig = tr.col(signal)
    ref = tr.col('iq_ref')
    act = tr.col('iq_actual')
    lab = tr.col('gt_label')
    e = i0
    while e < len(lab) and lab[e] == label:
        e += 1
    at = sig[i0 - 1]
    if at <= 0:
        raise MetricRefused(f'demand already zero when {label} began: nothing to cut')
    # Overshoot is judged against the highest demand of the revolution before the event, so a
    # pulsating demand caught in a dip at the event row is not misread as a post-event kick.
    try:
        j, _ = rev_before(tr, i0)
        pre_max = max(sig[j:i0])
    except MetricRefused:
        pre_max = at
    r0 = _first(ref, lambda x: x <= 0, i0)
    d0 = _first(act, lambda x: x <= CURRENT_ZERO_COUNTS, i0)
    r0 = r0 if r0 is not None and r0 < e else None
    d0 = d0 if d0 is not None and d0 < e else None
    return {'iq_at_event': at,
            'ref0_ms': INF if r0 is None else _dt_ms(tr, i0, r0),
            'drive0_ms': INF if d0 is None else _dt_ms(tr, i0, d0),
            'overshoot': max(sig[i0:e]) - pre_max}


def energy(tr: Trace) -> dict:
    t = tr.col('t_s')
    i = tr.col('iq_actual')
    e = 0.0
    for k in range(1, len(t)):
        dt = t[k] - t[k - 1]
        if dt <= 0:
            raise MetricRefused('time is not strictly increasing')
        e += 0.5 * (i[k] + i[k - 1]) * dt
    return {'iq_s': e, 'duration_s': t[-1] - t[0]}


def carry(tr: Trace) -> dict:
    if 'v3_carry_state' not in tr.cols:
        return {'status': 'N/A'}
    st = tr.col('v3_carry_state')
    t = tr.col('t_s')
    lab = tr.col('gt_label')
    spd = tr.col('speed_x100')
    act = 0
    dur = dist = 0.0
    during_cut = 0
    for k in range(len(st)):
        on = st[k] > 0
        if on and (k == 0 or st[k - 1] <= 0):
            act += 1
        if on and lab[k] in ('BRAKE', 'REVERSE'):
            during_cut += 1
        if on and k > 0:
            dt = t[k] - t[k - 1]
            dur += dt
            dist += spd[k] / 100.0 / 3.6 * dt
    return {'status': 'OK', 'activations': act, 'false_positives': act,
            'duration_s': dur, 'distance_m': dist, 'active_during_cut_rows': during_cut}


def cadence_invariance(values: dict[float, float]) -> dict:
    fin = {c: v for c, v in values.items() if v is not None and math.isfinite(v)}
    if len(fin) < 2:
        raise MetricRefused('fewer than two finite cadence points')
    lo, hi = min(fin.values()), max(fin.values())
    mid = sum(fin.values()) / len(fin)
    return {'min': lo, 'max': hi, 'spread': hi - lo,
            'rel_spread': (hi - lo) / abs(mid) if mid else INF, 'points': len(fin),
            'missing': sorted(c for c in values if c not in fin)}


# --------------------------------------------------------------------------------- gates

# Harness default acceptance bounds, used for the verdict column and by the self-tests to prove
# that each metric can say NO. They are placeholders for the Architect to fix per milestone, not
# accepted V3 requirements.
CHECKS: dict[str, tuple[str, float]] = {
    'release_latency_50_deg': ('<=', 90.0),
    'phase_dip_fp_rate': ('<=', 0.05),
    'ripple_rev_median': ('<=', 0.10),
    'lower_demand_lag_ms': ('<=', 600.0),
    'attack_t63_ms': ('<=', 400.0),
    'restart_dip_frac': ('<=', 0.20),
    'safety_ref0_ms': ('<=', 1500.0),
    'safety_overshoot': ('<=', 0.0),
    'cadence_rel_spread': ('<=', 0.5),
    'carry_active_during_cut_rows': ('<=', 0.0),
}


def check(name: str, value: float | None) -> bool:
    """True = within the bound. A missing/refused value is never a pass."""
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return False
    op, bound = CHECKS[name]
    return value <= bound if op == '<=' else value >= bound


# --------------------------------------------------------------------------------- per profile

def scenario_metrics(tr: Trace, profile: str, signal: str = 'iq_ref') -> dict[str, object]:
    """Flat {metric: value | 'REFUSED: reason' | 'N/A'} for one scenario trace."""
    out: dict[str, object] = {}

    def put(prefix: str, fn: Callable[[], dict], keys: dict[str, str]):
        try:
            r = fn()
        except MetricRefused as ex:
            for k in keys.values():
                out[k] = f'REFUSED: {ex}'
            return
        for src, dst in keys.items():
            out[dst] = r.get(src)

    if profile in ('steady', 'dead_spot', 'asymmetry', 'climb', 'pas_glitch', 'stall_noise',
                   'true_release', 'gradual_release', 'attack', 'brake'):
        put('ripple', lambda: per_rev_ripple(tr, signal),
            {'median': 'ripple_rev_median', 'max': 'ripple_rev_max'})
        put('dip', lambda: phase_dip_fp(tr, signal),
            {'rate': 'phase_dip_fp_rate', 'passages': 'phase_dip_passages',
             'worst_drop': 'phase_dip_worst_drop'})
    if profile == 'true_release':
        put('rel', lambda: release_latency(tr, signal),
            {'ms_50': 'release_latency_50_ms', 'deg_50': 'release_latency_50_deg',
             'ms_10': 'release_latency_10_ms', 'deg_10': 'release_latency_10_deg'})
    if profile in ('gradual_release', 'crest'):
        put('low', lambda: lower_demand(tr, signal),
            {'ms': 'lower_demand_ms', 'deg': 'lower_demand_deg', 'lag_ms': 'lower_demand_lag_ms'})
    if profile in ('attack', 'attack_stop', 'attack_stop_restart'):
        put('att', lambda: attack_response(tr, signal),
            {'ms': 'attack_t63_ms', 'deg': 'attack_t63_deg'})
    if profile == 'attack_stop_restart':
        put('rst', lambda: restart_continuity(tr, signal),
            {'first_iq_ms': 'restart_first_iq_ms', 't50_ms': 'restart_t50_ms',
             'dip_frac': 'restart_dip_frac'})
    label = {'attack_stop': 'PEDAL_STOP', 'attack_stop_restart': 'PEDAL_STOP', 'coast': 'COAST',
             'reverse': 'REVERSE', 'brake': 'BRAKE'}.get(profile)
    if label:
        put('safe', lambda: safety_zero(tr, label, signal),
            {'ref0_ms': 'safety_ref0_ms', 'drive0_ms': 'safety_drive0_ms',
             'overshoot': 'safety_overshoot', 'iq_at_event': 'safety_iq_at_event'})
    put('energy', lambda: energy(tr), {'iq_s': 'energy_iq_s'})
    c = carry(tr)
    if c.get('status') == 'N/A':
        for k in ('carry_activations', 'carry_false_positives', 'carry_duration_s',
                  'carry_distance_m', 'carry_active_during_cut_rows'):
            out[k] = 'N/A'
    else:
        for k in ('activations', 'false_positives', 'duration_s', 'distance_m',
                  'active_during_cut_rows'):
            out['carry_' + k] = c[k]
    return out


if __name__ == '__main__':
    import json
    import sys
    if len(sys.argv) != 3:
        print('usage: assist_v3_metrics.py <trace.csv> <profile>', file=sys.stderr)
        raise SystemExit(2)
    print(json.dumps(scenario_metrics(load_trace(sys.argv[1]), sys.argv[2]), indent=1,
                     default=str))
