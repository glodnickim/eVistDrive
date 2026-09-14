#!/usr/bin/env python3
"""The ripple analyzer must REFUSE bad evidence, not quietly pass on it.

A quality measurement that can be satisfied by a missing file, a NaN, a clipped signal or a
limiter doing the smoothing is not a measurement - it is a green light with nothing behind it.
Audit finding 8 pointed out that the first version could be satisfied that way, so each of those
cases is now a test: feed the analyzer evidence that should not convince it, and require that it
says no.

Every case below is a NEGATIVE test. A run that exits 0 is a failure of this file.
"""
from __future__ import annotations
import csv
import subprocess
import sys
import tempfile
from pathlib import Path

R = Path(__file__).resolve().parents[1]
ANALYZER = R / 'tools/analyze_assist_ripple.py'

COLUMNS = ['tick', 'time_s', 'crank_angle_deg', 'pas_state', 'cadence_input',
           'torque_raw', 'torque_corrected', 'torque_fast', 'torque_run', 'load_centikg',
           'rider_demand', 'assist_base', 'assist_dynamic', 'assist_response',
           'aggression', 'load_state', 'auto_factor', 'iq_request', 'iq_final']

# Every scenario the analyzer requires, read from the analyzer itself so the two cannot drift
# apart. Parsed rather than imported: importing would run the module, and this file exists to
# run it deliberately, under controlled conditions.
def required_scenarios() -> list[str]:
    import ast
    text = ANALYZER.read_text(encoding='utf-8')
    tree = ast.parse(text)
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == 'SCENARIOS':
                    return list(ast.literal_eval(node.value))
    raise AssertionError('could not read SCENARIOS out of the analyzer')


def synth_rows(n: int, *, pulsating=True, response=500, iq=300, iq_request=None):
    """A trace with an honest amount of pedal ripple and a smooth current."""
    rows = []
    for i in range(n):
        phase = (i % 400) / 400.0
        tq = 200.0 + (150.0 * phase if pulsating else 0.0)
        rows.append({
            'tick': i, 'time_s': i / 4000.0, 'crank_angle_deg': 0.0, 'pas_state': 2,
            'cadence_input': 60.0, 'torque_raw': tq, 'torque_corrected': tq,
            'torque_fast': tq, 'torque_run': tq, 'load_centikg': tq * 5,
            'rider_demand': 300, 'assist_base': 280, 'assist_dynamic': 20,
            'assist_response': response, 'aggression': 0, 'load_state': 0, 'auto_factor': 0,
            'iq_request': iq if iq_request is None else iq_request, 'iq_final': iq,
        })
    return rows


def write_trace(d: Path, scenario: str, rows):
    p = d / f'{scenario}_assist.csv'
    with p.open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS)
        w.writeheader()
        w.writerows(rows)
    return p


def run(d: Path):
    return subprocess.run([sys.executable, str(ANALYZER), str(d)],
                          cwd=R, capture_output=True, text=True)


def case(name: str, build, expect_reason: str):
    scenarios = required_scenarios()
    with tempfile.TemporaryDirectory() as td:
        d = Path(td)
        # A full, healthy set first, so the only thing under test is the damage `build` does.
        n = 4000 * 8
        for sc in scenarios:
            write_trace(d, sc, synth_rows(n))
        build(d, scenarios, n)
        r = run(d)
        if r.returncode == 0:
            raise SystemExit(
                f'FAIL [{name}]: the analyzer accepted evidence it should have rejected\n'
                f'--- stdout ---\n{r.stdout}\n--- stderr ---\n{r.stderr}')
        blob = r.stdout + r.stderr
        if expect_reason not in blob:
            raise SystemExit(
                f'FAIL [{name}]: rejected, but not for the stated reason '
                f'(expected {expect_reason!r})\n--- stdout ---\n{r.stdout}\n'
                f'--- stderr ---\n{r.stderr}')
        print(f'PASS [{name}]: rejected - {expect_reason}')


def main() -> int:
    # A healthy set must PASS, or every negative below proves nothing.
    scenarios = required_scenarios()
    with tempfile.TemporaryDirectory() as td:
        d = Path(td)
        for sc in scenarios:
            write_trace(d, sc, synth_rows(4000 * 8))
        r = run(d)
        if r.returncode != 0:
            raise SystemExit(f'FAIL [control]: a healthy set was rejected\n{r.stdout}\n{r.stderr}')
        print('PASS [control]: a healthy set is accepted')

    def drop_one(d, scenarios, n):
        (d / f'{scenarios[0]}_assist.csv').unlink()
    case('missing scenario', drop_one, 'trace not produced')

    def nan_it(d, scenarios, n):
        rows = synth_rows(n)
        rows[n // 2]['iq_final'] = 'nan'
        write_trace(d, scenarios[0], rows)
    case('non-finite value', nan_it, 'non-finite')

    def zero_iq(d, scenarios, n):
        write_trace(d, scenarios[0], synth_rows(n, iq=0))
    case('no assist at all', zero_iq, 'no assist was produced')

    def saturated(d, scenarios, n):
        write_trace(d, scenarios[0], synth_rows(n, response=1000))
    case('saturated response', saturated, 'clipping, not attenuation')

    def limiter_shaped(d, scenarios, n):
        # The chain asked for far more than it delivered: a protection is doing the smoothing.
        write_trace(d, scenarios[0], synth_rows(n, iq=300, iq_request=900))
    case('limiter-shaped output', limiter_shaped, 'limiter was binding')

    def too_short(d, scenarios, n):
        write_trace(d, scenarios[0], synth_rows(4000 * 3 + 10))
    case('no steady window', too_short, 'steady samples')

    def missing_column(d, scenarios, n):
        rows = synth_rows(n)
        for r_ in rows:
            del r_['assist_base']
        p = d / f'{scenarios[0]}_assist.csv'
        cols = [c for c in COLUMNS if c != 'assist_base']
        with p.open('w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            w.writerows(rows)
    case('missing column', missing_column, 'is missing from the trace')

    print('PASS: the ripple analyzer rejects every unusable form of evidence')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
