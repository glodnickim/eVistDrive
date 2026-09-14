#!/usr/bin/env python3
"""Replay infrastructure gate + registered real-ride regressions.

THREE SEPARATE VERDICTS PER CASE. They answer different questions and used to be printed as one
line reading CASE PASS, which promised more than it had checked - audit finding 8.

    REPLAY_EXECUTED     the production chain consumed the recorded sensor history and produced a
                        trace. Infrastructure only. Says nothing about the assist.
    BEHAVIOR_ACCEPTED   the trace satisfies criteria stated in the case manifest: assist present,
                        response in the right direction, release to zero, restart, attenuation.
                        A case with no criteria is BEHAVIOR_NOT_ASSESSED - never a pass.
    OUTPUT_PINNED       the bytes match a hash a human accepted after validating that ride. Most
                        cases are deliberately unpinned until the physical bike confirms them.
"""
from __future__ import annotations
import csv, hashlib, json, subprocess, sys
from pathlib import Path

R = Path(__file__).resolve().parents[1]
B = R / '.build/replay'
B.mkdir(parents=True, exist_ok=True)
sys.path.insert(0, str(R / 'tools'))
from replay_behavior import evaluate  # noqa: E402


def run(*args):
    subprocess.run([sys.executable, *map(str, args)], cwd=R, check=True)


def sha(p: Path):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def rows(p: Path):
    with p.open(newline='', encoding='utf-8') as f:
        return sum(1 for _ in f) - 1


def smoke():
    src = R / 'tests/host/out/RUN_60_ride.csv'
    can = B / 'smoke.canonical.csv'
    a = B / 'smoke.a.csv'
    b = B / 'smoke.b.csv'
    run('tools/import_ride_log.py', src, can)
    run('tools/run_replay.py', can, '--output', a)
    run('tools/run_replay.py', can, '--output', b)
    if rows(can) != 24000 or rows(a) != 24000:
        raise SystemExit('replay smoke row-count mismatch')
    if a.read_bytes() != b.read_bytes():
        raise SystemExit('replay nondeterministic: repeated output differs')
    print(f'REPLAY SMOKE PASS rows=24000 deterministic sha256={sha(a)}')


def registered():
    root = R / 'sim/replay/cases'
    if not root.exists():
        print('REPLAY registered cases: 0')
        return
    executed = accepted = not_assessed = pinned = 0
    failures: list[str] = []

    for mf in sorted(root.glob('*/manifest.json')):
        m = json.loads(mf.read_text())
        name = mf.parent.name
        inp = mf.parent / m.get('input', 'input.csv')
        out = B / (name + '.replayed.csv')

        run('tools/run_replay.py', inp, '--output', out)
        executed += 1
        print(f'REPLAY_EXECUTED    {name} rows={rows(out)}')

        res = evaluate(out, m.get('behavior') or {})
        if not res.assessed:
            not_assessed += 1
            print(f'  BEHAVIOR_NOT_ASSESSED {name}: the manifest states no criteria, so nothing '
                  f'about this ride has been checked beyond the replay completing')
        else:
            for cname, ok, detail in res.checks:
                print(f'  {"ok  " if ok else "FAIL"} {cname:<18} {detail}')
            if res.ok:
                accepted += 1
                print(f'  BEHAVIOR_ACCEPTED  {name} ({len(res.checks)} criteria)')
            else:
                bad = [c for c, ok, _ in res.checks if not ok]
                failures.append(f'{name}: {", ".join(bad)}')
                print(f'  BEHAVIOR_REJECTED  {name}: {", ".join(bad)}')

        expected = m.get('accepted_output_sha256')
        if expected:
            got = sha(out)
            if got != expected:
                failures.append(f'{name}: output changed ({got} != {expected})')
                print(f'  OUTPUT_PINNED      {name} CHANGED {got} != {expected}')
            else:
                pinned += 1
                print(f'  OUTPUT_PINNED      {name} matches')
        else:
            note = m.get('accepted_output_note', 'no accepted hash recorded')
            print(f'  OUTPUT_NOT_PINNED  {name}: {note}')

    print()
    print(f'REPLAY_EXECUTED    {executed}/{executed}')
    print(f'BEHAVIOR_ACCEPTED  {accepted}/{executed}'
          + (f', NOT_ASSESSED {not_assessed}' if not_assessed else ''))
    print(f'OUTPUT_PINNED      {pinned}/{executed}')
    if failures:
        print('FAIL: registered rides did not behave as their manifests require', file=sys.stderr)
        for f in failures:
            print(f'  {f}', file=sys.stderr)
        raise SystemExit(1)
    if not_assessed:
        print(f'WARNING: {not_assessed} case(s) carry no behaviour criteria; they prove the '
              f'replay runs and nothing more', file=sys.stderr)


def main():
    smoke()
    registered()


if __name__ == '__main__':
    main()
