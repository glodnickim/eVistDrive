#!/usr/bin/env python3
"""Single local quality gate for EVistDrive.

Runs the real-module host suites, deterministic whole-pipeline regression, closed-loop
supervisory SIL, real electrical FOC/PMSM/Hall SIL, Level-4 rider/bike/battery/SOC, recorded-ride replay, stress fuzzing, ASan/UBSan, BL820 packaging
and source-manifest hygiene. If the exact Arm GNU toolchain is available, --target also performs
the debug Developer target build using the cross-platform Python builder.
"""
from __future__ import annotations
import argparse, os, shutil, subprocess, sys, tempfile, time
from pathlib import Path
R=Path(__file__).resolve().parents[1]

def step(name, cmd, env=None):
    print(f'\n=== {name} ===',flush=True); t=time.time()
    p=subprocess.run(cmd,cwd=R,env=env)
    if p.returncode:
        print(f'FAIL {name}: exit {p.returncode}',file=sys.stderr); raise SystemExit(p.returncode)
    print(f'PASS {name} ({time.time()-t:.1f}s)')

def manifest_gate():
    entries=[]
    for line in (R/'scripts/sources-m820.txt').read_text().splitlines():
        line=line.strip()
        if line and not line.startswith('#'): entries.append(line)
    missing=[x for x in entries if not (R/x).is_file()]
    product={x for x in entries if x.startswith('src/')}
    actual={p.relative_to(R).as_posix() for p in (R/'src').glob('*.c')}
    unlisted=sorted(actual-product)
    if missing or unlisted:
        print('missing manifest files:',missing); print('unlisted src/*.c:',unlisted); raise SystemExit(2)
    print(f'PASS source manifest: {len(product)}/{len(actual)} production C files listed; total entries={len(entries)}')

def diff_gate():
    if not (R/'.git').exists():
        print('SKIP git diff --check: exported tree has no .git'); return
    p=subprocess.run(['git','diff','--check'],cwd=R,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    if p.returncode: print(p.stdout); raise SystemExit(p.returncode)
    print('PASS git diff --check')

def sanitizers_available() -> bool:
    cc=os.environ.get('CC','gcc')
    with tempfile.TemporaryDirectory() as td:
        src=os.path.join(td,'probe.c'); exe=os.path.join(td,'probe'+( '.exe' if os.name=='nt' else ''))
        with open(src,'w',encoding='ascii') as f: f.write('int main(void){return 0;}\n')
        p=subprocess.run([cc,'-fsanitize=address,undefined',src,'-o',exe],
                         cwd=R,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        return p.returncode==0

def target_build(require, variant):
    gcc=shutil.which('arm-none-eabi-gcc') or shutil.which('arm-none-eabi-gcc.exe')
    if not gcc and os.name == 'nt':
        candidate=Path(r'C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\13.2 Rel1\bin\arm-none-eabi-gcc.exe')
        if candidate.is_file(): gcc=str(candidate)
    if not gcc:
        msg='TARGET BUILD SKIP: arm-none-eabi-gcc not found'
        if require: print(msg,file=sys.stderr); raise SystemExit(3)
        print(msg); return
    ver=subprocess.check_output([gcc,'-dumpfullversion','-dumpversion'],text=True).strip().splitlines()[0]
    if ver!='13.2.1':
        msg=f'TARGET BUILD SKIP: exact compiler 13.2.1 required, found {ver}'
        if require: print(msg,file=sys.stderr); raise SystemExit(3)
        print(msg); return
    toolbin=str(Path(gcc).resolve().parent)
    cmd=[sys.executable,'tools/build_firmware.py','--variant',variant,'--mode','developer',
         '--toolchain',toolbin,'--output-dir',str(R/'.build/verify-target')]
    step('exact ARM target build',cmd)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--quick',action='store_true',help='1000 fuzz, skip sanitizers')
    ap.add_argument('--target',action='store_true',help='attempt exact ARM target build when tools exist')
    ap.add_argument('--require-target',action='store_true',help='fail if exact target toolchain cannot run')
    ap.add_argument('--target-variant',choices=['normal','diagnostic'],default='normal',
                    help='target build variant; diagnostic enables FW145 live telemetry')
    a=ap.parse_args()
    print('EVistDrive full verification gate')
    manifest_gate(); diff_gate()
    step('cross-platform target-tree + BL820 packager self-check',
         [sys.executable,'tools/build_firmware.py','--check-only'])
    step('independent BL820 container regression',[sys.executable,'tests/tools_prepare_m820_bl820.py'])
    step('real-module host suites',[sys.executable,'tools/run_host_tests.py'])
    step('whole-pipeline deterministic regression',[sys.executable,'tools/run_regression.py'])
    # The one property the assist pipeline exists for: the motor must not reproduce the
    # pedal ripple. Measured from the traces the step above just produced.
    step('assist ripple attenuation',[sys.executable,'tools/analyze_assist_ripple.py'])
    san_ok=(not a.quick) and sanitizers_available()
    if a.quick:
        san_tag=''
    elif san_ok:
        san_tag=' + ASan/UBSan'
    else:
        san_tag=' (ASan/UBSan SKIP: no sanitizer runtime linkable with CC=%s)'%os.environ.get('CC','gcc')
    sil=[sys.executable,'tools/run_sil.py','--fuzz','1000' if a.quick else '10000']
    if san_ok: sil += ['--sanitize','--sanitize-fuzz','1000']
    step('closed-loop SIL + deterministic fuzz'+san_tag,sil)
    electrical=[sys.executable,'tools/run_electrical_sil.py','--full-fuzz','250' if a.quick else '1000']
    if san_ok: electrical += ['--sanitize','--sanitize-fuzz','100']
    step('real FOC/PMSM/Hall electrical SIL'+san_tag,electrical)
    level4=[sys.executable,'tools/run_level4.py','--fuzz','25' if a.quick else '100']
    if san_ok: level4 += ['--sanitize']
    step('Level-4 virtual rider + bicycle + battery/SOC + real FOC'+san_tag,level4)
    step('recorded-ride import/replay deterministic regression',[sys.executable,'tools/run_replay_regression.py'])
    step('CANable FW145 raw-log decode -> canonical -> native replay',[sys.executable,'tests/test_canable_ride_decode.py'])
    if a.target or a.require_target: target_build(a.require_target, a.target_variant)
    print('\n==================================================')
    print('EVistDrive PC VERIFICATION: PASS')
    print('Target build is a separate gate; use --target / --require-target with Arm GNU 13.2.1.')
    print('==================================================')

if __name__=='__main__': main()
