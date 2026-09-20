#!/usr/bin/env python3
"""Electrical + end-to-end EVistDrive SIL gate.

Two complementary binaries are built from production C:
1) foc_electrical_sil: real FOC.c + PI/current-loop + SVPWM + PWM geometry against a PMSM plant.
2) evist_full_foc_sil: real PAS/torque/assist/limits/final-Iq path plus the same real FOC/PMSM,
   with physical rotor motion generating Hall timing that is reconstructed by production rotor_angle.c.
   The same full backend also runs production Walk Assist across its supported RPM/load matrix.

The fast supervisory SIL remains separate because it can fuzz many more cases cheaply.
"""
from __future__ import annotations
import argparse, os, subprocess, sys
from pathlib import Path

R = Path(__file__).resolve().parents[1]
OUT = R / '.build' / 'electrical-sil'
OUT.mkdir(parents=True, exist_ok=True)
CC = os.environ.get('CC', 'gcc')
# Audit finding 9: link to, and launch, the platform's real executable name.
EXE_SUFFIX = '.exe' if os.name == 'nt' else ''

SUPERVISORY_MODULES = [
    'src/torque_input.c','src/rider_input.c','src/assist_modes.c','src/cadence_filter.c',
    'src/tuning_config.c',
    'src/ap2_pas_state.c','src/ap2_torque_chain.c','src/ap2_estimators.c',
    'src/ap2_profiles.c','src/ap2_limits.c','src/assist_pipeline.c',
    'src/ride_control.c','src/fast_iq_slew.c','src/battery_iq_cap.c',
    'src/iq_chain.c',
    'src/motor_core.c','src/pas_quadrature.c','src/pas_direction.c','src/pas_liveness.c',
    'src/pas_sampler.c','src/pas_cadence.c','tests/host/common/map_adapter.c',
    'tests/host/common/motor_service_stub.c'
]
FOC_MODULES = ['src/FOC.c','src/foc_current_loop.c','src/pwm_geometry.c']
FULL_EXTRA = ['src/rotor_angle.c','src/quiet_zero.c','src/walk_assist_motor.c','src/walk_speed_controller.c']


def build(exe: Path, sources: list[str], *, full=False, sanitize=False) -> None:
    flags = ['-std=c11','-Wall','-Wextra','-Werror','-Wno-error=type-limits','-Wno-error=unused-parameter']
    flags += ['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer'] if sanitize else ['-O2']
    inc = ['-Isim/full_host_stubs','-Iinc']
    if full:
        flags += ['-DEVD_SIL_REAL_FOC=1']
        inc += ['-Itests/host/common']
    cmd = [CC,*flags,*inc,'-o',str(exe),*sources,'-lm']
    p = subprocess.run(cmd,cwd=R,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    if p.returncode:
        print(p.stdout, end='')
        raise SystemExit(p.returncode)
    if p.stdout:
        # Keep known production warnings visible without failing the gate; these are separately audited.
        print(p.stdout, end='')


def run(exe: Path, args: list[str], env=None) -> str:
    p = subprocess.run([str(exe),*args],cwd=R,text=True,stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT,env=env)
    print(p.stdout,end='')
    if p.returncode:
        raise SystemExit(p.returncode)
    return p.stdout



def xorshift32(x: int) -> int:
    x &= 0xFFFFFFFF
    x ^= (x << 13) & 0xFFFFFFFF
    x ^= (x >> 17)
    x ^= (x << 5) & 0xFFFFFFFF
    return x & 0xFFFFFFFF

def advance_real_foc_fuzz_seed(seed: int, cases: int) -> int:
    # sim/evist_sil.c consumes exactly 7 xorshift words per EVD_SIL_REAL_FOC fuzz case:
    # rpm, cadence ripple, mean torque, torque ripple, breakaway Iq, bounce, start angle.
    # Keep this beside the sharding code so a future fuzz-input change cannot be missed.
    x=seed & 0xFFFFFFFF
    for _ in range(cases * 7): x=xorshift32(x)
    return x

def run_sharded(exe: Path, total: int, seed_text: str, jobs: int, env=None) -> str:
    if total <= 0: return ''
    jobs=max(1,min(jobs,total)); base=int(seed_text,0)
    counts=[total//jobs + (1 if i < total%jobs else 0) for i in range(jobs)]
    procs=[]; offset=0
    for i,n in enumerate(counts):
        seed=advance_real_foc_fuzz_seed(base,offset)
        cmd=[str(exe),'--fuzz',str(n),hex(seed)]
        procs.append((i,n,seed,subprocess.Popen(cmd,cwd=R,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,env=env)))
        offset += n
    chunks=[]
    for i,n,seed,p in procs:
        out=p.communicate()[0]; print(out,end=''); chunks.append(f'SHARD {i+1}/{jobs} cases={n} seed={hex(seed)}\n'+out)
        if p.returncode: raise SystemExit(p.returncode)
    print(f'FUZZ SHARDS PASS total={total} jobs={jobs} baseSeed={seed_text}')
    return ''.join(chunks)

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--full-fuzz',type=int,default=1000,
                    help='end-to-end real-FOC/Hall randomized cases')
    ap.add_argument('--seed',default='0xE7157A39')
    ap.add_argument('--sanitize',action='store_true')
    ap.add_argument('--sanitize-fuzz',type=int,default=100)
    ap.add_argument('--jobs',type=int,default=min(4,os.cpu_count() or 1),help='parallel deterministic fuzz shards')
    a=ap.parse_args()

    standalone=OUT/('foc_electrical_sil'+EXE_SUFFIX)
    build(standalone,['sim/foc_electrical_sil.c',*FOC_MODULES])
    report='STANDALONE REAL FOC / PMSM\n'+run(standalone,[])

    full=OUT/('evist_full_foc_sil'+EXE_SUFFIX)
    build(full,['sim/evist_sil.c',*SUPERVISORY_MODULES,*FOC_MODULES,*FULL_EXTRA],full=True)
    fixed=run(full,[])
    fuzz=run_sharded(full,a.full_fuzz,a.seed,a.jobs)
    report += '\nEND-TO-END ASSIST -> REAL FOC -> PMSM -> HALL\n'+fixed+'\n'+fuzz

    if a.sanitize:
        san=OUT/('evist_full_foc_sil_asan'+EXE_SUFFIX)
        build(san,['sim/evist_sil.c',*SUPERVISORY_MODULES,*FOC_MODULES,*FULL_EXTRA],full=True,sanitize=True)
        env=os.environ.copy()
        env['ASAN_OPTIONS']='detect_leaks=1:halt_on_error=1'
        env['UBSAN_OPTIONS']='halt_on_error=1'
        sout=run_sharded(san,a.sanitize_fuzz,a.seed,min(2,a.jobs),env)
        report += '\nFULL FOC SANITIZERS ASan+UBSan\n'+sout

    path=OUT/'REPORT.txt'
    path.write_text(report)
    print(path)

if __name__=='__main__':
    main()
