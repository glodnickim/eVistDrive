#!/usr/bin/env python3
"""EVistDrive Level-4 virtual bicycle gate.

Builds one native executable from shipped production control/FOC/SOC modules plus virtual
rider, bicycle, road and battery plants. The simulator is intentionally slower than Level 2/3;
use --quick for the ordinary edit loop and the default/full mode before a firmware candidate.
"""
from __future__ import annotations
import argparse, os, subprocess, sys
from pathlib import Path
import re
R=Path(__file__).resolve().parents[1]
OUT=R/'.build/level4'; OUT.mkdir(parents=True,exist_ok=True)
CC=os.environ.get('CC','gcc')
# Audit finding 9: link to, and launch, the platform's real executable name.
EXE_SUFFIX='.exe' if os.name=='nt' else ''
PROD=[
 'src/torque_input.c','src/rider_input.c','src/assist_modes.c','src/cadence_filter.c',
 'src/tuning_config.c',
    'src/ap2_pas_state.c','src/ap2_rider_demand.c','src/ap2_estimators.c',
    'src/ap2_profiles.c','src/g53_port.c','src/g53_port_boundaries.c',
    'src/g53_port_pas.c','src/g53_port_chain.c','src/ap2_limits.c','src/assist_pipeline.c',
 'src/ride_control.c','src/fast_iq_slew.c','src/battery_iq_cap.c','src/iq_chain.c',
 'src/motor_core.c',
 'src/pas_quadrature.c','src/pas_direction.c','src/pas_liveness.c','src/pas_sampler.c','src/pas_cadence.c',
 'src/FOC.c','src/foc_current_loop.c','src/pwm_geometry.c','src/rotor_angle.c','src/quiet_zero.c',
 'src/soc_core.c','src/walk_assist_motor.c','src/walk_speed_controller.c',
 'tests/host/common/map_adapter.c','tests/host/common/motor_service_stub.c']
PLANT=['sim/l4/battery_pack.c','sim/l4/bike_rider.c']
L4_TEST_OBSERVER=['sim/l4/eb74_invocation_observer.c']

def verify_fuzz_contract():
    source=(R/'sim/l4/virtual_bike_l4.c').read_text(encoding='utf-8')
    required={
        'seed': 'static uint32_t fuzz_state=0x144B1CE5U;',
        'EXPECTED_START': 'bool robust_start_expected=rider_launch_nm>static_required_nm*1.15f;',
        'PRNG': 'static uint32_t rnd(void){uint32_t x=fuzz_state;x^=x<<13;x^=x>>17;x^=x<<5;return fuzz_state=x;}',
    }
    for label,needle in required.items():
        if needle not in source:
            print(f'L4-FUZZ-PRE-10 FAIL: frozen {label} source expression changed')
            raise SystemExit(1)
    start=source.index('static int run_fuzz(')
    end=source.index('\nint main(',start)
    run_body=source[start:end]
    if re.search(r'\b(?:check_count|startup_count|adaptive_zero|ready_flag)\s*=',run_body):
        print('L4-FUZZ-PRE-6 FAIL: direct EB74 readiness/state seeding found in fuzz runner')
        raise SystemExit(1)
    if 'l4_eb74_prehistory(&s' not in run_body or 'l4_eb74_observer_reset();' not in run_body:
        print('L4-FUZZ-PRE-6 FAIL: public reset/prehistory lifecycle is missing')
        raise SystemExit(1)
    print('L4-FUZZ-PRE-6 PASS: no direct EB74 state seeding; public production lifecycle only')
    print('L4-FUZZ-PRE-10 PASS: EXPECTED_START formula, seed, and xorshift32 PRNG source unchanged')

def build(exe:Path,sanitize=False):
    flags=['-std=c11','-Wall','-Wextra','-Werror','-Wno-error=type-limits','-Wno-error=unused-parameter']
    flags += ['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer'] if sanitize else ['-O2']
    cmd=[CC,*flags,'-Isim/full_host_stubs','-Isim/l4','-Iinc','-Itests/host/common','-o',str(exe),
         'sim/l4/virtual_bike_l4.c',*L4_TEST_OBSERVER,*PLANT,*PROD,
         '-Wl,--wrap=g53_ad7ec_step','-lm']
    p=subprocess.run(cmd,cwd=R,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    if p.returncode:
        print(p.stdout,end=''); raise SystemExit(p.returncode)
    if p.stdout: print(p.stdout,end='')

def run(exe:Path,args:list[str],env=None)->str:
    p=subprocess.run([str(exe),*args],cwd=R,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,env=env)
    print(p.stdout,end='')
    if p.returncode: raise SystemExit(p.returncode)
    return p.stdout

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--quick',action='store_true')
    ap.add_argument('--fuzz',type=int,default=None)
    ap.add_argument('--sanitize',action='store_true')
    a=ap.parse_args()
    verify_fuzz_contract()
    exe=OUT/('virtual_bike_l4'+EXE_SUFFIX); build(exe)
    report=run(exe,[])
    n=a.fuzz if a.fuzz is not None else (25 if a.quick else 100)
    report+='\n'+run(exe,['--fuzz',str(n)])
    if a.sanitize:
        san=OUT/('virtual_bike_l4_asan'+EXE_SUFFIX); build(san,True)
        env=os.environ.copy(); env['ASAN_OPTIONS']='detect_leaks=1:halt_on_error=1'; env['UBSAN_OPTIONS']='halt_on_error=1'
        report+='\nSANITIZERS\n'+run(san,['--fuzz','10' if a.quick else '25'],env)
    (OUT/'REPORT.txt').write_text(report)
    print(OUT/'REPORT.txt')
if __name__=='__main__': main()
