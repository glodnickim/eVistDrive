#!/usr/bin/env python3
from __future__ import annotations
import argparse, os, subprocess, sys
from pathlib import Path

R=Path(__file__).resolve().parents[1]
mods=[
 'src/torque_input.c','src/rider_input.c','src/assist_modes.c','src/cadence_filter.c',
 'src/tuning_config.c',
    'src/ap2_pas_state.c','src/ap2_rider_demand.c','src/ap2_estimators.c',
    'src/ap2_profiles.c','src/ap2_limits.c','src/assist_pipeline.c',
 'src/ride_control.c','src/fast_iq_slew.c','src/battery_iq_cap.c',
 'src/iq_chain.c',
 'src/motor_core.c','src/pas_quadrature.c','src/pas_direction.c','src/pas_liveness.c',
 'src/pas_sampler.c','src/pas_cadence.c','tests/host/common/map_adapter.c',
 'tests/host/common/motor_service_stub.c']

def build(exe:Path, sanitize=False):
    cc=os.environ.get('CC','gcc')
    flags=['-std=c11','-Wall','-Wextra','-Werror','-Wno-type-limits']
    if sanitize:
        flags += ['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer']
    else:
        flags += ['-O2']
    cmd=[cc,*flags,'-Iinc','-Itests/host/common/host_stubs','-Itests/host/common',
         '-o',str(exe),'sim/evist_sil.c',*mods,'-lm']
    p=subprocess.run(cmd,cwd=R,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    if p.returncode:
        print(p.stdout); raise SystemExit(p.returncode)

def run(exe:Path,args,env=None):
    p=subprocess.run([str(exe),*args],cwd=R,text=True,stdout=subprocess.PIPE,
                     stderr=subprocess.STDOUT,env=env)
    print(p.stdout,end='')
    if p.returncode: raise SystemExit(p.returncode)
    return p.stdout

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--fuzz',type=int,default=1000,help='normal deterministic fuzz cases')
    ap.add_argument('--seed',default='0xE7157A39')
    ap.add_argument('--sanitize',action='store_true',help='also run ASan+UBSan fuzz gate')
    ap.add_argument('--sanitize-fuzz',type=int,default=1000)
    a=ap.parse_args()

    out=R/'.build/sil'; out.mkdir(parents=True,exist_ok=True)
    exe=out/'evist_sil'; build(exe)
    fixed=run(exe,[])
    fuzz=run(exe,['--fuzz',str(a.fuzz),a.seed])
    report=fixed+'\n'+fuzz

    if a.sanitize:
        sout=R/'.build/sil-asan'; sout.mkdir(parents=True,exist_ok=True)
        sexe=sout/'evist_sil_asan'; build(sexe,True)
        env=os.environ.copy(); env['ASAN_OPTIONS']='detect_leaks=1:halt_on_error=1'; env['UBSAN_OPTIONS']='halt_on_error=1'
        san=run(sexe,['--fuzz',str(a.sanitize_fuzz),a.seed],env)
        report += '\nSANITIZERS ASan+UBSan\n'+san

    (out/'REPORT.txt').write_text(report)
    print(out/'REPORT.txt')

if __name__=='__main__': main()
