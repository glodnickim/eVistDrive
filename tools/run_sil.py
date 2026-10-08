#!/usr/bin/env python3
from __future__ import annotations
import argparse, os, shutil, subprocess, sys
from pathlib import Path
# Audit finding 9: link to, and launch, the platform's real executable name.
EXE_SUFFIX='.exe' if os.name=='nt' else ''

R=Path(__file__).resolve().parents[1]
mods=[
 'src/torque_input.c','src/rider_input.c','src/assist_modes.c','src/cadence_filter.c',
 'src/tuning_config.c',
    'src/ap2_pas_state.c','src/ap2_rider_demand.c','src/ap2_estimators.c',
    'src/ap2_profiles.c','src/g53_port.c','src/g53_port_boundaries.c',
    'src/g53_port_pas.c','src/g53_port_chain.c','src/g53_g1_limiter.c','src/ap2_limits.c','src/assist_pipeline.c',
 'src/ride_control.c','src/fast_iq_slew.c','src/battery_iq_cap.c',
 'src/iq_chain.c',
 'src/motor_core.c','src/pas_quadrature.c','src/pas_direction.c','src/pas_liveness.c',
 'src/pas_sampler.c','src/pas_cadence.c',
 # ASSIST-V3: production crank step accumulator (fed by the harness drain, REVIEW 1 #19) and the
 # V3 stage. Linked always; referenced by the pipeline only when built with -DASSIST_V3.
 'src/crank_phase.c','src/assist_v3.c','src/assist_v3_intent.c','src/assist_motion.c','src/motion_est.c',
 'src/assist_v3_config.c',
 'tests/host/common/map_adapter.c',
 'tests/host/common/motor_service_stub.c']
# The Assist V3 candidate compiles the V3 shadow stage in (tools/build_firmware.py default).
ASSIST_V3_FLAGS=['-DASSIST_V3=1']

def build(exe:Path, sanitize=False, extra=()):
    cc=os.environ.get('CC','gcc')
    flags=['-std=c11','-Wall','-Wextra','-Werror','-Wno-type-limits']
    if sanitize:
        flags += ['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer']
    else:
        flags += ['-O2']
    cmd=[cc,*flags,*extra,'-Isim/l4','-Iinc','-Itests/host/common/host_stubs','-Itests/host/common',
         '-o',str(exe),'sim/evist_sil.c','sim/l4/eb74_invocation_observer.c',*mods,
         '-Wl,--wrap=g53_ad7ec_step','-lm']
    p=subprocess.run(cmd,cwd=R,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    if p.returncode:
        print(p.stdout); raise SystemExit(p.returncode)

def run(exe:Path,args,env=None,cwd=R,echo=True):
    p=subprocess.run([str(exe),*args],cwd=cwd,text=True,stdout=subprocess.PIPE,
                     stderr=subprocess.STDOUT,env=env)
    if echo: print(p.stdout,end='')
    if p.returncode:
        if not echo: print(p.stdout,end='')
        raise SystemExit(p.returncode)
    return p.stdout

def assist_v3_equivalence(fixed_on:str, fuzz_on:str, fuzz:int, seed:str)->str:
    """TEST_MATRIX G-EQ: the same harness built WITHOUT -DASSIST_V3 must produce byte-identical
    fixed-mode stdout, fixed-mode CSVs and fuzz stdout. Milestone B is shadow only, so any
    difference is a defect of the shadow stage or of its plumbing."""
    out=R/'.build/sil-v3off'; wd=out/'wd'
    if wd.exists(): shutil.rmtree(wd)
    (wd/'.build'/'sil').mkdir(parents=True)
    exe_off=out/('evist_sil_v3off'+EXE_SUFFIX); build(exe_off)
    fixed_off=run(exe_off,[],cwd=wd,echo=False)
    fuzz_off=run(exe_off,['--fuzz',str(fuzz),seed],cwd=wd,echo=False)
    problems=[]
    if fixed_off!=fixed_on: problems.append('fixed-mode stdout differs')
    if fuzz_off!=fuzz_on: problems.append('fuzz stdout differs')
    on_dir=R/'.build/sil'; off_dir=wd/'.build'/'sil'
    names=sorted(p.name for p in off_dir.glob('*.csv'))
    if not names: problems.append('compiled-out run wrote no fixed-mode CSV')
    for n in names:
        if not (on_dir/n).exists() or (on_dir/n).read_bytes()!=(off_dir/n).read_bytes():
            problems.append(f'fixed-mode CSV differs: {n}')
    if problems:
        print('G-EQ ASSIST_V3 on/off: FAIL',problems); raise SystemExit(1)
    line=(f'G-EQ ASSIST_V3 on/off: PASS (fixed stdout + {len(names)} CSVs + fuzz({fuzz}) stdout '
          'byte-identical)\n')
    print(line,end='')
    return line

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--fuzz',type=int,default=1000,help='normal deterministic fuzz cases')
    ap.add_argument('--seed',default='0xE7157A39')
    ap.add_argument('--sanitize',action='store_true',help='also run ASan+UBSan fuzz gate')
    ap.add_argument('--sanitize-fuzz',type=int,default=1000)
    ap.add_argument('--assist-v3',choices=['on','off','both'],default='both',
                    help='on: candidate build (-DASSIST_V3); off: compiled out; both (default): '
                         'run "on" and require byte-identical outputs from "off" (G-EQ)')
    a=ap.parse_args()

    out=R/'.build/sil'; out.mkdir(parents=True,exist_ok=True)
    extra=ASSIST_V3_FLAGS if a.assist_v3 in ('on','both') else []
    exe=out/('evist_sil'+EXE_SUFFIX); build(exe,extra=extra)
    fixed=run(exe,[])
    fuzz=run(exe,['--fuzz',str(a.fuzz),a.seed])
    report=fixed+'\n'+fuzz
    if a.assist_v3=='both':
        report+='\n'+assist_v3_equivalence(fixed,fuzz,a.fuzz,a.seed)

    if a.sanitize:
        sout=R/'.build/sil-asan'; sout.mkdir(parents=True,exist_ok=True)
        sexe=sout/('evist_sil_asan'+EXE_SUFFIX); build(sexe,True,extra)
        env=os.environ.copy(); env['ASAN_OPTIONS']='detect_leaks=1:halt_on_error=1'; env['UBSAN_OPTIONS']='halt_on_error=1'
        san=run(sexe,['--fuzz',str(a.sanitize_fuzz),a.seed],env)
        report += '\nSANITIZERS ASan+UBSan\n'+san

    (out/'REPORT.txt').write_text(report)
    print(out/'REPORT.txt')

if __name__=='__main__': main()
