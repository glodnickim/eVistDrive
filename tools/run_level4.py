#!/usr/bin/env python3
"""EVistDrive Level-4 virtual bicycle gate.

Builds one native executable from shipped production control/FOC/SOC modules plus virtual
rider, bicycle, road and battery plants. The simulator is intentionally slower than Level 2/3;
use --quick for the ordinary edit loop and the default/full mode before a firmware candidate.
"""
from __future__ import annotations
import argparse, csv, os, shutil, subprocess, sys
from pathlib import Path
import re
R=Path(__file__).resolve().parents[1]
OUT=R/'.build/level4'; OUT.mkdir(parents=True,exist_ok=True)
CC=os.environ.get('CC','gcc')
# Audit finding 9: link to, and launch, the platform's real executable name.
EXE_SUFFIX='.exe' if os.name=='nt' else ''
PROD=[
 'src/torque_input.c','src/rider_input.c','src/ride_wheel.c','src/assist_modes.c','src/cadence_filter.c',
 'src/tuning_config.c',
    'src/ap2_pas_state.c','src/ap2_rider_demand.c','src/ap2_estimators.c',
    'src/ap2_profiles.c','src/g53_port.c','src/g53_port_boundaries.c',
    'src/g53_port_pas.c','src/g53_port_chain.c','src/g53_g1_limiter.c','src/ap2_limits.c','src/assist_pipeline.c',
 'src/ride_control.c','src/fast_iq_slew.c','src/battery_iq_cap.c','src/iq_chain.c',
 'src/motor_core.c',
 'src/pas_quadrature.c','src/pas_direction.c','src/pas_liveness.c','src/pas_sampler.c','src/pas_cadence.c',
 'src/FOC.c','src/foc_current_loop.c','src/pwm_geometry.c','src/rotor_angle.c','src/quiet_zero.c',
 'src/soc_core.c','src/walk_assist_motor.c','src/walk_speed_controller.c',
 # ASSIST-V3: production crank step accumulator (fed by the harness drain, REVIEW 1 #19) and the
 # V3 stage. Linked always; referenced by the pipeline only when built with -DASSIST_V3.
 'src/crank_phase.c','src/assist_v3.c','src/assist_v3_intent.c','src/assist_motion.c','src/motion_est.c',
 'src/assist_v3_config.c',
 'tests/host/common/map_adapter.c','tests/host/common/motor_service_stub.c']
# The Assist V3 candidate compiles the V3 shadow stage in (tools/build_firmware.py default).
ASSIST_V3_FLAGS=['-DASSIST_V3=1']
PLANT=['sim/l4/battery_pack.c','sim/l4/bike_rider.c','sim/l4/wheel_sensor.c']
L4_TEST_OBSERVER=['sim/l4/eb74_invocation_observer.c']
SCRIPTS=['climb_pedal_stop_obstacle','crest_pedal_stop','coast_stop','reverse_while_motor']

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

def build(exe:Path,sanitize=False,extra=()):
    flags=['-std=c11','-Wall','-Wextra','-Werror','-Wno-error=type-limits','-Wno-error=unused-parameter']
    flags += ['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer'] if sanitize else ['-O2']
    cmd=[CC,*flags,*extra,'-Isim/full_host_stubs','-Isim/l4','-Iinc','-Itests/host/common','-o',str(exe),
         'sim/l4/virtual_bike_l4.c',*L4_TEST_OBSERVER,*PLANT,*PROD,
         '-Wl,--wrap=g53_ad7ec_step','-lm']
    p=subprocess.run(cmd,cwd=R,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    if p.returncode:
        print(p.stdout,end=''); raise SystemExit(p.returncode)
    if p.stdout: print(p.stdout,end='')

def run(exe:Path,args:list[str],env=None,cwd=R,echo=True)->str:
    p=subprocess.run([str(exe),*args],cwd=cwd,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,env=env)
    if echo or p.returncode: print(p.stdout,end='')
    if p.returncode: raise SystemExit(p.returncode)
    return p.stdout

def assist_v3_equivalence(fixed_on:str,fuzz_on:str,n:int,script_on:dict[str,str])->str:
    """TEST_MATRIX G-EQ: the same Level-4 binary built WITHOUT -DASSIST_V3 must produce
    byte-identical stdout and CSVs. Milestone B is shadow only: any difference is a defect."""
    wd=OUT/'v3off'/'wd'
    if wd.exists(): shutil.rmtree(wd)
    (wd/'.build'/'level4').mkdir(parents=True)
    exe_off=OUT/'v3off'/('virtual_bike_l4_v3off'+EXE_SUFFIX); build(exe_off)
    fixed_off=run(exe_off,[],cwd=wd,echo=False)
    fuzz_off=run(exe_off,['--fuzz',str(n)],cwd=wd,echo=False)
    for name in SCRIPTS:
        off=run(exe_off,['--script',name],cwd=wd,echo=False)
        if off!=script_on[name]:
            print(f'G-EQ L4 script stdout differs: {name}')
            raise SystemExit(1)
    problems=[]
    if fixed_off!=fixed_on: problems.append('fixed stdout differs')
    if fuzz_off!=fuzz_on: problems.append('fuzz stdout differs')
    off_dir=wd/'.build'/'level4'
    names=sorted(p.name for p in off_dir.glob('*.csv'))
    if not names: problems.append('compiled-out run wrote no CSV')
    for name in names:
        if not (OUT/name).exists() or (OUT/name).read_bytes()!=(off_dir/name).read_bytes():
            problems.append(f'CSV differs: {name}')
    if problems:
        print('G-EQ L4 ASSIST_V3 on/off: FAIL',problems); raise SystemExit(1)
    line=f'G-EQ L4 ASSIST_V3 on/off: PASS (stdout + {len(names)} CSVs + fuzz({n}) byte-identical)\n'
    print(line,end='')
    return line

def carry_scripts(exe: Path) -> str:
    lines=[]
    for name in SCRIPTS:
        run(exe,['--script-v3',name],echo=False)
        with (OUT/f'{name}.v3.csv').open(newline='') as f:
            rows=list(csv.DictReader(f))
        active=[r for r in rows if int(r['v3_carry_state'])==1]
        starts=sum(int(r['v3_carry_state'])==1 and
                   (i==0 or int(rows[i-1]['v3_carry_state'])!=1)
                   for i,r in enumerate(rows))
        duration=len(active)*0.005
        distance=(float(active[-1]['distance_m'])-float(active[0]['distance_m'])) if active else 0.0
        reasons={int(r['v3_carry_cancel_reason']) for r in rows}
        want=1 if name=='climb_pedal_stop_obstacle' else 0
        ok=(starts==want and duration<=1.2 and distance<=1.5 and
            (want==0 or bool(reasons & {3,4,5,6})))
        line=f'L4 CARRY {name} active={starts} duration={duration:.3f}s distance={distance:.3f}m cancel={sorted(reasons)} {"PASS" if ok else "FAIL"}'
        print(line); lines.append(line)
        if not ok: raise SystemExit(1)
    return '\n'.join(lines)+'\n'

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--quick',action='store_true')
    ap.add_argument('--fuzz',type=int,default=None)
    ap.add_argument('--sanitize',action='store_true')
    ap.add_argument('--assist-v3',choices=['on','off','both'],default='both',
                    help='on: candidate build (-DASSIST_V3); off: compiled out; both (default): '
                         'run "on" and require byte-identical outputs from "off" (G-EQ)')
    a=ap.parse_args()
    verify_fuzz_contract()
    sensor_exe=OUT/('l4_wheel_sensor_host'+EXE_SUFFIX)
    sensor_cmd=[CC,'-std=c11','-Wall','-Wextra','-Werror','-Isim/l4','-Iinc',
                '-o',str(sensor_exe),'tests/l4_wheel_sensor_host.c','sim/l4/wheel_sensor.c','src/ride_wheel.c']
    subprocess.run(sensor_cmd,cwd=R,check=True)
    run(sensor_exe,[])
    freewheel_exe=OUT/('l4_freewheel_host'+EXE_SUFFIX)
    freewheel_cmd=[CC,'-std=c11','-Wall','-Wextra','-Werror','-Isim/l4',
                   '-o',str(freewheel_exe),'tests/l4_freewheel_host.c','sim/l4/bike_rider.c','-lm']
    subprocess.run(freewheel_cmd,cwd=R,check=True)
    run(freewheel_exe,[])
    extra=ASSIST_V3_FLAGS if a.assist_v3 in ('on','both') else []
    exe=OUT/('virtual_bike_l4'+EXE_SUFFIX); build(exe,extra=extra)
    fixed=run(exe,[])
    script_out={}
    for name in SCRIPTS:
        script_out[name]=run(exe,['--script',name])
        original=(OUT/f'{name}.csv').read_bytes()
        second=run(exe,['--script',name],echo=False)
        if original!=(OUT/f'{name}.csv').read_bytes() or second!=script_out[name]:
            raise SystemExit(f'L4 script nondeterministic: {name}')
    print(f'L4 scripts deterministic: {len(SCRIPTS)}/{len(SCRIPTS)} PASS')
    n=a.fuzz if a.fuzz is not None else (25 if a.quick else 100)
    fuzz=run(exe,['--fuzz',str(n)])
    report=fixed+'\n'+fuzz
    if a.assist_v3=='both':
        report+='\n'+assist_v3_equivalence(fixed,fuzz,n,script_out)
    if a.assist_v3 in ('on','both'):
        report+='\n'+carry_scripts(exe)
    if a.sanitize:
        san=OUT/('virtual_bike_l4_asan'+EXE_SUFFIX); build(san,True,extra)
        env=os.environ.copy(); env['ASAN_OPTIONS']='detect_leaks=1:halt_on_error=1'; env['UBSAN_OPTIONS']='halt_on_error=1'
        report+='\nSANITIZERS\n'+run(san,['--fuzz','10' if a.quick else '25'],env)
    (OUT/'REPORT.txt').write_text(report)
    print(OUT/'REPORT.txt')
if __name__=='__main__': main()
