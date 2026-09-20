#!/usr/bin/env python3
"""Portable whole-pipeline regression runner.

Mirrors tests/host/run_regression.ps1 with native gcc so Linux/CI executes the
same shipped C harnesses and production modules. It does not replace the
67-suite run_host_tests.py gate; it complements it with deterministic traces.
"""
from __future__ import annotations
import csv, math, os, statistics, subprocess, sys
from pathlib import Path

R = Path(__file__).resolve().parents[1]
T = R / 'tests/host'
C = T / 'common'
S = R / 'src'
I = R / 'inc'
STUB = C / 'host_stubs'
OUT = R / '.build/regression-linux'
OBJ = OUT / 'obj'
OUT.mkdir(parents=True, exist_ok=True)
OBJ.mkdir(parents=True, exist_ok=True)
CC = os.environ.get('CC', 'gcc')
EXE_SUFFIX = '.exe' if os.name == 'nt' else ''

TYPE_LIMITS = {'torque_input.c', 'assist_modes.c'}

def run(cmd, *, capture=False):
    p = subprocess.run(cmd, cwd=R, text=True,
                       stdout=subprocess.PIPE if capture else None,
                       stderr=subprocess.STDOUT if capture else None)
    if p.returncode:
        if capture and p.stdout: print(p.stdout)
        raise SystemExit(p.returncode)
    return p.stdout if capture else ''

def compile_obj(src: Path, obj: Path, incdirs, allow_type_limits=False):
    cmd=[CC,'-std=c11','-Wall','-Wextra','-Werror']
    if allow_type_limits: cmd += ['-Wno-type-limits']
    for d in incdirs: cmd += ['-I', str(d)]
    cmd += ['-c', str(src), '-o', str(obj)]
    run(cmd)

def build(name, harness_rel, common_files, modules, use_stubs=False):
    incdirs = ([STUB] if use_stubs else []) + [C, I]
    objs=[]
    ho=OBJ/f'{name}.harness.o'; compile_obj(T/harness_rel,ho,incdirs); objs.append(ho)
    for cf in common_files:
        o=OBJ/f'{name}.common.{Path(cf).name}.o'; compile_obj(C/cf,o,incdirs); objs.append(o)
    for m in modules:
        o=OBJ/f'{name}.src.{m}.o'; compile_obj(S/m,o,incdirs,m in TYPE_LIMITS); objs.append(o)
    # The linker output must carry the platform's executable extension, and the runner must
    # launch EXACTLY the path it linked. Without this, a Windows gcc writes name.exe while the
    # runner tries to execute a stale extensionless `name` left behind by a Linux build in the
    # same directory - which fails with WinError 193 and stops the whole gate before SIL,
    # Level-4 and replay ever run. Correctness must not depend on remembering to clean.
    exe=OBJ/(name+EXE_SUFFIX)
    run([CC,'-o',str(exe),*[str(x) for x in objs],'-lm'])
    return exe

def read_csv(path):
    with open(path,newline='') as f: return list(csv.DictReader(f))

def fval(row,k):
    try:return float(row[k])
    except:return math.nan

def metric(rows,k):
    vals=[fval(r,k) for r in rows]
    vals=[v for v in vals if math.isfinite(v)]
    if not vals:return None
    vals_s=sorted(vals); n=len(vals_s)
    def pct(p):
        if n==1:return vals_s[0]
        x=(n-1)*p; a=int(math.floor(x)); b=int(math.ceil(x));
        return vals_s[a] if a==b else vals_s[a]+(vals_s[b]-vals_s[a])*(x-a)
    mean=sum(vals)/n
    return dict(Mean=mean,Min=min(vals),Max=max(vals),P5=pct(.05),P95=pct(.95),
                PeakToPeak=max(vals)-min(vals),StdDev=statistics.pstdev(vals),
                Ripple=(max(vals)-min(vals))/abs(mean) if mean else 0.0)

# The whole assist chain, exactly as the firmware links it.
assist_mod=['torque_input.c','rider_input.c','assist_modes.c','tuning_config.c',
            'ap2_pas_state.c','ap2_rider_demand.c','ap2_estimators.c','ap2_profiles.c',
            'ap2_limits.c','battery_iq_cap.c','fast_iq_slew.c','assist_pipeline.c']
ride_mod=assist_mod+['ride_control.c','iq_chain.c','motor_core.c']

torque=build('torque_trace',Path('torque/torque_trace_host.c'),['crank_model.c'],['torque_input.c'])
assist=build('assist_pipeline',Path('pipeline/assist_pipeline_host.c'),['crank_model.c'],assist_mod)
ride=build('ride_control_pipeline',Path('pipeline/ride_control_pipeline_host.c'),
           ['crank_model.c','map_adapter.c','motor_service_stub.c'],ride_mod,True)
burst=build('missed_tick_burst',Path('scenarios/missed_tick_burst_host.c'),['crank_model.c'],
            ['torque_input.c','ride_episode.c'])

scenarios=['RUN_60','RUN_80','RUN_100','RUN_110','RUN_120','CADENCE_RAMP_50_120']
# Cruise scenarios exist to measure ripple attenuation and run only on the assist layer:
# the torque and ride layers have nothing extra to say about them.
assist_only=['CRUISE_20_ECO','CRUISE_20_TRAIL','CRUISE_20_SPORT','CRUISE_20_SPORTPLUS',
             'CRUISE_40_ECO','CRUISE_40_TRAIL','CRUISE_40_SPORT','CRUISE_40_SPORTPLUS',
             'CRUISE_60_ECO','CRUISE_60_TRAIL','CRUISE_60_SPORT','CRUISE_60_SPORTPLUS',
             'CRUISE_80_SPORT','CRUISE_100_SPORT']
files={}
for sc in scenarios:
    for exe,tag in ((torque,'torque'),(assist,'assist'),(ride,'ride')):
        path=OUT/f'{sc}_{tag}.csv'; run([str(exe),sc,str(path)]); files[(sc,tag)]=path
for sc in assist_only:
    path=OUT/f'{sc}_assist.csv'; run([str(assist),sc,str(path)]); files[(sc,'assist')]=path
burst_csv=OUT/'missed_tick_burst_summary.csv'; run([str(burst),str(burst_csv)])

# Exact deterministic rerun of a representative pipeline scenario.
repeat=OUT/'RUN_100_assist_repeat.csv'; run([str(assist),'RUN_100',str(repeat)])
base=(OUT/'RUN_100_assist.csv').read_bytes(); rep=repeat.read_bytes()
if base != rep:
    print('FAIL: whole-pipeline determinism differs on identical RUN_100 rerun', file=sys.stderr)
    raise SystemExit(1)

# The signals a ride-feel regression actually shows up in: the rider's pulsating input, the
# two halves of the demand model that absorb that pulsation, and the resulting request.
assist_cols=['torque_raw','torque_corrected','torque_fast','load_centikg','rider_demand',
             'assist_base','assist_dynamic','assist_response','iq_request','iq_final']
ride_cols=['torque_fast','rider_demand','assist_base','iq_request','iq_final']
summary=[]
for sc in scenarios+assist_only:
    for tag,cols in ([('assist',assist_cols),('ride',ride_cols)] if sc in scenarios
                     else [('assist',assist_cols)]):
        rows=read_csv(files[(sc,tag)])
        for c in cols:
            m=metric(rows,c)
            if m: summary.append(dict(Scenario=sc,Layer=tag,Column=c,**m))
with open(OUT/'metrics_summary.csv','w',newline='') as f:
    fields=['Scenario','Layer','Column','Mean','Min','Max','P5','P95','PeakToPeak','StdDev','Ripple']
    w=csv.DictWriter(f,fieldnames=fields); w.writeheader(); w.writerows(summary)

report=(
    'Portable whole-pipeline regression\n'
    '==================================\n'
    f'Scenarios: {len(scenarios)} x 3 layers = {len(scenarios)*3} traces\n'
    'Harnesses: torque_trace, assist_pipeline, ride_control_pipeline, missed_tick_burst\n'
    'Determinism RUN_100 assist rerun: PASS (byte-identical CSV)\n'
    'Build flags: -Wall -Wextra -Werror; documented type-limits exceptions only\n'
    'Result: PASS\n')
(OUT/'REPORT.txt').write_text(report)
print(report,end='')
print(OUT/'REPORT.txt')
