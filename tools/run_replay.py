#!/usr/bin/env python3
from __future__ import annotations
import argparse, os, subprocess
from pathlib import Path
R=Path(__file__).resolve().parents[1]; OUT=R/'.build/replay'; OUT.mkdir(parents=True,exist_ok=True)
CC=os.environ.get('CC','gcc')
# Audit finding 9: link to, and launch, the platform's real executable name.
EXE_SUFFIX='.exe' if os.name=='nt' else ''
PROD=['src/torque_input.c','src/rider_input.c','src/assist_modes.c','src/cadence_filter.c',
      'src/tuning_config.c','src/ride_control.c',
      'src/ap2_pas_state.c','src/ap2_torque_chain.c','src/ap2_estimators.c',
      'src/ap2_profiles.c','src/ap2_limits.c','src/assist_pipeline.c',
      'src/fast_iq_slew.c','src/battery_iq_cap.c','src/iq_chain.c',
      'src/motor_core.c','src/walk_assist_motor.c','src/walk_speed_controller.c',
      'tests/host/common/map_adapter.c','tests/host/common/motor_service_stub.c']
def main():
    ap=argparse.ArgumentParser(); ap.add_argument('input',type=Path); ap.add_argument('--output',type=Path); ap.add_argument('--tolerance',type=float,default=-1)
    # AP-0a: motor voltage utilisation (FOC _U_MAX domain, 2048 = full scale). Omitted -> the
    # harness keeps its historical 0, which exercises ONLY the launch-anchor branch of
    # finish_power_request(); see the long comment in sim/replay/replay_fw.c. Not a default to
    # change casually: sim/replay/cases/*/manifest.json pin accepted_output_sha256.
    ap.add_argument('--u-abs',type=int,default=None,help='0..2048; sweep it, do not guess one value')
    a=ap.parse_args(); exe=OUT/('replay_fw'+EXE_SUFFIX); out=a.output or OUT/(a.input.stem+'.replayed.csv')
    cmd=[CC,'-std=c11','-O2','-Wall','-Wextra','-Werror','-Wno-error=type-limits','-Wno-error=unused-parameter',
         '-Isim/full_host_stubs','-Iinc','-Itests/host/common','-o',str(exe),'sim/replay/replay_fw.c',*PROD,'-lm']
    subprocess.run(cmd,cwd=R,check=True)
    args=[str(exe),str(a.input),str(out)]
    if a.tolerance>=0: args.append(str(a.tolerance))
    env=dict(os.environ)
    if a.u_abs is not None: env['REPLAY_U_ABS']=str(a.u_abs)
    subprocess.run(args,cwd=R,check=True,env=env)
if __name__=='__main__': main()
