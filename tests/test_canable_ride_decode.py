#!/usr/bin/env python3
from __future__ import annotations
import csv, json, subprocess, sys, tempfile
from pathlib import Path
R=Path(__file__).resolve().parents[1]
BASE=0x10400
# A real capture logs the driver can_id verbatim: extended frames keep CAN_EFF_FLAG.
EFF=0x80000000

def u16(v): return [(v>>8)&255,v&255]
def i16(v): return u16(v&0xffff)
def line(us,ident,data):
    return f"[10:00:00]\t[INFO]\t{us}\tID:{ident:08X}\tDLC:8\tData:"+" ".join(f"{x:02X}" for x in data)+"\n"
def frame(tick,idx,payload): return u16(tick&0xffff)+payload

def main():
    with tempfile.TemporaryDirectory() as td:
        d=Path(td); raw=d/'ride.log'; prefix=d/'ride'
        tick=0x12345; us=1000000
        rows=[]
        rows.append(line(us,0x83106302,[0]*8)); us+=100 # ordinary CAN noise must be ignored
        # An ERR/RTR frame whose masked ID would land inside the block must never be decoded.
        rows.append(line(us,0x20010400,[0]*8)); us+=100
        payloads=[
          u16(987)+u16(321)+u16(654),
          i16(501)+i16(440)+i16(333),
          i16(-222)+i16(17)+u16(1444),
          u16(4123)+i16(1234)+u16(678),
          u16(2345)+u16(1701)+u16(0xA55A),
          [0x9B,0x42,(5|(2<<4)|(3<<6)),83,79,0],
          i16(-12345)+u16(234)+[(5|(1<<3)|(6<<4)|(1<<7)),(3|(2<<2)|(1<<4)|(1<<5)|(1<<6))],
        ]
        for idx,p in enumerate(payloads): rows.append(line(us,EFF|(BASE+idx),frame(tick,idx,p))); us+=3000
        meta=[1,1,(tick>>24)&255,(tick>>16)&255,(tick>>8)&255,tick&255,0,7]
        rows.append(line(us,EFF|(BASE+7),meta)); us+=3000
        # second snapshot: deliberately omit MOTOR. It must remain replayable, time must advance,
        # and metadata must report exactly one missing fragment instead of hiding the loss.
        tick2=tick+84
        for idx,p in enumerate(payloads):
            if idx==2: continue
            rows.append(line(us,BASE+idx,frame(tick2,idx,p))); us+=3000
        raw.write_text(''.join(rows))
        subprocess.run([sys.executable,str(R/'tools/decode_canable_ride_log.py'),str(raw),'--output-prefix',str(prefix)],cwd=R,check=True)
        meta=json.loads((d/'ride.metadata.json').read_text())
        assert meta['snapshots']==2 and meta['complete_snapshots']==1 and meta['replayable_snapshots']==2, meta
        assert meta['missing_frame_counts']['2']==1, meta
        assert meta['coverage']==['CORE','INTERNAL','LIMITS','MOTOR','ROTOR_PAS_STATE'], meta['coverage']
        with (d/'ride.decoded.csv').open(newline='') as f: dec=list(csv.DictReader(f))
        assert dec[0]['load_centikg']=='987' and dec[0]['iq_ref']=='333'
        assert dec[0]['battery_voltage_v']=='41.230000' and dec[0]['battery_current_a']=='12.340000'
        assert dec[0]['cadence_control_rpm']=='79' and dec[0]['qzero_state']=='3'
        assert dec[0]['hall_state']=='5' and dec[0]['pas_ab_snapshot']=='3'
        assert dec[1]['motor_erps']=='nan'
        with (d/'ride.canonical.csv').open(newline='') as f: can=list(csv.DictReader(f))
        assert len(can)==2
        assert can[0]['cadence_rpm']=='79' and can[0]['torque_ckg']=='987'
        assert can[0]['pas_ab']=='nan' and can[0]['pas_direction']=='-1'
        assert abs(float(can[1]['time_s'])-84/4000.0)<1e-9
        # Canonical output must be accepted by the native production-code replay harness.
        subprocess.run([sys.executable,str(R/'tools/run_replay.py'),str(d/'ride.canonical.csv'), '--output',str(d/'replayed.csv')],cwd=R,check=True)
        print('PASS schema 1 (hand-built): decode -> canonical -> native replay')
    schema2()
    return 0


def schema2():
    """The same journey for schema 2, from a capture the FIRMWARE produced.

    The hand-built fixture above proves the decoder agrees with this test's author. That is not
    the question a shipped decoder reading a shipped capture needs answered, so this one is
    emitted by the production serializer itself (tests/host/ride_telemetry_fixture.c links
    src/ride_telemetry.c). If the wire layout and the decoder drift apart, the fixture moves
    with the firmware and this fails.
    """
    import os, shutil
    cc = os.environ.get('CC', 'gcc')
    if shutil.which(cc) is None:
        print('SKIP schema 2 fixture: no host compiler')
        return
    with tempfile.TemporaryDirectory() as td:
        d = Path(td)
        exe = d / ('fixture.exe' if os.name == 'nt' else 'fixture')
        subprocess.run([cc, '-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-DCAN_DIAGNOSTICS_ENABLE=1', '-DCAN_RIDE_TELEMETRY_ENABLE=1',
                        '-I', str(R / 'inc'), '-I', str(R / 'tests/host/common/host_stubs'),
                        '-o', str(exe),
                        str(R / 'tests/host/ride_telemetry_fixture.c'),
                        str(R / 'src/ride_telemetry.c'), '-lm'], cwd=R, check=True)
        cap = subprocess.run([str(exe)], cwd=R, check=True, capture_output=True, text=True).stdout
        raw = d / 'v2.log'
        raw.write_text(cap)
        lines = [ln for ln in cap.splitlines(keepends=True) if ln.strip()]
        prefix = d / 'v2'
        subprocess.run([sys.executable, str(R / 'tools/decode_canable_ride_log.py'),
                        str(raw), '--output-prefix', str(prefix)], cwd=R, check=True)

        meta = json.loads((d / 'v2.metadata.json').read_text())
        assert meta['schema_versions'] == [2], meta['schema_versions']
        assert meta['unsupported_schema_versions'] == [], meta
        assert meta['meta_records'] and meta['meta_records'][0]['data_frame_count'] == 9, meta
        assert meta['snapshots'] == 2 and meta['complete_snapshots'] == 2, meta
        assert meta['missing_frame_counts'] == {str(i): 0 for i in (0,1,2,3,4,5,6,8,9)}, meta

        with (d / 'v2.decoded.csv').open(newline='') as f:
            dec = list(csv.DictReader(f))
        r = dec[0]
        assert r['schema_version'] == '2', r['schema_version']

        # Every schema-2 observation, at its own value and in its own unit. The numbers are the
        # ones tests/host/ride_telemetry_fixture.c fed to the production serializer.
        assert r['load_centikg'] == '987', r                       # 0.01 kgf
        assert r['torque_normalized_permille'] == '321', r         # permille
        assert r['rider_demand_permille'] == '654', r
        assert r['assist_base_permille'] == '410', r
        assert r['assist_dynamic_permille'] == '190', r
        assert r['assist_response_permille'] == '572', r
        assert r['rider_aggression_permille'] == '640', r
        assert r['load_state_permille'] == '275', r
        assert r['auto_factor_permille'] == '830', r

        # ...and the schema-1 names must NOT be populated from schema-2 bytes: that reuse is
        # exactly the misreading the version byte exists to prevent.
        assert r['torque_fast_native'] == 'nan', r
        assert r['torque_run_native'] == 'nan', r

        # The limiter flags travel in the STATE frame's schema-1 spare byte.
        assert r['lim_battery'] == '1' and r['lim_speed'] == '1' and r['lim_start'] == '1', r
        assert r['lim_power'] == '0' and r['lim_thermal'] == '0', r

        # The fields schema 1 already carried must still decode identically.
        assert r['iq_requested'] == '501' and r['iq_allowed'] == '440' and r['iq_ref'] == '333', r
        assert r['cadence_control_rpm'] == '79' and r['hall_state'] == '5', r

        # The second snapshot is a different row at a later tick, not a repeat.
        assert dec[1]['load_centikg'] == '1234', dec[1]
        assert dec[1]['rider_demand_permille'] == '777', dec[1]
        assert dec[1]['assist_base_permille'] == '555', dec[1]
        assert dec[1]['lim_battery'] == '0', dec[1]
        assert abs(float(dec[1]['time_s']) - float(dec[0]['time_s']) - 84 / 4000.0) < 1e-9, dec

        with (d / 'v2.canonical.csv').open(newline='') as f:
            can = list(csv.DictReader(f))
        assert len(can) == 2 and can[0]['torque_ckg'] == '987', can[0]

        # FRAME LOSS must be reported, not hidden. Drop one ASSIST frame (0x10408) and the
        # snapshot must stop being complete while everything else still decodes.
        lossy = d / 'v2_lossy.log'
        dropped = 0
        kept = []
        for ln in lines:
            if 'ID:80010408' in ln and dropped == 0:
                dropped += 1
                continue
            kept.append(ln)
        assert dropped == 1, 'the fixture must contain an ASSIST frame to drop'
        lossy.write_text(''.join(kept))
        subprocess.run([sys.executable, str(R / 'tools/decode_canable_ride_log.py'),
                        str(lossy), '--output-prefix', str(d / 'v2_lossy')], cwd=R, check=True)
        lmeta = json.loads((d / 'v2_lossy.metadata.json').read_text())
        assert lmeta['missing_frame_counts']['8'] == 1, lmeta['missing_frame_counts']
        assert lmeta['complete_snapshots'] == 1, lmeta
        with (d / 'v2_lossy.decoded.csv').open(newline='') as f:
            lrows = list(csv.DictReader(f))
        assert lrows[0]['assist_base_permille'] == 'nan', lrows[0]
        assert lrows[0]['load_centikg'] == '987', lrows[0]
        subprocess.run([sys.executable, str(R / 'tools/run_replay.py'),
                        str(d / 'v2.canonical.csv'), '--output', str(d / 'v2.replayed.csv')],
                       cwd=R, check=True)
        print('PASS schema 2 (production serializer): decode -> canonical -> native replay')
if __name__=='__main__': raise SystemExit(main())
