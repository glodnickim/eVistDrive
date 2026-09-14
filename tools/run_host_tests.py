#!/usr/bin/env python3
import os,re,sys,subprocess,tempfile,shlex
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
PS=ROOT/'tests/host/run-host-tests.ps1'
PSS=ROOT/'tests/host'
INC=ROOT/'inc'
text=PS.read_text(errors='replace')
start=text.index('$suites = @(')
end=text.index('\n)\n\nfunction Find-HostCompiler', start)
body=text[start:end]
# split top-level suite hash blocks using lines that begin 4 spaces + @{ Name
starts=[m.start() for m in re.finditer(r'(?m)^\s*@\{ Name = ', body)]
blocks=[]
for i,s in enumerate(starts):
    e=starts[i+1] if i+1<len(starts) else len(body)
    blocks.append(body[s:e])

varmap={
 'mainCPathForward': ROOT/'src/main.c',
 'mainHPathForward': ROOT/'inc/main.h',
 'canDisplayCPathForward': ROOT/'src/CAN_Display.c',
 'currentCalCPathForward': ROOT/'src/current_cal.c',
 'rideControlCPathForward': ROOT/'src/ride_control.c',
 'assistPipelineCPathForward': ROOT/'src/assist_pipeline.c',
 'ap2LimitsCPathForward': ROOT/'src/ap2_limits.c',
 'ap2PasStateCPathForward': ROOT/'src/ap2_pas_state.c',
 'ap2RiderDemandCPathForward': ROOT/'src/ap2_rider_demand.c',
 'motorCoreCPathForward': ROOT/'src/motor_core.c',
 'focCPathForward': ROOT/'src/FOC.c',
 'focCurrentLoopCPathForward': ROOT/'src/foc_current_loop.c',
 'sampleWindowCPathForward': ROOT/'src/sample_window.c',
 'armMathHPathForward': ROOT/'Firmware/CMSIS/arm_math.h',
 'configHPathForward': ROOT/'inc/config.h',
 'batteryCurrentCPathForward': ROOT/'src/battery_current.c',
 'walkAssistMotorCPathForward': ROOT/'src/walk_assist_motor.c',
}

def join_expr(scope,path):
    base=ROOT if scope=='root' else PSS
    return base/Path(path.replace('\\','/'))

def extract_joinpaths(section):
    out=[]
    for m in re.finditer(r"Join-Path \$(root|PSScriptRoot) '([^']+)'", section):
        out.append(join_expr(m.group(1),m.group(2)))
    return out

def get_section(block,key):
    m=re.search(r'(?m)^\s*'+re.escape(key)+r'\s*=\s*',block)
    if not m:return ''
    pos=m.end(); lines=[]
    # field ends at next assignment-like line or block end; arrays can span lines
    rest=block[pos:]
    if rest.startswith('@('):
        depth=0; in_s=False; in_d=False
        for i,ch in enumerate(rest):
            prev=rest[i-1] if i else ''
            if ch=="'" and not in_d: in_s=not in_s
            elif ch=='"' and not in_s and prev!='\\': in_d=not in_d
            if not in_s and not in_d:
                if ch=='(': depth+=1
                elif ch==')':
                    depth-=1
                    if depth==0:return rest[:i+1]
        return rest
    return rest.splitlines()[0]

def parse_strings(section):
    vals=[]
    # single or double quoted strings; handle variables in double quotes
    for m in re.finditer(r"'([^']*)'|\"([^\"]*)\"",section):
        s=m.group(1) if m.group(1) is not None else m.group(2)
        for v,p in varmap.items(): s=s.replace('$'+v, str(p).replace('\\','/'))
        vals.append(s)
    return vals

suites=[]
for b in blocks:
    nm=re.search(r"Name\s*=\s*'([^']+)'",b)
    hm=re.search(r"Harness\s*=\s*Join-Path \$(root|PSScriptRoot) '([^']+)'",b)
    if not nm or not hm: continue
    name=nm.group(1)
    harness=join_expr(hm.group(1),hm.group(2))
    modules=extract_joinpaths(get_section(b,'Modules'))
    incdirs=extract_joinpaths(get_section(b,'IncludeDirs'))
    defines=parse_strings(get_section(b,'Defines'))
    args=parse_strings(get_section(b,'Arguments'))
    expect='ExpectBuildFailure = $true' in b
    transport='TransportDecoder = $true' in b
    suites.append(dict(name=name,harness=harness,modules=modules,incdirs=incdirs,defines=defines,args=args,expect=expect,transport=transport))

print(f'Parsed {len(suites)} suites')
# The guard is that the parser found EVERY suite the registry declares, not that the
# registry is at least some remembered size. A magic minimum has to be edited by hand
# whenever a suite is legitimately retired, and at that moment it stops catching the
# thing it exists for - a parser that silently skipped a block it could not read.
declared=len(re.findall(r'(?m)^\s*@\{ Name = ', text))
if len(suites)!=declared:
    print(f'ERROR: parser found {len(suites)} suites but the registry declares {declared}',
          file=sys.stderr);sys.exit(3)

cc=os.environ.get('CC','gcc')
# Audit finding 9: link to, and launch, the platform's real executable name.
EXE_SUFFIX='.exe' if os.name=='nt' else ''
failed=[]; skipped=[]; passed=[]
outdir=ROOT/'.build/host-linux';outdir.mkdir(parents=True,exist_ok=True)
for idx,s in enumerate(suites,1):
    exe=outdir/f'suite_{idx:02d}{EXE_SUFFIX}'
    cmd=[cc,'-std=c11','-Wall','-Wextra','-Werror']+s['defines']
    for d in s['incdirs']:cmd+=['-I',str(d)]
    cmd+=['-I',str(INC),'-o',str(exe),str(s['harness'])]+[str(x) for x in s['modules']]+['-lm']
    cp=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    if s['expect']:
        if cp.returncode!=0:
            passed.append(s['name']);print(f'PASS build-refusal {idx:02d}: {s["name"]}')
        else:
            failed.append((s['name'],'unexpectedly built',cp.stdout));print(f'FAIL {idx:02d}: {s["name"]}')
        try: exe.unlink()
        except: pass
        continue
    if cp.returncode!=0:
        failed.append((s['name'],'build',cp.stdout));print(f'FAIL BUILD {idx:02d}: {s["name"]}')
        continue
    env=os.environ.copy()
    if s['transport']:
        # Harness itself is still executed; PS-only strict decoder is reported separately.
        env['RNA_CAPTURE_FILE']=str(outdir/f'transport_{idx:02d}.log')
    rp=subprocess.run([str(exe)]+s['args'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,env=env)
    if rp.returncode!=0:
        failed.append((s['name'],'run',rp.stdout));print(f'FAIL RUN {idx:02d}: {s["name"]}')
    else:
        passed.append(s['name']);print(f'PASS {idx:02d}: {s["name"]}')
        if s['transport']:
            skipped.append((s['name'],'PowerShell transport decoder post-step not run by Linux runner'))
    try: exe.unlink()
    except: pass

report=outdir/'REPORT.txt'
with report.open('w') as f:
    f.write(f'Parsed: {len(suites)}\nPassed: {len(passed)}\nFailed: {len(failed)}\nPost-step skipped: {len(skipped)}\n\n')
    if failed:
        f.write('FAILURES\n========\n')
        for n,phase,o in failed:
            f.write(f'\n[{phase}] {n}\n{o}\n')
    if skipped:
        f.write('\nSKIPPED POST-STEPS\n==================\n')
        for n,w in skipped:f.write(f'{n}: {w}\n')
print(f'\nSummary: PASS={len(passed)} FAIL={len(failed)} post-step-skip={len(skipped)}')
print(report)
sys.exit(1 if failed else 0)
