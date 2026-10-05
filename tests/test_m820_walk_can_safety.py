#!/usr/bin/env python3
"""M820 Walk / CAN / position-calibration safety, run from the CURRENT production text.

src/main.c and src/CAN_Display.c cannot be linked on the host (GD32 registers throughout), so
this test extracts the exact blocks that decide Walk, the comms watchdog, the 0x6300 level
decoder and the position-calibration service owner - by signature and anchor, never by line
number - and compiles them with tests/host/m820_walk_can_safety_harness.c and the production
ride_control / G53 pipeline / ap2_limits / fast_iq_slew / PAS modules.

A missing or ambiguous anchor is a FAILURE: if the production code moves, this test must be
re-pointed deliberately, not silently skip what it used to prove.
"""
from __future__ import annotations

import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

R = Path(__file__).resolve().parents[1]
CC = os.environ.get('CC', 'gcc')
EXE_SUFFIX = '.exe' if os.name == 'nt' else ''

MODULES = ['torque_input', 'rider_input', 'assist_modes', 'tuning_config', 'ride_control',
           'g53_port', 'g53_port_boundaries', 'g53_port_pas', 'g53_port_chain', 'g53_g1_limiter', 'battery_trip', 'ap2_limits',
           'assist_pipeline', 'fast_iq_slew', 'battery_iq_cap', 'iq_chain', 'motor_core',
           'walk_assist_motor', 'walk_speed_controller', 'pas_sampler', 'pas_quadrature',
           'pas_direction', 'pas_liveness', 'pa4_buttons']


class AnchorError(RuntimeError):
    pass


def read(path: Path) -> str:
    return path.read_text(encoding='utf-8', errors='strict').replace('\r\n', '\n')


def match_brace(text: str, open_at: int) -> int:
    """Index just past the '}' matching text[open_at]=='{', skipping comments and literals."""
    depth, i, n = 0, open_at, len(text)
    while i < n:
        c = text[i]
        if text.startswith('//', i):
            i = text.index('\n', i)
            continue
        if text.startswith('/*', i):
            i = text.index('*/', i) + 2
            continue
        if c in '"\'':
            j = i + 1
            while text[j] != c:
                j += 2 if text[j] == '\\' else 1
            i = j + 1
            continue
        if c == '{':
            depth += 1
        elif c == '}':
            depth -= 1
            if depth == 0:
                return i + 1
        i += 1
    raise AnchorError('unbalanced braces')


def strip_comments(text: str) -> str:
    """Code only - a comment explaining a rule must not be able to satisfy or trip a check."""
    text = re.sub(r'/\*.*?\*/', ' ', text, flags=re.S)
    return re.sub(r'//[^\n]*', '', text)


def definition(text: str, signature: str) -> str:
    """The one DEFINITION of `signature` (a prototype ending in ';' is skipped)."""
    hits = []
    for m in re.finditer(re.escape(signature), text):
        rest = text[m.end():].lstrip()
        if rest.startswith('{'):
            hits.append(m.start())
    if len(hits) != 1:
        raise AnchorError(f'{signature!r}: expected exactly one definition, found {len(hits)}')
    start = hits[0]
    open_at = text.index('{', start + len(signature))
    return text[start:match_brace(text, open_at)] + '\n'


def unique(text: str, needle: str) -> int:
    count = text.count(needle)
    if count != 1:
        raise AnchorError(f'{needle!r}: expected exactly once, found {count}')
    return text.index(needle)


def line_span(text: str, first_anchor: str, last_anchor: str) -> str:
    a = text.rfind('\n', 0, unique(text, first_anchor)) + 1
    b = text.index('\n', unique(text, last_anchor)) + 1
    if b <= a:
        raise AnchorError(f'{first_anchor!r} .. {last_anchor!r}: out of order')
    return text[a:b]


def extract(outdir: Path) -> None:
    main_c = read(R / 'src/main.c')
    can_c = read(R / 'src/CAN_Display.c')
    blocks = {
        'x_defines.inc': line_span(main_c, '#define AUTODETECT_STANDSTILL_MS', '#define AUTODETECT_STANDSTILL_TICKS'),
        'x_standstill.inc': definition(main_c, 'bool hall_calibration_standstill_confirmed(void)'),
        'x_watchdog.inc': definition(main_c, 'static void can_rx_consume_liveness_events(void)') +
                          definition(main_c, 'static void comm_watchdog_step(void)'),
        'x_hall_request.inc': main_c[main_c.rfind('\n', 0, unique(main_c, 'static volatile uint8_t hall_calibration_pending')) + 1:
                                     main_c.index(definition(main_c, 'static void hall_calibration_service(void)').rstrip('\n'))] +
                              definition(main_c, 'static void hall_calibration_service(void)'),
        'x_autodetect.inc': definition(main_c, 'void autodetect(void)'),
        'x_walk_iq.inc': definition(main_c, 'uint16_t walk_assist_iq_request(void)'),
        'x_hall_iq.inc': definition(main_c, 'uint16_t hall_calibration_iq_request(void)'),
        'x_walk_block.inc': line_span(main_c, '//--- Walk Assist physical button (PA4)', 'ui8_wa_level_prev=MS.assist_level;'),
        'x_phase2_abort.inc': line_span(main_c,
                                        'if(comm_inhibit && MS.hall_angle_detect_flag > 1U) hall_calibration_abort();',
                                        'if(comm_inhibit && MS.hall_angle_detect_flag > 1U) hall_calibration_abort();'),
    }
    # hall_calibration_bridge_off() .. hall_calibration_abort(): one contiguous region.
    exit_first = definition(main_c, 'static void hall_calibration_bridge_off(void)')
    exit_last = definition(main_c, 'static void hall_calibration_abort(void)')
    a = main_c.index(exit_first)
    b = main_c.index(exit_last) + len(exit_last)
    blocks['x_hall_exit.inc'] = main_c[a:b]
    # CAN_Display.c: the 0x6300 decoder, brace-matched.
    at = unique(can_c, 'if(Ext_ID_Rx.command==0x6300){')
    blocks['x_can6300.inc'] = can_c[at:match_brace(can_c, can_c.index('{', at))] + '\n'
    # The facts this harness depends on must be present in the extracted text itself.
    required = {
        'x_watchdog.inc': ['comm_inhibit = ((hmi_seen && hmi_lost_ticks >= COMM_CUT_TICKS)'],
        'x_walk_block.inc': ['if(comm_inhibit){', 'MS.walk_can_request=RESET;', 'walk_can_counter=0;',
                             'ui8_wa_latch_active=0;', '!ui8_wa_comm_block'],
        'x_autodetect.inc': ['if(comm_inhibit) return;', 'hall_calibration_snapshot();',
                             'comm_watchdog_step();', 'hall_calibration_abort();'],
        'x_hall_iq.inc': ['hall_calibration_bridge_off();', 'write_virtual_eeprom();'],
        'x_hall_exit.inc': ['hall_calibration_restore();', 'MS.hall_angle_detect_flag=1;'],
        'x_hall_request.inc': ['if(comm_inhibit) return false;'],
    }
    for name, needles in required.items():
        for needle in needles:
            if needle not in blocks[name]:
                raise AnchorError(f'{name}: required production fact missing: {needle!r}')
    if 'write_virtual_eeprom' in strip_comments(blocks['x_hall_exit.inc']):
        raise AnchorError('x_hall_exit.inc: the abort path must not write the EEPROM')
    for name, body in blocks.items():
        (outdir / name).write_text(body, encoding='utf-8')
    print(f'extracted {len(blocks)} production blocks from src/main.c and src/CAN_Display.c')


def main() -> int:
    with tempfile.TemporaryDirectory() as td:
        out = Path(td)
        try:
            extract(out)
        except AnchorError as e:
            print(f'FAIL anchor: {e}', file=sys.stderr)
            return 2
        exe = out / ('m820_walk_can_safety' + EXE_SUFFIX)
        cmd = [CC, '-std=c11', '-Wall', '-Wextra', '-Werror', '-Wno-type-limits',
               '-Wno-unused-function', '-Wno-unused-variable', '-Wno-unused-but-set-variable',
               '-I', str(out), '-I', str(R / 'sim/full_host_stubs'),
               '-I', str(R / 'tests/host/common'), '-I', str(R / 'inc'),
               '-o', str(exe), str(R / 'tests/host/m820_walk_can_safety_harness.c')]
        cmd += [str(R / 'src' / f'{m}.c') for m in MODULES]
        cmd += [str(R / 'tests/host/common/map_adapter.c'), '-lm']
        build = subprocess.run(cmd, cwd=R, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        if build.returncode:
            print(build.stdout)
            print('FAIL build: M820 Walk/CAN/calibration harness', file=sys.stderr)
            return 1
        run = subprocess.run([str(exe)], cwd=R, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        print(run.stdout, end='')
        return run.returncode


if __name__ == '__main__':
    raise SystemExit(main())
