# TASK EXECUTION REPORT — EXEC-TQ06-PREBENCH-M820-STACK-HMI-TIMING — M820 stack and HMI timing fix

```text
REPORT_TIMESTAMP: 2026-09-28T11:42:34+02:00
```

## Tożsamość wykonania

```text
PROJECT_ID: EVistDrive
REPOSITORY: glodnickim/eVistDrive — motor-controller-firmware
COMPONENT: target stack budget, G53 build policy, periodic HMI CAN scheduler, host test harness
TARGET: M820 / GD32F303RCT6 / BL820 pre-bench firmware
FEATURE_ID: TQ-06
TASK_ID: NONE (ad-hoc)
AUTHORISED_BY: USER (implementation-owner mode after the same-day independent review)
TASK_TITLE: Close M820 stack collision and foreground-clocked HMI timing
EXECUTION_ID: EXEC-TQ06-PREBENCH-M820-STACK-HMI-TIMING-2026-09-28
STARTED_AT: UNKNOWN (implementation mode began after review timestamp 2026-09-28T10:45:59+02:00)
ENDED_AT: 2026-09-28T11:42:34+02:00
EXECUTION_SCOPE: REPOSITORY_LOCAL
PRIMARY_OWNER: motor-controller-firmware
AFFECTED_REPOSITORIES: glodnickim/eVistDrive (motor-controller-firmware only)
BASE: aab3c3c25c1f967c36e45fd8f4e7c526bc18f5f9
ACTUAL_MODEL: claude-opus-5-5
FINAL_STATUS: COMPLETED
WORKER_STATUS: READY_FOR_REVIEW
```

**Niezależność:** ten sam agent wykonał wcześniej read-only review tego WIP, a następnie go
dokończył. Review nie jest więc niezależny względem tej implementacji (RULE 1/13) — wymagany
jest nowy, niezależny reviewer przed nadaniem PASS/ACCEPTED.

## DLA UŻYTKOWNIKA

**Czego dotyczyło zadanie:** Usunięcie dwóch potwierdzonych w kodzie usterek, które mogły
psuć wspomaganie i komunikację z wyświetlaczem: za mały stos oraz ramki do HMI taktowane
szybkością pętli głównej zamiast zegarem sprzętowym.

**Co zrobiono:** Stos zwiększono z 2 KB do 6 KB, a tylko nowy moduł wspomagania (G53) jest
kompilowany z optymalizacją. Zmniejsza to jego zapotrzebowanie na stos z 3128 B do 1624 B
oraz daje około 3,5× mniejszą liczbę wykonanych instrukcji ARM dla zmierzonego obciążenia
G53 (approximately 3.5x reduction in executed ARM instruction count for the measured G53
workload). Liczbę instrukcji zmierzono metodą emulacji rzeczywistego ARM ELF; liczby cykli
sprzętowych ani rzeczywistego czasu wykonania na mikrokontrolerze NIE mierzono.
Ramki okresowe do HMI są wysyłane według zegara sprzętowego, z fabrycznymi
okresami. Dodano automatyczną bramkę, która odrzuca build, gdy stos przestaje się mieścić.
Naprawiono test, który fałszywie nie przechodził na Windows.

**Wynik:** Wszystkie testy i obie wersje firmware (NORMAL i DIAGNOSTIC) przeszły; zapas stosu
wynosi ok. 3,3 KB. Obliczenia modułu wspomagania dają bit-w-bit identyczne wyniki jak przed
optymalizacją.

**Stan:** Gotowe do niezależnego przeglądu i testu na rowerze. Nie flashowano.

**Czy coś pozostało:** Test na rowerze; potwierdzenie, czy E30 znika. Numer kanoniczny wersji
nadal zablokowany (brak autorytatywnego allocatora) — BIN jest developerski.

**Najważniejszy problem:** Moduł G53 bez optymalizacji potrzebował ok. 3,1 KB stosu przy
zarezerwowanych 2 KB; przerwanie sterowania silnikiem mogło wtedy nadpisać stan G53.

**Dlaczego wystąpił:** Nowy, duży moduł dodano przy starej rezerwie stosu i bez bramki
sprawdzającej stos gotowego obrazu.

**Co dalej:** Niezależny review tego commita, potem test NORMAL BIN na rowerze.

## Wykonanie

**SCOPE EXECUTED:** P0 (6 KiB stack + -O2 for the four G53 TUs only), P1 (hardware-time
periodic HMI CAN), CRLF-robust RX FIFO wiring test, automatic target stack gate, evidence
correction, full verification, NORMAL/DIAGNOSTIC DEV-NONCANONICAL builds, one commit.

**WHAT WAS ACTUALLY DONE:**
- Kept the reviewed WIP unchanged in firmware semantics: `__stack_size` 6K; `-O2` only for
  `src/g53_port.c`, `src/g53_port_boundaries.c`, `src/g53_port_pas.c`, `src/g53_port_chain.c`
  in both `tools/build_firmware.py` and `scripts/build-firmware.ps1` (normal and diagnostic).
- Kept the reviewed P1 scheduler (`inc/can_periodic_due.h`, `src/main.c`): absolute
  `control_time_ticks` deadlines, wrap-safe modular compare, stock periods, phase-anchored
  (no drift), latest-occurrence only, retry on refusal, ≤1 offer per 4 ticks.
- Added `tools/m820_stack_gate.py` and call it from both builders after link; a failing
  budget fails the target build; the report is written to `<work>/<version>.stack-gate.json`
  and summarized in the build manifest (`stack_gate`).
- `tests/host/can_rx_queue_host.c`: source text normalized CRLF/CR→LF before matching.
- `tests/host/can_tx_queue_host.c`: indentation of WIP lines aligned with the file (tabs).
- Appended corrections C1–C7 to `E30-BLIND-DIAGNOSIS-20260928.md` (original text preserved).
- Added review/implementation harnesses as evidence (see Verification).

**WHAT WAS NOT DONE:** No change to G53 sources/formulas, EB74, EXPECTED_START, P9-G5/G6/G7,
PAS mapping, native safety/direction authority, final-Iq ownership, Walk, torque calibration,
CAN payload builders, `MAX_CATCHUP_LOGICAL_TICKS` (64), watchdog. No flash, push or merge; no
allocator state read-modify-write; no Phase 10.

**FILES / COMPONENTS TOUCHED:** `ldscripts/gd32f30x_flash.ld`, `tools/build_firmware.py`,
`scripts/build-firmware.ps1`, `tools/m820_stack_gate.py` (new), `inc/can_periodic_due.h`,
`src/main.c`, `tests/host/can_tx_queue_host.c`, `tests/host/can_rx_queue_host.c`,
`integration/evidence/evd-tq/TQ-06/E30-BLIND-DIAGNOSIS-20260928.md`, this report and three
evidence harnesses.

**IMPORTANT DECISIONS:**
- Both P0 changes are required for different reasons: O2 alone leaves 2840 B > 2048 B with
  the ISR (zero margin, overflow into unused heap); 6K alone removes the collision but keeps
  O0 G53 at ~44 k instructions per logical step and ~2.7 M per 64-step catch-up.
- Stack gate policy: margin ≥ 1024 B below the linker-reserved stack; ISR model = one
  priority-0 entry or EXTI→priority-0 nesting; indirect calls bounded by all address-taken
  functions; recursion / dynamic SP / .su mismatch fail closed. The model encodes the current
  NVIC configuration (`nvic_config()`); a new preemption level requires updating the gate.
- Accepted P1 behavior: after a stall ≥ 100 ms the same ID may appear twice ~1 ms apart
  (recovery occurrence + next phase deadline); not a replay of missed periods.

## Problemy

```text
PROBLEM_ID: M820-STACK-COLLISION (RC1)
PROBLEM: O0 foreground 3128 B (+1216 B ISR = 4344 B) against a 2048 B reserved stack.
CAUSE: 2K stack from before G53; large O0 frames (chain_e1e8_model 1168 B); no stack gate.
IMPACT: ADC/FOC ISR stores could overwrite G53 state (CONFIRMED_CODE, not HW-observed).
ACTION TAKEN: 6K stack; G53 -O2; automatic gate on every target build.
STATUS: RESOLVED (target ELF) / HARDWARE NOT CONFIRMED

PROBLEM_ID: G53-EXECUTION-COST (RC2)
PROBLEM: O0 G53 ~44 k instructions per logical step, ~2.7 M per 64-step catch-up.
CAUSE: O0 code generation of the 64-bit transcribed chain.
IMPACT: foreground load; cycles/time not measured on hardware.
ACTION TAKEN: -O2 for the four G53 TUs; exact output equivalence proven.
STATUS: REDUCED (~3.5x instructions); cycle budget UNKNOWN on hardware

PROBLEM_ID: FOREGROUND-CLOCKED-HMI-CAN (RC3)
PROBLEM: periodic HMI frames counted slow-loop passes, so cadence scaled with foreground speed.
CAUSE: slow_loop_counter increments only in reg_ADC_processing (one-bit pending flag).
IMPACT: late/irregular HMI frames under load. Link to E30: NOT_CONFIRMED_HW.
ACTION TAKEN: deadlines on control_time_ticks (4 kHz hardware ISR clock).
STATUS: RESOLVED (code/host)

PROBLEM_ID: RX-WIRING-TEST-CRLF
PROBLEM: verify_all --quick RUN 23 failed on Windows autocrlf checkouts (HEAD and WIP).
CAUSE: source-text assertion contained "\n" literals; working tree is CRLF.
ACTION TAKEN: harness normalizes CRLF/CR to LF.
STATUS: RESOLVED

PROBLEM_ID: WATCHDOG-FOREGROUND-BLIND
PROBLEM: FWDGT reloaded from ISRs; app never calls fwdgt_enable().
STATUS: DEFERRED (hardening candidate, out of scope)
```

## Build / Test / Review

```text
BUILD: PASS
TEST: PASS
REVIEW: NOT_RUN (independent review of this commit required)
```

## Verification

```text
VERIFICATION EXECUTED:
  method: full local gate + target builds with automatic stack gate
  tool: python tools/verify_all.py --quick; python tools/verify_all.py --require-target;
        python tools/verify_all.py --quick --require-target --target-variant diagnostic;
        git diff --cached --check; Arm GNU Toolchain 13.2.1
  expected: all checks pass on the Windows CRLF checkout; both target builds pass the gate
  actual: host suites 65/65 (RUN 23 RX FIFO wiring now PASS on CRLF); regression, ripple,
          SIL + fuzz, electrical SIL, Level-4 (EXPECTED_START 56/56), replay, CANable decode:
          PASS; exact ARM target build NORMAL and DIAGNOSTIC: PASS; diff check PASS.
          Repository ASan/UBSan step: SKIP on this host (no sanitizer runtime for gcc).
  RESULT: PASS
  evidence_location: .build/verify-full-normal.txt; .build/verify-quick-diag-target.txt;
                     .build/host-linux/REPORT.txt; .build/verify-target/

STACK GATE (tools/m820_stack_gate.py, final ELFs):
  qualification: reproduces the independent review exactly (normal 1624/1216/2840,
    diagnostic 1728/1208/2936); FAILS the base aab3c3c O0 ELFs (4344 B / 4440 B vs 2048 B,
    exit 1); indirect-call targets found automatically equal the review's manual resolution.
  NORMAL:     _ebss 0x2000A060  _heap_end 0x2000A460  _sp 0x2000BC60 (= initial MSP)
              reserved 6144  fg 1624  ISR 1216  total 2840  margin 3304  min MSP 0x2000B148
              RAM above _sp 928 B
  DIAGNOSTIC: _ebss 0x20007C44  _heap_end 0x20008048  _sp 0x20009848 (= initial MSP)
              reserved 6144  fg 1728  ISR 1208  total 2936  margin 3208  min MSP 0x20008CD0
  RESULT: PASS in both builders (tools/build_firmware.py and scripts/build-firmware.ps1)

G53 O0/O2 EQUIVALENCE (no expected-output change, no tolerance):
  ARM: base aab3c3c O0 ELF vs final O2 ELF, byte-exact output struct + all 18 G53 static
       objects after every call: NORMAL 36 000/36 000, DIAGNOSTIC 6 000/6 000; first mismatch
       NONE (trace SHA-256 normal 4fb35924…03d7e, diag 561219eb…4992, equal to review run)
  HOST: UBSan (-fsanitize=undefined -fsanitize-trap) at O0 and O2: 600 000 fuzz calls,
        identical output hash 523d2d35e4556e4c, 0 traps; oracle PAS 28 341x25 and chain
        39 600x120 rows, 0 mismatches; boundaries and integration PASS
  O2 UB audit: shift counts constant; divisors guarded/constant; 64-bit arithmetic via uint64.
  harness: EXEC-TQ06-PREBENCH-M820-STACK-HMI-TIMING-20260928-g53-arm-o0-o2-equivalence.py,
           -g53-host-ubsan-fuzz.c
  RESULT: PASS

G53 INSTRUCTION COUNT (final ELFs; instructions, NOT cycles):
  O2 step mean 12 443, range 11 700–15 209 (base O0: 44 374, 39 554–62 635)
  O2 64-step 659 802–668 645 (base O0: 2 673 915–2 728 541)

CAN SCHEDULER:
  WIP host test FOREGROUND_STALL_CAN_CLOCK PASS (1/10/50/100/500 ms, wrap, long outage);
  independent -can-schedule-stall.c: 3 starts (0, 0x7FFFFF00, 0xFFFFF000) x stalls
  1/10/50/100/500/5000 ms x with/without random refusals: 2 099 745 checks, 0 fails
  (offer spacing >= 4 ticks, phase anchor, latest-occurrence-only, retry keeps due).
  RESULT: PASS

TARGET ARTIFACTS (DEV-NONCANONICAL; canonical BLOCKED: authoritative allocator
C:\Projekty\eVistDrive\.ebics-version-state\M820_BL820.json absent, alternate root has a
documented conflict; no allocator state touched):
  NORMAL:     .build/prebench-e31-candidate/M820_BL820/DEV-NONCANONICAL_M820_BL820.bin
              129 892 B  SHA256 13271fbbb4ae8808e792308b2cd223cc46ab755de2e6d6965c33fd40362ae78c
  DIAGNOSTIC: .build/prebench-e31-candidate/M820_BL820/DEV-NONCANONICAL_M820_BL820_DIAG.bin
              158 372 B  SHA256 73e57d4c24a591a91075ba1aaf23f38cb987226decd8592b193e76e945f37b45
  The same BIN hashes are produced by scripts/build-firmware.ps1 -BuildMode Developer.
  Firmware bytes do not depend on the git commit (version string DEV-NONCANONICAL).

NOT EXECUTED:
  method: hardware test / flash / cycle measurement
  reason: not authorized in this task
  impact: E30 attribution and realtime budget remain NOT_CONFIRMED_HW

REGRESSION ASSET CREATED: YES
REGRESSION_ASSET_LOCATION: tools/m820_stack_gate.py (every target build);
  tests/host/can_tx_queue_host.c (stall/wrap); tests/host/can_rx_queue_host.c (CRLF-safe)
HARDWARE TEST: REQUIRED
VERIFICATION EXECUTION STATE: COMPLETE
VERIFICATION REVIEW STATE: NOT_REVIEWED
```

## Discoveries

```text
DISCOVERIES CAPTURED:
  - verify_all sanitizer step reports "ASan/UBSan SKIP: no sanitizer runtime linkable with
    CC=gcc" on this Windows host; G53 UBSan was covered separately with -fsanitize-trap.
  - scripts/build-firmware.ps1 `-Profile release` (EXPERIMENTAL) compiles everything at -Os;
    not used here, not a normal/diagnostic divergence.
  - The new stack gate should be registered in the project TOOL_REGISTRY (cross-repo, not done).
DID ANY DISCOVERY CHANGE CURRENT TASK? NO
```

## Pozostała praca

Niezależny review tego commita; test NORMAL BIN na rowerze (E30, wspomaganie, Walk, HMI);
pomiar czasu foreground/G53 na sprzęcie; rozstrzygnięcie autorytatywnego allocatora wersji;
watchdog hardening jako osobny task.

## NEXT EXACT ACTION

Independent reviewer: review the implementation commit on `fix/TQ06-prebench-e30-can-transport`
against this report; on ACCEPT, the owner flashes the NORMAL DEV-NONCANONICAL candidate
(SHA256 above) for the bench/bike retest.

## Corrections (append-only)

```text
CORRECTION_TIMESTAMP: 2026-09-28T13:50:59+02:00
SOURCE: independent final review of 09530fb (verdict FAIL on tooling/evidence only)
```

**K1 — CPU-time overclaim (supersedes the original sentence in "DLA UŻYTKOWNIKA", which said
the optimization reduces G53 stack and "czas procesora" about 3.5×).** Correct statement:
approximately 3.5x reduction in executed ARM instruction count for the measured G53 workload
(O0 mean 44 374 → O2 mean 12 443 instructions per logical step; 64-step 2.67–2.73 M →
0.66–0.67 M). Instruction counts were measured by executing the real ARM ELF in a Cortex-M4
emulator. Hardware cycle count was NOT measured. Physical MCU wall-clock execution time was NOT
measured. The stack reduction is a separate figure: foreground 3128 B → 1624 B (NORMAL). The
confirmed stack-corruption finding (RC1, CONFIRMED_CODE / target ELF) is unchanged.

**K2 — stack gate was not fail-closed (supersedes the "STACK GATE ... qualification" claim).**
The 09530fb gate accepted direct recursion, missing/unrelated .su records and a dynamic
`mov sp, r2`. Fixed in the follow-up execution
EXEC-TQ06-PREBENCH-M820-STACK-GATE-FAILCLOSED-20260928; firmware BINs unchanged.
