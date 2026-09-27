# TASK EXECUTION REPORT — EXEC-TQ06-PREBENCH-E30-CAN-TRANSPORT — CAN transport repair

```text
REPORT_TIMESTAMP: 2026-09-27T16:53:46+02:00
```

## Tożsamość wykonania

```text
PROJECT_ID: EVistDrive
REPOSITORY: glodnickim/eVistDrive — motor-controller-firmware
COMPONENT: CAN RX/TX transport and HMI liveness
TARGET: M820 / DP-C245 pre-bench firmware
FEATURE_ID: TQ-06
TASK_ID: NONE (ad-hoc)
AUTHORISED_BY: USER
TASK_TITLE: Preserve HMI CAN traffic under foreground load
EXECUTION_ID: EXEC-TQ06-PREBENCH-E30-CAN-TRANSPORT-2026-09-27
STARTED_AT: UNKNOWN
ENDED_AT: 2026-09-27T16:53:46+02:00
EXECUTION_SCOPE: REPOSITORY_LOCAL
PRIMARY_OWNER: motor-controller-firmware
AFFECTED_REPOSITORIES: glodnickim/eVistDrive
FINAL_STATUS: BLOCKED
```

## DLA UŻYTKOWNIKA

**Czego dotyczyło zadanie:** Naprawa utraty ramek HMI i okresowych ramek CAN, która mogła odcinać komunikację z wyświetlaczem.

**Co zrobiono:** Dodano 16-slotowy FIFO RX; nadano watchdogom liveness z fizycznego odbioru CAN; okresowe ramki zachowują stan „due” aż do przyjęcia przez TX queue. Dodano testy burstu HMI, opóźnionego parsera i nasycenia TX queue.

**Wynik:** Host/regresja i oba DEV-NONCANONICAL target buildy PASS. Wymagany canonical pair jest zablokowany przez brak autorytatywnego allocator state oraz wykryty konflikt rootów.

**Stan:** BLOCKED przed pełnym target acceptance. Naprawa kodu jest gotowa do niezależnego review.

**Czy coś pozostało:** Ustalenie jednego autorytatywnego HWM/root dla canonical pair; niezależny review oraz walidacja sprzętowa.

**Najważniejszy problem:** Oryginalny ISR nadpisywał pojedynczy mailbox; okresowe sendery mogły skasować stan due po odmowie enqueue.

**Dlaczego wystąpił:** Odbiór ISR i wolniejszy parser współdzieliły jedną strukturę zamiast kolejki; część okresowych wywołań ignorowała rezultat enqueue.

**Co dalej:** Project owner ma ustalić autorytatywny canonical allocator root/HWM; potem zbudować pair z tego repair HEAD. Nie inicjalizowałem ani nie zmieniałem żadnego allocator state.

## Wykonanie

**SCOPE EXECUTED:** CAN ISR→main RX transport, fizyczna liveness, retry okresowego TX, regresje host oraz static G53 disassembly audit.

**WHAT WAS ACTUALLY DONE:**
- Dodano statyczny SPSC FIFO o pojemności 16; przy przepełnieniu odrzuca najnowszą ramkę i zwiększa licznik overflow.
- RX ISR kopiuje ramkę i oznacza fizyczne BUS/HMI RX eventy; parser pozostał w foreground i obsługuje najwyżej jedną ramkę na przebieg pętli.
- Watchdog odbiera eventy fizycznego RX przed oceną timeoutów; próg ciszy pozostał bez zmian.
- Okresowe sendery resetują licznik due wyłącznie po zaakceptowanym enqueue.
- Poprawiono parser `tools/run_host_tests.py`, by mapował nową ścieżkę IRQ dla source-wiring suite.
- Wykonano host gate, quick regression gate, disassembly/object-size audit oraz oba warianty ARM build.
- Utworzono jeden repair commit na bazie `1c30343853165abe8b7df3ebd8588aa94c2715c8`, bez push/merge/flash.

**WHAT WAS NOT DONE:** Nie zmieniano Fake Taxi ID/payload/cadence, watchdog timeout, G53 source/semantics, native safety ani Phase 9 lifecycle; nie flashowano targetu.

**FILES / COMPONENTS TOUCHED:** `inc/can_rx_queue.h`, `src/can_rx_queue.c`, `inc/can_periodic_due.h`, `src/gd32f30x_it.c`, `src/main.c`, `src/CAN_Display.c`, `inc/CAN_Display.h`, `scripts/sources-m820.txt`, host tests and runners. G53 production files are unchanged.

**IMPORTANT DECISIONS:** Zachowano bounded foreground service; do not infer hardware behavior solely from host tests. O0 target optimization remained unchanged. G53 runtime risk is surfaced rather than repaired in this CAN task.

## Problemy

```text
PROBLEM_ID: E30-CAN-RX-OVERWRITE
PROBLEM: Pojedynczy globalny receive mailbox tracił wcześniejsze ramki burstu.
CAUSE: ISR nadpisywał wspólną strukturę przed obsługą przez parser.
IMPACT: Mogły zginąć HMI writes 0x6300..0x6304, a parser-based watchdog mógł uznać żywe HMI za nieobecne.
ACTION TAKEN: Wdrożono FIFO oraz fizyczne RX eventy.
STATUS: RESOLVED

PROBLEM_ID: E30-PERIODIC-TX-LOSS
PROBLEM: Odmowa enqueue mogła skasować okresową ramkę logiczną.
CAUSE: Niektóre schedulery resetowały due state mimo odmowy kolejki.
IMPACT: Heartbeat/status mógł zniknąć do następnego pełnego okresu lub bez retry.
ACTION TAKEN: Due state pozostaje nasycone do zaakceptowanego enqueue; retry nie blokuje.
STATUS: RESOLVED

PROBLEM_ID: G53-REALTIME-BUDGET-RISK
PROBLEM: Duży O0 G53 foreground hot path może zwiększać czas między obsługą ramek przez main.
CAUSE: Static audit znalazł `g53_chain_step` o rozmiarze 0xE2C B, wiele pośrednich wywołań oraz signed 64-bit divide helper’y; nie wykonano pomiaru cycle-accurate.
IMPACT: Budżet foreground/CAN servicing pozostaje niepotwierdzony na sprzęcie.
ACTION TAKEN: Zachowano kod G53 i poziom optymalizacji; przekazano ryzyko do niezależnej oceny.
STATUS: UNRESOLVED / REVIEW RISK
```

## Build / Test / Review

```text
BUILD: PASS
TEST: PASS
REVIEW: NOT_RUN
```

## Verification

```text
VERIFICATION EXECUTED:
  method: deterministic host/regression tests and clean target builds
  tool: python tools/verify_all.py --quick; tools/build_firmware.py; Arm GNU Toolchain 13.2.1; arm-none-eabi-nm/objdump/size
  input: committed source at base 1c30343853165abe8b7df3ebd8588aa94c2715c8 plus this repair
  expected: all existing checks pass; no source/semantic G53 changes; both target variants fit configured memory
  actual: 65/65 host; deterministic regression PASS; ripple analyzer PASS; SIL PASS, 1000/0; electrical SIL PASS; Level-4 fixed PASS and EXPECTED_START 12/12, fuzz 25/25; replay behavior 6/6; schema/decode PASS. Both DEV-NONCANONICAL target builds PASS from committed source. Canonical pair not run: authoritative allocator state is missing and the extant alternate root documents a duplicate-version conflict.
  RESULT: PARTIAL
  evidence_location: .build/host-linux/REPORT.txt; .build/regression-linux/REPORT.txt; .build/sil/REPORT.txt; .build/electrical-sil/REPORT.txt; .build/level4/REPORT.txt; .build/replay/; .build/repair-head/.build/final-validation-normal/; .build/repair-head/.build/final-validation-diagnostic/

G53 STATIC AUDIT:
  `g53_port_update`: 0x364 B; catch-up clamps to at most 64 logical updates per call.
  `g53_chain_step`: 0xE2C B; calls chain state, BDE8, E1E8 and D7EC processing helpers.
  `g53_ad7ec_step`: 0x168 B.
  `g53_pas_step`: 0x394 B.
  G53 objects: port 1,030 text B / 550 BSS; boundaries 958 text B / 30 BSS; chain 37,278 text B / 3,162 BSS; PAS 1,602 text B.
  64-bit signed division path uses `__aeabi_ldivmod` and `__udivmoddi4`; port hot path includes signed divide wrappers. No `__aeabi_*` change was introduced by this repair. Static inspection is not a timing measurement; classify as REALTIME_BUDGET_RISK.

NORMAL TARGET:
  CANONICAL: BLOCKED — authoritative allocator JSON is missing at `C:\Projekty\eVistDrive\.ebics-version-state\M820_BL820.json`; no canonical version allocated.
  DEV-NONCANONICAL CHECK: PASS; `.build/repair-head/.build/final-validation-normal/M820_BL820/DEV-NONCANONICAL_M820_BL820.bin`
  148,756 bytes; SHA256 `6e22c67e8847e25204d6f7f438addc66495652c9a2d75011bc9da28ed2f146ea`
  FLASH 148,756 / 230 KB; RAM 44,104 / 48 KB.
DIAGNOSTIC TARGET:
  CANONICAL: BLOCKED — same missing authoritative allocator root; no canonical version allocated.
  DEV-NONCANONICAL CHECK: PASS; `.build/repair-head/.build/final-validation-diagnostic/M820_BL820/DEV-NONCANONICAL_M820_BL820_DIAG.bin`
  177,240 bytes; SHA256 `ac5ee833cc80a577b849bbb94ff998188155a6ed84d443a5e141e72500dafeda`
  FLASH 177,240 / 230 KB; RAM 34,856 / 48 KB.

NOT EXECUTED:
  method: hardware test / flash
  reason: not part of the authorized action; no automatic flash
  impact: physical repair behavior remains to be confirmed on the target bike

REGRESSION ASSET CREATED: YES
REGRESSION_ASSET_LOCATION: tests/host/can_rx_queue_host.c; extended tests/host/can_tx_queue_host.c
HARDWARE TEST: REQUIRED
VERIFICATION EXECUTION STATE: COMPLETE
VERIFICATION REVIEW STATE: NOT_REVIEWED
```

## Discoveries

```text
DISCOVERIES CAPTURED: NONE
DID ANY DISCOVERY CHANGE CURRENT TASK? YES — static G53 audit identified a separately scoped timing risk; production G53 was left untouched.
```

## Pozostała praca

Niezależny przegląd naprawy, ocena `REALTIME_BUDGET_RISK` oraz osobna, kontrolowana walidacja sprzętowa.

## Jeżeli STOPPED lub BLOCKED

```text
WORK COMPLETED BEFORE STOP: CAN RX/TX repair, all host/regression gates, static timing audit, DEV-NONCANONICAL normal and diagnostic builds, one local repair commit.
STOP / BLOCK CONDITION: required canonical normal+diagnostic build pair cannot safely reserve versions.
WHY CONTINUING WAS NOT ALLOWED: repository allocator source resolves to C:\Projekty\eVistDrive\.ebics-version-state, whose M820_BL820.json is missing; the alternate C:\Projekty\.ebics-version-state exists but documents a prior two-root duplicate-version conflict and is not authority to reserve from.
PARTIAL / UNCOMMITTED STATE: tracked repair is committed; unrelated prior untracked evidence remains preserved.
SAFE STATE: no allocator state changed; no canonical release binaries, push, merge, or flash.
WHAT IS REQUIRED TO RESUME: project owner establishes the single authoritative allocator root/HWM or provides the approved canonical build path.
NEXT EXACT ACTION: confirm allocator authority, then run canonical normal+diagnostic pair from repair HEAD.
```

## NEXT EXACT ACTION

Project owner: reconcile the authoritative canonical version allocator; after that, rerun the canonical pair from the repair HEAD.
