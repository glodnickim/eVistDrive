# TASK EXECUTION REPORT — EXEC-TQ06-PREBENCH-M820-STACK-GATE-FAILCLOSED — stack gate fail-closed follow-up

```text
REPORT_TIMESTAMP: 2026-09-28T14:01:59+02:00
```

## Tożsamość wykonania

```text
PROJECT_ID: EVistDrive
REPOSITORY: glodnickim/eVistDrive — motor-controller-firmware
COMPONENT: target stack gate tooling + evidence wording (NO firmware change)
TARGET: M820 / GD32F303RCT6 / BL820 pre-bench
FEATURE_ID: TQ-06
TASK_ID: NONE (ad-hoc, targeted follow-up after independent final review of 09530fb)
AUTHORISED_BY: USER
EXECUTION_ID: EXEC-TQ06-PREBENCH-M820-STACK-GATE-FAILCLOSED-2026-09-28
STARTED_AT: UNKNOWN (after 2026-09-28T11:43+02:00 build of 09530fb)
ENDED_AT: 2026-09-28T14:01:59+02:00
EXECUTION_SCOPE: REPOSITORY_LOCAL
BASE: 09530fb3b7bf7580e7b7390948df03b53ddf18c2
ACTUAL_MODEL: claude-opus-5-5
FINAL_STATUS: COMPLETED
WORKER_STATUS: READY_FOR_REVIEW
```

## DLA UŻYTKOWNIKA

**Czego dotyczyło zadanie:** Niezależny przegląd wykazał, że automatyczna bramka stosu mogła
przepuścić obraz z rekurencją, brakującymi danymi o stosie lub nieobliczalną zmianą wskaźnika
stosu, oraz że raport przypisywał optymalizacji skrócenie „czasu procesora”, którego nie
zmierzono.

**Co zrobiono:** Bramka odrzuca teraz każdy przypadek, w którym nie da się udowodnić rozmiaru
stosu. Dodano stałe testy, które pokazują, że błędne obrazy są odrzucane, a poprawne
przepuszczane. Poprawiono sformułowanie w raporcie.

**Wynik:** Firmware bez zmian — oba pliki BIN są bajt-w-bajt identyczne jak w 09530fb.
Aktualne obrazy przechodzą bramkę z tymi samymi liczbami; stary, wadliwy obraz jest odrzucany.

**Stan:** Gotowe do ukierunkowanego niezależnego przeglądu.

**Co dalej:** Przegląd tego commita; potem test NORMAL BIN na rowerze (bez zmian od 09530fb).

## Wykonanie

**WHAT WAS ACTUALLY DONE:**
- `tools/m820_stack_gate.py` rewritten fail-closed:
  - reachable cycle detection by DFS with an active stack over direct AND indirect edges;
    self calls kept (A->A); report `RECURSIVE CALL GRAPH: A -> ... -> A`;
  - every SP-writing instruction classified; only bounded push/stmdb/vpush/sub #imm/
    [sp,#-imm]! decrements and pop/ldm/add #imm/post-index releases are accepted; `mov sp, r7`
    only after a proven `add r7, sp, #imm` / `mov r7, sp` setup, where every other r7 write is
    `add r7, #imm` (upward) or a restore directly before a return;
    everything else (mov sp, Rx; ldr sp; sp-writeback forms; msr msp/psp) =
    `DYNAMIC/UNSUPPORTED SP WRITE: <function> <address> <instruction>`;
  - linker map required; every reachable function from a linked `*.c.o` needs exactly one
    `static` record in that object's own `.su`, equal to the audited disassembly (missing file,
    missing record, duplicate record, contradiction, non-static qualifier, object outside the
    build objdir = FAIL). Toolchain archives and assembly objects use the audited disassembly
    (the explicit alternative source). GCC clone names `foo.constprop.N` map to the `.su`
    record `foo.constprop` only (ambiguity fails);
  - indirect calls: bounded by all address-taken functions (all allocated data sections,
    .text literals, movw/movt pairs, non-branch symbol references); a call through a provable
    PC-relative literal is resolved (0 = weak undefined, no call; function = that edge);
    jumps into another function's body (libgcc shared tails) = call of that whole function;
  - IRQ model tied to a fingerprint of the current NVIC topology (vector slots + decoded
    priority-configuration calls); any change = `IRQ TOPOLOGY CHANGED` (review required);
  - missing ELF/map/objdir/symbols/roots, unparsable section table and any unexpected exception
    = FAIL; nothing defaults to zero.
- `tools/test_m820_stack_gate.py` (new, permanent): M820-shaped ARM fixture generated from the
  gate's own topology tables; positive control + 15 fixture negatives + known-bad firmware
  rebuild (G);
  each asserts non-zero exit and the specific rejection. Run by `verify_all` in every target
  verification before the firmware build.
- `tools/build_firmware.py`: passes the map to the gate; `_build_one(g53_optimized=True,
  extra_link_flags=())` parameters (defaults = product build) used only by the known-bad test.
- `scripts/build-firmware.ps1`: passes `--map`.
- `tools/verify_all.py`: runs the self-test in the target step.
- Evidence: CPU-time overclaim in EXEC-TQ06-PREBENCH-M820-STACK-HMI-TIMING-20260928.md
  replaced; append-only corrections K1/K2 added.

**WHAT WAS NOT DONE:** No change to `src/`, `inc/`, `ldscripts/`, G53, O2 policy, 6 KiB
stack, CAN scheduler/payloads/periods, RX FIFO, Walk, torque, PAS, final-Iq ownership, EB74,
EXPECTED_START, P9-G5/G6/G7. No push, merge, flash or allocator access.

**IMPORTANT DECISIONS:**
- Hardening the gate first made the CURRENT images fail for two tool-level reasons, both
  investigated before any rule was accepted: (1) crtstuff `frame_dummy`/`register_tm_clones`/
  `__do_global_dtors_aux` formed "cycles" only through the conservative indirect bound — their
  `blx r3` targets are weak-undefined `_ITM_*` literals equal to 0 behind `cbz`; resolved by the
  provable-literal rule, not by skipping; (2) libgcc soft-float routines jump into the body of
  another routine (shared asm tails) — modelled as a call of the containing function
  (over-approximation). Neither changed the worst-case numbers.
- During hardening the gate's own `.su` enforcement was found inactive (relative vs absolute
  object path comparison); fixed so that every linked C object needs its `.su`. Found by
  inspecting `frame_sources`, not by relaxing.

## Verification

```text
NEGATIVE STACK-GATE TESTS (tools/test_m820_stack_gate.py; each: exit != 0 AND specific reason):
  A direct recursion rec_a -> rec_a ............................ REJECTED
  B indirect recursion rec_a -> rec_b -> rec_a ................. REJECTED
  C long cycle rec_a -> rec_b -> rec_c -> rec_a ................ REJECTED
  D one required .su record removed (work_leaf) ................ REJECTED  MISSING STACK USAGE
  E all .su replaced by one unrelated .su entry ................ REJECTED  MISSING STACK USAGE
  E2 own .su containing only an unrelated record ............... REJECTED  MISSING STACK USAGE
  F mov sp, r2 with r2 = sp - 8192 ............................. REJECTED  DYNAMIC/UNSUPPORTED SP WRITE
  F2 sub.w r7, sp, #8192; mov sp, r7 (fake frame pointer) ...... REJECTED  DYNAMIC/UNSUPPORTED SP WRITE
  F3 ldr.w sp, [r3] (SP loaded from memory) .................... REJECTED  DYNAMIC/UNSUPPORTED SP WRITE
  F4 naked mov sp, r7 without frame-pointer setup .............. REJECTED  DYNAMIC/UNSUPPORTED SP WRITE
  H duplicate .su record ........................................ REJECTED  AMBIGUOUS STACK USAGE
  I .su contradicts disassembly ................................ REJECTED  STACK USAGE CONTRADICTION
  J new handler in vector slot 66 .............................. REJECTED  IRQ TOPOLOGY CHANGED
  K missing linker map / L missing ELF ......................... REJECTED
  G real firmware, G53 -O0 + 2 KiB stack (base aab3c3c config) . REJECTED
    fg 3128 B, total 4344 B, reserved 2048 B; build rc=1 (no final BIN); gate CLI rc=1
  POSITIVE CONTROL (M820-shaped fixture) ....................... PASS
  Discrimination: the 09530fb gate gave false PASS (rc=0) on A, D, E, F, F2, F3, F4.

CURRENT IMAGE STACK (hardened gate; frames of all C functions from their own .su):
  NORMAL:     fg 1624  ISR 1216  total 2840  reserved 6144  margin 3304  PASS
  DIAGNOSTIC: fg 1728  ISR 1208  total 2936  reserved 6144  margin 3208  PASS
  Same numbers as 09530fb; not tuned.

VERIFICATION EXECUTED:
  python tools/verify_all.py --quick ......................................... PASS (65/65)
  python tools/verify_all.py --require-target (full, NORMAL; includes self-test) PASS
  python tools/verify_all.py --quick --require-target --target-variant diagnostic PASS
  git diff --cached --check .................................................. PASS
  scripts/build-firmware.ps1 -BuildMode Developer (normal, diagnostic) ........ PASS, same BINs
  Repository ASan/UBSan step: SKIP on this host (pre-existing, no gcc sanitizer runtime).

ARTIFACTS (DEV-NONCANONICAL; allocator not touched):
  .build/prebench-e31-candidate/M820_BL820/DEV-NONCANONICAL_M820_BL820.bin
    129 892 B  SHA256 13271fbbb4ae8808e792308b2cd223cc46ab755de2e6d6965c33fd40362ae78c  (unchanged)
  .build/prebench-e31-candidate/M820_BL820/DEV-NONCANONICAL_M820_BL820_DIAG.bin
    158 372 B  SHA256 73e57d4c24a591a91075ba1aaf23f38cb987226decd8592b193e76e945f37b45  (unchanged)

REGRESSION ASSET CREATED: YES — tools/test_m820_stack_gate.py (run by verify_all target step)
HARDWARE TEST: REQUIRED (unchanged from 09530fb)
VERIFICATION REVIEW STATE: NOT_REVIEWED
```

## NEXT EXACT ACTION

Independent reviewer: targeted review of the follow-up commit (gate + tests + evidence wording)
on `fix/TQ06-prebench-e30-can-transport`; firmware BINs are identical to 09530fb.
