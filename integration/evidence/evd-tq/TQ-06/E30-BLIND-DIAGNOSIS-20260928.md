REPORT_TIMESTAMP: 2026-09-28T07:07:04+02:00

# TQ-06 independent blind diagnosis: target stack and E30 hypotheses

## Execution identity

```text
PROJECT_ID: EVD
REPOSITORY: motor-controller-firmware
TARGET: M820 / GD32F303RCT6 / BL820
FEATURE_ID: FEAT-EVD-ASSIST-PIPELINE-001
TASK_ID: TASK-EVD-TQ-06
EXECUTION_ID: E30-BLIND-DIAGNOSIS-20260928
AUTHORISED_BY: PROJECT_OWNER / USER, delegated by parent agent
PRIMARY_OWNER: /root/blind_diagnosis
ACTUAL_MODEL: NOT_EXPOSED
STARTED_AT: UNKNOWN (initial diagnosis began in previous interrupted turn)
EXECUTION_SCOPE: CROSS_REPOSITORY read-only investigation
AFFECTED_REPOSITORIES: motor-controller-firmware; hmi-firmware (read only)
FINAL_STATUS: COMPLETED (bounded independent diagnosis)
WORKER_STATUS: READY_FOR_REVIEW
BUILD: NOT_RUN by this investigator; parent-produced artifacts inspected
TEST: PARTIAL (static target-artifact verification; no hardware execution)
REVIEW: NOT_RUN
HARDWARE TEST: REQUIRED to identify the reported bike's actual root cause
```

## Dla użytkownika

Znaleziono konkretny błąd pamięci w badanym oprogramowaniu silnika: zwykła ścieżka wspomagania potrzebuje więcej stosu niż zarezerwowano. Już bez przerwania dochodzi do zapisu w pamięci innych danych. Przerwanie sterowania silnikiem pogłębia kolizję. To jest błąd potwierdzony w kodzie maszynowym, ale bez pomiaru na rowerze nie nazywamy go jeszcze potwierdzoną przyczyną E30.

Badany kandydat z optymalizacją tylko nowego modułu i większym stosem usuwa tę kolizję w przeanalizowanych ścieżkach. Pozostają odrębne kwestie opóźnień komunikacji oraz test na sprzęcie. Nie zmieniano kodu produkcyjnego w tym wykonaniu.

## Scope and independence

The parent supplied symptoms (E30, no pedal assist, Walk briefly works), current HEAD and permission to inspect. Initial diagnosis was generated independently of the parent's findings. Subsequent messages requested a focused exact-ELF stack audit and supplied candidate artifact locations. Candidate inspection is therefore a follow-up check, not a second blind design.

Read-only source inspection covered native pedal vetoes, G53 boundaries/PAS/chain, Walk arbitration, CAN scheduling, native IRQ handlers, linker/startup, and HMI communication reverse notes. Older project snapshots still describe an unimplemented TQ-06 / V2 path; actual HEAD and source are authoritative for this investigation. Existing HMI WIP and parent build/linker WIP were not edited.

## Artifact identities

Base HEAD: `aab3c3c25c1f967c36e45fd8f4e7c526bc18f5f9`, branch `fix/TQ06-prebench-e30-can-transport`.

| Artifact | Identity |
|---|---|
| Exact clean base ELF | `.build/repair-head/.build/final-validation-normal/M820_BL820/work/normal/DEV-NONCANONICAL.elf` |
| Exact base ELF SHA-256 | `89f33046c59cc920e3a942c6df5cc31ffd9feba98dab7a2b6de75f7671e76972` |
| O0 audit ELF with `.su` | `.build/e30-o0/M820_BL820/work/normal/DEV-NONCANONICAL.elf` |
| O0 audit ELF SHA-256 | `5acd8f4e1720ae5f923b698a197ed499f5a1b4da43e19c57d5082c6102a85d4c` |
| Raw application BIN SHA-256, both base and O0 audit | `42484233f31bb258aa66ccbbc9a3d8dc72bc54f53fb2a31717520ebf54ed4353` |
| Candidate ELF | `.build/e30-opt-probe/M820_BL820/work/normal/DEV-NONCANONICAL.elf` |
| Candidate ELF SHA-256 | `d42556fec5ceb9ce6f0ef9530d2402fbde8e41fb0c660e5f1a92ac7599013aa9` |

Compiler is Arm GNU 13.2.1. Parent generated the O0 audit with `-fstack-usage`; raw application identity proves its executable payload matches the earlier exact base. Candidate manifest records `-O0` default and `-O2` only for `g53_port.c`, `g53_port_boundaries.c`, `g53_port_chain.c`, `g53_port_pas.c`. Candidate is an unaccepted experiment from a dirty worktree, not a release.

## Finding S1: actual stack reservation is insufficient

**[CONFIRMED_CODE / target ELF]** The initial MSP is not the physical RAM end. The vector table's first word is `0x2000AC48`; source linker lines 31–32 and 151–154 put `_sp` directly above the allocated heap and stack. RAM is 48 KiB (`0x20000000..0x2000C000`).

| Boundary | O0 base | O2 / 6 KiB candidate |
|---|---:|---:|
| `_ebss` | `0x2000A044` | `0x2000A048` |
| `_heap_end` | `0x2000A448` | `0x2000A448` |
| `_sp` | `0x2000AC48` | `0x2000BC48` |
| Declared stack | 2048 bytes | 6144 bytes |
| Distance `_sp - _ebss` | 3076 bytes | 7168 bytes |
| Physical RAM above initial MSP | 5048 bytes | 952 bytes |

RAM above MSP does not protect a descending stack. Heap is also not an implicit stack allowance: `_sbrk()` in `Firmware/CMSIS/GD/GD32F30x/Source/syscalls.c:25` allocates `_end.._heap_end`.

### Reachable O0 foreground call chain

These are **nested frames**, not a sum of all functions. GCC `.su` values were cross-checked against pushes/subtractions in target disassembly. All reported `.su` entries are `static`.

| Nested frame | Bytes | Source / evidence |
|---|---:|---|
| `main` | 80 | `src/main.c:1005`; O0 `src_main.c.su` |
| `reg_ADC_processing` | 280 | `src/main.c:2632` |
| `ride_control_update` | 152 | `src/ride_control.c:179` |
| `assist_pipeline_update` | 160 | `src/assist_pipeline.c:94` |
| `g53_port_update` | 712 | `src/g53_port.c:35` |
| `g53_chain_step` | 176 | `src/g53_port_chain.c:2229` |
| `chain_e1e8_model` | 1168 | `src/g53_port_chain.c:929` |
| `chain_lut_model` | 320 | `src/g53_port_chain.c:747` |
| `read_16` | 32 | `src/g53_port_chain.c:35` |
| `read_8` | 32 | `src/g53_port_chain.c:34` |
| `state_byte` | 16 | `src/g53_port_chain.c:22` |
| **Total** | **3128** | **1080 beyond declared stack; 52 beyond `_ebss`** |

Reachability is stronger than a conditional worst-case hypothesis: phase 3 calls E1E8 (`g53_port_chain.c:2262`); E1E8 has no early return before its LUT call (`:1243`); the LUT's initial `read_16` (`:764`) is unconditional. Parent caller frames are 672 bytes. Calling only `g53_port_update` in a harness omits those 672 bytes and is not a test of the complete application's stack.

Target call sites: `0x080267AA -> chain_e1e8_model`, `0x08022B8A -> chain_lut_model`, `0x08020CDE -> read_16`. The smallest frame actually writes, rather than only reserving unused space:

```text
0801e1a0 <state_byte>:
  push {r7}          ; 4 bytes
  sub sp, #12        ; total frame 16
  add r7, sp, #0
  str r0, [r7, #4]   ; with full caller chain: address 0x2000A014
```

At the 3128-byte depth, MSP is `0x2000A010`. `__sf` occupies `0x20009EFC..0x2000A034`, so this path writes into a live `.bss` object even without an IRQ. This establishes a real target memory-layout defect; it does not establish which displayed symptom a particular overwritten object produces.

### Reachable ADC/FOC interrupt chain

Independent direct-call inspection of the ADC subtree finds this largest software chain:

```text
ADC0_1_IRQHandler             112
  FOC_calculation            624
    runPIcontrol             104
      pi_iq_apply_inputs       8
        fast_iq_slew_tick    136
          fis_mailbox_read_verified 64
TOTAL                       1048 bytes
```

Sources: `main.c:4058,4230,3771,3751`; `FOC.c:96`; `fast_iq_slew.c:160,101`. The `FOC_calculation` call requires valid retained/current feedback, so this is a reachable active/armed control path, not an assertion that FOC executes before calibration. No IRQ is required for S1's foreground-only collision.

Library leaves in these audited subgraphs were inspected in disassembly: `arm_sin_cos_q31` 80 B, `sqrtf` 16 B, `__ieee754_sqrtf` 0 B, `__errno` 0 B, `memset` 0 B, `memcpy` 8 B, `__aeabi_ldivmod`/`__aeabi_uldivmod` 16 B, `__udivmoddi4` 32 B. None produces a larger ADC path than the mailbox chain above. There are no unresolved indirect calls or recursive edges in these two audited subgraphs.

### IRQ priorities and exception frames

`main.c:2264..2282` sets `NVIC_PRIGROUP_PRE1_SUB3`, then TIMER1, TIMER2, ADC and TIMER4 to priority `(0,0)`. CAN is also encoded zero. SysTick is explicitly zero (`systick.c:55`). These priority-zero exceptions **do not preempt one another**; `__enable_irq()` inside ADC does not change that priority rule. We do not sum CAN, Hall, timer, FOC and SysTick as if simultaneously nested.

EXTI2 and EXTI10_15 are assigned before the final priority grouping. Their raw encoding depends on the inherited AIRCR group (`gd32f30x_misc.c:62..91`). With reset/default fallback to PRE2_SUB2 they encode `0x80`, which becomes lower preemption priority under PRE1_SUB3. Other inherited groupings can encode a priority-zero result. Conservative audit therefore permits **one** lower-priority EXTI handler (24 B software including its leaf) to be preempted by one priority-zero ADC handler. No two EXTI handlers can mutually preempt on the final single preemption bit.

Architectural hardware stacking is 32 B for the basic exception frame, or 104 B including floating-point context; allow 0/4 B alignment padding per entry. Lazy FP save still requires reserving its frame. We do not claim to have measured FPCCR/CONTROL on this bike. For a safe static bound, allowing 108 B at each of two possible exception entries intentionally overestimates the EXTI-without-FP case.

| O0 path | Bytes |
|---|---:|
| Foreground alone | 3128 |
| Foreground + ADC + basic frame, no alignment | 4208 |
| Foreground + ADC + maximum FP/alignment allowance | 4284 |
| Conservative foreground + lower EXTI + ADC + two maximum exception allowances | **4416** |

The conservative minimum MSP is `0x20009B08`, 1340 bytes below `_ebss`. Potentially overlapped BSS ranges include `state_5` (`0x200099B0..0x20009CB0`), `state_6`, `state_7`, `port_trace` (`0x20009CD4..0x20009EBC`), PAS context (`0x20009EBC..0x20009EF4`), `control_remainder`, `normal_permission`, `dropped_ticks`, libc `__sf`, `errno`, and allocator globals. This is a mapped possible footprint, not a claim every byte was written on hardware.

## Candidate stack check and recommendation

Candidate `.su` plus actual call targets give the following largest port subtree:

```text
g53_port_update               696
  g53_chain_step               80
    chain_d7ec_model          128
      __aeabi_ldivmod          16
        __udivmoddi4           32
TOTAL                         952 bytes
unchanged caller frames       672
FOREGROUND TOTAL             1624 bytes
```

ADC software remains 1048 B. The same deliberately conservative two-exception allowance gives `1624 + 24 + 1048 + 108 + 108 = 2912 B`. Within the candidate's declared 6144-byte stack this leaves **3232 B**; minimum MSP under that bound is `0x2000B0E8`, above `_heap_end=0x2000A448`.

Recommendation: retain explicit 6 KiB stack provision and scoped optimization, verify each final ELF with its own `.su` and map, and preserve the O0 artifact as regression evidence. The 6 KiB choice fits the current normal target by 952 B at physical RAM end. A 4 KiB allocation is too small for the original O0 control-path bound. Optimization alone reduces demand but does not justify retaining the original 2 KiB allocation with IRQs.

This is a bound for the audited normal control path and configured IRQ subtrees, not a complete whole-firmware stack certification. Diagnostic builds, boot/service/string-formatting paths, NMI/fault recovery, future changes and final release images need their corresponding check. Fault handlers that deliberately stop forever are not ordinary nested ride execution.

## Other diagnosis findings and discriminating experiments

1. **[CONFIRMED_CODE] Separate owners matter.** `ride_control.c:229..244` selects Walk independently of the G53 pedal chain. Native pedal vetoes include torque validity, PAS validity, direction and real stop (`assist_pipeline.c:102..103,143..151`). A G53-only permission/scale fault alone cannot explain E30 plus short-lived Walk; corruption or a shared timing/communication fault can plausibly affect all three.
2. **[HYPOTHESIS] Target foreground load.** O0 G53 uses extensive 64-bit arithmetic and byte-address state helpers (`g53_port_chain.c:22..50`). `g53_port.c:37..69` catches up up to 64 logical steps before foreground CAN service (`main.c:1320..1334`). Latest-state G53 PAS sampling can lose transitions when foreground is late even while the native queue-based PAS decoder sees activity. Target timing or hardware-cycle measurements are needed; host/SIL correctness does not bound execution time.
3. **[EXTERNAL_REFERENCE, local reverse report] HMI evidence limits the earlier CAN explanation.** `hmi-firmware/docs/29_MISSING_CAN_INFO_REVERSE.md:44..107` supersedes older per-frame timeout hypotheses: its exact CF80301.2 analysis reports no timeout dedicated to absent 3201/3202 frames and identifies source-2-seen as a sticky latch. This investigation did not independently repeat that complete reverse. Therefore periodic enqueue loss alone must not be promoted to confirmed E30 root cause. Actual E30 generation/handshake, full-bus loss and exact installed HMI identity remain to be checked.

Useful discriminating experiment: emulate the exact target control chain with initial MSP from its vector and real parent-frame reservation, record minimum MSP and writes below `_heap_end` / `_ebss`, then compare the scoped candidate; separately capture HMI/CAN and foreground timing on hardware. A bare port harness without 672 B of caller frames proves a different stack scope. Do not weaken torque, direction, brake or communication safety vetoes to make the motor run.

## Verification and remaining work

Executed: pre-audit, exact ELF/map/vector/disassembly reads, GCC `.su` cross-check, raw application hash equality, reachable nested-call calculations, IRQ priority/dataflow inspection, candidate control-path bound, source-level symptom comparison. Tools: PowerShell, `rg`, Arm GNU 13.2.1 `objdump`/`nm`, small read-only in-memory Python call-graph inspection. Production files changed by this investigator: **NONE**. Only this report is written.

Not executed here: build, broad regression, hardware test, independent re-reverse of HMI E30, or final release acceptance. Those are not reported PASS. Parent owns integration/history indexing; this child was authorized to write only its report. No new regression executable was created by this child; preserve parent-produced `.su`/ELF/manifest artifacts and add the final qualified check through the normal tooling owner.

**Remaining work:** parent validates candidate behavior/timing and final binary identity, reviewer accepts evidence, owner tests on hardware. Root cause of the reported bike remains **UNKNOWN / NOT CONFIRMED_HW**, with S1 a confirmed defect deserving correction independently of that final attribution.

**NEXT EXACT ACTION:** parent reproduces/cross-checks the full application stack footprint against this 3128-byte O0 chain and 2912-byte conservative candidate interrupt bound, then submits the complete candidate and evidence to independent review.

## Corrections (append-only)

```text
CORRECTION_TIMESTAMP: 2026-09-28T11:36:41+02:00
SOURCE: independent read-only review of this report (2026-09-28, same day) and the
        implementation execution EXEC-TQ06-PREBENCH-M820-STACK-HMI-TIMING-20260928.
RULE: original text above is preserved; each item below SUPERSEDES the cited statement.
```

**C1 — stale candidate identity (supersedes "Artifact identities" candidate row and the
"O2 / 6 KiB candidate" column).** The candidate ELF `d42556fe…` was an intermediate
experiment. The final candidate adds the hardware-time CAN schedule state (+0x18 B BSS).
Final DEV-NONCANONICAL candidate, built by `tools/build_firmware.py` with the automatic
stack gate (`tools/m820_stack_gate.py`):

| Final ELF | `_ebss` | `_heap_end` | `_sp` = initial MSP | reserved stack | RAM above `_sp` |
|---|---:|---:|---:|---:|---:|
| NORMAL | `0x2000A060` | `0x2000A460` | `0x2000BC60` | 6144 B | 928 B |
| DIAGNOSTIC | `0x20007C44` | `0x20008048` | `0x20009848` | 6144 B | 10168 B |

The statement "Physical RAM above initial MSP: 952 bytes" is superseded by 928 B (NORMAL).

**C2 — candidate stack bound (supersedes "2912 B" / "3232 B" conservative figures).** The
reviewed realistic nesting model is one priority-0 entry, or one lower-priority EXTI handler
preempted by one priority-0 handler (FP-extended 108 B first frame, basic 36 B nested frame
because the EXTI handlers execute no FP code). Final ELFs:

| Final ELF | foreground | ISR addition | total | margin in 6144 B |
|---|---:|---:|---:|---:|
| NORMAL (G53 -O2) | 1624 | 1216 | 2840 | 3304 |
| DIAGNOSTIC (G53 -O2) | 1728 | 1208 | 2936 | 3208 |
| base aab3c3c NORMAL (O0, 2048 B) | 3128 | 1216 | 4344 | −2296 |
| base aab3c3c DIAGNOSTIC (O0, 2048 B) | 3232 | 1208 | 4440 | −2392 |

**C3 — "M-state" label.** M = `0x200039A4` maps to `state_5` (`0x200099B0..0x20009CB0` in the
base ELF), not `state_7`. Only `state_5` offsets M+476..M+767 lie inside the reachable
collision range. The base-ELF collision map itself is **CONFIRMED_CODE / target ELF**: actual
ADC-chain store instructions (FOC_calculation, runPIcontrol, fast_iq_slew_tick,
fis_mailbox_read_verified) address `pas`, `control_remainder`, `normal_permission`,
`dropped_ticks` (already while `chain_e1e8_model` is active, fg depth ≥ 2728 B),
`port_trace`, `state_6`, `state_7` and the upper part of `state_5`. Not observed on hardware.

**C4 — corruption rate.** "~0.8 corruption per 1 ms G53 step" is a **MODEL**, not a
measurement: 8.8 % of O0 G53 instructions run at a depth where an ADC frame covers
`pas`/`control_remainder`/`normal_permission` (1.8–3.1 % for `state_5..7`), multiplied by an
assumed 6–10 ADC entries per step at 16 kHz. No hardware measurement.

**C5 — Walk.** Whole-firmware foreground without the G53 pipeline is 872 B; with the ISR
addition 2028 B (direct) / 2088 B (EXTI nesting), not 2012 B. Lowest MSP stays above `_ebss`
(unused heap only), so Walk does not reproduce the G53-state collision. Walk bypasses G53.

**C6 — classification.** RC1 stack collision: CONFIRMED_CODE / target ELF (not CONFIRMED_HW).
RC2 G53 cost: instruction counts CONFIRMED (O0 mean 44 374, 39 554–62 635; O2 mean 12 443,
11 700–15 209; 64-step O0 2.67–2.73 M, O2 0.66–0.67 M); cycles/time NOT measured on hardware.
RC3 foreground-clocked periodic CAN: CONFIRMED_CODE. Link from any of RC1–RC3 to the displayed
E30: **NOT_CONFIRMED_HW** (the HMI reverse finds no per-frame 3201/3202 timeout).

**C7 — watchdog (deferred hardening, not fixed).** `fwdgt_config(65000, DIV256)` programs a
12-bit RLD = 3560 → 22.78 s at nominal IRC40K (15–30 s over tolerance), but the application
never calls `fwdgt_enable()`; activation depends on option bytes/BL820 (UNKNOWN). Reloads in
TIMER1 and ADC ISRs make it blind to a foreground hang. Classification: SUPPORTED contributor,
deferred.
