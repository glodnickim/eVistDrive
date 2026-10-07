# Assist Behavior V3 — Baseline Freeze

FROZEN_AT: 2026-10-07T12:15:18+02:00

| Item | Value |
|---|---|
| Repository | `glodnickim/eVistDrive` (local: `motor-controller-firmware`) |
| Baseline commit | `25df554d652ec62f672a622210fa2a03d8589f95` |
| Source branch | `feat/M560-auto-splus` (local == `origin`, verified after HTTPS fetch 2026-10-07) |
| Freeze ref | annotated tag `baseline/rideable-25df554` (local; push is done by the owner) |
| Release builds | 0.638 (NORMAL) / 0.639 (DIAG) |
| Gate claimed by commit | `verify_all --require-target` PASS, 75/75 host suites (Master, 2026-10-06) — not re-run at freeze time |
| V3 branch | `feature/assist-behavior-v3`, created from the baseline commit |
| V3 worktree | `C:\Projekty\eVistDrive-assist-v3` |

## Why this commit

- The owner names it as the bike's current riding reference.
- Local and remote refs agree; the working tree is clean.
- Its history is the whole M820 G53 line: TQ-06 G53 port, TQ-06-G1 battery limiter, TQ-06-G2 reverse/veto/fast-slew,
  M560 per-level parameters over 0x6010/0x6011, the G5300-only PEDAL stop path, and the Sport+ AUTO support ratio.

## Newer local commits, deliberately not in the baseline

| Commit | Branch | Content | Why excluded |
|---|---|---|---|
| `e2868f6` | `feat/M560-hmi-src3` | HMI node 3: READ 0xF203 reply, 0x6011 transfer from node 3 | CAN app path only, HW_PENDING; the F203 hypothesis was rejected on the bike (integration `5f7d813`) |
| `0c04f24` | `feat/M560-wheel-bcast` | periodic 0x02F83203 wheel/limit broadcast (builds 0.642/0.643) | CAN app path only, HW_PENDING |

Neither commit touches the assist path. Both touch CAN handling (0x6011 transfer ownership), so the V3 config protocol
must be merged with them once they are accepted. That is a known merge point, not a blocker.

## Rules

- Nobody commits to `feat/M560-auto-splus` or moves the tag for V3 work.
- All V3 work goes to `feature/assist-behavior-v3`.
- Every V3 comparison is "baseline `25df554` vs V3 candidate" on identical inputs.
