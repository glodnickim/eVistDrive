# AUDIT — Replay Attenuation Divergence (~2x) — 2026-09-20

**Scope:** why `max_attenuation` for `w1-f01-stable-cad49`, `w1-f07-pause-43s`, `w1-f08-pause-10s`
reads ~2x higher today than the value recorded in `docs/ASSIST_PIPELINE_V2.md` on 2026-09-14.
**Method:** empirical reproduction on the real toolchain at four commits (git worktrees, no
history rewrite, no working-tree mutation), not analysis from memory.

---

## A. Git identity

```text
START_HEAD (this audit)  3714ad08ffd4867a48386df63d45e3399890e39d
FINAL_HEAD                3714ad08ffd4867a48386df63d45e3399890e39d  (unchanged — read-only audit)
BRANCH                    feature/assist-pipeline-v2
working tree              clean (no production files touched by this audit)
```

`3714ad0`, its parent `8290bd9`, and the golden behavioral baseline `1d6c6ba` all exist locally
and match what was supplied. No divergence between the supplied task card and reality on this
point.

---

## B. Replay input identity

All three `sim/replay/cases/w1-*/input.csv` are unchanged across the entire window examined
(`5b251b2`..`3714ad0` — no commit in `git log 5b251b2..3714ad0 -- sim/replay/cases/w1-f01-stable-cad49
sim/replay/cases/w1-f07-pause-43s sim/replay/cases/w1-f08-pause-10s` touches them).

| case | rows | SHA256 (input.csv) | matches manifest `source_sha256` |
|---|---:|---|---|
| w1-f01-stable-cad49 | 120 (119 data) | `8fa22ae228d7a34bea989049a18957696da2e2bdcc68ae0cc63867ef42de0f0c` | yes |
| w1-f07-pause-43s | 1059 (1058 data) | `fae293d3acdddbcb736379c43babddab6f3fdb599aed1a70776ca11f4a51cd10` | yes |
| w1-f08-pause-10s | 392 (391 data) | `b4b79cc698c898a00d5d735711cf187a3a2708caff5abcae1098c0531997e0b6` | yes |

**SAME INPUT: YES**, both by SHA256 and by `git log` showing zero commits touching these paths
in the window.

---

## C. Metric definition (exact, from `tools/replay_behavior.py:65-143`)

```python
def _ripple(vals):
    m = mean(vals)
    return (max(vals) - min(vals)) / abs(m)      # peak-to-peak over mean, whole fragment

torque = column 'torque_ckg'        # == load_centikg, the DISPLAY/measured kg value
iq     = column 'iq_ref_new'        # the actual current request the pipeline produced

tq_r  = _ripple(torque)
iq_r  = _ripple(iq)
attenuation = iq_r / tq_r            # smaller = more attenuation = better
PASS if attenuation <= 0.60 AND request is not pinned at its ceiling for >10% of the fragment
```

No FFT, no fundamental/harmonic extraction, no RMS, no percentile, no detrending, no windowing
beyond "the whole fragment," no half-cycle/full-cycle distinction anywhere in this formula. This
rules out §9 of the task brief (half-stroke vs full-stroke, single- vs double-sided FFT, etc.) as
a candidate outright — the metric contains none of those constructs and never has, in the window
examined (§D confirms the file is byte-identical across the whole window).

**0 = full attenuation (no ripple reaches Iq). 1 = none. Smaller is better.**

---

## D. Historical source of 0.318 / 0.372 / 0.302

Found by `git log --all -p -S"0.318"`, not reconstructed:

```text
commit 5b251b238f0f08c2da0160f26271c49039b8e623   2026-09-14T09:41:50+02:00
"test: make the telemetry and ride evidence able to fail"
docs/ASSIST_PIPELINE_V2.md:
  "Measured today: f01 `0.318`, f07 `0.372`, f08 `0.302`."
```

That commit is also the one that *introduced* the `max_attenuation` criterion and
`replay_behavior.py` in their current form — the numbers were first published by the same commit
that defined how to compute them, on the state of the repo as it stood at that commit.

`tools/analyze_assist_ripple.py`, `tools/run_replay_regression.py`, `tools/replay_behavior.py`,
and all three `w1-*/manifest.json` are **byte-identical** between `5b251b2` and current `3714ad0`
(`git diff 5b251b2 3714ad0 -- <those paths>` produces no output). **SAME ANALYZER: YES.**
**SAME WINDOW: YES** (window = "the whole fragment," unchanged in the formula itself).

---

## E. Current results (reproduced, current HEAD)

```text
$ python tools/run_replay_regression.py   (at 3714ad0)
w1-f01-stable-cad49  torque ripple 3.561  iq ripple 2.201  attenuation 0.618  FAIL (limit 0.6)
w1-f07-pause-43s     torque ripple 11.401 iq ripple 8.947  attenuation 0.785  FAIL (limit 0.6)
w1-f08-pause-10s     torque ripple 6.919  iq ripple 4.070  attenuation 0.588  PASS (limit 0.6)
```

Matches the task card exactly.

---

## F. Cross-version matrix (empirical — built and run at each commit in an isolated git worktree,
same analyzer, same input, current toolchain)

| commit | role | f01 tq_r / iq_r / att | f07 tq_r / iq_r / att | f08 tq_r / iq_r / att |
|---|---|---|---|---|
| `1d6c6ba` | golden behavioral baseline | 6.920 / 2.201 / **0.318** | 24.079 / 8.947 / **0.372** | 13.469 / 4.070 / **0.302** |
| `5b251b2` | commit that published 0.318/0.372/0.302 | 6.920 / 2.201 / **0.318** | 24.079 / 8.947 / **0.372** | 13.469 / 4.070 / **0.302** |
| `3ccfb9d` | START_HEAD (AUDIT_FINAL's baseline) | 3.561 / 2.108 / **0.592** | 11.401 / 10.144 / **0.890** | 6.919 / 4.822 / **0.697** |
| `3714ad0` | current HEAD | 3.561 / 2.201 / **0.618** | 11.401 / 8.947 / **0.785** | 6.919 / 4.070 / **0.588** |

Read down each column and the mechanism is exact, not approximate:

- **`torque ripple` (denominator) drops from the baseline value to the current value once, between
  `1d6c6ba` and `3ccfb9d`, and then stays fixed** (`3ccfb9d` and `3714ad0` have identical torque
  ripple: 3.561 / 11.401 / 6.919 in both). It roughly halves.
- **`iq ripple` (numerator) at current HEAD (`3714ad0`) is byte-identical to the golden baseline
  (`1d6c6ba`)**: 2.201 / 8.947 / 4.070 in both. It is different, and genuinely regressed, only at
  the intermediate `3ccfb9d` state (2.108 / 10.144 / 4.822).

**SAME INPUT: YES. SAME ANALYZER: YES. SAME WINDOW: YES.**

---

## G. Difference analysis

| candidate cause | verdict | evidence |
|---|---|---|
| input file changed | **NO** | §B — SHA256 and `git log` both clean |
| schema/decoder changed | **NO** | no commit in the window touches the decode/replay-import path for these fixtures |
| window changed | **NO** | formula has one window ("whole fragment"), unchanged since introduction |
| metric/analyzer changed | **NO** | `replay_behavior.py` byte-identical `5b251b2`..`3714ad0` |
| half-cycle vs full-cycle / peak vs peak-to-peak / RMS vs amplitude / ×2 normalization | **NO** | the formula (§C) contains none of these constructs at any commit examined |
| control-path (Iq) regression, present today | **NO** | `iq_ripple` at current HEAD is identical to golden baseline, all 3 cases (§F) |
| control-path (Iq) regression, historical, already fixed | **YES, but not the current divergence** | `iq_ripple` was genuinely different only at `3ccfb9d`; `8290bd9`+`3714ad0` restored it exactly to baseline — this is the CLU domain-separation work described in `docs/AUDIT_FINAL_TORQUE_DOMAIN_PIPELINE2_2026-09-20.md` |
| **`torque_ckg` (metric input) redefined by a firmware change to the display kg curve** | **YES — this is the ~2x** | §H |

---

## H. Root cause

```text
FIRMWARE  regression in Iq:                NO (proven identical to baseline at current HEAD)
FIRMWARE  change to the kg display curve:  YES, intentional (FW-150 measurement correction)
ANALYZER  change:                          NO
INPUT     change:                          NO
SCHEMA    change:                          NO
METRIC    formula change:                  NO (formula itself never changed)
METRIC    INPUT-SIGNAL MEANING changed underneath an unchanged formula:  YES — this is the cause
```

**`max_attenuation` divides an unchanged signal (`iq_ref_new`, the real assist current) by a
signal whose scale changed for an intentional, documented, unrelated reason (`torque_ckg`, the
*displayed* kilogram value).**

`torque_ckg` = `load_centikg`, produced by `torque_input.c`'s native-mV → kg conversion curve.
That curve was replaced — introduced by commit `6ce18aa` (§I) — from the old two-point
`TORQUE_DEFAULT_LOW_NATIVE=146 → 600 centikg` characteristic to the FW-150 three-point *measured*
characteristic (`3.00 kg @ 760 mV`, `9.50 kg @ 925 mV`, `20.00 kg @ 1520 mV`). This is a
metrology correction to the number shown to the rider and logged to CAN — `docs/AUDIT_FINAL_TORQUE_DOMAIN_PIPELINE2_2026-09-20.md` §C independently documents that this curve swap changes sensitivity by 2.7x–4x depending on where on the curve you are, and that same audit's entire thesis (§F: "Why the kg table is NOT reused for control") is that this curve must **never** be allowed to retune the bike — and today, per the empirical `iq_ripple` match in §F, it does not.

But `replay_behavior.py`'s `max_attenuation` criterion was written against `torque_ckg` before
that separation existed, and nothing updated it when the kg curve changed. It is still comparing
"how much did the *displayed kilogram number* wiggle" against "how much did the *real assist
current* wiggle" — and the displayed-kilogram side of that ratio just got re-scaled by a
measurement fix that has nothing to do with assist quality. The `0.60` limit was calibrated
(`5b251b2`, 2026-09-14) against ripple magnitudes measured on the *old* kg curve. Comparing
today's ripple, measured on the *new* kg curve, against that same limit is not a like-for-like
comparison.

**This is a genuine finding of `CLAIM-EVD-TORQUE-SCALE-004` / `CLAIM-EVD-TORQUE-RANGE-COMPRESSION-005` bleeding into a downstream analyzer that was never told about the domain split** — same underlying event (the FW-150 kg curve replacement), different symptom (test-gate divergence instead of rider-facing threshold sensitivity). Both open claims track the *curve*; this audit tracks one *consumer* of the curve that assumed it was control-invariant, which — as of `8290bd9`/`3714ad0` — it now provably is, except in this one analyzer.

---

## I. First bad commit (for the metric divergence specifically)

```text
FIRST BAD COMMIT   6ce18aa0ba6d1e05f7ff84910d616bf83ebd0b77   2026-09-20T15:15:53+02:00
```

Its own commit message: `"docs/build: update references to ap2_torque_chain"` — described as a
documentation/build-reference-only commit ("AGENTS.md: rider demand owner now...", "Build scripts:
... references", "config.h: updated constants"). It is **not** docs/build-only: it adds
`TORQUE_CURVE_P1/P2/P3_NATIVE/CENTIKG`, `default_curve[]`, and the `interpolate()`-based
`default_native_delta_to_centikg()`/`default_centikg_to_native_delta()` to
`src/torque_input.c`/`inc/torque_input.h`, replacing the two-point characteristic with the FW-150
three-point measured one. This is the commit where `torque_ckg`'s scale changes; it is not called
out as a functional/behavioral change anywhere in the commit message.

Confirmed by direct `git show <sha>:inc/torque_input.h` at each commit:

| commit | curve in `inc/torque_input.h` |
|---|---|
| `1d6c6ba` | old: `TORQUE_DEFAULT_LOW_NATIVE=146`, `TORQUE_DEFAULT_LOW_CENTIKG=600` |
| `2091c2a` | unchanged (old curve) |
| `6ce18aa` | **changed**: `TORQUE_CURVE_P1_NATIVE=20` (`3.00 kg @ 760 mV`) et al. |
| `e4f2b88` | new curve retained |

This is *not* the same commit as the control-path regression documented in
`AUDIT_FIX_TORQUE_INPUT_PIPELINE2_2026-09-20.md` (root cause `2091c2a`, the `ap2_torque_chain`
double-normalization) or the one in `AUDIT_FINAL_TORQUE_DOMAIN_PIPELINE2_2026-09-20.md`
(`3ccfb9d`, the threshold-restoration defect) — those are real, independently diagnosed, and
already fixed. This audit adds a third, distinct event on the same day: the kg-curve swap that
those two audits correctly kept out of the control path, but that this analyzer still reads from.

---

## J. Exact line/function responsible

```text
tools/replay_behavior.py:86      torque = _col(rows, 'torque_ckg')
tools/replay_behavior.py:139     tq_r, iq_r = _ripple(torque), _ripple(iq)
tools/replay_behavior.py:140     att = iq_r / tq_r ...
```

`torque_ckg` in the replayed CSV is populated by the replay/import chain from `load_centikg`,
which is produced by `default_native_delta_to_centikg()` in `src/torque_input.c` (the function
changed in `6ce18aa`, §I).

---

## K. Recommendation (smallest possible; NOT applied by this audit)

Do not touch the `0.60` limit, and do not touch `ap2_rider_demand.c`, `assist_pipeline.c`, or any
other file on the production-code do-not-touch list — none of them are responsible for this
divergence, and this audit changed none of them.

The smallest change that resolves the divergence without weakening the gate: compute
`max_attenuation`'s **torque-side ripple from the canonical/control-domain torque signal** (the
CLU or native-delta value that `8290bd9`/`3714ad0` already established as the one true
control-invariant quantity — see `docs/AUDIT_FINAL_TORQUE_DOMAIN_PIPELINE2_2026-09-20.md` §E/§F),
not from `torque_ckg`/`load_centikg`, which is explicitly documented as display-only and
re-measurable without notice. That keeps the criterion doing what it was meant to do — proving the
Iq path attenuates *rider effort* ripple — without being disturbed the next time the kg display
curve is re-measured for accuracy. This is a **replay-tool change** (`tools/replay_behavior.py` +
the CSV column the replay/import chain emits), not a firmware change, and is explicitly in scope
per the task's §21 ("replay tool / analyzer / decoder / test harness / docs" may be modified once
root cause is unambiguous).

If the native/CLU column is not presently exported by the replay CSV, exporting it is the
prerequisite step, and is itself infrastructure, not firmware.

**Separately, and not a finding of this audit but relevant context:** `CLAIM-EVD-TORQUE-SCALE-004`
and `CLAIM-EVD-TORQUE-RANGE-COMPRESSION-005` remain **OPEN** — the FW-150 curve itself has not
been independently validated against the physical sensor to production accuracy. Nothing in this
audit resolves those claims; this audit only shows that the current control path (`iq_ref_new`)
does not depend on that curve, which is a narrower, already-established fact
(`AUDIT_FINAL_TORQUE_DOMAIN_PIPELINE2_2026-09-20.md`).

---

## L. Definition of done — answers

1. **Formula:** `attenuation = ripple(iq_ref_new) / ripple(torque_ckg)`, `ripple(x) = (max-min)/|mean|` over the whole fragment. §C.
2. **Source of 0.318/0.372/0.302:** commit `5b251b2`, 2026-09-14, `docs/ASSIST_PIPELINE_V2.md`. §D.
3. **Same input:** yes, SHA256-verified, no commit touches the fixtures. §B.
4. **Same signals:** input signal (`torque_ckg`) is the same *column name* but its *scale* changed underneath it (§H); output signal (`iq_ref_new`) is provably unchanged at current HEAD vs baseline (§F).
5. **Same window:** yes — the formula has always used the whole fragment, no windowing change exists to find. §C.
6. **Same analyzer version:** yes, byte-identical file. §D.
7. **Firmware or methodology:** methodology — specifically, an analyzer reading a signal whose meaning a firmware change (correctly) redefined, without the analyzer being told. §H.
8. **First bad commit:** `6ce18aa` for this specific divergence (kg curve swap); note two other, separate, already-fixed regressions exist nearby (`2091c2a`, `3ccfb9d`) — see §I for how they differ.
9. **Synthetic test confirms formula correctness:** not yet added — §M.
10. **Recommendation:** feed `max_attenuation` from the control-domain torque signal, not the display-kg signal. §K.

---

## M. Methodology regression test — NOT YET ADDED

The task requires a synthetic-signal regression test pinning `_ripple`/`attenuation` (e.g.
input=100/output=50 → 0.500; input=100/output=25 → 0.250) and confirming no accidental ×2/half-
cycle factor exists in the formula. §C already establishes by *code reading* that no such factor
exists anywhere in `_ripple()`/`evaluate()` — there is no cycle-counting, no FFT, no RMS, nothing
that could introduce a stray factor of 2. Adding the pinned test itself is test-only, in scope per
§21, and is recommended as a follow-up alongside §K, not performed in this audit to keep this
change to analysis only, per the task's "propose only after report" instruction.

---

## FINAL ANSWER BLOCK

```text
START_HEAD:  3714ad08ffd4867a48386df63d45e3399890e39d
FINAL_HEAD:  3714ad08ffd4867a48386df63d45e3399890e39d   (unchanged — read-only)

ROOT CAUSE: METRIC (analyzer reads a display-domain signal whose firmware-side scale
            intentionally changed; the real control-path signal is unchanged)

OLD METRIC (5b251b2 / 1d6c6ba, reproduced):
f01: 0.318
f07: 0.372
f08: 0.302

CURRENT METRIC (3714ad0, reproduced):
f01: 0.618
f07: 0.785
f08: 0.588

SAME INPUT:     YES
SAME ANALYZER:  YES
SAME WINDOW:    YES

FIRST BAD COMMIT (for this divergence): 6ce18aa0ba6d1e05f7ff84910d616bf83ebd0b77
  (mislabeled "docs/build" commit; actually swaps the torque->kg curve in torque_input.c/h)

FILES CHANGED BY THIS AUDIT: none (docs/AUDIT_REPLAY_ATTENUATION_DIVERGENCE_2026-09-20.md only)

TESTS: NOT_RUN by this audit beyond the reproduction runs shown in §E/§F (all reproduced
       successfully; no production code was built differently than upstream tooling already does)

PIPELINE 2 MODIFIED: NO

RECOMMENDATION: change tools/replay_behavior.py's max_attenuation torque-side input from
  torque_ckg (display kg, load_centikg) to the control-domain torque signal (native delta or
  CLU) that docs/AUDIT_FINAL_TORQUE_DOMAIN_PIPELINE2_2026-09-20.md already established as
  control-invariant. Do not change the 0.60 limit. Do not touch ap2_*.c / assist_pipeline.c /
  torque_input.c. Add the synthetic-signal pinning test from §M as part of that same change.
```

---

## POST-AUDIT IMPLEMENTATION ADDENDUM

Everything above this heading is the audit as it was written on 2026-09-20, at
`START_HEAD = 3714ad08ffd4867a48386df63d45e3399890e39d`, and is left unchanged. Section M and the
FINAL ANSWER BLOCK correctly record that, at the time of writing, the recommendation had been
stated but not carried out. This addendum records only that it was carried out afterwards; it
revises none of the audit's findings, root cause, or first-bad-commit determination.

```text
implemented in commit:
97b8a86a76a05f4df5657fc686187bc17871819e
```

### What the fix does

- `sim/replay/replay_fw.c` now exports a `torque_ctrl` column, written from `ts->load_ctrl` - the
  frozen control-domain pedal signal (CLU) that `ap2_pas_state.c` and `ap2_rider_demand.c`
  actually gate and scale on.
- `max_attenuation` in `tools/replay_behavior.py` computes its torque-side ripple from
  `torque_ctrl`, not from `torque_ckg` (the display kilogram curve).
- A trace with no `torque_ctrl` column raises `MissingControlDomainSignal`. The criterion fails
  loudly instead of reporting a number computed from the wrong domain.
- There is **no fallback to `torque_ckg`** on the `max_attenuation` path. (`responds_to_load`
  keeps a documented soft preference, because it checks only the DIRECTION of movement and is not
  sensitive to the kg-curve-vs-CLU distinction the way a ripple ratio is - see the comment in
  `replay_behavior.py`.)
- The `0.60` limit is unchanged. The fix corrects which signal is measured, not what is required
  of it.
- **Pipeline 2 was not modified.** No file under `src/ap2_*.c`, `src/assist_pipeline.c`,
  `src/torque_input.c`, `src/fast_iq_slew.c`, `src/FOC.c`, `src/foc_current_loop.c`,
  `src/ap2_profiles.c` or `src/ap2_limits.c` was touched. Commit `97b8a86` changes exactly four
  files: this document, `sim/replay/replay_fw.c`, `tests/test_replay_behavior.py` and
  `tools/replay_behavior.py`.

### Results after the fix

`python tools/run_replay_regression.py`, control-domain metric, limit `0.60`:

```text
f01 = 0.318 PASS
f07 = 0.372 PASS
f08 = 0.302 PASS
```

These are the pre-divergence values from §E reproduced on current HEAD, which is the point: the
`0.618 / 0.785 / 0.588` figures recorded in the FINAL ANSWER BLOCK were the display-kg curve
moving, never the assist. `iq_ref_new` was byte-identical throughout, as §D established.

### Tests

In `tests/test_replay_behavior.py`:

- **kg-rescale invariance — PASS.** Two traces share identical `torque_ctrl` and `iq_ref_new`
  while one has `torque_ckg` re-scaled 2.1x, as if its kg curve had just been re-measured. Both
  must report the same attenuation. This is the 2026-09-14 → 2026-09-20 divergence reproduced
  synthetically.
- **missing `torque_ctrl` refusal — PASS.** A trace without the column must raise
  `MissingControlDomainSignal` rather than silently read `torque_ckg`.
- **synthetic attenuation math — PASS** (added as the follow-up §M asked for, in commit
  `test: pin replay attenuation math and close audit`). The two tests above prove the criterion
  reads the right *column*; they do not prove it computes the right *number*, because each
  compares two runs against each other rather than against arithmetic. This one pins the
  arithmetic itself, through the real `tools/replay_behavior.py` rather than a local copy of the
  formula:

  ```text
  torque_ctrl 200..600 about a mean of 400  -> ripple 1.000
  iq          225..375 about a mean of 300  -> ripple 0.500
  expected attenuation                      =       0.500   PASS

  torque_ctrl 200..600 about a mean of 400  -> ripple 1.000
  iq        262.5..337.5 about a mean of 300 -> ripple 0.250
  expected attenuation                      =       0.250   PASS
  ```

  Both reported ripples are asserted, not only the quotient. That is deliberate: a half-cycle
  (amplitude instead of full peak-to-peak) error cancels in the ratio and would leave the quotient
  correct, so asserting the quotient alone would not catch the ×2 class of defect §M named. The
  check was confirmed to fail against a deliberately mutated `_ripple()` before being accepted.

§C's conclusion — that no stray ×2 or half-cycle factor exists in `_ripple()`/`evaluate()` — was
reached by code reading. It is now pinned by an executable test, and §M is closed.
