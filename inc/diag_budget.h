#ifndef DIAG_BUDGET_H_
#define DIAG_BUDGET_H_

#include "config.h"  /* CAN_DIAGNOSTICS_ENABLE, FW117_TRACE_ENABLE */

/* QS-1: 48 x 44 B full-rate samples + recorder metadata, measured by sizeof(R). */
#if CAN_DIAGNOSTICS_ENABLE
#define DIAG_BUDGET_QS_TRANSITION_BYTES 2240U
#else
#define DIAG_BUDGET_QS_TRANSITION_BYTES 0U
#endif

/*
 * FW-106: the diagnostic RAM budget, checked by the COMPILER rather than by a note in a
 * document or a number measured once by hand.
 *
 * WHAT IS ACTUALLY CHECKED. Each of the four modules groups ALL of its mutable state - every
 * buffer, counter, index and flag, no exceptions - into one file-static struct, and asserts
 * `sizeof(that struct) <= ` its line item below. Nothing here inspects "a couple of arrays": a
 * new scalar added anywhere in one of those structs grows its sizeof() and is checked by the
 * SAME assert that checks the buffers, so it can trip the ceiling exactly as a new buffer would.
 * (`src/pas_raw.c`'s `struct pas_raw_state S`, `src/pas_trace.c`'s `struct T`,
 * `src/ride_episode.c`'s `struct E`, `src/diag_session.c`'s `struct D`,
 * `src/rearm_delay_diag.c`'s `struct R`.)
 *
 * HOW THE NUMBERS BELOW WERE SET. Measured, not estimated: each module was compiled for the
 * real ARM target (arm-none-eabi-gcc, the same flags build_firmware.ps1 uses) with this header's
 * ceilings temporarily raised out of the way, and `arm-none-eabi-size` read off data+bss for the
 * resulting object file. Figures found 2026-08-11 (GCC 13.2.1, CAN_DIAGNOSTICS_ENABLE=1), first
 * measurement and after the second-iteration owner review (candidate-cancel, the 0x10212 fix,
 * freezing the aggregate block per session, and the session-lost bit) side by side:
 *
 *                     first    after review   after pacing fix   after FW-107 fast-rearm marker
 *     pas_raw.c        4136 B      4136 B     4136 B             4136 B   (untouched)
 *     pas_trace.c      7240 B      7240 B     7240 B             7240 B   (untouched)
 *     ride_episode.c   2412 B      2416 B     2416 B             2676 B   (+260 B: one new
 *                                              uint8_t fast_rearm field in the QUEUED result -
 *                                              ride_episode_result_t was already exactly 36 B
 *                                              with no spare padding, so the new byte grows
 *                                              each of the 64 queued entries to 40 B - measured,
 *                                              not the naive +64 B, because the struct's 4-byte
 *                                              alignment (from its int32_t member) rounds every
 *                                              entry up regardless of exactly where the new
 *                                              byte sits)
 *     diag_session.c    388 B     1176 B      1180 B             1180 B   (untouched)
 *     rearm_delay_diag.c                                             312 B  (FW-111: new module,
 *                                              one struct R: 2 records x 136 B + ~40 B of
 *                                              FSM/counters/queue bookkeeping - measured)
 *
 * FW-111 changed diag_session.c: the fourth record source (DIAG_SRC_REARM) grows the per-source
 * counters and the per-summary refused_at_open/close arrays by ~60 B, and DIAG_AGGREGATE_SNAPSHOT_MAX
 * was reduced 16 -> 15 (see diag_session.c) which gives back 48 B. Re-measured 2026-08-13:
 * diag_session.c 1192 B, rearm_delay_diag.c 312 B.
 *
 * FW-111 v2 (owner review rework): the record is now the guaranteed snapshot set ENTER/PROBLEM/
 * COMMIT/CLOSE plus a 24 B timing block, 160 B per record, and the rearm module re-measured at
 * 364 B (2 x 160 B records + ~44 B of FSM/counters/queue/weak bookkeeping). To keep the 12 KB
 * ceiling, DIAG_AGGREGATE_SNAPSHOT_MAX was reduced 15 -> 14 (see diag_session.c), giving back
 * 12 B x 4 summaries = 48 B. Re-measured 2026-08-13 (GCC 13.2.1, CAN_DIAGNOSTICS_ENABLE=1):
 * diag_session.c 1144 B, rearm_delay_diag.c 364 B.
 *
 * FW-111 v3 (this card's execution): the explicit TRACE/RAW reservation cycle adds ONE bool
 * (rearm_refusal_reported, the once-per-hold refusal guard) to pas_trace.c's state - 4 B with
 * the struct's alignment padding. Re-measured 2026-08-13 (GCC 13.2.1, CAN_DIAGNOSTICS_ENABLE=1):
 * pas_trace.c 7244 B (DIAG=0 preexisting slot: 3632 B, +4 B from the same bool),
 * rearm_delay_diag.c 364 B (unchanged - the reservation flags were already in struct R),
 * diag_session.c 1144 B, pas_raw.c 4136 B, ride_episode.c 2676 B. rearm_delay_diag.c stays at
 * 0 B at DIAG=0 (the whole module compiles out).
 *
 * FW-111 v4 (dynamic-slot ownership rework): T.rearm_hold (bool) became T.rearm_slot (uint8_t) -
 * same size - and struct R gained a second one-shot edge bool (prearm_edge, alongside the renamed
 * release_edge) replacing the single reserve_wanted bool it displaces - net field count and total
 * size both unchanged. Re-measured 2026-08-14 (GCC 13.2.1, isolated single-file compiles, same
 * method as above): pas_trace.c 7244 B at DIAG=1 (DIAG=0 slot: 3632 B - BOTH byte-identical to
 * v3), rearm_delay_diag.c 364 B at DIAG=1 (byte-identical to v2/v3), 0 B at DIAG=0. No ceiling in
 * this file needed to move. A full-image link (all 59 sources, arm-none-eabi-gcc 13.2.1, same
 * flags as build_firmware.ps1) shows DIAG=1 total bss 23564 B vs the last full-image measurement
 * of 23568 B - a 4 B DECREASE, not an increase, and NOT visible in either module's own isolated
 * size - ordinary cross-object linker layout/alignment noise between unrelated translation units,
 * not a real cost change (see the FW-111 v4 report for the full text/data/bss table).
 *
 * FW-111 v5 (reservation lifecycle rewrite - PRECOMMIT/POSTCOMMIT ownership, retained-capture
 * split, token-based trigger finalization): struct R (rearm_delay_diag.c) gains
 * pending_record_id/pending_session_id (2 x uint8_t) and struct T (pas_trace.c) gains
 * rearm_retained_slot (uint8_t). Re-measured 2026-08-14 (GCC 13.2.1, isolated single-file
 * compiles, same method as above): pas_trace.c 7244 B at DIAG=1 (byte-identical to v4 - the new
 * field was absorbed by existing padding), 3632 B at DIAG=0 (also unchanged); rearm_delay_diag.c
 * 368 B at DIAG=1 (+4 B, genuinely the two new pending_* fields this time - still 60 B under its
 * 428 B ceiling), 0 B at DIAG=0. A full-image link shows DIAG=0 bss UNCHANGED at 11692 B (this
 * card's own explicit requirement - the production build gains no state) and DIAG=1 bss ALSO
 * unchanged at 23564 B despite rearm_delay_diag.c's own +4 B in isolation - the same kind of
 * link-level alignment absorption already documented for v4, reported here without smoothing
 * over the apparent (but not real) discrepancy. No ceiling in this file needed to move.
 *
 * FW-112-DIAG: a FIFTH record source (DIAG_SRC_FW112) and a new recorder module
 * (src/fw112_diag.c, struct R). FW-112-DIAG.1 raised the ring 8 -> 24 x 32 B event records
 * (768 B buffer + ~32 B of queue/edge bookkeeping = 800 B measured, +64 B headroom = 864 B).
 * The previous capture filled all 8 slots exactly at RECOVERY_ENTER, hiding the recovery tail.
 * The fifth source grew diag_session.c's struct D by exactly
 * 60 B (measured 1204 B: four per-source counter arrays and two per-summary refused arrays each
 * gained one element), line item 1268 B. The line-item total is 13192 B - already past the old
 * 12 KB ceiling (12288) even before the new module's 864 B, so the ceiling is raised to 13 KB:
 * the audit budget is NOT free RAM (48 KB total SRAM, of which the DIAG=1 full-image bss was
 * 23564 B before this card and 23932 B after, +368 B), and the real cost is re-checked by the
 * linker map step before any ride, exactly as this header always required.
 *
 * FW-112 A/B: a SIXTH record source (DIAG_SRC_AB) and a new recorder module (src/fw112_ab.c,
 * struct R). The rearm-episode logger's ring is 144 x 32 B = 4608 B (six complete worst-case
 * episodes per batch, reject-on-full, see inc/fw112_ab.h) plus ~36 B of queue/FSM/edge
 * bookkeeping and a 4 x 4 B pre-grant milestone tick table - measured 4664 B, +64 B headroom =
 * 4728 B. The sixth source grew diag_session.c's struct D by ~60 B (measured 1264 B: the same
 * nine per-source arrays each gained one element), line item 1328 B. The line-item total is
 * 17980 B - past the 13 KB ceiling - so the ceiling is raised to 18 KB. The real cost is
 * re-checked by the linker map step before any ride, exactly as this header always required.
 *
 * FW-117 (TEMPORARY - REMOVE BEFORE SHIPPING): a SEVENTH record source (DIAG_SRC_FW117) and a new
 * recorder module (src/fw117_trace.c, struct R). The bridge lifecycle trace's ring is
 * 480 x 32 B = 15360 B (240 ms pre + 240 ms post at 1 kHz, see inc/fw117_trace.h) plus ~32 B of
 * queue/FSM/edge bookkeeping - measured 15392 B, +64 B headroom = 15456 B, line item 15500 B. The
 * seventh source grows diag_session.c's struct D by ~64 B (the nine per-source arrays each gained
 * one element) - measured 1328 B, line item 1392 B. The line-item total is 33316 B - past the
 * 18 KB ceiling - so the ceiling is raised to 34 KB for this TEMPORARY card. The trace must be
 * removed (module, source, budget line) before the fix ships; the BRIDGE FIX ITSELF (OSSR/OSSI
 * ENABLE + the start deadzone) stays. Dead time is NOT part of that fix - it stayed at 32, see
 * timer0_config() in src/main.c - and neither is the 3 s stop-tick value, which now lives behind
 * FW117_BRIDGE_TIMING_TEST in inc/config.h and is off by default.
 *
 * FW-112-STABILITY (diagnostic-only recovery instrumentation): fw112_diag.c's struct R gains a
 * 4 B edge-metadata pair beside each of the 24 records (24 x 36 B = 864 B instead of 768 B) plus
 * three u16 episode counters and a bool (~8 B) - static layout 904 B, up from 800 B. No new wire
 * traffic: the metadata is written into the record's spare wire bytes (schema 3). The retention
 * eviction then added two u32 counters (evicted_saga_total/evicted_record_total, internal only,
 * never on the wire) - static layout 912 B. Line item raised 864 -> 976 B (measured 904 B + 64 B
 * headroom).
 *
 * FW-122.1 (D2 diagnostic extension, diagnostic-only): rolling_no_assist_diag.c's sample struct
 * grows 44 -> 48 B (two of schema v2's four DATA-5 wire-padding bytes become real fields -
 * pwm_cutoff_progress, hall_timeout_progress - plus one status_flags bit, RNA_STATUS_PWM_
 * CUTOFF_ACTIVE, at zero size cost). Ring 256 x 48 B = 12288 B. Re-measured 2026-08-24
 * (GCC 13.2.1, isolated single-file compile, same method as above): 12316 B bss (was 11264 B),
 * NORMAL object 0/0/0 (unchanged - the whole module still compiles out). Line item raised
 * 11356 -> 12380 B (measured 12316 B + 64 B headroom). Full-image DIAG build (0.0422):
 * FLASH 143836 B (+716 B vs 0.0418's 143120 B), RAM 42592 B (+1040 B vs 41552 B) - both within
 * the 34 KB diag-budget ceiling and the controller's 48 KB total SRAM. Full-image NORMAL build
 * (0.0421): FLASH 100140 B, RAM 12032 B - byte-identical to the pre-FW-122.1 baseline.
 *
 * Each line item below is that measurement plus a fixed 64 B of headroom for compiler/alignment
 * drift between toolchain versions - not slack for casually adding new fields. Re-measure and
 * update these (with the same method) whenever a module's state genuinely needs to grow.
 *
 * The 34 KB ceiling is NOT "free RAM". The controller has 48 KB in total and how much of it is
 * already spoken for is a question only the linker map answers - a separate, explicit step
 * before any ride, which this header does not and cannot replace.
 *
 * ASSIST PIPELINE V2 (2026-09-13): four recorders retired with the legacy assist pipeline -
 * rearm_delay_diag, fw112_diag, fw112_ab and rolling_no_assist_diag/_dump - releasing 18656 B of
 * reservation. Measured on the real link, arm-none-eabi-gcc 13.2.1, whole image: the DIAGNOSTIC
 * variant now reports RAM 30424 B of 48 KB (61.90 %) against FLASH 142024 B (60.30 %); the
 * NORMAL variant, which never allocated any of it, is RAM 39664 B (80.70 %) against 39888 B
 * before - the small change there is the assist chain's own state, not the recorders.
 *
 * The diagnostic variant had NEVER been built before this measurement: --variant was ignored
 * outside auto mode, so every "diagnostic" target build silently produced a normal binary. That
 * is why this is the first entry here with a real DIAG=1 whole-image figure rather than an
 * isolated per-object one.
 */

#define DIAG_BUDGET_PAS_RAW_BYTES        4200U  /* measured 4136 B + 64 B headroom */
#define DIAG_BUDGET_PAS_TRACE_BYTES      7308U  /* measured 7244 B + 64 B headroom (FW-111 v3) */
#define DIAG_BUDGET_EPISODE_QUEUE_BYTES  2740U  /* measured 2676 B + 64 B headroom (FW-107) */
#define DIAG_BUDGET_SESSION_BYTES        1788U  /* FW-121.0: measured 1660 B + 64 B headroom (DIAG_AGGREGATE_SNAPSHOT_MAX 14 -> 21); +64 B for 8th source (DIAG_SRC_ROLLING_NO_ASSIST) */
/*
 * ASSIST PIPELINE V2: three recorders RETIRED, so their reservations are 0.
 *
 * rearm_delay_diag.c, fw112_diag.c and fw112_ab.c instrumented the legacy rearm machinery, the
 * FW-112 permission chain and the rearm-episode lifecycle. Those mechanisms were replaced by one
 * PAS lifecycle and one demand chain, so the recorders had nothing left to observe and were
 * deleted with them. Their DIAG_SRC_* slots stay reserved (wire indices), but they allocate
 * nothing.
 *
 * The names are kept at 0 rather than removed from the total expression: this header is an
 * append-only ledger of what was measured when, and a line that reads "this used to cost 6148 B
 * and now costs nothing" is the useful record. Deleting the term would leave the next reader
 * unable to tell a retirement from an omission.
 */
#define DIAG_BUDGET_REARM_DELAY_BYTES    0U     /* retired with the legacy rearm machinery (was 444 B) */
#define DIAG_BUDGET_FW112_DIAG_BYTES     0U     /* retired with the FW-112 permission chain (was 976 B) */
#define DIAG_BUDGET_FW112_AB_BYTES       0U     /* retired with the rearm-episode logger (was 4728 B) */
#if FW117_TRACE_ENABLE
#define DIAG_BUDGET_FW117_TRACE_BYTES    3425U /* FW-126: 70 x 48 B compact start trace + state */
#else
#define DIAG_BUDGET_FW117_TRACE_BYTES    0U     /* FW-117 trace disabled: replaced by rolling_no_assist_diag */
#endif
/* FW-126.5: the FW-121/FW-126.2 CH3 sweep was DELETED once its question was answered, and its
 * 450 B line item with it. What remains under this name is the FW-126.5 campaign probe: five
 * snapshots, two dark statistic sets and three ADC configuration captures. */
#define DIAG_BUDGET_ADC_TRIGGER_BYTES    260U   /* FW-126.5 campaign probe state */
/* Retired with the legacy assist pipeline, same reasoning as the three above. The rolling
 * no-assist hunt was looking for a defect in a demand chain that no longer exists; its 12380 B
 * ring was the single largest diagnostic allocation in the firmware. */
#define DIAG_BUDGET_ROLLING_NO_ASSIST_DIAG_BYTES 0U     /* retired (was 12380 B) */
#define DIAG_BUDGET_ROLLING_NO_ASSIST_DUMP_BYTES 0U     /* retired (was 128 B) */

/*
 * pas_trace.c keeps ONE slot even in the production build (CAN_DIAGNOSTICS_ENABLE=0) - that slot
 * predates FW-106 and is not this card's spend. Measured directly, the same way, at DIAG=0:
 * 3628 B. Only the SECOND slot and this card's added bookkeeping are new cost, which is why the
 * total below subtracts exactly this figure rather than a rounded guess.
 */
#define DIAG_BUDGET_PAS_TRACE_PREEXISTING_BYTES 3632U

#define DIAG_BUDGET_TOTAL_BYTES ( \
	DIAG_BUDGET_PAS_RAW_BYTES + \
	(DIAG_BUDGET_PAS_TRACE_BYTES - DIAG_BUDGET_PAS_TRACE_PREEXISTING_BYTES) + \
	DIAG_BUDGET_EPISODE_QUEUE_BYTES + \
	DIAG_BUDGET_SESSION_BYTES + \
	DIAG_BUDGET_REARM_DELAY_BYTES + \
	DIAG_BUDGET_FW112_DIAG_BYTES + \
	DIAG_BUDGET_FW112_AB_BYTES + \
	DIAG_BUDGET_FW117_TRACE_BYTES + \
	DIAG_BUDGET_ADC_TRIGGER_BYTES + \
	DIAG_BUDGET_ROLLING_NO_ASSIST_DIAG_BYTES + \
	DIAG_BUDGET_ROLLING_NO_ASSIST_DUMP_BYTES)

/*
 * Ceiling:34 KB when fw117 is disabled (production+rolling_no_assist). 40 KB when
 * fw117 is enabled (temporary bench image, not for shipping). The linker map is the
 * authoritative check before any ride.
 */
#if FW117_TRACE_ENABLE
_Static_assert(DIAG_BUDGET_TOTAL_BYTES <= 40U * 1024U,
	"FW-106: the diagnostic RAM budget exceeds its 40 KB ceiling (fw117 trace enabled)");
#else
_Static_assert(DIAG_BUDGET_TOTAL_BYTES <= 34U * 1024U,
	"FW-106: the diagnostic RAM budget exceeds its 34 KB ceiling");
#endif

#endif /* DIAG_BUDGET_H_ */
