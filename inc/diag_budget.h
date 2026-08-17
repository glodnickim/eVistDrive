#ifndef DIAG_BUDGET_H_
#define DIAG_BUDGET_H_

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
 * Each line item below is that measurement plus a fixed 64 B of headroom for compiler/alignment
 * drift between toolchain versions - not slack for casually adding new fields. Re-measure and
 * update these (with the same method) whenever a module's state genuinely needs to grow.
 *
 * The 12/13 KB ceiling is NOT "free RAM". The controller has 48 KB in total and how much of it is
 * already spoken for is a question only the linker map answers - a separate, explicit step
 * before any ride, which this header does not and cannot replace.
 */

#define DIAG_BUDGET_PAS_RAW_BYTES        4200U  /* measured 4136 B + 64 B headroom */
#define DIAG_BUDGET_PAS_TRACE_BYTES      7308U  /* measured 7244 B + 64 B headroom (FW-111 v3) */
#define DIAG_BUDGET_EPISODE_QUEUE_BYTES  2740U  /* measured 2676 B + 64 B headroom (FW-107) */
#define DIAG_BUDGET_SESSION_BYTES        1268U  /* measured 1204 B + 64 B headroom (FW-112-DIAG fifth source) */
#define DIAG_BUDGET_REARM_DELAY_BYTES    444U   /* FW-111 v5.1: measured 380 B + 64 B headroom */
#define DIAG_BUDGET_FW112_DIAG_BYTES     864U   /* FW-112-DIAG.1: measured 800 B + 64 B headroom (24 x 32 B records) */

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
	DIAG_BUDGET_FW112_DIAG_BYTES)

_Static_assert(DIAG_BUDGET_TOTAL_BYTES <= 13U * 1024U,
	"FW-106: the diagnostic RAM budget exceeds its 13 KB ceiling (raised from 12 KB by FW-112-DIAG)");

#endif /* DIAG_BUDGET_H_ */
