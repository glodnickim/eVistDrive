/*
 * Assist Behavior V3 - DIAG frame group on the FW-145 live-ride stream (ARCHITECTURE_V3.md 11,
 * inc/ride_telemetry.h, inc/config.h ASSIST_V3_TELEMETRY_FRAMES). Real module: src/ride_telemetry.c
 * built as the diagnostic V3 firmware builds it (CAN_DIAGNOSTICS_ENABLE=1, ASSIST_V3=1; main.c
 * marks the snapshot valid while V3 is the ACTIVE engine, or always with the shadow opt-in).
 *
 *  F1 a snapshot WITHOUT V3 data (v3.valid = false, i.e. G5300 engine / G-EQ rule 3) sends
 *     exactly the nine schema-2 frames per cycle and never an identifier above RIDER;
 *  F2 a snapshot WITH V3 data appends exactly five frames, BASE+10..14, after RIDER; every
 *     pre-existing frame keeps its identifier and position; all fourteen carry the same tick;
 *  F3 payload layout of V3A..V3E (big-endian, packed class/mode/schema byte, flags, published
 *     final_iq, backstop_iq, CPU probe, dropped G53 ticks, backstop state, flags2), V3 schema 2;
 *  F4 pacing is unchanged: frame starts stay >= RIDE_TELEMETRY_FRAME_INTERVAL_TICKS apart, so
 *     the group lengthens the snapshot period instead of adding bus load;
 *  F5 the diagnostic CAN-id map reserves the V3 range (compile-time, diag_efid_map.h).
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ride_telemetry.h"
#include "diag_efid_map.h"

#if !ASSIST_V3_TELEMETRY_FRAMES
#error "build with -DCAN_DIAGNOSTICS_ENABLE=1 -DASSIST_V3=1"
#endif

_Static_assert(DIAG_EFID_RIDE_TELEM_HI == RIDE_TELEMETRY_EFID_BASE + 14U,
               "F5 the V3 frame range is reserved in the diagnostic id map");
_Static_assert(RIDE_TELEMETRY_V3_SCHEMA == 2U, "F3 V3 schema 2 (Milestone C: V3D/V3E added)");

#define MAX_FRAMES 64

typedef struct { uint32_t id; uint8_t d[8]; uint32_t t; } frame_t;
static frame_t frames[MAX_FRAMES];
static unsigned frame_count;
static uint32_t now;

static uint8_t fake_tx(uint32_t efid, const uint8_t *data)
{
    if (frame_count >= MAX_FRAMES) return DIAG_CAN_NOMAILBOX;
    frames[frame_count].id = efid;
    memcpy(frames[frame_count].d, data, 8U);
    frames[frame_count].t = now;
    frame_count++;
    return 1U;
}
static uint8_t fake_state(uint8_t mailbox) { (void)mailbox; return DIAG_CAN_OK; }
static const diag_can_ops_t ops = { fake_tx, fake_state };

static uint16_t be16(const uint8_t *d) { return (uint16_t)(((uint16_t)d[0] << 8) | d[1]); }
static int16_t bei16(const uint8_t *d) { return (int16_t)be16(d); }

static int failures;
static void check(int ok, const char *what) { if (!ok) { failures++; printf("  FAIL  %s\n", what); } }

/* Run the stream from `t0` until `n` data frames (META excluded) were sent; returns the index of
 * the first of them in frames[]. */
static unsigned send_data_frames(uint32_t t0, unsigned n, uint32_t *t_end)
{
    unsigned first = frame_count, data = 0U;
    for (now = t0; data < n && now < t0 + 2000U; now++) {
        const unsigned before = frame_count;
        ride_telemetry_step(now, true, true);
        if (frame_count != before) {
            if (frames[frame_count - 1U].id == RIDE_TELEMETRY_EFID_META) { if (first == before) first++; }
            else data++;
        }
    }
    *t_end = now;
    return first;
}

int main(void)
{
    ride_telemetry_snapshot_t s;
    uint32_t t = 0U;
    puts("assist_v3 DIAG frame group on the FW-145 stream (V3 diagnostic build)");

    ride_telemetry_init(&ops);
    memset(&s, 0, sizeof(s));
    s.control_tick = 0x2468U;
    s.v3.valid = false;
    ride_telemetry_capture(&s);
    /* F1: two cycles without V3 data -> 18 data frames, never above RIDER */
    {
        const unsigned first = send_data_frames(10U, 2U * RIDE_TELEMETRY_DATA_FRAMES, &t);
        int above = 0, order_ok = 1;
        for (unsigned i = first; i < frame_count; i++) {
            if (frames[i].id == RIDE_TELEMETRY_EFID_META) continue;
            if (frames[i].id > RIDE_TELEMETRY_EFID_RIDER) above++;
        }
        for (unsigned i = 0U; i < RIDE_TELEMETRY_DATA_FRAMES; i++) {
            const uint32_t want = RIDE_TELEMETRY_EFID_BASE + ((i <= 6U) ? i : (i + 1U));
            if (frames[first + i].id != want) order_ok = 0;
        }
        check(above == 0, "F1a no V3 frame when the snapshot carries no V3 data");
        check(order_ok, "F1b the nine schema-2 frames keep their identifiers and order");
    }

    /* F2/F3: V3 data present */
    s.control_tick = 0x1357U;
    s.v3.valid = true;
    s.v3.intent = 1234U; s.v3.env_equiv = 567U; s.v3.demand_iq = -21;
    s.v3.base_target_iq = 400; s.v3.target_iq = 399; s.v3.e_short = 1111U;
    s.v3.kappa_q12 = 6029U; s.v3.template_conf = 200U; s.v3.phase = 95U;
    s.v3.release_class = 4U; s.v3.rate_mode = 5U; s.v3.flags = 0xA5U;
    s.v3.final_iq = 321; s.v3.backstop_iq = -1; s.v3.cpu_max_div16 = 0x1234U;
    s.v3.dropped_logical_ticks = 7U; s.v3.cpu_last_div16 = 0x0456U; s.v3.backstop_state = 3U;
    s.v3.flags2 = 0x0BU;
    ride_telemetry_capture(&s);
    {
        /* finish whatever cycle is in flight, then one full V3 cycle */
        unsigned first = 0U;
        const unsigned want_n = RIDE_TELEMETRY_DATA_FRAMES + RIDE_TELEMETRY_V3_FRAMES;
        unsigned start = frame_count;
        (void)send_data_frames(t, 3U * want_n, &t);
        /* locate a complete cycle carrying tick 0x1357 starting at CORE */
        for (unsigned i = start; i < frame_count; i++) {
            if (frames[i].id == RIDE_TELEMETRY_EFID_CORE && be16(frames[i].d) == 0x1357U) { first = i; break; }
        }
        check(first != 0U, "F2a a V3-carrying cycle was sent");
        unsigned k = 0U, coherent = 1U;
        uint32_t ids[20];
        for (unsigned i = first; i < frame_count && k < want_n; i++) {
            if (frames[i].id == RIDE_TELEMETRY_EFID_META) continue;
            ids[k++] = frames[i].id;
            if (be16(frames[i].d) != 0x1357U) coherent = 0U;
        }
        check(k == want_n, "F2b the V3 cycle has 9 + 5 data frames");
        int ids_ok = 1;
        for (unsigned i = 0U; i < k; i++) {
            const uint32_t want = RIDE_TELEMETRY_EFID_BASE + ((i <= 6U) ? i : (i + 1U));
            if (ids[i] != want) ids_ok = 0;
        }
        check(ids_ok, "F2c identifiers BASE+0..6, 8..9, then V3 BASE+10..14");
        check(coherent, "F2d one coherent tick across all fourteen frames");
        /* the frame after the V3 group starts the next cycle at CORE (or META) */
        unsigned after = first;
        for (unsigned seen = 0U; after < frame_count && seen < want_n; after++)
            if (frames[after].id != RIDE_TELEMETRY_EFID_META) seen++;
        check(after >= frame_count || frames[after].id == RIDE_TELEMETRY_EFID_CORE ||
              frames[after].id == RIDE_TELEMETRY_EFID_META, "F2e the next cycle restarts at CORE");

        const frame_t *a = 0, *b = 0, *c = 0, *dd = 0, *e = 0;
        for (unsigned i = first; i < frame_count; i++) {
            if (frames[i].id == RIDE_TELEMETRY_EFID_V3_BASE && !a) a = &frames[i];
            if (frames[i].id == RIDE_TELEMETRY_EFID_V3_BASE + 1U && !b) b = &frames[i];
            if (frames[i].id == RIDE_TELEMETRY_EFID_V3_BASE + 2U && !c) c = &frames[i];
            if (frames[i].id == RIDE_TELEMETRY_EFID_V3_BASE + 3U && !dd) dd = &frames[i];
            if (frames[i].id == RIDE_TELEMETRY_EFID_V3_BASE + 4U && !e) e = &frames[i];
        }
        check(a && be16(&a->d[2]) == 1234U && be16(&a->d[4]) == 567U && bei16(&a->d[6]) == -21,
              "F3a V3A intent / env_equiv / demand");
        check(b && bei16(&b->d[2]) == 400 && bei16(&b->d[4]) == 399 && be16(&b->d[6]) == 1111U,
              "F3b V3B base target / target / e_short");
        check(c && be16(&c->d[2]) == 6029U && c->d[4] == 200U && c->d[5] == 95U &&
              (c->d[6] & 7U) == 4U && ((c->d[6] >> 3) & 7U) == 5U &&
              ((c->d[6] >> 6) & 3U) == RIDE_TELEMETRY_V3_SCHEMA && c->d[7] == 0xA5U,
              "F3c V3C kappa / conf / phase / class|mode|schema / flags");
        check(dd && bei16(&dd->d[2]) == 321 && bei16(&dd->d[4]) == -1 && be16(&dd->d[6]) == 0x1234U,
              "F3d V3D published final_iq / backstop_iq (-1 open) / cpu max");
        check(e && be16(&e->d[2]) == 7U && be16(&e->d[4]) == 0x0456U && e->d[6] == 3U && e->d[7] == 0x0BU,
              "F3e V3E dropped G53 ticks / cpu last / backstop state / flags2");
    }

    /* F4: pacing */
    {
        int pace_ok = 1;
        for (unsigned i = 1U; i < frame_count; i++)
            if (frames[i].t - frames[i - 1U].t < RIDE_TELEMETRY_FRAME_INTERVAL_TICKS) pace_ok = 0;
        check(pace_ok, "F4 frame starts stay >= the 3 ms frame interval apart");
    }

    if (failures) { printf("V3 DIAG frames: %d check(s) FAILED\n", failures); return 1; }
    puts("V3 DIAG frames: PASS");
    return 0;
}
