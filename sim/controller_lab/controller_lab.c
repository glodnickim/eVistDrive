/*
 * EVistDrive Controller Lab - forced-input, source-linked host harness.
 *
 * This file owns ONLY synthetic sensor generation and observation. The control path remains
 * production C: torque_input -> cadence/PAS -> rider_input -> ride_control/assist_modes/limits
 * -> fast_iq_slew. No assist law, limiter, ramp, or Iq conversion is duplicated here.
 */
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FOC.h"
#include "assist_modes.h"
#include "cadence_filter.h"
#include "config.h"
#include "fast_iq_slew.h"
#include "iq_chain.h"
#include "motor_core.h"
#include "pas_cadence.h"
#include "pas_direction.h"
#include "pas_liveness.h"
#include "pas_sampler.h"
#include "power_curve.h"
#include "ride_control.h"
#include "ride_session.h"
#include "rider_input.h"
#include "soc_core.h"
#include "torque_input.h"
#include "tuning_config.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define CTRL_HZ 4000U
#define INNER_PER_CTRL 4U
#define CRANK_LENGTH_M 0.17
#define ASSIST_BANK_HEADER_LEN 13U
#define ASSIST_BANK_RECORD_LEN 48U
#define LAB_ASSIST_LEVEL_COUNT ((ASSIST_BANK_BLOB_LEN - ASSIST_BANK_HEADER_LEN - 2U) / ASSIST_BANK_RECORD_LEN)
#define ASSIST_BANK_CRC_AT (ASSIST_BANK_HEADER_LEN + LAB_ASSIST_LEVEL_COUNT * ASSIST_BANK_RECORD_LEN)
#define DEFAULT_BATTERY_CURRENT_MAX_MA 15000

/* Link-time host globals required by the production FOC objects included in the same native build. */
PI_control_t PI_iq, PI_id;
uint8_t ui_8_PWM_ON_Flag = 1U;
uint8_t bridge_lifecycle = BRIDGE_LIFECYCLE_RUN;
int32_t switchtime[3];
uint16_t pwm_applied[3];
void timer_channel_output_pulse_value_config(uint32_t timer, uint16_t ch, uint32_t value)
{ (void)timer; (void)ch; (void)value; }
void timer_primary_output_config(uint32_t timer, uint32_t enable)
{ (void)timer; (void)enable; }
void runPIcontrol(void) { }

/*
 * Canable-sourced settings arrive as the SAME wire blobs Canable writes to the controller
 * (0x6023/0x6024 global tuning, bank blob per profile). They are handed straight to the
 * production apply functions, so no tuning field is redefined, re-scaled or re-validated
 * here: this file never becomes a second place where a setting means something.
 */
#define LAB_BLOB_HEX_MAX (2U * 255U + 1U)

typedef struct {
    double duration_s;
    double cadence_rpm;
    double cadence_ripple_pct;
    double cadence_ripple_hz;
    double torque_nm;
    double torque_ripple_pct;
    double asymmetry_pct;
    double speed_kph;
    double battery_v;
    double soc_pct;
    int assist_level;
    int mode;              /* -1 = keep whatever the applied bank already says */
    int sample_ms;
    /*
     * START -> STOP scenario. The rider begins pedalling at ride_start_s and stops at
     * ride_stop_s; after that the cranks simply stop turning. Everything that follows -
     * PAS liveness, the release, the Iq ramp down to zero - is production behaviour
     * observed, never scripted here.  ride_stop_s <= 0 means "never stop".
     */
    double ride_start_s;
    double ride_stop_s;
    /*
     * Optional mid-run step of the MEAN crank torque (AP-01 transients-001). At
     * torque_step_at_s the generated mean changes to torque_step_nm; the existing ripple
     * shape and asymmetry are applied unchanged around the new mean. This alters the
     * SYNTHETIC force reading only - no filter output, Iq value or session state is scripted.
     * Both keys must be given together; a lone one is rejected, never silently ignored.
     */
    double torque_step_at_s;
    double torque_step_nm;
    /*
     * Optional SECOND pedalling interval. The rider stops at ride_stop_s and starts again at
     * ride_restart_s; ride_stop2_s (optional) ends the second interval, otherwise the rider
     * pedals to the end of the run. Both intervals are half-open [start, stop). Nothing is
     * reset across the gap: crank angle, filters, PAS, session and ramp keep running as the
     * production path leaves them. Wheel speed stays independent of both intervals.
     */
    double ride_restart_s;
    double ride_stop2_s;
    double reverse_at_s;
    double forward_at_s;
    double reverse_bounce_at_s;
    double torque_invalid_from_s;
    double torque_invalid_to_s;
    double pas_invalid_from_s;
    double pas_invalid_to_s;
    double pas_invalid_seq_from_s;
    double pas_invalid_seq_to_s;
    double fg_delay_at_s;
    /*
     * PAS EDGE disturbances. These are ELECTRICAL: they change what the 4 kHz PAS ISR is shown,
     * while the rider keeps turning the crank. They are a different class from miss_tick /
     * fg_delay, which leave the ISR untouched and only defer the FOREGROUND call.
     *   drop   - every n-th quadrature transition never reaches the ISR (missed edge)
     *   jitter - every transition reaches the ISR late, by an alternating number of ticks
     */
    double pas_edge_drop_from_s;
    double pas_edge_drop_to_s;
    double pas_edge_jitter_from_s;
    double pas_edge_jitter_to_s;
    double miss_tick_from_s;
    double miss_tick_to_s;
    uint32_t pas_edge_drop_every_n;
    uint32_t pas_edge_jitter_ticks;
    uint32_t sample_ticks_override;
    uint32_t miss_tick_every_n;
    uint32_t fg_delay_ticks;
    bool has_torque_step_at;
    bool has_torque_step_nm;
    bool has_ride_restart;
    bool has_ride_stop2;
    bool has_reverse_at;
    bool has_forward_at;
    bool has_reverse_bounce_at;
    bool has_torque_invalid_from;
    bool has_torque_invalid_to;
    bool has_pas_invalid_from;
    bool has_pas_invalid_to;
    bool has_pas_invalid_seq_from;
    bool has_pas_invalid_seq_to;
    bool has_fg_delay_at;
    bool has_fg_delay_ticks;
    bool has_pas_edge_drop_from;
    bool has_pas_edge_drop_to;
    bool has_pas_edge_drop_every_n;
    bool has_pas_edge_jitter_from;
    bool has_pas_edge_jitter_to;
    bool has_pas_edge_jitter_ticks;
    bool has_miss_tick_from;
    bool has_miss_tick_to;
    bool has_sample_ticks;
    char tuning_blob[LAB_BLOB_HEX_MAX];
    char bank_blob[LAB_BLOB_HEX_MAX];
} lab_cfg_t;

typedef struct {
    MotorState_t ms;
    uint32_t tick;
    uint32_t last_fg_tick;
    uint32_t sample_ticks;
    uint32_t elapsed_last;
    uint32_t fg_delay_remaining;
    uint32_t fg_skips_total;
    uint32_t fg_resume_count;
    uint32_t fg_last_resume_tick;
    uint32_t fg_last_resume_elapsed;
    uint32_t reverse_bounce_ticks;
    uint32_t pas_edges_dropped;
    uint32_t pas_edges_deferred;
    int64_t pas_edge_seen_index;
    int64_t pas_edge_change_count;
    uint64_t fast_iq_slew_ticks;
    int64_t pas_transition_index;
    double crank_rev;
    uint8_t crank_direction;
    uint8_t pas_normal_ab;
    uint8_t pas_line_ab;
    uint8_t pas_last_step;
    uint8_t pas_invalid_line_ab;
    uint8_t reverse_bounce_line_ab;
    uint16_t pas_invalid_hold_ticks;
    uint16_t reverse_bounce_hold_ticks;
    uint16_t pas_edge_hold_ticks;
    uint8_t pas_edge_presented_ab;
    uint8_t pas_edge_pending_ab;
    uint16_t last_forward_gap;
    uint16_t stop_timeout;
    uint8_t start_phase;
    bool fg_delay_active;
    bool fg_delay_started;
    bool fg_processed_last;
    bool reverse_bounce_active;
    bool reverse_bounce_done;
    bool reverse_bounce_clear_pending;
    bool pas_invalid_seq_active;
    bool pas_edge_drop_active;
    bool pas_edge_jitter_active;
    bool fg_skipped_prev;
} lab_t;

/* PAS raw ring for PAS_DIR_SIGN=-1: 00 -> 10 -> 11 -> 01 -> 00. */
static const uint8_t FWD_AB[4] = {0U, 2U, 3U, 1U};

static uint8_t fwd_ab_at(int64_t idx)
{
    int mod = (int)(idx % 4);
    if (mod < 0) mod += 4;
    return FWD_AB[(size_t)mod];
}

static double clampd(double x, double lo, double hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static uint16_t crc16_ccitt(const uint8_t *buffer, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)buffer[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void put_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value & 0xFFU);
    p[1] = (uint8_t)(value >> 8);
}

/* hex -> bytes. Returns decoded length, or -1 on malformed input. */
static int hex_to_bytes(const char *hex, uint8_t *out, size_t out_max)
{
    size_t n = strlen(hex);
    if (n == 0U) return 0;
    if (n % 2U) return -1;
    if (n / 2U > out_max) return -1;
    for (size_t i = 0; i < n; i += 2U) {
        int hi = -1, lo = -1;
        for (int k = 0; k < 2; k++) {
            char c = hex[i + (size_t)k];
            int v;
            if (c >= '0' && c <= '9') v = c - '0';
            else if (c >= 'a' && c <= 'f') v = c - 'a' + 10;
            else if (c >= 'A' && c <= 'F') v = c - 'A' + 10;
            else return -1;
            if (k == 0) hi = v; else lo = v;
        }
        out[i / 2U] = (uint8_t)((hi << 4) | lo);
    }
    return (int)(n / 2U);
}

/*
 * Apply Canable's global ride-feel tuning blob through the production owner. The lab does not
 * inspect or reinterpret a single field: tuning_config.c validates version, length and CRC and
 * decides what each byte means, exactly as it does for a blob arriving over CAN.
 */
static bool apply_tuning_blob_hex(const char *hex)
{
    if (!hex || !hex[0]) return true;
    uint8_t blob[TUNING_BLOB_LEN];
    int len = hex_to_bytes(hex, blob, sizeof(blob));
    if (len <= 0) {
        fprintf(stderr, "tuning_blob: malformed hex\n");
        return false;
    }
    if (!tuning_config_apply_blob(blob, (uint16_t)len)) {
        fprintf(stderr, "tuning_blob: rejected by production tuning_config_apply_blob (%d B)\n", len);
        return false;
    }
    return true;
}

/* Same contract for the per-level bank blob (assist modes, the four Iq ramps, limits). */
static bool apply_bank_blob_hex(const char *hex)
{
    if (!hex || !hex[0]) return true;
    uint8_t blob[ASSIST_BANK_BLOB_LEN];
    int len = hex_to_bytes(hex, blob, sizeof(blob));
    if (len <= 0) {
        fprintf(stderr, "bank_blob: malformed hex\n");
        return false;
    }
    if (!assist_modes_apply_bank_blob(blob, (uint16_t)len)) {
        fprintf(stderr, "bank_blob: rejected by production assist_modes_apply_bank_blob (%d B)\n", len);
        return false;
    }
    assist_modes_set_active_bank(0U);
    return true;
}

static bool configure_mode(int assist_level, int mode)
{
    /* mode < 0: keep the mode the applied bank already carries (Canable-sourced run). */
    if (mode < 0) return true;
    if (assist_level < 1 || assist_level > (int)LAB_ASSIST_LEVEL_COUNT) return true;
    if (mode < ASSIST_MODE_POWER_LINEAR || mode > ASSIST_MODE_POWER_CURVE || mode == ASSIST_MODE_EMTB_CUSTOM)
        return false;

    uint8_t blob[ASSIST_BANK_BLOB_LEN];
    uint16_t len = assist_modes_serialize_bank(0U, blob);
    if (len != ASSIST_BANK_BLOB_LEN) return false;
    uint8_t *record = &blob[ASSIST_BANK_HEADER_LEN + (uint16_t)(assist_level - 1) * ASSIST_BANK_RECORD_LEN];
    record[0] = (uint8_t)mode;
    /* POWER_CURVE reuses two fields as exponents. Seed valid defaults instead of interpreting
     * POWER_LINEAR support bytes as gamma values. */
    if (mode == ASSIST_MODE_POWER_CURVE) {
        record[1] = POWER_CURVE_EXP_DEFAULT_X10;
        record[2] = 0U;
        record[9] = POWER_CURVE_EXP_DEFAULT_X10;
    }
    put_u16(&blob[ASSIST_BANK_CRC_AT], crc16_ccitt(blob, ASSIST_BANK_CRC_AT));
    if (!assist_modes_apply_bank_blob(blob, len)) return false;
    assist_modes_set_active_bank(0U);
    return true;
}

static void lab_init(lab_t *s, const lab_cfg_t *cfg)
{
    memset(s, 0, sizeof(*s));
    torque_input_init();
    torque_input_startup_zero(TORQUE_ZERO_TARGET_NATIVE);
    torque_input_set_run_window_deg(tuning_config_assist_torque_run_window_deg());
    assist_modes_init();
    assist_modes_set_active_bank(0U);
    motor_core_init(&s->ms);
    ride_control_init();
    pas_direction_init();
    pas_liveness_init();
    pas_cadence_reset();
    cadence_filter_reset();
    pas_sampler_init(0U);
    s->last_fg_tick = 0U;
    s->sample_ticks = 1U;
    s->last_forward_gap = PAS_STOP_TICKS;
    s->stop_timeout = PAS_STOP_TICKS;
    s->pas_normal_ab = FWD_AB[0];
    s->pas_line_ab = FWD_AB[0];
    s->crank_direction = 1U;
    pas_sampler_isr_tick(s->pas_line_ab, 0U);
}

static void process_pas(lab_t *s, uint8_t ab)
{
    pas_sampler_isr_tick(ab, s->tick);
    pas_step_event_t ev;
    while (pas_sampler_pop(&ev)) {
        int8_t st = ev.step;
        if (st > 0) {
            s->pas_last_step = 1U;
            s->last_forward_gap = ev.gap ? ev.gap : s->last_forward_gap;
            uint32_t x = (uint32_t)s->last_forward_gap * 2U;
            if (x < PAS_STOP_TICKS) x = PAS_STOP_TICKS;
            if (x > PAS_STOP_TICKS_MAX) x = PAS_STOP_TICKS_MAX;
            s->stop_timeout = (uint16_t)x;
            pas_direction_on_step(st);
            torque_input_run_filter_step();
            pas_cadence_step_t cad = pas_cadence_forward_step(ev.tick,
                pas_direction_fwd_run() == 1U ? 1U : 0U);
            if (cad.pulse && cad.measured) {
                s->ms.cadence = cad.rpm;
                s->start_phase = 0U;
                cadence_filter_update(s->ms.cadence);
            }
        } else if (st < 0) {
            s->pas_last_step = 2U;
            pas_cadence_break_epoch(0U);
            pas_direction_on_step(st);
        } else {
            s->pas_last_step = 3U;
            pas_cadence_break_epoch(1U);
            pas_direction_on_step(st);
        }
    }
    if (pas_sampler_take_overflow()) pas_cadence_break_epoch(2U);
    if (s->ms.cadence == 0U && !s->start_phase && pas_direction_fwd_run() >= START_PHASE_STEPS)
        s->start_phase = 1U;
}

static uint8_t pas_from_crank(lab_t *s)
{
    int64_t idx = (int64_t)floor(s->crank_rev * (double)PAS_TRANSITIONS_PER_REV);
    while (idx > s->pas_transition_index) {
        s->pas_transition_index++;
        s->pas_normal_ab = fwd_ab_at(s->pas_transition_index);
    }
    while (idx < s->pas_transition_index) {
        s->pas_transition_index--;
        s->pas_normal_ab = fwd_ab_at(s->pas_transition_index);
    }
    return s->pas_normal_ab;
}

/*
 * Which pedalling interval is time t in? 0 = none, 1 = first, 2 = second (after a restart).
 * Generator-only: no control decision lives here. Both intervals are half-open [start, stop).
 */
static int ride_interval_at(const lab_cfg_t *cfg, double t)
{
    if (t >= cfg->ride_start_s && (cfg->ride_stop_s <= 0.0 || t < cfg->ride_stop_s)) return 1;
    if (cfg->has_ride_restart && t >= cfg->ride_restart_s
        && (!cfg->has_ride_stop2 || t < cfg->ride_stop2_s)) return 2;
    return 0;
}

/* Are the rider's legs turning at time t? Generator-only: no control decision lives here. */
static bool rider_pedalling(const lab_cfg_t *cfg, double t)
{
    return ride_interval_at(cfg, t) != 0;
}

static double generated_cadence(const lab_cfg_t *cfg, double t)
{
    if (!rider_pedalling(cfg, t)) return 0.0;
    double frac = cfg->cadence_ripple_pct / 100.0;
    double cad = cfg->cadence_rpm * (1.0 + frac * sin(2.0 * M_PI * cfg->cadence_ripple_hz * t));
    return clampd(cad, 0.0, 180.0);
}

/*
 * A bounce is ONE short electrical excursion, not a periodic one.
 *
 * Without reverse_bounce_done this re-armed on the very next tick after the 3-tick hold expired,
 * because `t >= reverse_bounce_at_s` stays true for the rest of the run: the scenario declared a
 * 3-tick event and the generator actually produced a permanent 3-tick-periodic disturbance. The
 * assist then never recovered, which looks like a firmware finding and is not one.
 */
static void start_reverse_bounce(lab_t *s, const lab_cfg_t *cfg, double t)
{
    if (s->reverse_bounce_active || s->reverse_bounce_done
        || !cfg->has_reverse_bounce_at || t < cfg->reverse_bounce_at_s)
        return;
    s->reverse_bounce_active = true;
    s->reverse_bounce_hold_ticks = 3U;
    s->reverse_bounce_line_ab = fwd_ab_at(s->pas_transition_index - 1);
}

/*
 * Which way the RIDER'S CRANK is turning. reverse_bounce_at_s is deliberately absent here.
 *
 * REVIEW-EVD-AP-01-011 (D011-01): this used to return -1 while reverse_bounce_active, so the
 * "electrical bounce" also drove crank_rev backwards (measured: 72.00 -> 71.89 -> 71.78 ->
 * 71.68 deg at 72 rpm) and, because generated_torque() takes its phase from crank_rev, it moved
 * the forced pressure phase too. A bounce is a LINE artefact: the rider keeps pedalling forward
 * at constant cadence and only the quadrature the ISR is shown misbehaves. Sustained real
 * backpedalling is a different scenario key (reverse_at_s), and it keeps its reverse motion.
 */
static int crank_direction_at(lab_t *s, const lab_cfg_t *cfg, double t)
{
    if (!rider_pedalling(cfg, t)) return 0;
    if (cfg->has_reverse_at && t >= cfg->reverse_at_s
        && (!cfg->has_forward_at || t < cfg->forward_at_s)) return -1;
    return 1;
}

/*
 * ELECTRICAL PAS edge disturbance, applied to the line the 4 kHz ISR is shown.
 *
 * This is deliberately NOT the same thing as miss_tick_every_n / fg_delay_*: those leave the PAS
 * ISR running normally and only defer the FOREGROUND call. Here the crank keeps turning and the
 * quadrature index keeps advancing, but an edge is either never presented (drop = missed edge) or
 * presented late (jitter). A dropped edge makes the NEXT presented value two steps away from the
 * previous one, which is an illegal quadrature step for the sampler - that is the point.
 *
 * Outside both windows the line follows the crank with no shaping at all, so the control case and
 * the recovery after the window are the unmodified generator.
 */
static uint8_t pas_edge_shape(lab_t *s, const lab_cfg_t *cfg, double t)
{
    bool drop_win = cfg->has_pas_edge_drop_from
        && t >= cfg->pas_edge_drop_from_s && t < cfg->pas_edge_drop_to_s;
    bool jitter_win = cfg->has_pas_edge_jitter_from
        && t >= cfg->pas_edge_jitter_from_s && t < cfg->pas_edge_jitter_to_s;
    s->pas_edge_drop_active = drop_win;
    s->pas_edge_jitter_active = jitter_win;

    if (!drop_win && !jitter_win) {
        s->pas_edge_hold_ticks = 0U;
        s->pas_edge_presented_ab = s->pas_normal_ab;
        s->pas_edge_pending_ab = s->pas_normal_ab;
        s->pas_edge_seen_index = s->pas_transition_index;
        return s->pas_edge_presented_ab;
    }

    if (s->pas_edge_hold_ticks > 0U) {
        /* A deferred edge keeps catching up with the crank until it is finally presented. */
        s->pas_edge_seen_index = s->pas_transition_index;
        s->pas_edge_pending_ab = s->pas_normal_ab;
        if (--s->pas_edge_hold_ticks == 0U) s->pas_edge_presented_ab = s->pas_edge_pending_ab;
        return s->pas_edge_presented_ab;
    }

    if (s->pas_transition_index != s->pas_edge_seen_index) {
        s->pas_edge_change_count++;
        s->pas_edge_seen_index = s->pas_transition_index;
        if (drop_win && cfg->pas_edge_drop_every_n != 0U
            && (s->pas_edge_change_count % (int64_t)cfg->pas_edge_drop_every_n) == 0) {
            s->pas_edges_dropped++;
            return s->pas_edge_presented_ab;   /* this edge never reaches the ISR */
        }
        if (jitter_win && cfg->pas_edge_jitter_ticks > 0U) {
            /* Alternating deferral, so the ISR sees uneven edge spacing, not a constant lag. */
            uint32_t d = (s->pas_edge_change_count & 1)
                ? cfg->pas_edge_jitter_ticks
                : (cfg->pas_edge_jitter_ticks + 1U) / 2U;
            if (d > 0U) {
                s->pas_edge_hold_ticks = (uint16_t)d;
                s->pas_edge_pending_ab = s->pas_normal_ab;
                s->pas_edges_deferred++;
                return s->pas_edge_presented_ab;
            }
        }
        s->pas_edge_presented_ab = s->pas_normal_ab;
    }
    return s->pas_edge_presented_ab;
}

static void update_pas_line(lab_t *s, const lab_cfg_t *cfg, double t)
{
    if (s->reverse_bounce_active) {
        /* Bounce and invalid-sequence own the line outright; keep the edge shaper in step so it
         * does not invent a phantom transition when the override ends. validate_events() forbids
         * combining them with the edge disturbances, so the classes stay separable. */
        s->pas_edge_seen_index = s->pas_transition_index;
        s->pas_edge_presented_ab = s->pas_normal_ab;
        s->pas_edge_pending_ab = s->pas_normal_ab;
        s->pas_line_ab = s->reverse_bounce_line_ab;
        s->reverse_bounce_ticks++;
        /*
         * Clearing `active` here would clear it on the LAST forced tick, before that tick is
         * emitted - the CSV then showed reverse_bounce_active=0 on a row whose line was still
         * forced (REVIEW-EVD-AP-01-011, D011-01). The clear is deferred to the start of the
         * next tick, so the flag marks exactly the ticks that were disturbed.
         */
        if (--s->reverse_bounce_hold_ticks == 0U) s->reverse_bounce_clear_pending = true;
        return;
    }
    if (!s->pas_invalid_seq_active && cfg->has_pas_invalid_seq_from && t >= cfg->pas_invalid_seq_from_s) {
        s->pas_invalid_seq_active = true;
        s->pas_invalid_hold_ticks = PAS_SAMPLER_GLITCH_TICKS;
        s->pas_invalid_line_ab = (uint8_t)(s->pas_normal_ab ^ 3U);
    }
    if (s->pas_invalid_seq_active) {
        if (t >= cfg->pas_invalid_seq_to_s) {
            s->pas_invalid_seq_active = false;
            s->pas_line_ab = s->pas_normal_ab;
        } else {
            s->pas_edge_seen_index = s->pas_transition_index;
            s->pas_edge_presented_ab = s->pas_normal_ab;
            s->pas_edge_pending_ab = s->pas_normal_ab;
            s->pas_line_ab = s->pas_invalid_line_ab;
            if (--s->pas_invalid_hold_ticks == 0U) {
                s->pas_invalid_line_ab = (uint8_t)(s->pas_invalid_line_ab ^ 3U);
                s->pas_invalid_hold_ticks = PAS_SAMPLER_GLITCH_TICKS;
            }
        }
        return;
    }
    s->pas_line_ab = pas_edge_shape(s, cfg, t);
}

static bool torque_invalid_at(const lab_cfg_t *cfg, double t)
{
    return cfg->has_torque_invalid_from && t >= cfg->torque_invalid_from_s && t < cfg->torque_invalid_to_s;
}

static bool pas_invalid_at(const lab_cfg_t *cfg, double t)
{
    return cfg->has_pas_invalid_from && t >= cfg->pas_invalid_from_s && t < cfg->pas_invalid_to_s;
}

/* Commanded MEAN crank torque at time t - the only thing the optional step changes. */
static double torque_mean_at(const lab_cfg_t *cfg, double t)
{
    if (cfg->has_torque_step_at && t >= cfg->torque_step_at_s) return cfg->torque_step_nm;
    return cfg->torque_nm;
}

static double generated_torque(const lab_cfg_t *cfg, double crank_rev, double t)
{
    double mean_nm = torque_mean_at(cfg, t);
    if (mean_nm <= 0.0) return 0.0;
    double phase = 2.0 * M_PI * crank_rev;
    double ripple = cfg->torque_ripple_pct / 100.0;
    double asym = cfg->asymmetry_pct / 100.0;
    double shape = 1.0 + ripple * sin(2.0 * phase) + asym * sin(phase);
    if (shape < 0.0) shape = 0.0;
    return clampd(mean_nm * shape, 0.0, 180.0);
}

static uint16_t torque_raw_from_nm(double torque_nm)
{
    double centikg = torque_nm / (9.80665 * CRANK_LENGTH_M) * 100.0;
    centikg = clampd(centikg, 0.0, (double)TORQUE_INPUT_MAX_CENTIKG);
    uint16_t delta = torque_input_centikg_to_native_delta((uint16_t)llround(centikg));
    uint32_t raw = (uint32_t)TORQUE_ZERO_TARGET_NATIVE + delta;
    if (raw > 65535U) raw = 65535U;
    return (uint16_t)raw;
}

static const char *mode_name(int mode)
{
    if (mode < 0) return "canable-bank";
    switch (mode) {
    case ASSIST_MODE_POWER_LINEAR: return "power_linear";
    case ASSIST_MODE_POWER_PROGRESSIVE: return "power_progressive";
    case ASSIST_MODE_EMTB: return "emtb";
    case ASSIST_MODE_TORQUE: return "torque";
    case ASSIST_MODE_POWER_CURVE: return "power_curve";
    default: return "unknown";
    }
}

static void print_header(void)
{
    puts("time_s,crank_angle_deg,torque_gen_nm,torque_load_kg,torque_fast_native,torque_run_native,cadence_gen_rpm,cadence_raw_rpm,cadence_control_rpm,speed_kph,battery_v,soc_pct,human_power_w,assist_basis_power_w,motor_power_w,support_pct,iq_mode_request,iq_before_profile_limit,iq_requested,iq_allowed,iq_pre_ramp,iq_ref,session,debug_flags,battery_limit,gate_steps,gate_load_ckg,pedalling,torque_cmd_mean_nm,ride_interval,crank_direction,pas_ab,pas_normal_ab,pas_transition_index,pas_last_step,pas_direction_state,pas_inhibit_reason,pas_fwd_run,pas_rev_run,pas_backpedal_confirmed,pas_sampler_forward,pas_sampler_reverse,pas_sampler_invalid,pas_sampler_glitch,pas_sampler_overflow,torque_sensor_valid,pas_sensor_valid,rider_torque_sensor_valid,rider_pas_sensor_valid,elapsed_ticks,fg_processed,fg_delay_active,reverse_bounce_active,pas_invalid_seq_active,fast_iq_slew_ticks,fg_skips_total,fg_resume_count,fg_last_resume_tick,fg_last_resume_elapsed,pas_edge_drop_active,pas_edge_jitter_active,pas_edges_dropped,pas_edges_deferred,reverse_bounce_ticks");
}

static void lab_tick(lab_t *s, const lab_cfg_t *cfg, bool emit)
{
    const double dt = 1.0 / (double)CTRL_HZ;
    s->tick++;
    double t = (double)s->tick / (double)CTRL_HZ;

    if (s->reverse_bounce_clear_pending) {
        s->reverse_bounce_active = false;
        s->reverse_bounce_done = true;          /* fires exactly once per run */
        s->reverse_bounce_clear_pending = false;
    }
    start_reverse_bounce(s, cfg, t);
    int direction = crank_direction_at(s, cfg, t);
    s->crank_direction = (uint8_t)(direction + 1);

    double cadence_gen = generated_cadence(cfg, t);
    s->crank_rev += direction * cadence_gen / 60.0 * dt;

    uint8_t normal = pas_from_crank(s);
    update_pas_line(s, cfg, t);
    pas_sampler_isr_tick(s->pas_line_ab, s->tick);

    if (!s->fg_delay_started && cfg->has_fg_delay_at && t >= cfg->fg_delay_at_s) {
        s->fg_delay_active = true;
        s->fg_delay_remaining = cfg->fg_delay_ticks;
        s->fg_delay_started = true;
    }

    /*
     * miss_tick is bounded by an explicit window so a scenario has a real control phase before
     * the injection and a real recovery phase after it. With no window given the behaviour is
     * the historic one (from the first tick to the end of the run).
     */
    bool miss_win = (!cfg->has_miss_tick_from || t >= cfg->miss_tick_from_s)
                 && (!cfg->has_miss_tick_to || t < cfg->miss_tick_to_s);
    bool miss = cfg->miss_tick_every_n != 0U && miss_win
             && (s->tick % cfg->miss_tick_every_n) == 0U;
    bool skip = s->fg_delay_active || miss;

    const torque_snapshot_t *ts = torque_input_get_snapshot();
    uint32_t elapsed = 0U;

    if (skip) {
        if (s->fg_delay_active && s->fg_delay_remaining > 0U) {
            s->fg_delay_remaining--;
        }
        if (s->fg_delay_active && s->fg_delay_remaining == 0U) {
            s->fg_delay_active = false;
        }
        s->fg_processed_last = false;
        s->elapsed_last = 0U;
        s->fg_skips_total++;
        s->fg_skipped_prev = true;
    } else {
        elapsed = s->tick - s->last_fg_tick;
        if (elapsed == 0U) elapsed = 1U;
        /*
         * Latched resume diagnostics. The CSV is decimated by sample_ms/sample_ticks, so a resume
         * row can fall between two emitted samples. These counters are cumulative/sticky and
         * therefore survive decimation - a test does not have to catch the exact row.
         */
        if (s->fg_skipped_prev) {
            s->fg_resume_count++;
            s->fg_last_resume_tick = s->tick;
            s->fg_last_resume_elapsed = elapsed;
            s->fg_skipped_prev = false;
        }

        process_pas(s, normal);

        uint32_t idle = s->tick - pas_sampler_last_transition_tick();
        pas_liveness_update(idle, s->stop_timeout);
        bool real_stop = pas_liveness_stopped();
        if (real_stop) {
            s->ms.cadence = 0U;
            s->start_phase = 0U;
            cadence_filter_reset();
            pas_cadence_reset();
            pas_direction_on_stop();
        }
        bool direction_ok = (s->ms.cadence > 0U || s->start_phase) && !real_stop;
        bool pedaling = direction_ok && pas_direction_fwd_run() >= tuning_config_start_steps();
        uint8_t control_cadence = cadence_filter_get();

        int ride_interval = ride_interval_at(cfg, t);
        double torque_nm = ride_interval ? generated_torque(cfg, s->crank_rev, t) : 0.0;
        uint16_t raw = torque_raw_from_nm(torque_nm);
        int16_t corrected = torque_input_correct(raw);

        bool torque_valid = !torque_invalid_at(cfg, t);
        bool pas_valid = !pas_invalid_at(cfg, t);

        torque_input_update_elapsed(raw, corrected, torque_valid, elapsed);

        rider_input_t r;
        memset(&r, 0, sizeof(r));
        r.torque_raw_mv = raw;
        r.torque_corrected_mv = corrected;
        r.torque_filtered = ts->delta_native;
        r.torque_assist_now_native = ts->assist_delta_native;
        r.torque_assist_filtered = ts->assist_delta_filtered_native;
        r.torque_run_filtered = ts->assist_delta_run_native;
        r.torque_load_ctrl = ts->load_ctrl;
        r.torque_load_centikg = ts->load_centikg;
        r.cadence_rpm = control_cadence;
        r.wheel_speed_x100 = (uint32_t)llround(clampd(cfg->speed_kph, 0.0, 100.0) * 100.0);
        r.motor_erps = (uint16_t)llround(clampd(cadence_gen * 4.0 / 3.0, 0.0, 65535.0));
        r.motor_erps_age_ticks = cadence_gen > 0.1 ? 0U : 0xFFFFU;
        r.motor_voltage_utilization = 0U;
        r.pas_forward = pedaling;
        r.pedaling_active = pedaling;
        r.crank_forward_steps = pas_direction_fwd_run();
        r.crank_direction_ok = direction_ok;
        r.real_stop = real_stop;
        r.wheel_valid = true;
        r.direction_inhibit_active = pas_direction_direction_inhibit_active();
        r.forward_confirmed_this_tick = pas_direction_forward_confirmed_last_call();
        r.sample_tick = s->tick;
        r.start_phase = s->start_phase != 0U;
        r.torque_sensor_valid = torque_valid;
        r.pas_sensor_valid = pas_valid;
        r.pas_backward = pas_direction_backpedal_confirmed();
        rider_input_update(&r);

        float limp = soc_core_limp_factor((float)clampd(cfg->soc_pct, 0.0, 100.0), 10U, 5U);
        int32_t iq_limit = (int32_t)lroundf((float)PH_CURRENT_MAX * limp);
        uint16_t voltage_raw = (uint16_t)llround(cfg->battery_v * 1000.0 / (double)CAL_BAT_V);

        ride_control_input_t in;
        memset(&in, 0, sizeof(in));
        in.speed_x100 = r.wheel_speed_x100;
        in.cadence_rpm = control_cadence;
        in.assist_level_index = (uint8_t)cfg->assist_level;
        in.battery_voltage_mv = (uint32_t)llround(cfg->battery_v * 1000.0);
        in.iq_scale = PH_CURRENT_MAX;
        in.ride_core_iq_limit = iq_limit;
        in.phase_current_max = iq_limit;
        in.battery_current_mA = 0;
        in.battery_current_max = DEFAULT_BATTERY_CURRENT_MAX_MA;
        in.u_abs = 0;
        in.cal_i = CAL_I;
        in.current_iq = s->ms.i_q;
        in.current_id = s->ms.i_d;
        in.voltage_raw = voltage_raw;
        in.voltage_min_raw = VOLTAGE_MIN;
        in.controller_temperature_c = 25;
        in.cadence_filtered_x8 = cadence_filter_get_x8();
        in.speed_limit_x100 = SPEEDLIMIT;
        in.legal_enabled = true;
        in.offroad = false;
        in.walk_active = false;
        in.safety_cut_non_direction = false;
        in.service_cut_active = false;
        in.elapsed_ticks = elapsed;
        ride_control_update(&in);

        s->last_fg_tick = s->tick;
        s->elapsed_last = elapsed;
        s->fg_processed_last = true;
        if (s->fg_delay_active && s->fg_delay_remaining == 0U) {
            s->fg_delay_active = false;
        }
    }

    for (unsigned k = 0; k < INNER_PER_CTRL; k++) {
        fast_iq_slew_tick(ride_control_final_iq_slew_mailbox(), &s->ms.i_q_setpoint);
    }
    s->fast_iq_slew_ticks += INNER_PER_CTRL;

    if (emit) {
        const assist_mode_output_t *mo = assist_modes_get_last_output();
        const iq_chain_t *chain = iq_chain_get();
        ride_arm_snapshot_t arm;
        ride_gate_snapshot_t gate;
        ride_control_get_arm_snapshot(&arm);
        ride_control_get_gate_snapshot(&gate);

        const rider_input_t *ri = rider_input_get();
        const pas_sampler_stats_t *ps = pas_sampler_get_stats();

        double angle = fmod(s->crank_rev * 360.0, 360.0);
        if (angle < 0.0) angle += 360.0;

        uint32_t out_elapsed = s->elapsed_last;
        bool out_fg = s->fg_processed_last;
        bool torque_cmd_valid = !torque_invalid_at(cfg, t);
        bool pas_cmd_valid = !pas_invalid_at(cfg, t);
        bool rider_tq_valid = ri && ri->torque_sensor_valid;
        bool rider_pas_valid = ri && ri->pas_sensor_valid;

        printf(
            "%.6f,%.2f,%.3f,%.3f,"          /* 1-4: t, angle, torque, load */
            "%u,%u,%.3f,%u,%u,"             /* 5-9: assist_delta_filtered, assist_delta_run, cadence_gen, cadence_raw, cadence_control */
            "%.3f,%.3f,%.3f,"               /* 10-12: speed, battery, soc */
            "%u,%u,%u,%u,"                  /* 13-16: human, assist_basis, motor, support */
            "%d,%d,%d,%d,%d,%d,"            /* 17-22: iq_request, iq_before_pu, chain_req, chain_all, arm_pre, iq_setpoint */
            "%u,%u,%u,"                     /* 23-25: session, debug, battery_limit */
            "%u,%u,"                        /* 26-27: gate_steps, gate_load */
            "%u,"                           /* 28: pedalling */
            "%.3f,"                         /* 29: torque_mean */
            "%d,"                           /* 30: ride_interval (0=none, 1=first, 2=after restart) */
            "%d,"                           /* 31: crank_direction */
            "%u,%u,%lld,%u,"                /* 32-35: pas_ab, pas_normal_ab, pas_transition_index, pas_last_step */
            "%u,%u,%u,%u,%u,"               /* 36-40: pas_dir_state, pas_inhibit_reason, pas_fwd, pas_rev, pas_backpedal */
            "%u,%u,%u,%u,%u,"               /* 41-45: sampler fwd, rev, inv, glitch, overflow */
            "%u,%u,%u,%u,"                  /* 46-49: torque_cmd_valid, pas_cmd_valid, rider_tq, rider_pas */
            "%u,%u,%u,%u,%u,"               /* 50-54: out_elapsed, out_fg, fg_delay, rev_bounce, pas_inv_seq */
            "%llu,"                         /* 55: fast_iq_slew_ticks */
            "%u,%u,%u,%u,"                  /* 56-59: fg skips, resumes, last resume tick/elapsed */
            "%u,%u,%u,%u,"                  /* 60-63: pas edge drop/jitter, dropped, deferred */
            "%u\n",                        /* 64: reverse_bounce_ticks (cumulative forced ticks) */
            t, angle,
            (ride_interval_at(cfg, t) ? generated_torque(cfg, s->crank_rev, t) : 0.0),
            (double)ts->load_centikg / 100.0,
            ts->assist_delta_filtered_native, ts->assist_delta_run_native,
            cadence_gen, s->ms.cadence, cadence_filter_get(), cfg->speed_kph, cfg->battery_v, cfg->soc_pct,
            mo->human_power_w, mo->assist_basis_power_w, mo->motor_power_w, mo->applied_support_ratio_pct,
            mo->iq_request, mo->iq_before_pu,
            chain->valid ? chain->requested : 0, chain->valid ? chain->allowed : 0,
            arm.iq_pre_ramp, (int)s->ms.i_q_setpoint,
            (unsigned)ride_control_get_session_state(), (unsigned)ride_control_get_debug_flags(),
            ride_control_battery_limit_active() ? 1U : 0U,
            (unsigned)gate.required_steps, (unsigned)gate.load_threshold_centikg,
            ride_interval_at(cfg, t) ? 1U : 0U,
            torque_mean_at(cfg, t),
            ride_interval_at(cfg, t),
            (int)s->crank_direction - 1,
            s->pas_line_ab, s->pas_normal_ab, (long long)s->pas_transition_index, s->pas_last_step,
            (unsigned)pas_direction_get_state(),
            (unsigned)pas_direction_last_inhibit_reason(),
            pas_direction_fwd_run(), pas_direction_rev_run(),
            pas_direction_backpedal_confirmed() ? 1U : 0U,
            ps->forward_count, ps->reverse_count, ps->invalid_count, ps->glitch_count, ps->overflow_count,
            torque_cmd_valid ? 1U : 0U, pas_cmd_valid ? 1U : 0U,
            rider_tq_valid ? 1U : 0U, rider_pas_valid ? 1U : 0U,
            out_elapsed, out_fg ? 1U : 0U,
            s->fg_delay_active ? 1U : 0U,
            s->reverse_bounce_active ? 1U : 0U,
            s->pas_invalid_seq_active ? 1U : 0U,
            (unsigned long long)s->fast_iq_slew_ticks,
            s->fg_skips_total, s->fg_resume_count,
            s->fg_last_resume_tick, s->fg_last_resume_elapsed,
            s->pas_edge_drop_active ? 1U : 0U, s->pas_edge_jitter_active ? 1U : 0U,
            s->pas_edges_dropped, s->pas_edges_deferred,
            s->reverse_bounce_ticks);
    }
}

static int parse_mode(const char *s)
{
    /* "keep" = do not override the mode carried by the applied (Canable) bank. */
    if (!strcmp(s, "keep") || !strcmp(s, "canable")) return -1;
    if (!strcmp(s, "power") || !strcmp(s, "power_linear")) return ASSIST_MODE_POWER_LINEAR;
    if (!strcmp(s, "progressive") || !strcmp(s, "power_progressive")) return ASSIST_MODE_POWER_PROGRESSIVE;
    if (!strcmp(s, "emtb")) return ASSIST_MODE_EMTB;
    if (!strcmp(s, "torque")) return ASSIST_MODE_TORQUE;
    if (!strcmp(s, "curve") || !strcmp(s, "power_curve")) return ASSIST_MODE_POWER_CURVE;
    return atoi(s);
}

/*
 * Strict numeric parse for the NEW event keys only. The pre-existing keys keep their historic
 * atof()/atoi() behaviour on purpose: this task extends the generator, it does not rebuild the
 * old parser. NaN, inf, trailing garbage and an empty value are rejected here.
 */
static int parse_double_strict(const char *v, double *out)
{
    char *end = NULL;
    errno = 0;
    double d = strtod(v, &end);
    if (end == v || *end != '\0') return -1;
    if (!isfinite(d)) return -1;
    *out = d;
    return 0;
}

static int parse_u32_strict(const char *v, uint32_t *out)
{
    char *end = NULL;
    errno = 0;
    unsigned long long x = strtoull(v, &end, 10);
    if (end == v || *end != '\0') return -1;
    if (errno == ERANGE || x > 0xFFFFFFFFULL) return -1;
    *out = (uint32_t)x;
    return 0;
}

/* Returns 0 on success, -1 when a NEW key carries an unusable value. */
static int parse_arg(lab_cfg_t *c, const char *arg)
{
    const char *eq = strchr(arg, '=');
    if (!eq) return 0;
    size_t n = (size_t)(eq - arg);
    const char *v = eq + 1;
#define KEY(name) (n == strlen(name) && strncmp(arg, name, n) == 0)
    if (KEY("duration")) c->duration_s = atof(v);
    else if (KEY("cadence")) c->cadence_rpm = atof(v);
    else if (KEY("cadence_ripple")) c->cadence_ripple_pct = atof(v);
    else if (KEY("cadence_ripple_hz")) c->cadence_ripple_hz = atof(v);
    else if (KEY("torque")) c->torque_nm = atof(v);
    else if (KEY("torque_ripple")) c->torque_ripple_pct = atof(v);
    else if (KEY("asymmetry")) c->asymmetry_pct = atof(v);
    else if (KEY("speed")) c->speed_kph = atof(v);
    else if (KEY("voltage")) c->battery_v = atof(v);
    else if (KEY("soc")) c->soc_pct = atof(v);
    else if (KEY("assist")) c->assist_level = atoi(v);
    else if (KEY("mode")) c->mode = parse_mode(v);
    else if (KEY("sample_ms")) c->sample_ms = atoi(v);
    else if (KEY("ride_start_s")) c->ride_start_s = atof(v);
    else if (KEY("ride_stop_s")) c->ride_stop_s = atof(v);
    else if (KEY("torque_step_at_s")) {
        if (parse_double_strict(v, &c->torque_step_at_s) != 0) return -1;
        c->has_torque_step_at = true;
    }
    else if (KEY("torque_step_nm")) {
        if (parse_double_strict(v, &c->torque_step_nm) != 0) return -1;
        c->has_torque_step_nm = true;
    }
    else if (KEY("ride_restart_s")) {
        if (parse_double_strict(v, &c->ride_restart_s) != 0) return -1;
        c->has_ride_restart = true;
    }
    else if (KEY("ride_stop2_s")) {
        if (parse_double_strict(v, &c->ride_stop2_s) != 0) return -1;
        c->has_ride_stop2 = true;
    }
    else if (KEY("tuning_blob")) {
        strncpy(c->tuning_blob, v, sizeof(c->tuning_blob) - 1U);
        c->tuning_blob[sizeof(c->tuning_blob) - 1U] = '\0';
    }
    else if (KEY("bank_blob")) {
        strncpy(c->bank_blob, v, sizeof(c->bank_blob) - 1U);
        c->bank_blob[sizeof(c->bank_blob) - 1U] = '\0';
    }
    else if (KEY("reverse_at_s")) {
        if (parse_double_strict(v, &c->reverse_at_s) != 0) return -1;
        c->has_reverse_at = true;
    }
    else if (KEY("forward_at_s")) {
        if (parse_double_strict(v, &c->forward_at_s) != 0) return -1;
        c->has_forward_at = true;
    }
    else if (KEY("reverse_bounce_at_s")) {
        if (parse_double_strict(v, &c->reverse_bounce_at_s) != 0) return -1;
        c->has_reverse_bounce_at = true;
    }
    else if (KEY("torque_invalid_from_s")) {
        if (parse_double_strict(v, &c->torque_invalid_from_s) != 0) return -1;
        c->has_torque_invalid_from = true;
    }
    else if (KEY("torque_invalid_to_s")) {
        if (parse_double_strict(v, &c->torque_invalid_to_s) != 0) return -1;
        c->has_torque_invalid_to = true;
    }
    else if (KEY("pas_invalid_from_s")) {
        if (parse_double_strict(v, &c->pas_invalid_from_s) != 0) return -1;
        c->has_pas_invalid_from = true;
    }
    else if (KEY("pas_invalid_to_s")) {
        if (parse_double_strict(v, &c->pas_invalid_to_s) != 0) return -1;
        c->has_pas_invalid_to = true;
    }
    else if (KEY("pas_invalid_seq_from_s")) {
        if (parse_double_strict(v, &c->pas_invalid_seq_from_s) != 0) return -1;
        c->has_pas_invalid_seq_from = true;
    }
    else if (KEY("pas_invalid_seq_to_s")) {
        if (parse_double_strict(v, &c->pas_invalid_seq_to_s) != 0) return -1;
        c->has_pas_invalid_seq_to = true;
    }
    else if (KEY("miss_tick_every_n")) {
        if (parse_u32_strict(v, &c->miss_tick_every_n) != 0) return -1;
    }
    else if (KEY("fg_delay_ticks")) {
        if (parse_u32_strict(v, &c->fg_delay_ticks) != 0) return -1;
        c->has_fg_delay_ticks = true;
    }
    else if (KEY("fg_delay_at_s")) {
        if (parse_double_strict(v, &c->fg_delay_at_s) != 0) return -1;
        c->has_fg_delay_at = true;
    }
    else if (KEY("miss_tick_from_s")) {
        if (parse_double_strict(v, &c->miss_tick_from_s) != 0) return -1;
        c->has_miss_tick_from = true;
    }
    else if (KEY("miss_tick_to_s")) {
        if (parse_double_strict(v, &c->miss_tick_to_s) != 0) return -1;
        c->has_miss_tick_to = true;
    }
    else if (KEY("pas_edge_drop_from_s")) {
        if (parse_double_strict(v, &c->pas_edge_drop_from_s) != 0) return -1;
        c->has_pas_edge_drop_from = true;
    }
    else if (KEY("pas_edge_drop_to_s")) {
        if (parse_double_strict(v, &c->pas_edge_drop_to_s) != 0) return -1;
        c->has_pas_edge_drop_to = true;
    }
    else if (KEY("pas_edge_drop_every_n")) {
        if (parse_u32_strict(v, &c->pas_edge_drop_every_n) != 0) return -1;
        c->has_pas_edge_drop_every_n = true;
    }
    else if (KEY("pas_edge_jitter_from_s")) {
        if (parse_double_strict(v, &c->pas_edge_jitter_from_s) != 0) return -1;
        c->has_pas_edge_jitter_from = true;
    }
    else if (KEY("pas_edge_jitter_to_s")) {
        if (parse_double_strict(v, &c->pas_edge_jitter_to_s) != 0) return -1;
        c->has_pas_edge_jitter_to = true;
    }
    else if (KEY("pas_edge_jitter_ticks")) {
        if (parse_u32_strict(v, &c->pas_edge_jitter_ticks) != 0) return -1;
        c->has_pas_edge_jitter_ticks = true;
    }
    else if (KEY("sample_ticks")) {
        if (parse_u32_strict(v, &c->sample_ticks_override) != 0) return -1;
        c->has_sample_ticks = true;
    }
#undef KEY
    return 0;
}

/*
 * Validate the new event keys AFTER the existing clamping, so a rejection is decided against the
 * duration the run will actually use. A contradictory scenario is an error, never silently turned
 * into a different (working) scenario.
 */
static bool validate_events(const lab_cfg_t *c)
{
    if (c->has_torque_step_at != c->has_torque_step_nm) {
        fprintf(stderr, "torque step: torque_step_at_s and torque_step_nm must be given together\n");
        return false;
    }
    if (c->has_torque_step_at) {
        if (!(c->torque_step_at_s > 0.0) || c->torque_step_at_s > c->duration_s) {
            fprintf(stderr, "torque step: torque_step_at_s=%g must satisfy 0 < t <= duration=%g\n",
                c->torque_step_at_s, c->duration_s);
            return false;
        }
        if (c->torque_step_nm < 0.0 || c->torque_step_nm > 180.0) {
            fprintf(stderr, "torque step: torque_step_nm=%g out of range [0,180]\n", c->torque_step_nm);
            return false;
        }
    }
    if (c->has_ride_stop2 && !c->has_ride_restart) {
        fprintf(stderr, "ride intervals: ride_stop2_s requires ride_restart_s\n");
        return false;
    }
    if (c->has_ride_restart) {
        if (!(c->ride_stop_s > 0.0)) {
            fprintf(stderr, "ride intervals: ride_restart_s requires a first stop (ride_stop_s > 0)\n");
            return false;
        }
        if (!(c->ride_restart_s > c->ride_stop_s)) {
            fprintf(stderr, "ride intervals: ride_restart_s=%g must be > ride_stop_s=%g\n",
                c->ride_restart_s, c->ride_stop_s);
            return false;
        }
        if (c->ride_restart_s > c->duration_s) {
            fprintf(stderr, "ride intervals: ride_restart_s=%g exceeds duration=%g\n",
                c->ride_restart_s, c->duration_s);
            return false;
        }
    }
    if (c->has_ride_stop2) {
        if (!(c->ride_stop2_s > c->ride_restart_s)) {
            fprintf(stderr, "ride intervals: ride_stop2_s=%g must be > ride_restart_s=%g\n",
                c->ride_stop2_s, c->ride_restart_s);
            return false;
        }
        if (c->ride_stop2_s > c->duration_s) {
            fprintf(stderr, "ride intervals: ride_stop2_s=%g exceeds duration=%g\n",
                c->ride_stop2_s, c->duration_s);
            return false;
        }
    }

    if (c->has_reverse_at) {
        if (c->reverse_at_s < 0.0 || c->reverse_at_s > c->duration_s) {
            fprintf(stderr, "reverse_at_s=%g out of range [0, duration=%g]\n", c->reverse_at_s, c->duration_s);
            return false;
        }
    }
    if (c->has_forward_at) {
        if (!c->has_reverse_at) {
            fprintf(stderr, "forward_at_s requires reverse_at_s\n");
            return false;
        }
        if (c->forward_at_s <= c->reverse_at_s || c->forward_at_s > c->duration_s) {
            fprintf(stderr, "forward_at_s=%g must be > reverse_at_s and <= duration=%g\n", c->forward_at_s, c->duration_s);
            return false;
        }
    }
    if (c->has_reverse_bounce_at) {
        if (c->reverse_bounce_at_s < 0.0 || c->reverse_bounce_at_s > c->duration_s) {
            fprintf(stderr, "reverse_bounce_at_s=%g out of range [0, duration=%g]\n", c->reverse_bounce_at_s, c->duration_s);
            return false;
        }
        if (c->has_reverse_at || c->has_forward_at) {
            fprintf(stderr, "reverse_bounce_at_s cannot be used together with reverse_at_s/forward_at_s\n");
            return false;
        }
    }

    if (c->has_torque_invalid_from != c->has_torque_invalid_to) {
        fprintf(stderr, "torque invalid window: torque_invalid_from_s and torque_invalid_to_s must be given together\n");
        return false;
    }
    if (c->has_torque_invalid_from) {
        if (c->torque_invalid_from_s < 0.0 || c->torque_invalid_to_s <= c->torque_invalid_from_s || c->torque_invalid_to_s > c->duration_s) {
            fprintf(stderr, "torque invalid window: must satisfy 0 <= from < to <= duration=%g\n", c->duration_s);
            return false;
        }
    }

    if (c->has_pas_invalid_from != c->has_pas_invalid_to) {
        fprintf(stderr, "pas invalid window: pas_invalid_from_s and pas_invalid_to_s must be given together\n");
        return false;
    }
    if (c->has_pas_invalid_from) {
        if (c->pas_invalid_from_s < 0.0 || c->pas_invalid_to_s <= c->pas_invalid_from_s || c->pas_invalid_to_s > c->duration_s) {
            fprintf(stderr, "pas invalid window: must satisfy 0 <= from < to <= duration=%g\n", c->duration_s);
            return false;
        }
    }

    if (c->has_pas_invalid_seq_from != c->has_pas_invalid_seq_to) {
        fprintf(stderr, "pas invalid seq window: pas_invalid_seq_from_s and pas_invalid_seq_to_s must be given together\n");
        return false;
    }
    if (c->has_pas_invalid_seq_from) {
        if (c->pas_invalid_seq_from_s < 0.0 || c->pas_invalid_seq_to_s <= c->pas_invalid_seq_from_s || c->pas_invalid_seq_to_s > c->duration_s) {
            fprintf(stderr, "pas invalid seq window: must satisfy 0 <= from < to <= duration=%g\n", c->duration_s);
            return false;
        }
        double dur = c->pas_invalid_seq_to_s - c->pas_invalid_seq_from_s;
        if (dur * CTRL_HZ < 2.0 * PAS_SAMPLER_GLITCH_TICKS) {
            fprintf(stderr, "pas invalid seq window too short: need at least %.0f ms for two accepted transitions\n",
                (2.0 * PAS_SAMPLER_GLITCH_TICKS) / CTRL_HZ * 1000.0);
            return false;
        }
    }

    if (c->miss_tick_every_n != 0U && c->miss_tick_every_n < 2U) {
        fprintf(stderr, "miss_tick_every_n must be 0 (disabled) or >= 2\n");
        return false;
    }
    if (c->has_miss_tick_from != c->has_miss_tick_to) {
        fprintf(stderr, "miss tick window: miss_tick_from_s and miss_tick_to_s must be given together\n");
        return false;
    }
    if (c->has_miss_tick_from) {
        if (c->miss_tick_every_n == 0U) {
            fprintf(stderr, "miss tick window given but miss_tick_every_n is 0 (disabled)\n");
            return false;
        }
        if (c->miss_tick_from_s < 0.0 || c->miss_tick_to_s <= c->miss_tick_from_s
            || c->miss_tick_to_s > c->duration_s) {
            fprintf(stderr, "miss tick window: must satisfy 0 <= from < to <= duration=%g\n", c->duration_s);
            return false;
        }
    }

    if (c->has_pas_edge_drop_from != c->has_pas_edge_drop_to
        || c->has_pas_edge_drop_from != c->has_pas_edge_drop_every_n) {
        fprintf(stderr, "pas edge drop: pas_edge_drop_from_s, _to_s and _every_n must be given together\n");
        return false;
    }
    if (c->has_pas_edge_drop_from) {
        if (c->pas_edge_drop_from_s < 0.0 || c->pas_edge_drop_to_s <= c->pas_edge_drop_from_s
            || c->pas_edge_drop_to_s > c->duration_s) {
            fprintf(stderr, "pas edge drop window: must satisfy 0 <= from < to <= duration=%g\n", c->duration_s);
            return false;
        }
        if (c->pas_edge_drop_every_n < 2U) {
            fprintf(stderr, "pas_edge_drop_every_n=%u must be >= 2 (n=1 is a dead line, not a missed edge)\n",
                (unsigned)c->pas_edge_drop_every_n);
            return false;
        }
    }

    if (c->has_pas_edge_jitter_from != c->has_pas_edge_jitter_to
        || c->has_pas_edge_jitter_from != c->has_pas_edge_jitter_ticks) {
        fprintf(stderr, "pas edge jitter: pas_edge_jitter_from_s, _to_s and _ticks must be given together\n");
        return false;
    }
    if (c->has_pas_edge_jitter_from) {
        if (c->pas_edge_jitter_from_s < 0.0 || c->pas_edge_jitter_to_s <= c->pas_edge_jitter_from_s
            || c->pas_edge_jitter_to_s > c->duration_s) {
            fprintf(stderr, "pas edge jitter window: must satisfy 0 <= from < to <= duration=%g\n", c->duration_s);
            return false;
        }
        if (c->pas_edge_jitter_ticks == 0U || c->pas_edge_jitter_ticks > 400U) {
            fprintf(stderr, "pas_edge_jitter_ticks=%u must be in [1,400] 4 kHz ticks\n",
                (unsigned)c->pas_edge_jitter_ticks);
            return false;
        }
    }

    /* The electrical edge disturbances and the line OVERRIDES are separate classes on purpose. */
    if ((c->has_pas_edge_drop_from || c->has_pas_edge_jitter_from)
        && (c->has_pas_invalid_seq_from || c->has_reverse_bounce_at)) {
        fprintf(stderr, "pas edge drop/jitter cannot be combined with pas_invalid_seq_* or reverse_bounce_at_s\n");
        return false;
    }

    if (c->has_sample_ticks && (c->sample_ticks_override == 0U || c->sample_ticks_override > 40000U)) {
        fprintf(stderr, "sample_ticks=%u must be in [1,40000] 4 kHz ticks\n", (unsigned)c->sample_ticks_override);
        return false;
    }

    if (c->has_fg_delay_at != c->has_fg_delay_ticks) {
        fprintf(stderr, "foreground delay: fg_delay_at_s and fg_delay_ticks must be given together\n");
        return false;
    }
    if (c->has_fg_delay_at) {
        if (c->fg_delay_at_s < 0.0 || c->fg_delay_at_s > c->duration_s) {
            fprintf(stderr, "fg_delay_at_s=%g out of range [0, duration=%g]\n", c->fg_delay_at_s, c->duration_s);
            return false;
        }
        if (c->fg_delay_ticks == 0U) {
            fprintf(stderr, "fg_delay_ticks must be >= 1\n");
            return false;
        }
    }

    return true;
}

int main(int argc, char **argv)
{
    lab_cfg_t cfg = {
        .duration_s = 6.0,
        .cadence_rpm = 72.0,
        .cadence_ripple_pct = 0.0,
        .cadence_ripple_hz = 0.6,
        .torque_nm = 28.0,
        .torque_ripple_pct = 45.0,
        .asymmetry_pct = 8.0,
        .speed_kph = 18.0,
        .battery_v = 39.0,
        .soc_pct = 55.0,
        .assist_level = 3,
        .mode = ASSIST_MODE_POWER_LINEAR,
        .sample_ms = 5,
        .ride_start_s = 0.0,
        .ride_stop_s = 0.0,
        .torque_step_at_s = 0.0,
        .torque_step_nm = 0.0,
        .ride_restart_s = 0.0,
        .ride_stop2_s = 0.0,
        .has_torque_step_at = false,
        .has_torque_step_nm = false,
        .has_ride_restart = false,
        .has_ride_stop2 = false
    };
    for (int i = 1; i < argc; i++) {
        if (parse_arg(&cfg, argv[i]) != 0) {
            fprintf(stderr, "invalid argument: %s\n", argv[i]);
            return 2;
        }
    }
    cfg.duration_s = clampd(cfg.duration_s, 0.25, 30.0);
    cfg.cadence_rpm = clampd(cfg.cadence_rpm, 0.0, 180.0);
    cfg.cadence_ripple_pct = clampd(cfg.cadence_ripple_pct, 0.0, 80.0);
    cfg.cadence_ripple_hz = clampd(cfg.cadence_ripple_hz, 0.05, 5.0);
    cfg.torque_nm = clampd(cfg.torque_nm, 0.0, 180.0);
    cfg.torque_ripple_pct = clampd(cfg.torque_ripple_pct, 0.0, 150.0);
    cfg.asymmetry_pct = clampd(cfg.asymmetry_pct, -90.0, 90.0);
    cfg.speed_kph = clampd(cfg.speed_kph, 0.0, 100.0);
    cfg.battery_v = clampd(cfg.battery_v, 20.0, 65.0);
    cfg.soc_pct = clampd(cfg.soc_pct, 0.0, 100.0);
    if (cfg.assist_level < 0) cfg.assist_level = 0;
    if (cfg.assist_level > (int)LAB_ASSIST_LEVEL_COUNT) cfg.assist_level = LAB_ASSIST_LEVEL_COUNT;
    if (cfg.sample_ms < 1) cfg.sample_ms = 1;
    if (cfg.sample_ms > 100) cfg.sample_ms = 100;
    cfg.ride_start_s = clampd(cfg.ride_start_s, 0.0, cfg.duration_s);
    if (cfg.ride_stop_s > 0.0)
        cfg.ride_stop_s = clampd(cfg.ride_stop_s, cfg.ride_start_s, cfg.duration_s);

    /* New event keys are REJECTED when inconsistent, not clamped into something else. */
    if (!validate_events(&cfg)) return 2;

    /* Canable global tuning goes first: lab_init reads the RUN window from tuning_config. */
    if (!apply_tuning_blob_hex(cfg.tuning_blob)) return 2;

    lab_t lab;
    lab_init(&lab, &cfg);

    /* Bank blob after lab_init - assist_modes_init() inside it re-seeds the banks. */
    if (!apply_bank_blob_hex(cfg.bank_blob)) return 2;

    if (!configure_mode(cfg.assist_level, cfg.mode)) {
        fprintf(stderr, "invalid/unavailable assist mode %d\n", cfg.mode);
        return 2;
    }

    fprintf(stderr, "ControllerLab mode=%s assist=%d duration=%.2fs cadence=%.1frpm torque=%.1fNm\n",
        mode_name(cfg.mode), cfg.assist_level, cfg.duration_s, cfg.cadence_rpm, cfg.torque_nm);
    print_header();
    uint32_t total = (uint32_t)llround(cfg.duration_s * CTRL_HZ);
    /*
     * sample_ms is a MILLISECOND decimation: at 4 kHz the finest it can express is 4 ticks, so
     * sample_ms=1 already aliases anything shorter. sample_ticks= overrides it directly and is
     * what the timebase trials use to observe a single skipped tick. Unset -> unchanged.
     */
    uint32_t sample_ticks = (uint32_t)cfg.sample_ms * CTRL_HZ / 1000U;
    if (cfg.has_sample_ticks) sample_ticks = cfg.sample_ticks_override;
    if (sample_ticks == 0U) sample_ticks = 1U;
    for (uint32_t i = 0; i < total; i++)
        lab_tick(&lab, &cfg, ((i + 1U) % sample_ticks) == 0U);
    return 0;
}
