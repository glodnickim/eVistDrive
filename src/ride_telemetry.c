#include "ride_telemetry.h"

#include <string.h>

#if CAN_RIDE_TELEMETRY_ENABLE

typedef struct {
    const diag_can_ops_t *can_ops;
    ride_telemetry_snapshot_t latest;
    ride_telemetry_snapshot_t cycle;
    bool latest_valid;
    bool cycle_valid;
    uint8_t frame_index;
    uint8_t mailbox;
    bool pending;
    bool pending_meta;
    uint32_t last_tx_tick;
    uint32_t last_meta_tick;
    uint32_t sent_frames;
    uint32_t failed_frames;
    uint32_t completed_snapshots;
} ride_telemetry_state_t;

static ride_telemetry_state_t T;

static uint16_t tick16(const ride_telemetry_snapshot_t *s)
{
    return (uint16_t)(s->control_tick & 0xFFFFU);
}

static void put_u16(uint8_t *d, uint16_t v)
{
    d[0] = (uint8_t)(v >> 8);
    d[1] = (uint8_t)v;
}

static void put_i16(uint8_t *d, int16_t v)
{
    put_u16(d, (uint16_t)v);
}

static void put_u32(uint8_t *d, uint32_t v)
{
    d[0] = (uint8_t)(v >> 24);
    d[1] = (uint8_t)(v >> 16);
    d[2] = (uint8_t)(v >> 8);
    d[3] = (uint8_t)v;
}

static void build_data_frame(uint8_t index, const ride_telemetry_snapshot_t *s,
                             uint32_t *efid, uint8_t d[8])
{
    memset(d, 0, 8U);
    put_u16(&d[0], tick16(s));
    /* Indices 0..6 sit at BASE+0..6. The two schema-2 frames step OVER META at BASE+7, which
     * keeps every pre-existing frame at the identifier its decoder already knows. */
    *efid = RIDE_TELEMETRY_EFID_BASE + ((index <= 6U) ? index : (index + 1U));

    switch (index) {
    case 0U: /* rider input: the measurement and the demand derived from it */
        put_u16(&d[2], s->load_centikg);
        put_u16(&d[4], s->torque_normalized_permille);
        put_u16(&d[6], s->rider_demand_permille);
        break;
    case 1U: /* canonical Iq chain */
        put_i16(&d[2], s->iq_requested);
        put_i16(&d[4], s->iq_allowed);
        put_i16(&d[6], s->iq_ref);
        break;
    case 2U: /* measured motor state */
        put_i16(&d[2], s->iq_actual);
        put_i16(&d[4], s->id_actual);
        put_u16(&d[6], s->motor_erps);
        break;
    case 3U: /* battery + production SOC */
        put_u16(&d[2], s->battery_voltage_10mv);
        put_i16(&d[4], s->battery_current_10ma);
        put_u16(&d[6], s->soc_display_x10);
        break;
    case 4U: /* wheel / voltage utilization / limiter + lifecycle flags */
        put_u16(&d[2], s->wheel_speed_x100);
        put_u16(&d[4], s->u_abs);
        put_u16(&d[6], s->flags);
        break;
    case 5U: /* policy state + both raw and conditioned cadence */
        d[2] = s->permission_bits;
        d[3] = s->debug_flags;
        d[4] = (uint8_t)((s->assist_level & 0x0FU) |
                         ((s->session_state & 0x03U) << 4) |
                         ((s->qzero_state & 0x03U) << 6));
        d[5] = s->cadence_raw_rpm;
        d[6] = s->cadence_control_rpm;
        /* SCHEMA 2: the low eight limit flags. Bit 8 (RELEASE) is recoverable from the reason
         * byte in d[3], so the spare byte carries the eight that are not. */
        d[7] = (uint8_t)(s->limit_flags & 0xFFU);
        break;
    case 7U: /* SCHEMA 2: the assist request, split into its two halves */
        put_u16(&d[2], s->assist_base_permille);
        put_u16(&d[4], s->assist_dynamic_permille);
        put_u16(&d[6], s->assist_response_permille);
        break;
    case 8U: /* SCHEMA 2: the two estimators, the AUTO decision and the binding limit */
        put_u16(&d[2], s->rider_aggression_permille);
        put_u16(&d[4], s->load_state_permille);
        /* auto_factor and the limit flags share the last word: the factor is permille (10 bits
         * of range) and the flags are nine bits, so both fit only if they are split across the
         * two frames they belong to - the factor stays whole here and the flags travel in the
         * frame-5 spare byte plus this one. */
        put_u16(&d[6], s->auto_factor_permille);
        break;
    case 6U: /* rotor estimator + PAS physical/direction facts */
        put_i16(&d[2], s->theta_q15);
        put_u16(&d[4], s->hall_age_ticks);
        d[6] = (uint8_t)((s->hall_state & 0x07U) |
                         ((s->rotor_trusted ? 1U : 0U) << 3) |
                         ((s->bridge_lifecycle & 0x07U) << 4) |
                         ((s->pwm_on ? 1U : 0U) << 7));
        d[7] = (uint8_t)((s->pas_ab & 0x03U) |
                         ((s->pas_direction_state & 0x03U) << 2) |
                         ((s->pas_backpedal ? 1U : 0U) << 4) |
                         ((s->direction_inhibit ? 1U : 0U) << 5) |
                         ((s->start_phase ? 1U : 0U) << 6));
        break;
    default:
        break;
    }
}

static void build_meta_frame(const ride_telemetry_snapshot_t *s, uint8_t d[8])
{
    d[0] = RIDE_TELEMETRY_SCHEMA_VERSION;
    d[1] = s->active_profile_bank;
    put_u32(&d[2], s->control_tick);
    d[6] = (T.failed_frames > 255U) ? 255U : (uint8_t)T.failed_frames;
    d[7] = RIDE_TELEMETRY_DATA_FRAMES;
}

void ride_telemetry_init(const diag_can_ops_t *can_ops)
{
    memset(&T, 0, sizeof(T));
    T.can_ops = can_ops;
}

bool ride_telemetry_capture_due(uint32_t now_tick)
{
    return !T.latest_valid ||
        (uint32_t)(now_tick - T.latest.control_tick) >= RIDE_TELEMETRY_CAPTURE_INTERVAL_TICKS;
}

void ride_telemetry_capture(const ride_telemetry_snapshot_t *snapshot)
{
    if (snapshot == 0) return;
    T.latest = *snapshot;
    T.latest_valid = true;
}

static void finish_pending(bool ok)
{
    if (ok) T.sent_frames++;
    else T.failed_frames++;

    if (T.pending_meta) {
        T.pending_meta = false;
        T.last_meta_tick = T.cycle.control_tick;
        return;
    }

    T.frame_index++;
    if (T.frame_index >= RIDE_TELEMETRY_DATA_FRAMES) {
        T.frame_index = 0U;
        T.cycle_valid = false;
        T.completed_snapshots++;
    }
}

void ride_telemetry_step(uint32_t now_tick, bool allow_new_tx, bool ride_active)
{
    if (T.can_ops == 0) return;

    if (T.pending) {
        uint8_t st = T.can_ops->state(T.mailbox);
        if (st == DIAG_CAN_PENDING) return;
        T.pending = false;
        finish_pending(st == DIAG_CAN_OK);
    }

    if (!ride_active || !T.latest_valid || !allow_new_tx) return;
    if ((uint32_t)(now_tick - T.last_tx_tick) < RIDE_TELEMETRY_FRAME_INTERVAL_TICKS) return;

    /* META is inserted only between complete snapshots, never in the middle of one. */
    bool meta_due = (T.frame_index == 0U) && !T.cycle_valid &&
                    ((uint32_t)(now_tick - T.last_meta_tick) >= RIDE_TELEMETRY_META_INTERVAL_TICKS);

    uint32_t efid;
    uint8_t data[8];
    if (meta_due) {
        T.cycle = T.latest; /* tick/bank identity of the meta record */
        build_meta_frame(&T.cycle, data);
        efid = RIDE_TELEMETRY_EFID_META;
        T.pending_meta = true;
    } else {
        if (!T.cycle_valid) {
            T.cycle = T.latest;     /* freeze one coherent 4 kHz snapshot for all seven frames */
            T.cycle_valid = true;
        }
        build_data_frame(T.frame_index, &T.cycle, &efid, data);
        T.pending_meta = false;
    }

    uint8_t mb = T.can_ops->transmit(efid, data);
    if (mb == DIAG_CAN_NOMAILBOX) {
        /* Nothing was written. Keep the exact same fragment and try again later; do not advance
         * pacing, do not count a drop, and never spin. */
        T.pending_meta = false;
        return;
    }

    T.mailbox = mb;
    T.pending = true;
    T.last_tx_tick = now_tick;
}

uint32_t ride_telemetry_sent_frames(void) { return T.sent_frames; }
uint32_t ride_telemetry_failed_frames(void) { return T.failed_frames; }
uint32_t ride_telemetry_completed_snapshots(void) { return T.completed_snapshots; }

#else

void ride_telemetry_init(const diag_can_ops_t *can_ops) { (void)can_ops; }
bool ride_telemetry_capture_due(uint32_t now_tick) { (void)now_tick; return false; }
void ride_telemetry_capture(const ride_telemetry_snapshot_t *snapshot) { (void)snapshot; }
void ride_telemetry_step(uint32_t now_tick, bool allow_new_tx, bool ride_active)
{ (void)now_tick; (void)allow_new_tx; (void)ride_active; }
uint32_t ride_telemetry_sent_frames(void) { return 0U; }
uint32_t ride_telemetry_failed_frames(void) { return 0U; }
uint32_t ride_telemetry_completed_snapshots(void) { return 0U; }

#endif
