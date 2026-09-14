/*
 * SCHEMA-2 CAPTURE FIXTURE, produced by the PRODUCTION serializer.
 *
 * The decoder test used to build its frames by hand. That proves the decoder agrees with the
 * test author, which is not the question: the question is whether the decoder agrees with the
 * firmware. So this links the real src/ride_telemetry.c, feeds it a snapshot with known values,
 * captures whatever it actually transmits, and prints it as a raw CANable log.
 *
 * If the wire layout and the decoder ever drift apart, this fixture moves with the firmware and
 * the decoder test fails - which is exactly what a shipped decoder reading a shipped capture
 * needs someone to notice.
 *
 * Usage: ride_telemetry_fixture > capture.log
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ride_telemetry.h"

/* The values the decoder test asserts on. Distinct and non-round, so a field read from the
 * wrong offset cannot accidentally match. */
#define FX_TICK                     0x12345U
#define FX_LOAD_CENTIKG             987U
#define FX_TORQUE_NORMALIZED        321U
#define FX_RIDER_DEMAND             654U
#define FX_ASSIST_BASE              410U
#define FX_ASSIST_DYNAMIC           190U
#define FX_ASSIST_RESPONSE          572U
#define FX_RIDER_AGGRESSION         640U
#define FX_LOAD_STATE               275U
#define FX_AUTO_FACTOR              830U

/* The second snapshot: a later tick and different values, so the decoder has to assemble two
 * distinct rows rather than one repeated one. */
#define FX_TICK_STEP                84U
#define FX_LOAD_CENTIKG_2           1234U
#define FX_RIDER_DEMAND_2           777U
#define FX_ASSIST_BASE_2            555U

static uint32_t g_capture_us = 1000000U;

typedef struct {
    uint32_t efid;
    uint8_t data[8];
    bool busy;
} fx_mailbox_t;

static fx_mailbox_t g_box;

static void emit(uint32_t efid, const uint8_t *d)
{
    /* A real capture logs the driver can_id verbatim: extended frames keep CAN_EFF_FLAG. */
    printf("[10:00:00]\t[INFO]\t%u\tID:%08X\tDLC:8\tData:", g_capture_us, efid | 0x80000000U);
    for (unsigned i = 0; i < 8U; i++) {
        printf("%s%02X", i ? " " : "", d[i]);
    }
    printf("\n");
    g_capture_us += 3000U;
}

static uint8_t fx_transmit(uint32_t efid, const uint8_t *data)
{
    memcpy(g_box.data, data, 8U);
    g_box.efid = efid;
    g_box.busy = true;
    emit(efid, data);
    return 0U;   /* mailbox 0 */
}

static uint8_t fx_state(uint8_t mailbox)
{
    (void)mailbox;
    if (g_box.busy) {
        g_box.busy = false;
        return DIAG_CAN_OK;
    }
    return DIAG_CAN_OK;
}

static const diag_can_ops_t FX_OPS = { fx_transmit, fx_state };

int main(void)
{
    ride_telemetry_snapshot_t s;
    uint32_t t;

    memset(&s, 0, sizeof(s));
    s.control_tick = FX_TICK;
    s.load_centikg = FX_LOAD_CENTIKG;
    s.torque_normalized_permille = FX_TORQUE_NORMALIZED;
    s.rider_demand_permille = FX_RIDER_DEMAND;
    s.assist_base_permille = FX_ASSIST_BASE;
    s.assist_dynamic_permille = FX_ASSIST_DYNAMIC;
    s.assist_response_permille = FX_ASSIST_RESPONSE;
    s.rider_aggression_permille = FX_RIDER_AGGRESSION;
    s.load_state_permille = FX_LOAD_STATE;
    s.auto_factor_permille = FX_AUTO_FACTOR;
    s.limit_flags = AP2_TELEM_LIM_BATTERY | AP2_TELEM_LIM_SPEED | AP2_TELEM_LIM_START;
    s.cadence_raw_rpm = 83U;
    s.cadence_control_rpm = 79U;
    s.iq_requested = 501;
    s.iq_allowed = 440;
    s.iq_ref = 333;
    s.iq_actual = -222;
    s.id_actual = 17;
    s.motor_erps = 1444U;
    s.battery_voltage_10mv = 4123U;
    s.battery_current_10ma = 1234;
    s.soc_display_x10 = 678U;
    s.wheel_speed_x100 = 2345U;
    s.u_abs = 1701U;
    s.flags = 0xA55AU;
    s.permission_bits = 0x9BU;
    s.debug_flags = 0x42U;
    s.session_state = 2U;
    s.qzero_state = 3U;
    s.assist_level = 5U;
    s.theta_q15 = -12345;
    s.hall_age_ticks = 234U;
    s.hall_state = 5U;
    s.rotor_trusted = 1U;
    s.bridge_lifecycle = 6U;
    s.pwm_on = 1U;
    s.pas_ab = 3U;
    s.pas_direction_state = 2U;
    s.pas_backpedal = 1U;
    s.direction_inhibit = 1U;
    s.start_phase = 1U;
    s.active_profile_bank = 1U;

    ride_telemetry_init(&FX_OPS);
    ride_telemetry_capture(&s);

    /*
     * Drive the module long enough for two complete snapshots AND a META frame. It paces its
     * own frames, so the loop gives it time rather than assuming an order - the order the
     * decoder has to cope with is whatever the module chooses.
     *
     * The loop clock and the snapshot's control_tick are the SAME clock, as they are in the
     * firmware - the module paces META against the snapshot tick it last sent, so feeding it
     * two unrelated clocks makes the version frame permanently overdue. Starting at the
     * snapshot tick, which is already past the META interval, makes META due on the first
     * opportunity: that is what a capture joined mid-ride looks like.
     */
    for (t = FX_TICK; ; t++) {
        ride_telemetry_step(t, true, true);
        if (ride_telemetry_completed_snapshots() >= 1U) {
            break;
        }
        if (t > FX_TICK + 20000U) {
            fputs("fixture: the first snapshot was never completed", stderr);
            return 1;
        }
    }

    /*
     * A SECOND snapshot, at a later control tick and with different values.
     *
     * Two snapshots that share a tick are one snapshot as far as the decoder is concerned - the
     * tick IS the identity. Advancing it is what exercises tick unwrapping, virtual-time
     * ordering and the decoder's per-snapshot assembly, which a repeat of the same values
     * cannot.
     */
    s.control_tick = FX_TICK + FX_TICK_STEP;
    s.load_centikg = FX_LOAD_CENTIKG_2;
    s.rider_demand_permille = FX_RIDER_DEMAND_2;
    s.assist_base_permille = FX_ASSIST_BASE_2;
    s.limit_flags = 0U;
    ride_telemetry_capture(&s);

    for (t = t + 1U; ; t++) {
        ride_telemetry_step(t, true, true);
        if (ride_telemetry_completed_snapshots() >= 2U) {
            break;
        }
        if (t > FX_TICK + 40000U) {
            fputs("fixture: the second snapshot was never completed", stderr);
            return 1;
        }
    }

    return 0;
}
