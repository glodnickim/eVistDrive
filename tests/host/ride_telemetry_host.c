#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ride_telemetry.h"

#define MAX_FRAMES 32

typedef struct { uint32_t id; uint8_t d[8]; } frame_t;
static frame_t frames[MAX_FRAMES];
static unsigned frame_count;
static uint8_t mailbox_state = DIAG_CAN_OK;
static uint8_t next_mailbox = 1U;

static uint8_t fake_tx(uint32_t efid, const uint8_t *data)
{
    if (frame_count >= MAX_FRAMES) return DIAG_CAN_NOMAILBOX;
    frames[frame_count].id = efid;
    memcpy(frames[frame_count].d, data, 8U);
    frame_count++;
    return next_mailbox;
}

static uint8_t fake_state(uint8_t mailbox)
{
    (void)mailbox;
    return mailbox_state;
}

static const diag_can_ops_t ops = { fake_tx, fake_state };

static uint16_t be16(const uint8_t *d) { return (uint16_t)(((uint16_t)d[0] << 8) | d[1]); }
static int16_t bei16(const uint8_t *d) { return (int16_t)be16(d); }

static int fail(const char *what)
{
    fprintf(stderr, "FAIL: %s\n", what);
    return 1;
}

int main(void)
{
    ride_telemetry_snapshot_t s;
    memset(&s, 0, sizeof(s));
    s.control_tick=0x12345U;
    s.load_centikg=987U; s.torque_normalized_permille=321U; s.rider_demand_permille=654U;
    s.assist_base_permille=410U; s.assist_dynamic_permille=190U; s.assist_response_permille=572U;
    s.rider_aggression_permille=640U; s.load_state_permille=275U; s.auto_factor_permille=830U;
    s.limit_flags=AP2_TELEM_LIM_BATTERY|AP2_TELEM_LIM_SPEED|AP2_TELEM_LIM_START;
    s.cadence_raw_rpm=83U; s.cadence_control_rpm=79U;
    s.iq_requested=501; s.iq_allowed=440; s.iq_ref=333;
    s.iq_actual=-222; s.id_actual=17; s.motor_erps=1444U;
    s.battery_voltage_10mv=4123U; s.battery_current_10ma=1234; s.soc_display_x10=678U;
    s.wheel_speed_x100=2345U; s.u_abs=1701U; s.flags=0xA55AU;
    s.permission_bits=0x9BU; s.debug_flags=0x42U; s.session_state=2U; s.qzero_state=3U;
    s.assist_level=5U;
    s.theta_q15=-12345; s.hall_age_ticks=234U; s.hall_state=5U; s.rotor_trusted=1U;
    s.bridge_lifecycle=6U; s.pwm_on=1U; s.pas_ab=3U; s.pas_direction_state=2U;
    s.pas_backpedal=1U; s.direction_inhibit=1U; s.start_phase=1U; s.active_profile_bank=1U;

    ride_telemetry_init(&ops);
    if (!ride_telemetry_capture_due(s.control_tick)) return fail("first capture not due");
    ride_telemetry_capture(&s);
    if (ride_telemetry_capture_due(s.control_tick + RIDE_TELEMETRY_CAPTURE_INTERVAL_TICKS - 1U))
        return fail("capture work runs faster than the telemetry snapshot rate");
    if (!ride_telemetry_capture_due(s.control_tick + RIDE_TELEMETRY_CAPTURE_INTERVAL_TICKS))
        return fail("next coherent capture never becomes due");

    /* Never transmit while the caller says the ride is inactive or critical CAN owns priority. */
    for (uint32_t t=0;t<100;t++) ride_telemetry_step(t, true, false);
    if (frame_count != 0U) return fail("transmitted while ride inactive");
    for (uint32_t t=100;t<200;t++) ride_telemetry_step(t, false, true);
    if (frame_count != 0U) return fail("transmitted while allow_new_tx=false");

    /* Run enough ticks for one complete seven-frame snapshot. A fake mailbox completes on the
     * next call; the module must pace frame starts by RIDE_TELEMETRY_FRAME_INTERVAL_TICKS. */
    for (uint32_t t=200;t<400;t++) ride_telemetry_step(t, true, true);
    if (frame_count < RIDE_TELEMETRY_DATA_FRAMES) return fail("did not send a complete snapshot");
    for (unsigned i=0;i<RIDE_TELEMETRY_DATA_FRAMES;i++) {
        /* SCHEMA 2: data frames 0..6 keep BASE+0..6; the two added frames step OVER META at
         * BASE+7 so no pre-existing frame ever changes identifier. */
        uint32_t want = RIDE_TELEMETRY_EFID_BASE + ((i<=6U) ? i : (i+1U));
        if (frames[i].id != want) return fail("wrong data EFID order");
        if (be16(frames[i].d) != (uint16_t)s.control_tick) return fail("snapshot tick not coherent across frames");
    }
    if (frames[7].id != RIDE_TELEMETRY_EFID_ASSIST) return fail("ASSIST frame must not take META id");
    if (frames[8].id != RIDE_TELEMETRY_EFID_RIDER) return fail("RIDER frame id");
    if (be16(&frames[0].d[2])!=987U || be16(&frames[0].d[4])!=321U || be16(&frames[0].d[6])!=654U)
        return fail("CORE payload");
    if (bei16(&frames[1].d[2])!=501 || bei16(&frames[1].d[4])!=440 || bei16(&frames[1].d[6])!=333)
        return fail("DEMAND payload");
    if (bei16(&frames[2].d[2])!=-222 || bei16(&frames[2].d[4])!=17 || be16(&frames[2].d[6])!=1444U)
        return fail("MOTOR payload");
    if (be16(&frames[3].d[2])!=4123U || bei16(&frames[3].d[4])!=1234 || be16(&frames[3].d[6])!=678U)
        return fail("BATT payload");
    if (be16(&frames[4].d[2])!=2345U || be16(&frames[4].d[4])!=1701U || be16(&frames[4].d[6])!=0xA55AU)
        return fail("LIMITS payload");
    if (frames[5].d[2]!=0x9BU || frames[5].d[3]!=0x42U || frames[5].d[5]!=83U || frames[5].d[6]!=79U)
        return fail("STATE payload");
    if (frames[5].d[7]!=(uint8_t)(s.limit_flags & 0xFFU))
        return fail("STATE spare byte must carry the low limit flags");
    if ((frames[5].d[4]&0x0FU)!=5U || ((frames[5].d[4]>>4)&3U)!=2U || ((frames[5].d[4]>>6)&3U)!=3U)
        return fail("STATE packed level/session/qzero");
    if (bei16(&frames[6].d[2])!=-12345 || be16(&frames[6].d[4])!=234U)
        return fail("ROTOR numeric payload");
    if ((frames[6].d[6]&7U)!=5U || ((frames[6].d[6]>>3)&1U)!=1U || ((frames[6].d[6]>>4)&7U)!=6U || ((frames[6].d[6]>>7)&1U)!=1U)
        return fail("ROTOR packed hall/lifecycle");
    if ((frames[6].d[7]&3U)!=3U || ((frames[6].d[7]>>2)&3U)!=2U || ((frames[6].d[7]>>4)&1U)!=1U || ((frames[6].d[7]>>5)&1U)!=1U || ((frames[6].d[7]>>6)&1U)!=1U)
        return fail("ROTOR packed PAS");
    if (be16(&frames[7].d[2])!=410U || be16(&frames[7].d[4])!=190U || be16(&frames[7].d[6])!=572U)
        return fail("ASSIST payload: base / dynamic / response");
    if (be16(&frames[8].d[2])!=640U || be16(&frames[8].d[4])!=275U || be16(&frames[8].d[6])!=830U)
        return fail("RIDER payload: aggression / load / auto factor");
    if (ride_telemetry_completed_snapshots() < 1U) return fail("snapshot completion accounting");

    /* A failed accepted mailbox is counted once and the stream advances instead of retry-flooding
     * a stale sample forever. */
    unsigned before=frame_count;
    mailbox_state=DIAG_CAN_FAILED;
    for (uint32_t t=400;t<450 && frame_count==before;t++) ride_telemetry_step(t,true,true);
    if (frame_count==before) return fail("could not start failure probe frame");
    ride_telemetry_step(450,true,true);
    if (ride_telemetry_failed_frames()!=1U) return fail("failed frame counter");

    printf("PASS: FW-145 live ride telemetry pacing/payload/priority\n");
    return 0;
}
