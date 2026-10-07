/*
 * Assist Behavior V3 - motion / IMU seam sanitiser (ARCHITECTURE_V3.md section 3.3).
 * Pure, integer only, no state.
 */
#include "assist_motion.h"

#include <string.h>

void assist_motion_sanitize(const motion_input_t *raw, motion_input_t *out)
{
    motion_input_t tmp;
    unsigned char valid_byte = 0u;

    if (out == NULL) {
        return;
    }
    memset(&tmp, 0, sizeof(tmp));
    if (raw != NULL) {
        /* Read the flag through its object representation: a random byte in a bool is not a
         * valid bool value, so it must not be loaded as one. */
        memcpy(&valid_byte, &raw->valid, 1u);
        if (valid_byte != 0u && raw->age_ms <= MOTION_MAX_AGE_MS) {
            tmp.valid = true;
            tmp.age_ms = raw->age_ms;
            tmp.pitch_cdeg = raw->pitch_cdeg;
            tmp.roll_cdeg = raw->roll_cdeg;
            tmp.pitch_rate_cdps = raw->pitch_rate_cdps;
            tmp.accel_long_mg = raw->accel_long_mg;
            tmp.accel_vert_mg = raw->accel_vert_mg;
        }
    }
    /* memset above also zeroes padding, so the output is bit-identical for every invalid input. */
    memcpy(out, &tmp, sizeof(tmp));
}
