#ifndef CAN_PERIODIC_DUE_H_
#define CAN_PERIODIC_DUE_H_

#include <stdbool.h>
#include <stdint.h>

/* All periods are hardware 4 kHz ticks, independent of foreground frequency.
 * Index order is the CAN sender switch in main.c. Wire payloads are unchanged. */
enum { CAN_PERIODIC_COUNT = 8, CAN_PERIODIC_OFFER_SPACING = 4 };
static const uint32_t can_periodic_periods[CAN_PERIODIC_COUNT] = {
    1980U, 7920U, 39600U, 988U, 7920U, 15840U, 396U, 3960U
}; /* 1200, 320F, 3000, 3201, 3200, 3205, 3202, 3210 */
typedef struct {
    uint32_t next_due[CAN_PERIODIC_COUNT];
    uint32_t next_offer;
    uint8_t cursor;
} can_periodic_schedule_t;

/* Modular comparison: valid when a service gap is less than 2^31 ticks
 * (over six days). Avoid implementation-defined unsigned-to-signed casts. */
static inline bool can_periodic_reached(uint32_t now, uint32_t deadline)
{
    return (uint32_t)(now - deadline) < UINT32_C(0x80000000);
}

static inline void can_periodic_init(can_periodic_schedule_t *s, uint32_t now)
{
    for (unsigned i = 0; i < CAN_PERIODIC_COUNT; ++i)
        s->next_due[i] = now + can_periodic_periods[i];
    s->next_offer = now;
    s->cursor = 0;
}

/* At most ONE current-state offer per real millisecond, even after a long
 * stall. Round robin prevents a refused frame from starving another ID.
 * A refusal does NOT advance that ID's deadline. No missed-period replay. */
static inline int can_periodic_next_due(can_periodic_schedule_t *s, uint32_t now)
{
    if (!can_periodic_reached(now, s->next_offer)) return -1;
    for (unsigned n = 0; n < CAN_PERIODIC_COUNT; ++n) {
        unsigned i = (s->cursor + n) % CAN_PERIODIC_COUNT;
        if (can_periodic_reached(now, s->next_due[i])) {
            s->cursor = (uint8_t)((i + 1U) % CAN_PERIODIC_COUNT);
            s->next_offer = now + CAN_PERIODIC_OFFER_SPACING;
            return (int)i;
        }
    }
    return -1;
}

static inline void can_periodic_accepted(can_periodic_schedule_t *s, unsigned i, uint32_t now)
{
    const uint32_t period = can_periodic_periods[i];
    const uint32_t late = now - s->next_due[i];
    /* First phase-aligned deadline strictly after now; unsigned wrap is intended. */
    s->next_due[i] += (late / period + 1U) * period;
}

#endif /* CAN_PERIODIC_DUE_H_ */
