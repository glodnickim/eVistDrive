#ifndef CAN_PERIODIC_DUE_H_
#define CAN_PERIODIC_DUE_H_

#include <stdbool.h>
#include <stdint.h>

/* Count to a due threshold and then stay saturated until the caller confirms
 * that enqueue accepted the current logical occurrence. This keeps one due
 * occurrence per periodic ID and prevents a full TX queue from resetting its
 * period and silently losing the frame. */
static inline bool can_periodic_is_due(uint16_t *counter, uint16_t period)
{
	if (period == 0U) return false;
	if (*counter < period) (*counter)++;
	return *counter >= period;
}

static inline void can_periodic_enqueue_accepted(uint16_t *counter)
{
	*counter = 0U;
}

#endif /* CAN_PERIODIC_DUE_H_ */
