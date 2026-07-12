#include "ride_control.h"

#include "legacy_assist.h"

static ride_engine_t active_engine;

void ride_control_init(void)
{
	active_engine = RIDE_ENGINE_LEGACY;
}

int32_t ride_control_update_request(void)
{
	switch (active_engine) {
	case RIDE_ENGINE_LEGACY:
		return legacy_assist_calculate();
	case RIDE_ENGINE_TSDZ:
	default:
		/* TSDZ is intentionally unavailable until the Legacy refactor is verified. */
		return 0;
	}
}

ride_engine_t ride_control_get_engine(void)
{
	return active_engine;
}
