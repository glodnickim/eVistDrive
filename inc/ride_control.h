#ifndef RIDE_CONTROL_H_
#define RIDE_CONTROL_H_

#include <stdint.h>

typedef enum {
	RIDE_ENGINE_LEGACY = 0,
	RIDE_ENGINE_TSDZ
} ride_engine_t;

void ride_control_init(void);
int32_t ride_control_update_request(void);
ride_engine_t ride_control_get_engine(void);

#endif /* RIDE_CONTROL_H_ */
