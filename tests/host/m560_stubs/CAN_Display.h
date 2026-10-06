#ifndef M560_HOST_CAN_H
#define M560_HOST_CAN_H
#include <stdint.h>
extern uint8_t Para0[64],Para1[64],Para2[64];
void update_checksum(void);
#endif
