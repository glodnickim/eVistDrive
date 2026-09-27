#ifndef CAN_RX_QUEUE_H_
#define CAN_RX_QUEUE_H_

#include <stdbool.h>
#include <stdint.h>

/* Single producer: CAN RX IRQ. Single consumer: foreground main loop. Capacity is
 * exactly 16 entries; byte indices advance modulo 256 and a power-of-two mask
 * selects the ring slot, so all 16 slots are usable without a shared count. */
#define CAN_RXQ_CAPACITY 16U
#define CAN_RXQ_MASK (CAN_RXQ_CAPACITY - 1U)

#define CAN_RX_EVENT_BUS 0x01U
#define CAN_RX_EVENT_HMI 0x02U

typedef struct {
	uint32_t rx_efid;
	uint8_t rx_dlen;
	uint8_t rx_data[8];
} can_rx_frame_t;

void can_rx_queue_init(void);
bool can_rx_queue_push_from_isr(const can_rx_frame_t *frame);
bool can_rx_queue_pop(can_rx_frame_t *frame);
uint8_t can_rx_queue_depth(void);
uint32_t can_rx_queue_overflow_count(void);

/* Set for every physical receive; HMI only for a source-3 extended data frame.
 * The foreground consumes these with IRQs briefly masked. */
void can_rx_queue_note_received(uint32_t efid, bool is_extended, bool is_data);
extern volatile uint8_t can_rx_event_flags;

#endif /* CAN_RX_QUEUE_H_ */
