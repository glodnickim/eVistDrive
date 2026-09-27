#include "can_rx_queue.h"

#if CAN_RXQ_CAPACITY == 0U || (CAN_RXQ_CAPACITY & CAN_RXQ_MASK) != 0U
#error "CAN_RXQ_CAPACITY must be a non-zero power of two"
#endif

/* Published frame slots are volatile so compiler accesses remain ordered
 * around the producer/consumer index publication. DMB also orders SRAM writes
 * before publishing head on Cortex-M; host compilers get a compiler barrier. */
#if defined(__GNUC__) && (defined(__arm__) || defined(__thumb__))
#define CAN_RX_MEMORY_BARRIER() __asm__ volatile ("dmb" ::: "memory")
#elif defined(__GNUC__)
#define CAN_RX_MEMORY_BARRIER() __asm__ volatile ("" ::: "memory")
#else
#define CAN_RX_MEMORY_BARRIER() do { } while (0)
#endif

static volatile can_rx_frame_t ring[CAN_RXQ_CAPACITY];
static volatile uint8_t write_index;
static volatile uint8_t read_index;
static volatile uint32_t overflow_count;
volatile uint8_t can_rx_event_flags;

void can_rx_queue_init(void)
{
	write_index = 0U;
	read_index = 0U;
	overflow_count = 0U;
	can_rx_event_flags = 0U;
}

void can_rx_queue_note_received(uint32_t efid, bool is_extended, bool is_data)
{
	can_rx_event_flags |= CAN_RX_EVENT_BUS;
	if (is_extended && is_data && (((efid >> 24) & 0x1FU) == 3U)) {
		can_rx_event_flags |= CAN_RX_EVENT_HMI;
	}
}

bool can_rx_queue_push_from_isr(const can_rx_frame_t *frame)
{
	uint8_t head = write_index;
	uint8_t tail = read_index;
	if ((uint8_t)(head - tail) >= CAN_RXQ_CAPACITY) {
		overflow_count++;
		return false; /* drop newest; never overwrite an unread older frame */
	}

	volatile can_rx_frame_t *slot = &ring[head & CAN_RXQ_MASK];
	slot->rx_efid = frame->rx_efid;
	slot->rx_dlen = frame->rx_dlen;
	for (uint8_t i = 0U; i < 8U; i++) slot->rx_data[i] = frame->rx_data[i];
	CAN_RX_MEMORY_BARRIER();
	write_index = (uint8_t)(head + 1U);
	return true;
}

bool can_rx_queue_pop(can_rx_frame_t *frame)
{
	uint8_t tail = read_index;
	uint8_t head = write_index;
	if (tail == head) return false;

	CAN_RX_MEMORY_BARRIER();
	volatile can_rx_frame_t *slot = &ring[tail & CAN_RXQ_MASK];
	frame->rx_efid = slot->rx_efid;
	frame->rx_dlen = slot->rx_dlen;
	for (uint8_t i = 0U; i < 8U; i++) frame->rx_data[i] = slot->rx_data[i];
	CAN_RX_MEMORY_BARRIER();
	read_index = (uint8_t)(tail + 1U);
	return true;
}

uint8_t can_rx_queue_depth(void)
{
	return (uint8_t)(write_index - read_index);
}

uint32_t can_rx_queue_overflow_count(void)
{
	return overflow_count;
}
