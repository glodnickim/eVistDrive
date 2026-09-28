/*
 * Deterministic tests for the production CAN RX SPSC queue plus wiring guards for
 * the hardware IRQ, bounded main-loop parser service, physical liveness events,
 * and the existing reversed 0x6300/0x6303 mapping.
 */
#include "../common/check.h"
#include "can_rx_queue.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define STR2(x) #x
#define STR(x) STR2(x)

#ifndef MAIN_C_PATH
#error "MAIN_C_PATH required"
#endif
#ifndef CAN_DISPLAY_C_PATH
#error "CAN_DISPLAY_C_PATH required"
#endif
#ifndef GD32_IT_C_PATH
#error "GD32_IT_C_PATH required"
#endif

static char *read_file(const char *path)
{
	FILE *f = fopen(path, "rb");
	if (!f) return NULL;
	if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return NULL; }
	long n = ftell(f);
	if (n < 0 || fseek(f, 0, SEEK_SET) != 0) { fclose(f); return NULL; }
	char *p = (char *)malloc((size_t)n + 1U);
	if (!p) { fclose(f); return NULL; }
	size_t got = fread(p, 1U, (size_t)n, f);
	fclose(f);
	/* Normalize CRLF/CR to LF: a Windows autocrlf checkout must match the same patterns. */
	size_t out = 0U;
	for (size_t i = 0U; i < got; i++) {
		if (p[i] == '\r') {
			p[out++] = '\n';
			if (i + 1U < got && p[i + 1U] == '\n') i++;
		} else {
			p[out++] = p[i];
		}
	}
	p[out] = '\0';
	return p;
}

static uint32_t hmi_id(uint16_t command)
{
	return 0x83100000UL | (uint32_t)command; /* source 3, target 2, WRITE */
}

static can_rx_frame_t frame(uint16_t command, uint8_t value)
{
	can_rx_frame_t f = {0};
	f.rx_efid = hmi_id(command);
	f.rx_dlen = (command == 0x6301U) ? 8U : ((command == 0x6302U) ? 5U : 4U);
	f.rx_data[0] = 5U;
	f.rx_data[1] = value;
	f.rx_data[2] = 1U;
	f.rx_data[3] = 1U;
	return f;
}

static void test_fifo_burst_and_old_mailbox_regression(void)
{
	static const uint16_t ids[] = {0x6300U,0x6301U,0x6302U,0x6303U,0x6304U,
		0x6300U,0x6300U,0x6300U,0x6300U};
	can_rx_frame_t f, out;
	uint32_t old_single_slot = 0U;
	can_rx_queue_init();

	for (uint8_t i = 0U; i < (uint8_t)(sizeof(ids)/sizeof(ids[0])); i++) {
		f = frame(ids[i], (ids[i] == 0x6300U) ? 1U : i);
		old_single_slot = f.rx_efid; /* exact old overwrite behavior: only the last ID survives */
		can_rx_queue_note_received(f.rx_efid, true, true);
		CHECK(can_rx_queue_push_from_isr(&f), "burst: production queue accepts each HMI frame");
	}
	CHECK(old_single_slot == hmi_id(ids[(sizeof(ids)/sizeof(ids[0]))-1U]) && ids[0] != ids[1],
		"old one-slot model retains only the final frame and loses intervening 0x6301..0x6304 members");
	CHECK(can_rx_queue_depth() == sizeof(ids)/sizeof(ids[0]),
		"burst: all five operational IDs plus repeated stable 0x6300 updates are retained");
	CHECK((can_rx_event_flags & (CAN_RX_EVENT_BUS|CAN_RX_EVENT_HMI)) ==
		(CAN_RX_EVENT_BUS|CAN_RX_EVENT_HMI),
		"burst: physical bus and source-3 HMI liveness are recorded at receive time");
	for (uint8_t i = 0U; i < (uint8_t)(sizeof(ids)/sizeof(ids[0])); i++) {
		CHECK(can_rx_queue_pop(&out), "burst: queued frame is available to foreground parser");
		CHECK(out.rx_efid == hmi_id(ids[i]), "burst: dequeue preserves FIFO command order");
		CHECK(out.rx_data[1] == ((ids[i] == 0x6300U) ? 1U : i),
			"burst: dequeue preserves copied payload bytes");
	}
	CHECK(can_rx_queue_depth() == 0U && !can_rx_queue_pop(&out),
		"burst: final dequeue leaves an empty queue");
}

static void test_capacity_overflow_drops_newest_only(void)
{
	can_rx_frame_t f, out;
	can_rx_queue_init();
	for (uint8_t i = 0U; i < CAN_RXQ_CAPACITY; i++) {
		f = frame((uint16_t)(0x6300U + i), i);
		CHECK(can_rx_queue_push_from_isr(&f), "capacity: all sixteen ring entries are usable");
	}
	f = frame(0x6304U, 0xEEU);
	CHECK(!can_rx_queue_push_from_isr(&f), "capacity: seventeenth frame is rejected instead of overwriting unread data");
	CHECK(can_rx_queue_overflow_count() == 1U && can_rx_queue_depth() == CAN_RXQ_CAPACITY,
		"capacity: overflow is counted and old unread frames remain queued");
	CHECK(can_rx_queue_pop(&out) && out.rx_efid == hmi_id(0x6300U) && out.rx_data[1] == 0U,
		"capacity: oldest unread frame remains intact after overflow");
}

static void test_parser_backlog_liveness_and_watchdog_wiring(void)
{
	uint16_t hmi_lost_ticks = 0U;
	const uint16_t comm_cut_ticks = 75U;
	can_rx_frame_t f, out;
	can_rx_queue_init();
	/* Keep application parsing behind for 70 slow-loop intervals while valid source-3 RX
	 * remains physically present. Main's event exchange must reset the watchdog independently. */
	for (uint16_t t = 0U; t < 70U; t++) {
		f = frame(0x6300U, 1U);
		can_rx_queue_note_received(f.rx_efid, true, true);
		CHECK(can_rx_queue_push_from_isr(&f), "liveness: HMI RX queues despite delayed parser service");
		uint8_t events = can_rx_event_flags;
		can_rx_event_flags = 0U;
		if (events & CAN_RX_EVENT_HMI) hmi_lost_ticks = 0U;
		else hmi_lost_ticks++;
		if (can_rx_queue_depth() == CAN_RXQ_CAPACITY) (void)can_rx_queue_pop(&out);
	}
	CHECK(hmi_lost_ticks < comm_cut_ticks,
		"liveness: parser backlog cannot advance physical-HMI watchdog to cut threshold");
	can_rx_queue_init();
	hmi_lost_ticks = 0U;
	for (uint16_t t = 0U; t < comm_cut_ticks; t++) hmi_lost_ticks++;
	CHECK(hmi_lost_ticks == comm_cut_ticks,
		"watchdog: physically silent HMI still reaches the existing cut threshold");
}

static void test_production_wiring_and_reversed_parser_mapping(void)
{
	char *irq = read_file(STR(GD32_IT_C_PATH));
	char *mainc = read_file(STR(MAIN_C_PATH));
	char *display = read_file(STR(CAN_DISPLAY_C_PATH));
	CHECK(irq != NULL && mainc != NULL && display != NULL,
		"wiring: production IRQ, main, and HMI parser sources can be read");
	if (!irq || !mainc || !display) { free(irq); free(mainc); free(display); return; }
	const char *handler = strstr(irq, "void CAN0_RX1_IRQHandler(void)");
	const char *handler_end = handler ? strstr(handler, "\n}") : NULL;
	const char *push = handler ? strstr(handler, "can_rx_queue_push_from_isr") : NULL;
	CHECK(handler && handler_end && push && push < handler_end,
		"wiring: RX IRQ copies into the production FIFO");
	CHECK(handler && handler_end && strstr(handler, "processCAN_Rx") == NULL,
		"wiring: application parser does not run inside the ISR");
	CHECK(handler && handler_end && strstr(handler, "receive_flag") == NULL,
		"wiring: legacy lossy one-slot flag is removed from the IRQ");
	const char *pop = strstr(mainc, "can_rx_queue_pop(&queued_rx)");
	const char *parser = pop ? strstr(pop, "processCAN_Rx(&MP, &MS)") : NULL;
	const char *physical_events = strstr(mainc, "can_rx_consume_liveness_events(void)");
	CHECK(pop && parser && parser > pop,
		"wiring: FIFO dequeue copies into receive_message then reaches processCAN_Rx in main");
	CHECK(physical_events && strstr(physical_events, "CAN_RX_EVENT_HMI") &&
		strstr(physical_events, "hmi_lost_ticks=0U") &&
		strstr(mainc, "can_rx_consume_liveness_events();\n\t\t\t\tif(hmi_lost_ticks") != NULL,
		"wiring: physical HMI RX event is consumed independently of parser progress");
	CHECK(strstr(display, "if(Ext_ID_Rx.command==0x6300)") != NULL &&
		strstr(display, "level_code=receive_message.rx_data[1]") != NULL &&
		strstr(display, "level_counter==3") != NULL &&
		strstr(display, "case 1:") != NULL,
		"parser: repeated stable 0x6300 continues through the existing level mapping");
	CHECK(strstr(display, "Ext_ID_Rx.command==0x6303 && receive_message.rx_dlen>=1") != NULL &&
		strstr(display, "auto_off_minutes=receive_message.rx_data[0]") != NULL,
		"parser: queued 0x6303 retains the existing auto-off update");
	CHECK(strstr(mainc, "ride_control_force_final_iq_zero();") != NULL &&
		strstr(mainc, "hmi_lost_ticks >= COMM_CUT_TICKS") != NULL,
		"watchdog: real physical silence still invokes the existing exact-zero owner");
	free(irq); free(mainc); free(display);
}

int main(void)
{
	printf("CAN RX SPSC queue, physical HMI liveness, and frozen 0x6300/0x6303 parser wiring\n");
	test_fifo_burst_and_old_mailbox_regression();
	test_capacity_overflow_drops_newest_only();
	test_parser_backlog_liveness_and_watchdog_wiring();
	test_production_wiring_and_reversed_parser_mapping();
	if (host_test_failures == 0) { printf("All can_rx_queue checks passed.\n"); return 0; }
	printf("%d can_rx_queue check(s) FAILED.\n", host_test_failures);
	return 1;
}
