/*
 * HOST TEST STUB - not part of the firmware.
 *
 * The real inc/gd32f307c_eval.h always resolves via the including file's own directory
 * (inc/), so a host build cannot avoid pulling it in through inc/main.h. That real file
 * `#include`s "gd32f30x.h" (the vendor GD32 CMSIS device header, ~thousands of lines of
 * memory-mapped register definitions) purely to get a handful of opaque pin/clock/IRQ
 * name tokens it re-exports as its own macros - it never dereferences a real register.
 *
 * This stub is placed on the host compiler's include path AHEAD of the real
 * Firmware/CMSIS/GD/GD32F30x/Include/gd32f30x.h (which is not on this build's include
 * path at all - see tests/host/run_regression.ps1), so `#include "gd32f30x.h"` from
 * inc/main.h and inc/gd32f307c_eval.h resolves here instead. It defines ONLY the two
 * real types main.h's structs use (FlagStatus, the CAN message structs) plus the
 * opaque integer tokens gd32f307c_eval.h chains into its own #defines. Nothing here is
 * ever linked against real peripheral behaviour - no host test drives a GPIO/RCU/EXTI
 * register through these names.
 *
 * NEVER included by the ARM build: the .cproject include path list is
 * Firmware/CMSIS/GD/GD32F30x/Include, Firmware/GD32F30x_standard_peripheral/Include,
 * inc/ - this directory is not on it.
 */
#ifndef HOST_STUB_GD32F30X_H
#define HOST_STUB_GD32F30X_H

#include <stdint.h>

typedef enum { RESET = 0, SET = !RESET } FlagStatus;
typedef enum { ERROR = 0, SUCCESS = !ERROR } ErrStatus;

/* main.h: extern can_trasnmit_message_struct transmit_message; extern
 * can_receive_message_struct receive_message; - never dereferenced by any module a host
 * test links, so field content is irrelevant; the type just has to be complete. */
typedef struct { uint8_t host_stub_unused; } can_trasnmit_message_struct;
typedef struct { uint8_t host_stub_unused; } can_receive_message_struct;

/* Opaque tokens gd32f307c_eval.h chains into LED2_PIN/Hall1_PIN/EVAL_COM0/... #defines.
 * Never evaluated as real register addresses in any translation unit a host test builds. */
#define GPIO_PIN_0   0
#define GPIO_PIN_2   2
#define GPIO_PIN_3   3
#define GPIO_PIN_6   6
#define GPIO_PIN_7   7
#define GPIO_PIN_8   8
#define GPIO_PIN_9   9
#define GPIO_PIN_10  10
#define GPIO_PIN_12  12
#define GPIO_PIN_13  13
#define GPIO_PIN_14  14
#define GPIOA 0
#define GPIOB 1
#define GPIOC 2
#define RCU_GPIOA 0
#define RCU_GPIOB 1
#define RCU_GPIOC 2
#define RCU_USART0 3
#define RCU_USART1 4
#define USART0 0
#define USART1 1
#define EXTI_0  0
#define EXTI_13 13
#define EXTI_14 14
#define GPIO_PORT_SOURCE_GPIOA 0
#define GPIO_PORT_SOURCE_GPIOB 1
#define GPIO_PORT_SOURCE_GPIOC 2
#define GPIO_PIN_SOURCE_0  0
#define GPIO_PIN_SOURCE_13 13
#define GPIO_PIN_SOURCE_14 14
#define EXTI0_IRQn 0
#define EXTI10_15_IRQn 1

#endif /* HOST_STUB_GD32F30X_H */
