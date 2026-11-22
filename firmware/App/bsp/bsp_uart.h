/**
 * @file bsp_uart.h
 * @brief USART1 console BSP (HAL edition): init verify + polled TX/RX +
 *        self-test.
 *
 * Teaching module (HAL edition of f407-periph-lab bsp_uart). CubeMX owns the
 * peripheral: MX_USART1_UART_Init() (Core/Src/usart.c) configured PA9/PA10
 * AF7, 115200 8N1, oversampling 16 from APB2 = 84 MHz. This module only
 * VERIFIES that configuration and drives the peripheral through HAL_UART_*.
 * A3 rule (PLAN A3): no peripheral control-register writes here.
 *
 * BRR expectation math (unchanged from the register edition):
 *   USARTDIV = 84000000 / (16 * 115200) = 45.5729
 *   mantissa 45, fraction round(0.5729 * 16) = 9 -> BRR = 0x2D9
 *   actual baud 115226 (+0.023% error, far inside the 8N1 ~2% budget).
 */
#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>

/* Expected BRR for APB2 = 84 MHz, 115200 baud, oversampling 16. */
#define BSP_UART_BRR_EXPECTED  0x2D9u

/* Verify the CubeMX-generated configuration: BRR matches the 115200
 * derivation above and UE|TE|RE are set. Read-back only -- the peripheral
 * stays HAL-owned. Returns 0 = PASS, -1 = mismatch. */
int bsp_uart_init(void);

/* Blocking string write through HAL_UART_Transmit with a bounded timeout
 * (never hangs: on HAL_TIMEOUT it returns early). */
void bsp_uart_puts(const char *s);

/* Non-blocking RX: HAL_UART_Receive with a zero tick timeout -- returns the
 * byte 0..255, or -1 when RXNE is clear (HAL_TIMEOUT). */
int bsp_uart_getc_nonblock(void);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. Steps:
 *   1 = BRR read-back against BSP_UART_BRR_EXPECTED + UE/TE/RE set
 *   2 = HAL_UART_Transmit of "UART-SELFTEST-OK" (bounded timeout)
 *   3 = RX idle-timeout path: HAL_UART_Receive must time out in a quiet
 *       window (RXNE stays clear) */
int bsp_uart_selftest(void);

#endif /* BSP_UART_H */
