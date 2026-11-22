/**
 * @file bsp_uart.h
 * @brief USART1 console BSP: register-level init + polled TX/RX + self-test.
 *
 * Teaching module (P3 DL-A5 single-module pilot, template for the P3 batch).
 * Full CubeMX config table / register walkthrough / verification recipe live
 * in the bsp_uart.c file header.
 *
 * Ownership rule (framework knowledge #4): after bsp_uart_init() this module
 * OWNS the USART1 peripheral registers. CubeMX HAL does GPIO AF7 config and
 * peripheral clocks first (MX_USART1_UART_Init path); this module then
 * rewrites BRR/CR1/CR2/CR3 at register level. Never interleave HAL_UART_*
 * calls on USART1 afterwards.
 */
#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>

/* Expected BRR for APB2 = 84 MHz, 115200 baud, oversampling 16.
 * Derivation (RM0090 ch.30 "Fractional baud rate generation"):
 *   USARTDIV = fCK / (16 * baud) = 84000000 / 1843200 = 45.5729
 *   mantissa = 45 (0x2D), fraction = round(0.5729 * 16) = 9
 *   BRR = 45*16 + 9 = 729 = 0x2D9
 *   actual baud = 84000000 / (16*45.5625) = 115226 -> error +0.023%
 *   (8N1 tolerance budget is ~2%, so 0.02% is comfortably exact). */
#define BSP_UART_BRR_EXPECTED  0x2D9u

/* Register-level re-init of USART1: clock enable, BRR, CR1 UE/TE|RE,
 * CR2/CR3 zeroed. Returns 0 = PASS (BRR + CR1 read-back matches),
 * -1 = read-back mismatch. */
int bsp_uart_init(void);

/* Polled string write. Per byte: wait TXE with an iteration cap, then write
 * DR; after the last byte wait TC (shift register fully drained). Never
 * hangs: on flag timeout it returns early (watchdog discipline, spec #5). */
void bsp_uart_puts(const char *s);

/* Non-blocking RX: returns the next byte 0..255, or -1 when RXNE is clear.
 * Reads SR then DR -- the RM0090 sequence that also clears ORE/FE/NE. */
int bsp_uart_getc_nonblock(void);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. Prints step detail lines on the console as it goes.
 * Steps:
 *   1 = BRR/CR1 read-back against BSP_UART_BRR_EXPECTED / UE|TE|RE
 *   2 = TC-gated TX of "UART-SELFTEST-OK" (bounded waits)
 *   3 = RX idle-timeout path: RXNE must stay clear in a quiet window and
 *       must clear on a DR read if a stray byte arrives */
int bsp_uart_selftest(void);

#endif /* BSP_UART_H */
