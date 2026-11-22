/**
 * @file log_uart.h
 * @brief Polling register-level UART log writer (template).
 *
 * Why not HAL_UART_Transmit: HAL blocks on HAL_GetTick(); if SysTick is
 * starved (clock misconfig, fault handler), HAL_UART_Transmit spins forever
 * -- and the log is exactly the tool needed to diagnose that state. This
 * module polls the UART status register directly with an iteration cap
 * (LOG_POLL_LIMIT), so a broken clock still yields readable boot logs.
 */
#ifndef LOG_UART_H
#define LOG_UART_H

#include <stdint.h>

/* Target instance is fixed at template level (USART1 per project.yaml).
 * Port to another instance by changing the register macros below. */
#ifndef LOG_POLL_LIMIT
#define LOG_POLL_LIMIT 2000000u /* bounded spin: ~tens of ms worst case */
#endif

void log_uart_init(void);            /* called after HAL UART init or standalone */
void log_uart_write(const char *s);  /* null-terminated ASCII string */
void log_uart_line(const char *s);   /* write + CRLF */

/* Printf-style one-liner built on a tiny internal formatter; supports
 * %s %d %u %x %02x %% only (no float; keep logs ASCII). */
void log_printf(const char *fmt, ...);

#endif /* LOG_UART_H */
