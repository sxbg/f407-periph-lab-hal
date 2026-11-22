/**
 * @file log_uart.c
 * @brief Polling register-level UART log writer (family-neutral).
 *
 * Register symbols (USART base, SR/DR register names) come from
 * hal_port.h generated per family pack; this file contains no family
 * literal. Note: F4/H7 both spell the polling registers SR/DR; families
 * using ISR/TDR (G0/L4/U5) need HAL_LOG_TXE/HAL_LOG_DR macros added to
 * the pack seam.
 */
#include "log_uart.h"

#include "hal_port.h"

#include <stdarg.h>

static void usart_write_byte(uint8_t b)
{
    uint32_t guard = LOG_POLL_LIMIT;
    while ((HAL_LOG_USART->HAL_LOG_SR & HAL_LOG_TXE) == 0u) {
        if (--guard == 0u) {
            return; /* bounded spin: never hang the log path */
        }
    }
    HAL_LOG_USART->HAL_LOG_DR = (uint32_t)b;
}

void log_uart_write(const char *s)
{
    if (s == 0) { return; }
    while (*s != '\0') {
        usart_write_byte((uint8_t)(*s++));
    }
}

void log_uart_line(const char *s)
{
    log_uart_write(s);
    usart_write_byte((uint8_t)'\r');
    usart_write_byte((uint8_t)'\n');
}

/* ---- minimal printf (%s %d %u %x %X %02x %%) ---------------------------- */
static void put_hex(uint32_t v, unsigned width)
{
    static const char d[16] = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F' };
    char buf[9];
    unsigned n = 0u;
    do { buf[n++] = d[v & 0xFu]; v >>= 4; } while (v > 0u && n < 8u);
    while (n < width && n < 8u) { buf[n++] = '0'; }
    while (n > 0u) { usart_write_byte((uint8_t)buf[--n]); }
}

static void put_dec(uint32_t v)
{
    char buf[11];
    unsigned n = 0u;
    do { buf[n++] = (char)('0' + (v % 10u)); v /= 10u; } while (v > 0u);
    while (n > 0u) { usart_write_byte((uint8_t)buf[--n]); }
}

void log_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    for (const char *p = fmt; *p != '\0'; p++) {
        if (*p != '%') { usart_write_byte((uint8_t)*p); continue; }
        p++;
        if (*p == '\0') { break; }
        switch (*p) {
        case 's': {
            const char *s = va_arg(ap, const char *);
            log_uart_write(s ? s : "(null)");
            break;
        }
        case 'd': {
            int32_t v = va_arg(ap, int32_t);
            if (v < 0) { usart_write_byte((uint8_t)'-'); put_dec((uint32_t)(-v)); }
            else { put_dec((uint32_t)v); }
            break;
        }
        case 'u': put_dec(va_arg(ap, uint32_t)); break;
        case 'x': put_hex(va_arg(ap, uint32_t), 0u); break;
        case 'X': put_hex(va_arg(ap, uint32_t), 0u); break;
        case '0': {
            if (p[1] == '2' && p[2] == 'x') {
                put_hex(va_arg(ap, uint32_t), 2u);
                p += 2;
            } else {
                usart_write_byte((uint8_t)'%');
                usart_write_byte((uint8_t)'0');
            }
            break;
        }
        case '%': usart_write_byte((uint8_t)'%'); break;
        default:
            usart_write_byte((uint8_t)'%');
            usart_write_byte((uint8_t)*p);
            break;
        }
    }
    va_end(ap);
}
