/**
 * @file boot_diag.h
 * @brief Bring-up diagnostics trio (template headers).
 *
 * Lessons encoded:
 *  - boot_diag: blink a raw pin BEFORE clock config proves the image runs
 *    (a dead heartbeat may just mean clock setup never completed).
 *  - clock_fallback: HSE -> backup -> HSI staged probe; the engaged stage is
 *    reported both as an RTT event and via log_uart.
 *  - error_handler: count and RETURN instead of silent while(1); the count
 *    is a logcheck-monitored event.
 */
#ifndef BOOT_DIAG_H
#define BOOT_DIAG_H

#include <stdint.h>

#include "hal_port.h"   /* HAL_GPIO_TYPEDEF (family seam, ADR-0002) */

/* Raw-register fast blink, called before HAL/clock init. 6 blinks. */
void boot_diag_pre_clock(uint32_t gpio_port_base, uint32_t pin_mask);

/* Post-clock indicator: N pulses encode the clock stage reached.
 * gpio_port is the family-seam port pointer (HAL_GPIO_PORT(x)). */
void boot_diag_clock_ok(uint32_t stage, HAL_GPIO_TYPEDEF *gpio_port,
                        uint32_t pin_mask);

#endif /* BOOT_DIAG_H */

#ifndef CLOCK_FALLBACK_H_GUARD
#define CLOCK_FALLBACK_H_GUARD

/* Returns engaged sysclk stage: 0=HSE PLL (nominal), 1=HSI PLL, 2=HSI bare.
 * Emits RTT event CLOCK_FALLBACK on any non-zero stage. */
uint32_t clock_init_with_fallback(void);

#endif /* CLOCK_FALLBACK_H_GUARD */

#ifndef ERROR_HANDLER_H_GUARD
#define ERROR_HANDLER_H_GUARD

/* Counted error: increments the counter, logs an RTT event with the caller
 * tag, and RETURNS (never silent while(1)). The app loop may inspect the
 * counter and decide to stop. */
void app_error_handler(uint16_t where);

/* Current error count (monitored by tests/logcheck). */
uint32_t app_error_count(void);

#endif /* ERROR_HANDLER_H_GUARD */
