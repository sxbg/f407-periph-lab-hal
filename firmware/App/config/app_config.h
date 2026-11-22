/**
 * @file app_config.h
 * @brief Compile-time app constants.
 *
 * APP_HEARTBEAT_PORT/PIN are rendered from the family pack + init options
 * (asm init); they are the ONLY board-specific symbols here. Every constant
 * is referenced somewhere; delete instead of accumulate unused ones.
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "hal_port.h"   /* APP_HEARTBEAT_PORT uses HAL_GPIO_PORT(x) */

/* Heartbeat LED: rendered by asm init --led-port/--led-pin. */
#define APP_HEARTBEAT_PORT  HAL_GPIO_PORT(C)
#define APP_HEARTBEAT_PIN   (1u << 5u)
#define APP_HEARTBEAT_PIN_NUMBER 32

/* Blink half-period in ms (full on+off = 1000ms). */
#define APP_BLINK_PERIOD_MS   500u

/* RTT heartbeat period in ms (logcheck liveness probe window). */
#define APP_HEARTBEAT_PERIOD_MS 1000u

#endif /* APP_CONFIG_H */
