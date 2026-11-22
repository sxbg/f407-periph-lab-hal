/**
 * @file adapters.h
 * @brief Adapter constructors: bind ports (interfaces) to HAL/platform impls.
 *
 * The composition root (app_main) calls these to wire domain -> hardware.
 * Family-neutral: all register access goes through platform/hal_port.h.
 */
#ifndef ADAPTERS_H
#define ADAPTERS_H

#include "ports.h"
#include "hal_port.h"

#include <stdint.h>

/* Time source backed by HAL_GetTick (or a SOF/DWT counter). */
const time_port_t *adapter_time(void);

/* Digital output bound to a GPIO port/pin pair (raw register write). */
const digout_port_t *adapter_digout(HAL_GPIO_TYPEDEF *port, uint16_t pin);

#endif /* ADAPTERS_H */
