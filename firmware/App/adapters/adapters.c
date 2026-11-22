/**
 * @file adapters.c
 * @brief Adapter implementations (registers bound to port interfaces).
 *
 * Family-neutral: register symbols come from hal_port.h (generated from
 * the family pack), never from a family CMSIS header directly.
 */
#include "adapters.h"

#include <stdbool.h>

/* ---- time ---------------------------------------------------------------- */
static uint32_t time_tick_ms(void)
{
    return HAL_GetTick();
}

static void time_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

static const time_port_t s_time = {
    .tick_ms = time_tick_ms,
    .delay_ms = time_delay_ms,
};

const time_port_t *adapter_time(void)
{
    return &s_time;
}

/* ---- digout ---------------------------------------------------------------- */
typedef struct {
    HAL_GPIO_TYPEDEF *port;
    uint16_t pin;
} digout_ctx_t;

static digout_ctx_t s_digout_ctx;

static void digout_set(bool level)
{
    if (level) {
        s_digout_ctx.port->BSRR = (uint32_t)s_digout_ctx.pin;
    } else {
        s_digout_ctx.port->BSRR = (uint32_t)s_digout_ctx.pin << 16u;
    }
}

static void digout_toggle(void)
{
    const uint32_t pin = (uint32_t)s_digout_ctx.pin;
    if ((s_digout_ctx.port->ODR & pin) != 0u) {
        s_digout_ctx.port->BSRR = pin << 16u;
    } else {
        s_digout_ctx.port->BSRR = pin;
    }
}

static const digout_port_t s_digout = {
    .set = digout_set,
    .toggle = digout_toggle,
};

const digout_port_t *adapter_digout(HAL_GPIO_TYPEDEF *port, uint16_t pin)
{
    s_digout_ctx.port = port;
    s_digout_ctx.pin = pin;
    return &s_digout;
}
