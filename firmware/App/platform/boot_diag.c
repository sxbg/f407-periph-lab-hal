/**
 * @file boot_diag.c
 * @brief Bring-up diagnostics trio (family-neutral implementation).
 *
 * Register access goes through hal_port.h (generated per family pack,
 * ADR-0002); this file contains no family literal.
 */
#include "boot_diag.h"
#include "hal_port.h"
#include "rtt_log.h"
#include "log_uart.h"

/* ---- boot_diag ----------------------------------------------------------- */
static void raw_delay(volatile uint32_t loops)
{
    while (loops--) {
        __asm volatile ("nop");
    }
}

void boot_diag_pre_clock(uint32_t gpio_port, uint32_t pin_mask)
{
    /* Enable every GPIO bus clock the pack knows about (A..H/K), then
     * raw-blink the given pin BEFORE any HAL/clock init. */
    HAL_GPIO_BUS_REG |= HAL_GPIO_CLK_A | HAL_GPIO_CLK_B | HAL_GPIO_CLK_C
                     | HAL_GPIO_CLK_D | HAL_GPIO_CLK_E
#ifdef HAL_GPIO_CLK_H
                     | HAL_GPIO_CLK_H
#endif
#ifdef HAL_GPIO_CLK_K
                     | HAL_GPIO_CLK_K
#endif
                     ;
    (void)HAL_GPIO_BUS_REG;

    volatile HAL_GPIO_TYPEDEF *port = (volatile HAL_GPIO_TYPEDEF *)gpio_port;
    /* output mode, push-pull: set MODER bits for the lowest set pin */
    uint32_t bit = 0u;
    uint32_t m = pin_mask;
    while ((m & 1u) == 0u) { m >>= 1; bit++; }
    port->MODER |= (0x1u << (2u * bit));
    port->OTYPER &= ~pin_mask;
    for (uint32_t k = 0u; k < 6u; k++) {
        port->BSRR = pin_mask;            /* on */
        raw_delay(400000u);
        port->BSRR = (pin_mask << 16u);   /* off */
        raw_delay(400000u);
    }
}

void boot_diag_clock_ok(uint32_t stage, HAL_GPIO_TYPEDEF *gpio_port,
                        uint32_t pin_mask)
{
    /* 3 pulses = stage 0 (nominal), fewer = deeper degradation. */
    volatile HAL_GPIO_TYPEDEF *port = gpio_port;
    uint32_t pulses = 3u - (stage > 2u ? 2u : stage);
    while (pulses--) {
        port->BSRR = pin_mask;
        raw_delay(600000u);
        port->BSRR = (pin_mask << 16u);
        raw_delay(600000u);
    }
}

/* ---- clock_fallback ------------------------------------------------------- */
uint32_t clock_init_with_fallback(void)
{
    /* Stage probe AFTER HAL clock config: which source actually engaged?
     * Register names below are family-common (RCC CFGR SWS / CR HSERDY
     * exist on all STM32 with these exact spellings). */
    uint32_t stage = 0u;
    uint32_t sysclk_mhz = SystemCoreClock / 1000000u;

    if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL) {
        if ((RCC->CR & RCC_CR_HSERDY) != 0u) {
            stage = 0u; /* PLL from HSE: nominal */
        } else {
            stage = 1u; /* PLL but HSE not ready: unexpected, treat as fallback */
        }
    } else {
        stage = 2u; /* not on PLL at all: HSI bare */
    }

    if (stage != 0u) {
        rtt_ev(RTT_EV_CLOCK_FALLBACK, (uint16_t)stage, sysclk_mhz, 0u);
        log_printf("clock: fallback stage=%u sysclk=%uMHz\r\n",
                   (unsigned)stage, (unsigned)sysclk_mhz);
    }
    return stage;
}

/* ---- error_handler -------------------------------------------------------- */
static volatile uint32_t s_error_count = 0u;

void app_error_handler(uint16_t where)
{
    s_error_count++;
    rtt_ev(RTT_EV_ERR_HANDLER,
           s_error_count > 0xFFFFu ? 0xFFFFu : (uint16_t)s_error_count,
           (uint32_t)where, 0u);
    log_printf("error_handler: count=%u where=0x%04x\r\n",
               (unsigned)s_error_count, (unsigned)where);
    /* Deliberately returns. The main loop decides whether to continue. */
}

uint32_t app_error_count(void)
{
    return s_error_count;
}
