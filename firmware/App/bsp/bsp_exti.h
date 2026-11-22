/**
 * @file bsp_exti.h
 * @brief EXTI routes for KEY1 (PA0 = EXTI0, rising) and KEY2 (PA1 =
 *        EXTI1, falling) -- HAL edition with ISR press counter,
 *        interactive self-test.
 *
 * Teaching module (HAL edition of f407-periph-lab bsp_exti). Pin truth:
 * PA0 KEY1 pull-down (press closes to VDD -> press IS the rising edge),
 * PA1 KEY2 pull-up (press grounds it -> press IS the falling edge).
 * This module arms the interrupt ROUTES through HAL calls and owns the
 * HAL callback; A3 rule (PLAN A3): no peripheral control-register
 * writes; the HAL API performs every peripheral change.
 *
 * Register-concept -> HAL-call mapping (vs the register edition):
 *   SYSCFG EXTICR nibble (line <- port)  -> HAL_GPIO_Init(...,
 *       GPIO_MODE_IT_*) writes the same nibble
 *   FTSR/RTSR trigger latches           -> the Mode field of the same
 *       HAL_GPIO_Init call (IT_RISING / IT_FALLING)
 *   IMR unmask                          -> also HAL_GPIO_Init (IT modes
 *       set the line's interrupt mask)
 *   EXTI->PR rc_w1 stale clear          -> __HAL_GPIO_EXTI_CLEAR_IT
 *       (the HAL macro spelling of the same write-1-to-clear)
 *   NVIC_SetPriority/EnableIRQ          -> HAL_NVIC_SetPriority/
 *       EnableIRQ (EXTI0 + EXTI1, priority 5 per the .ioc NVIC section)
 *   EXTI1_IRQHandler ISR (PR test + rc_w1 clear + counter)  ->
 *       Core-owned vectors (Core/Src/stm32f4xx_it.c) dispatching into
 *       the HAL_GPIO_EXTI_Callback weak override HERE -- filtered by
 *       GPIO_Pin because BOTH lines share the one callback
 *   IWDG->KR = 0xAAAA window feed       -> HAL_IWDG_Refresh(&hiwdg)
 *   SysTick COUNTFLAG spin (window)     -> HAL_GetTick() delta
 *   direct-USART selftest prompt        -> dropped (rule #7
 *       console-free; the GREEN LED blink is the cue -- the old
 *       edition already taught "watch the BOARD, not a terminal")
 *
 * CORE-SIDE WIRING DEPENDENCY (flagged, not fixable from a bsp module):
 * the generated EXTI0_IRQHandler/EXTI1_IRQHandler in
 * Core/Src/stm32f4xx_it.c are EMPTY bodies (no HAL_GPIO_EXTI_IRQHandler
 * call). Until their USER CODE sections dispatch into the HAL (one line
 * each), presses never reach HAL_GPIO_EXTI_Callback below -- and a
 * press with vectors live would leave the pending bit set (rc_w1
 * semantics: an empty ISR cannot clear it, the IRQ re-fires forever).
 * The fill is a 2-line USER CODE edit on the Core side; the MX_GPIO_Init
 * electricals (both pins IT_RISING, no pull) are re-armed correctly by
 * bsp_exti_init through HAL meanwhile.
 */
#ifndef BSP_EXTI_H
#define BSP_EXTI_H

#include <stdint.h>

/* HAL-edition route init: re-arm PA0/PA1 through HAL_GPIO_Init as EXTI
 * lines (KEY1 rising + pull-down, KEY2 falling + pull-up -- the IOC
 * truth; the generated config currently has both pins rising with no
 * pull), clear stale pending bits (HAL macro), enable the EXTI0/EXTI1
 * NVIC vectors at priority 5. Returns 0 = PASS (route read-back),
 * -1 = mismatch. */
int bsp_exti_init(void);

/* Presses counted by the ISR-side callback since init (volatile
 * counter, mainline read; never resets short of a reboot). Counts
 * nothing until the Core-side vector dispatch is filled in (see the
 * wiring note above). */
uint32_t bsp_exti_key2_count(void);

/* One-line report of the LAST selftest run, for the menu to append to its
 * verdict line: "(N presses)" or "(idle, N presses)". Empty string before
 * the first run (data-not-print keeps the module console-free,
 * bsp-module-spec rule #7). */
const char *bsp_exti_last_report(void);

/* Self-test. Returns 0 = PASS (always -- a missed window is SKIP-state
 * semantics, not a failure; only a config read-back failure FAILs),
 * otherwise the 1-based step number that failed. Steps:
 *   1 = route read-back: EXTI IMR lines 0/1 unmasked, KEY1 rising +
 *       KEY2 falling trigger latches, EXTICR nibbles map lines 0/1 to
 *       port A, PA0/PA1 input mode + IOC pulls, NVIC vectors enabled
 *   2 = interactive 6 s window: the GREEN LED blinks while the window
 *       is open (watch the BOARD); >= 3 KEY2 presses -> "(N presses)",
 *       timeout -> "(idle, N presses)" -- both PASS
 *       (hardware-absence-is-SKIP rule; the cumulative count across
 *       uptime also passes, per the register edition's verdict
 *       semantics) */
int bsp_exti_selftest(void);

#endif /* BSP_EXTI_H */
