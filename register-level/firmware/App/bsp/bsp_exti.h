/**
 * @file bsp_exti.h
 * @brief EXTI line 1 <- KEY2 (PA1), falling edge, ISR press counter --
 *        register-level init, interactive self-test.
 *
 * Teaching module (P3e wave, bsp-module-spec.md template -- see bsp_uart).
 * Full config table / register walkthrough / verification recipe live
 * in the bsp_exti.c file header.
 *
 * Ownership rule: the SYSCFG/EXTI wiring is NOT in the IOC (knowledge #13:
 * adapter-owned). The PA1 PIN config (input + pull-up) belongs to bsp_gpio
 * (IOC truth: GPIO_Input, GPIO_PULLUP) -- this module wires the EXTI ROUTE
 * only (EXTICR/IMR/FTSR/PR + the NVIC vector), the bsp_pwr/EXTI0 precedent.
 * The ISR is 3 statements: read-test the pending bit, rc_w1 clear, bump a
 * volatile counter.
 */
#ifndef BSP_EXTI_H
#define BSP_EXTI_H

#include <stdint.h>

/* Register-level init: SYSCFG + GPIOA clocks, EXTICR[0] EXTI1 <- PA (0x0),
 * falling trigger only (FTSR TR1=1, RTSR TR1=0), IMR MR1 unmasked, stale
 * PR cleared, NVIC EXTI1_IRQn enabled at priority 10. Returns 0 = PASS
 * (config read-back), -1 = mismatch. */
int bsp_exti_init(void);

/* Presses counted by the ISR since init (volatile counter, mainline read;
 * never resets short of a reboot). */
uint32_t bsp_exti_key2_count(void);

/* One-line report of the LAST selftest run, for the menu to append to its
 * verdict line: "(N presses)" or "(idle, N presses)". Empty string before
 * the first run (bsp_tim_ic report pattern: data, not printing). */
const char *bsp_exti_last_report(void);

/* Self-test. Returns 0 = PASS (always -- a missed window is SKIP-state
 * semantics, not a failure; only a config read-back failure FAILs),
 * otherwise the 1-based step number that failed. Steps:
 *   1 = config read-back: EXTICR[0] EXTI1 nibble = 0 (PA), IMR MR1,
 *       FTSR TR1 set, RTSR TR1 clear, NVIC IRQ enabled
 *   2 = interactive 6 s window: prints the prompt on the console, then
 *       waits; >= 3 KEY2 presses -> "(N presses)"; timeout -> "(idle,
 *       N presses)" -- both PASS (hardware-absence-is-SKIP rule) */
int bsp_exti_selftest(void);

#endif /* BSP_EXTI_H */
