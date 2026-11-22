/**
 * @file bsp_tim_basic.h
 * @brief TIM5/TIM6/TIM7 pure timebases -- TIM5 as a 1 Hz update interrupt
 *        counter, TIM6/TIM7 as polling-flag timers; register-level init
 *        and self-test.
 *
 * Teaching module (P3a wave, bsp-module-spec.md template -- see bsp_uart).
 * Full CubeMX config table / register walkthrough / verification recipe live
 * in the bsp_tim_basic.c file header.
 *
 * Ownership rule: TIM5 is in the IOC (base init only, interrupts off) --
 * after bsp_tim_basic_init() this module OWNS TIM5's CR1/PSC/ARR/EGR/DIER
 * and the TIM5 NVIC line. TIM6/TIM7 are NOT in the IOC at all -- the module
 * is their full owner: RCC APB1ENR gates, PSC/ARR, interrupt enables and
 * the TIM6_DAC_IRQHandler/TIM7_IRQHandler definitions (in the .c). Never
 * interleave HAL_TIM_* on these three timers.
 */
#ifndef BSP_TIM_BASIC_H
#define BSP_TIM_BASIC_H

#include <stdint.h>

/* Register-level init of the three timebases:
 *   TIM5  1 Hz update interrupt (PSC=8399, ARR=9999, UIE + NVIC on)
 *   TIM6  1 kHz update, polling only (UIE off)
 *   TIM7  100 Hz update, polling only (UIE off)
 * Returns 0 = PASS (PSC/ARR/CR1/DIER read-back matches), -1 = mismatch. */
int bsp_tim_basic_init(void);

/* Seconds elapsed since... the TIM5 interrupt counter (one tick per
 * update event). Wraps at 2^32 seconds (~136 years). Read once: volatile
 * on the ISR side, plain read here is safe on a 32-bit bus. */
uint32_t bsp_tim_basic_uptime_seconds(void);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. Steps:
 *   1 = TIM5 interrupt counter: register config read-back, then the
 *       counter must advance by >= 1 inside a 1.2 s window
 *   2 = TIM6 polling: SR UIF must set within a 200 ms window, must clear
 *       on the rc_w0 write, and must set again within another window
 *   3 = TIM7 polling: same contract with its 10 ms period */
int bsp_tim_basic_selftest(void);

#endif /* BSP_TIM_BASIC_H */
