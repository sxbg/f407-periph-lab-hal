/**
 * @file bsp_tim_pwm.h
 * @brief TIM1_CH1 (PA8) + TIM4_CH1 (PD12) PWM output -- register-level init,
 *        per-mille duty API, self-test.
 *
 * Teaching module (P3a wave, bsp-module-spec.md template -- see bsp_uart).
 * Full CubeMX config table / register walkthrough / verification recipe live
 * in the bsp_tim_pwm.c file header.
 *
 * Ownership rule: after bsp_tim_pwm_init() this module OWNS TIM1 and TIM4
 * register sets (CR1/PSC/ARR/EGR/CCMR1/CCER/BDTR) and the PA8/PD12 pin
 * config. CubeMX MX_TIM1/MX_TIM4_Init only did base init (PSC=0, ARR=max,
 * CEN=0) and routed the GPIO AF; the PWM channel registers are written
 * here at register level. Never interleave HAL_TIM_PWM_* on these timers.
 */
#ifndef BSP_TIM_PWM_H
#define BSP_TIM_PWM_H

#include <stdint.h>

/* Channel selectors for bsp_tim_pwm_set_duty(). One channel per timer:
 * TIM1_CH1 -> PA8 (advanced timer, APB2 kernel 168 MHz), TIM4_CH1 -> PD12
 * (general timer, APB1 kernel 84 MHz). */
#define BSP_TIM_PWM_CH_TIM1_CH1 0u
#define BSP_TIM_PWM_CH_TIM4_CH1 1u

/* Register-level PWM init for both channels: kernel clocks, GPIO AF,
 * 1 kHz frame (see frequency derivation in the .c header), 50% duty,
 * TIM1 main output (BDTR MOE) on. Returns 0 = PASS (PSC/ARR/CCER/BDTR
 * read-back matches), -1 = mismatch. */
int bsp_tim_pwm_init(void);

/* Set duty as per-mille of the frame (1000 = full on, 0 = full off;
 * values >1000 are rejected with -1 and no register write). PWM mode 1:
 * output is active while CNT < CCR, so CCR = round((ARR+1)*per_mille/1000).
 * With OC1PE preload the new compare value takes effect at the next update
 * event (<=1 frame = 1 ms later), glitch-free. Returns 0 = written. */
int bsp_tim_pwm_set_duty(uint32_t ch, uint32_t per_mille);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. Steps:
 *   1 = PSC/ARR read-back against the derived 1 kHz values (TIM1 167/999,
 *       TIM4 83/999)
 *   2 = duty round-trip: set_duty(500) then CCR read-back must land within
 *       +/-1 of ARR/2 on both channels
 *   3 = enable bits read-back: CCER CC1E on both, TIM1 BDTR MOE (the bit
 *       whose absence silences every advanced-timer output) */
int bsp_tim_pwm_selftest(void);

#endif /* BSP_TIM_PWM_H */
