/**
 * @file bsp_tim_pwm.h
 * @brief TIM1_CH1 (PA8) + TIM4_CH1 (PD12) PWM output -- HAL channel config,
 *        per-mille duty API, self-test.
 *
 * Teaching module (HAL edition of f407-periph-lab bsp_tim_pwm, P3a wave --
 * see bsp_uart). CubeMX config table / HAL mapping / verification recipe
 * live in the bsp_tim_pwm.c file header.
 *
 * Ownership rule (A3): CubeMX owns the peripherals' clocks, pins and base
 * timebase (MX_TIM1/MX_TIM4_Init + MspInit: PSC=0, ARR=max, CEN=0). This
 * module owns the channel SHAPE only, written exclusively through
 * HAL_TIM_PWM_ConfigChannel / HAL_TIM_PWM_Start and the __HAL_TIM_SET_*
 * macros. No peripheral control-register writes live here; register reads
 * (PSC/CCR/CCER/BDTR/CR1 probes) are allowed as verification and carry the
 * "read-only (A3 exemption)" comment.
 */
#ifndef BSP_TIM_PWM_H
#define BSP_TIM_PWM_H

#include <stdint.h>

/* Channel selectors for bsp_tim_pwm_set_duty(). One channel per timer:
 * TIM1_CH1 -> PA8 (advanced timer, APB2 kernel 168 MHz), TIM4_CH1 -> PD12
 * (general timer, APB1 kernel 84 MHz). */
#define BSP_TIM_PWM_CH_TIM1_CH1 0u
#define BSP_TIM_PWM_CH_TIM4_CH1 1u

/* HAL PWM init for both channels: 1 kHz frame (frequency derivation in the
 * .c header), 50% duty, TIM1 main output (BDTR MOE, set inside
 * HAL_TIM_PWM_Start) on. Returns 0 = PASS (PSC/ARR read-back matches,
 * CC1E/MOE verified), -1 = mismatch or a HAL call failed. Idempotent
 * (safe to call repeatedly; each call disarms first). */
int bsp_tim_pwm_init(void);

/* Set duty as per-mille of the frame (1000 = full on, 0 = full off;
 * values >1000 are rejected with -1 and no update). PWM mode 1: output is
 * active while CNT < CCR, so CCR = round((ARR+1)*per_mille/1000).
 * With the OC1PE preload (written by HAL_TIM_PWM_ConfigChannel) the new
 * compare value takes effect at the next update event (<=1 frame = 1 ms
 * later), glitch-free. Returns 0 = written. */
int bsp_tim_pwm_set_duty(uint32_t ch, uint32_t per_mille);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. Steps:
 *   1 = PSC/ARR read-back against the derived 1 kHz values (TIM1 167/999,
 *       TIM4 83/999)
 *   2 = duty round-trip: set_duty(500) then CCR read-back (via
 *       __HAL_TIM_GET_COMPARE) must land within +/-1 of ARR/2 on both
 *       channels
 *   3 = enable bits read-back: CCER CC1E on both, TIM1 BDTR MOE (the bit
 *       whose absence silences every advanced-timer output), CR1 CEN on
 *       both */
int bsp_tim_pwm_selftest(void);

#endif /* BSP_TIM_PWM_H */
