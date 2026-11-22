/**
 * @file bsp_tim_adv.h
 * @brief TIM8 advanced-timer complementary outputs + dead-time generator
 *        (CH1=PC6 plain PWM; CH2=PC7 / CH2N=PB14 complementary pair) --
 *        register-level init, dead-time API, self-test.
 *
 * Teaching module (P3b wave, bsp-module-spec.md template -- see bsp_uart).
 * Full CubeMX config table / register walkthrough / verification recipe
 * (including the EP14 dead-time derivation this module implements) live in
 * the bsp_tim_adv.c file header.
 *
 * Ownership rule: TIM8 is in the IOC (PWM Generation1 CH1 + Generation2
 * CH2 + CH2N, PSC=0, base init only) -- after bsp_tim_adv_init() this
 * module OWNS TIM8's CR1/PSC/ARR/EGR/RCR/CCMR1/CCER/BDTR and the PC6/PC7/
 * PB14 pin config. Never interleave HAL_TIM_PWM_* on TIM8.
 */
#ifndef BSP_TIM_ADV_H
#define BSP_TIM_ADV_H

#include <stdint.h>

/* Kernel clock feeding TIM8 (APB2 x2 = 168 MHz on this clock tree) and the
 * dead-time generator's tick: with CR1 CKD=00, t_DTS = 1/f_tim = 5.952 ns.
 * Kept as the exact fraction 10000 ns / 1680 ticks so the ns<->DTG
 * conversions below are integer-exact. */
#define BSP_TIM_ADV_REF_CLOCK_HZ 168000000u
#define BSP_TIM_ADV_TDTS_NS_NUM  10000u
#define BSP_TIM_ADV_TDTS_NS_DEN  1680u

/* Register-level init of both outputs: kernel clock, PC6/PC7/PB14 AF3,
 * 1 kHz frame on both channels (PSC=167, ARR=999 -> 1 MHz tick), 50% duty,
 * PWM mode 1, CH2/CH2N driven as a complementary pair with 1 us dead time
 * (BDTR DTG=0x94), MOE on. Returns 0 = PASS (CCER/BDTR/ARR read-back),
 * -1 = mismatch. */
int bsp_tim_adv_init(void);

/* Encode a dead time in nanoseconds into the BDTR DTG field, walking the
 * RM0090 two-range encoding (<=756 ns: x t_DTS; up to 1512 ns: 2 x t_DTS
 * steps with the 0x80 bias; up to 3024: x8 / 0xC0; up to 6048: x16 / 0xE0).
 * Out-of-range values (>6048 ns or 0) are rejected with -1 and no write;
 * otherwise the value is rounded to the nearest legal tick count (max
 * quantization error = half a step). Returns 0 = written. */
int bsp_tim_adv_set_deadtime_ns(uint32_t ns);

/* Decode the CURRENT BDTR DTG field back to nanoseconds (the exact inverse
 * walk of the set function). The selftest uses this to prove the register
 * holds what set_deadtime_ns asked for, within one quantization step. */
uint32_t bsp_tim_adv_get_deadtime_ns(void);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. Steps:
 *   1 = DTG/CCER/MOE read-back (DTG byte = 0x94, CC1E|CC2E|CC2NE, MOE=1)
 *   2 = dead-time round-trip: get_deadtime_ns() vs the set value, |error|
 *       <= one quantization step (12 ns)
 *   3 = complementary polarity: CC2P=0 and CC2NP=0 (both phases active
 *       high -- the pair is symmetric, only the dead time separates them) */
int bsp_tim_adv_selftest(void);

#endif /* BSP_TIM_ADV_H */
