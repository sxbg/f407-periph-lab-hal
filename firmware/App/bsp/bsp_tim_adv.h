/**
 * @file bsp_tim_adv.h
 * @brief TIM8 advanced-timer complementary outputs + dead-time generator
 *        (CH1=PC6 plain PWM; CH2=PC7 / CH2N=PB14 complementary pair) --
 *        HAL channel/break-deadtime config, dead-time API, self-test.
 *
 * Teaching module (HAL edition of f407-periph-lab bsp_tim_adv, P3b wave --
 * see bsp_uart). Full CubeMX config table / HAL mapping / verification
 * recipe (including the EP14 dead-time derivation this module implements)
 * live in the bsp_tim_adv.c file header.
 *
 * Ownership rule (A3): CubeMX owns TIM8's clock, base timebase (PSC=0,
 * ARR=65535) and the PC6/PC7/PB14 pins (MspInit + MspPostInit). This
 * module owns the channel/dead-time SHAPE only, written exclusively
 * through HAL_TIM_PWM_ConfigChannel / HAL_TIMEx_ConfigBreakDeadTime /
 * HAL_TIM_PWM_Start / HAL_TIMEx_PWMN_Start and the __HAL_TIM_SET_* macros.
 * No peripheral control-register writes live here; register reads
 * (PSC/CCER/BDTR/CR1 probes) are allowed as verification and carry the
 * "read-only (A3 exemption)" comment.
 */
#ifndef BSP_TIM_ADV_H
#define BSP_TIM_ADV_H

#include <stdint.h>

/* Kernel clock feeding TIM8 (APB2 x2 = 168 MHz on this clock tree) and the
 * dead-time generator's tick: with CKD=00 (IOC ClockDivision=DIV1),
 * t_DTS = 1/f_tim = 5.952 ns. Kept as the exact fraction 10000 ns / 1680
 * ticks so the ns<->DTG conversions below are integer-exact. */
#define BSP_TIM_ADV_REF_CLOCK_HZ 168000000u
#define BSP_TIM_ADV_TDTS_NS_NUM  10000u
#define BSP_TIM_ADV_TDTS_NS_DEN  1680u

/* HAL init of both outputs: 1 kHz frame on both channels (PSC=167,
 * ARR=999 -> 1 MHz tick), 50% duty, PWM mode 1, CH2/CH2N driven as a
 * complementary pair with 1 us dead time (BDTR DTG=0x94 via
 * HAL_TIMEx_ConfigBreakDeadTime), MOE on (set by the Start calls).
 * Returns 0 = PASS (CCER/BDTR/PSC/ARR read-back), -1 = mismatch or a HAL
 * call failed. Idempotent. */
int bsp_tim_adv_init(void);

/* Encode a dead time in nanoseconds into the BDTR DTG field, walking the
 * RM0090 two-range encoding (<=756 ns: x t_DTS; up to 1512 ns: 2 x t_DTS
 * steps with the 0x80 bias; up to 3024: x8 / 0xC0; up to 6048: x16 / 0xE0),
 * applied through HAL_TIMEx_ConfigBreakDeadTime. Out-of-range values
 * (>6048 ns or 0) are rejected with -1 and no write; otherwise the value
 * is rounded to the nearest legal tick count (max quantization error =
 * half a step). Returns 0 = written. */
int bsp_tim_adv_set_deadtime_ns(uint32_t ns);

/* Decode the CURRENT BDTR DTG field back to nanoseconds (a read-only probe
 * of the field the set function wrote -- no HAL getter exists for DTG).
 * The selftest uses this to prove the register holds what set_deadtime_ns
 * asked for, within one quantization step. */
uint32_t bsp_tim_adv_get_deadtime_ns(void);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. Steps:
 *   1 = DTG/CCER/MOE read-back via read-only probes (DTG byte = 0x94,
 *       CC1E|CC2E|CC2NE, MOE=1, CEN=1)
 *   2 = dead-time round-trip: get_deadtime_ns() vs the set value, |error|
 *       <= one quantization step (12 ns)
 *   3 = complementary polarity: CC2P=0 and CC2NP=0 (both phases active
 *       high -- the pair is symmetric, only the dead time separates them) */
int bsp_tim_adv_selftest(void);

#endif /* BSP_TIM_ADV_H */
