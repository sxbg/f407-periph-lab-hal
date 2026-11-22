/**
 * @file bsp_tim_basic.h
 * @brief TIM5 pure timebase -- 1 Hz update source counted by POLLING (HAL
 *        edition; the old TIM5-IRQ + TIM6/TIM7-poll trio collapses to TIM5
 *        alone because TIM6/TIM7 are not in this project's IOC).
 *
 * Teaching module (HAL edition of f407-periph-lab bsp_tim_basic, P3a wave
 * -- see bsp_uart). CubeMX config / HAL mapping / verification recipe live
 * in the bsp_tim_basic.c file header.
 *
 * W4b HAL edition: TIM5 polled, IRQ demo deferred to W6 when NVIC wiring
 * lands. HAL_TIM_Base_Start (not _IT) arms the counter; DIER.UIE stays
 * clear and no NVIC line is touched -- the update flag is drained with
 * __HAL_TIM_GET_FLAG / __HAL_TIM_CLEAR_FLAG.
 *
 * Ownership rule (A3): CubeMX owns TIM5's clock gate and base init
 * (MX_TIM5_Init: PSC=0, ARR=max, CEN=0). This module owns the timebase
 * SHAPE via the __HAL_TIM_SET_* macros + HAL_TIM_Base_Start. No peripheral
 * control-register writes live here; register reads (PSC/ARR/CR1/DIER
 * probes) are allowed as verification and carry the
 * "read-only (A3 exemption)" comment.
 */
#ifndef BSP_TIM_BASIC_H
#define BSP_TIM_BASIC_H

#include <stdint.h>

/* Timebase init: TIM5 1 Hz update (PSC=8399, ARR=9999, derivation in the
 * .c header), interrupts deliberately NOT enabled (polled lesson). Returns
 * 0 = PASS (PSC/ARR/CEN read-back matches, UIE verified clear), -1 =
 * mismatch or a HAL call failed. Idempotent. */
int bsp_tim_basic_init(void);

/* Seconds counter fed by the TIM5 update events (one tick per overflow).
 * POLLED edition: each call drains a pending update flag and counts it, so
 * it keeps true time only while someone calls it at least once per second
 * (menu/selftest pace). Wraps at 2^32 seconds (~136 years). The
 * HAL_TIM_Base_Start_IT + HAL_TIM_PeriodElapsedCallback edition returns in
 * W6 when the NVIC wiring lands. */
uint32_t bsp_tim_basic_uptime_seconds(void);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. Steps:
 *   1 = TIM5 timebase: config read-back, then the update counter must
 *       advance by >= 1 inside a 1.2 s window
 *   2 = update-flag contract: UIF must set within a 1.2 s window, must
 *       clear via __HAL_TIM_CLEAR_FLAG (the HAL spelling of the rc_w0
 *       write), and must set again within another window (periodic, not
 *       a one-shot)
 *   3 = cadence: after a drained flag, exactly one update lands inside a
 *       1.2 s window (the 1 Hz pace holds across consecutive windows) */
int bsp_tim_basic_selftest(void);

/* Verdict-line suffix from the last selftest run (step-1 diagnostics),
 * NULL-terminated, "" when no diagnostic. Same pattern as
 * bsp_tim_ic_last_report. */
const char *bsp_tim_basic_last_report(void);

#endif /* BSP_TIM_BASIC_H */
