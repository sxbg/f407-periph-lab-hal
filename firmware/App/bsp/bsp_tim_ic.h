/**
 * @file bsp_tim_ic.h
 * @brief TIM2_CH2 (PB3) input capture frequency meter -- 32-bit period
 *        measurement with overflow compensation, HAL capture channel,
 *        self-test.
 *
 * Teaching module (HAL edition of f407-periph-lab bsp_tim_ic, P3a wave --
 * see bsp_uart). Full CubeMX config table / HAL mapping / verification
 * recipe live in the bsp_tim_ic.c file header.
 *
 * Ownership rule (A3): CubeMX owns TIM2's clock, base timebase (PSC=0,
 * ARR=0xFFFFFFFF) and the PB3 pin (MspInit, no pull). This module owns the
 * capture channel SHAPE only, written exclusively through
 * HAL_TIM_IC_ConfigChannel / HAL_TIM_IC_Start. No peripheral
 * control-register writes live here; register reads (CCMR1/CCER/PSC/ARR
 * probes) are allowed as verification and carry the
 * "read-only (A3 exemption)" comment.
 */
#ifndef BSP_TIM_IC_H
#define BSP_TIM_IC_H

#include <stdint.h>

/* Kernel clock feeding TIM2 (APB1 x2 = 84 MHz on this clock tree). Every
 * returned Hz is f_kernel / period_ticks against this constant. */
#define BSP_TIM_IC_REF_CLOCK_HZ 84000000u

/* HAL capture-channel init: TIM2_CH2 <- TI2PB3, direct (CC2S=01),
 * filtered (ICFilter=3), rising edge, every edge (ICPrescaler=DIV1),
 * 32-bit free-run timebase kept from CubeMX. Returns 0 = PASS (CCMR1/
 * CCER/PSC/ARR read-back matches), -1 = mismatch. Idempotent. */
int bsp_tim_ic_init(void);

/* Measure the frequency on PB3. Two rising edges, period = edge-to-edge
 * ticks (overflow-compensated, see walkthrough), freq = 84 MHz / period.
 * The whole measurement shares a 1 s window: no edge pair inside it
 * returns 0 (= "no signal" / period longer than the window). Frequencies
 * of roughly 1 Hz .. 84 MHz are representable; below ~2 Hz the window
 * rarely catches two edges, so slow signals also report 0. */
uint32_t bsp_tim_ic_measure_freq(void);

/* One-line report of the LAST selftest run, for the menu to append to its
 * verdict line (the module itself has no console dependency). Returns
 * "(no signal, 0 Hz)" or "(measured N Hz)"; also "" before the first run.
 * Returning data instead of printing keeps bsp modules mutually
 * independent (bsp-module-spec rule #7). */
const char *bsp_tim_ic_last_report(void);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. Steps:
 *   1 = capture config read-back via read-only probes (CCMR1 CC2S/IC2F/
 *       IC2PSC, CCER CC2E/CC2P, PSC=0, ARR=0xFFFFFFFF -- the shape
 *       HAL_TIM_IC_ConfigChannel wrote)
 *   2 = one measurement: no signal -> PASS anyway (0 Hz, SKIP-state
 *       semantics: an idle probe pin is not a defect); signal -> report
 *       the measured value via bsp_tim_ic_last_report() */
int bsp_tim_ic_selftest(void);

#endif /* BSP_TIM_IC_H */
