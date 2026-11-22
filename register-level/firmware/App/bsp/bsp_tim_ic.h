/**
 * @file bsp_tim_ic.h
 * @brief TIM2_CH2 (PB3) input capture frequency meter -- 32-bit period
 *        measurement with overflow compensation, register-level init,
 *        self-test.
 *
 * Teaching module (P3a wave, bsp-module-spec.md template -- see bsp_uart).
 * Full CubeMX config table / register walkthrough / verification recipe live
 * in the bsp_tim_ic.c file header.
 *
 * Ownership rule: TIM2 is in the IOC (base init only: PSC=0, ARR=max,
 * CEN=0, no channel config, no NVIC) -- after bsp_tim_ic_init() this
 * module OWNS TIM2's CR1/PSC/ARR/EGR/CCMR1/CCER/DIER and the PB3 pin
 * config. Never interleave HAL_TIM_IC_* on TIM2.
 */
#ifndef BSP_TIM_IC_H
#define BSP_TIM_IC_H

#include <stdint.h>

/* Kernel clock feeding TIM2 (APB1 x2 = 84 MHz on this clock tree). Every
 * returned Hz is f_kernel / period_ticks against this constant. */
#define BSP_TIM_IC_REF_CLOCK_HZ 84000000u

/* Register-level init of the capture channel: kernel clock, PB3 AF1,
 * TI2FP2 routed to IC2 (CC2S=01), filtered (IC2F=0011), rising edge,
 * counter free-running in 32-bit mode. Returns 0 = PASS (CCMR1/CCER/
 * PSC/ARR read-back matches), -1 = mismatch. */
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
 *   1 = capture config read-back (CCMR1 CC2S/IC2F/IC2PSC, CCER CC2E/CC2P,
 *       PSC=0, ARR=0xFFFFFFFF)
 *   2 = one measurement: no signal -> PASS anyway (0 Hz, SKIP-state
 *       semantics: an idle probe pin is not a defect); signal -> report
 *       the measured value via bsp_tim_ic_last_report() */
int bsp_tim_ic_selftest(void);

#endif /* BSP_TIM_IC_H */
