/**
 * @file bsp_tim_encoder.h
 * @brief TIM3 quadrature encoder interface (PB4=CH1 A-phase, PB5=CH2
 *        B-phase) -- x4 decoding, HAL encoder interface, count/speed APIs,
 *        self-test.
 *
 * Teaching module (HAL edition of f407-periph-lab bsp_tim_encoder, P3b
 * wave -- see bsp_uart). Full CubeMX config table / HAL mapping /
 * verification recipe live in the bsp_tim_encoder.c file header.
 *
 * Ownership rule (A3): CubeMX owns TIM3's clock, base timebase (PSC=0,
 * ARR=65535, ARPE=ENABLE) and the PB4/PB5 pins (MspInit, ENC_A/ENC_B).
 * This module owns the encoder SHAPE only, written exclusively through
 * HAL_TIM_Encoder_Init / HAL_TIM_Encoder_Start (plus one HAL_GPIO_Init to
 * restore the idle-stable pull-ups the IOC leaves off -- see the .c
 * header). No peripheral control-register writes live here; register reads
 * (SMCR/CCMR1/CCER/ARR/CNT probes) are allowed as verification and carry
 * the "read-only (A3 exemption)" comment.
 */
#ifndef BSP_TIM_ENCODER_H
#define BSP_TIM_ENCODER_H

#include <stdint.h>

/* Pulses per shaft revolution of the encoder. This is the parameter the
 * rpm math divides by; the default 1024 matches the lab's quadrature
 * encoder. Override at runtime with bsp_tim_encoder_set_ppr(). */
#define BSP_TIM_ENCODER_PPR_DEFAULT 1024u

/* HAL encoder-interface init: TI1FP1/TI2FP2 mapped to their pins (CC1S/
 * CC2S =01), x4 counting (EncoderMode TI12 -> SMCR SMS=011), filtered
 * inputs (IC1F/IC2F=0011), pull-up inputs re-applied via HAL_GPIO_Init,
 * ARR=65535 free-wrapping window kept from CubeMX. Returns 0 = PASS
 * (SMCR/CCMR1/CCER/ARR read-back matches), -1 = mismatch. Idempotent. */
int bsp_tim_encoder_init(void);

/* Raw 16-bit counter (0..65535). In encoder mode the hardware counts
 * quadrature edges both ways and wraps 0<->65535 (ARR=65535), so this IS
 * the position modulo one 16-bit lap -- the EP23 wrap-around lesson. */
uint32_t bsp_tim_encoder_read_count(void);

/* Set the pulses-per-revolution parameter used by the rpm conversion.
 * Accepts 1..65535 (0 is rejected, -1, no change). Returns 0 = applied. */
int bsp_tim_encoder_set_ppr(uint32_t ppr);

/* Speed in shaft rpm over a win_ms window (>=1): sample CNT, wait win_ms,
 * sample again, and convert the signed 16-bit wrap-safe delta:
 *   rpm = delta * 60000 / (4 * PPR * win_ms)
 * (4x because SMS=011 counts every TI1/TI2 edge). Negative rotation gives
 * negative rpm, no signal gives 0. Keep win_ms small enough that the
 * mechanical speed never exceeds |32767| counts per window -- beyond that
 * the 16-bit wrap aliases the sign (walkthrough, EP23). */
int32_t bsp_tim_encoder_speed_rpm(uint32_t win_ms);

/* One-line report of the LAST selftest run, for the menu to append to its
 * verdict line (the module itself has no console dependency). Returns
 * "(idle)" when the shaft stood still (SKIP semantics: no encoder wired is
 * not a defect) or "(moved, N cts/300ms)" when edges arrived; "" before the
 * first run. Returning data instead of printing keeps bsp modules mutually
 * independent (bsp-module-spec rule #7). */
const char *bsp_tim_encoder_last_report(void);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. Steps:
 *   1 = config read-back via read-only probes: SMCR SMS=011, CCMR1 CC1S/
 *       CC2S=01 + filters, CCER CC1E|CC2E with clean polarities,
 *       ARR=65535, CEN=1
 *   2 = stationary check: CNT delta must be 0 across a 200 ms window
 *       (pull-ups hold both lines high; the filter would eat glitch edges)
 *   3 = rotation probe: CNT delta over a 300 ms window; 0 -> idle report,
 *       nonzero -> counts reported; both outcomes PASS */
int bsp_tim_encoder_selftest(void);

#endif /* BSP_TIM_ENCODER_H */
