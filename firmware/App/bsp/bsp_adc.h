/**
 * @file bsp_adc.h
 * @brief ADC1 (APB2) HAL-edition reader: config verify, 4-rank scan
 *        polling, mV conversion API, Vrefint supply check, self-test.
 *
 * Teaching module (HAL edition of f407-periph-lab bsp_adc). CubeMX owns
 * the peripheral: MX_ADC1_Init() (Core/Src/adc.c) configured ADC1 as a
 * 4-conversion regular scan on PCLK2/4:
 *   Rank 1 = CH8  (PB0) 3-cycle sample
 *   Rank 2 = CH10 (PC0) 3-cycle sample
 *   Rank 3 = CH14 (PC4) 3-cycle sample
 *   Rank 4 = VREFINT     480-cycle sample (high-impedance bandgap)
 * software trigger, single (non-continuous) mode, 12-bit right-aligned,
 * EOC per conversion (EOCS=1 via ADC_EOC_SINGLE_CONV -- enables overrun
 * detection too). This module only VERIFIES that configuration and drives
 * the ADC through HAL_ADC_*. A3 rule (PLAN A3): no peripheral
 * control-register writes here; register access is read-only probes.
 *
 * Register-concept -> HAL-call mapping (vs the register edition):
 *   SQR3 SQ1 re-point per read   -> fixed 4-rank scan; a read converts
 *                                   the whole scan and picks the rank
 *   CR2 SWSTART=1                -> HAL_ADC_Start(&hadc1)
 *   poll SR EOC (bounded)        -> HAL_ADC_PollForConversion(&hadc1, ms)
 *   read DR (clears EOC)         -> HAL_ADC_GetValue(&hadc1)
 *   SMPR sample-time writes      -> CubeMX per-channel config (verified
 *                                   read-only here)
 *   CCR TSVREFE=1                -> HAL_ADC_ConfigChannel(VREFINT)
 *                                   (sets TSVREFE; verified read-only)
 *   ADCPRE left at PCLK2/2=42MHz -> CubeMX picks ADC_CLOCK_SYNC_PCLK_DIV4
 *                                   = 21 MHz: in-spec against the F407
 *                                   36 MHz datasheet cap. The register
 *                                   edition's EP08 "honest caveat" is
 *                                   solved by configuration here.
 *
 * Vrefint calibration note: F4 has NO factory VREFINT_CAL (the
 * __HAL_ADC_CALC_VREFANALOG_VOLTAGE macro is F3/L4/G4-era; it needs
 * VREFINT_CAL_ADDR which F4 lacks). The 3.3 V rail assumption and the
 * mV math stay exactly as in the register edition.
 */
#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>

/* Channel selectors for bsp_adc_read_raw()/bsp_adc_read_mv(). The values
 * ARE the ADC channel numbers (Rank -> channel mapping fixed by the
 * CubeMX sequence) -- named once so no call site passes a bare magic
 * number. */
#define BSP_ADC_CH_IN8_PB0   8u  /* ADC1_IN8  = PB0 (loop test pin)      */
#define BSP_ADC_CH_IN10_PC0  10u /* ADC1_IN10 = PC0                      */
#define BSP_ADC_CH_IN14_PC4  14u /* ADC1_IN14 = PC4                      */
#define BSP_ADC_CH_VREFINT   17u /* ADC1_IN17 internal reference         */

/* Verify the CubeMX-generated configuration with read-only probes:
 * handle shape (4-rank scan, single mode, 12-bit right), CR2 ADON,
 * CCR TSVREFE (Vrefint path woken by the rank-4 channel config), SQR1
 * L=3 (four conversions), SMPR sample-time fields (3/3/3/480), and the
 * analog MODER mode on PB0/PC0/PC4. Returns 0 = PASS, -1 = mismatch. */
int bsp_adc_init(void);

/* One software-triggered scan pass picking out 'ch' (one of the four
 * selectors above). Sequence: HAL_ADC_Start(&hadc1) fires the whole
 * 4-rank scan; every rank is then polled with HAL_ADC_PollForConversion
 * and read with HAL_ADC_GetValue in order (the EOC-sequencing
 * discipline -- ranks must be captured as they land, EOCS=1 makes a
 * lost rank visible as OVR). Returns the 12-bit right-aligned code
 * 0..4095, or 0 on timeout/unknown channel/overrun (0 is also the legal
 * code for 0 V -- the mV API documents the caveat). */
uint32_t bsp_adc_read_raw(uint32_t ch);

/* One conversion converted to millivolts against the board's VREF+ =
 * VDDA = 3.3 V rail: mv = raw * 3300 / 4096 (integer math, max operand
 * 4095*3300 fits uint32_t). Quantization step = 3300/4096 = 0.806 mV.
 * F4 note: no factory VREFINT_CAL, so the rail stays an assumption --
 * the Vrefint selftest window is what guards it. */
uint32_t bsp_adc_read_mv(uint32_t ch);

/* One-line report of the LAST selftest run, for the menu to append to its
 * verdict line: "(vrefint NNNNmV, vdda NNNNmV)" -- vdda is back-solved
 * from the measured Vrefint code against the 1.21 V typical value. Empty
 * string before the first run. Data-not-print keeps the module free of
 * console dependencies (bsp-module-spec rule #7). */
const char *bsp_adc_last_report(void);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. Steps:
 *   1 = config read-back: handle shape (scan/4 ranks/single/12-bit),
 *       CR2 ADON, CCR TSVREFE, SQR1 L=3, SMPR 3/3/3/480 fields, MODER
 *       analog on PB0/PC0/PC4
 *   2 = one full 4-rank scan: HAL_ADC_Start + HAL_ADC_PollForConversion
 *       per rank (4 conversions, HAL_ADC_GetValue each, in order) with
 *       no overrun flagged (the EOC-sequencing proof), the Vrefint
 *       reading inside [1150,1280] mV, and the report filled
 *       "(vrefint NNNNmV, vdda NNNNmV)". External pins (PB0/PC0/PC4)
 *       float at either rail on an open header -- only the 12-bit
 *       domain bounds them, same tolerance stance as the old edition. */
int bsp_adc_selftest(void);

#endif /* BSP_ADC_H */
