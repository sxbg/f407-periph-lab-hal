/**
 * @file bsp_adc.h
 * @brief ADC1 (APB2) single-channel polling reader -- register-level init,
 *        mV conversion API, Vrefint supply check, self-test.
 *
 * Teaching module (P3e wave, bsp-module-spec.md template -- see bsp_uart).
 * Full config table / register walkthrough / verification recipe live
 * in the bsp_adc.c file header.
 *
 * Ownership rule: the ADC IP is NOT in the IOC (knowledge #13: adapter-owned
 * peripherals stay out; the analog-capable PINS are, as GPIO_Analog
 * placeholders). After bsp_adc_init() this module OWNS ADC1 (SR/CR1/CR2/
 * SMPR1/SMPR2/SQR1/SQR3) and the ADC_CCR TSVREFE bit, plus the PB0/PC0/PC4
 * pin mode (analog). Never interleave HAL_ADC_* on ADC1.
 */
#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>

/* Channel selectors for bsp_adc_read_raw()/bsp_adc_read_mv(). The values
 * ARE the ADC channel numbers (SQR3 SQ1[4:0] field content) -- named once
 * so no call site passes a bare magic number. */
#define BSP_ADC_CH_IN8_PB0   8u  /* ADC1_IN8  = PB0 (loop test pin)      */
#define BSP_ADC_CH_IN10_PC0  10u /* ADC1_IN10 = PC0                      */
#define BSP_ADC_CH_IN14_PC4  14u /* ADC1_IN14 = PC4                      */
#define BSP_ADC_CH_VREFINT   17u /* ADC1_IN17 internal reference (TSVREFE)*/

/* Register-level init: ADC1 + GPIOB/GPIOC clocks, analog mode on PB0/PC0/
 * PC4, 480-cycle sampling on all four channels, 1-conversion regular
 * sequence (SQR1 L=0), CR2 ADON=1 (single-conversion polling base), CCR
 * TSVREFE=1 (Vrefint powered). Returns 0 = PASS (SMPR/CR2/CCR read-back
 * matches), -1 = mismatch. */
int bsp_adc_init(void);

/* One software-triggered conversion on 'ch' (one of the four selectors
 * above). Sequence: SQR3 <- ch, clear stale EOC, CR2 SWSTART=1 (RM0090
 * 13.3.4 single-conversion start), poll SR EOC (bounded), read DR (the
 * DR read also clears EOC -- RM 13.13.1). Returns the 12-bit right-
 * aligned code 0..4095, or 0 on timeout/unknown channel (0 is also the
 * legal code for 0 V -- the mV API documents the caveat). */
uint32_t bsp_adc_read_raw(uint32_t ch);

/* One conversion converted to millivolts against the board's VREF+ =
 * VDDA = 3.3 V rail: mv = raw * 3300 / 4096 (integer math, max operand
 * 4095*3300 fits uint32_t). Quantization step = 3300/4096 = 0.806 mV. */
uint32_t bsp_adc_read_mv(uint32_t ch);

/* One-line report of the LAST selftest run, for the menu to append to its
 * verdict line: "(vrefint NNNNmV, vdda NNNNmV)" -- vdda is back-solved
 * from the measured Vrefint code against the 1.21 V typical value. Empty
 * string before the first run. Data-not-print keeps the module free of
 * console dependencies (bsp-module-spec rule #7). */
const char *bsp_adc_last_report(void);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. Steps:
 *   1 = register read-back: RCC ADC1EN, CR2 ADON, SMPR 480-cycle fields
 *       (SMP8/SMP10/SMP14/SMP17), SQR1 L=0, CCR TSVREFE, MODER analog on
 *       PB0/PC0/PC4
 *   2 = Vrefint conversion: code 0 (timeout) fails; the mV value computed
 *       against the 3.3 V rail must land in [1150,1280] (Vrefint typical
 *       1.21 V, +/- tolerance covering VDDA spread) */
int bsp_adc_selftest(void);

#endif /* BSP_ADC_H */
