/**
 * @file bsp_dac.h
 * @brief DAC channel 1 (PA4 = DAC_OUT1) -- register-level init, mV setpoint
 *        API, DOR read-back self-test.
 *
 * Teaching module (P3e wave, bsp-module-spec.md template -- see bsp_uart).
 * Full config table / register walkthrough / verification recipe live
 * in the bsp_dac.c file header.
 *
 * Ownership rule: the DAC IP is NOT in the IOC (knowledge #13: adapter-owned
 * peripherals stay out; the PA4 pin is, as a GPIO_Analog placeholder).
 * After bsp_dac_init() this module OWNS DAC->CR (channel 1 fields) and the
 * PA4 pin mode (analog). Never interleave HAL_DAC_* on the DAC. Note: the
 * W25Q128 flash on this board uses a SOFTWARE NSS (SSI), so PA4 is genuinely
 * free for the DAC -- the pin-budget matrix says so.
 */
#ifndef BSP_DAC_H
#define BSP_DAC_H

#include <stdint.h>

/* The conversion rail (mV): DACoutput = VREF+ * DOR / 4096 (RM0090
 * 14.3.5), VREF+ = VDDA = 3.3 V on this board. Codes map linearly, so
 * mV-setpoints above the rail clamp at full scale (see set_mv). */
#define BSP_DAC_VREF_MV 3300u

/* Register-level init: APB1 DACEN clock gate, PA4 to analog mode, channel 1
 * enabled in software mode (TEN1=0: no trigger, DHR->DOR auto after one
 * APB1 cycle), output buffer ON (BOFF1=0), no wave generator. Returns
 * 0 = PASS (CR read-back matches), -1 = mismatch. */
int bsp_dac_init(void);

/* Set the channel-1 output in millivolts: code = round(mv*4096/3300),
 * clamped to the 12-bit full scale 4095 (the RM formula saturates at
 * VREF+ anyway). Writes DHR12R1; with TEN1=0 the value lands in DOR1
 * after one APB1 cycle (RM 14.3.4). Returns the code actually written. */
uint32_t bsp_dac_set_mv(uint32_t mv);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. Steps:
 *   1 = CR read-back: EN1=1, TEN1=0 (software mode), BOFF1=0 (buffer on),
 *       WAVE1=00 (no generator); PA4 MODER analog
 *   2 = three-point DOR round-trip: set_mv 0/1650/3300 must land as
 *       DOR1 = 0 / 2048 / 4095 (the endpoints prove the rail mapping and
 *       the clamp) */
int bsp_dac_selftest(void);

#endif /* BSP_DAC_H */
