/**
 * @file bsp_dac.h
 * @brief DAC channel 1 (PA4 = DAC_OUT1) -- HAL edition: config verify,
 *        mV setpoint API, DOR read-back self-test.
 *
 * Teaching module (HAL edition of f407-periph-lab bsp_dac). CubeMX owns
 * the peripheral: MX_DAC_Init() (Core/Src/dac.c) configured DAC
 * channel OUT1 with trigger NONE (the register edition's TEN1=0
 * software mode: a DHR write reaches DOR after one APB1 cycle, no
 * SWTRIG needed) and output buffer ENABLED (BOFF1=0: low-Z drive for
 * the jumper wire). This module only VERIFIES that configuration and
 * drives the DAC through HAL_DAC_*. A3 rule (PLAN A3): no peripheral
 * control-register writes here; register access is read-only probes.
 *
 * Register-concept -> HAL-call mapping (vs the register edition):
 *   CR EN1=1 (channel wake)      -> HAL_DAC_Start(&hdac, DAC_CHANNEL_1)
 *   DHR12R1 write                -> HAL_DAC_SetValue(&hdac,
 *                                   DAC_CHANNEL_1, DAC_ALIGN_12B_R, code)
 *   DOR1 read-back               -> HAL_DAC_GetValue(&hdac, DAC_CHANNEL_1)
 *   MODER analog write (PA4)     -> CubeMX MspInit (verified read-only)
 *
 * Pin budget note unchanged: the W25Q128 flash uses a SOFTWARE NSS
 * (SSI), so PA4 is genuinely free for the DAC.
 */
#ifndef BSP_DAC_H
#define BSP_DAC_H

#include <stdint.h>

/* The conversion rail (mV): DACoutput = VREF+ * DOR / 4096 (RM0090
 * 14.3.5), VREF+ = VDDA = 3.3 V on this board. Codes map linearly, so
 * mV-setpoints above the rail clamp at full scale (see set_mv). */
#define BSP_DAC_VREF_MV 3300u

/* Verify the CubeMX-generated configuration: channel-1 CR shape (no
 * trigger, buffer enabled, no wave generator -- EN1 not yet set, the
 * channel wakes at the first Start), PA4 analog mode. Then wakes the
 * analog macrocell with HAL_DAC_Start (DHR reset default 0 -> PA4
 * idles at 0 V, the register edition's EN1=1 + tWAKEUP equivalent).
 * Returns 0 = PASS, -1 = mismatch/Start failure. */
int bsp_dac_init(void);

/* Set the channel-1 output in millivolts: code = round(mv*4096/3300),
 * clamped to the 12-bit full scale 4095 (the RM formula saturates at
 * VREF+ anyway). Loads the value via HAL_DAC_SetValue + HAL_DAC_Start
 * (trigger NONE: DHR -> DOR after one APB1 cycle). Returns the code
 * actually loaded. */
uint32_t bsp_dac_set_mv(uint32_t mv);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. Steps:
 *   1 = config read-back: channel-1 CR fields (TEN1/TSEL1/WAVE1/MAMP1/
 *       BOFF1 all 0 -- software mode, buffer on, no generator), PA4
 *       MODER analog
 *   2 = three-point DOR round-trip through the public API: set_mv
 *       0/1650/3300 must land as DOR1 = 0 / 2048 / 4095 read back via
 *       HAL_DAC_GetValue (the endpoints prove the rail mapping and the
 *       clamp; DOR is what the analog stage actually sees) */
int bsp_dac_selftest(void);

#endif /* BSP_DAC_H */
