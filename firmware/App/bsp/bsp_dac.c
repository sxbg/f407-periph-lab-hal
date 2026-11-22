/**
 * @file    bsp_dac.c
 * @brief   DAC channel 1 on PA4 (DAC_OUT1) -- HAL edition software-mode
 *          module with DOR read-back self-test -- see bsp_dac.h for the
 *          contract and the register->HAL map.
 ******************************************************************************
 * Console-free module (bsp-module-spec rule #7): selftest returns the
 * failing step number; the shell prints the verdict.
 */
#include "bsp_dac.h"

#include "main.h" /* DAC_OUT1 pin label macros from CubeMX */
#include "stm32f4xx_hal.h"
#include "dac.h" /* hdac from MX_DAC_Init (CubeMX-generated) */

/* ---- module constants ----------------------------------------------------- */

/* The channel-1 CR fields MX_DAC_Init must have left at 0: TEN1 (no
 * trigger), TSEL1, WAVE1/MAMP1 (no generator), BOFF1 (buffer ENABLED
 * = bit clear). HAL_DAC_ConfigChannel writes exactly this shape and
 * deliberately leaves EN1 = 0 -- the analog macrocell wakes at the
 * first HAL_DAC_Start (the old edition set CR = EN1 in one write). */
#define BSP_DAC_CR_SHAPE_MASK (DAC_CR_TEN1 | DAC_CR_TSEL1 | \
                               DAC_CR_WAVE1 | DAC_CR_MAMP1 | DAC_CR_BOFF1)

/* Self-test step numbers (the value bsp_dac_selftest returns). */
#define BSP_DAC_ST1_READBACK 1u /* CR + pin-mode read-back  */
#define BSP_DAC_ST2_DOR      2u /* 3-point DOR round-trip   */

/* DHR -> DOR takes one APB1 cycle with trigger NONE (RM0090 14.3.4);
 * HAL polls with tick granularity, so 1 ms is the portable settle. */
#define BSP_DAC_DOR_SETTLE_MS 1u

/* ---- private helpers -------------------------------------------------------- */

/* The config probes shared by init and selftest step 1. The CR + MODER
 * reads are read-only (A3 read-only exemption): verifying the
 * HAL-owned configuration, no register writes here.
 * Returns 0 = config intact, -1 = mismatch. */
static int dac_config_intact(void)
{
    /* Channel-1 shape: software trigger, buffer on, no wave. */
    if ((hdac.Instance->CR & BSP_DAC_CR_SHAPE_MASK) != 0u) {
        return -1;
    }
    /* PA4 analog mode (MspInit GPIO block): MODER4 [9:8] = 11. */
    if ((GPIOA->MODER & GPIO_MODER_MODER4_Msk) != GPIO_MODER_MODER4_Msk) {
        return -1;
    }
    return 0;
}

/* Load 'code' through the HAL and hand back the DOR1 the analog stage
 * actually sees (HAL_DAC_GetValue). One APB1-cycle settle is included
 * (the old edition's read-spin after the DHR12R1 write). */
static uint32_t dac_load_and_read(uint32_t code)
{
    (void)HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, code);
    (void)HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
    HAL_Delay(BSP_DAC_DOR_SETTLE_MS);
    return HAL_DAC_GetValue(&hdac, DAC_CHANNEL_1) & 0x0FFFu;
}

/* ---- public API ------------------------------------------------------------ */

int bsp_dac_init(void)
{
    /* Handle + config shape (read-only; see dac_config_intact). */
    if (hdac.Instance != DAC) {
        return -1;
    }
    if (dac_config_intact() != 0) {
        return -1;
    }

    /* Pin self-config (knowledge #17): the .ioc loader leaves PA4 in an
     * AF mode it never drives -- the DAC analog output needs MODER=11
     * (analog, the COMP_DAC1_group electricals). HAL call, A3-clean. */
    {
        GPIO_InitTypeDef da = {0};
        da.Pin   = DAC_OUT1_Pin;
        da.Mode  = GPIO_MODE_ANALOG;
        da.Pull  = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &da);
    }

    /* Wake the analog macrocell: HAL_DAC_Start sets EN1 (the old
     * edition's CR EN1=1), then 1 ms covers the tWAKEUP startup window
     * (RM0090 14.3.1, us-class) before the first real setpoint. DHR is
     * at reset default 0, so PA4 idles at 0 V after this. */
    if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK) {
        return -1;
    }
    HAL_Delay(BSP_DAC_DOR_SETTLE_MS);
    return 0;
}

uint32_t bsp_dac_set_mv(uint32_t mv)
{
    /* code = round(mv * 4096 / 3300) = (mv*4096 + 1650) / 3300, clamped
     * to the 12-bit full scale. Max operand: mv*4096 with mv <= 4095
     * stays under 2^24; larger mv only clamps harder (the sum fits
     * uint32_t up to mv ~ 1M, beyond which the clamp already applies --
     * and the RM formula saturates at VREF+ regardless). */
    uint32_t code = ((mv * 4096u) + (BSP_DAC_VREF_MV / 2u)) / BSP_DAC_VREF_MV;
    if (code > 4095u) {
        code = 4095u; /* full scale: the RM formula's rail saturation */
    }
    (void)dac_load_and_read(code); /* DOR check is the selftest's job */
    return code;
}

int bsp_dac_selftest(void)
{
    /* Fresh init first (idempotent, menu-driven selftest pattern). */
    if (bsp_dac_init() != 0) {
        return (int)BSP_DAC_ST1_READBACK;
    }

    /* Step 1: CR + pin-mode read-back (see the init contract). */
    if (dac_config_intact() != 0) {
        return (int)BSP_DAC_ST1_READBACK;
    }

    /* Step 2: three-point set->DOR round-trip THROUGH the public API
     * (the rounding math is part of the contract; HAL_DAC_GetValue is
     * the window into the conversion datapath -- DHR is a holding
     * register, DOR is what the analog stage actually sees).
     *   0    mV -> code 0    (zero scale)
     *   1650 mV -> code 2048 (mid-rail, exact)
     *   3300 mV -> code 4095 (full scale: the clamp path) */
    if ((bsp_dac_set_mv(0u) != 0u) || (dac_load_and_read(0u) != 0u)) {
        return (int)BSP_DAC_ST2_DOR;
    }
    if ((bsp_dac_set_mv(1650u) != 2048u) ||
        (dac_load_and_read(2048u) != 2048u)) {
        return (int)BSP_DAC_ST2_DOR;
    }
    if ((bsp_dac_set_mv(3300u) != 4095u) ||
        (dac_load_and_read(4095u) != 4095u)) {
        return (int)BSP_DAC_ST2_DOR;
    }
    return 0; /* all steps passed */
}
