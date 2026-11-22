/**
 ******************************************************************************
 * @file    bsp_dac.c
 * @brief   DAC channel 1 on PA4 (DAC_OUT1) -- register-level software-mode
 *          teaching module with DOR read-back self-test (EP09).
 ******************************************************************************
 * @addtogroup BSP_DAC
 * @{
 *
 * == CUBEMX CONFIG (absent from f407-periph-lab.ioc -- by design) ==========
 * | Field             | Value (module truth)                 | Why
 * |-------------------|--------------------------------------|-------------------
 * | DAC in IOC        | NOT PRESENT as an IP                 | knowledge #13:
 * |                   |                                      | adapter-owned
 * |                   |                                      | peripherals stay
 * |                   |                                      | out of the .ioc
 * | Pin (IOC truth)   | PA4, label DAC_OUT1, GPIO_Analog     | locked analog
 * |                   |                                      | placeholder --
 * |                   |                                      | this module owns
 * |                   |                                      | its MODER
 * | (PA4 freedom)     | SPI1 NSS is virtual (SSM=1, SSI=1 in | the pin-budget
 * |                   | bsp_spi_flash)                       | matrix released
 * |                   |                                      | PA4 to the DAC
 * | OUT1 only         | OUT2 = PA5 is taken (W25Q128 CLK)    | single-channel
 * | Trigger           | NONE (TEN1=0)                        | software mode:
 * |                   |                                      | DHR -> DOR auto
 * | Output buffer     | ENABLED (BOFF1=0)                    | low-Z drive for
 * |                   |                                      | the jumper wire
 * | Wave generation   | NONE (WAVE1=00)                      | DC setpoint API
 * | NVIC              | DAC_IRQn off                         | nothing to poll
 *
 * == REGISTER WALKTHROUGH (RM0090 ch.14 "Digital-to-analog converter") =====
 * Clock gate: DAC sits on APB1 -- RCC->APB1ENR bit29 (DACEN). The digital
 *   interface runs from PCLK1 always; EN1 (below) gates the ANALOG
 *   macrocell (RM 14.3.1 note).
 * Pins (RM0090 ch.8.4.1): PA4 -> MODER4 [9:8] = 11 (analog mode; the
 *   output buffer drives the pad directly, no digital stages involved).
 *   Pulls stay off -- a pull on a driven analog output is pointless load.
 * CR (14.5.1), the channel-1 control bits this module owns:
 *   bit0 EN1   = 1  channel enable; the analog part needs a startup time
 *                  tWAKEUP (us class) after this -- hence the settle spin.
 *   bit1 BOFF1 = 0  OUTPUT BUFFER ENABLED (BOFFx=0 keeps it on). The
 *                  buffer drops the output impedance so a dupont wire +
 *                  ADC input load cannot drag the setpoint; the price is
 *                  the rail margin: buffered output swings ~0.2 V above
 *                  GND to ~VDDA-0.2 V, so 0/3300 mV endpoints saturate
 *                  slightly INSIDE the rails. The loop test picks
 *                  500/1650/2800 mV -- all comfortably inside.
 *   bit2 TEN1  = 0  TRIGGER DISABLED = software mode. With TEN1=0 a
 *                  DHR12R1 write reaches DOR1 after ONE APB1 cycle
 *                  (RM 14.3.4) -- no SWTRIG, no timer event needed.
 *   bits [6:5] WAVE1 = 00  no noise/triangle generator (those need a
 *                  trigger too; they are the EP09 waveform lesson's next
 *                  step beyond DC).
 *   bits [3:2] TSEL1 -- don't care while TEN1=0 (only read with a
 *                  trigger selected).
 * Data path (14.3.3/14.3.4): DOR1 cannot be written -- software loads
 *   DHR12R1 [11:0] (12-bit right-aligned), hardware mirrors it into the
 *   internal DHR1 then DOR1. Reading DOR1 back is the proof the value
 *   arrived (and the selftest's whole mechanism).
 * Output (14.3.5): DACoutput = VREF+ * DOR/4096, linear 0..VREF+. Code
 *   math for set_mv: code = round(mv * 4096 / 3300), +1650 in the
 *   numerator for round-half-up; mv > rail clamps at 4095 (full scale =
 *   the formula's saturation). Quantization step = 3300/4096 = 0.806 mV.
 * == HOW TO VERIFY =========================================================
 * menu> dac
 *   PASS dac            (steps: 1 CR read-back, 2 DOR round-trip 0/1650/3300)
 *   FAIL dac step<N> maps to the steps above.
 * DMM on PA4: 'dac' leaves 3300 mV on the pin (last selftest point).
 * Closed loop: jumper PA4 -> PB0 and run menu 'dac_adc_loop' -- the ADC
 *   measures back what the DAC set, three points, |delta| <= 60 mV.
 */
#include "bsp_dac.h"

#include "hal_port.h" /* family seam: pulls in the CMSIS device header */

/* ---- module constants ----------------------------------------------------- */

/* Post-EN1 settle spin (~2000 iterations ~= 12 us at 168 MHz): covers the
 * tWAKEUP analog startup window of RM0090 14.3.1 before the first DHR
 * write (belt-and-braces; menu latency already exceeds it). */
#define BSP_DAC_WAKEUP_SPIN 2000u

/* Self-test step numbers (the value bsp_dac_selftest returns). */
#define BSP_DAC_ST1_READBACK 1u /* CR + pin-mode read-back */
#define BSP_DAC_ST2_DOR      2u /* 3-point DOR round-trip  */

/* ---- private helpers -------------------------------------------------------- */

/* One DHR12R1 write plus the mandatory DOR1 settle (one APB1 cycle,
 * RM 14.3.4 -- a short read-spin is the portable way to guarantee it).
 * Returns the DOR1 value observed. */
static uint32_t dac_write_dhr_read_dor(uint32_t code)
{
    volatile uint32_t spin;
    DAC->DHR12R1 = code & 0x0FFFu; /* 12-bit right-aligned data (14.3.3) */
    spin = 4u;                     /* >= 1 APB1 cycle (42 MHz): 4 reads */
    while (spin > 0u) {
        spin--;
    }
    return DAC->DOR1 & 0x0FFFu;
}

/* ---- public API ------------------------------------------------------------ */

int bsp_dac_init(void)
{
    /* Clock gate (RM0090 ch.6): DACEN = APB1ENR bit29. Registers below
     * are write-safe only when this is on. */
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
    (void)RCC->APB1ENR; /* read-back stalls the core until clocks are live */

    /* Port clock for PA4 (idempotent re-arm of the HAL bit). */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    (void)RCC->AHB1ENR;

    /* PA4 -> analog mode (walkthrough): MODER4 [9:8]=11, pulls off.
     * HAL equivalent: GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
     * HAL_GPIO_Init(GPIOA, ...). */
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER4_Msk) |
                   GPIO_MODER_MODER4_Msk;         /* [9:8]=11 analog */
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD4_Msk;        /* no pull */

    /* Channel 1 shape in ONE write (walkthrough bit-by-bit): EN1=1,
     * BOFF1=0 (buffer ON), TEN1=0 (software mode), WAVE1=00. HAL
     * equivalent:
     *   hdac.Init = ...; HAL_DAC_Init(&hdac);
     *   HAL_DACConfigChannel(&hdac, DAC_TRIGGER_NONE + buffer enable,
     *                        DAC_CHANNEL_1);
     *   HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, code); */
    DAC->CR = DAC_CR_EN1;

    /* tWAKEUP settle (RM 14.3.1) before the first conversion. */
    {
        volatile uint32_t spin = BSP_DAC_WAKEUP_SPIN;
        while (spin > 0u) {
            spin--;
        }
    }

    /* Read-back proof: exactly the bits above must have stuck (in
     * particular TEN1=0 -- a stray trigger enable would make set_mv
     * writes wait forever for an event that never comes). */
    if ((DAC->CR & (DAC_CR_EN1 | DAC_CR_TEN1 | DAC_CR_BOFF1 |
                    DAC_CR_WAVE1)) != DAC_CR_EN1) {
        return -1;
    }
    return 0;
}

uint32_t bsp_dac_set_mv(uint32_t mv)
{
    /* code = round(mv * 4096 / 3300) = (mv*4096 + 1650) / 3300, clamped
     * to the 12-bit full scale. Max operand mv=2^32 is impossible to
     * overflow here: mv*4096 with mv <= 4095 stays under 2^24; larger mv
     * only clamps harder (the sum fits uint32_t up to mv ~ 1M, beyond
     * which the clamp already applies -- and the RM formula saturates at
     * VREF+ regardless). */
    uint32_t code = ((mv * 4096u) + (BSP_DAC_VREF_MV / 2u)) / BSP_DAC_VREF_MV;
    if (code > 4095u) {
        code = 4095u; /* full scale: the RM formula's rail saturation */
    }
    (void)dac_write_dhr_read_dor(code);
    return code;
}

int bsp_dac_selftest(void)
{
    /* Fresh init first (idempotent, menu-driven selftest pattern). */
    if (bsp_dac_init() != 0) {
        return (int)BSP_DAC_ST1_READBACK;
    }

    /* Step 1: CR + pin-mode read-back (see the init contract). */
    if (((DAC->CR & (DAC_CR_EN1 | DAC_CR_TEN1 | DAC_CR_BOFF1 |
                     DAC_CR_WAVE1)) != DAC_CR_EN1) ||
        ((GPIOA->MODER & GPIO_MODER_MODER4_Msk) !=
         GPIO_MODER_MODER4_Msk)) {
        return (int)BSP_DAC_ST1_READBACK;
    }

    /* Step 2: three-point set->DOR round-trip THROUGH the public API (the
     * rounding math is part of the contract; the DOR read-back is the only
     * window into the conversion datapath -- DHR is a holding register,
     * DOR is what the analog stage actually sees).
     *   0    mV -> code 0    (zero scale)
     *   1650 mV -> code 2048 (mid-rail, exact)
     *   3300 mV -> code 4095 (full scale: the clamp path) */
    if ((bsp_dac_set_mv(0u) != 0u) || (dac_write_dhr_read_dor(0u) != 0u)) {
        return (int)BSP_DAC_ST2_DOR;
    }
    if ((bsp_dac_set_mv(1650u) != 2048u) ||
        (dac_write_dhr_read_dor(2048u) != 2048u)) {
        return (int)BSP_DAC_ST2_DOR;
    }
    if ((bsp_dac_set_mv(3300u) != 4095u) ||
        (dac_write_dhr_read_dor(4095u) != 4095u)) {
        return (int)BSP_DAC_ST2_DOR;
    }
    return 0; /* all steps passed */
}

/** @} */
