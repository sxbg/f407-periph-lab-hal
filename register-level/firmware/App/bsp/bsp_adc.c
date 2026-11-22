/**
 ******************************************************************************
 * @file    bsp_adc.c
 * @brief   ADC1 register-level single-channel reader (SWSTART/EOC polling)
 *          with Vrefint supply check -- teaching module (EP08/EP09).
 ******************************************************************************
 * @addtogroup BSP_ADC
 * @{
 *
 * == CUBEMX CONFIG (absent from f407-periph-lab.ioc -- by design) ==========
 * | Field             | Value (module truth)                 | Why
 * |-------------------|--------------------------------------|-------------------
 * | ADC1 in IOC       | NOT PRESENT as an IP                 | knowledge #13:
 * |                   |                                      | adapter-owned
 * |                   |                                      | peripherals stay
 * |                   |                                      | out of the .ioc
 * | Pins (IOC truth)  | PB0=ADC1_IN8, PC0=ADC1_IN10,         | locked as
 * |                   | PC4=ADC1_IN14                        | GPIO_Analog
 * |                   |                                      | placeholders --
 * |                   |                                      | this module owns
 * |                   |                                      | their MODER
 * | Vrefint           | ADC1_IN17 (internal, master ADC1     | RM0090 13.3.3:
 * |                   | only)                                | VREFINT on IN17;
 * |                   |                                      | needs CCR TSVREFE
 * | ADCCLK            | PCLK2/2 = 42 MHz (CCR ADCPRE reset   | DATASHEET MAX IS
 * |                   | default 00, kept)                    | 36 MHz -- kept
 * |                   |                                      | default + the one-
 * |                   |                                      | line fix documented
 * |                   |                                      | below (EP08 clock
 * |                   |                                      | lesson)
 * | Sampling time     | 480 cycles (SMPx=111) on all four    | Vrefint source is
 * |                   | channels                             | high-impedance:
 * |                   |                                      | datasheet minimum
 * |                   |                                      | ~10 us (RM 13.10)
 * | Sequence          | 1 conversion (SQR1 L=0)              | single-channel
 * |                   |                                      | polling lesson
 * | NVIC              | ADC_IRQn off, EOCIE off              | EOC polled (EP08)
 *
 * == REGISTER WALKTHROUGH (RM0090 ch.13 "Analog-to-digital converter") =====
 * Clocks (13.3.2): the ADC has TWO clocks. The DIGITAL interface clock =
 *   PCLK2, gated by RCC->APB2ENR bit8 (ADC1EN) -- registers are write-safe
 *   only when this is on. The ANALOG circuitry clock = ADCCLK, tapped off
 *   PCLK2 through the ADCPRE prescaler in the COMMON register block
 *   (ADC->CCR, shared by ADC1/2/3, bits [17:16]): 00=/2, 01=/4, 10=/6,
 *   11=/8. THE HONEST CAVEAT: reset default 00 keeps PCLK2/2 = 84/2 =
 *   42 MHz on this board, while the F407 datasheet caps ADCCLK at 36 MHz.
 *   Silicon tolerates it (14-bit-ish accuracy at the cost of a spec
 *   violation); the IN-SPEC fix is one line: ADC->CCR |= ADC_CCR_ADCPRE_0
 *   (=/4 -> 21 MHz). Left at default because the lesson is "know your
 *   ADCCLK" -- EP08's whole point.
 * Vrefint (13.3.3/13.10): the internal 1.21 V-typical bandgap hangs on
 *   ADC1_IN17, powered down by default; ADC->CCR bit23 (TSVREFE) wakes it
 *   (with the temperature sensor, IN16). Master ADC1 ONLY -- never on
 *   ADC2/ADC3. Its real use: measure the RAIL with a KNOWN reference.
 *   raw_vref = VREFINT/VDDA * 4096, so VDDA(mV) = 1210 * 4096 / raw_vref
 *   -- the selftest report's "vdda" number. If Vrefint reads ~1210 mV
 *   against an assumed 3.3 V rail, the rail really is ~3.3 V (1210 mV is
 *   itself approximate: +/- tolerance in the selftest window).
 * Pins (RM0090 ch.8.4.1): MODER 2 bits/pin, 11 = ANALOG mode (Schmitt
 *   trigger and input buffer disconnected -- the reading is the raw pad
 *   voltage). PB0 [1:0]=11, PC0 [1:0]=11, PC4 [9:8]=11. Pulls must stay
 *   OFF on an ADC input (PUPDR 00): a pull would form a divider with the
 *   source and shift every reading.
 * Sampling time (13.5, SMPR1/SMPR2): 3 bits per channel select 3..480
 *   ADCCLK cycles of sample-and-hold charge time. 111 = 480 cycles, the
 *   longest. Channel placement: IN10/IN14/IN17 live in SMPR1 (SMP10
 *   [2:0], SMP14 [14:12], SMP17 [23:21]), IN8 in SMPR2 ([26:24]). 480
 *   cycles is REQUIRED for the internal channels (datasheet minimum
 *   sampling ~10 us; 480/42M = 11.4 us) and generous for the header
 *   pins. Longer sample = lower effective source-impedance error.
 * Sequence (13.3.3, SQR1/SQR3): the regular group holds up to 16
 *   conversions; L[3:0] in SQR1 [23:20] stores "count - 1". L=0 means
 *   exactly ONE conversion, whose channel is SQ1[4:0] in SQR3 [4:0].
 *   Rewriting SQ1 between conversions is how this module "switches
 *   channels" without scan mode -- the teaching-simple way.
 * CR2 (13.13.3): bit0 ADON wakes the converter from power-down (F4
 *   semantics -- UNLIKE the F1, writing ADON=1 again does NOT start a
 *   conversion; only SWSTART/JSWSTART do). Bit1 CONT=0 single mode. Bit10
 *   EOCS=0: EOC set at end of the whole sequence (= each conversion
 *   here, L=1). Bits [29:28] EXTEN=00: no external trigger. Bit30
 *   SWSTART=1 starts the regular sequence (note in the RM: it is only
 *   obeyed when ADON=1) and self-clears once the conversion starts.
 * SR (13.13.1): rc_w0 flags -- cleared BY WRITING 0 (same convention as
 *   TIM SR). Bit1 EOC sets when the data lands in DR; it ALSO clears on
 *   a DR read, so the sequence "poll EOC, read DR" is self-cleaning.
 *   Bit5 OVR: overrun detection needs DMA=1 or EOCS=1 -- with both off,
 *   this polling pattern cannot overrun (a second conversion is only
 *   started after DR was read).
 * DR (13.13.14): right-aligned 12-bit data in [11:0] (CR2 ALIGN=0, the
 *   reset default -- no bit to write, just a fact to verify).
 * Timing (13.3.6): total conversion = sampling + 12 ADCCLK cycles
 *   (12-bit). Here 480 + 12 = 492 / 42 MHz = 11.7 us per reading. The
 *   first conversion after ADON additionally needs tSTAB -- hence the
 *   settle spin in init.
 * == HOW TO VERIFY =========================================================
 * menu> adc
 *   PASS adc (vrefint 1502mV, vdda 3298mV)
 *   -- the suffix is the live Vrefint reading and the back-solved rail;
 *   vrefint ~1150..1280 mV proves the analog front end + a sane VDDA.
 *   FAIL adc step1 = a register read-back broke (clock gate/ANALOG/SMPR);
 *   FAIL adc step2 = Vrefint outside the window (check TSVREFE, the 480-
 *   cycle sample, or the rail itself).
 * Scope/DMM cross-check: 'dac_adc_loop' (menu) drives PA4 -> PB0 through
 *   a jumper and compares DAC setpoint vs ADC reading at three points --
 *   the closed-loop proof for both analog peripherals.
 */
#include "bsp_adc.h"

#include "hal_port.h" /* family seam: pulls in the CMSIS device header */

/* ---- module constants ----------------------------------------------------- */

/* VREF+ / VDDA rail of this board (mV). Every mV in this module is
 * raw * BSP_ADC_VREF_MV / 4096 -- change the rail, change this. */
#define BSP_ADC_VREF_MV 3300u

/* Vrefint typical output (mV, RM0090 13.10 datasheet table: min 1.182 /
 * typ 1.210 / max 1.242). Used to back-solve the rail in the report. */
#define BSP_ADC_VREFINT_TYP_MV 1210u

/* Self-test acceptance window for the Vrefint mV reading: typ 1210 with
 * tolerance for the bandgap spread + the rail deviation the window is
 * designed to flag (a reading outside means TSVREFE/sample-time/ADCCLK
 * trouble -- or a genuinely drooping rail). */
#define BSP_ADC_VREFINT_MIN_MV 1150u
#define BSP_ADC_VREFINT_MAX_MV 1280u

/* 480-cycle sampling = SMPx field value 111 (RM0090 13.5 table). */
#define BSP_ADC_SMP_480 0x7u

/* Conversion wait: iteration cap only (no SysTick needed). One conversion
 * is 11.7 us; the cap is orders of magnitude longer, so a timeout means
 * the ADC never set EOC (real breakage), not latency (spec #5). */
#define BSP_ADC_EOC_GUARD 2000000u

/* Post-ADON settle spin (~1000 iterations ~= 6 us at 168 MHz) covering
 * the tSTAB stabilization window of 13.3.6 before the first SWSTART. */
#define BSP_ADC_STAB_SPIN 1000u

/* Self-test step numbers (the value bsp_adc_selftest returns). */
#define BSP_ADC_ST1_READBACK 1u /* gates/config read-back */
#define BSP_ADC_ST2_VREFINT  2u /* Vrefint window check   */

/* ---- module state ---------------------------------------------------------- */

/* Report of the last selftest run (menu appends it to "PASS adc"). */
static char s_report[48];

/* ---- private helpers -------------------------------------------------------- */

/* Copy a NUL-terminated string into s_report without libc. */
static void report_set(const char *msg)
{
    unsigned i = 0u;
    while ((msg[i] != '\0') && (i < (sizeof(s_report) - 1u))) {
        s_report[i] = msg[i];
        i++;
    }
    s_report[i] = '\0';
}

/* Append one decimal number to s_report (hand-rolled, no libc printf).
 * Bounded by the buffer size; the report shape is fixed, so this only
 * ever appends 3-4 digits. */
static void report_append_uint(uint32_t v)
{
    char digits[10];
    unsigned n = 0u;
    do {
        digits[n++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v > 0u);
    while (n > 0u) {
        unsigned i = 0u;
        while (s_report[i] != '\0') {
            i++;
        }
        if (i < (sizeof(s_report) - 1u)) {
            s_report[i] = digits[--n];
            s_report[i + 1u] = '\0';
        } else {
            break; /* buffer full: stop appending (never overflows) */
        }
    }
}

/* Append one literal string to s_report (bounded, no libc). */
static void report_append_str(const char *msg)
{
    unsigned i = 0u;
    while (s_report[i] != '\0') {
        i++;
    }
    while ((*msg != '\0') && (i < (sizeof(s_report) - 1u))) {
        s_report[i] = *msg++;
        i++;
    }
    s_report[i] = '\0';
}

/* The four channels this module serves share one SMP shape: 480 cycles.
 * Writing the SMPx field for one of them. (SMPR1 holds IN10..IN18:
 * SMP10 [2:0], SMP14 [14:12], SMP17 [23:21]; SMPR2 holds IN0..IN9:
 * SMP8 [26:24].) */
static void adc_set_sample_480(uint32_t ch)
{
    if (ch == BSP_ADC_CH_IN10_PC0) {
        ADC1->SMPR1 = (ADC1->SMPR1 & ~ADC_SMPR1_SMP10_Msk) |
                     (BSP_ADC_SMP_480 << ADC_SMPR1_SMP10_Pos);
    } else if (ch == BSP_ADC_CH_IN14_PC4) {
        ADC1->SMPR1 = (ADC1->SMPR1 & ~ADC_SMPR1_SMP14_Msk) |
                     (BSP_ADC_SMP_480 << ADC_SMPR1_SMP14_Pos);
    } else if (ch == BSP_ADC_CH_VREFINT) {
        ADC1->SMPR1 = (ADC1->SMPR1 & ~ADC_SMPR1_SMP17_Msk) |
                     (BSP_ADC_SMP_480 << ADC_SMPR1_SMP17_Pos);
    } else if (ch == BSP_ADC_CH_IN8_PB0) {
        ADC1->SMPR2 = (ADC1->SMPR2 & ~ADC_SMPR2_SMP8_Msk) |
                     (BSP_ADC_SMP_480 << ADC_SMPR2_SMP8_Pos);
    } else {
        /* unknown channel: no field to touch (caller validates anyway) */
    }
}

/* True when the given channel's SMPx field reads back 480 cycles. */
static uint32_t adc_sample_field(uint32_t ch)
{
    if (ch == BSP_ADC_CH_IN10_PC0) {
        return (ADC1->SMPR1 & ADC_SMPR1_SMP10_Msk) >> ADC_SMPR1_SMP10_Pos;
    }
    if (ch == BSP_ADC_CH_IN14_PC4) {
        return (ADC1->SMPR1 & ADC_SMPR1_SMP14_Msk) >> ADC_SMPR1_SMP14_Pos;
    }
    if (ch == BSP_ADC_CH_VREFINT) {
        return (ADC1->SMPR1 & ADC_SMPR1_SMP17_Msk) >> ADC_SMPR1_SMP17_Pos;
    }
    if (ch == BSP_ADC_CH_IN8_PB0) {
        return (ADC1->SMPR2 & ADC_SMPR2_SMP8_Msk) >> ADC_SMPR2_SMP8_Pos;
    }
    return 0xFFFFFFFFu; /* unknown -> never matches */
}

/* Put one analog pin into MODER analog mode (11) with pulls off. */
static void adc_pin_analog(volatile uint32_t *moder, volatile uint32_t *pupdr,
                           uint32_t moder_msk, uint32_t pupdr_msk)
{
    *moder = (*moder & ~moder_msk) | moder_msk; /* 11: both bits set */
    *pupdr &= ~pupdr_msk;                       /* 00: no pull divider */
}

/* ---- public API ------------------------------------------------------------ */

int bsp_adc_init(void)
{
    /* Digital interface clock (RM0090 13.3.2): ADC1EN = APB2ENR bit8.
     * Registers below are write-safe only when this is on. */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC->APB2ENR; /* read-back stalls the core until clocks are live */

    /* Port clocks for the analog pins (RM0090 ch.6). HAL's MX_GPIO_Init
     * already set these; re-arming keeps init self-sufficient. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;

    /* Pins -> analog mode (walkthrough): Schmitt/input buffer off, pulls
     * off. HAL equivalent: GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
     * HAL_GPIO_Init(...). */
    adc_pin_analog(&GPIOB->MODER, &GPIOB->PUPDR,
                   GPIO_MODER_MODER0_Msk, GPIO_PUPDR_PUPD0_Msk); /* PB0  */
    adc_pin_analog(&GPIOC->MODER, &GPIOC->PUPDR,
                   GPIO_MODER_MODER0_Msk, GPIO_PUPDR_PUPD0_Msk); /* PC0  */
    adc_pin_analog(&GPIOC->MODER, &GPIOC->PUPDR,
                   GPIO_MODER_MODER4_Msk, GPIO_PUPDR_PUPD4_Msk); /* PC4  */

    /* Common block (ADC1/2/3 shared, RM0090 13.13.16): TSVREFE=1 wakes
     * Vrefint on IN17 (master ADC1 only). ADCPRE stays at its RESET
     * DEFAULT 00 = PCLK2/2 = 42 MHz -- the datasheet cap is 36 MHz; the
     * honest in-spec fix is documented in the walkthrough (ADCPRE_0).
     * Nothing else in CCR is touched (multi-ADC modes unused). */
    ADC->CCR |= ADC_CCR_TSVREFE;

    /* Sampling time on every served channel: 480 cycles (walkthrough). */
    adc_set_sample_480(BSP_ADC_CH_IN8_PB0);
    adc_set_sample_480(BSP_ADC_CH_IN10_PC0);
    adc_set_sample_480(BSP_ADC_CH_IN14_PC4);
    adc_set_sample_480(BSP_ADC_CH_VREFINT);

    /* Regular sequence: exactly ONE conversion (SQR1 L[3:0]=0; the channel
     * itself is re-pointed per read in bsp_adc_read_raw via SQR3 SQ1). */
    ADC1->SQR1 &= ~ADC_SQR1_L_Msk; /* L=0 (reset default, written to own it) */

    /* CR1: SCAN=0 (no scan mode needed with a 1-conversion sequence),
     * RES[1:0]=00 (12-bit, 15 ADCCLK cycles), everything else reset.
     * One write = full owner. */
    ADC1->CR1 = 0u;

    /* CR2 in one write: ADON=1 (power up), CONT=0 (single mode), EOCS=0
     * (EOC per sequence), EXTEN=00 (no external trigger; SWSTART rules),
     * ALIGN=0 (right-aligned data). HAL equivalent:
     *   hadc.Init.ContinuousConvMode = DISABLE;
     *   HAL_ADC_Init(&hadc); HAL_ADC_Start + polling. */
    ADC1->CR2 = ADC_CR2_ADON;

    /* tSTAB (13.3.6): a short spin before the first SWSTART so the very
     * first reading is not taken while the converter is still stabilizing.
     * First real conversion also happens only after init returns (menu
     * latency), so this is belt-and-braces. */
    {
        volatile uint32_t spin = BSP_ADC_STAB_SPIN;
        while (spin > 0u) {
            spin--;
        }
    }

    /* Read-back proof: gates, wake bit, sample shape must have stuck. */
    if (((RCC->APB2ENR & RCC_APB2ENR_ADC1EN) == 0u) ||
        ((ADC1->CR2 & ADC_CR2_ADON) == 0u) ||
        ((ADC->CCR & ADC_CCR_TSVREFE) == 0u) ||
        ((ADC1->SQR1 & ADC_SQR1_L_Msk) != 0u)) {
        return -1;
    }
    return 0;
}

uint32_t bsp_adc_read_raw(uint32_t ch)
{
    uint32_t guard;

    if ((ch != BSP_ADC_CH_IN8_PB0) && (ch != BSP_ADC_CH_IN10_PC0) &&
        (ch != BSP_ADC_CH_IN14_PC4) && (ch != BSP_ADC_CH_VREFINT)) {
        return 0u; /* unknown selector: fail loudly with the 0-code */
    }

    /* Point the 1-conversion sequence at the channel (SQR3 SQ1[4:0],
     * RM0090 13.3.3). Safe between conversions; a write during an active
     * conversion would RESET it (RM note) -- this polling module converts
     * one at a time, so that case cannot happen. */
    ADC1->SQR3 = ch & 0x1Fu;

    /* Clean stale EOC before arming (rc_w0: write 0 clears, 13.13.1). */
    ADC1->SR = ~ADC_SR_EOC;

    /* Software start (13.3.4): SWSTART=1 with ADON=1 (RM note) and
     * EXTEN=00. The bit self-clears once the conversion starts. HAL
     * equivalent: HAL_ADC_Start(&hadc). */
    ADC1->CR2 |= ADC_CR2_SWSTART;

    /* Poll EOC, bounded (spec #5). 492 ADCCLK cycles = 11.7 us nominal. */
    guard = BSP_ADC_EOC_GUARD;
    while ((ADC1->SR & ADC_SR_EOC) == 0u) {
        if (--guard == 0u) {
            return 0u; /* EOC never set: bounded exit */
        }
    }

    /* DR read delivers the right-aligned 12-bit code AND clears EOC
     * (13.13.1/13.13.14). HAL equivalent: HAL_ADC_GetValue(&hadc). */
    return ADC1->DR & 0x0FFFu;
}

uint32_t bsp_adc_read_mv(uint32_t ch)
{
    /* raw * 3300 / 4096: max operand 4095*3300 = 13.5M, fits uint32_t.
     * Quantization step 3300/4096 = 0.806 mV -- the LSB of any reading
     * (and part of every loop-test delta). */
    return (bsp_adc_read_raw(ch) * BSP_ADC_VREF_MV) / 4096u;
}

const char *bsp_adc_last_report(void)
{
    return s_report;
}

int bsp_adc_selftest(void)
{
    /* Fresh init first (idempotent, menu-driven selftest pattern). */
    if (bsp_adc_init() != 0) {
        return (int)BSP_ADC_ST1_READBACK;
    }

    /* Step 1: register read-back -- clocks, wake, sample shape, sequence
     * length, and the ANALOG pin mode of all three header pins (a pin
     * left in digital input mode reads as a floating ADC value). */
    if (((RCC->APB2ENR & RCC_APB2ENR_ADC1EN) == 0u) ||
        ((ADC1->CR2 & ADC_CR2_ADON) == 0u) ||
        ((ADC->CCR & ADC_CCR_TSVREFE) == 0u) ||
        ((ADC1->SQR1 & ADC_SQR1_L_Msk) != 0u) ||
        (adc_sample_field(BSP_ADC_CH_IN8_PB0) != BSP_ADC_SMP_480) ||
        (adc_sample_field(BSP_ADC_CH_IN10_PC0) != BSP_ADC_SMP_480) ||
        (adc_sample_field(BSP_ADC_CH_IN14_PC4) != BSP_ADC_SMP_480) ||
        (adc_sample_field(BSP_ADC_CH_VREFINT) != BSP_ADC_SMP_480) ||
        ((GPIOB->MODER & GPIO_MODER_MODER0_Msk) != GPIO_MODER_MODER0_Msk) ||
        ((GPIOC->MODER & GPIO_MODER_MODER0_Msk) != GPIO_MODER_MODER0_Msk) ||
        ((GPIOC->MODER & GPIO_MODER_MODER4_Msk) != GPIO_MODER_MODER4_Msk)) {
        return (int)BSP_ADC_ST1_READBACK;
    }

    /* Step 2: Vrefint -- the known-reference check. raw 0 = the EOC-timeout
     * path (analog block dead). The mV reading assumes VDDA=3300; the
     * window [1150,1280] then also proves the rail is sane (see the
     * back-solve in the report below: vdda = 1210*4096/raw). */
    {
        const uint32_t raw = bsp_adc_read_raw(BSP_ADC_CH_VREFINT);
        if (raw == 0u) {
            return (int)BSP_ADC_ST2_VREFINT;
        }
        {
            const uint32_t mv = (raw * BSP_ADC_VREF_MV) / 4096u;
            if ((mv < BSP_ADC_VREFINT_MIN_MV) ||
                (mv > BSP_ADC_VREFINT_MAX_MV)) {
                return (int)BSP_ADC_ST2_VREFINT;
            }
            /* Report BOTH numbers (the task-sheet format):
             * "(vrefint 1502mV, vdda 3298mV)". vdda = 1210*4096/raw --
             * 1210*4096 = 4.96M fits uint32_t; raw is nonzero here. */
            {
                const uint32_t vdda =
                    (BSP_ADC_VREFINT_TYP_MV * 4096u) / raw;
                report_set("(vrefint ");
                report_append_uint(mv);
                report_append_str("mV, vdda ");
                report_append_uint(vdda);
                report_append_str("mV)");
            }
        }
    }
    return 0; /* all steps passed */
}

/** @} */
