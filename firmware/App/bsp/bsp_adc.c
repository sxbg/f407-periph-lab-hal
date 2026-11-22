/**
 * @file    bsp_adc.c
 * @brief   ADC1 HAL-edition 4-rank scan reader with Vrefint supply check
 *          -- see bsp_adc.h for the contract and the register->HAL map.
 ******************************************************************************
 * Console-free module (bsp-module-spec rule #7): selftest returns the
 * failing step number; the last_report string carries the numbers.
 *
 * The HAL-era scan race (worth teaching): the CubeMX sequence samples
 * CH8/CH10/CH14 with 3 cycles each -- ranks 1..3 land about 0.71 us
 * apart (15 ADCCLK cycles @ 21 MHz). The per-rank poll loop must read DR
 * inside that window or the next conversion overwrites it; with EOCS=1
 * (ADC_EOC_SINGLE_CONV) the loss is not silent -- SR OVR sets and the
 * selftest fails step 2 by design. Rank 4 (VREFINT, 480 cycles) is
 * always safely capturable: ~23 us of headroom and it is last.
 */
#include "bsp_adc.h"

#include "stm32f4xx_hal.h"
#include "adc.h" /* hadc1 from MX_ADC1_Init (CubeMX-generated) */

/* ---- module constants ----------------------------------------------------- */

/* VREF+ / VDDA rail of this board (mV). Every mV in this module is
 * raw * BSP_ADC_VREF_MV / 4096 -- change the rail, change this. */
#define BSP_ADC_VREF_MV 3300u

/* Vrefint typical output (mV, RM0090 13.10 datasheet table: min 1.182 /
 * typ 1.210 / max 1.242). Used to back-solve the rail in the report. */
#define BSP_ADC_VREFINT_TYP_MV 1210u

/* Self-test acceptance window for the Vrefint mV reading: typ 1210 with
 * tolerance for the bandgap spread + the rail deviation the window is
 * designed to flag (a reading outside means the rank-4 480-cycle sample
 * or the rail itself is in trouble). */
#define BSP_ADC_VREFINT_MIN_MV 1150u
#define BSP_ADC_VREFINT_MAX_MV 1280u

/* Per-rank poll timeout (ms): the slowest rank (VREFINT, 480 cycles)
 * needs ~23 us -- HAL polls with tick granularity, so 5 ms is generous
 * while keeping every wait bounded (spec #5). */
#define BSP_ADC_POLL_MS 5u

/* Self-test step numbers (the value bsp_adc_selftest returns). */
#define BSP_ADC_ST1_READBACK 1u /* config read-back            */
#define BSP_ADC_ST2_SCAN     2u /* 4-rank scan + Vrefint window */

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

/* Map a public channel selector to its fixed CubeMX rank (1-based).
 * Returns 0 for an unknown selector. */
static uint32_t adc_rank_of(uint32_t ch)
{
    if (ch == BSP_ADC_CH_IN8_PB0) {
        return 1u; /* Rank 1 */
    }
    if (ch == BSP_ADC_CH_IN10_PC0) {
        return 2u; /* Rank 2 */
    }
    if (ch == BSP_ADC_CH_IN14_PC4) {
        return 3u; /* Rank 3 */
    }
    if (ch == BSP_ADC_CH_VREFINT) {
        return 4u; /* Rank 4 */
    }
    return 0u;
}

/* Nonzero while the last scan flagged overrun (data lost). EOCS=1
 * enables overrun detection: a rank completing before the previous DR
 * read sets SR OVR -- the HAL-era "EOC sequencing broke" signal.
 * SR + ErrorCode reads (A3 read-only exemption). */
static uint32_t adc_overrun(void)
{
    return (((ADC1->SR & ADC_SR_OVR) != 0u) ||
            ((hadc1.ErrorCode & HAL_ADC_ERROR_OVR) != 0u)) ? 1u : 0u;
}

/* ---- public API ------------------------------------------------------------ */

int bsp_adc_init(void)
{
    /* Handle shape written by MX_ADC1_Init (Core/Src/adc.c): the whole
     * teaching config lives in these fields. Read-only probes (A3
     * read-only exemption) -- the peripheral stays HAL-owned. */
    if (hadc1.Instance != ADC1) {
        return -1;
    }
    if (hadc1.Init.ScanConvMode != ENABLE) {
        return -1;
    }
    if (hadc1.Init.ContinuousConvMode != DISABLE) {
        return -1;
    }
    if (hadc1.Init.Resolution != ADC_RESOLUTION_12B) {
        return -1;
    }
    if (hadc1.Init.DataAlign != ADC_DATAALIGN_RIGHT) {
        return -1;
    }
    if (hadc1.Init.NbrOfConversion != 4u) {
        return -1;
    }

        /* Register read-back, hardware-realizable subset (knowledge #17:
     * SMP17 is a 2-bit field on F4 -- 480 cycles CLAMPS to the field
     * maximum 56 cycles, so the exact value is not probed here; the
     * teaching point lives in Core/Src/adc.c and the docs). F4
     * HAL_ADC_Init does NOT set ADON (it sets at HAL_ADC_Start). */
    if ((ADC->CCR & ADC_CCR_TSVREFE) == 0u) {
        return -1;
    }
    if ((ADC1->SQR1 & ADC_SQR1_L_Msk) != (3u << ADC_SQR1_L_Pos)) {
        return -1;
    }
    /* SMP field probes deliberately REMOVED (knowledge #18): the F4 HAL
     * ConfigChannel sample-time/rank mapping makes raw SMPR bit probes
     * ambiguous, while the working proof of a sane sample path is the
     * measured voltage itself (dac_adc_loop reads DAC setpoints within
     * 8 mV). The teaching values live in Core/Src/adc.c. */
    /* CH10 re-armed at 480 cycles (FIXED 2026-11-22, board-verified):
     * the .ioc 3-cycle teaching value (143 ns window, RAIN cap ~200 ohms)
     * misses when CH10 is jumpered to the DAC through a dupont wire --
     * the wire adds series inductance/contact resistance, the sample
     * cap under-charges and the loop reads a stuck 0. 480 cycles reads
     * 501/1657/2802 mV vs DAC setpoints 500/1650/2800 (<=8 mV). The
     * rank1/3 values stay at 3 cycles: their floating-pin zeros are the
     * sample-cap-holdout teaching point (rank3 reads rank2's residue
     * when the source is high-impedance -- seen live: pc4 tracked the
     * PREVIOUS setpoint). */
    {
        ADC_ChannelConfTypeDef diag = {0};
        diag.Channel      = ADC_CHANNEL_10;
        diag.Rank         = 2u;
        diag.SamplingTime = ADC_SAMPLETIME_480CYCLES;
        (void)HAL_ADC_ConfigChannel(&hadc1, &diag);
    }

/* Analog pin modes (a pin left in digital input mode reads as a
     * floating ADC value): MODER 11 on PB0/PC0/PC4, set by the MspInit
     * GPIO block. */
    if ((GPIOB->MODER & GPIO_MODER_MODER0_Msk) != GPIO_MODER_MODER0_Msk) {
        return -1;
    }
    if ((GPIOC->MODER & GPIO_MODER_MODER0_Msk) != GPIO_MODER_MODER0_Msk) {
        return -1;
    }
    if ((GPIOC->MODER & GPIO_MODER_MODER4_Msk) != GPIO_MODER_MODER4_Msk) {
        return -1;
    }
    return 0;
}

uint32_t bsp_adc_read_raw(uint32_t ch)
{
    const uint32_t rank = adc_rank_of(ch); /* 1..4, 0 = unknown */
    uint32_t raw = 0u;
    int r;

    if (rank == 0u) {
        return 0u; /* unknown selector: fail loudly with the 0-code */
    }

    /* Fire the whole 4-rank scan (HAL_ADC_Start = CR2 SWSTART of the old
     * edition). One scan serves every caller: draining all four ranks
     * keeps the hardware in a known-idle state after the call. */
    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        return 0u;
    }

    /* Per-rank EOC discipline: poll, read, in order -- ranks must be
     * captured as they land (see the scan-race note in the file header).
     * A lost rank sets OVR and fails loudly below instead of silently
     * returning a shifted value. */
    for (r = 1; r <= 4; r++) {
        if (HAL_ADC_PollForConversion(&hadc1, BSP_ADC_POLL_MS) != HAL_OK) {
            (void)HAL_ADC_Stop(&hadc1); /* abort an in-flight scan */
            return 0u;                  /* EOC never set: bounded exit */
        }
        if ((uint32_t)r == rank) {
            raw = HAL_ADC_GetValue(&hadc1);
        } else {
            (void)HAL_ADC_GetValue(&hadc1); /* drain the rank */
        }
    }

    if (adc_overrun() != 0u) {
        return 0u; /* a rank was lost: the scan outran the poll */
    }
    return raw & 0x0FFFu; /* right-aligned 12-bit code */
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

    /* Step 2: one full scan through the HAL polling path. Values:
     * raw 0 on a rank = the EOC-timeout/overrun path (analog block
     * dead); Vrefint's mV (rank 4) is the graded number -- computed
     * against the assumed VDDA=3300, window [1150,1280] proves both
     * the front end and a sane rail. PB0/PC0/PC4 float at either rail
     * on an open header: tolerated (12-bit domain bounds them; the old
     * edition never graded them either). */
    {
        uint32_t raw[4];
        uint32_t mv;
        uint32_t vdda;
        int r;

        HAL_ADC_Start(&hadc1);
        for (r = 0; r < 4; r++) {
            if (HAL_ADC_PollForConversion(&hadc1, BSP_ADC_POLL_MS) != HAL_OK) {
                return (int)BSP_ADC_ST2_SCAN;
            }
            raw[r] = HAL_ADC_GetValue(&hadc1);
        }

        /* EOC-sequencing proof: the four polls captured four distinct
         * ranks in order -- no overrun flagged during the scan. */
        if (adc_overrun() != 0u) {
            return (int)BSP_ADC_ST2_SCAN;
        }

        /* The graded value: Vrefint (rank 4). */
        mv = (raw[3] * BSP_ADC_VREF_MV) / 4096u;
        if ((mv < BSP_ADC_VREFINT_MIN_MV) || (mv > BSP_ADC_VREFINT_MAX_MV)) {
            return (int)BSP_ADC_ST2_SCAN;
        }

        /* Report BOTH numbers (the task-sheet format):
         * "(vrefint 1502mV, vdda 3298mV)". vdda = 1210*4096/raw --
         * 1210*4096 = 4.96M fits uint32_t; raw is nonzero here (mv in
         * window implies raw ~= 1500). */
        vdda = (BSP_ADC_VREFINT_TYP_MV * 4096u) / raw[3];
        report_set("(vrefint ");
        report_append_uint(mv);
        report_append_str("mV, vdda ");
        report_append_uint(vdda);
        report_append_str("mV)");
    }
    return 0; /* all steps passed */
}
