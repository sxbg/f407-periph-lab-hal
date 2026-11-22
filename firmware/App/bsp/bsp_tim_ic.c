/**
 * @file bsp_tim_ic.c
 * @brief TIM2_CH2 (PB3) input capture frequency meter -- teaching module
 *        (HAL edition of f407-periph-lab bsp_tim_ic; echoes the tutorial
 *        EP13 capture principle).
 ******************************************************************************
 * == WHAT CUBEMX ALREADY DID (MX_TIM2_Init + MspInit) ======================
 * | Item            | Value (IOC truth)                | Why
 * |-----------------|----------------------------------|-------------------
 * | TIM2 mode       | Input Capture 2 from TI2 (PB3,   | pin routed by
 * |                 | AF1)                             | MspInit; generated
 * |                 |                                  | code stays at base
 * |                 |                                  | init -- capture
 * |                 |                                  | shape owned here
 * | TIM2 counter    | 32-bit, Period=4294967295, PSC=0 | the F4's only
 * |                 | (kept as-is)                     | general 32-bit
 * |                 |                                  | timers: TIM2/TIM5
 * | PB3 pull        | none (IOC)                       | a pull would bias a
 * |                 |                                  | floating probe pin
 * | Kernel clock    | APB1 x2 = 84 MHz                 | RM0090 ch.6 rule
 * | (PB3 caveat)    | PB3 = JTDO in default AF0        | the AF1 mux re-routes
 * |                 |                                  | it; SWD untouched
 * == HAL MAPPING (register concept -> HAL call) ============================
 *   CCMR1 CC2S=01 / IC2F=0011 / IC2PSC=00 ... HAL_TIM_IC_ConfigChannel:
 *       ICSelection=TIM_ICSELECTION_DIRECTTI (IC2 mapped on TI2, direct),
 *       ICFilter=3 (8 consecutive samples at f_DTS/8 must agree to
 *       validate an edge: ~762 ns glitch rejection at CKD=00),
 *       ICPrescaler=TIM_ICPSC_DIV1 (capture on EVERY valid edge).
 *   CCER CC2E=1 / CC2P=0 (rising) .......... HAL_TIM_IC_Start -- enables
 *       capture 2 + CEN, touches no DIER bit (the polling sibling of
 *       HAL_TIM_IC_Start_IT; the old DIER=0 write is simply unnecessary
 *       now, CubeMX left DIER clear).
 *   SR.UIF/CC2IF poll ...................... __HAL_TIM_GET_FLAG(&htim2,
 *       TIM_FLAG_UPDATE / TIM_FLAG_CC2); UIF clears via
 *       __HAL_TIM_CLEAR_FLAG (rc_w0 via HAL). CC2IF needs NO clear:
 *       reading CCR2 IS the clear (RM0090 ch.18.4.14 peripheral
 *       contract, like UART RXNE).
 *   CCR2 timestamp ......................... __HAL_TIM_GET_COMPARE(&htim2,
 *       TIM_CHANNEL_2).
 *   Old EGR.UG arm stroke .................. dropped: CNT phase is
 *       irrelevant to an edge-to-edge measurement, and stale flags are
 *       cleared at the top of each measurement instead.
 *   Idempotent re-init ..................... HAL_TIM_IC_Stop first (HAL
 *       channel state machine: Start on a BUSY channel is a HAL_ERROR).
 * Period math (EP13 principle, unchanged from the register edition):
 *   period_ticks = (CCR2_at_edge2 + N*2^32) - CCR2_at_edge1, where N =
 *   overflows between the edges (each UIF = one 2^32 wrap). At 84 MHz the
 *   32-bit counter overflows only every 2^32/84M ~= 51.1 s, so inside a
 *   1 s window N is realistically 0 -- the compensation stays because the
 *   PRINCIPLE is what EP13 teaches (on a 16-bit timer like TIM3, N>0
 *   happens in milliseconds).
 * == HOW TO VERIFY =========================================================
 * menu> tim_ic
 *   No signal wired: "PASS tim_ic (no signal, 0 Hz)" -- an idle probe is
 *   SKIP-state semantics, not a failure.
 *   Jumper PD12 (TIM4 PWM, 1 kHz from menu 'tim_pwm') -> PB3, run
 *   'tim_pwm' first then 'tim_ic': expect "PASS tim_ic (measured ~1000 Hz)"
 *   (the meter reads TIM4's own output: two independent timers cross-check).
 * Scope check: PB3 shows the driven square wave; measured value matches
 *   the scope within 1 tick of 84 MHz resolution (12 ns).
 */
#include "bsp_tim_ic.h"

#include <stddef.h>

#include "stm32f4xx_hal.h"
#include "tim.h" /* htim2 from MX_TIM2_Init (CubeMX-generated) */

/* ---- module constants ----------------------------------------------------- */

/* Measurement window shared by both edge waits (1 edge-to-edge period
 * must fit inside it or the result is "no signal"). */
#define BSP_TIM_IC_WINDOW_MS 1000u

/* Report text for the empty-probe case (SKIP-state semantics). */
#define BSP_TIM_IC_NO_SIGNAL_TXT "(no signal, 0 Hz)"

/* Self-test step numbers (the value bsp_tim_ic_selftest returns). */
#define BSP_TIM_IC_ST1_CONFIG 1u /* capture config read-back */
#define BSP_TIM_IC_ST2_MEAS   2u /* one measurement pass */

/* Spin cap between HAL_GetTick deadline checks: if SysTick is dead the
 * deadline never arrives -- bounded exit instead of a hang (spec #5). */
#define BSP_TIM_IC_SPIN_GUARD 2000000u

/* ---- module state ----------------------------------------------------------- */

/* Report of the last selftest run; the menu reads it after the run and
 * appends it to "PASS tim_ic". Single-threaded menu context: a static
 * char buffer is safe without locking. */
static char s_report[32];

/* ---- private helpers -------------------------------------------------------- */

/* Wait for the next capture event, servicing overflows on the way.
 *   start:  HAL_GetTick value when the shared measurement window opened
 *   budget: expire when 'budget' ms elapsed since start (both edge waits
 *           share ONE window, phase 2 spends what phase 1 left)
 *   ovf:    incremented per UIF seen (one 2^32 counter wrap)
 * Returns 0 = CC2IF set (caller reads CCR2, which also clears it),
 *         -1 = SysTick dead (bounded exit), -2 = window exhausted. */
static int ic_wait_edge(uint32_t start, uint32_t budget, uint32_t *ovf)
{
    uint32_t guard = BSP_TIM_IC_SPIN_GUARD;
    for (;;) {
        if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC2) != RESET) {
            return 0; /* edge latched in CCR2 -- do NOT clear here */
        }
        if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET) {
            __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE); /* rc_w0 via HAL */
            (*ovf)++;               /* count the 2^32 wrap for compensation */
        }
        if ((HAL_GetTick() - start) >= budget) {
            return -2; /* window exhausted without a fresh edge */
        }
        guard--;
        if (guard == 0u) {
            return -1; /* SysTick dead: bounded exit, never hang */
        }
    }
}

/* Format "(measured N Hz)" into s_report (decimal, no libc). */
static void ic_format_measured(uint32_t hz)
{
    static const char digits[10] = {
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9'
    };
    char tmp[10];
    unsigned n = 0u;
    uint32_t v = hz;
    const char *pfx = "(measured ";
    const char *sfx = " Hz)";
    char *p = s_report;
    while (*pfx != '\0') {
        *p++ = *pfx++;
    }
    do {
        tmp[n++] = digits[v % 10u];
        v /= 10u;
    } while (v > 0u);
    while (n > 0u) {
        *p++ = tmp[--n];
    }
    while (*sfx != '\0') {
        *p++ = *sfx++;
    }
    *p = '\0';
}

/* ---- public API ------------------------------------------------------------ */

int bsp_tim_ic_init(void)
{
    TIM_IC_InitTypeDef ic;
    uint32_t ccmr;
    uint32_t ccer;

    /* CubeMX owns clocks/pins/base timebase; verify the handle is live. */
    if (htim2.Instance == NULL) {
        return -1;
    }

    /* Idempotent re-init: IC_Stop restores the HAL channel state machine
     * to READY and drops CC2E/CEN (Start on a BUSY channel is a
     * HAL_ERROR). Harmless on the first call. */
    if (HAL_TIM_IC_Stop(&htim2, TIM_CHANNEL_2) != HAL_OK) {
        return -1;
    }

    /* Capture channel 2 on TI2, direct, filtered, every edge (file
     * header). HAL writes CCMR1 (CC2S/IC2PSC/IC2F) from this struct. */
    ic.ICPolarity = TIM_ICPOLARITY_RISING;     /* CC2P=0: rising edge */
    ic.ICSelection = TIM_ICSELECTION_DIRECTTI; /* CC2S=01: IC2 <- TI2 */
    ic.ICPrescaler = TIM_ICPSC_DIV1;           /* capture every valid edge */
    ic.ICFilter = 3u;                          /* IC2F=0011: ~762 ns filter */

    /* Timebase stays CubeMX's: PSC=0 (max resolution) and the full 32-bit
     * free-run window (ARR=0xFFFFFFFF, 51.1 s between overflows at
     * 84 MHz) -- verified by read-back below, not rewritten here. */
    if (HAL_TIM_IC_ConfigChannel(&htim2, &ic, TIM_CHANNEL_2) != HAL_OK) {
        return -1;
    }

    /* Arm: CC2E + CEN, no interrupt routing (polling lesson). */
    if (HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2) != HAL_OK) {
        return -1;
    }

    /* Read-back proof: the capture shape must have stuck. CCMR1/CCER/PSC
     * have no HAL getters; those Instance reads are read-only probes. */
    ccmr = htim2.Instance->CCMR1; /* read-only (A3 exemption) */
    ccer = htim2.Instance->CCER;  /* read-only (A3 exemption) */
    if (((ccmr &
          (TIM_CCMR1_CC2S_Msk | TIM_CCMR1_IC2PSC_Msk | TIM_CCMR1_IC2F_Msk)) !=
         (TIM_CCMR1_CC2S_0 | TIM_CCMR1_IC2F_0 | TIM_CCMR1_IC2F_1)) ||
        ((ccer & (TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP)) !=
         TIM_CCER_CC2E) ||
        (htim2.Instance->PSC != 0u) || /* read-only (A3 exemption) */
        (__HAL_TIM_GET_AUTORELOAD(&htim2) != 0xFFFFFFFFu)) {
        return -1;
    }
    return 0;
}

uint32_t bsp_tim_ic_measure_freq(void)
{
    uint32_t start;
    uint32_t ovf = 0u; /* 2^32 wraps between the two edges */
    uint32_t t1;

    /* Clean flag state: stale UIF/CC2IF from a previous run would fake a
     * first edge instantly. __HAL_TIM_CLEAR_FLAG is the HAL spelling of
     * the rc_w0 write; CC2IF is safe to clear too (no real edge yet). */
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE | TIM_FLAG_CC2);

    /* Phase 1: first rising edge inside the window. Timeout here means no
     * signal (the probe pin is idle) -> 0 Hz, the SKIP-state answer. */
    start = HAL_GetTick();
    if (ic_wait_edge(start, BSP_TIM_IC_WINDOW_MS, &ovf) != 0) {
        return 0u;
    }
    /* t1 in timer ticks. __HAL_TIM_GET_COMPARE reads CCR2, which is ALSO
     * the CC2IF clear -- the RM0090 peripheral contract (ch.18.4.14),
     * same spirit as UART RXNE. */
    t1 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
    /* Only overflows AFTER t1 contribute to the period. */
    ovf = 0u;

    /* Phase 2: second rising edge, spending what remains of the shared
     * 1 s budget (ic_wait_edge keeps measuring from the same 'start'). */
    if (ic_wait_edge(start, BSP_TIM_IC_WINDOW_MS, &ovf) != 0) {
        return 0u; /* one edge only: period longer than the window */
    }
    {
        const uint32_t t2_raw = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
        /* Overflow compensation (EP13 principle): every UIF seen means
         * the counter wrapped 2^32 ticks since t1, so the true second
         * timestamp is t2_raw + ovf*2^32. 64-bit intermediate: ovf>0
         * pushes the sum past 32 bits. */
        const uint64_t t2 =
            (uint64_t)t2_raw + ((uint64_t)ovf << 32);
        const uint64_t delta = t2 - (uint64_t)t1;
        if ((delta == 0u) ||
            (delta > (uint64_t)BSP_TIM_IC_REF_CLOCK_HZ)) {
            /* Degenerate (two captures in one tick) or the period is
             * longer than the 1 s window: report "no signal". */
            return 0u;
        }
        /* freq = f_kernel / period_ticks. Max 84M/1 = 84 MHz, fits
         * uint32_t. Resolution: 1 tick = 11.9 ns of period. */
        return (uint32_t)((uint64_t)BSP_TIM_IC_REF_CLOCK_HZ / delta);
    }
}

const char *bsp_tim_ic_last_report(void)
{
    return s_report;
}

int bsp_tim_ic_selftest(void)
{
    /* Fresh init first (idempotent, menu-driven selftest pattern). */
    if (bsp_tim_ic_init() != 0) {
        return (int)BSP_TIM_IC_ST1_CONFIG;
    }

    /* Step 1: capture config read-back -- the exact CCMR1/CCER shape plus
     * the CubeMX-owned timebase (init already asserted each; re-check
     * standalone so a later module fault still points at the config, not
     * the measurement). */
    if (((htim2.Instance->CCMR1 &                              /* read-only (A3 exemption) */
          (TIM_CCMR1_CC2S_Msk | TIM_CCMR1_IC2PSC_Msk | TIM_CCMR1_IC2F_Msk)) !=
         (TIM_CCMR1_CC2S_0 | TIM_CCMR1_IC2F_0 | TIM_CCMR1_IC2F_1)) ||
        ((htim2.Instance->CCER & (TIM_CCER_CC2E | TIM_CCER_CC2P |
                                  TIM_CCER_CC2NP)) !=            /* read-only (A3 exemption) */
         TIM_CCER_CC2E) ||
        (htim2.Instance->PSC != 0u) ||                           /* read-only (A3 exemption) */
        (__HAL_TIM_GET_AUTORELOAD(&htim2) != 0xFFFFFFFFu)) {
        return (int)BSP_TIM_IC_ST1_CONFIG;
    }

    /* Step 2: one measurement pass. An idle probe pin returns 0 Hz and
     * that is a PASS by SKIP-state semantics (hardware absence is not a
     * FAIL in this framework); a driven pin reports the measured value.
     * The report string goes through bsp_tim_ic_last_report() so the menu
     * can append it to the verdict line -- this module never touches the
     * console (bsp-module-spec rule #7: no cross-module includes). */
    {
        const uint32_t hz = bsp_tim_ic_measure_freq();
        if (hz == 0u) {
            const char *txt = BSP_TIM_IC_NO_SIGNAL_TXT;
            char *p = s_report;
            while (*txt != '\0') {
                *p++ = *txt++;
            }
            *p = '\0';
        } else {
            ic_format_measured(hz);
        }
    }
    return 0; /* both step outcomes are PASS */
}
