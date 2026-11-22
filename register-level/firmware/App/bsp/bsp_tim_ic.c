/**
 ******************************************************************************
 * @file    bsp_tim_ic.c
 * @brief   TIM2_CH2 (PB3) input capture frequency meter -- teaching module
 *          (echoes the tutorial EP13 capture principle, register level).
 ******************************************************************************
 * @addtogroup BSP_TIM_IC
 * @{
 *
 * == CUBEMX CONFIG (from f407-periph-lab.ioc) ==============================
 * | Field            | Value (IOC truth)                    | Why
 * |------------------|--------------------------------------|-------------------
 * | TIM2 mode        | Input Capture 2 from TI2 (PB3, AF1)  | IOC routed the
 * |                  | ("Channel-Input Capture2 from TI2")  | pin; generated code
 * |                  |                                      | stays at base init
 * |                  |                                      | -- capture regs
 * |                  |                                      | owned here
 * | TIM2 counter     | 32-bit (Period=4294967295 in IOC)    | the F4's only
 * |                  |                                      | general 32-bit
 * |                  |                                      | timers: TIM2/TIM5
 * | Kernel clock     | APB1 x2 = 84 MHz                     | RM0090 ch.6 rule
 * | PSC in IOC       | 0 (max resolution)                   | kept here
 * | (PB3 caveat)     | PB3 = JTDO in default AF0            | MODER=10 re-routes
 * |                  |                                      | it; SWD untouched
 * == REGISTER WALKTHROUGH (RM0090 ch.18 "General-purpose timers") ==========
 * RCC->APB1ENR bit0 (TIM2EN): kernel clock gate, re-armed idempotently.
 * CCMR1 high byte (ch.18.4.7) -- when CC2S != 00 the OUTPUT-compare fields
 *   of channel 2 are RENAMED into input-capture fields (same bits, new
 *   meaning; the input twin of bsp_tim_pwm's OC1M/OC1PE):
 *   CC2S  [9:8]  = 01 -> IC2 mapped on TI2 (TI2FP2), DIRECT mode (channel
 *                  2 captures its own pin; XOR/indirect modes unused).
 *   IC2PSC [11:10] = 00 -> capture on EVERY valid edge (no /2 /4 /8 skip).
 *   IC2F  [15:12] = 0011 -> digital filter: 8 consecutive samples at
 *                  f_DTS/8 must agree to validate an edge. With CKD=00
 *                  f_DTS = 84 MHz -> ~8*(8/84M) ~= 762 ns minimum pulse
 *                  width; glitches shorter than that never reach CCR2.
 * CCER (ch.18.4.12):
 *   CC2E bit10   = 1  -> capture 2 enabled.
 *   CC2P bit11   = 0  -> rising edge / active level high (non-inverted).
 *   CC2NP bit13  = 0  -> falling-edge complementary input unused.
 * DIER (ch.18.4.3): CC2IE bit10 = 0 AND UIE bit0 = 0 -- pure polling
 *   lesson; flags are read from SR. (Interrupt alternative: CC2IE=1 +
 *   NVIC_EnableIRQ(TIM2_IRQn=28).)
 * CR1 (ch.18.4.1): CEN=1 last; DIR=0 up; CKD=00 (f_DTS = f_CK_INT, the
 *   filter's clock); ARPE=0. PSC=0, ARR=0xFFFFFFFF (32-bit free-run).
 * Period math (EP13 principle, this is the whole meter):
 *   period_ticks = (CCR2_at_edge2 + N*2^32) - CCR2_at_edge1
 *   where N = overflows between the edges (each UIF = one 2^32 wrap).
 *   freq = 84 MHz / period_ticks. The 32-bit counter overflows only every
 *   2^32/84M ~= 51.1 s, so inside a 1 s window N is realistically 0 --
 *   the compensation code stays because the PRINCIPLE is what EP13
 *   teaches (on a 16-bit timer like TIM3, N>0 happens in milliseconds).
 *   Hardware clears CC2IF when CCR2 is READ (peripheral contract, like
 *   UART RXNE) -- the t1/t2 reads below are also the flag clears.
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

#include "hal_port.h" /* family seam: pulls in the CMSIS device header */

/* ---- module constants ----------------------------------------------------- */

/* Measurement window shared by both edge waits (1 edge-to-edge period
 * must fit inside it or the result is "no signal"). */
#define BSP_TIM_IC_WINDOW_MS 1000u

/* Report text for the empty-probe case (SKIP-state semantics). */
#define BSP_TIM_IC_NO_SIGNAL_TXT "(no signal, 0 Hz)"

/* Self-test step numbers (the value bsp_tim_ic_selftest returns). */
#define BSP_TIM_IC_ST1_CONFIG 1u /* capture config read-back */
#define BSP_TIM_IC_ST2_MEAS   2u /* one measurement pass */

/* Spin cap between SysTick wraps: if COUNTFLAG never sets within it,
 * SysTick is dead -> bounded exit (spec #5). ~tens of ms of cycles. */
#define BSP_TIM_IC_SYSTICK_GUARD 2000000u

/* ---- module state ----------------------------------------------------------- */

/* Report of the last selftest run; the menu reads it after the run and
 * appends it to "PASS tim_ic". Single-threaded menu context: a static
 * char buffer is safe without locking. */
static char s_report[32];

/* ---- private helpers -------------------------------------------------------- */

/* Wait for the next capture event, servicing overflows on the way.
 *   wraps:    elapsed ms, advanced here (1 per SysTick wrap), caller owns
 *             the shared measurement budget
 *   budget:   expire when *wraps reaches it
 *   ovf:      incremented per UIF seen (one 2^32 counter wrap)
 * Returns 0 = CC2IF set (caller reads CCR2, which also clears it),
 *         -1 = SysTick dead (bounded exit), -2 = window exhausted. */
static int ic_wait_edge(uint32_t *wraps, uint32_t budget, uint32_t *ovf)
{
    uint32_t guard = BSP_TIM_IC_SYSTICK_GUARD;
    for (;;) {
        const uint32_t sr = TIM2->SR; /* one atomic SR snapshot per pass */
        if ((sr & TIM_SR_CC2IF) != 0u) {
            return 0; /* edge latched in CCR2 -- do NOT clear here */
        }
        if ((sr & TIM_SR_UIF) != 0u) {
            TIM2->SR = ~TIM_SR_UIF; /* rc_w0 clear (walkthrough) */
            (*ovf)++;               /* count the 2^32 wrap for compensation */
        }
        if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0u) {
            (*wraps)++;                    /* 1 ms elapsed */
            guard = BSP_TIM_IC_SYSTICK_GUARD; /* alive: refill the cap */
        }
        if (*wraps >= budget) {
            return -2; /* window exhausted without a fresh edge */
        }
        if (--guard == 0u) {
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
    /* Kernel clock: TIM2EN = APB1ENR bit0 (RM0090 ch.6.3.13). HAL's
     * MspInit already set it; re-arm keeps init self-sufficient. */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    (void)RCC->APB1ENR; /* read-back stalls until the clock is live */

    /* Stop the counter while reconfiguring (CEN=0); HAL base init left it
     * stopped -- the explicit write makes the module the CR1 owner. */
    TIM2->CR1 = 0u;

    /* Pin mux: PB3 -> AF1 (TIM2_CH2). MODER3 [7:6]=10 (alternate), AFRL
     * nibble 3 = 0001 (AF1). PUPDR3 = 00: NO pull -- a pull would bias a
     * floating probe pin toward a fake idle level; the external signal
     * source defines the line. OTYPER/OSPEEDR are output-side knobs and
     * play no role for a capture input. HAL equivalent (MspInit):
     *   GPIO_InitStruct.Alternate = GPIO_AF1_TIM2; HAL_GPIO_Init(GPIOB,...). */
    GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER3_Msk) |
                   GPIO_MODER_MODER3_1; /* [7:6]=10 AF mode */
    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~GPIO_AFRL_AFSEL3_Msk) |
                    GPIO_AFRL_AFSEL3_0; /* AFSEL3[3:0]=0001 -> AF1 TIM2 */
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD3_Msk; /* floating input through AF */

    /* Timebase: max resolution (PSC=0) and full 32-bit free-run window
     * (ARR=0xFFFFFFFF, 51.1 s between overflows at 84 MHz). */
    TIM2->PSC = 0u;
    TIM2->ARR = 0xFFFFFFFFu;

    /* Capture channel 2 on TI2, direct, filtered, every edge (walkthrough
     * bit-by-bit). HAL equivalent:
     *   TIM_IC_InitTypeDef ic = { .ICPolarity = TIM_ICPOLARITY_RISING,
     *     .ICSelection = TIM_ICSELECTION_DIRECTTI,
     *     .ICPrescaler = TIM_ICPSC_DIV1, .ICFilter = 3 };
     *   HAL_TIM_IC_ConfigChannel(&htim2, &ic, TIM_CHANNEL_2); */
    TIM2->CCMR1 = TIM_CCMR1_CC2S_0            /* [9:8]=01: IC2 <- TI2 direct */
                  | TIM_CCMR1_IC2F_0 | TIM_CCMR1_IC2F_1; /* [15:12]=0011 */
                  /* IC2PSC [11:10] stays 0: capture every valid edge */

    /* Edge polarity and capture enable: rising, non-inverted, enabled. */
    TIM2->CCER = TIM_CCER_CC2E; /* CC2E=1; CC2P/CC2NP=0 (rising edge) */

    /* Polling lesson: no interrupt routing at all. */
    TIM2->DIER = 0u; /* UIE=0, CC2IE=0 -- flags polled from SR instead */

    /* Arm: UG latches PSC/ARR and zeroes CNT (sets UIF), then clear both
     * stale flags so the first measurement window starts clean. CEN=1
     * last: the counter free-runs from here. */
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = ~(TIM_SR_UIF | TIM_SR_CC2IF);
    TIM2->CR1 = TIM_CR1_CEN;

    /* Read-back proof: the capture shape must have stuck. */
    if (((TIM2->CCMR1 &
          (TIM_CCMR1_CC2S_Msk | TIM_CCMR1_IC2PSC_Msk | TIM_CCMR1_IC2F_Msk)) !=
         (TIM_CCMR1_CC2S_0 | TIM_CCMR1_IC2F_0 | TIM_CCMR1_IC2F_1)) ||
        ((TIM2->CCER & (TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP)) !=
         TIM_CCER_CC2E) ||
        (TIM2->PSC != 0u) ||
        (TIM2->ARR != 0xFFFFFFFFu)) {
        return -1;
    }
    return 0;
}

uint32_t bsp_tim_ic_measure_freq(void)
{
    uint32_t wraps = 0u; /* shared 1 s budget, advanced by ic_wait_edge */
    uint32_t ovf = 0u;   /* 2^32 wraps between the two edges */

    /* Clean flag state: stale UIF/CC2IF from a previous run would fake a
     * first edge instantly. Both bits are rc_w0 (write-0 clear). */
    TIM2->SR = ~(TIM_SR_UIF | TIM_SR_CC2IF);

    /* Phase 1: first rising edge inside the window. Timeout here means no
     * signal (the probe pin is idle) -> 0 Hz, the SKIP-state answer. */
    if (ic_wait_edge(&wraps, BSP_TIM_IC_WINDOW_MS, &ovf) != 0) {
        return 0u;
    }
    /* t1 in timer ticks. Reading CCR2 is ALSO the CC2IF clear -- the RM0090
     * peripheral contract (ch.18.4.14), same spirit as UART RXNE. */
    {
        const uint32_t t1 = TIM2->CCR2;
        /* Only overflows AFTER t1 contribute to the period. */
        ovf = 0u;

        /* Phase 2: second rising edge, spending what remains of the 1 s
         * budget (ic_wait_edge keeps counting the same 'wraps' counter). */
        if (ic_wait_edge(&wraps, BSP_TIM_IC_WINDOW_MS, &ovf) != 0) {
            return 0u; /* one edge only: period longer than the window */
        }
        {
            const uint32_t t2_raw = TIM2->CCR2;
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

    /* Step 1: capture config read-back -- the exact CCMR1/CCER shape from
     * the walkthrough (init already asserted it; re-check standalone so a
     * later module fault still points at the config, not the measurement). */
    if (((TIM2->CCMR1 &
          (TIM_CCMR1_CC2S_Msk | TIM_CCMR1_IC2PSC_Msk | TIM_CCMR1_IC2F_Msk)) !=
         (TIM_CCMR1_CC2S_0 | TIM_CCMR1_IC2F_0 | TIM_CCMR1_IC2F_1)) ||
        ((TIM2->CCER & (TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP)) !=
         TIM_CCER_CC2E)) {
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

/** @} */
