/**
 ******************************************************************************
 * @file    bsp_tim_encoder.c
 * @brief   TIM3 quadrature encoder interface (PB4/PB5) -- teaching module
 *          with register-level init and self-test.
 ******************************************************************************
 * @addtogroup BSP_TIM_ENCODER
 * @{
 *
 * == CUBEMX CONFIG (from f407-periph-lab.ioc) ==============================
 * | Field             | Value (IOC truth)                    | Why
 * |-------------------|--------------------------------------|-------------------
 * | TIM3 mode         | Encoder Interface (EncoderMode=TI12) | IOC records the
 * |                   |                                      | intent; generated
 * |                   |                                      | code stays at
 * |                   |                                      | base init -- the
 * |                   |                                      | encoder regs are
 * |                   |                                      | owned here
 * | TIM3 Period       | 65535 (ARPE=ENABLE)                  | full 16-bit
 * |                   |                                      | window; the free
 * |                   |                                      | wrap IS the EP23
 * |                   |                                      | lesson
 * | Pins              | PB4=S_TIM3_CH1 (ENC_A), PB5=S_TIM3_CH2 (ENC_B) | AF2
 * | Kernel clock      | APB1 x2 = 84 MHz                     | RM0090 ch.6
 * |                   |                                      | (only the input
 * |                   |                                      | filters tick at
 * |                   |                                      | this rate -- the
 * |                   |                                      | counter itself is
 * |                   |                                      | clocked by the
 * |                   |                                      | encoder edges)
 * == REGISTER WALKTHROUGH (RM0090 ch.18, "Encoder interface mode" 18.3.12) ==
 * RCC->APB1ENR bit1 (TIM3EN): kernel clock gate, re-armed idempotently.
 * Pin mux: PB4 -> AF2 (TIM3_CH1). PB4 is NJTRST in default AF0 -- a JTAG
 *   remnant; writing AF2 re-routes it and leaves SWD (PA13/PA14) untouched
 *   (the PB3/JTDO twin move in bsp_tim_ic). PB5 has no debug role: plain
 *   AF2. Both pins get PULL-UP (PUPDR=01): an unwired encoder line then
 *   rests at a stable high instead of floating around the logic threshold
 *   and generating phantom edges; real encoders (push-pull or open-
 *   collector with external pull-ups) drive through it without effort.
 * CCMR1 (ch.18.4.7) -- both channels in INPUT mode, the encoder's wiring
 *   harness:
 *   CC1S [1:0]   = 01 -> IC1 mapped on TI1 (TI1FP1 on the CH1 pin).
 *   CC2S [9:8]   = 01 -> IC2 mapped on TI2 (TI2FP2 on the CH2 pin).
 *   IC1F [7:4] / IC2F [15:12] = 0011 -> digital filter: 8 consecutive
 *                  samples at f_DTS/8 must agree to validate an edge. With
 *                  CKD=00 f_DTS = 84 MHz -> ~762 ns minimum pulse width;
 *                  contact-bounce spikes shorter than that never reach the
 *                  counter.
 *   IC1PSC/IC2PSC = 00 -> every valid edge (a capture prescaler != 1 would
 *                  silently drop quadrature counts).
 * SMCR (ch.18.4.3) -- the mode switch:
 *   SMS [2:0]    = 011 -> Encoder mode 3: counter clocks on BOTH TI1 and
 *                  TI2 edges (rising AND falling) = full x4 decoding; the
 *                  sequencer compares the two channels to synthesize the
 *                  count pulses AND the direction. SMS=001/010 would count
 *                  one channel only (x1 with x2 resolution loss).
 *   TS [6:4]/MSM = 000/0 -- ignored in encoder mode (no trigger chain).
 * CCER (ch.18.4.9):
 *   CC1E bit0 / CC2E bit4 = 1 -> both capture inputs enabled.
 *   CC1P/CC1NP/CC2P/CC2NP = 0 -> non-inverted phases (swap CC1P=1 to flip
 *                  the "forward" direction on TI1, Table 97).
 * CR1/ARR (ch.18.4.1/.4.11): PSC=0 (meaningless in encoder mode, edges ARE
 *   the clock), ARR=65535 -> the 16-bit counter wraps 0<->65535 and the
 *   content always represents position modulo a 16-bit lap (EP23 wrap-
 *   around counting). ARPE=1 honors the IOC AutoReloadPreload=ENABLE.
 *   DIER=0: polling lesson, no interrupts. CEN=1 written LAST; the UG
 *   stroke latches ARR into the shadow register first.
 * Count/speed math (the API formulas):
 *   counts_per_rev = 4 * PPR  (x4 from counting every edge of both phases).
 *   delta = (int16_t)(uint16_t)(end - start)  -- wrap-safe across a 16-bit
 *   boundary because the cast re-folds the modulo result into [-32768, +32767].
 *   rpm = delta * 60000 / (4 * PPR * win_ms). delta*60000 <= 32768*60000 ~
 *   1.97e9 < 2^31, so int32 arithmetic is exact for any legal win_ms.
 * == HOW TO VERIFY =========================================================
 * menu> tim_enc
 *   No encoder wired: "PASS tim_enc (idle)" -- an unwired probe is
 *   SKIP-state semantics, not a failure.
 *   With the encoder on PB4/PB5: step 2 still PASSes when the shaft is
 *   held still; spinning the shaft by hand prints "(moved, N cts/300ms)"
 *   and bsp_tim_encoder_speed_rpm(200) returns signed rpm (negative =
 *   reversed direction, the DIR-bit lesson).
 * Scope check: PB4/PB5 show the two quadrature square waves, B lagging A
 *   by 90 degrees for the "forward" direction.
 */
#include "bsp_tim_encoder.h"

#include "hal_port.h" /* family seam: pulls in the CMSIS device header */

/* ---- module constants ----------------------------------------------------- */

#define BSP_TIM_ENCODER_ARR 65535u /* full 16-bit window (IOC Period) */

/* Self-test windows (SysTick COUNTFLAG granularity, 1 ms). */
#define BSP_TIM_ENCODER_ST2_WIN_MS 200u /* stationary proof window */
#define BSP_TIM_ENCODER_ST3_WIN_MS 300u /* rotation probe window */

/* Self-test step numbers (the value bsp_tim_encoder_selftest returns). */
#define BSP_TIM_ENCODER_ST1_CONFIG 1u /* SMS/CCMR/CCER/ARR read-back */
#define BSP_TIM_ENCODER_ST2_STILL  2u /* 200 ms stationary, delta = 0 */
#define BSP_TIM_ENCODER_ST3_PROBE  3u /* 300 ms rotation probe */

/* Spin cap between SysTick wraps: if COUNTFLAG never sets within it,
 * SysTick is dead -> bounded exit (spec #5). Same guard as bsp_tim_ic. */
#define BSP_TIM_ENCODER_SYSTICK_GUARD 2000000u

/* ---- module state ----------------------------------------------------------- */

/* PPR is a parameter with a default (spec): runtime-writable so the rpm
 * math adapts to a different encoder without a rebuild. Single-threaded
 * menu context: plain static is safe. */
static uint32_t s_ppr = BSP_TIM_ENCODER_PPR_DEFAULT;

/* Report of the last selftest run; the menu reads it after the run. */
static char s_report[32];

/* ---- private helpers -------------------------------------------------------- */

/* Delay win_ms by polling the SysTick COUNTFLAG (1 ms per wrap). Returns
 * 0 = window elapsed, -1 = SysTick dead (bounded exit, never hang). */
static int enc_wait_ms(uint32_t win_ms)
{
    uint32_t guard = BSP_TIM_ENCODER_SYSTICK_GUARD;
    while (win_ms > 0u) {
        if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0u) {
            win_ms--;
            guard = BSP_TIM_ENCODER_SYSTICK_GUARD; /* alive: refill */
        }
        if (--guard == 0u) {
            return -1;
        }
    }
    return 0;
}

/* Copy a fixed string into s_report (tiny, no libc). */
static void enc_set_report(const char *txt)
{
    char *p = s_report;
    while (*txt != '\0') {
        *p++ = *txt++;
    }
    *p = '\0';
}

/* Format "(moved, N cts/300ms)" with the decimal magnitude of delta. */
static void enc_format_moved(int32_t delta)
{
    static const char digits[10] = {
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9'
    };
    char tmp[10];
    unsigned n = 0u;
    uint32_t v = (delta < 0) ? (uint32_t)(-delta) : (uint32_t)delta;
    const char *pfx = "(moved, ";
    const char *sfx = " cts/300ms)";
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

int bsp_tim_encoder_init(void)
{
    /* Kernel clock: TIM3EN = APB1ENR bit1 (RM0090 ch.6.3.13). HAL's
     * MspInit already set it; re-arm keeps init self-sufficient. */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    (void)RCC->APB1ENR; /* read-back stalls until the clock is live */

    /* Stop the counter while reconfiguring (CEN=0); the explicit write
     * makes this module the CR1 owner. */
    TIM3->CR1 = 0u;

    /* Pin mux: PB4 -> AF2 (TIM3_CH1), PB5 -> AF2 (TIM3_CH2). MODER [9:8]/
     * [11:10] = 10 (alternate), AFRL nibbles 4/5 = 0010. PUPDR = 01
     * (pull-up): idle-stable input, open-collector friendly. HAL
     * equivalent (MspInit):
     *   GPIO_InitStruct.Alternate = GPIO_AF2_TIM3; HAL_GPIO_Init(GPIOB,...). */
    GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER4_Msk) |
                   GPIO_MODER_MODER4_1; /* [9:8]=10 AF mode */
    GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER5_Msk) |
                   GPIO_MODER_MODER5_1; /* [11:10]=10 AF mode */
    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~GPIO_AFRL_AFSEL4_Msk) |
                    GPIO_AFRL_AFSEL4_1; /* AFSEL4[3:0]=0010 -> AF2 TIM3 */
    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~GPIO_AFRL_AFSEL5_Msk) |
                    GPIO_AFRL_AFSEL5_1; /* AFSEL5[3:0]=0010 -> AF2 TIM3 */
    GPIOB->PUPDR = (GPIOB->PUPDR & ~GPIO_PUPDR_PUPD4_Msk) |
                   GPIO_PUPDR_PUPD4_0; /* PB4 pull-up */
    GPIOB->PUPDR = (GPIOB->PUPDR & ~GPIO_PUPDR_PUPD5_Msk) |
                   GPIO_PUPDR_PUPD5_0; /* PB5 pull-up */

    /* Timebase: PSC=0 (edges clock the counter directly), ARR = 65535
     * full window. UG latches the ARR shadow (ARPE=1) and zeroes CNT. */
    TIM3->PSC = 0u;
    TIM3->ARR = BSP_TIM_ENCODER_ARR;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = ~TIM_SR_UIF; /* UG raised UIF; clear it (SR is rc_w0) */

    /* Both channels as filtered inputs on their own pins (walkthrough).
     * HAL equivalent:
     *   TIM_Encoder_InitTypeDef enc = {
     *     .EncoderMode = TIM_ENCODERMODE_TI12,
     *     .IC1Polarity = TIM_ICPOLARITY_RISING, .IC1Filter = 3,
     *     .IC2Polarity = TIM_ICPOLARITY_RISING, .IC2Filter = 3 };
     *   HAL_TIM_Encoder_Init(&htim3, &enc); */
    TIM3->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0            /* TI1FP1/TI2FP2 */
                  | TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1          /* IC1F=0011 */
                  | TIM_CCMR1_IC2F_0 | TIM_CCMR1_IC2F_1;         /* IC2F=0011 */
                  /* IC1PSC/IC2PSC stay 0: capture EVERY edge */

    /* The mode switch: SMS=011 encoder mode 3 = count both channels,
     * both edge polarities (x4 decoding, ch.18.3.12). */
    TIM3->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1; /* TS/MSM stay 0 */

    /* Inputs live, phases non-inverted (CC1P/CC2P/CCxNP = 0). */
    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;

    /* Polling lesson: no interrupt routing at all. */
    TIM3->DIER = 0u;

    /* Arm LAST: CEN=1 + ARPE=1 (IOC AutoReloadPreload=ENABLE). */
    TIM3->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;

    /* Read-back proof: the encoder shape must have stuck. */
    if ((TIM3->SMCR & TIM_SMCR_SMS_Msk) !=
        (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1)) {
        return -1;
    }
    if ((TIM3->CCMR1 &
         (TIM_CCMR1_CC1S_Msk | TIM_CCMR1_CC2S_Msk | TIM_CCMR1_IC1F_Msk |
          TIM_CCMR1_IC2F_Msk)) !=
        (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 | TIM_CCMR1_IC1F_0 |
         TIM_CCMR1_IC1F_1 | TIM_CCMR1_IC2F_0 | TIM_CCMR1_IC2F_1)) {
        return -1;
    }
    if (((TIM3->CCER & (TIM_CCER_CC1E | TIM_CCER_CC2E)) !=
         (TIM_CCER_CC1E | TIM_CCER_CC2E)) ||
        (TIM3->ARR != BSP_TIM_ENCODER_ARR)) {
        return -1;
    }
    return 0;
}

uint32_t bsp_tim_encoder_read_count(void)
{
    /* CNT wraps 0<->65535 (ARR max): the raw 16-bit position, EP23 style. */
    return TIM3->CNT;
}

int bsp_tim_encoder_set_ppr(uint32_t ppr)
{
    if ((ppr < 1u) || (ppr > 65535u)) {
        return -1; /* reject nonsense, keep the previous value */
    }
    s_ppr = ppr;
    return 0;
}

int32_t bsp_tim_encoder_speed_rpm(uint32_t win_ms)
{
    if (win_ms == 0u) {
        return 0; /* zero window would divide by nothing */
    }
    {
        /* Sample, wait, sample. (int16_t)(uint16_t) refolds the 16-bit
         * modulo difference into [-32768, +32767] -- the wrap-safe delta
         * trick that makes a 65535-lap counter usable as an odometer. */
        const uint32_t start = TIM3->CNT;
        if (enc_wait_ms(win_ms) != 0) {
            return 0; /* SysTick dead: report standstill, never hang */
        }
        {
            const int32_t delta =
                (int32_t)(int16_t)(uint16_t)(TIM3->CNT - start);
            /* rpm = delta * 60000 / (4 * PPR * win_ms). Max magnitude
             * 32768 * 60000 ~= 1.97e9 < 2^31: exact in int32. */
            const int32_t denom =
                (int32_t)(4u * s_ppr) * (int32_t)win_ms;
            return (delta * 60000) / denom;
        }
    }
}

const char *bsp_tim_encoder_last_report(void)
{
    return s_report;
}

int bsp_tim_encoder_selftest(void)
{
    /* Fresh init first (idempotent, menu-driven selftest pattern). */
    if (bsp_tim_encoder_init() != 0) {
        return (int)BSP_TIM_ENCODER_ST1_CONFIG;
    }

    /* Step 1: config read-back -- SMS=011, both channels mapped + filtered,
     * enables on, ARR = 65535 (init already asserted each; re-check
     * standalone so a later failure points at the config, not the window). */
    if ((TIM3->SMCR & TIM_SMCR_SMS_Msk) !=
        (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1)) {
        return (int)BSP_TIM_ENCODER_ST1_CONFIG;
    }
    if ((TIM3->CCMR1 &
         (TIM_CCMR1_CC1S_Msk | TIM_CCMR1_CC2S_Msk | TIM_CCMR1_IC1F_Msk |
          TIM_CCMR1_IC2F_Msk)) !=
        (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 | TIM_CCMR1_IC1F_0 |
         TIM_CCMR1_IC1F_1 | TIM_CCMR1_IC2F_0 | TIM_CCMR1_IC2F_1)) {
        return (int)BSP_TIM_ENCODER_ST1_CONFIG;
    }
    if ((TIM3->ARR != BSP_TIM_ENCODER_ARR) ||
        ((TIM3->CR1 & TIM_CR1_CEN) == 0u)) {
        return (int)BSP_TIM_ENCODER_ST1_CONFIG;
    }

    /* Step 2: stationary proof -- 200 ms without a single quadrature edge.
     * Pull-ups hold both lines high, the filter eats glitches: an idle
     * system MUST show delta = 0 (a nonzero delta here means phantom
     * counting -- wiring or noise, not software). */
    {
        const uint32_t c0 = TIM3->CNT;
        if (enc_wait_ms(BSP_TIM_ENCODER_ST2_WIN_MS) != 0) {
            return (int)BSP_TIM_ENCODER_ST2_STILL; /* SysTick dead */
        }
        if (((int32_t)(int16_t)(uint16_t)(TIM3->CNT - c0)) != 0) {
            return (int)BSP_TIM_ENCODER_ST2_STILL;
        }
    }

    /* Step 3: rotation probe -- 300 ms. 0 delta -> "(idle)": nothing wired,
     * PASS by SKIP-state semantics (same verdict shape as tim_ic). Any
     * delta -> the encoder is alive; report the count magnitude. */
    {
        const uint32_t c0 = TIM3->CNT;
        if (enc_wait_ms(BSP_TIM_ENCODER_ST3_WIN_MS) != 0) {
            return (int)BSP_TIM_ENCODER_ST3_PROBE;
        }
        {
            const int32_t delta =
                (int32_t)(int16_t)(uint16_t)(TIM3->CNT - c0);
            if (delta == 0) {
                enc_set_report("(idle)");
            } else {
                enc_format_moved(delta);
            }
        }
    }
    return 0; /* all step outcomes are PASS */
}

/** @} */
