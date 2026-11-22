/**
 * @file bsp_tim_encoder.c
 * @brief TIM3 quadrature encoder interface (PB4/PB5) -- teaching module
 *        (HAL edition of f407-periph-lab bsp_tim_encoder).
 ******************************************************************************
 * == WHAT CUBEMX ALREADY DID (MX_TIM3_Init + MspInit) ======================
 * | Item          | Value (IOC truth)                 | Why
 * |---------------|-----------------------------------|-------------------
 * | TIM3 mode     | Encoder Interface (EncoderMode=   | IOC records the
 * |               | TI12, Period=65535, ARPE=ENABLE)  | intent; generated
 * |               |                                   | code stays at base
 * |               |                                   | init -- the encoder
 * |               |                                   | shape owned here
 * | Pins          | PB4=S_TIM3_CH1 (ENC_A),           | AF2, MspInit
 * |               | PB5=S_TIM3_CH2 (ENC_B)            | routed, no pull
 * | Kernel clock  | APB1 x2 = 84 MHz                  | RM0090 ch.6 (only
 * |               |                                   | the input filters
 * |               |                                   | tick at this rate --
 * |               |                                   | the counter itself
 * |               |                                   | is clocked by the
 * |               |                                   | encoder edges)
 * == HAL MAPPING (register concept -> HAL call) ============================
 *   SMCR SMS=011 (x4 decoding) ........ HAL_TIM_Encoder_Init with
 *       EncoderMode=TIM_ENCODERMODE_TI12 (encoder mode 3: counter clocks
 *       on BOTH TI1 and TI2 edges, rising AND falling). SMS=001/010 would
 *       count one channel only (x1 with x2 resolution loss).
 *   CCMR1 CC1S/CC2S=01 + IC1F/IC2F=0011 ... the same call: IC1Selection/
 *       IC2Selection=TIM_ICSELECTION_DIRECTTI (TI1FP1/TI2FP2 on their
 *       pins), IC1Filter/IC2Filter=3 (8 samples at f_DTS/8 must agree:
 *       ~762 ns contact-bounce rejection), IC1Prescaler/IC2Prescaler=
 *       TIM_ICPSC_DIV1 (a capture prescaler != 1 would silently drop
 *       quadrature counts).
 *   CCER CC1P/CC2P=0 (non-inverted) ... IC1Polarity/IC2Polarity=
 *       TIM_ICPOLARITY_RISING (swap CC1P=1 to flip the "forward"
 *       direction on TI1, Table 97).
 *   CCER CC1E|CC2E + CR1 CEN .......... HAL_TIM_Encoder_Start(&htim3,
 *       TIM_CHANNEL_ALL).
 *   PUPDR pull-up restore ............. HAL_GPIO_Init (GPIO_PULLUP) on
 *       ENC_A/ENC_B: the IOC wires the pair no-pull, but an unwired
 *       encoder line must rest at a stable high instead of floating
 *       around the logic threshold and faking phantom edges (the
 *       register edition set PUPDR itself; HAL edition re-applies the
 *       discipline through HAL, A3-clean).
 *   CNT reads ......................... __HAL_TIM_GET_COUNTER(&htim3).
 *   DIER .............................. never touched (polling lesson).
 *   Idempotent re-init ................ HAL_TIM_Encoder_Stop first (both
 *       channel state machines back to READY; Start on BUSY is a
 *       HAL_ERROR). HAL_TIM_Encoder_Init re-latches the CubeMX base
 *       values (PSC=0/ARR=65535/ARPE=1) via its internal
 *       TIM_Base_SetConfig and strokes UG: CNT=0, clean lap.
 * Count/speed math (the API formulas, unchanged):
 *   counts_per_rev = 4 * PPR (x4 from counting every edge of both phases).
 *   delta = (int16_t)(uint16_t)(end - start) -- wrap-safe across a 16-bit
 *   boundary because the cast re-folds the modulo result into
 *   [-32768, +32767].
 *   rpm = delta * 60000 / (4 * PPR * win_ms). delta*60000 <= 32768*60000
 *   ~ 1.97e9 < 2^31, so int32 arithmetic is exact for any legal win_ms.
 * == HOW TO VERIFY =========================================================
 * menu> tim_enc
 *   No encoder wired: "PASS tim_enc (idle)" -- an unwired probe is
 *   SKIP-state semantics, not a failure.
 *   With the encoder on PB4/PB5: step 2 still PASSes when the shaft is
 *   held still; spinning the shaft by hand reports "(moved, N cts/300ms)"
 *   and bsp_tim_encoder_speed_rpm(200) returns signed rpm (negative =
 *   reversed direction, the DIR-bit lesson).
 * Scope check: PB4/PB5 show the two quadrature square waves, B lagging A
 *   by 90 degrees for the "forward" direction.
 */
#include "bsp_tim_encoder.h"

#include <stddef.h>

#include "main.h"  /* ENC_A/ENC_B pin labels from CubeMX */
#include "stm32f4xx_hal.h"
#include "tim.h" /* htim3 from MX_TIM3_Init (CubeMX-generated) */

/* ---- module constants ----------------------------------------------------- */

#define BSP_TIM_ENCODER_ARR 65535u /* full 16-bit window (IOC Period) */

/* Self-test windows (HAL tick granularity, 1 ms). */
#define BSP_TIM_ENCODER_ST2_WIN_MS 200u /* stationary proof window */
#define BSP_TIM_ENCODER_ST3_WIN_MS 300u /* rotation probe window */

/* Self-test step numbers (the value bsp_tim_encoder_selftest returns). */
#define BSP_TIM_ENCODER_ST1_CONFIG 1u /* SMS/CCMR/CCER/ARR read-back */
#define BSP_TIM_ENCODER_ST2_STILL  2u /* 200 ms stationary, delta = 0 */
#define BSP_TIM_ENCODER_ST3_PROBE  3u /* 300 ms rotation probe */

/* ---- module state ----------------------------------------------------------- */

/* PPR is a parameter with a default (spec): runtime-writable so the rpm
 * math adapts to a different encoder without a rebuild. Single-threaded
 * menu context: plain static is safe. */
static uint32_t s_ppr = BSP_TIM_ENCODER_PPR_DEFAULT;

/* Report of the last selftest run; the menu reads it after the run. */
static char s_report[32];

/* ---- private helpers -------------------------------------------------------- */

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
    TIM_Encoder_InitTypeDef enc;
    GPIO_InitTypeDef gpio;

    /* CubeMX owns clocks/pins/base timebase; verify the handle is live. */
    if (htim3.Instance == NULL) {
        return -1;
    }

    /* Idempotent re-init: Encoder_Stop restores BOTH channel state
     * machines to READY and drops CC1E/CC2E/CEN (Start on a BUSY channel
     * is a HAL_ERROR). Harmless on the first call. */
    if (HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL) != HAL_OK) {
        return -1;
    }

    /* Idle-stable inputs: re-apply pull-ups over the IOC's no-pull
     * MspInit config (file header -- phantom-edge discipline). HAL call,
     * A3-clean: same pins HAL already owns, only the pull changes. */
    gpio.Pin = ENC_A_Pin | ENC_B_Pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* The mode switch + wiring harness in one HAL call: TI12 = count both
     * channels, both edge polarities (x4 decoding, ch.18.3.12); direct
     * inputs on their own pins, filtered, every edge (file header). */
    enc.EncoderMode = TIM_ENCODERMODE_TI12;
    enc.IC1Polarity = TIM_ICPOLARITY_RISING;     /* CC1P=0 */
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI; /* CC1S=01: TI1FP1 */
    enc.IC1Prescaler = TIM_ICPSC_DIV1;           /* every valid edge */
    enc.IC1Filter = 3u;                          /* IC1F=0011 */
    enc.IC2Polarity = TIM_ICPOLARITY_RISING;     /* CC2P=0 */
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI; /* CC2S=01: TI2FP2 */
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter = 3u;                          /* IC2F=0011 */
    if (HAL_TIM_Encoder_Init(&htim3, &enc) != HAL_OK) {
        return -1;
    }

    /* Timebase stays CubeMX's: PSC=0 (meaningless in encoder mode, edges
     * ARE the clock), ARR=65535 (IOC Period), ARPE=1 (IOC AutoReload
     * preload) -- Encoder_Init re-latches exactly those values and
     * strokes UG (CNT=0). Verified by read-back below, not rewritten. */

    /* Arm LAST: CC1E|CC2E + CEN in one HAL call (no interrupt routing --
     * polling lesson). */
    if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) {
        return -1;
    }

    /* Read-back proof: the encoder shape must have stuck. SMCR/CCMR1/
     * CCER have no HAL getters; those Instance reads are read-only
     * probes. */
    if ((htim3.Instance->SMCR & TIM_SMCR_SMS_Msk) != /* read-only (A3 exemption) */
        (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1)) {
        return -1;
    }
    if ((htim3.Instance->CCMR1 &                     /* read-only (A3 exemption) */
         (TIM_CCMR1_CC1S_Msk | TIM_CCMR1_CC2S_Msk | TIM_CCMR1_IC1F_Msk |
          TIM_CCMR1_IC2F_Msk)) !=
        (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 | TIM_CCMR1_IC1F_0 |
         TIM_CCMR1_IC1F_1 | TIM_CCMR1_IC2F_0 | TIM_CCMR1_IC2F_1)) {
        return -1;
    }
    if (((htim3.Instance->CCER &                      /* read-only (A3 exemption) */
          (TIM_CCER_CC1E | TIM_CCER_CC2E)) !=
         (TIM_CCER_CC1E | TIM_CCER_CC2E)) ||
        (__HAL_TIM_GET_AUTORELOAD(&htim3) != BSP_TIM_ENCODER_ARR)) {
        return -1;
    }
    return 0;
}

uint32_t bsp_tim_encoder_read_count(void)
{
    /* CNT wraps 0<->65535 (ARR max): the raw 16-bit position, EP23 style. */
    return __HAL_TIM_GET_COUNTER(&htim3);
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
         * trick that makes a 65535-lap counter usable as an odometer.
         * The wait is HAL_Delay (HAL tick): the whole HAL firmware
         * already bets on SysTick being alive (the register edition's
         * dead-SysTick special case has no HAL-side equivalent). */
        const uint32_t start = __HAL_TIM_GET_COUNTER(&htim3);
        HAL_Delay(win_ms);
        {
            const int32_t delta =
                (int32_t)(int16_t)(uint16_t)(__HAL_TIM_GET_COUNTER(&htim3) -
                                             start);
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
     * enables on, ARR = 65535, counter running (init already asserted
     * each; re-check standalone so a later failure points at the config,
     * not the window). */
    if ((htim3.Instance->SMCR & TIM_SMCR_SMS_Msk) != /* read-only (A3 exemption) */
        (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1)) {
        return (int)BSP_TIM_ENCODER_ST1_CONFIG;
    }
    if ((htim3.Instance->CCMR1 &                     /* read-only (A3 exemption) */
         (TIM_CCMR1_CC1S_Msk | TIM_CCMR1_CC2S_Msk | TIM_CCMR1_IC1F_Msk |
          TIM_CCMR1_IC2F_Msk)) !=
        (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 | TIM_CCMR1_IC1F_0 |
         TIM_CCMR1_IC1F_1 | TIM_CCMR1_IC2F_0 | TIM_CCMR1_IC2F_1)) {
        return (int)BSP_TIM_ENCODER_ST1_CONFIG;
    }
    if ((__HAL_TIM_GET_AUTORELOAD(&htim3) != BSP_TIM_ENCODER_ARR) ||
        ((htim3.Instance->CR1 & TIM_CR1_CEN) == 0u)) { /* read-only (A3 exemption) */
        return (int)BSP_TIM_ENCODER_ST1_CONFIG;
    }

    /* Step 2: stationary proof -- 200 ms without a single quadrature edge.
     * Pull-ups hold both lines high, the filter eats glitches: an idle
     * system MUST show delta = 0 (a nonzero delta here means phantom
     * counting -- wiring or noise, not software). */
    {
        const uint32_t c0 = __HAL_TIM_GET_COUNTER(&htim3);
        HAL_Delay(BSP_TIM_ENCODER_ST2_WIN_MS);
        if (((int32_t)(int16_t)(uint16_t)(__HAL_TIM_GET_COUNTER(&htim3) -
                                          c0)) != 0) {
            return (int)BSP_TIM_ENCODER_ST2_STILL;
        }
    }

    /* Step 3: rotation probe -- 300 ms. 0 delta -> "(idle)": nothing
     * wired, PASS by SKIP-state semantics (same verdict shape as tim_ic).
     * Any delta -> the encoder is alive; report the count magnitude. */
    {
        const uint32_t c0 = __HAL_TIM_GET_COUNTER(&htim3);
        HAL_Delay(BSP_TIM_ENCODER_ST3_WIN_MS);
        {
            const int32_t delta =
                (int32_t)(int16_t)(uint16_t)(__HAL_TIM_GET_COUNTER(&htim3) -
                                             c0);
            if (delta == 0) {
                enc_set_report("(idle)");
            } else {
                enc_format_moved(delta);
            }
        }
    }
    return 0; /* all step outcomes are PASS */
}
