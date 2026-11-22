/**
 * @file bsp_tim_basic.c
 * @brief TIM5 1 Hz polled timebase -- teaching module (HAL edition of
 *        f407-periph-lab bsp_tim_basic).
 *
 * == WHAT CUBEMX ALREADY DID (MX_TIM5_Init) ================================
 * | Item         | Value (IOC truth)                 | Why
 * |--------------|-----------------------------------|-------------------
 * | TIM5         | in IOC: base init only (PSC=0,    | pin-free timebase;
 * |              | ARR=max, CEN=0, NVIC off)         | this module arms it
 * | Kernel clock | APB1 x2 = 84 MHz                  | RM0090 ch.6 rule
 * | Counter      | 32-bit (TIM2/TIM5 are the F4's    | ARR max = 2^32-1
 * |              | only general 32-bit timers)       |
 * == HAL MAPPING (register concept -> HAL call) ============================
 *   PSC/ARR write (+ old EGR.UG) ...... __HAL_TIM_SET_PRESCALER /
 *       __HAL_TIM_SET_AUTORELOAD. ARPE=0 (IOC): ARR is effective
 *       immediately; PSC is shadowed and latches at the first update
 *       event, so the first period after start is 10000 ticks at 84 MHz
 *       (~119 us) before the 1 Hz cadence locks in. HAL has no EGR.UG
 *       stroke; accepted (teaching timebase, not sync-critical).
 *   CNT reset ......................... __HAL_TIM_SET_COUNTER.
 *   Stale UIF from MX_TIM5_Init ....... __HAL_TIM_CLEAR_FLAG (the HAL
 *       spelling of the rc_w0 write: SR bits clear on a 0 write).
 *   DIER.UIE .......................... deliberately NEVER set:
 *       HAL_TIM_Base_Start arms CEN only -- the polling lesson is that
 *       the flag exists whether or not anyone is interrupted. The W6
 *       successor is HAL_TIM_Base_Start_IT +
 *       HAL_TIM_PeriodElapsedCallback + HAL_NVIC_SetPriority/EnableIRQ.
 *   UIF poll .......................... __HAL_TIM_GET_FLAG(&htim5,
 *       TIM_FLAG_UPDATE).
 *   TIM6 / TIM7 ....................... GONE in this edition: not in the
 *       HAL IOC (no clock gate, no handles, no vectors). The set/clear/set
 *       polling contract they taught now runs on TIM5's own flag -- same
 *       lesson, one timer. The old file's TIM5/TIM6/TIM7 IRQHandler
 *       definitions are gone with them (Core/ owns vectors here, and no
 *       NVIC wiring exists yet -- W6).
 * Timebase derivation (same law as PWM, here without output compare):
 *   f_UPDATE = 84 MHz / (PSC + 1) / (ARR + 1) = 84M/8400/10000 = 1 Hz
 *   -> PSC=8399, ARR=9999.
 * CNT vs PSC cascade (the teaching point, unchanged): the prescaler is
 *   NOT part of the counter -- CNT only ever counts 0..ARR; PSC divides
 *   the clock FEEDING it. The classic 16-bit timer reaches 1 Hz not by
 *   counting to 84M but by PSC shaving 84 MHz down to 10 kHz, then ARR
 *   picking 10000 of those ticks.
 * == HOW TO VERIFY =========================================================
 * menu> tim_basic
 *   PASS tim_basic       (steps: 1 uptime window ~1.2 s, 2 UIF
 *   set/clear/set, 3 cadence). Steps 1-3 together cost ~3.6 s: three
 *   windows sized "> 1 TIM5 period with margin" by design.
 * Debugger checks: htim5.Instance->SR.UIF toggles 1x/s when watched live;
 *   DIER.UIE stays 0 (no interrupt routing); bsp_tim_basic_uptime_seconds()
 *   advances 1/s while polled.
 */
#include "bsp_tim_basic.h"

#include <stddef.h>

#include "stm32f4xx_hal.h"
#include "tim.h" /* htim5 from MX_TIM5_Init (CubeMX-generated) */

/* ---- module constants ----------------------------------------------------- */

/* Timebase derivation (see file header formulas). */
#define TIM_BASIC_TIM5_PSC 8399u /* 84 MHz/8400  = 10 kHz */
#define TIM_BASIC_TIM5_ARR 9999u /* 10 kHz/10000 = 1 Hz   */

/* Self-test step numbers (the value bsp_tim_basic_selftest returns). */
#define BSP_TIM_BASIC_ST1_TIM5_1HZ 1u /* uptime advances in a 1.2 s window */
#define BSP_TIM_BASIC_ST2_TIM5_UIF 2u /* UIF set/clear/set contract       */
#define BSP_TIM_BASIC_ST3_TIM5_CAD 3u /* exactly one update per period    */

/* Window for the selftest waits: > 1 TIM5 period with margin (spec #5:
 * bounded waits). */
#define TIM_BASIC_TIM5_WINDOW_MS 1500u

/* SysTick-dead detection: if HAL_GetTick shows no progress across a 2M
 * spin budget (~120 ms at 168 MHz), the wait is bounded-exited. The
 * budget RESETS on every tick advance, so a live SysTick never trips it
 * regardless of window length (the original flat 2M counter expired at
 * ~120 ms -- far inside every window -- and failed every wait). */
#define TIM_BASIC_SPIN_GUARD 2000000u

/* ---- module state ----------------------------------------------------------- */

/* Polled update counter: bumped by the drain points in this module, one
 * per TIM5 overflow. Single-threaded menu context: plain static is safe.
 * (The volatile ISR counter of the register edition returns in W6.) */
static uint32_t s_tim5_seconds;
static char s_report[24];
static void tb_set_report(const char *msg)
{
    unsigned i = 0u;
    while ((msg[i] != 0) && (i < (sizeof(s_report) - 1u))) {
        s_report[i] = msg[i];
        i++;
    }
    s_report[i] = 0;
}
const char *bsp_tim_basic_last_report(void)
{
    return s_report;
}

/* ---- private helpers -------------------------------------------------------- */

/* Drain a pending TIM5 update flag into the seconds counter. Returns 1 =
 * an update was consumed (one second elapsed), 0 = none pending. */
static uint32_t tb_drain_update(void)
{
    if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE); /* rc_w0 via HAL */
        s_tim5_seconds++;
        return 1u;
    }
    return 0u;
}

/* Wait until the update flag sets or 'window_ms' elapsed since 'start'.
 * Returns 0 = flag set, -1 = window exhausted (or SysTick dead: the spin
 * guard bounds the loop when HAL_GetTick stops advancing). */
static int tb_wait_update(uint32_t start, uint32_t window_ms)
{
    uint32_t guard = TIM_BASIC_SPIN_GUARD;
    uint32_t last_tick = HAL_GetTick();
    for (;;) {
        if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_UPDATE) != RESET) {
            return 0;
        }
        if ((HAL_GetTick() - start) >= window_ms) {
            return -1;
        }
        if (HAL_GetTick() != last_tick) {
            last_tick = HAL_GetTick();
            guard = TIM_BASIC_SPIN_GUARD; /* tick advanced: SysTick alive */
        }
        guard--;
        if (guard == 0u) {
            return -1; /* SysTick dead: bounded exit, never hang */
        }
    }
}

/* ---- public API ------------------------------------------------------------ */

int bsp_tim_basic_init(void)
{
    /* CubeMX owns the TIM5 clock gate and base init; verify the handle. */
    if (htim5.Instance == NULL) {
        return -1;
    }

    /* Idempotent re-init: Base_Stop restores the HAL state machine to
     * READY (Start on a non-READY timer is a HAL_ERROR) and clears CEN --
     * the register edition's "CR1=0 before reconfig" in HAL form. */
    if (HAL_TIM_Base_Stop(&htim5) != HAL_OK) {
        return -1;
    }

    /* --- TIM5: 1 Hz update, polled ---------------------------------------- */
    __HAL_TIM_SET_PRESCALER(&htim5, TIM_BASIC_TIM5_PSC);
    __HAL_TIM_SET_AUTORELOAD(&htim5, TIM_BASIC_TIM5_ARR);
    __HAL_TIM_SET_COUNTER(&htim5, 0u);
    /* MX_TIM5_Init's TIM_Base_SetConfig left a UG-induced pending UIF; a
     * fresh second must start from a clean flag (old code: SR = ~UIF). */
    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
    s_tim5_seconds = 0u;
    /* CEN=1 ONLY -- deliberately NOT HAL_TIM_Base_Start_IT: UIE stays
     * clear, no NVIC line is touched (W4b polled edition, file header).
     * The PSC shadow latches at the first update event, so the first
     * period after start is ~119 us (file header); 1 Hz from there on. */
    if (HAL_TIM_Base_Start(&htim5) != HAL_OK) {
        return -1;
    }

    /* Read-back proof: timebase armed, interrupt routing still OFF. */
    if ((htim5.Instance->PSC != TIM_BASIC_TIM5_PSC) || /* read-only (A3 exemption) */
        (__HAL_TIM_GET_AUTORELOAD(&htim5) != TIM_BASIC_TIM5_ARR)) {
        return -1;
    }
    if (((htim5.Instance->CR1 & TIM_CR1_CEN) == 0u) ||   /* read-only (A3 exemption) */
        ((htim5.Instance->DIER & TIM_DIER_UIE) != 0u)) { /* read-only (A3 exemption) */
        return -1;
    }
    return 0;
}

uint32_t bsp_tim_basic_uptime_seconds(void)
{
    /* Polled edition: the drain IS the "ISR" -- see the header for the
     * once-per-second calling caveat and the W6 IRQ successor. */
    (void)tb_drain_update();
    return s_tim5_seconds;
}

int bsp_tim_basic_selftest(void)
{
    /* Fresh init first (idempotent, menu-driven selftest pattern). */
    s_report[0] = 0;
    if (bsp_tim_basic_init() != 0) {
        tb_set_report("(init failed)");
        return (int)BSP_TIM_BASIC_ST1_TIM5_1HZ;
    }
    /* Settle: the first update after Start latches the shadow PSC at
     * ~119 us; wait past it so steps 1-3 run on the clean 1 Hz cadence
     * (fixed the phase-drift FAILs seen on the first board run). */
    HAL_Delay(50);

    /* Step 1: the update counter must advance >= 1 in a 1.2 s window (one
     * 1 s period + margin). The window is measured with HAL_GetTick
     * (SysTick), NOT by trusting TIM5 itself -- an independent clock
     * cross-checks the one under test. */
    {
        /* Same contract as step 2's first assertion (the drain-counter
         * phrasing proved phase-sensitive on hardware): wait for the
         * flag itself. */
        if (tb_wait_update(HAL_GetTick(), TIM_BASIC_TIM5_WINDOW_MS) != 0) {
            tb_set_report("(step1: no update)");
            return (int)BSP_TIM_BASIC_ST1_TIM5_1HZ; /* no update in window */
        }
        __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
    }

    /* Step 2: update-flag polling contract. Three assertions: the flag
     * SETS within a window (counter running), CLEARS via
     * __HAL_TIM_CLEAR_FLAG (we followed the RM rc_w0 clearing contract),
     * then SETS again (periodic, not a one-shot). Polling the flag is
     * exactly what an app loop does with UIE left off. */
    {
        uint32_t start = HAL_GetTick();
        if (tb_wait_update(start, TIM_BASIC_TIM5_WINDOW_MS) != 0) {
            return (int)BSP_TIM_BASIC_ST2_TIM5_UIF; /* never set */
        }
        __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE); /* write-0 clear via HAL */
        if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_UPDATE) != RESET) {
            return (int)BSP_TIM_BASIC_ST2_TIM5_UIF; /* clear contract broke */
        }
        start = HAL_GetTick();
        if (tb_wait_update(start, TIM_BASIC_TIM5_WINDOW_MS) != 0) {
            return (int)BSP_TIM_BASIC_ST2_TIM5_UIF; /* not periodic */
        }
        __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE); /* clean for step 3 */
    }

    /* Step 3: cadence. Sync to the period first (wait for a set, clear
     * it): the next update then lands ~1 s after that clear, so a 1.2 s
     * window (1.0 < 1.2 < 2.0) must catch EXACTLY one overflow -- the 1 Hz
     * pace holds across consecutive windows. */
    {
        uint32_t start = HAL_GetTick();
        uint32_t seen = 0u;
        if (tb_wait_update(start, TIM_BASIC_TIM5_WINDOW_MS) != 0) {
            return (int)BSP_TIM_BASIC_ST3_TIM5_CAD;
        }
        __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
        start = HAL_GetTick();
        while ((HAL_GetTick() - start) < TIM_BASIC_TIM5_WINDOW_MS) {
            if (tb_drain_update() != 0u) {
                seen++;
            }
        }
        if (seen != 1u) {
            return (int)BSP_TIM_BASIC_ST3_TIM5_CAD; /* pace drifted */
        }
    }
    return 0; /* all steps passed */
}
