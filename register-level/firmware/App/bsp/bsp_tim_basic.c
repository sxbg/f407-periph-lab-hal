/**
 ******************************************************************************
 * @file    bsp_tim_basic.c
 * @brief   TIM5/TIM6/TIM7 pure timebases -- teaching module: one interrupt
 *          counter (TIM5), two polling-flag timers (TIM6/TIM7).
 ******************************************************************************
 * @addtogroup BSP_TIM_BASIC
 * @{
 *
 * == CUBEMX CONFIG (from f407-periph-lab.ioc) ==============================
 * | Field            | Value (IOC truth)                    | Why
 * |------------------|--------------------------------------|-------------------
 * | TIM5             | in IOC: base init only (PSC=0,       | pin-free timebase
 * |                  | ARR=max, CEN=0, NVIC off)            | this module arms
 * | TIM6 / TIM7      | NOT in the IOC at all                | module = full
 * |                  |                                      | owner: RCC gate,
 * |                  |                                      | PSC/ARR, NVIC, ISRs
 * | Kernel clock     | all three on APB1 x2 = 84 MHz        | RM0090 ch.6 rule
 * == REGISTER WALKTHROUGH (TIM5: ch.18, TIM6/7: ch.20 "Basic timers") ======
 * RCC->APB1ENR (ch.6.3.13): TIM5EN bit3, TIM6EN bit4, TIM7EN bit5 --
 *   kernel clock gates. Clocks first; dummy read after the OR.
 * Two-step divider (same law as PWM, here without output compare):
 *   f_CK_CNT = 84 MHz / (PSC + 1)   -- prescaler ticks
 *   f_UPDATE = f_CK_CNT / (ARR + 1) -- one update event per overflow
 *   TIM5: 84M/8400 = 10 kHz tick; 10 kHz/10000 = 1 Hz  -> PSC=8399 ARR=9999
 *   TIM6: 84M/84   = 1 MHz  tick; 1 MHz /1000  = 1 kHz -> PSC=83   ARR=999
 *   TIM7: 84M/8400 = 10 kHz tick; 10 kHz/100  = 100 Hz -> PSC=8399 ARR=99
 * CR1 (ch.18.4.1/ch.20.4.1):
 *   CEN   bit0  counter enable -- written LAST, after the IRQ chain is armed
 *   UDIS  bit1  update DISABLE: 0 = update events flow (we need them)
 *   URS   bit2  update request SOURCE: 0 = every source (overflow AND the
 *               EGR.UG we fire in init) raises UIF; 1 = overflow only. Kept
 *               0 so the init-time UG behaves like a real overflow.
 *   ARPE  bit7  ARR preload: 0 (static ARR, no shadow games needed)
 * EGR.UG (ch.18.4.5/ch.20.4.5): re-initializes the counter and reloads the
 *   PSC/ARR shadows immediately; side effect: sets SR.UIF (cleared below).
 * SR.UIF bit0 (ch.18.4.4/ch.20.4.4): "update flag", raised on overflow (and
 *   UG). SR bits are rc_w0: software clears by WRITING 0; writing 1s to the
 *   other bits is ignored -- hence "SR = ~UIF", not a read-modify-write.
 * DIER.UIE bit0 (ch.18.4.3/ch.20.4.3): update interrupt enable -- routes
 *   UIF to the NVIC line. TIM5 sets it; TIM6/TIM7 keep it CLEAR (polling
 *   lesson: the flag exists whether or not anyone is interrupted).
 * NVIC (PM0056/B3): TIM5_IRQn=50 -> ISER[1] bit18, IPR byte 50. CMSIS
 *   NVIC_SetPriority/NVIC_EnableIRQ are the portable seam over those regs.
 * CNT vs PSC cascade (the teaching point): the prescaler is NOT part of the
 *   counter -- CNT itself only ever counts 0..ARR; PSC divides the clock
 *   FEEDING it. A 16-bit timer therefore reaches 1 Hz not by counting to
 *   84M (impossible in 16 bits) but by PSC first shaving 84 MHz down to
 *   10 kHz, then ARR picking 10000 of those ticks.
 * == HOW TO VERIFY =========================================================
 * menu> tim_basic
 *   PASS tim_basic       (steps: 1 TIM5 IRQ window, 2 TIM6 UIF, 3 TIM7 UIF)
 *   step1 takes ~1.2 s (waits one full TIM5 period by design).
 * Scope/debugger checks: the counter via bsp_tim_basic_uptime_seconds()
 *   advances 1/s; TIM6->SR.UIF toggles 1000x/s when watched live; run
 *   'reboot' and the boot diag time reference is unchanged (TIM5 IRQ adds
 *   <1% CPU load: 1 ISR/s).
 * ISR contract (framework rule): ISRs here are short -- read SR, test OUR
 *   enabled flag, clear it, bump a volatile counter. No loops, no calls
 *   into other modules, no printing.
 */
#include "bsp_tim_basic.h"

#include "hal_port.h" /* family seam: pulls in the CMSIS device header */

/* ---- module constants ----------------------------------------------------- */

/* Timebase derivations (see walkthrough formulas). */
#define TIM_BASIC_TIM5_PSC 8399u /* 84 MHz/8400  = 10 kHz */
#define TIM_BASIC_TIM5_ARR 9999u /* 10 kHz/10000 = 1 Hz   */
#define TIM_BASIC_TIM6_PSC 83u   /* 84 MHz/84    = 1 MHz  */
#define TIM_BASIC_TIM6_ARR 999u  /* 1 MHz/1000   = 1 kHz  */
#define TIM_BASIC_TIM7_PSC 8399u /* 84 MHz/8400  = 10 kHz */
#define TIM_BASIC_TIM7_ARR 99u   /* 10 kHz/100   = 100 Hz */

/* Self-test step numbers (the value bsp_tim_basic_selftest returns). */
#define BSP_TIM_BASIC_ST1_TIM5_IRQ 1u /* 1 s counter advances in 1.2 s */
#define BSP_TIM_BASIC_ST2_TIM6_UIF 2u /* TIM6 UIF poll set/clear/set   */
#define BSP_TIM_BASIC_ST3_TIM7_UIF 3u /* TIM7 UIF poll set/clear/set   */

/* Windows and caps for the selftest waits (spec #5: bounded waits). */
#define TIM_BASIC_TIM5_WINDOW_MS 1200u /* > 1 TIM5 period with margin */
#define TIM_BASIC_TIM6_WINDOW_MS 200u  /* >> 1 ms TIM6 period         */
#define TIM_BASIC_TIM7_WINDOW_MS 300u  /* >> 10 ms TIM7 period        */
#define TIM_BASIC_SYSTICK_GUARD  2000000u /* if COUNTFLAG never sets:
                                           * SysTick is dead -> bounded exit */

/* ---- ISR state (volatile: written by ISRs, read by mainline) --------------- */

static volatile uint32_t s_tim5_seconds; /* TIM5 update IRQ counter   */
static volatile uint32_t s_tim6_updates; /* dormant until UIE is set  */
static volatile uint32_t s_tim7_updates; /* dormant until UIE is set  */

/* ---- private helpers -------------------------------------------------------- */

/* Count SysTick wraps for up to 'ms' (1 wrap = 1 ms, PM0056/B3.3
 * COUNTFLAG semantics -- see bsp_gpio for the derivation). Returns the
 * number of wraps actually seen (caller uses it as elapsed ms), or
 * 0xFFFFFFFF when SysTick looks dead. */
#define TIM_BASIC_DEAD_SYSTICK 0xFFFFFFFFu

static uint32_t tim_basic_count_ms(uint32_t ms)
{
    uint32_t seen = 0u;
    while (seen < ms) {
        uint32_t guard = TIM_BASIC_SYSTICK_GUARD;
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0u) {
            if (--guard == 0u) {
                return TIM_BASIC_DEAD_SYSTICK;
            }
        }
        seen++;
    }
    return seen;
}

/* ---- ISRs (vector names MUST match startup_stm32f407xx.s exactly) ---------- */

/* TIM5 update interrupt: the live 1 Hz heartbeat. Check-then-clear keeps
 * the handler honest if it ever shares the line with another source. */
void TIM5_IRQHandler(void)
{
    if ((TIM5->SR & TIM_SR_UIF) != 0u) {
        TIM5->SR = ~TIM_SR_UIF; /* rc_w0 clear (walkthrough) */
        s_tim5_seconds++;       /* short: one volatile increment, no calls */
    }
}

/* TIM6 update interrupt: DEFINED here (vector-table ownership) but dormant
 * in the polling lesson -- TIM6->DIER.UIE stays clear, so this never runs.
 * The live alternative: TIM6->DIER |= TIM_DIER_UIE (plus NVIC_EnableIRQ
 * with TIM6_DAC_IRQn=54) and the same check-clear-bump body goes live.
 * Name note: the TIM6 vector is shared with the DAC underrun error
 * (hence "TIM6_DAC"); DAC has no other reason to fire here. */
void TIM6_DAC_IRQHandler(void)
{
    if ((TIM6->SR & TIM_SR_UIF) != 0u) {
        TIM6->SR = ~TIM_SR_UIF;
        s_tim6_updates++;
    }
}

/* TIM7 update interrupt: dormant like TIM6's (TIM7_IRQn=55 to enable). */
void TIM7_IRQHandler(void)
{
    if ((TIM7->SR & TIM_SR_UIF) != 0u) {
        TIM7->SR = ~TIM_SR_UIF;
        s_tim7_updates++;
    }
}

/* ---- public API ------------------------------------------------------------ */

int bsp_tim_basic_init(void)
{
    /* Kernel clocks: APB1ENR TIM5EN bit3, TIM6EN bit4, TIM7EN bit5
     * (RM0090 ch.6.3.13). TIM6/7 gates were never set by anyone (not in
     * the IOC) -- this is the only code in the firmware turning them on. */
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN | RCC_APB1ENR_TIM6EN |
                    RCC_APB1ENR_TIM7EN;
    (void)RCC->APB1ENR; /* read-back stalls until the clocks are live */

    /* --- TIM5: 1 Hz update interrupt ------------------------------------- */
    /* Stop first (CEN=0) so re-init never races a running counter. */
    TIM5->CR1 = 0u;
    TIM5->PSC = TIM_BASIC_TIM5_PSC;
    TIM5->ARR = TIM_BASIC_TIM5_ARR;
    TIM5->EGR = TIM_EGR_UG; /* latch shadows, CNT=0 (also sets UIF) */
    TIM5->SR = ~TIM_SR_UIF; /* clear the UG-induced flag BEFORE arming */
    /* NVIC first, then DIER.UIE, then CEN: if the chain armed in the other
     * order a stale UIF could fire an ISR against a half-ready handler.
     * HAL equivalent: HAL_TIM_Base_Init + HAL_NVIC_SetPriority/
     * EnableIRQ(TIM5_IRQn) + HAL_TIM_Base_Start_IT. */
    NVIC_SetPriority(TIM5_IRQn, 8u); /* low urgency, below bus faults */
    NVIC_EnableIRQ(TIM5_IRQn);
    TIM5->DIER = TIM_DIER_UIE; /* route UIF to the NVIC line */
    TIM5->CR1 = TIM_CR1_CEN;   /* URS=0, UDIS=0, ARPE=0: plain up-counter */

    /* --- TIM6: 1 kHz polling timebase ------------------------------------- */
    TIM6->CR1 = 0u;
    TIM6->PSC = TIM_BASIC_TIM6_PSC;
    TIM6->ARR = TIM_BASIC_TIM6_ARR;
    TIM6->EGR = TIM_EGR_UG;
    TIM6->SR = ~TIM_SR_UIF;
    TIM6->DIER = 0u;         /* polling lesson: UIE deliberately CLEAR --
                              * the ISR above stays dormant (walkthrough) */
    TIM6->CR1 = TIM_CR1_CEN; /* start counting; UIF raises every 1 ms */

    /* --- TIM7: 100 Hz polling timebase ------------------------------------ */
    TIM7->CR1 = 0u;
    TIM7->PSC = TIM_BASIC_TIM7_PSC;
    TIM7->ARR = TIM_BASIC_TIM7_ARR;
    TIM7->EGR = TIM_EGR_UG;
    TIM7->SR = ~TIM_SR_UIF;
    TIM7->DIER = 0u;
    TIM7->CR1 = TIM_CR1_CEN; /* UIF every 10 ms */

    /* Read-back proof: timebases armed, interrupt routing as intended. */
    if ((TIM5->PSC != TIM_BASIC_TIM5_PSC) ||
        (TIM5->ARR != TIM_BASIC_TIM5_ARR) ||
        (TIM6->PSC != TIM_BASIC_TIM6_PSC) ||
        (TIM6->ARR != TIM_BASIC_TIM6_ARR) ||
        (TIM7->PSC != TIM_BASIC_TIM7_PSC) ||
        (TIM7->ARR != TIM_BASIC_TIM7_ARR)) {
        return -1;
    }
    if (((TIM5->DIER & TIM_DIER_UIE) == 0u) ||
        ((TIM6->DIER & TIM_DIER_UIE) != 0u) ||
        ((TIM7->DIER & TIM_DIER_UIE) != 0u) ||
        ((TIM5->CR1 & TIM_CR1_CEN) == 0u) ||
        ((TIM6->CR1 & TIM_CR1_CEN) == 0u) ||
        ((TIM7->CR1 & TIM_CR1_CEN) == 0u)) {
        return -1;
    }
    return 0;
}

uint32_t bsp_tim_basic_uptime_seconds(void)
{
    return s_tim5_seconds;
}

int bsp_tim_basic_selftest(void)
{
    /* Fresh init first (idempotent, menu-driven selftest pattern). */
    if (bsp_tim_basic_init() != 0) {
        return (int)BSP_TIM_BASIC_ST1_TIM5_IRQ;
    }

    /* Step 1: TIM5 interrupt counter must advance >= 1 in a 1.2 s window
     * (one 1 s period + margin). The window is measured with the SysTick
     * wrap counter, NOT by trusting TIM5 itself -- an independent clock
     * cross-checks the one under test. */
    {
        const uint32_t t0 = s_tim5_seconds;
        if (tim_basic_count_ms(TIM_BASIC_TIM5_WINDOW_MS) ==
            TIM_BASIC_DEAD_SYSTICK) {
            return (int)BSP_TIM_BASIC_ST1_TIM5_IRQ;
        }
        if (s_tim5_seconds == t0) {
            return (int)BSP_TIM_BASIC_ST1_TIM5_IRQ; /* no IRQ in 1.2 s */
        }
    }

    /* Step 2: TIM6 UIF polling contract. Three assertions: the flag SETS
     * within a window (counter running), CLEARS on the rc_w0 write (we
     * followed the RM clearing contract), then SETS again (periodic, not
     * a one-shot). Polling the flag is exactly what an app loop does with
     * UIE left off. */
    {
        uint32_t guard = TIM_BASIC_TIM6_WINDOW_MS;
        while ((TIM6->SR & TIM_SR_UIF) == 0u) {
            if (tim_basic_count_ms(1u) == TIM_BASIC_DEAD_SYSTICK) {
                return (int)BSP_TIM_BASIC_ST2_TIM6_UIF;
            }
            if (--guard == 0u) {
                return (int)BSP_TIM_BASIC_ST2_TIM6_UIF; /* never set */
            }
        }
        TIM6->SR = ~TIM_SR_UIF; /* write-0 clear (walkthrough) */
        if ((TIM6->SR & TIM_SR_UIF) != 0u) {
            return (int)BSP_TIM_BASIC_ST2_TIM6_UIF; /* clear contract broke */
        }
        guard = TIM_BASIC_TIM6_WINDOW_MS;
        while ((TIM6->SR & TIM_SR_UIF) == 0u) {
            if (tim_basic_count_ms(1u) == TIM_BASIC_DEAD_SYSTICK) {
                return (int)BSP_TIM_BASIC_ST2_TIM6_UIF;
            }
            if (--guard == 0u) {
                return (int)BSP_TIM_BASIC_ST2_TIM6_UIF; /* not periodic */
            }
        }
        TIM6->SR = ~TIM_SR_UIF; /* leave the flag clean for the next run */
    }

    /* Step 3: TIM7 same contract on its 10 ms period. */
    {
        uint32_t guard = TIM_BASIC_TIM7_WINDOW_MS;
        while ((TIM7->SR & TIM_SR_UIF) == 0u) {
            if (tim_basic_count_ms(1u) == TIM_BASIC_DEAD_SYSTICK) {
                return (int)BSP_TIM_BASIC_ST3_TIM7_UIF;
            }
            if (--guard == 0u) {
                return (int)BSP_TIM_BASIC_ST3_TIM7_UIF;
            }
        }
        TIM7->SR = ~TIM_SR_UIF;
        if ((TIM7->SR & TIM_SR_UIF) != 0u) {
            return (int)BSP_TIM_BASIC_ST3_TIM7_UIF;
        }
        guard = TIM_BASIC_TIM7_WINDOW_MS;
        while ((TIM7->SR & TIM_SR_UIF) == 0u) {
            if (tim_basic_count_ms(1u) == TIM_BASIC_DEAD_SYSTICK) {
                return (int)BSP_TIM_BASIC_ST3_TIM7_UIF;
            }
            if (--guard == 0u) {
                return (int)BSP_TIM_BASIC_ST3_TIM7_UIF;
            }
        }
        TIM7->SR = ~TIM_SR_UIF;
    }
    return 0; /* all steps passed */
}

/** @} */
