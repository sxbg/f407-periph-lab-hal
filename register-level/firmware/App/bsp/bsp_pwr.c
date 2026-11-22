/**
 ******************************************************************************
 * @file    bsp_pwr.c
 * @brief   PWR low-power modes -- teaching module: Sleep (tested), Stop
 *          (demo command, KEY1/EXTI0 wake), Standby (API only), with the
 *          register discipline of SLEEPDEEP/PDDS/LPDS spelled out.
 ******************************************************************************
 * @addtogroup BSP_PWR
 * @{
 *
 * == CUBEMX CONFIG (absent from f407-periph-lab.ioc -- by design) ==========
 * | Field             | Value (module truth)                 | Why
 * |-------------------|--------------------------------------|-------------------
 * | PWR in IOC        | NOT PRESENT                          | knowledge #13:
 * |                   |                                      | adapter-owned
 * |                   |                                      | peripherals stay
 * |                   |                                      | out of the .ioc
 * | PA0 (KEY1)        | GPIO_Input + pull-down (IOC, bsp_    | pressed = high:
 * |                   | gpio owns the PIN); EXTI0 wired in   | rising edge =
 * |                   | THIS module only when entering Stop  | press (walkthrough)
 * | NVIC              | EXTI0_IRQn (6) enabled only inside   | wake needs the
 * |                   | bsp_pwr_enter_stop                   | vector on (RM
 * |                   |                                      | Table: Stop exit)
 * == REGISTER WALKTHROUGH (RM0090 ch.5 PWR; PM0056 SCB/EXTI) ===============
 * The three low-power modes are ONE Cortex-M instruction (WFI) plus the
 * arrangement of three control bits -- that is the whole teaching point:
 *   Sleep  : SLEEPDEEP=0. Clock gates off the CPU clock only; every SRAM/
 *            register lives, ANY enabled NVIC interrupt wakes (SysTick's
 *            1 ms tick = our alarm clock), execution resumes right after
 *            the WFI. Wake latency ~ a few CPU cycles.
 *   Stop   : SLEEPDEEP=1 AND PWR->CR PDDS=0. 1.2 V domain KEPT powered
 *            (LPDS=1 selects the low-power regulator for it) -- SRAM and
 *            registers survive -- but HCLK/PCLK stop and HSE/HSI/PLL are
 *            shut down. Wake: an EXTI line in INTERRUPT mode (IMR set)
 *            whose NVIC vector is enabled -- Table "Stop mode entry and
 *            exit" is explicit about the NVIC requirement. On exit the
 *            hardware hands SYSCLK to HSI 16 MHz WITH THE PLL OFF. Any
 *            peripheral left running now runs at 16/168 = 9.5x slow:
 *            the EP17 lesson (UART at a stale BRR spews ~10.9 kBd
 *            garbage). THE FIX THIS FIRMWARE CHOSES: re-run SystemInit()
 *            immediately on wake -- it redoes flash latency + bus
 *            prescalers + HSE/PLL lock + SW SYSCLK switch, all idempotent.
 *   Standby: SLEEPDEEP=1 AND PWR->CR PDDS=1 (plus CWUF=1 to clear the
 *            wake flag and a clean WUF, else the entry is ignored). The
 *            REGULATOR POWERS DOWN: SRAM/registers lost, wake = reset-
 *            class boot (SP/PC from the reset vector). Wake sources:
 *            WKUP1 rising (PA0 here = KEY1; PWR->CSR EWUP bit8 arms it),
 *            RTC alarm/wake/tamper/timestamp, NRST, IWDG reset. Evidence
 *            after the reboot: PWR->CSR SBF (readable via
 *            bsp_pwr_standby_flag, cleared by CR CSBF=1); a FAILED low-
 *            power entry instead leaves LPWRRSTF set in RCC->CSR (see
 *            bsp_wdg for the reset-source decode).
 * Entry discipline (RM "Entering low-power mode"): before any deepsleep,
 *   clear what would fire instantly -- EXTI->PR pending bits (a set bit
 *   makes the Stop entry IGNORED, RM Table note), PWR->CSR WUF (CWUF
 *   write). This module does both in bsp_pwr_enter_stop.
 * EXTI line 0 = PA0 wiring (the only cross-peripheral wiring in here):
 *   SYSCFG->EXTICR[0] bits [3:0] = 0000 selects PA as the line-0 source
 *   (needs RCC->APB2ENR SYSCFGEN bit14); EXTI->RTSR bit0 = rising trigger
 *   (KEY1 idles LOW via its pull-down -- bsp_gpio -- so a press IS the
 *   rising edge; PWR_CSR EWUP is NOT involved in Stop, it is the Standby
 *   wake path); EXTI->IMR bit0 unmasks the line; NVIC EXTI0_IRQn=6 enables
 *   the vector; EXTI->PR bit0 is rc_w1 (write 1 to clear) and the handler
 *   below does exactly that.
 * Selftest note (honest measurement caveat): the 100 ms Sleep window is
 *   timed BY SysTick and WOKEN BY SysTick -- the same oscillator serves
 *   both roles, so the test proves the WFI/wake mechanics, not the
 *   oscillator's absolute accuracy. A fully independent cross-check would
 *   use the TIM6 1 kHz flag (see bsp_tim_basic) as the timekeeper.
 * == HOW TO VERIFY =========================================================
 * menu> pwr
 *   PASS pwr (sleep 10X ms via SysTick wake)  -- X ~ 0..3
 * Ammeter/scope on the board: entering Stop ('pwr_stop_demo') drops the
 * core current measurably (LPDS regulator + PLL/HSI/HSE off); pressing
 * KEY1 resumes the menu. Watch PA0 on the scope: menu heartbeat pauses
 * while asleep, EXTI0 counts once per wake (bsp_pwr_exti0_wake_count).
 */
#include "bsp_pwr.h"

#include "hal_port.h" /* family seam: pulls in the CMSIS device header */

/* ---- module constants ----------------------------------------------------- */

/* Self-test step numbers (the value bsp_pwr_selftest returns). */
#define BSP_PWR_ST1_PRESTATE 1u /* gates set, deepsleep bits resting */
#define BSP_PWR_ST2_SLEEP    2u /* 100 ms Sleep window via SysTick   */

/* Sleep window and slack for the selftest (spec #5: bounded waits).
 * Each WFI is woken by the next 1 ms SysTick interrupt; the 30 ms slack
 * covers ISR/poll jitter, not physics. */
#define BSP_PWR_SLEEP_MS    100u
#define BSP_PWR_SLEEP_SLACK 30u
#define BSP_PWR_SYSTICK_GUARD 2000000u

/* ---- private state ---------------------------------------------------------- */

static char s_report[40];
static int s_report_valid = 0;

/* EXTI0 wake counter (ISR-side volatile, mainline read). */
static volatile uint32_t s_exti0_wakes;

static void report_set(const char *msg)
{
    unsigned i = 0u;
    while ((msg[i] != '\0') && (i < (sizeof(s_report) - 1u))) {
        s_report[i] = msg[i];
        i++;
    }
    s_report[i] = '\0';
    s_report_valid = 1;
}

/* ---- private helpers -------------------------------------------------------- */

/* SysTick COUNTFLAG-based wrap accounting lives inside bsp_pwr_sleep_ms
 * (the only place that counts wraps); the guards below bound it. */

/* ---- ISRs (vector names MUST match startup_stm32f407xx.s exactly) ---------- */

/* EXTI line 0 = PA0 = KEY1 press. The Stop-mode wake needs this vector
 * enabled (RM Table: Stop exit), and the pending bit is rc_w1: writing 1
 * clears it. Short by contract: one read-test-clear, one volatile bump. */
void EXTI0_IRQHandler(void)
{
    if ((EXTI->PR & EXTI_PR_PR0) != 0u) {
        EXTI->PR = EXTI_PR_PR0; /* rc_w1 clear (walkthrough) */
        s_exti0_wakes++;
    }
}

/* ---- public API ------------------------------------------------------------ */

int bsp_pwr_init(void)
{
    /* Bus-clock gates: PWR on APB1 (PWREN bit28), SYSCFG on APB2
     * (SYSCFGEN bit14 -- needed later for EXTICR). HAL equivalent:
     * __HAL_RCC_PWR_CLK_ENABLE + HAL_PWR functions on the same bits. */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB1ENR;
    (void)RCC->APB2ENR;

    /* Read-back: deepsleep decision bits all resting (LPDS=1 would mean
     * "next deepsleep uses the low-power regulator"; PDDS=1 would mean
     * "next deepsleep is Standby" -- neither may be armed accidentally). */
    if (((RCC->APB1ENR & RCC_APB1ENR_PWREN) == 0u) ||
        ((RCC->APB2ENR & RCC_APB2ENR_SYSCFGEN) == 0u) ||
        ((PWR->CR & (PWR_CR_LPDS | PWR_CR_PDDS)) != 0u) ||
        ((SCB->SCR & SCB_SCR_SLEEPDEEP_Msk) != 0u)) {
        return -1;
    }
    return 0;
}

uint32_t bsp_pwr_sleep_ms(uint32_t ms)
{
    uint32_t seen = 0u;
    uint32_t ctrl0;
    int restore_tickint = 0;

    /* The wake source is the SysTick interrupt. HAL normally runs with
     * TICKINT on (1 ms tick -> HAL_IncTick); if some other config turned
     * it off, arm it here and restore the original CTRL afterwards --
     * WFI with zero enabled interrupts would sleep forever (bounded by
     * nothing: that is the hang this pre-emptive arm prevents). */
    ctrl0 = SysTick->CTRL;
    if ((ctrl0 & SysTick_CTRL_TICKINT_Msk) == 0u) {
        SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
        restore_tickint = 1;
    }

    /* Sleep-mode decision bits: SLEEPDEEP CLEAR (walkthrough table) --
     * this is the ONLY delta from Stop/Standby besides PWR->CR. */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    /* WFI loop: each iteration sleeps until the next SysTick interrupt
     * (~1 ms), then one COUNTFLAG wrap is counted (the independent ms
     * measure; same pattern as every P3 module's wait helper). */
    while (seen < ms) {
        uint32_t guard = BSP_PWR_SYSTICK_GUARD;
        __WFI(); /* PM0056 B3.2: sleep until an enabled interrupt */
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0u) {
            if (--guard == 0u) {
                seen = BSP_PWR_SLEEP_DEAD_SYSTICK;
                break;
            }
        }
        if (seen == BSP_PWR_SLEEP_DEAD_SYSTICK) {
            break;
        }
        seen++;
    }

    if (restore_tickint != 0) {
        SysTick->CTRL = ctrl0; /* leave TICKINT exactly as we found it */
    }
    return seen;
}

void bsp_pwr_enter_stop(void)
{
    /* Gates: PWR (APB1), SYSCFG (APB2), GPIOA (AHB1 -- PA0's port; boot
     * already turned it on, kept explicit so the function is standalone). */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    (void)RCC->APB1ENR;
    (void)RCC->APB2ENR;
    (void)RCC->AHB1ENR;

    /* EXTI line 0 <- PA0 (walkthrough): route (EXTICR), trigger (rising:
     * KEY1 idles low via its pull-down, a press IS the edge), unmask,
     * NVIC vector (RM requires it for a Stop-mode WFI exit). PA0's PIN
     * config (input+pull-down) belongs to bsp_gpio and survives Stop. */
    SYSCFG->EXTICR[0] &= ~0xFu;          /* EXTI0 source = PA0 (0000)      */
    EXTI->FTSR &= ~EXTI_FTSR_TR0;        /* rising only: release must not  */
    EXTI->RTSR |= EXTI_RTSR_TR0;         /* wake -- only the press edge    */
    EXTI->IMR |= EXTI_IMR_MR0;           /* interrupt mode (RM Table)      */
    EXTI->PR = EXTI_PR_PR0;              /* rc_w1: kill stale pending -- a
                                          * set PR bit makes the Stop entry
                                          * get IGNORED (RM Table note)    */
    NVIC_SetPriority(EXTI0_IRQn, 10u);   /* below SysTick/fault priorities */
    NVIC_EnableIRQ(EXTI0_IRQn);

    /* Stop-mode decision bits (walkthrough): LPDS=1 selects the low-power
     * regulator for the 1.2 V domain, PDDS=0 says "Stop, NOT Standby",
     * CWUF=1 clears a stale wake flag. FMT note: this OVERWRITES nothing
     * else -- CR bits PVDE/PLS etc. are untouched by the masked form. */
    PWR->CR = (PWR->CR & ~PWR_CR_PDDS) | PWR_CR_LPDS | PWR_CR_CWUF;

    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; /* deepsleep requested            */
    __WFI();                            /* enters Stop; HCLK/PLL/HSE stop */
    /* ---- execution resumes HERE on the KEY1 rising edge -----------------
     * Hardware state now (RM "Exiting Stop mode"): SYSCLK = HSI 16 MHz,
     * PLL OFF. Running the old 168 MHz BRR here would print ~10.9 kBd
     * garbage (EP17). Fix: SLEEPDEEP down FIRST (so a follow-up WFI would
     * be plain Sleep), then SystemInit() re-runs the whole clock tree:
     * flash latency, bus prescalers, HSE ready + PLL lock + SW switch --
     * idempotent, ~1 ms, and the only code touched before any UART/RTT
     * traffic resumes (inside this function). */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    SystemInit(); /* CMSIS: back to HSE 25 MHz -> PLL -> 168 MHz      */
}

void bsp_pwr_arm_standby(void)
{
    /* Decision bits for Standby (walkthrough): PDDS=1, plus CWUF=1 --
     * RM Table demands WUF=0 at entry, and a set WUF makes the Standby
     * entry get IGNORED just like the Stop case. NOT armed anywhere in
     * the firmware: entering Standby loses SRAM and the wake is a reset. */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;
    PWR->CR = (PWR->CR & ~PWR_CR_LPDS) | PWR_CR_PDDS | PWR_CR_CWUF;
    /* The caller pairs this with SCB->SCR SLEEPDEEP=1 + WFI (or simply
     * documents the option bytes' nRST_STDBY behavior) -- deliberately
     * not done HERE: see the safety contract in the header. */
}

int bsp_pwr_standby_flag(void)
{
    return (PWR->CSR & PWR_CSR_SBF) ? 1 : 0;
}

uint32_t bsp_pwr_exti0_wake_count(void)
{
    return s_exti0_wakes;
}

const char *bsp_pwr_last_report(void)
{
    return (s_report_valid != 0) ? s_report : "";
}

int bsp_pwr_selftest(void)
{
    /* Step 1: gates + resting-state read-back (header contract). */
    if (bsp_pwr_init() != 0) {
        return (int)BSP_PWR_ST1_PRESTATE;
    }

    /* Step 2: Sleep 100 ms, woken by SysTick's 1 ms interrupt each time.
     * Elapsed must land in [100 .. 130] ms: shorter is impossible (the
     * loop counts SysTick wraps), longer means the wake path stalled --
     * both are failures. The measurement caveat (same oscillator times
     * and wakes) is in the walkthrough; this proves the MECHANICS. */
    {
        const uint32_t slept = bsp_pwr_sleep_ms(BSP_PWR_SLEEP_MS);
        if ((slept < BSP_PWR_SLEEP_MS) ||
            (slept > (BSP_PWR_SLEEP_MS + BSP_PWR_SLEEP_SLACK))) {
            return (int)BSP_PWR_ST2_SLEEP;
        }
        /* SLEEPDEEP must be back to clear after Sleep (Sleep never set
         * it, but the read-back proves the SCR was left honest). */
        if ((SCB->SCR & SCB_SCR_SLEEPDEEP_Msk) != 0u) {
            return (int)BSP_PWR_ST2_SLEEP;
        }
        /* Tiny local decimal formatter (no libc printf in BSP). */
        {
            char msg[40];
            char digits[10];
            unsigned n = 0u, k = 0u;
            unsigned v = (unsigned)slept;
            msg[k++] = '(';
            msg[k++] = 's'; msg[k++] = 'l'; msg[k++] = 'e'; msg[k++] = 'e';
            msg[k++] = 'p'; msg[k++] = ' ';
            do {
                digits[n++] = (char)('0' + (v % 10u));
                v /= 10u;
            } while (v > 0u);
            while (n > 0u) {
                msg[k++] = digits[--n];
            }
            msg[k++] = ' ';
            msg[k++] = 'm'; msg[k++] = 's'; msg[k++] = ')';
            msg[k] = '\0';
            report_set(msg);
        }
    }
    return 0; /* all steps passed */
}

/** @} */
