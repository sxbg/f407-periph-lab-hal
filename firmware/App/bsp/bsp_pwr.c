/**
 ******************************************************************************
 * @file    bsp_pwr.c
 * @brief   PWR low-power modes -- teaching module (HAL edition of
 *          projects/f407-periph-lab bsp_pwr): Sleep (tested), Stop (demo
 *          command, KEY1/EXTI0 wake), Standby (API only), with the HAL
 *          entry discipline and the SLEEPDEEP/PDDS/LPDS semantics spelled
 *          out.
 ******************************************************************************
 * @addtogroup BSP_PWR
 * @{
 *
 * == HAL EDITION MAPPING ===================================================
 * The teaching payload is unchanged: the three low-power modes are ONE
 * Cortex-M instruction (WFI) plus the arrangement of three control bits.
 * What changed is WHO writes those bits: the HAL_PWR_* entries do it, and
 * this module (A3 rule) never touches a peripheral control register. What
 * each HAL call does under the hood (stm32f4xx_hal_pwr.c) is the mapping
 * the teaching now points at:
 *
 * | register edition                  | HAL edition (what HAL writes)       |
 * |-----------------------------------|-------------------------------------|
 * | SCB->SCR &= ~SLEEPDEEP; __WFI     | HAL_PWR_EnterSLEEPMode(MAINREG_ON,  |
 * |                                   |   PWR_SLEEPENTRY_WFI): clears       |
 * |                                   |   SLEEPDEEP, then WFI               |
 * | PWR->CR LPDS=1 PDDS=0 CWUF=1;     | HAL_PWR_EnterSTOPMode(LOWPOWERREG,  |
 * |   SCB->SCR SLEEPDEEP=1; __WFI;    |   PWR_STOPENTRY_WFI): sets LPDS +   |
 * |   SLEEPDEEP clear on wake         |   SLEEPDEEP, WFI, and clears        |
 * |                                   |   SLEEPDEEP again on exit           |
 * | PWR->CR PDDS=1 CWUF=1 (+ caller   | HAL_PWR_EnterSTANDBYMode(): PDDS=1, |
 * |   does SLEEPDEEP=1 + WFI)         |   WUF cleared, SLEEPDEEP=1, WFI --  |
 * |                                   |   arm and entry in ONE call         |
 *
 * == CUBEMX CONFIG (f407-periph-lab-hal.ioc) ===============================
 * | Field             | Value                                | Why
 * |-------------------|--------------------------------------|----------------
 * | PWR in IOC        | NOT PRESENT                          | knowledge #13:
 * |                   | (no CubeMX page for PWR)             | adapter-owned
 * |                   |                                      | peripherals stay
 * |                   |                                      | out of the .ioc
 * | PA0 (KEY1)        | GPIO_ModeDefaultEXTI = IT_RISING +   | pressed = high:
 * |                   | pull-down (label KEY1); NVIC EXTI0_  | rising edge =
 * |                   | IRQn enabled -- CubeMX owns the      | press; Stop exit
 * |                   | EXTI line-0 wiring AND the vector    | needs the vector
 * |                   |                                      | on (RM0090 Table)
 *
 * NOTE (integration state, to be closed by the integrator): the .ioc
 * carries the EXTI mode parameters but the NVIC EXTI0 enable + the
 * GPIO_EXTI0 signal still need the .ioc plan/apply + regeneration; until
 * then KEY1 is generated as a plain input and the wake vector is absent.
 * THIS MODULE deliberately does no EXTI wiring of its own -- that is the
 * HAL-edition ownership change.
 *
 * == MODE WALKTHROUGH (RM0090 ch.5 PWR; PM0056 SCB) ========================
 *   Sleep  : SLEEPDEEP=0. Clock gates off the CPU clock only; every SRAM/
 *            register lives, ANY enabled NVIC interrupt wakes (SysTick's
 *            1 ms HAL tick = our alarm clock), execution resumes right
 *            after the WFI inside HAL_PWR_EnterSLEEPMode. Wake latency ~
 *            a few CPU cycles.
 *   Stop   : SLEEPDEEP=1 AND PWR->CR PDDS=0 (HAL sets both via
 *            HAL_PWR_EnterSTOPMode; LPDS=1 selects the low-power
 *            regulator for the 1.2 V domain). SRAM and registers survive
 *            -- but HCLK/PCLK stop and HSE/HSI/PLL are shut down. Wake:
 *            an EXTI line in INTERRUPT mode whose NVIC vector is enabled
 *            -- the .ioc owns both halves now. On exit the hardware
 *            hands SYSCLK to HSI 16 MHz WITH THE PLL OFF. Any peripheral
 *            left running now runs at 16/168 = 9.5x slow: the EP17
 *            lesson (UART at a stale BRR spews ~10.9 kBd garbage). THE
 *            FIX (kept from the register edition): re-run SystemInit()
 *            immediately on wake -- it redoes flash latency + bus
 *            prescalers + HSE/PLL lock + SW SYSCLK switch, idempotent.
 *            SLEEPDEEP is already cleared by HAL before the function
 *            returns, so no SCB->SCR write is needed here (A3).
 *   Standby: SLEEPDEEP=1 AND PDDS=1. The REGULATOR POWERS DOWN: SRAM/
 *            registers lost, wake = reset-class boot. Wake sources:
 *            WKUP1 rising (PA0 here = KEY1), RTC alarm/wake/tamper/
 *            timestamp, NRST, IWDG reset. Evidence after the reboot:
 *            PWR->CSR SBF, read via __HAL_PWR_GET_FLAG(PWR_FLAG_SB)
 *            (bsp_pwr_standby_flag), cleared by __HAL_PWR_CLEAR_FLAG
 *            (CR CSBF=1); a FAILED low-power entry instead leaves
 *            LPWRRSTF set in RCC->CSR (see bsp_wdg for the decode).
 * Entry discipline (RM "Entering low-power mode"): before Stop, clear
 *   what would fire instantly -- a set EXTI->PR pending bit makes the
 *   Stop entry IGNORED (RM Table note). HAL idiom:
 *   __HAL_GPIO_EXTI_CLEAR_IT(KEY1_Pin) -- the same rc_w1 write CubeMX's
 *   own handler performs.
 * Callback routing (collision avoidance, see bsp_pwr.h): bsp_pwr does NOT
 *   override the weak HAL_GPIO_EXTI_Callback (bsp_exti does too). The
 *   integrator's single callback routes KEY1_Pin here -> bsp_pwr_on_exti0
 *   -> one volatile increment.
 * Selftest note (honest measurement caveat): the 100 ms Sleep window is
 *   timed BY SysTick and WOKEN BY SysTick -- the same oscillator serves
 *   both roles, so the test proves the WFI/wake mechanics, not the
 *   oscillator's absolute accuracy. A fully independent cross-check would
 *   use the TIM6 1 kHz flag (see bsp_tim_basic) as the timekeeper.
 * == HOW TO VERIFY =========================================================
 * menu> pwr
 *   PASS pwr (sleep 10X ms via SysTick wake)  -- X ~ 0..3
 * Ammeter/scope on the board: entering Stop ('pwr_stop_demo', re-added by
 * the integrator) drops the core current measurably (LPDS regulator +
 * PLL/HSI/HSE off); pressing KEY1 resumes the menu. Watch PA0 on the
 * scope: heartbeat pauses while asleep, the wake counter bumps once per
 * wake (bsp_pwr_exti0_wake_count).
 */
#include "bsp_pwr.h"

#include "main.h"          /* KEY1_Pin = PA0 label macros from CubeMX */
#include "stm32f4xx_hal.h"

/* ---- module constants ----------------------------------------------------- */

/* Self-test step numbers (the value bsp_pwr_selftest returns). */
#define BSP_PWR_ST1_PRESTATE 1u /* gates set, deepsleep bits resting */
#define BSP_PWR_ST2_SLEEP    2u /* 100 ms Sleep window via SysTick   */

/* Sleep window and slack for the selftest (spec #5: bounded waits).
 * Each HAL_PWR_EnterSLEEPMode WFI is woken by the next 1 ms SysTick
 * interrupt; the 30 ms slack covers ISR/poll jitter, not physics. */
#define BSP_PWR_SLEEP_MS    100u
#define BSP_PWR_SLEEP_SLACK 30u
#define BSP_PWR_SYSTICK_GUARD 2000000u

/* ---- private state ---------------------------------------------------------- */

static char s_report[40];
static int s_report_valid = 0;

/* EXTI0 wake counter (hook-side volatile, mainline read). */
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

/* ---- public API ------------------------------------------------------------ */

void bsp_pwr_on_exti0(void)
{
    /* Called from the integrator's SINGLE HAL_GPIO_EXTI_Callback when the
     * pending line is KEY1_Pin (PA0 / EXTI line 0). Short by contract:
     * one volatile bump -- the Stop-wake proof. */
    s_exti0_wakes++;
}

int bsp_pwr_init(void)
{
    /* Bus-clock gates, HAL form (the register edition's APB1ENR PWREN /
     * APB2ENR SYSCFGEN bit writes): idempotent macros. */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    /* Read-back: deepsleep decision bits all resting. Reads only (A3
     * read-only exemption): LPDS=1 would mean "next deepsleep uses the
     * low-power regulator"; PDDS=1 would mean "next deepsleep is
     * Standby" -- neither may be armed accidentally. */
    if (((PWR->CR & (PWR_CR_LPDS | PWR_CR_PDDS)) != 0u) || /* read-only (A3 exemption) */
        ((SCB->SCR & SCB_SCR_SLEEPDEEP_Msk) != 0u)) {      /* read-only (A3 exemption) */
        return -1;
    }
    return 0;
}

uint32_t bsp_pwr_sleep_ms(uint32_t ms)
{
    uint32_t seen = 0u;
    uint32_t ctrl0;
    int restore_tickint = 0;

    /* The wake source is the SysTick interrupt (HAL 1 ms tick ->
     * HAL_IncTick). If some other config turned TICKINT off, arm it via
     * HAL_ResumeTick() and restore via HAL_SuspendTick() afterwards --
     * WFI with zero enabled interrupts would sleep forever (bounded by
     * nothing: that is the hang this pre-emptive arm prevents). The CTRL
     * probe is a read (A3 read-only exemption); HAL owns the bit write. */
    ctrl0 = SysTick->CTRL; /* read-only (A3 exemption) */
    if ((ctrl0 & SysTick_CTRL_TICKINT_Msk) == 0u) {
        HAL_ResumeTick(); /* SET_BIT(SysTick->CTRL, TICKINT) under the hood */
        restore_tickint = 1;
    }

    /* Sleep-mode WFI loop: HAL clears SLEEPDEEP inside every call (the
     * ONLY mode decision, walkthrough), each entry sleeps until the next
     * SysTick interrupt (~1 ms), then one COUNTFLAG wrap is counted (the
     * independent ms measure; same pattern as every P3 module's wait
     * helper). */
    while (seen < ms) {
        uint32_t guard = BSP_PWR_SYSTICK_GUARD;
        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0u) { /* read-only (A3 exemption) */
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
        HAL_SuspendTick(); /* leave TICKINT exactly as we found it */
    }
    return seen;
}

void bsp_pwr_enter_stop(void)
{
    /* Gates: PWR (APB1), SYSCFG (APB2), GPIOA (PA0's port) -- HAL clock
     * macros, idempotent; kept explicit so the function is standalone. */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Kill a stale EXTI line-0 pending bit (a set PR makes the Stop
     * entry get IGNORED, RM Table note): HAL macro, the same rc_w1 write
     * CubeMX's handler does. The rest of the EXTI wiring (source = PA0,
     * rising trigger, unmask, NVIC vector) is owned by the .ioc now --
     * the HAL-edition ownership change, no register moves here. */
    __HAL_GPIO_EXTI_CLEAR_IT(KEY1_Pin);

    /* Stop entry: LPDS=1 (low-power regulator), PDDS=0 (Stop, NOT
     * Standby), SLEEPDEEP=1, WFI -- all inside the HAL call. Execution
     * resumes HERE on the KEY1 rising edge. Hardware state now (RM
     * "Exiting Stop mode"): SYSCLK = HSI 16 MHz, PLL OFF. Running the
     * old 168 MHz BRR here would print ~10.9 kBd garbage (EP17). */
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

    /* HAL already cleared SLEEPDEEP on exit (A3: no SCB->SCR write
     * here), so a follow-up WFI would be plain Sleep. Fix the clock
     * tree: SystemInit() re-runs flash latency, bus prescalers, HSE
     * ready + PLL lock + SW switch -- idempotent, ~1 ms, and the only
     * code touched before any log/UART traffic resumes (inside this
     * function). */
    SystemInit(); /* CMSIS: back to HSE 25 MHz -> PLL -> 168 MHz      */
}

void bsp_pwr_arm_standby(void)
{
    /* HAL semantics (mapping note in bsp_pwr.h): there is no arm-only
     * HAL call -- HAL_PWR_EnterSTANDBYMode() arms AND enters in one
     * atomic sequence (WUF cleared, PDDS=1, SLEEPDEEP=1, WFI). It does
     * not return: the 1.2 V domain powers down and the wake is a reset-
     * class boot. NOT armed anywhere in the firmware: entering Standby
     * loses SRAM -- see the safety contract in the header. */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU); /* CR CWUF=1: stale WUF would make
                                        * the Standby entry get IGNORED   */
    HAL_PWR_EnterSTANDBYMode();
    /* No code after this line ever runs on this chip (header contract). */
}

int bsp_pwr_standby_flag(void)
{
    return (__HAL_PWR_GET_FLAG(PWR_FLAG_SB)) ? 1 : 0;
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
        if ((SCB->SCR & SCB_SCR_SLEEPDEEP_Msk) != 0u) { /* read-only (A3 exemption) */
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
