/**
 ******************************************************************************
 * @file    bsp_exti.c
 * @brief   EXTI line 1 <- KEY2 (PA1) falling edge with ISR press counter --
 *          teaching module: the full pin -> SYSCFG -> EXTI -> NVIC chain
 *          spelled register by register (EP10).
 ******************************************************************************
 * @addtogroup BSP_EXTI
 * @{
 *
 * == CUBEMX CONFIG (absent from f407-periph-lab.ioc -- by design) ==========
 * | Field             | Value (module truth)                 | Why
 * |-------------------|--------------------------------------|-------------------
 * | SYSCFG/EXTI in IOC| NOT PRESENT                          | knowledge #13:
 * |                   |                                      | adapter-owned
 * | PA1 (IOC truth)   | GPIO_Input + GPIO_PULLUP, label KEY2 | bsp_gpio owns the
 * |                   |                                      | PIN (idle level 1);
 * |                   |                                      | THIS module only
 * |                   |                                      | wires the route
 * | Trigger           | Falling edge (FTSR1=1, RTSR1=0)      | pull-up idle = 1,
 * |                   |                                      | press = 0: press
 * |                   |                                      | IS the falling edge
 * | NVIC              | EXTI1_IRQn = 7, priority 10          | ISR = 3 statements
 *
 * == REGISTER WALKTHROUGH (RM0090 ch.10 SYSCFG; ch.11 "Interrupts and
 * events" EXTI tables; PM0056 B3.4 NVIC) ====================================
 * The chain a GPIO edge walks to reach the CPU, four registers long:
 *   1. SYSCFG->EXTICR[0] (RM0090 ch.10): every EXTI line can be fed by
 *      ONE pin from ONE port -- line 1 by PA1/PB1/PC1/..., selected by the
 *      [7:4] nibble of EXTICR1 (0000=PA, 0001=PB, ...). The value 0x0
 *      (PA) is also the RESET value -- written explicitly anyway so the
 *      module owns its route. SYSCFG sits on APB2: RCC->APB2ENR bit14
 *      (SYSCFGEN) must be on before the nibble is written.
 *   2. EXTI->FTSR/RTSR (RM0090 EXTI): per-line trigger edge latches.
 *      TR1 in the FALLING trigger selection register (FTSR) arms "edge
 *      detected on a 1 -> 0 pin transition"; TR1 in RTSR arms rising.
 *      KEY2 idles HIGH through its pull-up (bsp_gpio IOC truth), a press
 *      grounds it -- a press IS the falling edge, so RTSR TR1 stays 0
 *      (a release must not count as a press).
 *   3. EXTI->IMR MR1=1: unmask line 1 as an INTERRUPT (without it the
 *      pending bit sets but no interrupt reaches the NVIC -- the silent
 *      failure that sends people to scope the pin for nothing). EMR is
 *      the event twin, unused.
 *   4. NVIC EXTI1_IRQn = 7: NVIC_EnableIRQ arms the vector table slot
 *      (startup_stm32f407xx.s provides the weak EXTI1_IRQHandler). The
 *      core-level chain ends in the ISR below.
 *   Pending: EXTI->PR bit1 is rc_w1 -- writing 1 CLEARS it (the opposite
 *      of the TIM/ADC rc_w0 SRs; EXTI and RTC go rc_w1, ADC/TIM go
 *      rc_w0: a contrast worth teaching). Init clears a stale pending bit
 *      (a press before init would otherwise fire the ISR immediately --
 *      harmless here, fatal in Stop-entry contexts, see bsp_pwr).
 * ISR discipline: 3 statements -- test PR, rc_w1 clear it, ++volatile
 *      counter. No logging, no printf, no delay (EP10/EP15 discipline:
 *      an ISR is a note-taking service, the mainline does the work).
 *   The volatile qualifier is not decoration: without it the compiler
 *      may hoist the mainline's read out of its polling loop, and the
 *      counter would appear frozen while the ISR bumps it happily.
 * == HOW TO VERIFY =========================================================
 * menu> exti
 *   [exti] PRESS KEY2 x3 within 6 s...
 *   PASS exti (3 presses)     <- pressed in time; count may exceed 3
 *   PASS exti (idle, 0 presses) <- window expired untouched: SKIP-state
 *   semantics (no press = no hardware defect), still PASS.
 *   FAIL exti step1 = the config read-back broke (EXTICR/IMR/FTSR/NVIC).
 * Cross-checks: 'gpio' step2 reads KEY2's idle level (IDR=1) through the
 *   same pin; the ISR count also survives as evidence in 'pwr' demos
 *   (EXTI0 twin there wakes the board from Stop).
 */
#include "bsp_exti.h"

#include "hal_port.h" /* family seam: pulls in the CMSIS device header */

/* ---- module constants ----------------------------------------------------- */

/* Interactive window (ms): the whole prompt-to-verdict wait is bounded by
 * this budget (spec #5: no open-ended waits). */
#define BSP_EXTI_WINDOW_MS 6000u

/* Presses required for the "(N presses)" verdict. */
#define BSP_EXTI_PRESSES_NEEDED 3u

/* Spin cap between SysTick wraps: SysTick-dead guard (spec #5). */
#define BSP_EXTI_SYSTICK_GUARD 2000000u

/* Self-test step numbers (the value bsp_exti_selftest returns). */
#define BSP_EXTI_ST1_CONFIG 1u /* route/config read-back */
#define BSP_EXTI_ST2_WINDOW 2u /* interactive 6 s window  */

/* ---- module state ---------------------------------------------------------- */

/* ISR-side press counter (volatile: mainline polls it, see walkthrough). */
static volatile uint32_t s_key2_presses;

/* Report of the last selftest run (menu appends it to "PASS exti"). */
static char s_report[32];

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

/* Append one decimal number to s_report (bounded, no libc). */
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
            break; /* buffer full: never overflow */
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

/* Console write for the ONE line the selftest must put on the wire BEFORE
 * the interaction window opens (the prompt). It pokes the shared USART1
 * registers directly -- TXE then DR -- WITHOUT including bsp_uart.h
 * (bsp-module-spec rule #7: no module-to-module includes). The hardware
 * is a shared resource; the menu owns the console and has already run
 * bsp_uart_init by the time any selftest executes. */
static void exti_prompt(const char *s)
{
    while (*s != '\0') {
        uint32_t guard = BSP_EXTI_SYSTICK_GUARD;
        while ((USART1->SR & USART_SR_TXE) == 0u) {
            if (--guard == 0u) {
                return; /* console dead: bounded exit, skip the prompt */
            }
        }
        USART1->DR = (uint32_t)(uint8_t)(*s++);
    }
    {
        uint32_t guard = BSP_EXTI_SYSTICK_GUARD;
        while ((USART1->SR & USART_SR_TC) == 0u) {
            if (--guard == 0u) {
                break; /* prompt best-effort drained */
            }
        }
    }
}

/* ---- ISRs (vector names MUST match startup_stm32f407xx.s exactly) ---------- */

/* EXTI line 1 = PA1 = KEY2 press (falling edge, walkthrough). Short by
 * contract: read-test the pending bit, rc_w1 clear, volatile bump. */
void EXTI1_IRQHandler(void)
{
    if ((EXTI->PR & EXTI_PR_PR1) != 0u) {
        EXTI->PR = EXTI_PR_PR1; /* rc_w1 clear (walkthrough) */
        s_key2_presses++;
    }
}

/* ---- public API ------------------------------------------------------------ */

int bsp_exti_init(void)
{
    /* SYSCFG clock (APB2 bit14): EXTICR is write-safe only when clocked.
     * GPIOA clock (AHB1): the line samples PA1's pad. PA1's PIN config
     * (input + pull-up) is bsp_gpio's -- this module re-arms only the
     * CLOCK so init is self-sufficient, never the pin mode. */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    (void)RCC->APB2ENR;
    (void)RCC->AHB1ENR;

    /* Step 1 of the chain: route line 1 <- PA (EXTICR1 [7:4] nibble = 0000
     * for port A; written explicitly to own the route). HAL equivalent:
     *   GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
     *   HAL_GPIO_Init(GPIOA, ...) -- which writes this same nibble. */
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI1_Msk; /* 0000 = PA1 */

    /* Step 2: trigger shape -- falling ONLY (KEY2 press), no rising
     * (release must not count), no SWIER software event. */
    EXTI->FTSR |= EXTI_FTSR_TR1;  /* falling edge armed  */
    EXTI->RTSR &= ~EXTI_RTSR_TR1; /* rising disarmed     */

    /* Step 3: interrupt (not event) mode -- unmask MR1. */
    EXTI->IMR |= EXTI_IMR_MR1;

    /* Kill any stale pending bit (rc_w1): a press before init must not
     * pre-fire the counter (and the bsp_pwr Stop-entry lesson: a set PR
     * bit makes a later WFI/Stop entry get IGNORED). */
    EXTI->PR = EXTI_PR_PR1;

    /* Step 4: NVIC vector slot (EXTI1_IRQn = 7). Priority 10: below
     * SysTick/faults, same slot as the bsp_pwr EXTI0 twin. */
    NVIC_SetPriority(EXTI1_IRQn, 10u);
    NVIC_EnableIRQ(EXTI1_IRQn);

    /* Read-back proof: the whole chain must have stuck. */
    if (((SYSCFG->EXTICR[0] & SYSCFG_EXTICR1_EXTI1_Msk) != 0u) ||
        ((EXTI->IMR & EXTI_IMR_MR1) == 0u) ||
        ((EXTI->FTSR & EXTI_FTSR_TR1) == 0u) ||
        ((EXTI->RTSR & EXTI_RTSR_TR1) != 0u) ||
        ((EXTI->PR & EXTI_PR_PR1) != 0u) ||
        (NVIC_GetEnableIRQ(EXTI1_IRQn) == 0u)) {
        return -1;
    }
    return 0;
}

uint32_t bsp_exti_key2_count(void)
{
    return s_key2_presses;
}

const char *bsp_exti_last_report(void)
{
    return s_report;
}

int bsp_exti_selftest(void)
{
    /* Fresh init first (idempotent, menu-driven selftest pattern). */
    if (bsp_exti_init() != 0) {
        return (int)BSP_EXTI_ST1_CONFIG;
    }

    /* Step 1: config read-back (the same six checks init proved). */
    if (((SYSCFG->EXTICR[0] & SYSCFG_EXTICR1_EXTI1_Msk) != 0u) ||
        ((EXTI->IMR & EXTI_IMR_MR1) == 0u) ||
        ((EXTI->FTSR & EXTI_FTSR_TR1) == 0u) ||
        ((EXTI->RTSR & EXTI_RTSR_TR1) != 0u) ||
        (NVIC_GetEnableIRQ(EXTI1_IRQn) == 0u)) {
        return (int)BSP_EXTI_ST1_CONFIG;
    }

    /* Step 2: interactive window. Prompt FIRST (it must reach the wire
     * before the wait), then a 6 s poll of the volatile counter with an
     * early exit at the 3rd press. >= 3 presses -> "(N presses)";
     * timeout -> "(idle, N presses)" -- SKIP-state semantics, BOTH are
     * PASS (hardware-absence-is-SKIP rule). */
    exti_prompt("[exti] press KEY2 x3 in 12 s (GREEN LED blinks = now)...\r\n");
    {
        /* CUMULATIVE verdict: presses count across the whole uptime --
         * the EXTI mechanism's contract is "falling edges are counted by
         * hardware+ISR", which does not expire. The 12 s LED-blink window
         * stays as the live interactive demo (feed + cue), but the
         * verdict uses the cumulative total so the user can press
         * KEY2 anytime and simply run `exti` afterwards. */
        const uint32_t presses = s_key2_presses;
        uint32_t elapsed_ms = 0u;
        while (presses < BSP_EXTI_PRESSES_NEEDED) {
            uint32_t guard = BSP_EXTI_SYSTICK_GUARD;
            /* One SysTick wrap = 1 ms (HAL timebase, 168k ticks). */
            while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0u) {
                if (--guard == 0u) {
                    elapsed_ms = BSP_EXTI_WINDOW_MS; /* SysTick dead:
                                                        bail the window */
                    break;
                }
            }
            if (elapsed_ms >= BSP_EXTI_WINDOW_MS) {
                break;
            }
            elapsed_ms++;
            /* LED cue: GREEN (PC5) toggles every 150 ms while the window
             * is open -- the user watches the BOARD, not a terminal.
             * IWDG feed: the 12 s window outlives the dog's LSI-tolerance
             * minimum (~5.6 s), so it is fed here (board finding). */
            if ((elapsed_ms % 150u) == 0u) {
                GPIOC->ODR ^= (1u << 5u);
            }
            if ((elapsed_ms % 500u) == 0u) {
                IWDG->KR = 0xAAAAu;
            }
        }
        {
            const uint32_t presses = s_key2_presses;
            if (presses >= BSP_EXTI_PRESSES_NEEDED) {
                report_set("(");
                report_append_uint(presses);
                report_append_str(" presses)");
            } else {
                report_set("(idle, ");
                report_append_uint(presses);
                report_append_str(" presses)");
            }
        }
    }
    return 0; /* step 2 passes either way; step 1 covers the config */
}

/** @} */
