/**
 * @file    bsp_exti.c
 * @brief   EXTI routes for KEY1 (PA0 = EXTI0, rising) and KEY2 (PA1 =
 *          EXTI1, falling) with ISR press counter -- HAL edition -- see
 *          bsp_exti.h for the contract and the register->HAL map.
 ******************************************************************************
 * Console-free module (bsp-module-spec rule #7): no prompt goes to the
 * UART (the old edition poked USART1 registers directly -- banned under
 * the A3 rule here); the GREEN LED blink is the interactive cue. The
 * IWDG is fed through the HAL handle (hiwdg, ~8.2 s period at prescaler
 * 64/reload 4095): the 6 s window outlives the idle-menu margin, so the
 * feed is mandatory, not decoration.
 *
 * WIRING STATE AT THE TIME OF WRITING (see bsp_exti.h for the full
 * note): the generated MX_GPIO_Init electricals for PA0/PA1 are degraded
 * (both IT_RISING, no pull) and the generated it.c EXTI vectors are
 * EMPTY bodies -- this module re-arms the routes via HAL_GPIO_Init and
 * enables the vectors; the Core-side USER CODE fill (one
 * HAL_GPIO_EXTI_IRQHandler call per vector) is what eventually connects
 * the edges to HAL_GPIO_EXTI_Callback below. Pin macros: the regenerated
 * main.h carries no KEY labels, so the PA0/PA1 pins are named by
 * literals with the KEY mapping spelled out.
 */
#include "bsp_exti.h"

#include "main.h" /* LED_GREEN pin macros from CubeMX */
#include "stm32f4xx_hal.h"
#include "iwdg.h" /* hiwdg from MX_IWDG_Init (CubeMX-generated) */
#include "bsp_pwr.h" /* bsp_pwr_on_exti0: EXTI0 router target */ /* hiwdg from MX_IWDG_Init (CubeMX-generated) */

/* ---- module constants ----------------------------------------------------- */

/* Pin truth (unlabeled in the regenerated main.h -- the mapping):
 * PA0 = KEY1 = EXTI line 0 (rising, pull-down),
 * PA1 = KEY2 = EXTI line 1 (falling, pull-up). */
#define BSP_EXTI_KEY1_PIN GPIO_PIN_0
#define BSP_EXTI_KEY2_PIN GPIO_PIN_1

/* Interactive window (ms): the whole cue-to-verdict wait is bounded by
 * this budget (spec #5: no open-ended waits). */
#define BSP_EXTI_WINDOW_MS 6000u

/* Presses required for the "(N presses)" verdict. */
#define BSP_EXTI_PRESSES_NEEDED 3u

/* LED cue period (ms): GREEN toggles while the window is open. */
#define BSP_EXTI_LED_PERIOD_MS 150u

/* IWDG feed period (ms): comfortably inside the ~8.2 s dog window. */
#define BSP_EXTI_FEED_PERIOD_MS 500u

/* NVIC priority for both vectors: 5, matching the .ioc NVIC section
 * (and the priority MX_GPIO_Init sets). */
#define BSP_EXTI_NVIC_PRIO 5u

/* Self-test step numbers (the value bsp_exti_selftest returns). */
#define BSP_EXTI_ST1_CONFIG 1u /* route/config read-back */
#define BSP_EXTI_ST2_WINDOW 2u /* interactive 6 s window  */

/* ---- module state ---------------------------------------------------------- */

/* ISR-side press counter (volatile: mainline polls it, see walkthrough
 * in the register edition -- without the qualifier the compiler may
 * hoist the poll out of the wait loop and the count appears frozen). */
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

/* Route read-back probes, shared by init and selftest step 1. All
 * read-only (A3 read-only exemption): the HAL-owned configuration is
 * verified, not rewritten. Returns 0 = intact, -1 = mismatch. */
static int exti_route_intact(void)
{
    /* Interrupt masks: both lines unmasked as interrupts (not events). */
    if (((EXTI->IMR & EXTI_IMR_MR0) == 0u) ||
        ((EXTI->IMR & EXTI_IMR_MR1) == 0u)) {
        return -1;
    }
    /* Trigger shapes: KEY1 (line 0) rising ONLY, KEY2 (line 1) falling
     * ONLY -- a release must not count as a press on either key. */
    if (((EXTI->RTSR & EXTI_RTSR_TR0) == 0u) ||
        ((EXTI->FTSR & EXTI_FTSR_TR0) != 0u) ||
        ((EXTI->FTSR & EXTI_FTSR_TR1) == 0u) ||
        ((EXTI->RTSR & EXTI_RTSR_TR1) != 0u)) {
        return -1;
    }
    /* Route: lines 0 and 1 fed by port A (EXTICR nibbles 0000). */
    if ((SYSCFG->EXTICR[0] &
         (SYSCFG_EXTICR1_EXTI0_Msk | SYSCFG_EXTICR1_EXTI1_Msk)) != 0u) {
        return -1;
    }
    /* Pin modes: input (MODER 00) with IOC pulls -- KEY1 pull-down
     * (10), KEY2 pull-up (01). */
    if (((GPIOA->MODER & GPIO_MODER_MODER0_Msk) != 0u) ||
        ((GPIOA->MODER & GPIO_MODER_MODER1_Msk) != 0u)) {
        return -1;
    }
    if (((GPIOA->PUPDR & GPIO_PUPDR_PUPD0_Msk) != (2u << GPIO_PUPDR_PUPD0_Pos)) ||
        ((GPIOA->PUPDR & GPIO_PUPDR_PUPD1_Msk) != (1u << GPIO_PUPDR_PUPD1_Pos))) {
        return -1;
    }
    /* NVIC vectors live. */
    if ((NVIC_GetEnableIRQ(EXTI0_IRQn) == 0u) ||
        (NVIC_GetEnableIRQ(EXTI1_IRQn) == 0u)) {
        return -1;
    }
    return 0;
}

/* ---- HAL callback (weak override) ------------------------------------------- */

/* Weak-override callback: fires for BOTH EXTI lines once the Core-side
 * vectors dispatch into HAL (one callback serves the whole port), so
 * filter by pin -- only KEY2 (PA1) presses count. Short by contract:
 * bump a volatile counter, nothing else (an ISR is a note-taking
 * service, the mainline does the work). */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BSP_EXTI_KEY2_PIN) {
        s_key2_presses++;
    }
    if (GPIO_Pin == BSP_EXTI_KEY1_PIN) {
        /* EXTI0 router: the pwr module counts wakes through this hook
         * (single HAL_GPIO_EXTI_Callback owner rule, W4b integration). */
        bsp_pwr_on_exti0();
    }
}

/* ---- public API ------------------------------------------------------------ */

int bsp_exti_init(void)
{
    GPIO_InitTypeDef route;

    /* KEY1 (PA0): rising edge = press (pull-down idle 0, press to VDD).
     * The HAL_GPIO_Init call writes the whole old chain in one go:
     * SYSCFG EXTICR nibble, FTSR/RTSR latches, IMR unmask. The re-arm
     * also repairs the generated electricals (MX_GPIO_Init currently
     * leaves both pins rising with no pull). */
    route.Pin   = BSP_EXTI_KEY1_PIN;
    route.Mode  = GPIO_MODE_IT_RISING;
    route.Pull  = GPIO_PULLDOWN;
    route.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &route);

    /* KEY2 (PA1): falling edge = press (pull-up idle 1, press to GND). */
    route.Pin   = BSP_EXTI_KEY2_PIN;
    route.Mode  = GPIO_MODE_IT_FALLING;
    route.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &route);

    /* Kill stale pending bits before the vectors go live (the HAL macro
     * spelling of the old PR = rc_w1 write): a press before init must
     * not pre-fire the counter -- and a set PR bit would make a later
     * WFI/Stop entry get IGNORED (the bsp_pwr lesson, unchanged). */
    __HAL_GPIO_EXTI_CLEAR_IT(BSP_EXTI_KEY1_PIN);
    __HAL_GPIO_EXTI_CLEAR_IT(BSP_EXTI_KEY2_PIN);

    /* NVIC vectors (EXTI0 = 6, EXTI1 = 7), priority 5 per the .ioc
     * (MX_GPIO_Init sets the priorities; the ENABLE is this module's
     * route-arming job). */
    HAL_NVIC_SetPriority(EXTI0_IRQn, BSP_EXTI_NVIC_PRIO, 0u);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_SetPriority(EXTI1_IRQn, BSP_EXTI_NVIC_PRIO, 0u);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    /* Read-back proof: the whole chain must have stuck. */
    if (exti_route_intact() != 0) {
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

    /* Step 1: route read-back (the same checks init proved). */
    if (exti_route_intact() != 0) {
        return (int)BSP_EXTI_ST1_CONFIG;
    }

    /* Step 2: interactive window. The GREEN LED toggles every 150 ms
     * while the window is open (the cue the register edition paired
     * with its UART prompt -- console-free here, watch the BOARD).
     * IWDG fed every 500 ms: the window outlives the idle-menu margin
     * to the ~8.2 s dog bite. CUMULATIVE verdict: presses count across
     * the whole uptime -- the EXTI mechanism's contract is "edges are
     * counted by hardware+ISR", which does not expire; >= 3 presses ->
     * "(N presses)", timeout -> "(idle, N presses)" -- both PASS
     * (hardware-absence-is-SKIP rule). */
    HAL_IWDG_Refresh(&hiwdg);
    {
        const uint32_t start = HAL_GetTick();
        uint32_t elapsed = 0u;
        uint32_t last_led = 0u;
        uint32_t last_feed = 0u;

        while (elapsed < BSP_EXTI_WINDOW_MS) {
            if (s_key2_presses >= BSP_EXTI_PRESSES_NEEDED) {
                break; /* early exit: the demo window already told */
            }
            if ((elapsed - last_led) >= BSP_EXTI_LED_PERIOD_MS) {
                HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
                last_led = elapsed;
            }
            if ((elapsed - last_feed) >= BSP_EXTI_FEED_PERIOD_MS) {
                HAL_IWDG_Refresh(&hiwdg);
                last_feed = elapsed;
            }
            HAL_Delay(1);
            elapsed = HAL_GetTick() - start;
        }
        /* LED boot state back (PC5 boot low), whatever happened. */
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,
                          GPIO_PIN_RESET);

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
