/**
 ******************************************************************************
 * @file    bsp_wdg.c
 * @brief   IWDG + WWDG + reset-source flags BSP (HAL edition) -- see
 *          bsp_wdg.h for the inverted safety contract, the feeding
 *          discipline and the razor-window finding.
 ******************************************************************************
 * @addtogroup BSP_WDG
 * @{
 *
 * == HAL EDITION MAP (register edition -> here) ===========================
 * RCC->CSR LSION + LSIRDY wait               -> HAL_RCC_OscConfig (LSI ON,
 *                                               SystemClock_Config, boot).
 * KR 0x5555 unlock / PR / RLR / 0xFFFF lock  -> HAL_IWDG_Init (driver does
 *                                               the key dance internally).
 * KR 0xCCCC START (one-way)                  -> same, inside HAL_IWDG_Init
 *                                               at MX_IWDG_Init (boot).
 * KR 0xAAAA reload                           -> HAL_IWDG_Refresh.
 * APB1 WWDGEN gate + CFR + WDGA activation   -> HAL_WWDG_MspInit +
 *                                               HAL_WWDG_Init at MX_WWDG_Init
 *                                               (boot; sets WDGA one-way).
 * WWDG CR rewrite (refresh)                  -> HAL_WWDG_Refresh, issued
 *                                               window-guarded here.
 * RCC->CSR RMVF write-1-to-clear             -> DROPPED (A3); app_main.c
 *                                               clears the flags at boot.
 * RCC->CSR flag reads                        -> kept as read-only probes.
 *
 * == CUBEMX CONFIG (in f407-periph-lab-hal.ioc now) ========================
 * | Field             | Value (generated truth)                | Note
 * |-------------------|----------------------------------------|------------
 * | IWDG              | prescaler /64, reload 4095             | 8.192 s
 * |                   |                                        | nominal
 * | WWDG              | prescaler /8, counter 127, window 64,  | razor window
 * |                   | EWI off                                | (header)
 * | WDG_SW option     | software start (CubeMX default)        | both start
 * |                   |                                        | at MX_*_Init
 * | NVIC              | WWDG_IRQn off (EWI unused)             | polling
 * == HOW TO VERIFY =========================================================
 * menu> wdg
 *   PASS wdg (iwdg ok, wwdg razor win)  -- plans verified + flag decode
 * menu> wdg_reset_demo   (or the shell demo alias)
 *   resets in seconds; next boot's RTT log carries EV_WATCHDOG_RESET;
 *   re-run 'wdg' to see the flag decode. Debugger: watch IWDG->RLR (4095)
 *   and WWDG->CR (0xFF = WDGA|0x7F -- the counter visibly descending).
 */
#include "bsp_wdg.h"

#include <stddef.h>

#include "stm32f4xx_hal.h"
#include "iwdg.h" /* hiwdg from MX_IWDG_Init (CubeMX-generated) */
#include "wwdg.h" /* hwwdg from MX_WWDG_Init (CubeMX-generated) */

/* ---- module constants ----------------------------------------------------- */

/* Self-test step numbers (the value bsp_wdg_selftest returns). */
#define BSP_WDG_ST1_IWDG_CFG 1u /* plan + PR/RLR read-back   */
#define BSP_WDG_ST2_RSTFLAGS 2u /* CSR snapshot + decode     */
#define BSP_WDG_ST3_WWDG_CFG 3u /* CFR read-back + WDGA=1    */

/* The seven reset-cause flag bits, for snapshot/decode. */
#define BSP_WDG_RSTFLAGS_MASK (RCC_CSR_BORRSTF | RCC_CSR_PINRSTF | \
                               RCC_CSR_PORRSTF | RCC_CSR_SFTRSTF | \
                               RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF | \
                               RCC_CSR_LPWRRSTF)

/* Razor-window threshold (header finding): the legal refresh strip is
 * (W - 0x40 + 1) ticks; at /8 a tick is 780.2 us, so anything below
 * 0x42 leaves less than 2.4 ms of strip -- un-feedable for any loop. */
#if (BSP_WDG_WWDG_W < 0x42u)
#define BSP_WDG_WWDG_RAZOR 1
#else
#define BSP_WDG_WWDG_RAZOR 0
#endif

/* ---- private state ---------------------------------------------------------- */

static char s_report[48];
static int s_report_valid = 0;

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

/* Step-2 decode table: flag bit -> report word (register edition kept
 * the same vocabulary; the report stays printf-free). */
typedef struct {
    uint32_t mask;
    const char *name;
} rst_flag_name_t;

static const rst_flag_name_t s_rst_names[7] = {
    { RCC_CSR_PINRSTF, "pin" },   { RCC_CSR_PORRSTF, "por" },
    { RCC_CSR_SFTRSTF, "sft" },   { RCC_CSR_IWDGRSTF, "iwdg" },
    { RCC_CSR_WWDGRSTF, "wwdg" }, { RCC_CSR_BORRSTF, "bor" },
    { RCC_CSR_LPWRRSTF, "lpwr" },
};

/* Build "rst: pin+por" or "rst: none" into dst (cap = sizeof dst - 1). */
static void rst_flags_decode(uint32_t flags, char *dst, unsigned cap)
{
    unsigned i = 0u;
    unsigned k;
    dst[i++] = 'r';
    dst[i++] = 's';
    dst[i++] = 't';
    dst[i++] = ':';
    dst[i++] = ' ';
    for (k = 0u; k < 7u; k++) {
        if ((flags & s_rst_names[k].mask) != 0u) {
            const char *n = s_rst_names[k].name;
            /* cap-2 keeps room for one more separator + the terminator:
             * a full seven-flag decode truncates, it never overflows. */
            if ((dst[i - 1u] != ' ') && (i < (cap - 2u))) {
                dst[i++] = '+';
            }
            while ((*n != '\0') && (i < (cap - 2u))) {
                dst[i++] = *n++;
            }
        }
    }
    if (dst[i - 1u] == ' ') {
        dst[i++] = 'n';
        dst[i++] = 'o';
        dst[i++] = 'n';
        dst[i++] = 'e';
    }
    dst[i] = '\0';
}

/* ---- public API ------------------------------------------------------------ */

int bsp_wdg_init(void)
{
    /* --- IWDG: the boot-armed 8.192 s plan ------------------------------ */
    if ((hiwdg.Instance == NULL) ||
        (hiwdg.Init.Prescaler != IWDG_PRESCALER_64) ||
        (hiwdg.Init.Reload != BSP_WDG_IWDG_RLR)) {
        report_set("(iwdg plan mismatch)");
        return -1;
    }
    /* Read-only (A3 exemption). The PR/RLR reads are trustworthy here
     * BECAUSE the dog is armed: the register edition's 2026-11-21 board
     * finding was that an UNARMED IWDG never completes its shadow sync
     * and serves stale reads -- HAL_IWDG_Init starting the dog at boot
     * is exactly the design that makes verification possible (EP15
     * teaching point). */
    if ((IWDG->PR != BSP_WDG_IWDG_PR) ||
        (IWDG->RLR != BSP_WDG_IWDG_RLR)) {
        report_set("(iwdg readback mismatch)");
        return -1;
    }

    /* --- WWDG: the boot-armed /8 + window plan --------------------------- */
    if ((hwwdg.Instance == NULL) ||
        (hwwdg.Init.Prescaler != WWDG_PRESCALER_8) ||
        (hwwdg.Init.Counter != BSP_WDG_WWDG_COUNTER) ||
        (hwwdg.Init.Window != BSP_WDG_WWDG_W)) {
        report_set("(wwdg plan mismatch)");
        return -1;
    }
    /* Read-back (read-only, A3 exemption) + the WDGA inversion: the
     * register edition proved the dog PARKED (WDGA = 0); the HAL
     * edition proves the dog RUNNING (MX_WWDG_Init set WDGA -- one-way,
     * only reset undoes it). */
    if ((WWDG->CFR != (WWDG_PRESCALER_8 | BSP_WDG_WWDG_W)) ||
        ((WWDG->CR & WWDG_CR_WDGA) == 0u)) {
        report_set("(wwdg readback mismatch)");
        return -1;
    }

#if BSP_WDG_WWDG_RAZOR
    report_set("(iwdg ok, wwdg razor win)");
#else
    report_set("(iwdg ok, wwdg ok)");
#endif
    return 0;
}

void bsp_wdg_iwdg_feed(void)
{
    HAL_IWDG_Refresh(&hiwdg); /* KR=0xAAAA inside the driver: reload */
}

void bsp_wdg_refresh(void)
{
    /* IWDG: unconditional -- no window law, just stay inside the 5.6 s
     * fast-edge deadline (feeding discipline in the header). */
    HAL_IWDG_Refresh(&hiwdg);
    /* WWDG: window-GUARDED. The counter value is CR T[6:0]; refreshing
     * while it runs ABOVE the window value resets instantly (RM0090
     * ch.22.4.2). The read-only probe (A3 exemption) gates the driver's
     * refresh until the counter has dropped into the legal strip. With
     * the generated W = 0x40 the strip is the single tick T == 0x40, so
     * the guard rarely fires and the ~49 ms WWDG deadline stands until
     * the .ioc relaxes the window (header, deferred decision). */
    if ((WWDG->CR & WWDG_CR_T) <= (BSP_WDG_WWDG_W & WWDG_CR_T)) {
        HAL_WWDG_Refresh(&hwwdg);
    }
}

void bsp_wdg_reset_demo(void)
{
    /* The dogs are ALREADY armed (boot init) -- the register edition's
     * "configure then KR=0xCCCC" preamble collapsed into one final
     * feed: the full 8.192 s nominal window starts NOW, then the feed
     * path dies. NEVER RETURNS. The shell prints the warning BEFORE
     * calling this; after the reset, boot_diag turns the CSR flags into
     * the RTT EV_WATCHDOG_RESET event and 'wdg' re-decodes them. */
    HAL_IWDG_Refresh(&hiwdg);
    for (;;) {
        /* Deliberately NO feed: IWDG deadline 5.58..15.42 s across the
         * LSI spread (EP15 margin lesson). Sleep-safe: the IWDG counts
         * its own LSI in the VDD domain. NOTE (header): with the razor
         * WWDG window unfixed in the .ioc, the WWDG fires first at
         * ~49 ms and its flag becomes the observed reset cause. */
    }
}

uint32_t bsp_wdg_reset_flags(void)
{
    /* read-only (A3 exemption): sticky reset-cause bits, cleared only
     * by app_main's boot-time __HAL_RCC_CLEAR_RESET_FLAGS(). */
    return RCC->CSR & BSP_WDG_RSTFLAGS_MASK;
}

const char *bsp_wdg_last_report(void)
{
    return (s_report_valid != 0) ? s_report : "";
}

int bsp_wdg_selftest(void)
{
    /* Keep the armed dog quiet across the run; the real feeding
     * discipline lives in the shell's bsp_wdg_refresh() loop call. */
    bsp_wdg_iwdg_feed();

    /* Step 1: IWDG plan + PR/RLR read-back (also verifies the WWDG
     * plan -- init covers both dogs). */
    if (bsp_wdg_init() != 0) {
        return (int)BSP_WDG_ST1_IWDG_CFG;
    }

    /* Step 2: reset-source mechanics. Snapshot -> decode into the
     * report. INTENTIONAL CHANGE: the RMVF clear + verify-empty cycle
     * is dropped (A3 forbids the CSR write; app_main.c already clears
     * the flags at boot). What the snapshot CONTAINS depends on how
     * long ago the last reset was: after a real wdg_reset_demo + reboot
     * the RTT log holds the EV_WATCHDOG_RESET evidence and a fresh
     * snapshot here usually reads "none" (app_main cleared the flags
     * minutes earlier). Both are correct; the decodable truth is the
     * contract. */
    {
        char msg[32];
        rst_flags_decode(bsp_wdg_reset_flags(), msg, (unsigned)sizeof(msg));
        report_set(msg);
    }

    /* Step 3: WWDG read-back, repeated standalone so a step-3 failure
     * is distinguishable from a step-1 failure. WDGA must be SET (the
     * boot-armed dog; inversion documented in the header). The razor
     * window is reported, not failed -- it is the .ioc's truth, and the
     * escalation note lives in the file header. */
    if ((WWDG->CFR != (WWDG_PRESCALER_8 | BSP_WDG_WWDG_W)) ||
        ((WWDG->CR & WWDG_CR_WDGA) == 0u)) {
#if BSP_WDG_WWDG_RAZOR
        report_set("(wwdg cfg broke razor)");
#else
        report_set("(wwdg cfg broke)");
#endif
        return (int)BSP_WDG_ST3_WWDG_CFG;
    }
    {
        /* Compose the final report: decode + config verdict. Keep the
         * flag decode visible AND mark the razor finding. */
        char msg[32];
        unsigned i = 0u;
        rst_flags_decode(bsp_wdg_reset_flags(), msg, (unsigned)sizeof(msg));
        while ((msg[i] != '\0') && (i < (sizeof(s_report) - 1u))) {
            s_report[i] = msg[i];
            i++;
        }
#if BSP_WDG_WWDG_RAZOR
        {
            static const char tail[] = ", wwdg razor";
            unsigned k = 0u;
            while ((tail[k] != '\0') && (i < (sizeof(s_report) - 1u))) {
                s_report[i++] = tail[k++];
            }
        }
#endif
        s_report[i] = '\0';
        s_report_valid = 1;
    }
    return 0; /* all steps passed */
}

/** @} */
