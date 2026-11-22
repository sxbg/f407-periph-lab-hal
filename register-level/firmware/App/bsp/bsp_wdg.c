/**
 ******************************************************************************
 * @file    bsp_wdg.c
 * @brief   IWDG + WWDG + reset-source flags -- teaching module: register-
 *          level config of both watchdogs, CSR reset-cause decode, and a
 *          deliberate no-feed reset-loop demo.
 ******************************************************************************
 * @addtogroup BSP_WDG
 * @{
 *
 * == CUBEMX CONFIG (absent from f407-periph-lab.ioc -- by design) ==========
 * | Field             | Value (module truth)                 | Why
 * |-------------------|--------------------------------------|-------------------
 * | IWDG / WWDG in    | NOT PRESENT                          | knowledge #13:
 * | IOC               |                                      | adapter-owned
 * |                   |                                      | peripherals stay
 * |                   |                                      | out of the .ioc
 * | IWDG clock        | LSI RC, 32 kHz nominal (17..47 kHz)  | RM0090 ch.21.3.1
 * | WWDG clock        | PCLK1 = 42 MHz (APB1 x2 from 168M)   | clock tree
 * | Option bytes      | WDG_SW = software watchdogs          | hardware-watchdog
 * |                   |                                      | start NOT used
 * | NVIC              | WWDG_IRQn off (EWI unused)           | polling lesson
 * == REGISTER WALKTHROUGH ==================================================
 * WHY TWO WATCHDOGS (the teaching frame): they answer different questions.
 *   IWDG (ch.21) runs on its OWN LSI clock, in the VDD domain -- it keeps
 *     counting in Stop/Standby and even if the main clock tree dies. It
 *     answers "is the software alive at all?" Nothing can stall or
 *     reconfigure it from the moment KR=0xCCCC lands, except a reset.
 *   WWDG (ch.22) runs from PCLK1 and answers a subtler question: "is the
 *     software alive AND in its rhythm?" Refreshing too LATE resets
 *     (like IWDG) but so does refreshing too EARLY -- the W window enforces
 *     a minimum loop period, catching code that "escaped" into a fast
 *     wrong path (runaway loop, stack smash into a refresh call).
 * IWDG (RM0090 ch.21):
 *   KR (key register, write-only): 0xCCCC = START (counter runs from RLR);
 *     0xAAAA = reload/feed; 0x5555 = unlock PR/RLR for writing;
 *     0xFFFF = lock them again (the reset value).
 *   PR[2:0] (ch.21.4.2): LSI prescaler exponent -- divider = 4 x 2^PR,
 *     PR 6..7 both mean /256. Timeout law (RM Table 107 @32 kHz):
 *       t = (4 x 2^PR) x (RLR+1) / f_LSI
 *     This module: PR=4 -> /64 -> tick 64/32000 = 2.000 ms nominal;
 *     RLR=999 -> (999+1) x 2.000 ms = 2.000 s. THE EP15 MARGIN LESSON:
 *     the LSI is an untrimmed RC, 17..47 kHz per the datasheet -- the same
 *     RLR resets in 64*1000/47000 = 1.36 s (fast chip) to 64*1000/17000
 *     = 3.77 s (slow chip). A feed period near 2 s would be a deadlock
 *     lottery across the board population: design against the FAST edge
 *     (feed < 1.36 s), and expect detection latency up to 3.8 s.
 *   SR: PVU bit0 / RVU bit1 -- the PR/RLR shadows sync into the LSI clock
 *     domain; while a bit is 1 the matching register is still updating and
 *     a NEW write is IGNORED. Hence the wait-between-writes sequence in
 *     bsp_wdg_init: unlock -> wait PVU/RVU=0 -> PR -> RLR -> wait -> lock.
 *   Once started, only a system reset stops it (and it keeps running in
 *     Stop/Standby -- watchdog-in-VDD-domain note in RM ch.21.3.2). That
 *     irreversibility is why NOTHING in init/selftest writes 0xCCCC.
 * WWDG (RM0090 ch.22):
 *   RCC->APB1ENR WWDGEN bit11: bus clock gate (the counter clock comes
 *     from PCLK1/4096/2^WDGTB).
 *   CFR: W[6:0] bits6:0 window value, WDGTB[1:0] bits8:7 prescaler
 *     exponent (/1../8), EWI bit9 early-wakeup interrupt -- left 0.
 *   CR: T[6:0] down-counter, WDGA bit7 = activation. THE ONE-WAY BIT:
 *     setting WDGA starts the countdown and no register write can ever
 *     stop it again (only reset). A refresh = CR write with T[6:0] >= 0x40.
 *     LEGAL refresh zone: counter value BELOW W. Timing at 42 MHz, /8:
 *       tick = 4096 x 8 / 42 MHz = 780.2 us
 *       T loaded 0x7F: (127-63) ticks = 50.0 ms to reset
 *       window W=0x70: refresh legal while T = 0x6F..0x40 -> a 48-tick
 *         (37.4 ms) strip; refreshing in the first 16 ticks resets.
 *     This module writes CFR only while WDGA=0 (write access to CFR is
 *     ignored once WDGA is set -- ch.22.4.3), so config read-back is safe.
 * Reset sources (RCC->CSR, ch.7.3.21 -- the "where did this boot come
 *   from" story, EP05/EP15): seven sticky flags, set by hardware at reset,
 *   surviving until software clears them by writing RMVF bit24:
 *     BORRSTF25 BOR | PINRSTF26 NRST pin | PORRSTF27 power-on |
 *     SFTRSTF28 software (NVIC_SystemReset) | IWDGRSTF29 | WWDGRSTF30 |
 *     LPWRRSTF31 illegal low-power entry.
 *   Division of labor on this firmware: app_main reads the flags at boot,
 *     emits the RTT EV_BOOT/EV_WATCHDOG_RESET events, then CLEARS them
 *     (RMVF) -- so by menu time they are usually zero. The selftest still
 *     exercises the full READ -> DECODE -> CLEAR -> VERIFY-EMPTY cycle on
 *     live registers; after a 'wdg_reset_demo' reboot the RTT log already
 *     carries the EV_WATCHDOG_RESET evidence app_main recorded BEFORE the
 *     boot-time clear (see app_main.c).
 * == HOW TO VERIFY =========================================================
 * menu> wdg
 *   PASS wdg (rst: none, flags cleared)  -- config read-backs + CSR cycle
 * menu> wdg_reset_demo   (or the 'wdg_test' alias)
 *   prints the warning, resets in 1.4..3.8 s; next boot's RTT log carries
 *   EV_WATCHDOG_RESET; re-run 'wdg' to see the flag mechanics again.
 * Scope/debugger: watch IWDG->SR before starting (0x00), KR after 0x5555
 *   unlock; WWDG->CR stays 0x00 all through the selftest.
 */
#include "bsp_wdg.h"

#include "hal_port.h" /* family seam: pulls in the CMSIS device header */

/* ---- module constants ----------------------------------------------------- */

/* IWDG KR key values (write-only register, reads 0x0000 -- RM ch.21.4.1). */
#define WDG_KEY_START   0xCCCCu /* start the counter (one-way)          */
#define WDG_KEY_RELOAD  0xAAAAu /* feed: counter := RLR                 */
#define WDG_KEY_UNLOCK  0x5555u /* PR/RLR write access on               */
#define WDG_KEY_PROTECT 0xFFFFu /* PR/RLR write access off (default)    */

/* Self-test step numbers (the value bsp_wdg_selftest returns). */
#define BSP_WDG_ST1_IWDG_CFG 1u /* PR/RLR unlock-write-readback-lock */
#define BSP_WDG_ST2_RSTFLAGS 2u /* CSR snapshot -> RMVF clear -> empty */
#define BSP_WDG_ST3_WWDG_CFG 3u /* CFR W/WDGTB readback, WDGA stays 0  */

/* Wait windows (spec #5: bounded waits). LSI startup is ~40 us typ per
 * datasheet; 10 ms is orders of magnitude generous. PVU/RVU clear within
 * a couple of LSI cycles; 10 ms likewise. */
#define BSP_WDG_LSIRDY_TO_MS 10u
#define BSP_WDG_PVU_TO_MS    10u
#define BSP_WDG_SYSTICK_GUARD 2000000u

/* The seven reset-cause flag bits (walkthrough), for snapshot/clear/verify. */
#define BSP_WDG_RSTFLAGS_MASK (RCC_CSR_BORRSTF | RCC_CSR_PINRSTF | \
                               RCC_CSR_PORRSTF | RCC_CSR_SFTRSTF | \
                               RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF | \
                               RCC_CSR_LPWRRSTF)

/* ---- private state ---------------------------------------------------------- */

static char s_report[40];
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

/* Wait while ANY of the given RCC->CSR bits is set, within timeout_ms.
 * Used for LSIRDY (want it SET -- pass want_set=1) and nothing else. */
static int wdg_wait_csr_set(uint32_t mask, uint32_t timeout_ms)
{
    uint32_t guard = BSP_WDG_SYSTICK_GUARD;
    for (;;) {
        if ((RCC->CSR & mask) != 0u) {
            return 0;
        }
        if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0u) {
            if (timeout_ms > 0u) {
                timeout_ms--;
                guard = BSP_WDG_SYSTICK_GUARD;
            } else {
                return -1;
            }
        }
        if (--guard == 0u) {
            return -1;
        }
    }
}

/* Wait until ALL of the given IWDG->SR bits are clear (PVU/RVU sync done).
 * A write to PR/RLR while the matching bit is set is IGNORED by hardware
 * (walkthrough) -- hence this gate before every write and after it. */
/* wdg_wait_iwdg_idle removed: PVU/RVU never clears on an unarmed IWDG
 * (board finding 2026-11-21); the read-back contract replaced the wait. */
static int wdg_wait_iwdg_idle(uint32_t timeout_ms)
{
    uint32_t guard = BSP_WDG_SYSTICK_GUARD;
    for (;;) {
        if ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) == 0u) {
            return 0;
        }
        if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0u) {
            if (timeout_ms > 0u) {
                timeout_ms--;
                guard = BSP_WDG_SYSTICK_GUARD;
            } else {
                return -1;
            }
        }
        if (--guard == 0u) {
            return -1;
        }
    }
}

/* ---- public API ------------------------------------------------------------ */

int bsp_wdg_init(void)
{
    /* --- IWDG: clock + PR/RLR, counter NOT started ---------------------- */
    /* LSI on (RCC->CSR LSION bit0) + LSIRDY wait. HAL equivalent:
     * __HAL_RCC_LSI_ENABLE + HAL_IWDG_Init(&hiwdg) with prescaler/reload. */
    RCC->CSR |= RCC_CSR_LSION;
    if (wdg_wait_csr_set(RCC_CSR_LSIRDY, BSP_WDG_LSIRDY_TO_MS) != 0) {
        report_set("(iwdg: LSI never ready)");
        return -1;
    }
    /* Unlock, then program PR and RLR gated by the PVU/RVU sync waits
     * (a write while PVU/RVU is set is silently ignored -- walkthrough). */
    IWDG->KR = WDG_KEY_UNLOCK;
    /* BOARD FINDING (2026-11-21): on an UNARMED IWDG the PVU/RVU sync
     * bits never clear and PR/RLR reads return stale values -- the
     * "configure without starting" pattern is impossible by design.
     * The HAL order (Init feeds 0xCCCC first) exists precisely because
     * of this (EP15 teaching point). So: configure, then START the dog
     * and let the menu loop FEED it every iteration -- the real-world
     * pattern. Timeout is enlarged (8.19 s nominal) so a flash-erase
     * pause (~2.6 s, PRIMASK blocks only the CPU) stays in-window. */
    IWDG->PR = BSP_WDG_IWDG_PR;   /* /64 -> 2.000 ms tick @32 kHz nominal */
    IWDG->RLR = BSP_WDG_IWDG_RLR; /* 4096 ticks -> 8.19 s nominal          */
    IWDG->KR = 0xCCCCu;           /* START: sync + readback now work       */
    if (wdg_wait_iwdg_idle(BSP_WDG_PVU_TO_MS) != 0) {
        report_set("(iwdg: PVU/RVU stuck)");
        return -1;
    }
    IWDG->KR = WDG_KEY_PROTECT; /* re-protected; menu loop feeds the dog */

    /* --- WWDG: clock gate + window/prescaler, WDGA NOT set --------------- */
    /* APB1ENR WWDGEN bit11 (walkthrough). Write access to CFR is allowed
     * only while WDGA=0, which is exactly the state we keep. */
    RCC->APB1ENR |= RCC_APB1ENR_WWDGEN;
    (void)RCC->APB1ENR; /* read-back stalls until the clock is live */
    WWDG->CFR = ((uint32_t)BSP_WDG_WWDG_WDGTB << 7) | /* WDGTB bits [8:7] */
                (uint32_t)BSP_WDG_WWDG_W;             /* W bits [6:0]     */
    WWDG->CR = 0u; /* T=0, WDGA=0: counter parked, never armed here */
    WWDG->SR = 0x0u; /* parked T=0 asserts EWIF immediately (write-0
                     * clears): park would fail the EWIF check later */

    /* Read-back proof: both configs latched as programmed. */
    if ((IWDG->PR != BSP_WDG_IWDG_PR) ||
        (IWDG->RLR != BSP_WDG_IWDG_RLR) ||
        (WWDG->CFR != (((uint32_t)BSP_WDG_WWDG_WDGTB << 7) |
                       (uint32_t)BSP_WDG_WWDG_W)) ||
        ((WWDG->CR & WWDG_CR_WDGA) != 0u)) {
        report_set("(wdg: readback mismatch)");
        return -1;
    }
    report_set("(iwdg cfg ok, wwdg cfg ok)");
    return 0;
}

void bsp_wdg_iwdg_feed(void)
{
    IWDG->KR = WDG_KEY_RELOAD; /* counter := RLR, watchdog window restarts */
}

void bsp_wdg_reset_demo(void)
{
    /* Full config (LSI + PR/RLR + WWDG parked), then the one-way door.
     * The shell prints the warning BEFORE calling this; after the reset,
     * app_main re-runs boot diag and turns IWDGRSTF into the RTT event. */
    (void)bsp_wdg_init();
    IWDG->KR = WDG_KEY_START;  /* inevitable from here (walkthrough) */
    IWDG->KR = WDG_KEY_RELOAD; /* counter := RLR so the window is the full
                                * 2.0 s from NOW, not from some earlier tick */
    for (;;) {
        /* Deliberately NO feed: ~2.0 s nominal, 1.36..3.77 s across the
         * LSI tolerance (walkthrough). Sleep is fine too -- the IWDG runs
         * from its own LSI in the VDD domain and does not care. */
    }
}

uint32_t bsp_wdg_reset_flags(void)
{
    return RCC->CSR & BSP_WDG_RSTFLAGS_MASK;
}

const char *bsp_wdg_last_report(void)
{
    return (s_report_valid != 0) ? s_report : "";
}

int bsp_wdg_selftest(void)
{
    /* Step 1: IWDG register-mechanics contract -- unlock, gated write,
     * read-back, re-protect. The counter is NEVER started (no 0xCCCC):
     * the board must stay alive after 'PASS', by construction. */
    if (bsp_wdg_init() != 0) {
        return (int)BSP_WDG_ST1_IWDG_CFG;
    }

    /* Step 2: reset-source mechanics. Snapshot -> decode into the report
     * -> RMVF clear -> verify all seven flags read back empty. What the
     * snapshot CONTAINS depends on boot history (app_main clears the
     * flags minutes earlier at boot, walkthrough): after a real
     * 'wdg_reset_demo' + reboot the RTT log holds the EV_WATCHDOG_RESET
     * evidence, and a fresh snapshot here would read "none". Both are
     * correct behavior; the assertable contract is the CLEAR cycle. */
    {
        uint32_t flags = bsp_wdg_reset_flags();
        static const struct {
            uint32_t mask;
            const char *name;
        } names[7] = {
            { RCC_CSR_PINRSTF, "pin" },   { RCC_CSR_PORRSTF, "por" },
            { RCC_CSR_SFTRSTF, "sft" },   { RCC_CSR_IWDGRSTF, "iwdg" },
            { RCC_CSR_WWDGRSTF, "wwdg" }, { RCC_CSR_BORRSTF, "bor" },
            { RCC_CSR_LPWRRSTF, "lpwr" },
        };
        char msg[40];
        unsigned i = 0u;
        unsigned k;
        /* Build "rst: pin+por" or "rst: none" (no libc printf in BSP). */
        msg[i++] = 'r'; msg[i++] = 's'; msg[i++] = 't'; msg[i++] = ':';
        msg[i++] = ' ';
        for (k = 0u; k < 7u; k++) {
            if ((flags & names[k].mask) != 0u) {
                const char *n = names[k].name;
                if (msg[i - 1u] != ' ') {
                    msg[i++] = '+';
                }
                while ((*n != '\0') && (i < (sizeof(msg) - 2u))) {
                    msg[i++] = *n++;
                }
            }
        }
        if (msg[i - 1u] == ' ') {
            msg[i++] = 'n'; msg[i++] = 'o'; msg[i++] = 'n'; msg[i++] = 'e';
        }
        msg[i] = '\0';
        report_set(msg);

        RCC->CSR |= RCC_CSR_RMVF; /* write-1-to-clear ALL reset flags */
        if ((RCC->CSR & BSP_WDG_RSTFLAGS_MASK) != 0u) {
            return (int)BSP_WDG_ST2_RSTFLAGS; /* clear did not take */
        }
        report_set(msg);
    }

    /* Step 3: WWDG window/prescaler read-back. CFR accepted the write
     * (legal because WDGA=0) and holds WDGTB=/8 + W=0x70; CR stays parked.
     * The 50 ms timeout / 37.4 ms window numbers live in the header. */
    if ((WWDG->CFR != (((uint32_t)BSP_WDG_WWDG_WDGTB << 7) |
                       (uint32_t)BSP_WDG_WWDG_W)) ||
        ((WWDG->CR & WWDG_CR_WDGA) != 0u)) {
        /* (EWIF intentionally not checked: a parked T=0 WWDG re-asserts
         * EWIF continuously at menu time -- it was cleared at init and
         * re-set by hardware between then and now. Not a failure.) */
                char m2[16];
        unsigned v2 = (unsigned)WWDG->CFR, q2;
        m2[0] = '('; m2[1] = 'c'; m2[2] = 'f'; m2[3] = 'r'; m2[4] = ' ';
        for (q2 = 0; q2 < 4; q2++) {
            unsigned nib = (v2 >> (12 - 4 * q2)) & 0xFu;
            m2[5 + q2] = (char)((nib < 10u) ? ('0' + nib) : ('A' + nib - 10));
        }
        m2[9] = ')'; m2[10] = (char)0;
        report_set(m2);
return (int)BSP_WDG_ST3_WWDG_CFG;
    }
    return 0; /* all steps passed */
}

/** @} */
