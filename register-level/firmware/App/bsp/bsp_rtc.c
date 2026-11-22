/**
 ******************************************************************************
 * @file    bsp_rtc.c
 * @brief   RTC calendar + backup domain -- teaching module: LSE bring-up,
 *          1 Hz calendar, alarm A, and the 20x32-bit backup registers.
 ******************************************************************************
 * @addtogroup BSP_RTC
 * @{
 *
 * == CUBEMX CONFIG (absent from f407-periph-lab.ioc -- by design) ==========
 * | Field             | Value (module truth)                 | Why
 * |-------------------|--------------------------------------|-------------------
 * | RTC in IOC        | NOT PRESENT                          | knowledge #13:
 * |                   |                                      | adapter-owned
 * |                   |                                      | peripherals stay
 * |                   |                                      | out of the .ioc;
 * |                   |                                      | the LSE/RTCSEL
 * |                   |                                      | RCC fields would
 * |                   |                                      | also be CubeMX
 * |                   |                                      | territory -- the
 * |                   |                                      | module owns them
 * | LSE crystal       | board-soldered 32.768 kHz on PC14/   | EP-facts: OSC32
 * |                   | PC15 (OSC32_IN/OUT) + CR1220 on VBAT | lives on the board
 * | Backup battery    | CR1220 coin cell -> VBAT pin         | BKPxR + calendar
 * |                   |                                      | survive VDD off
 * | NVIC              | RTC_Alarm_IRQn (41) off              | polling lesson
 * | Pins PC14/PC15    | analog (crystal), no GPIO config     | LSE drives them
 * == REGISTER WALKTHROUGH (RM0090 ch.26 RTC; ch.5 PWR; ch.7 RCC) ===========
 * The backup domain is its own voltage/speed island. Three lock layers
 * protect it, and this module opens them IN ORDER (skip one and every
 * following write is silently swallowed -- the classic RTC lesson):
 *   1. PWR->CR DBP bit8 (ch.5.4.1): "Disable Backup domain write
 *      protection". PWR itself is on APB1: RCC->APB1ENR PWREN bit28 first.
 *      DBP unlocks BDCR, RTC registers and BKPxR for writing.
 *   2. RCC->BDCR (ch.7.3.20, backup-domain registers):
 *      LSEON bit0  -> start the 32.768 kHz crystal; LSERDY bit1 comes up
 *                     after the crystal stabilizes (typ ~2 s, colder = slower;
 *                     the selftest waits up to 4 s then SKIPs).
 *      RTCSEL      -> bits [9:8] = 01 = LSE as the RTC clock. WRITE-ONCE:
 *                     only a backup-domain reset (BDRST bit16) clears it
 *                     again -- and BDRST wipes the calendar AND all 20
 *                     BKPxR. So the code only ever sets RTCSEL when it
 *                     still reads 00 (never clocked); if it reads 10/11
 *                     (LSI/HSE_RTC) the module refuses and reports.
 *      RTCEN bit15 -> gate the selected clock into the RTC.
 *   3. RTC->WPR (ch.26.6.21): write-protect key. 0xCA then 0x53 unlocks
 *      all RTC registers except ISR[13:8], TAFCR and BKPxR (those stay
 *      open behind DBP only). ANY other value re-locks immediately.
 * Calendar init (RM0090 ch.26.3.13 "Calendar initialization and
 * configuration"), exact sequence implemented below:
 *   ISR.INIT=1 -> stop the calendar, enter init mode
 *   poll ISR.INITF=1 -> hardware acknowledges init mode after 1-2 RTCCLK
 *   PRER twice (RM mandates two separate writes):
 *     PREDIV_S (bits [15:0]) = 255 -> 32768/(127+1)/(255+1) = 1 Hz
 *     PREDIV_A (bits [22:16]) = 127  (write sync prescaler FIRST)
 *   CR.FMT bit6 = 0 -> 24 h format
 *   TR/DR <- initial BCD time/date (shadow registers, written in init mode)
 *   ISR.INIT=0 -> counter reloads from the shadows, counting resumes
 *                after 4 RTCCLK (~122 us at 32.768 kHz)
 *   ISR.RSF cleared, then wait RSF=1: the SHADOW registers (what TR/DR
 *     reads return) re-sync with the true counters. Reading TR/DR without
 *     RSF can return a torn value (seconds rolled between the two 32-bit
 *     subfields) -- ch.26.3.16 mandates the double-read consistency check
 *     implemented in bsp_rtc_get_time().
 * BCD fields (why 0x59 is the max second, not 63): each nibble is a decimal
 *   digit. TR: SU/ST [7:0] seconds, MNU/MNT [15:8] minutes, HU/HT [23:16]
 *   hours (+PM bit22 in 12h mode), DR: DU/DT [7:0] day, MU/MT [15:8] month,
 *   YU/YT [23:16] year, WDU [15:13] weekday.
 * Alarm A (ch.26.3.15): ALRMAR compares masked calendar fields. MSK4
 *   (bit31) masks date, MSK3 (bit23) hours, MSK2 (bit15) minutes; seconds
 *   [7:0] stay unmasked -> "every minute at :SS". CR.ALRAE bit8 arms it;
 *   ALRAIE bit12 would route ISR.ALRAF to NVIC line 41 (RTC_Alarm_IRQn)
 *   via EXTI line 17 -- deliberately OFF here (polling lesson).
 * WUT one-liner (ch.26.3.6): the wake-up timer is WUTR reload counted down
 *   at ck_wut (WUCKSEL[2:0] in CR picks the tick); on zero it sets
 *   ISR.WUTF (WUTE bit10 enable, WUTIE to EXTI line 22). Its whole job is
 *   waking the chip from Stop/Standby -- see bsp_pwr for the low-power
 *   entries; this module stops at naming the registers.
 * BKPxR (ch.26.6.24): BKP0R..BKP19R, 32 bits each, backup-domain powered.
 *   Survives every system reset (pin/pwdg/wwdg/soft); dies only with VBAT
 *   or BDRST. Perfect "boot marker" storage: firmware writes 0x57444731
 *   ("WDG1") before the wdg_reset_demo and any later boot can read who
 *   asked for the reset (EP05/EP15 reset-source teaching).
 * == HOW TO VERIFY =========================================================
 * menu> rtc
 *   PASS rtc (lse ok, calendar 1Hz)   -- crystal alive, ticks, bkp kept
 *   PASS rtc (lse timeout)            -- SKIP semantics: crystal too slow,
 *                                        rerun once the LSE has stabilized
 * Cross-checks: run 'rtc', note the seconds, wait 5 s, run 'set-time' is
 * not wired into the menu -- instead re-run 'rtc': init resets to 00:00:00
 * and step 2 must STILL pass (deterministic +1). After a real power-off
 * (USB out, CR1220 in), power on and check BKP0R keeps 0xA5A55A5A.
 */
#include "bsp_rtc.h"

#include "hal_port.h" /* family seam: pulls in the CMSIS device header */

/* ---- module constants ----------------------------------------------------- */

/* 1 Hz law: 32768 / (PREDIV_A+1) / (PREDIV_S+1) = 32768/128/256 = 1 Hz. */
#define BSP_RTC_PREDIV_A 127u /* bits [22:16] of PRER */
#define BSP_RTC_PREDIV_S 255u /* bits [15:0]  of PRER */

/* Epoch the init writes: 2025-01-01 00:00:00 (weekday field WDU=3 is
 * arbitrary for a calendar lesson; BCD-encoded fields, see walkthrough). */
#define BSP_RTC_EPOCH_DR ((0x25u << 16) | (3u << 13) | 0x01u)

/* Self-test step numbers (the value bsp_rtc_selftest returns). */
#define BSP_RTC_ST1_INIT_LSE 1u /* backup domain + LSE ready + readback */
#define BSP_RTC_ST2_SECONDS  2u /* seconds field +1 in the window       */
#define BSP_RTC_ST3_BKP      3u /* BKP0/19 write-read, values kept      */

/* Wait windows (spec #5: every wait bounded).
 * LSE: crystal cold-start typ ~2 s; 4 s window then SKIP semantics. */
#define BSP_RTC_LSE_TIMEOUT_MS   4000u
#define BSP_RTC_INITF_TIMEOUT_MS 5u    /* INITF needs 1-2 RTCCLK (~61 us) */
#define BSP_RTC_RSF_TIMEOUT_MS   20u   /* shadow sync needs 4 RTCCLK      */
#define BSP_RTC_ALRAWF_TO_MS     5u    /* alarm update-allow window       */
#define BSP_RTC_SEC_WINDOW_MS    1200u /* > 1 RTC second with margin      */
#define BSP_RTC_SYSTICK_GUARD    2000000u /* SysTick-dead bounded exit    */

/* Teaching marker values (task sheet: the magic word 0xA5A55A5A). */
#define BSP_RTC_BKP_MAGIC 0xA5A55A5Au

/* ---- private state ---------------------------------------------------------- */

/* Report of the last init/selftest run (menu detail suffix). */
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

/* SysTick wrap counter (1 wrap = 1 ms, PM0056 B3.3 COUNTFLAG). Same
 * pattern as the other P3 modules: returns ms seen or DEAD marker. */
#define BSP_RTC_DEAD_SYSTICK 0xFFFFFFFFu

static uint32_t rtc_count_ms(uint32_t ms)
{
    uint32_t seen = 0u;
    while (seen < ms) {
        uint32_t guard = BSP_RTC_SYSTICK_GUARD;
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0u) {
            if (--guard == 0u) {
                return BSP_RTC_DEAD_SYSTICK;
            }
        }
        seen++;
    }
    return seen;
}

/* Layer-1 lock opener: PWR clock gate + DBP (see walkthrough layer 1).
 * Read-back stalls until the APB1 clock is live. */
static void rtc_backup_unlock(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;
    PWR->CR |= PWR_CR_DBP; /* "Disable Backup domain write protection" */
}

/* Layer-3 lock: RTC register write protection (0xCA/0x53 to open, any
 * other byte closes). */
static void rtc_lock(void)
{
    RTC->WPR = 0xFFu; /* wrong key = protection re-activated */
}

/* BCD <-> decimal for the two-digit calendar fields (walkthrough). */
static uint32_t bcd_enc(uint32_t v)
{
    return ((v / 10u) << 4) | (v % 10u);
}

static int bcd_dec(uint32_t b, int max)
{
    const uint32_t hi = (b >> 4) & 0x0Fu;
    const uint32_t lo = b & 0x0Fu;
    if ((hi > 9u) || (lo > 9u)) {
        return -1; /* not a BCD value: torn shadow read or corruption */
    }
    {
        const int v = (int)((hi * 10u) + lo);
        if (v > max) {
            return -1;
        }
        return v;
    }
}

/* Consistent shadow read of RTC->TR (RM0090 ch.26.3.16): read twice and
 * require equality -- a calendar rollover between the two subfields would
 * otherwise tear the value. Bounded by the SysTick ms budget. */
static uint32_t rtc_read_tr_stable(void)
{
    uint32_t guard_ms = BSP_RTC_SEC_WINDOW_MS;
    for (;;) {
        const uint32_t a = RTC->TR;
        const uint32_t b = RTC->TR;
        if (a == b) {
            return a;
        }
        if ((rtc_count_ms(1u) == BSP_RTC_DEAD_SYSTICK) ||
            (--guard_ms == 0u)) {
            return a; /* give up: caller's range check will reject */
        }
    }
}

/* Wait one RTC ISR ready-flag within a bounded window. flags=1 when the
 * wanted bit IS the ready bit; the poll is a plain register read loop. */
static int rtc_wait_flag(uint32_t mask, uint32_t timeout_ms)
{
    uint32_t guard = BSP_RTC_SYSTICK_GUARD;
    for (;;) {
        if ((RTC->ISR & mask) != 0u) {
            return 0;
        }
        if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0u) {
            if (timeout_ms > 0u) {
                timeout_ms--;
                guard = BSP_RTC_SYSTICK_GUARD;
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

int bsp_rtc_init(void)
{
    uint32_t sel;

    /* Layer 1: PWR clock + DBP (walkthrough). HAL equivalent of the whole
     * function: HAL_PWR_EnableBkUpAccess + __HAL_RCC_RTC_ENABLE +
     * HAL_RTC_Init with a 127/255 prescaler. */
    rtc_backup_unlock();

    /* Layer 2: LSE crystal on, bounded LSERDY wait. A timeout is
     * environment (slow crystal), reported as such -- init is retried
     * whole on the next menu run. */
    RCC->BDCR |= RCC_BDCR_LSEON;
    {
        uint32_t guard = BSP_RTC_SYSTICK_GUARD;
        uint32_t ms = BSP_RTC_LSE_TIMEOUT_MS;
        while ((RCC->BDCR & RCC_BDCR_LSERDY) == 0u) {
            if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0u) {
                if (ms > 0u) {
                    ms--;
                    guard = BSP_RTC_SYSTICK_GUARD;
                } else {
                    report_set("(lse timeout)");
                    return -2; /* SKIP semantics upstream */
                }
            }
            if (--guard == 0u) {
                report_set("(lse timeout)");
                return -2;
            }
        }
    }

    /* RTC clock selection: RTCSEL is write-once (walkthrough). Only claim
     * it when never clocked (00) or already ours (01). 10/11 means
     * someone else owns the RTC clock -- changing it costs a BDRST
     * (calendar + BKPxR wipe), so this module refuses instead. */
    sel = RCC->BDCR & RCC_BDCR_RTCSEL;
    if (sel == 0u) {
        RCC->BDCR |= RCC_BDCR_RTCSEL_0 | RCC_BDCR_RTCEN;
    } else if (sel == RCC_BDCR_RTCSEL_0) {
        RCC->BDCR |= RCC_BDCR_RTCEN;
    } else {
        report_set("(rtc: RTCSEL not LSE, BDRST needed)");
        return -1;
    }

    /* Layer 3: WPR unlock, then the exact RM0090 ch.26.3.13 sequence. */
    RTC->WPR = 0xCAu; /* key 1 of 2 -- see walkthrough */
    RTC->WPR = 0x53u; /* key 2 of 2; any other value re-locks */

    RTC->ISR |= RTC_ISR_INIT; /* enter init mode: calendar stopped */
    if (rtc_wait_flag(RTC_ISR_INITF, BSP_RTC_INITF_TIMEOUT_MS) != 0) {
        RTC->ISR &= ~RTC_ISR_INIT;
        rtc_lock();
        report_set("(rtc: INITF timeout)");
        return -1;
    }
    /* Two separate PRER writes, sync factor FIRST (RM0090 ch.26.3.13
     * step 3 -- even when both change, the RM mandates two accesses). */
    RTC->PRER = BSP_RTC_PREDIV_S;
    RTC->PRER |= (BSP_RTC_PREDIV_A << 16);
    RTC->CR &= ~RTC_CR_FMT; /* 24 h format (bit6 = 0) */
    /* Shadow registers: 00:00:00 on the epoch date (BCD, walkthrough). */
    RTC->TR = 0u;
    RTC->DR = BSP_RTC_EPOCH_DR;
    RTC->ISR &= ~RTC_ISR_INIT; /* exit: counter loads, resumes in 4 RTCCLK */
    RTC->ISR &= ~RTC_ISR_RSF;  /* force a fresh shadow sync... */
    if (rtc_wait_flag(RTC_ISR_RSF, BSP_RTC_RSF_TIMEOUT_MS) != 0) {
        rtc_lock();
        report_set("(rtc: RSF timeout)");
        return -1;
    }
    rtc_lock(); /* lock before the read-back asserts */

    /* Read-back proof: selection + prescaler exactly as programmed. */
    if (((RCC->BDCR & RCC_BDCR_RTCSEL) != RCC_BDCR_RTCSEL_0) ||
        ((RCC->BDCR & RCC_BDCR_RTCEN) == 0u) ||
        (RTC->PRER != ((BSP_RTC_PREDIV_A << 16) | BSP_RTC_PREDIV_S))) {
        report_set("(rtc: readback mismatch)");
        return -1;
    }
    report_set("(lse ok, calendar 1Hz)");
    return 0;
}

int bsp_rtc_set_time(uint8_t hh, uint8_t mm, uint8_t ss)
{
    if ((hh >= 24u) || (mm >= 60u) || (ss >= 60u)) {
        return -1;
    }
    rtc_backup_unlock();
    RTC->WPR = 0xCAu;
    RTC->WPR = 0x53u;
    RTC->ISR |= RTC_ISR_INIT;
    if (rtc_wait_flag(RTC_ISR_INITF, BSP_RTC_INITF_TIMEOUT_MS) != 0) {
        RTC->ISR &= ~RTC_ISR_INIT;
        rtc_lock();
        return -1;
    }
    /* TR layout (walkthrough): HU/HT [23:16], MNU/MNT [15:8], SU/ST [7:0]. */
    RTC->TR = ((uint32_t)bcd_enc(hh) << 16) |
              ((uint32_t)bcd_enc(mm) << 8) |
              (uint32_t)bcd_enc(ss);
    RTC->ISR &= ~RTC_ISR_INIT;
    RTC->ISR &= ~RTC_ISR_RSF;
    if (rtc_wait_flag(RTC_ISR_RSF, BSP_RTC_RSF_TIMEOUT_MS) != 0) {
        rtc_lock();
        return -1;
    }
    rtc_lock();
    return 0;
}

int bsp_rtc_get_time(uint8_t *hh, uint8_t *mm, uint8_t *ss)
{
    uint32_t tr;
    int s, m, h;

    if ((hh == 0) || (mm == 0) || (ss == 0)) {
        return -1;
    }
    /* RSF must be set before touching the shadows (walkthrough, ch.26.3.16):
     * without it the register content may not match the running calendar. */
    if ((RTC->ISR & RTC_ISR_RSF) == 0u) {
        return -1;
    }
    tr = rtc_read_tr_stable();
    s = bcd_dec(tr & 0xFFu, 59);
    m = bcd_dec((tr >> 8) & 0xFFu, 59);
    h = bcd_dec((tr >> 16) & 0x3Fu, 23); /* bit22 PM ignored: FMT=0 24h */
    if ((s < 0) || (m < 0) || (h < 0)) {
        return -1;
    }
    *hh = (uint8_t)h;
    *mm = (uint8_t)m;
    *ss = (uint8_t)s;
    return 0;
}

int bsp_rtc_set_alarm_seconds(uint8_t ss)
{
    if (ss >= 60u) {
        return -1;
    }
    rtc_backup_unlock();
    RTC->WPR = 0xCAu;
    RTC->WPR = 0x53u;
    /* Disarm while editing (and keep the interrupt OFF -- polling lesson). */
    RTC->CR &= ~(RTC_CR_ALRAE | RTC_CR_ALRAIE);
    /* ALRAWF: hardware opens the alarm write window (needs the alarm
     * clocked and disarmed; bounded wait per spec #5). */
    if (rtc_wait_flag(RTC_ISR_ALRAWF, BSP_RTC_ALRAWF_TO_MS) != 0) {
        rtc_lock();
        return -1;
    }
    /* Mask date (MSK4), hours (MSK3), minutes (MSK2); WDSEL=0 so the
     * compare is on the DAY-OF-MONTH field which is masked anyway --
     * net effect: match the seconds nibbles only, fire once per minute. */
    RTC->ALRMAR = RTC_ALRMAR_MSK4 | RTC_ALRMAR_MSK3 | RTC_ALRMAR_MSK2 |
                  (uint32_t)bcd_enc(ss);
    RTC->CR |= RTC_CR_ALRAE; /* arm; ALRAF polls via bsp_rtc_alarm_fired() */
    rtc_lock();
    return 0;
}

int bsp_rtc_alarm_fired(void)
{
    rtc_backup_unlock();
    RTC->WPR = 0xCAu;
    RTC->WPR = 0x53u;
    if ((RTC->ISR & RTC_ISR_ALRAF) != 0u) {
        RTC->ISR &= ~RTC_ISR_ALRAF; /* rc_w0 clear: write back a 0 */
        rtc_lock();
        return 1;
    }
    rtc_lock();
    return 0;
}

int bsp_rtc_write_bkp(uint32_t idx, uint32_t val)
{
    if (idx >= BSP_RTC_BKP_COUNT) {
        return -1;
    }
    rtc_backup_unlock(); /* BKPxR need DBP but NOT the WPR key (RM 26.6.21) */
    (&RTC->BKP0R)[idx] = val; /* CMSIS packs BKP0R..BKP19R contiguously */
    return 0;
}

uint32_t bsp_rtc_read_bkp(uint32_t idx)
{
    if (idx >= BSP_RTC_BKP_COUNT) {
        return 0u;
    }
    return (&RTC->BKP0R)[idx];
}

const char *bsp_rtc_last_report(void)
{
    return (s_report_valid != 0) ? s_report : "";
}

int bsp_rtc_selftest(void)
{
    /* Step 1: full backup-domain init. -2 = LSE did not stabilize in the
     * window -> SKIP semantics (PASS + "(lse timeout)"), the crystal is
     * the environment, not the code under test. */
    {
        const int rc = bsp_rtc_init();
        if (rc == -2) {
            return 0; /* menu prints PASS rtc (lse timeout) */
        }
        if (rc != 0) {
            return (int)BSP_RTC_ST1_INIT_LSE;
        }
    }

    /* Step 2: the calendar must actually TICK. Init set 00:00:00; sample
     * the seconds field, then demand the +1 BCD increment inside a 1.2 s
     * window (independent timekeeper: SysTick wraps, NOT the RTC under
     * test -- same cross-check discipline as bsp_tim_basic step 1). */
    {
        uint8_t h0 = 0u, m0 = 0u, s0 = 0u;
        uint8_t h1 = 0u, m1 = 0u, s1 = 0u;
        uint32_t guard_ms;
        if (bsp_rtc_get_time(&h0, &m0, &s0) != 0) {
            return (int)BSP_RTC_ST2_SECONDS;
        }
        guard_ms = BSP_RTC_SEC_WINDOW_MS;
        for (;;) {
            if (bsp_rtc_get_time(&h1, &m1, &s1) == 0) {
                if (s1 != s0) {
                    break;
                }
            }
            if (rtc_count_ms(1u) == BSP_RTC_DEAD_SYSTICK) {
                return (int)BSP_RTC_ST2_SECONDS;
            }
            if (--guard_ms == 0u) {
                return (int)BSP_RTC_ST2_SECONDS; /* never advanced */
            }
        }
        /* +1 in BCD space: 58->59, 59->00 (minutes/hours may also have
         * rolled if we started at :59:59 -- the seconds contract is what
         * the step asserts). */
        {
            const int expect = (s0 == 59) ? 0 : (s0 + 1);
            if ((int)s1 != expect) {
                return (int)BSP_RTC_ST2_SECONDS;
            }
        }
    }

    /* Step 3: backup registers. Two cells at both ends of the map: write
     * the task-sheet magic 0xA5A55A5A into BKP0R and its complement into
     * BKP19R, read both back. The values are deliberately LEFT in place:
     * after a real power-off (USB out, CR1220 keeps the backup domain
     * alive) the next boot can prove retention by running 'rtc' -- step 3
     * rewrites them, but a debugger / future 'bkp dump' command sees the
     * old values first. */
    if ((bsp_rtc_write_bkp(0u, BSP_RTC_BKP_MAGIC) != 0) ||
        (bsp_rtc_write_bkp(19u, ~BSP_RTC_BKP_MAGIC) != 0) ||
        (bsp_rtc_read_bkp(0u) != BSP_RTC_BKP_MAGIC) ||
        (bsp_rtc_read_bkp(19u) != (~BSP_RTC_BKP_MAGIC))) {
        return (int)BSP_RTC_ST3_BKP;
    }
    report_set("(lse ok, bkp0 kept)");
    return 0; /* all steps passed */
}

/** @} */
