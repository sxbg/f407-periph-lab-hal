/**
 ******************************************************************************
 * @file    bsp_rtc.c
 * @brief   RTC calendar + backup domain BSP (HAL edition) -- see
 *          bsp_rtc.h for the contract, the ownership change and the
 *          clock-source fact.
 ******************************************************************************
 * @addtogroup BSP_RTC
 * @{
 *
 * == HAL EDITION MAP (register edition -> here) ===========================
 * PWR->CR DBP unlock + APB1 PWREN            -> HAL_PWR_EnableBkUpAccess
 *                                               inside HAL_RCCEx_PeriphCLK
 *                                               Config (MspInit, boot).
 * BDCR LSEON/LSERDY/RTCSEL/RTCEN             -> HAL_RCCEx_PeriphCLKConfig
 *                                               (this build: RTCCLKSOURCE_
 *                                               LSI despite the .ioc LSE
 *                                               claim -- header note).
 * WPR 0xCA/0x53 key writes                   -> HAL_RTC_* drivers manage
 *                                               the write protection.
 * ISR INIT/INITF + PRER two writes + TR/DR   -> HAL_RTC_Init (MspInit +
 *                                               generated SetTime/SetDate)
 *                                               + HAL_RTC_SetTime.
 * ISR RSF shadow-sync wait + TR double-read  -> HAL_RTC_GetTime contract:
 *                                               shadow locked until the
 *                                               trailing GetDate.
 * CR ALRAE + ALRMAR masked compare + ALRAWF  -> HAL_RTC_SetAlarm (non-IT;
 *                                               disarms, waits ALRAWF,
 *                                               programs, re-arms itself).
 * ISR ALRAF poll + rc_w0 clear               -> __HAL_RTC_ALARM_GET_FLAG
 *                                               (read-only) +
 *                                               __HAL_RTC_ALARM_CLEAR_FLAG
 *                                               (the sanctioned clear).
 * (&RTC->BKP0R)[idx] raw writes              -> HAL_RTCEx_BKUPWrite/Read.
 *
 * == CUBEMX CONFIG (in f407-periph-lab-hal.ioc now) ========================
 * | Field             | Value (generated truth)              | Note
 * |-------------------|--------------------------------------|-------------
 * | RTC               | 24h, Asynch 127, Synch 255           | 1 Hz plan
 * | Clock source      | LSI (generated code) vs LSE (.ioc    | stale-code
 * |                   | field) -- generated code is truth    | note
 * | Time/Date at boot | 00:00:00, Monday 2000-01-01, reset   | every boot
 * |                   | unconditionally (generated behavior) |
 * | NVIC              | no RTC_Alarm_IRQHandler in it.c      | polling
 * | Pins              | none (LSI is internal; PC14/PC15     | LSE unused
 * |                   | free for other uses in this build)   | in code
 * == HOW TO VERIFY =========================================================
 * menu> rtc
 *   PASS rtc (rtc src lsi, bkp0 kept) -- plan verified, ticks, bkp kept.
 * After a real power-off (USB out, CR1220 in): power on and check BKP0R
 * keeps 0xA5A55A5A (step 3 rewrote it, but the debugger sees the old
 * value first -- retention proof from the register edition stands).
 */
#include "bsp_rtc.h"

#include <stddef.h>

#include "stm32f4xx_hal.h"
#include "rtc.h" /* hrtc from MX_RTC_Init (CubeMX-generated) */

/* ---- module constants ----------------------------------------------------- */

/* 1 Hz plan as generated (LSI build note: 32768 cycles per second-tick,
 * so one calendar second lasts 0.70..1.93 s across the LSI 17..47 kHz
 * spread -- the law is fixed-cycle, not fixed-time). */
#define BSP_RTC_PREDIV_A 127u
#define BSP_RTC_PREDIV_S 255u

/* Self-test step numbers (the value bsp_rtc_selftest returns). */
#define BSP_RTC_ST1_INIT_LSE 1u /* backup-domain plan verification */
#define BSP_RTC_ST2_SECONDS  2u /* seconds field +1 in the window  */
#define BSP_RTC_ST3_BKP      3u /* BKP0/19 write-read, values kept */

/* Step-2 window: the +1 tick must land inside this HAL_GetTick window.
 * Widened from the register edition's 1.2 s (LSE assumption) to cover
 * the LSI slow edge: 32768/17000 = 1.93 s per calendar second. */
#define BSP_RTC_SEC_WINDOW_MS 2500u

/* Teaching marker values (task sheet: the magic word 0xA5A55A5A). */
#define BSP_RTC_BKP_MAGIC 0xA5A55A5Au

/* ---- private state ---------------------------------------------------------- */

/* Report of the last init/selftest run (menu detail suffix) + the live
 * RTC clock source seen by the latest init (steps compose it back in). */
static char s_report[40];
static int s_report_valid = 0;
static int s_src_is_lse = 0;

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

int bsp_rtc_init(void)
{
    const uint32_t src = __HAL_RCC_GET_RTC_SOURCE(); /* read-only probe */

    if (hrtc.Instance == NULL) {
        report_set("(rtc: no handle)");
        return -1; /* CubeMX init never ran */
    }
    /* The generated calendar plan is the single truth this module
     * verifies (handle fields + register read-back below). */
    if ((hrtc.Init.AsynchPrediv != BSP_RTC_PREDIV_A) ||
        (hrtc.Init.SynchPrediv != BSP_RTC_PREDIV_S) ||
        (hrtc.Init.HourFormat != RTC_HOURFORMAT_24)) {
        report_set("(rtc: plan mismatch)");
        return -1;
    }
    /* Accept exactly the sources this lab teaches: LSE (crystal) or LSI
     * (RC). 00 = never clocked, 11 = HSE/128 -- neither is a state this
     * firmware can produce, so treat both as foreign config. */
    if ((src != RCC_RTCCLKSOURCE_LSE) && (src != RCC_RTCCLKSOURCE_LSI)) {
        report_set("(rtc: src not LSE/LSI)");
        return -1;
    }
    /* Register read-back (read-only, A3 exemption): PRER accepted the
     * generated 127/255 plan -- the shadow story (RSF) is the driver's
     * business. */
    if (RTC->PRER != ((BSP_RTC_PREDIV_A << 16) | BSP_RTC_PREDIV_S)) {
        report_set("(rtc: PRER mismatch)");
        return -1;
    }
    s_src_is_lse = (src == RCC_RTCCLKSOURCE_LSE) ? 1 : 0;
    report_set(s_src_is_lse != 0 ? "(rtc src lse)" : "(rtc src lsi)");
    return 0;
}

int bsp_rtc_set_time(uint8_t hh, uint8_t mm, uint8_t ss)
{
    RTC_TimeTypeDef t;

    if ((hh >= 24u) || (mm >= 60u) || (ss >= 60u)) {
        return -1;
    }
    t.Hours = hh;
    t.Minutes = mm;
    t.Seconds = ss;
    t.TimeFormat = RTC_HOURFORMAT12_AM; /* ignored: 24h format */
    t.SubSeconds = 0u;
    t.SecondFraction = 0u;
    t.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    t.StoreOperation = RTC_STOREOPERATION_RESET;
    /* Driver-internal: WPR unlock, INIT=1, INITF wait, BCD encode, TR
     * write, INIT=0 -- the register edition's ch.26.3.13 sequence. */
    if (HAL_RTC_SetTime(&hrtc, &t, RTC_FORMAT_BIN) != HAL_OK) {
        return -1;
    }
    return 0;
}

int bsp_rtc_get_time(uint8_t *hh, uint8_t *mm, uint8_t *ss)
{
    RTC_TimeTypeDef t;
    RTC_DateTypeDef d;

    if ((hh == 0) || (mm == 0) || (ss == 0)) {
        return -1;
    }
    if (HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN) != HAL_OK) {
        return -1;
    }
    /* HAL contract: a GetDate call must follow GetTime, or the shadow
     * registers stay locked against the next read (RM0090 ch.26.3.16;
     * the driver performs the consistency dance internally). Date is
     * irrelevant for this API -- read and drop. */
    (void)HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN);
    /* Glitch guard kept from the register edition: a torn or corrupt
     * shadow read cannot hide behind the driver. */
    if ((t.Hours >= 24u) || (t.Minutes >= 60u) || (t.Seconds >= 60u)) {
        return -1;
    }
    *hh = t.Hours;
    *mm = t.Minutes;
    *ss = t.Seconds;
    return 0;
}

int bsp_rtc_set_alarm_seconds(uint8_t ss)
{
    RTC_AlarmTypeDef a;

    if (ss >= 60u) {
        return -1;
    }
    a.Alarm = RTC_ALARM_A;
    a.AlarmTime.Hours = 0u;
    a.AlarmTime.Minutes = 0u;
    a.AlarmTime.Seconds = ss;
    a.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM; /* ignored: 24h format */
    a.AlarmTime.SubSeconds = 0u;
    a.AlarmTime.SecondFraction = 0u;
    a.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    a.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
    /* Mask date/weekday, hours and minutes; compare seconds exactly.
     * Subsecond comparison fully masked -- "fire once per minute at
     * :ss" regardless of the subsecond phase. */
    a.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_HOURS |
                  RTC_ALARMMASK_MINUTES;
    a.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
    a.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE; /* masked anyway */
    a.AlarmDateWeekDay = 1u;
    /* NON-IT on purpose (header note): no RTC_Alarm_IRQHandler exists in
     * stm32f4xx_it.c. The driver disarms, waits ALRAWF, programs ALRMAR
     * and re-arms ALRAE; ALRAF then polls via bsp_rtc_alarm_fired(). */
    if (HAL_RTC_SetAlarm(&hrtc, &a, RTC_FORMAT_BIN) != HAL_OK) {
        return -1; /* driver ALRAWF wait timed out */
    }
    return 0;
}

int bsp_rtc_alarm_fired(void)
{
    /* Read-only flag poll (A3 exemption; this HAL vintage has no
     * HAL_RTC_GetFlagStatus function, so the documented read-only macro
     * is the HAL-blessed path). */
    if (__HAL_RTC_ALARM_GET_FLAG(&hrtc, RTC_FLAG_ALRAF) != 0U) {
        /* The clear write is the one register access the contract
         * sanctions on this polling path (ISR is rc_w0 -- writing the
         * bit back to 0 clears it). */
        __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);
        return 1;
    }
    return 0;
}

int bsp_rtc_write_bkp(uint32_t idx, uint32_t val)
{
    if (idx >= BSP_RTC_BKP_COUNT) {
        return -1;
    }
    HAL_RTCEx_BKUPWrite(&hrtc, idx, val); /* driver handles DBP context */
    return 0;
}

uint32_t bsp_rtc_read_bkp(uint32_t idx)
{
    if (idx >= BSP_RTC_BKP_COUNT) {
        return 0u;
    }
    return HAL_RTCEx_BKUPRead(&hrtc, idx);
}

const char *bsp_rtc_last_report(void)
{
    return (s_report_valid != 0) ? s_report : "";
}

int bsp_rtc_selftest(void)
{
    /* Step 1: backup-domain plan verification. No LSE wait anymore (the
     * generated source needs none) -- the register edition's -2 SKIP
     * semantics are gone with it (header note). */
    if (bsp_rtc_init() != 0) {
        return (int)BSP_RTC_ST1_INIT_LSE;
    }

    /* Step 2: the calendar must actually TICK. Sample the seconds
     * field, then demand the +1 increment inside a 2.5 s HAL_GetTick
     * window (independent timekeeper: HAL tick, NOT the RTC under test
     * -- same cross-check discipline as the register edition; widened
     * for the LSI clock spread, see the header note). */
    {
        uint8_t h0 = 0u, m0 = 0u, s0 = 0u;
        uint8_t h1 = 0u, m1 = 0u, s1 = 0u;
        const uint32_t t0 = HAL_GetTick();
        if (bsp_rtc_get_time(&h0, &m0, &s0) != 0) {
            return (int)BSP_RTC_ST2_SECONDS;
        }
        for (;;) {
            if (bsp_rtc_get_time(&h1, &m1, &s1) == 0) {
                if (s1 != s0) {
                    break;
                }
            }
            if ((HAL_GetTick() - t0) >= BSP_RTC_SEC_WINDOW_MS) {
                return (int)BSP_RTC_ST2_SECONDS; /* never advanced */
            }
        }
        /* +1 exactly: 58->59, 59->00 (minutes/hours may also have
         * rolled if we started at :59:59 -- the seconds contract is
         * what the step asserts). */
        {
            const uint8_t expect = (s0 == 59u) ? 0u : (uint8_t)(s0 + 1u);
            if (s1 != expect) {
                return (int)BSP_RTC_ST2_SECONDS;
            }
        }
    }

    /* Step 3: backup registers. Two cells at both ends of the map: the
     * task-sheet magic 0xA5A55A5A into BKP0R and its complement into
     * BKP19R, read both back. Values deliberately LEFT in place -- the
     * VBAT retention story survives the HAL migration unchanged. */
    if ((bsp_rtc_write_bkp(0u, BSP_RTC_BKP_MAGIC) != 0) ||
        (bsp_rtc_write_bkp(19u, ~BSP_RTC_BKP_MAGIC) != 0) ||
        (bsp_rtc_read_bkp(0u) != BSP_RTC_BKP_MAGIC) ||
        (bsp_rtc_read_bkp(19u) != (~BSP_RTC_BKP_MAGIC))) {
        return (int)BSP_RTC_ST3_BKP;
    }
    report_set(s_src_is_lse != 0 ? "(rtc src lse, bkp0 kept)"
                                 : "(rtc src lsi, bkp0 kept)");
    return 0; /* all steps passed */
}

/** @} */
