/**
 * @file bsp_rtc.h
 * @brief RTC calendar + backup domain (HAL edition of the register-level
 *        bsp_rtc) -- verify-only init, clock/alarm/backup APIs, self-test.
 *
 * Teaching module (HAL edition of f407-periph-lab bsp_rtc; the backup-
 * domain walkthrough -- PWR DBP / BDCR RTCSEL / WPR lock layers, the
 * PRER two-write law, shadow-register tearing -- stays in the old
 * project's file header as the companion reference).
 *
 * OWNERSHIP CHANGE vs the register edition: the RTC IS in the .ioc now.
 * MX_RTC_Init (Core/Src/rtc.c) owns the whole backup-domain bring-up
 * chain that the register edition taught: HAL_PWR_EnableBkUpAccess (DBP)
 * + clock-source selection + LSE/LSI start live in HAL_RTC_MspInit /
 * HAL_RCCEx_PeriphCLKConfig, the 24h/127/255 calendar plan in
 * HAL_RTC_Init, and the boot time/date stamp in the generated SetTime/
 * SetDate calls. This module only VERIFIES the result (read-only
 * probes, A3 exemption) and serves the calendar/alarm/BKP APIs through
 * HAL_RTC_*.
 *
 * CLOCK-SOURCE FACT (this build): Core/Src/rtc.c selects
 * RCC_RTCCLKSOURCE_LSI even though the .ioc field claims LSE -- the
 * generated code is the build truth. Consequences the module absorbs:
 *   - there is NO LSE ready-wait here anymore (the old -2 SKIP path is
 *     gone; LSI needs no crystal warm-up),
 *   - one calendar "second" = 32768 clock cycles / f_LSI, so across the
 *     LSI 17..47 kHz spread it lasts 0.70..1.93 s -- the selftest step-2
 *     window is widened from the register edition's 1.2 s to 2.5 s,
 *   - init reads the LIVE clock source and reports it ("lse" / "lsi"),
 *     so a later .ioc regeneration to LSE needs no code change.
 *
 * ALARM PATH (intentional change): HAL_RTC_SetAlarm (non-IT) only --
 * stm32f4xx_it.c has no RTC_Alarm_IRQHandler to dispatch
 * HAL_RTC_SetAlarm_IT through (Core file, out of scope). The polling
 * lesson is unchanged: watch RTC_FLAG_ALRAF via bsp_rtc_alarm_fired().
 */

#ifndef BSP_RTC_H
#define BSP_RTC_H

#include <stdint.h>

/* 20 x 32-bit backup registers live in the backup domain: powered by VBAT
 * (CR1220 on this board), reset ONLY by VBAT loss / backup-domain reset
 * (BDRST) / tamper -- they survive every system reset, which is exactly
 * what makes them the teaching vehicle for "where did this boot come
 * from" markers (EP05/EP15 reset-source lessons). */
#define BSP_RTC_BKP_COUNT 20u

/* Verify the generated backup-domain plan (A3 read-only probes):
 * handle alive, prescalers 127/255 + 24h format as generated, live RTC
 * clock source is LSE or LSI, PRER register matches the plan.
 * Returns 0 = PASS (report carries the live source),
 *        -1 = mismatch (no handle / plan changed / foreign source). */
int bsp_rtc_init(void);

/* Write the calendar time (decimal inputs, validated). HAL_RTC_SetTime
 * with RTC_FORMAT_BIN does the BCD conversion and the INIT/INITF
 * enter/exit sequence internally. Date is left untouched.
 * Returns 0 = set, -1 = range violation (h<24, m<60, s<60) or the
 * driver failed. */
int bsp_rtc_set_time(uint8_t hh, uint8_t mm, uint8_t ss);

/* Read the calendar time (decimal outputs). HAL_RTC_GetTime + a
 * trailing HAL_RTC_GetDate: the HAL contract locks the shadow registers
 * until the date read completes (RM0090 ch.26.3.16 shadow story -- the
 * register edition's manual double-read consistency check is done by
 * the driver here). Returns 0 = values delivered, -1 = driver error or
 * the range check caught a glitch (h<24, m<60, s<60 re-verified here). */
int bsp_rtc_get_time(uint8_t *hh, uint8_t *mm, uint8_t *ss);

/* Alarm A: fire once per minute when the seconds field == ss (decimal).
 * Date/weekday, hours and minutes are masked (AlarmMask), subseconds
 * masked (AlarmSubSecondMask) -- seconds compared exactly; armed via
 * HAL_RTC_SetAlarm (non-IT, polling lesson, see the header note).
 * Returns 0 = armed, -1 = range violation or the driver failed (its
 * internal ALRAWF wait timed out). */
int bsp_rtc_set_alarm_seconds(uint8_t ss);

/* 1 = alarm A fired since the last call (RTC_FLAG_ALRAF), then the
 * flag is cleared (__HAL_RTC_ALARM_CLEAR_FLAG -- the one register write
 * the contract sanctions on this polling path); 0 = not fired yet.
 * INTENTIONAL CHANGE: the old WPR-error -1 return cannot happen --
 * the HAL manages WPR internally on this path. */
int bsp_rtc_alarm_fired(void);

/* Backup registers 0..19 (see BSP_RTC_BKP_COUNT), via
 * HAL_RTCEx_BKUPWrite/Read. write: 0 = stored, -1 = bad index.
 * read: register content (0 for a bad index). */
int bsp_rtc_write_bkp(uint32_t idx, uint32_t val);
uint32_t bsp_rtc_read_bkp(uint32_t idx);

/* One-line report of the LAST selftest/init run, for the menu to append
 * to its verdict line (no console dependency). Examples:
 * "(rtc src lsi, bkp0 kept)" / "(rtc src lse, bkp0 kept)". */
const char *bsp_rtc_last_report(void);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. Steps (numbering kept from the register edition):
 *   1 = init verification (no LSE wait anymore -- see the header note;
 *       the old -2 SKIP semantics are gone with the LSE wait)
 *   2 = calendar ticks: seconds field must change by exactly +1 inside
 *       a 2.5 s window (widened for the LSI clock spread; window
 *       measured on HAL_GetTick, an independent timekeeper -- same
 *       cross-check discipline as the register edition)
 *   3 = backup registers: BKP0R = 0xA5A55A5A and BKP19R = its
 *       complement written and read back; the values are LEFT in place
 *       so the next power cycle can demonstrate retention (VBAT story) */
int bsp_rtc_selftest(void);

#endif /* BSP_RTC_H */
