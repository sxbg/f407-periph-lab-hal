/**
 * @file bsp_rtc.h
 * @brief RTC calendar + backup domain (LSE, alarm A, 20x BKPxR) --
 *        register-level init, clock/alarm/backup APIs, self-test.
 *
 * Teaching module (P3c wave, bsp-module-spec.md template -- see bsp_uart).
 * Full config table / register walkthrough / verification recipe live in the
 * bsp_rtc.c file header.
 *
 * Ownership rule: the RTC is NOT in the IOC (knowledge #13: adapter-owned
 * peripherals stay out; the RTC would also need LSE/RTCSEL which CubeMX puts
 * in RCC only when RTC is activated). This module is the FULL owner of the
 * backup-domain bring-up chain: PWR->CR DBP, RCC->BDCR (LSEON/LSERDY/
 * RTCSEL/RTCEN), RTC->WPR/TR/DR/CR/ISR/PRER/ALRMAR and the 20 BKP registers.
 * No CubeMX/HAL RTC code can ever conflict.
 */
#ifndef BSP_RTC_H
#define BSP_RTC_H

#include <stdint.h>

/* 20 x 32-bit backup registers live in the backup domain: powered by VBAT
 * (CR1220 on this board), reset ONLY by VBAT loss / backup-domain reset
 * (BDRST) / tamper -- they survive every system reset, which is exactly what
 * makes them the teaching vehicle for "where did this boot come from"
 * markers (EP05/EP15 reset-source lessons). */
#define BSP_RTC_BKP_COUNT 20u

/* Register-level bring-up of the whole backup domain (RM0090 ch.26):
 * PWR DBP unlock -> LSE ON -> LSERDY wait -> RTCSEL=LSE + RTCEN ->
 * calendar init (INIT/INITF, PRER 127/255, TR/DR, RSF).
 * Returns 0 = PASS (prescaler/clock-selection read-back matches),
 *        -1 = config error (RTCSEL not LSE-capable / INITF/RSF timeout),
 *        -2 = LSE never became ready inside the 4 s window (caller decides
 *             SKIP semantics -- on a board with the 32.768 kHz crystal this
 *             is usually "crystal cold-start slower than the window"). */
int bsp_rtc_init(void);

/* Write the calendar time (decimal inputs, validated, converted to BCD).
 * Runs the RM init sequence against TR only (INIT=1 -> INITF -> TR -> INIT=0).
 * Returns 0 = set, -1 = range violation (h<24, m<60, s<60) or INITF timeout. */
int bsp_rtc_set_time(uint8_t hh, uint8_t mm, uint8_t ss);

/* Read the calendar time through the shadow registers (BCD -> decimal).
 * Returns 0 = values delivered, -1 = RSF not set yet (calendar never
 * synced) or a non-BCD/glitch value was caught by the range check. */
int bsp_rtc_get_time(uint8_t *hh, uint8_t *mm, uint8_t *ss);

/* Alarm A: fire once per minute when the seconds field == 'ss' (decimal).
 * Date/hours/minutes are masked off (ALRMAR MSK4/MSK3/MSK2), seconds
 * compared exactly; ALRAE enabled, ALRAIE left 0 (polling lesson: watch
 * ISR.ALRAF with bsp_rtc_alarm_fired(), no NVIC line taken).
 * Returns 0 = armed, -1 = range violation or ALRAWF timeout. */
int bsp_rtc_set_alarm_seconds(uint8_t ss);

/* 1 = alarm A fired since the last call (ALRAF), then the flag is cleared;
 * 0 = not fired yet. Returns -1 if the WPR-protected flag region errors. */
int bsp_rtc_alarm_fired(void);

/* Backup registers 0..19 (see BSP_RTC_BKP_COUNT). write: 0 = stored,
 * -1 = bad index. read: register content (0 for a bad index). */
int bsp_rtc_write_bkp(uint32_t idx, uint32_t val);
uint32_t bsp_rtc_read_bkp(uint32_t idx);

/* One-line report of the LAST selftest/init run, for the menu to append to
 * its verdict line (the module itself has no console dependency).
 * Examples: "(lse ok, calendar 1Hz)" / "(lse timeout)" / "(bkp0 kept)". */
const char *bsp_rtc_last_report(void);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. Steps:
 *   1 = backup-domain init: LSE ready wait (4 s window; on timeout the
 *       run reports "(lse timeout)" and PASSES -- SKIP semantics: a slow
 *       crystal is an environment property, not a code defect), then
 *       RTCSEL=LSE / RTCEN / PRER=127:255 read-back
 *   2 = calendar ticks: after init the seconds field must advance by
 *       exactly +1 (BCD, 0x59->0x00 wrap handled) inside a 1.2 s window
 *   3 = backup registers: BKP0R = 0xA5A55A5A and BKP19R = its complement
 *       written and read back; the values are LEFT in place so the next
 *       power cycle can demonstrate retention across a real power-off */
int bsp_rtc_selftest(void);

#endif /* BSP_RTC_H */
