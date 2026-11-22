/**
 * @file bsp_pwr.h
 * @brief PWR low-power modes (Sleep / Stop / Standby) + KEY1=PA0 wake path
 *        -- register-level entries, EXTI0 wake wiring, self-test.
 *
 * Teaching module (P3c wave, bsp-module-spec.md template -- see bsp_uart).
 * Full config table / register walkthrough / verification recipe live in the
 * bsp_pwr.c file header.
 *
 * Ownership rule: PWR is NOT in the IOC (knowledge #13: adapter-owned
 * peripherals stay out). This module is the FULL owner of PWR->CR/CSR, the
 * SCB->SCR SLEEPDEEP discipline, the EXTI line-0 wake wiring
 * (SYSCFG->EXTICR[0], EXTI->IMR/RTSR/FTSR/PR + the EXTI0_IRQHandler
 * definition) and the NVIC EXTI0 line. bsp_gpio owns the PA0 PIN config
 * (input + pull-down) -- mode/pull survive Stop, so no GPIO work here.
 *
 * SAFETY CONTRACT (read before calling anything):
 *  - bsp_pwr_selftest() only exercises Sleep (wake = SysTick interrupt,
 *    guaranteed by HAL's 1 ms tick) and register read-backs. It NEVER
 *    enters Stop or Standby.
 *  - Stop is exposed as a menu DEMO command only ('pwr_stop_demo' in
 *    test_menu.c -> bsp_pwr_enter_stop): while asleep the RTT heartbeat
 *    stops (logcheck will call the MCU "hung" -- expected), and if the
 *    KEY1 wake fails the rescue is an SWD reset. Standby is API-only
 *    (bsp_pwr_arm_standby): entering it kills the 1.2 V domain and the
 *    wake is a reset-class event -- by design no menu command runs it.
 */
#ifndef BSP_PWR_H
#define BSP_PWR_H

#include <stdint.h>

/* Bus-clock gates + pre-state check (PWR->CR LPDS/PDDS must be clear,
 * SCB->SCR SLEEPDEEP clear). Returns 0 = PASS, -1 = mismatch. */
int bsp_pwr_init(void);

/* Sleep mode (SLEEPDEEP=0): __WFI in a loop until 'ms' SysTick wraps have
 * been counted; every wrap fires the 1 ms SysTick interrupt, which is the
 * guaranteed wake source (any enabled NVIC interrupt would do). Returns
 * the measured elapsed ms, or BSP_PWR_SLEEP_DEAD_SYSTICK when SysTick
 * looks dead (bounded exit, spec #5). Requires the SysTick tick interrupt
 * to be enabled -- temporarily arms TICKINT if it was off, restores it. */
#define BSP_PWR_SLEEP_DEAD_SYSTICK 0xFFFFFFFFu
uint32_t bsp_pwr_sleep_ms(uint32_t ms);

/* Stop mode entry (PDDS=0, LPDS=1, SLEEPDEEP=1, WFI). Arms EXTI line 0 on
 * PA0 rising edge first -- KEY1 press is the wake source (RM0090 Table:
 * Stop exit needs an EXTI line in interrupt mode with the NVIC vector
 * enabled, which this function sets up, plus the handler in the .c).
 * Returns after the wake (execution resumes in-place); the function then
 * CLEARS SLEEPDEEP and re-runs SystemInit() because Stop exit hands the
 * system clock to HSI 16 MHz with PLL off (EP17 baud-garbage lesson).
 * DEMO-ONLY: prints nothing itself; while asleep every interrupt-driven
 * service (RTT heartbeat, UART RX) is stalled. */
void bsp_pwr_enter_stop(void);

/* Standby arming (PDDS=1 + CWUF=1). The NEXT deepsleep entry with
 * SLEEPDEEP=1 powers the 1.2 V domain DOWN: registers/SRAM lost, wake =
 * reset-class boot. Hardware wake sources: WKUP1 pin rising (PA0 on this
 * board = KEY1, see PWR->CSR EWUP bit8), RTC alarm/wake/tamper, NRST,
 * IWDG reset. NOT called by any selftest or menu demo -- kept as the
 * register-level teaching entry point. */
void bsp_pwr_arm_standby(void);

/* 1 = PWR->CSR SBF set (the last boot was a Standby wake); 0 = no.
 * The flag survives until cleared by writing PWR->CR CSBF=1 (teaching
 * note: pairing this with bsp_pwr_arm_standby() completes the Standby
 * story: arm -> KEY1 press -> reset boot -> SBF=1 here). */
int bsp_pwr_standby_flag(void);

/* Number of EXTI0 (KEY1) wakes counted since boot -- the shell prints it
 * after pwr_stop_demo returns as proof the wake really came through the
 * EXTI path. */
uint32_t bsp_pwr_exti0_wake_count(void);

/* One-line report of the LAST selftest run, for the menu to append to its
 * verdict line (the module itself has no console dependency). Examples:
 * "(cr state ok)" / "(sleep 101 ms via SysTick wake)". */
const char *bsp_pwr_last_report(void);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. SAFE by construction: only Sleep, never Stop/Standby.
 * Steps:
 *   1 = gates + pre-state read-back (PWREN/SYSCFGEN set, LPDS/PDDS/SLEEP-
 *       DEEP all clear -- the "about to deepsleep" bits resting state)
 *   2 = Sleep 100 ms: bsp_pwr_sleep_ms(100) must return an elapsed value
 *       in [100 .. 130] ms, proving WFI + interrupt wake + SysTick time
 *       keeping all work; SLEEPDEEP verified still clear afterwards
 *   (Stop/Standby are deliberately absent -- entering either from a
 *    selftest either strands the console (Stop) or resets the chip
 *    (Standby); they live in demo commands / API only.) */
int bsp_pwr_selftest(void);

#endif /* BSP_PWR_H */
