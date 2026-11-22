/**
 * @file bsp_pwr.h
 * @brief PWR low-power modes (Sleep / Stop / Standby) + KEY1=PA0 wake path
 *        -- HAL_PWR_* entries, EXTI0 wake hook, self-test. (HAL edition of
 *        projects/f407-periph-lab bsp_pwr.)
 *
 * HAL MAPPING NOTE (register edition -> HAL edition):
 *   register edition            | HAL edition
 *   ----------------------------+-------------------------------------------
 *   PWR->CR edits + __WFI       | HAL_PWR_EnterSLEEPMode /
 *                               | HAL_PWR_EnterSTOPMode /
 *                               | HAL_PWR_EnterSTANDBYMode
 *   SCB->SCR SLEEPDEEP writes   | inside the HAL_PWR_* entries (A3: none
 *                               | here; SCB->SCR is only READ, with an
 *                               | exemption comment)
 *   RCC->APBxENR gate writes    | __HAL_RCC_PWR_CLK_ENABLE etc.
 *   PWR->CR CSBF/CWUF writes    | __HAL_PWR_CLEAR_FLAG (HAL macro)
 *   PWR->CSR SBF read           | __HAL_PWR_GET_FLAG(PWR_FLAG_SB)
 *   EXTI->PR stale-pending clear| __HAL_GPIO_EXTI_CLEAR_IT(KEY1_Pin)
 *   SysTick TICKINT arm/restore | HAL_ResumeTick()/HAL_SuspendTick()
 *
 * Ownership rule: PWR is NOT in the IOC (knowledge #13: adapter-owned
 * peripherals stay out). The KEY1=PA0 EXTI0 rising wake IS in the .ioc:
 * PA0 = GPIO_MODE_IT_RISING + pull-down, NVIC EXTI0_IRQn enabled, so
 * CubeMX owns the EXTI line-0 wiring and the NVIC vector and
 * stm32f4xx_it.c owns the EXTI0_IRQHandler -> HAL_GPIO_EXTI_IRQHandler
 * chain. This module owns ONLY the wake COUNTING, through the
 * bsp_pwr_on_exti0() hook below.
 *
 * CALLBACK ROUTING (collision avoidance): HAL_GPIO_EXTI_Callback is a WEAK
 * symbol that both this module (KEY1) and bsp_exti (KEY2) would want to
 * override -- two strong overrides collide at link time. bsp_pwr therefore
 * does NOT define HAL_GPIO_EXTI_Callback; the integrator provides ONE
 * callback (in one place) that routes KEY1_Pin to bsp_pwr_on_exti0().
 *
 * SAFETY CONTRACT (read before calling anything):
 *  - bsp_pwr_selftest() only exercises Sleep (wake = SysTick interrupt,
 *    guaranteed by HAL's 1 ms tick) and flag read-backs. It NEVER enters
 *    Stop or Standby.
 *  - Stop is exposed as a menu DEMO command only ('pwr_stop_demo' in the
 *    shell, re-added by the integrator): while asleep the log heartbeat
 *    stops (logcheck will call the MCU "hung" -- expected), and if the
 *    KEY1 wake fails the rescue is an SWD reset. Standby is API-only
 *    (bsp_pwr_arm_standby): it POWERS DOWN immediately and does not
 *    return -- the wake is a reset-class boot.
 */
#ifndef BSP_PWR_H
#define BSP_PWR_H

#include <stdint.h>

/* EXTI0 (KEY1) wake hook -- the integrator's single HAL_GPIO_EXTI_Callback
 * calls this for KEY1_Pin. One volatile increment: the Stop-wake proof the
 * shell prints after the demo (see bsp_pwr_exti0_wake_count). */
void bsp_pwr_on_exti0(void);

/* Clock gates + pre-state check (PWR->CR LPDS/PDDS must be clear,
 * SCB->SCR SLEEPDEEP clear -- read-only probes, A3 exemption). Returns
 * 0 = PASS, -1 = mismatch. */
int bsp_pwr_init(void);

/* Sleep mode (SLEEPDEEP=0): HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON,
 * PWR_SLEEPENTRY_WFI) in a loop until 'ms' SysTick wraps have been
 * counted; every wrap fires the 1 ms SysTick interrupt, which is the
 * guaranteed wake source (any enabled NVIC interrupt would do). Returns
 * the measured elapsed ms, or BSP_PWR_SLEEP_DEAD_SYSTICK when SysTick
 * looks dead (bounded exit, spec #5). Requires the SysTick tick interrupt
 * to be enabled -- temporarily arms it via HAL_ResumeTick() if it was
 * off, restores it via HAL_SuspendTick(). */
#define BSP_PWR_SLEEP_DEAD_SYSTICK 0xFFFFFFFFu
uint32_t bsp_pwr_sleep_ms(uint32_t ms);

/* Stop mode entry: arms nothing (the .ioc owns EXTI line 0 rising on PA0
 * + NVIC EXTI0), clears a stale EXTI pending bit, then
 * HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI) --
 * HAL sets LPDS=1, PDDS=0, SLEEPDEEP=1 under the hood. KEY1 press is the
 * wake source (Stop exit needs an EXTI line in interrupt mode with the
 * NVIC vector enabled -- exactly what the .ioc configures). Returns after
 * the wake (execution resumes in-place); HAL already cleared SLEEPDEEP on
 * exit, and this function then re-runs SystemInit() because Stop exit
 * hands the system clock to HSI 16 MHz with PLL off (EP17 baud-garbage
 * lesson). DEMO-ONLY: prints nothing itself; while asleep every
 * interrupt-driven service (log heartbeat, UART RX) is stalled. */
void bsp_pwr_enter_stop(void);

/* Standby entry -- HAL semantics (DIFFERENT from the register edition's
 * arm-only API): HAL has no arm-only call, HAL_PWR_EnterSTANDBYMode() is
 * arm + entry ATOMIC (PDDS=1, WUF cleared, SLEEPDEEP=1, WFI). This
 * function powers the 1.2 V domain DOWN and DOES NOT RETURN: registers/
 * SRAM lost, wake = reset-class boot (SP/PC from the reset vector).
 * Hardware wake sources: WKUP1 pin rising (PA0 on this board = KEY1),
 * RTC alarm/wake/tamper, NRST, IWDG reset. NOT called by any selftest or
 * menu demo -- kept as the API-only teaching entry point. */
void bsp_pwr_arm_standby(void);

/* 1 = Standby flag set (the last boot was a Standby wake), 0 = no --
 * __HAL_PWR_GET_FLAG(PWR_FLAG_SB), read-only. The flag survives until
 * cleared by __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB) (CR CSBF=1 under the
 * hood; pairing that with bsp_pwr_arm_standby() completes the Standby
 * story: enter -> KEY1 press -> reset boot -> flag set here). */
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
 *   1 = gates + pre-state read-back (PWR/SYSCFG clocks on, LPDS/PDDS/
 *       SLEEPDEEP all clear -- the "about to deepsleep" bits resting
 *       state)
 *   2 = Sleep 100 ms: bsp_pwr_sleep_ms(100) must return an elapsed value
 *       in [100 .. 130] ms, proving HAL_PWR sleep entry + interrupt wake
 *       + SysTick time keeping all work; SLEEPDEEP verified still clear
 *       afterwards
 *   (Stop/Standby are deliberately absent -- entering either from a
 *    selftest either strands the console (Stop) or resets the chip
 *    (Standby); they live in demo commands / API only.) */
int bsp_pwr_selftest(void);

#endif /* BSP_PWR_H */
