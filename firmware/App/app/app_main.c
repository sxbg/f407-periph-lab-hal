/**
 * @file app_main.c
 * @brief Application composition root (HAL teaching-lab edition).
 *
 * Flow: boot_diag -> rtt init -> reset-flag triage -> test menu (never
 * returns). The menu (shell/test_menu.c) owns the console from there;
 * per-peripheral modules register in its registry (W4 batches).
 */
#include "app_main.h"

#include "rtt_log.h"
#include "log_uart.h"
#include "boot_diag.h"
#include "hal_port.h"
#include "ports.h"
#include "adapters.h"
#include "wwdg.h"  /* hwwdg: generator-limit correction below */
#include "rtc.h"   /* hrtc:  generator-limit correction below */
#include "iwdg.h"  /* hiwdg: refresh during the LSE settle wait */
#include "main.h"  /* Error_Handler */

#include "app_config.h"
#include "test_menu.h"

/* ---- composition root ------------------------------------------------------ */
void app_main(void)
{
    /* 1. Liveness + clock triage (bring-up trio). */
    boot_diag_clock_ok(clock_init_with_fallback(),
                       APP_HEARTBEAT_PORT, APP_HEARTBEAT_PIN);
    log_uart_line("APP: boot");

    /* 2. RTT logging: post-mortem recovery of the previous run's tail. */
    rtt_log_init(adapter_time()->tick_ms);
    {
        uint32_t rst = RCC->CSR;
        uint16_t is_wdg = (rst & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF)) ? 1u : 0u;
        if (is_wdg != 0u) {
            rtt_ev(RTT_EV_WATCHDOG_RESET, is_wdg, 0u, 0u);
        }
        rtt_ev(RTT_EV_BOOT, is_wdg, SystemCoreClock, 0u);
        rtt_ring_recover_emit(); /* replay previous crash tail as R-lines */
        __HAL_RCC_CLEAR_RESET_FLAGS();
    }

    /* 2b. Generator-limit corrections (knowledge #17). Watchdogs are FULLY
     * DISABLED for the debugging phase (user decision 2026-11-22): the
     * MX_IWDG_Init / MX_WWDG_Init wrappers short-circuit in their USER
     * CODE 0 regions, and no re-arm happens here. RTC clock source: the
     * .ioc loader regenerates it as LSI -- re-select LSE non-fatally with
     * a bounded crystal settle wait. */
    {
        RCC_PeriphCLKInitTypeDef rtc_clk;
        uint32_t t0 = HAL_GetTick();

        __HAL_RCC_LSE_CONFIG(RCC_LSE_ON);
        while (!__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY)) {
            if ((HAL_GetTick() - t0) > 2000u) {
                break; /* LSE did not start: RTC stays on LSI, report says so */
            }
        }
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY)) {
            rtc_clk.PeriphClockSelection = RCC_PERIPHCLK_RTC;
            rtc_clk.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
            if ((HAL_RCCEx_PeriphCLKConfig(&rtc_clk) == HAL_OK) &&
                (HAL_RTC_DeInit(&hrtc) == HAL_OK) &&
                (HAL_RTC_Init(&hrtc) == HAL_OK)) {
                rtt_ev(RTT_EV_BOOT, 2u, 32768u, 0u); /* marker: LSE re-armed */
            }
        }
    }

    /* 3. Test menu: never returns (RX polling IS the main loop). */
    test_menu_run();
}
