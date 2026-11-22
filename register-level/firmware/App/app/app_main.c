/**
 * @file app_main.c
 * @brief Application composition root + main loop.
 *
 * Flow: boot_diag -> rtt init -> clock check -> port wiring -> UART test
 * menu (never returns). P3 DL-A5: the blink demo loop was REPLACED by
 * test_menu_run(); blink_state.* is retained (see blink_state.h) but is no
 * longer called from here.
 */
#include "app_main.h"

#include "ports.h"
#include "adapters.h"
#include "rtt_log.h"
#include "log_uart.h"
#include "boot_diag.h"
#include "test_menu.h"

#include "app_config.h"

/* ---- main loop ------------------------------------------------------------ */
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

    /* 3. UART test menu = main loop (P3 DL-A5). Registers bsp_uart_init,
     * prints the banner, then polls RX forever: "uart" runs the module
     * selftest, "reboot" triggers NVIC_SystemReset, LED_GREEN toggles as
     * the throttled loop heartbeat, and the RTT #H heartbeat keeps the
     * logcheck liveness probe alive inside the menu loop. */
    test_menu_run(); /* never returns */
}
