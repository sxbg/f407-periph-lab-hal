/**
 * @file test_menu.h
 * @brief UART command menu: registry-driven per-peripheral selftest runner.
 *
 * Presentation/integration layer (bsp-module-spec.md): the ONLY place that
 * knows about multiple bsp modules. Runs on USART1 (115200 8N1) and never
 * returns, so it IS the main loop after wiring.
 */
#ifndef TEST_MENU_H
#define TEST_MENU_H

/* Initializes bsp_uart, prints the banner + help, then polls RX forever.
 * Line commands: "help", "reboot", plus one entry per registered module
 * (uart, gpio, tim_pwm, tim_basic, tim_ic, ... -- append at the END, the
 * menu order is stable by spec). Output per module command: "PASS <name>"
 * (optionally " <module report>") or "FAIL <name> step<N>".
 * A second registry holds DEMO commands (not selftests: they reset or
 * park the board): wdg_test / wdg_reset_demo (arm IWDG ~2s, never feed,
 * board resets) and pwr_stop_demo (enter Stop, KEY1=PA0 wakes). Each
 * prints its warning before running. */
void test_menu_run(void);

#endif /* TEST_MENU_H */
