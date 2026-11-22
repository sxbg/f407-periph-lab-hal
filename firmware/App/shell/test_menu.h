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
 * (uart, gpio, ... -- append at the END, the menu order is stable by
 * spec). Output per module command: "PASS <name>" (optionally " <module
 * report>") or "FAIL <name> step<N>". */
void test_menu_run(void);

#endif /* TEST_MENU_H */
