/**
 * @file bsp_gpio.h
 * @brief GPIO BSP: LEDs / keys / LCD backlight -- register-level init,
 *        atomic BSRR output API, polled key input, self-test.
 *
 * Teaching module (P3a wave, bsp-module-spec.md template -- see bsp_uart).
 * Full CubeMX config table / register walkthrough / verification recipe live
 * in the bsp_gpio.c file header.
 *
 * Ownership rule: after bsp_gpio_init() this module OWNS PC5 (LED_GREEN),
 * PB2 (LED_BLUE), PA0 (KEY1), PA1 (KEY2), PA15 (LCD_BL). CubeMX MX_GPIO_Init
 * already configured them; this module rewrites MODER/OTYPER/OSPEEDR/PUPDR
 * at register level (idempotent). PC13 (FLASH_CS) and all other pins stay
 * HAL-owned -- the module only touches its five pins' bit fields.
 */
#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include <stdint.h>

/* Register-level (re-)config of the five owned pins: port clock gates,
 * MODER/OTYPER/OSPEEDR/PUPDR fields, boot-state BSRR writes. Returns
 * 0 = PASS (MODER/OTYPER/PUPDR read-back matches), -1 = mismatch. */
int bsp_gpio_init(void);

/* Atomic LED / backlight control via BSRR (RM0090 ch.8): one 32-bit write,
 * no read-modify-write, ISR-safe. level != 0 = pin high, 0 = pin low. */
void bsp_gpio_led_green(uint32_t on);      /* PC5, IOC boot state: low   */
void bsp_gpio_led_blue(uint32_t on);       /* PB2, IOC boot state: high  */
void bsp_gpio_lcd_backlight(uint32_t on);  /* PA15, IOC boot state: low  */

/* Polled key state. 1 = pressed, 0 = released (read once from IDR).
 * KEY1 (PA0, pull-down) closes to VDD  -> active-high, reads 1 when pressed.
 * KEY2 (PA1, pull-up)   closes to GND  -> active-low,  reads 0 when pressed. */
uint32_t bsp_gpio_key1_pressed(void);
uint32_t bsp_gpio_key2_pressed(void);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. Steps:
 *   1 = register config read-back (MODER/OTYPER/OSPEEDR/PUPDR of all five
 *       pins) + ODR round-trip: BSRR set/clear of the three outputs and the
 *       boot state restored afterwards
 *   2 = KEY idle levels via IDR: KEY1 must read 0 (pull-down), KEY2 must
 *       read 1 (pull-up); 3 samples over ~2 ms must agree (release keys!)
 *   3 = visible LED flash: green and blue each toggle twice 200 ms apart,
 *       then boot state restored (for the human watching the board) */
int bsp_gpio_selftest(void);

#endif /* BSP_GPIO_H */
