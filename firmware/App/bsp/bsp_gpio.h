/**
 * @file bsp_gpio.h
 * @brief GPIO BSP (HAL edition): LEDs / keys / LCD backlight -- HAL output
 *        API, polled key input, self-test.
 *
 * Teaching module (HAL edition of f407-periph-lab bsp_gpio). CubeMX owns the
 * five pins (MX_GPIO_Init, Core/Src/gpio.c):
 *   PC5 LED_GREEN  output PP, boot low
 *   PB2 LED_BLUE   output PP, boot high
 *   PA0 KEY1       input  pull-down (closes to VDD -> pressed reads 1)
 *   PA1 KEY2       input  pull-up   (closes to GND -> pressed reads 0)
 *   PA15 LCD_BL    output PP, boot low
 * This module drives them via HAL_GPIO_WritePin/TogglePin/ReadPin only.
 */
#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include <stdint.h>

/* Verify the CubeMX-generated configuration with read-only probes:
 * output pins must read back their boot state on IDR (push-pull IDR
 * mirrors ODR), keys must sit at their pull-defined idle levels.
 * Returns 0 = PASS, -1 = mismatch. */
int bsp_gpio_init(void);

/* LED / backlight control via HAL_GPIO_WritePin (level != 0 = high). */
void bsp_gpio_led_green(uint32_t on);      /* PC5, boot state: low   */
void bsp_gpio_led_blue(uint32_t on);       /* PB2, boot state: high  */
void bsp_gpio_lcd_backlight(uint32_t on);  /* PA15, boot state: low  */

/* Polled key state. 1 = pressed, 0 = released (single HAL_GPIO_ReadPin). */
uint32_t bsp_gpio_key1_pressed(void);
uint32_t bsp_gpio_key2_pressed(void);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. Steps:
 *   1 = boot-state read-back: outputs at boot levels, keys at idle levels
 *       (3 samples over ~2 ms must agree -- release keys!)
 *   2 = output round-trip: toggle each LED via HAL and verify the IDR
 *       follows, then restore boot state
 *   3 = visible flash: green and blue each toggle twice 200 ms apart,
 *       boot state restored afterwards (for the human watching the board) */
int bsp_gpio_selftest(void);

#endif /* BSP_GPIO_H */
