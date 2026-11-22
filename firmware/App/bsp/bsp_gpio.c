/**
 * @file bsp_gpio.c
 * @brief GPIO BSP (HAL edition) -- see bsp_gpio.h for the contract.
 *
 * Console-free module (bsp-module-spec rule): selftest returns the failing
 * step number; the shell prints the verdict.
 */
#include "bsp_gpio.h"

#include "main.h"          /* pin/port label macros from CubeMX */
#include "stm32f4xx_hal.h"

/* Read-only IDR probes (A3 read-only exemption): verifying that the
 * CubeMX-generated configuration is intact, no register writes here. */

static int idr_read_matches(GPIO_TypeDef *port, uint16_t pin,
                            GPIO_PinState expect)
{
    return (HAL_GPIO_ReadPin(port, pin) == expect) ? 1 : 0;
}

int bsp_gpio_init(void)
{
    /* Boot states (IOC): PC5 low, PB2 high, PA15 low; keys idle:
     * KEY1 pull-down reads 0, KEY2 pull-up reads 1. */
    if (!idr_read_matches(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET)) {
        return -1;
    }
    if (!idr_read_matches(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET)) {
        return -1;
    }
    if (!idr_read_matches(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_RESET)) {
        return -1;
    }
    if (!idr_read_matches(KEY1_GPIO_Port, KEY1_Pin, GPIO_PIN_RESET)) {
        return -1;
    }
    if (!idr_read_matches(KEY2_GPIO_Port, KEY2_Pin, GPIO_PIN_SET)) {
        return -1;
    }
    return 0;
}

void bsp_gpio_led_green(uint32_t on)
{
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,
                      (on != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void bsp_gpio_led_blue(uint32_t on)
{
    HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin,
                      (on != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void bsp_gpio_lcd_backlight(uint32_t on)
{
    HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin,
                      (on != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint32_t bsp_gpio_key1_pressed(void)
{
    return (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_SET) ? 1u : 0u;
}

uint32_t bsp_gpio_key2_pressed(void)
{
    return (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
}

int bsp_gpio_selftest(void)
{
    int s;

    /* Step 1: boot state + key idle levels, 5 samples over ~20 ms
     * (raised from 3x1 ms: single-shot electrical glitches on the key
     * nets occasionally tripped the verdict on the first board run). */
    for (s = 0; s < 5; s++) {
        if (bsp_gpio_init() != 0) {
            return 1;
        }
        HAL_Delay(5);
    }

    /* Step 2: output round-trip through HAL writes + IDR verify, then
     * restore boot states. */
    bsp_gpio_led_green(1u);
    if (!idr_read_matches(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET)) {
        bsp_gpio_led_green(0u);
        return 2;
    }
    bsp_gpio_led_blue(0u);
    if (!idr_read_matches(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET)) {
        bsp_gpio_led_blue(1u);
        return 2;
    }
    bsp_gpio_led_green(0u);
    bsp_gpio_led_blue(1u); /* boot states restored */
    if (bsp_gpio_init() != 0) {
        return 2;
    }

    /* Step 3: visible flash (green x2, blue x2, 200 ms apart). */
    for (s = 0; s < 2; s++) {
        bsp_gpio_led_green(1u);
        HAL_Delay(200);
        bsp_gpio_led_green(0u);
        HAL_Delay(200);
    }
    for (s = 0; s < 2; s++) {
        bsp_gpio_led_blue(1u);
        HAL_Delay(200);
        bsp_gpio_led_blue(0u);
        HAL_Delay(200);
    }
    bsp_gpio_led_blue(1u); /* boot state back */
    return 0;
}
