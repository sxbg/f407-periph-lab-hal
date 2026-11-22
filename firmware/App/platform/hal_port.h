/* GENERATED from families/STM32F4.yaml by asm init (ADR-0002).
 * Family seam: template .c files include ONLY this header. DO NOT EDIT;
 * regenerate via asm init --family STM32F4. */
#ifndef HAL_PORT_H
#define HAL_PORT_H

#include "stm32f4xx.h"

/* ---- GPIO ---------------------------------------------------------------- */
#define HAL_GPIO_TYPEDEF GPIO_TypeDef
#define HAL_GPIO_BUS_REG  (RCC->AHB1ENR)
#define HAL_GPIO_CLK_A  RCC_AHB1ENR_GPIOAEN
#define HAL_GPIO_CLK_B  RCC_AHB1ENR_GPIOBEN
#define HAL_GPIO_CLK_C  RCC_AHB1ENR_GPIOCEN
#define HAL_GPIO_CLK_D  RCC_AHB1ENR_GPIODEN
#define HAL_GPIO_CLK_E  RCC_AHB1ENR_GPIOEEN
#define HAL_GPIO_CLK_H  RCC_AHB1ENR_GPIOHEN
/* Port base addresses are FAMILY data (F4 0x4002xxxx vs H7 0x5802xxxx) --
 * pack key: gpio.port_bases */
#define HAL_GPIO_PORT_A  ((HAL_GPIO_TYPEDEF *)0x40020000u)
#define HAL_GPIO_PORT_B  ((HAL_GPIO_TYPEDEF *)0x40020400u)
#define HAL_GPIO_PORT_C  ((HAL_GPIO_TYPEDEF *)0x40020800u)
#define HAL_GPIO_PORT_D  ((HAL_GPIO_TYPEDEF *)0x40020C00u)
#define HAL_GPIO_PORT_E  ((HAL_GPIO_TYPEDEF *)0x40021000u)
#define HAL_GPIO_PORT_H  ((HAL_GPIO_TYPEDEF *)0x40021C00u)
#define HAL_GPIO_PORT(port_letter)  HAL_GPIO_PORT_##port_letter

/* ---- Log UART ------------------------------------------------------------- */
#define HAL_LOG_USART      USART1
#define HAL_LOG_UART_CLK_REG   (RCC->APB2ENR)
#define HAL_LOG_UART_CLK_MASK  (RCC_APB2ENR_USART1EN)
/* Polling register names: F1/F4/F7/H7 spell SR/DR; G0/L4/U5 spell ISR/TDR */
#define HAL_LOG_SR         SR
#define HAL_LOG_DR         DR
#define HAL_LOG_TXE        USART_SR_TXE

/* ---- Fault (consumed by fault capture, roadmap U2) ------------------------ */
#define HAL_HAS_CFSR 1

#endif /* HAL_PORT_H */
