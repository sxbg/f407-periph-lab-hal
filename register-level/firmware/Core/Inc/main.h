/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define T_MOSI_Pin GPIO_PIN_2
#define T_MOSI_GPIO_Port GPIOE
#define T_MISO_Pin GPIO_PIN_3
#define T_MISO_GPIO_Port GPIOE
#define T_PEN_Pin GPIO_PIN_4
#define T_PEN_GPIO_Port GPIOE
#define FLASH_CS_Pin GPIO_PIN_13
#define FLASH_CS_GPIO_Port GPIOC
#define ADC1_IN10_Pin GPIO_PIN_0
#define ADC1_IN10_GPIO_Port GPIOC
#define ADC2_IN11_Pin GPIO_PIN_1
#define ADC2_IN11_GPIO_Port GPIOC
#define ADC3_IN12_Pin GPIO_PIN_2
#define ADC3_IN12_GPIO_Port GPIOC
#define ADC3_IN13_Pin GPIO_PIN_3
#define ADC3_IN13_GPIO_Port GPIOC
#define KEY1_Pin GPIO_PIN_0
#define KEY1_GPIO_Port GPIOA
#define KEY2_Pin GPIO_PIN_1
#define KEY2_GPIO_Port GPIOA
#define LOG2_TX_Pin GPIO_PIN_2
#define LOG2_TX_GPIO_Port GPIOA
#define LOG2_RX_Pin GPIO_PIN_3
#define LOG2_RX_GPIO_Port GPIOA
#define DAC_OUT1_Pin GPIO_PIN_4
#define DAC_OUT1_GPIO_Port GPIOA
#define FLASH_SCK_Pin GPIO_PIN_5
#define FLASH_SCK_GPIO_Port GPIOA
#define FLASH_MISO_Pin GPIO_PIN_6
#define FLASH_MISO_GPIO_Port GPIOA
#define FLASH_MOSI_Pin GPIO_PIN_7
#define FLASH_MOSI_GPIO_Port GPIOA
#define ADC1_IN14_Pin GPIO_PIN_4
#define ADC1_IN14_GPIO_Port GPIOC
#define LED_GREEN_Pin GPIO_PIN_5
#define LED_GREEN_GPIO_Port GPIOC
#define ADC1_IN8_Pin GPIO_PIN_0
#define ADC1_IN8_GPIO_Port GPIOB
#define ADC2_IN9_Pin GPIO_PIN_1
#define ADC2_IN9_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_2
#define LED_BLUE_GPIO_Port GPIOB
#define LCD_D4_Pin GPIO_PIN_7
#define LCD_D4_GPIO_Port GPIOE
#define LCD_D5_Pin GPIO_PIN_8
#define LCD_D5_GPIO_Port GPIOE
#define LCD_D6_Pin GPIO_PIN_9
#define LCD_D6_GPIO_Port GPIOE
#define LCD_D7_Pin GPIO_PIN_10
#define LCD_D7_GPIO_Port GPIOE
#define LCD_D8_Pin GPIO_PIN_11
#define LCD_D8_GPIO_Port GPIOE
#define LCD_D9_Pin GPIO_PIN_12
#define LCD_D9_GPIO_Port GPIOE
#define LCD_D10_Pin GPIO_PIN_13
#define LCD_D10_GPIO_Port GPIOE
#define LCD_D11_Pin GPIO_PIN_14
#define LCD_D11_GPIO_Port GPIOE
#define LCD_D12_Pin GPIO_PIN_15
#define LCD_D12_GPIO_Port GPIOE
#define RS485_TX_Pin GPIO_PIN_10
#define RS485_TX_GPIO_Port GPIOB
#define RS485_RX_Pin GPIO_PIN_11
#define RS485_RX_GPIO_Port GPIOB
#define I2S_WS_Pin GPIO_PIN_12
#define I2S_WS_GPIO_Port GPIOB
#define I2S_CK_Pin GPIO_PIN_13
#define I2S_CK_GPIO_Port GPIOB
#define I2S_SD_Pin GPIO_PIN_15
#define I2S_SD_GPIO_Port GPIOB
#define LCD_D13_Pin GPIO_PIN_8
#define LCD_D13_GPIO_Port GPIOD
#define LCD_D14_Pin GPIO_PIN_9
#define LCD_D14_GPIO_Port GPIOD
#define LCD_D15_Pin GPIO_PIN_10
#define LCD_D15_GPIO_Port GPIOD
#define LCD_RS_Pin GPIO_PIN_11
#define LCD_RS_GPIO_Port GPIOD
#define T_CS_Pin GPIO_PIN_13
#define T_CS_GPIO_Port GPIOD
#define LCD_D0_Pin GPIO_PIN_14
#define LCD_D0_GPIO_Port GPIOD
#define LCD_D1_Pin GPIO_PIN_15
#define LCD_D1_GPIO_Port GPIOD
#define TF_D0_Pin GPIO_PIN_8
#define TF_D0_GPIO_Port GPIOC
#define TF_D1_Pin GPIO_PIN_9
#define TF_D1_GPIO_Port GPIOC
#define LOG_TX_Pin GPIO_PIN_9
#define LOG_TX_GPIO_Port GPIOA
#define LOG_RX_Pin GPIO_PIN_10
#define LOG_RX_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define LCD_BL_Pin GPIO_PIN_15
#define LCD_BL_GPIO_Port GPIOA
#define TF_D2_Pin GPIO_PIN_10
#define TF_D2_GPIO_Port GPIOC
#define TF_D3_Pin GPIO_PIN_11
#define TF_D3_GPIO_Port GPIOC
#define TF_CK_Pin GPIO_PIN_12
#define TF_CK_GPIO_Port GPIOC
#define LCD_D2_Pin GPIO_PIN_0
#define LCD_D2_GPIO_Port GPIOD
#define LCD_D3_Pin GPIO_PIN_1
#define LCD_D3_GPIO_Port GPIOD
#define TF_CMD_Pin GPIO_PIN_2
#define TF_CMD_GPIO_Port GPIOD
#define EXP_PD3_Pin GPIO_PIN_3
#define EXP_PD3_GPIO_Port GPIOD
#define LCD_RD_Pin GPIO_PIN_4
#define LCD_RD_GPIO_Port GPIOD
#define LCD_WE_Pin GPIO_PIN_5
#define LCD_WE_GPIO_Port GPIOD
#define EXP_PD6_Pin GPIO_PIN_6
#define EXP_PD6_GPIO_Port GPIOD
#define LCD_CS_Pin GPIO_PIN_7
#define LCD_CS_GPIO_Port GPIOD
#define ENC_A_Pin GPIO_PIN_4
#define ENC_A_GPIO_Port GPIOB
#define ENC_B_Pin GPIO_PIN_5
#define ENC_B_GPIO_Port GPIOB
#define EE_SCL_Pin GPIO_PIN_6
#define EE_SCL_GPIO_Port GPIOB
#define EE_SDA_Pin GPIO_PIN_7
#define EE_SDA_GPIO_Port GPIOB
#define CAN_RX_Pin GPIO_PIN_8
#define CAN_RX_GPIO_Port GPIOB
#define CAN_TX_Pin GPIO_PIN_9
#define CAN_TX_GPIO_Port GPIOB
#define T_SCK_Pin GPIO_PIN_0
#define T_SCK_GPIO_Port GPIOE
#define EXP_PE1_Pin GPIO_PIN_1
#define EXP_PE1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
