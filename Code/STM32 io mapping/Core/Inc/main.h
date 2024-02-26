/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define HSE_OSC_IN_Pin GPIO_PIN_0
#define HSE_OSC_IN_GPIO_Port GPIOH
#define HSE_OSC_OUT_Pin GPIO_PIN_1
#define HSE_OSC_OUT_GPIO_Port GPIOH
#define DEBUG_LED1_Pin GPIO_PIN_0
#define DEBUG_LED1_GPIO_Port GPIOA
#define DEBUG_LED2_Pin GPIO_PIN_1
#define DEBUG_LED2_GPIO_Port GPIOA
#define BT_TX_Pin GPIO_PIN_2
#define BT_TX_GPIO_Port GPIOA
#define BT_RX_Pin GPIO_PIN_3
#define BT_RX_GPIO_Port GPIOA
#define USB_Detect_Pin GPIO_PIN_5
#define USB_Detect_GPIO_Port GPIOC
#define Col0_Pin GPIO_PIN_0
#define Col0_GPIO_Port GPIOB
#define Col1_Pin GPIO_PIN_1
#define Col1_GPIO_Port GPIOB
#define Col2_Pin GPIO_PIN_2
#define Col2_GPIO_Port GPIOB
#define SD_Detect_Pin GPIO_PIN_10
#define SD_Detect_GPIO_Port GPIOB
#define SD_CS_Pin GPIO_PIN_12
#define SD_CS_GPIO_Port GPIOB
#define SD_SCK_Pin GPIO_PIN_13
#define SD_SCK_GPIO_Port GPIOB
#define SD_MISO_Pin GPIO_PIN_14
#define SD_MISO_GPIO_Port GPIOB
#define SD_MOSI_Pin GPIO_PIN_15
#define SD_MOSI_GPIO_Port GPIOB
#define Battery_CE_Pin GPIO_PIN_7
#define Battery_CE_GPIO_Port GPIOC
#define Battery_GPOUT_Pin GPIO_PIN_8
#define Battery_GPOUT_GPIO_Port GPIOC
#define Battery_GPOUT_EXTI_IRQn EXTI9_5_IRQn
#define Battery_SDA_Pin GPIO_PIN_9
#define Battery_SDA_GPIO_Port GPIOC
#define Battery_SCL_Pin GPIO_PIN_8
#define Battery_SCL_GPIO_Port GPIOA
#define DEBUG_TX_Pin GPIO_PIN_9
#define DEBUG_TX_GPIO_Port GPIOA
#define DEBUG_RX_Pin GPIO_PIN_10
#define DEBUG_RX_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_15
#define LCD_CS_GPIO_Port GPIOA
#define LCD_SCK_Pin GPIO_PIN_10
#define LCD_SCK_GPIO_Port GPIOC
#define LCD_DC_Pin GPIO_PIN_11
#define LCD_DC_GPIO_Port GPIOC
#define LCD_MOSI_Pin GPIO_PIN_12
#define LCD_MOSI_GPIO_Port GPIOC
#define LCD_Reset_Pin GPIO_PIN_2
#define LCD_Reset_GPIO_Port GPIOD
#define Col3_Pin GPIO_PIN_3
#define Col3_GPIO_Port GPIOB
#define Row0_Pin GPIO_PIN_4
#define Row0_GPIO_Port GPIOB
#define Row1_Pin GPIO_PIN_5
#define Row1_GPIO_Port GPIOB
#define Row2_Pin GPIO_PIN_6
#define Row2_GPIO_Port GPIOB
#define Row3_Pin GPIO_PIN_7
#define Row3_GPIO_Port GPIOB
#define Button17_Pin GPIO_PIN_8
#define Button17_GPIO_Port GPIOB
#define Button18_Pin GPIO_PIN_9
#define Button18_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
