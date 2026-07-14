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
#include "stm32f1xx_hal.h"

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
#define LED_DEBUG_ON_BOARD_Pin GPIO_PIN_13
#define LED_DEBUG_ON_BOARD_GPIO_Port GPIOC
#define ADC1_IN0_PS_Pin GPIO_PIN_0
#define ADC1_IN0_PS_GPIO_Port GPIOA
#define ADC1_IN1_BAT_Pin GPIO_PIN_1
#define ADC1_IN1_BAT_GPIO_Port GPIOA
#define NSS_Pin GPIO_PIN_4
#define NSS_GPIO_Port GPIOA
#define RF_RESET_Pin GPIO_PIN_0
#define RF_RESET_GPIO_Port GPIOB
#define DIO0_Pin GPIO_PIN_1
#define DIO0_GPIO_Port GPIOB
#define DIO0_EXTI_IRQn EXTI1_IRQn
#define SET_MODE_Pin GPIO_PIN_15
#define SET_MODE_GPIO_Port GPIOB
#define GET_IRQ_Pin GPIO_PIN_8
#define GET_IRQ_GPIO_Port GPIOA
#define GET_IRQ_EXTI_IRQn EXTI9_5_IRQn
#define UART_TX_Spare_Pin GPIO_PIN_9
#define UART_TX_Spare_GPIO_Port GPIOA
#define UART_RX_Spare_Pin GPIO_PIN_10
#define UART_RX_Spare_GPIO_Port GPIOA
#define MCU_BUZZER_Pin GPIO_PIN_11
#define MCU_BUZZER_GPIO_Port GPIOA
#define DETECT_ZC_PS_Pin GPIO_PIN_15
#define DETECT_ZC_PS_GPIO_Port GPIOA
#define LED_PA_Pin GPIO_PIN_5
#define LED_PA_GPIO_Port GPIOB
#define LED_PB_Pin GPIO_PIN_6
#define LED_PB_GPIO_Port GPIOB
#define LED_PC_Pin GPIO_PIN_7
#define LED_PC_GPIO_Port GPIOB
#define LED_STATUS_SLAVE_2_Pin GPIO_PIN_8
#define LED_STATUS_SLAVE_2_GPIO_Port GPIOB
#define LED_STATUS_SLAVE_1_Pin GPIO_PIN_9
#define LED_STATUS_SLAVE_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
