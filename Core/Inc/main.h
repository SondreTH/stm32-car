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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define ENC_A_Pin GPIO_PIN_0
#define ENC_A_GPIO_Port GPIOA
#define ENC_B_Pin GPIO_PIN_1
#define ENC_B_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define Batterim_ling_Pin GPIO_PIN_4
#define Batterim_ling_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define Sonar_ECHO_Pin GPIO_PIN_7
#define Sonar_ECHO_GPIO_Port GPIOA
#define Sonar_ECHO_EXTI_IRQn EXTI9_5_IRQn
#define Startknap_Pin GPIO_PIN_0
#define Startknap_GPIO_Port GPIOB
#define Motor_PWM_Pin GPIO_PIN_8
#define Motor_PWM_GPIO_Port GPIOA
#define Bluetooth_RXD_Pin GPIO_PIN_9
#define Bluetooth_RXD_GPIO_Port GPIOA
#define Bluetooth_TXD_Pin GPIO_PIN_10
#define Bluetooth_TXD_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Motor_IN1_Pin GPIO_PIN_4
#define Motor_IN1_GPIO_Port GPIOB
#define Motor_IN2_Pin GPIO_PIN_5
#define Motor_IN2_GPIO_Port GPIOB
#define Sonar_TRIG_Pin GPIO_PIN_6
#define Sonar_TRIG_GPIO_Port GPIOB
#define Gyro_SCL_Pin GPIO_PIN_8
#define Gyro_SCL_GPIO_Port GPIOB
#define Gyro_SDA_Pin GPIO_PIN_9
#define Gyro_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
