/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   Pin labels below come from Team7_Car.ioc. CubeMX rewrites
  *                   this file on "Generate Code".
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
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define ENC_A_Pin GPIO_PIN_0
#define ENC_A_GPIO_Port GPIOA
#define ENC_B_Pin GPIO_PIN_1
#define ENC_B_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define BATT_SENSE_Pin GPIO_PIN_4
#define BATT_SENSE_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define SONAR_ECHO_Pin GPIO_PIN_7
#define SONAR_ECHO_GPIO_Port GPIOA
#define SONAR_ECHO_EXTI_IRQn EXTI9_5_IRQn
#define START_BTN_Pin GPIO_PIN_0
#define START_BTN_GPIO_Port GPIOB
#define SERVO_PWM_Pin GPIO_PIN_10
#define SERVO_PWM_GPIO_Port GPIOB
#define MOTOR_ENA_Pin GPIO_PIN_8
#define MOTOR_ENA_GPIO_Port GPIOA
#define BT_TX_Pin GPIO_PIN_9
#define BT_TX_GPIO_Port GPIOA
#define BT_RX_Pin GPIO_PIN_10
#define BT_RX_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define MOTOR_IN1_Pin GPIO_PIN_4
#define MOTOR_IN1_GPIO_Port GPIOB
#define MOTOR_IN2_Pin GPIO_PIN_5
#define MOTOR_IN2_GPIO_Port GPIOB
#define SONAR_TRIG_Pin GPIO_PIN_6
#define SONAR_TRIG_GPIO_Port GPIOB
#define IMU_SCL_Pin GPIO_PIN_8
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_9
#define IMU_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
