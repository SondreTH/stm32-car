/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
void NMI_Handler(void)        { while (1) { } }
void HardFault_Handler(void)  { while (1) { } }
void MemManage_Handler(void)  { while (1) { } }
void BusFault_Handler(void)   { while (1) { } }
void UsageFault_Handler(void) { while (1) { } }
void SVC_Handler(void)        { }
void DebugMon_Handler(void)   { }
void PendSV_Handler(void)     { }

void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

/** EXTI line[9:5] interrupts: PA7 = HC-SR04 echo */
void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(SONAR_ECHO_Pin);
}

/** USART1 global interrupt: Bluetooth */
void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}

/** USART2 global interrupt: ST-LINK virtual COM */
void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}
