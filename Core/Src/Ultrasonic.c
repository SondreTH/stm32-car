/*
 * Ultrasonic.c
 *
 *  Created on: Oct 22, 2026
 *      Author: hakon
 */


#include "ultrasonic.h"

/* Global distance variable in cm */
volatile float Obstacle_Dist_cm = 999.0f;

/* Internal tracking variables */
static volatile uint32_t echo_start_tick = 0;
static volatile uint32_t echo_width_us = 0;
static volatile uint8_t echo_measuring = 0;

/**
 * @brief Initializes the Cortex-M DWT cycle counter for microsecond delays
 */
void Ultrasonic_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief Microsecond delay using hardware CPU cycles
 */
static void delay_us(uint32_t us) {
    uint32_t start_cycles = DWT->CYCCNT;
    uint32_t wait_cycles = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start_cycles) < wait_cycles);
}

/**
 * @brief Sends a 10 microsecond pulse on the trigger pin
 */
void Ultrasonic_Trigger(void) {
    HAL_GPIO_WritePin(GPIOB, Sonar_TRIG_Pin, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(GPIOB, Sonar_TRIG_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Returns the most recent valid measured distance in cm
 */
float Ultrasonic_GetDistance(void) {
    return Obstacle_Dist_cm;
}

/**
 * @brief Handles Echo pulse start/stop via EXTI interrupt
 *        Must be called from HAL_GPIO_EXTI_Callback() in main.c
 */
void Ultrasonic_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == Sonar_ECHO_Pin) {
        if (HAL_GPIO_ReadPin(Sonar_ECHO_GPIO_Port, Sonar_ECHO_Pin) == GPIO_PIN_SET) {
            /* Rising edge: start measuring */
            echo_start_tick = DWT->CYCCNT;
            echo_measuring = 1;
        } else {
            /* Falling edge: calculate duration and convert to cm */
            if (echo_measuring) {
                uint32_t cycles = DWT->CYCCNT - echo_start_tick;
                echo_width_us = cycles / (SystemCoreClock / 1000000);
                echo_measuring = 0;

                /* Distance = (Time in us * speed of sound 0.0343 cm/us) / 2 */
                Obstacle_Dist_cm = ((float)echo_width_us * 0.0343f) / 2.0f;
            }
        }
    }
}
