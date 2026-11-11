/*
 * Ultrasonic.c
 *
 *  Created on: Oct 22, 2026
 *      Author: hakon
 */

#include "ultrasonic.h"
#include <math.h>

#define TARGET_STOP_DIST   30.0f   // Always stop 30 cm before the obstacle
#define DECEL_RATE         12.0f   // Braking deceleration: 12 cm/s^2 (smooth deceleration)

/* Global distance variable in cm */
volatile float Obstacle_Dist_cm = 999.0f;

/* Internal tracking variables */
static volatile uint32_t echo_start_tick = 0;
static volatile uint32_t echo_width_us = 0;
static volatile uint8_t echo_measuring = 0;
static volatile uint8_t obstacle_stop_hits = 0;

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
    /* If previous echo measurement timed out (> 35 ms ~ 6 m), reset measuring flag */
    if (echo_measuring) {
        uint32_t elapsed_cycles = DWT->CYCCNT - echo_start_tick;
        if (elapsed_cycles > (SystemCoreClock / 1000 * 35)) {
            echo_measuring = 0;
        }
    }
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
 * @brief Returns 1 if obstacle <= TARGET_STOP_DIST is debounced and confirmed
 */
uint8_t Ultrasonic_IsObstacleConfirmed(void) {
    return (obstacle_stop_hits >= 2) ? 1 : 0;
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
                float raw_dist = ((float)echo_width_us * 0.0343f) / 2.0f;

                /* Validate physical limits (HC-SR04 operates reliably 2..400 cm) */
                if (raw_dist >= 2.0f && raw_dist <= 400.0f) {
                    /* Low-pass exponential smoothing filter */
                    if (Obstacle_Dist_cm > 400.0f) {
                        Obstacle_Dist_cm = raw_dist;
                    } else {
                        Obstacle_Dist_cm = (0.6f * Obstacle_Dist_cm) + (0.4f * raw_dist);
                    }

                    /* Debounce counter for 30 cm stop line */
                    if (Obstacle_Dist_cm <= TARGET_STOP_DIST) {
                        if (obstacle_stop_hits < 5) {
                            obstacle_stop_hits++;
                        }
                    } else if (Obstacle_Dist_cm > (TARGET_STOP_DIST + 5.0f)) {
                        /* Clear stop debounce when obstacle is beyond 35 cm */
                        obstacle_stop_hits = 0;
                    }
                }
            }
        }
    }
}

/**
 * @brief Calculates safe maximum speed based on distance to 30 cm stop line.
 *        Follows kinematic curve: v = sqrt(2 * a * d)
 */
float Ultrasonic_GetSafeSpeed(float requested_speed, float current_speed) {
    float dist = Obstacle_Dist_cm;

    /* Out of range or uninitialized -> proceed with requested speed */
    if (dist <= 2.0f || dist > 200.0f) {
        return requested_speed;
    }

    /* Target stop line reached -> cut target speed */
    if (dist <= TARGET_STOP_DIST) {
        return 0.0f;
    }

    /* Remaining distance before hitting the 30 cm line */
    float dist_to_stop_line = dist - TARGET_STOP_DIST;

    /* Physics deceleration limit: v = sqrt(2 * a * d) */
    float max_allowable_speed = sqrtf(2.0f * DECEL_RATE * dist_to_stop_line);

    /* If braking limit is lower than requested speed, decelerate smoothly */
    if (requested_speed > max_allowable_speed) {
        return max_allowable_speed;
    }

    return requested_speed;
}
