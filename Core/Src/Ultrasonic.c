/*
 * Ultrasonic.c
 *
 *  Created on: Oct 22, 2026
 *      Author: hakon
 */

#include "ultrasonic.h"
#include <math.h>

#define TARGET_STOP_DIST   30.0f   // Always stop 30 cm before the obstacle
#define DECEL_SLOPE        0.9f    // Linear slope: 0.9 (cm/s) per cm (27 cm/s at 60 cm, 18 at 50 cm, 4.5 at 35 cm, 0 at 30 cm)

/* Global distance variable in cm */
volatile float Obstacle_Dist_cm = 999.0f;

/* Internal tracking variables */
static volatile uint32_t echo_start_tick = 0;
static volatile uint32_t echo_width_us = 0;
static volatile uint8_t echo_measuring = 0;
static volatile uint8_t obstacle_stop_hits = 0;

/* Circular buffer for 3-sample median filter */
static float sample_buf[3] = {999.0f, 999.0f, 999.0f};
static uint8_t sample_idx = 0;

/**
 * @brief Computes median of 3 values (perfect single-spike outlier rejection)
 */
static float median3(float a, float b, float c) {
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
}

/**
 * @brief Feeds a new distance measurement into the median filter.
 *        If dist >= 400.0f or timeout, distance is treated as open clear space (999.0 cm).
 */
static void update_distance(float dist) {
    if (dist > 400.0f || dist < 2.0f) {
        dist = 999.0f; // Clear open space
    }

    sample_buf[sample_idx] = dist;
    sample_idx = (sample_idx + 1) % 3;

    Obstacle_Dist_cm = median3(sample_buf[0], sample_buf[1], sample_buf[2]);

    /* Debounce counter for 30 cm stop line */
    if (Obstacle_Dist_cm <= TARGET_STOP_DIST) {
        if (obstacle_stop_hits < 5) {
            obstacle_stop_hits++;
        }
    } else if (Obstacle_Dist_cm > (TARGET_STOP_DIST + 5.0f)) {
        /* Clear stop debounce when obstacle is beyond 35 cm or open */
        obstacle_stop_hits = 0;
    }
}

/**
 * @brief Initializes the Cortex-M DWT cycle counter for microsecond delays
 */
void Ultrasonic_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    sample_buf[0] = 999.0f;
    sample_buf[1] = 999.0f;
    sample_buf[2] = 999.0f;
    sample_idx = 0;
    Obstacle_Dist_cm = 999.0f;
    obstacle_stop_hits = 0;
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
    /* If previous echo measurement timed out (> 30 ms ~ 5.1 m), clear open space */
    if (echo_measuring) {
        uint32_t elapsed_cycles = DWT->CYCCNT - echo_start_tick;
        if (elapsed_cycles > (SystemCoreClock / 1000 * 30)) {
            echo_measuring = 0;
            update_distance(999.0f); // Clear open space
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

                /* Filter pulse width:
                 * - Pulses < 120 us (< 2 cm) are electrical glitches -> ignore
                 * - Pulses > 23300 us (> 400 cm) or HC-SR04 timeout (~36 ms / 600 cm) -> open space (999 cm)
                 * - Pulses between 2 and 400 cm -> valid measured distance
                 */
                if (raw_dist >= 2.0f && raw_dist <= 400.0f) {
                    update_distance(raw_dist);
                } else if (raw_dist > 400.0f) {
                    update_distance(999.0f); // Sensor internal timeout = open space
                }
            }
        }
    }
}

/**
 * @brief Calculates linear safe speed down to 30 cm stop line.
 *        Formula: v = 0.9 * (dist - 30)
 *        - At 60 cm: 27.0 cm/s
 *        - At 50 cm: 18.0 cm/s
 *        - At 35 cm: 4.5 cm/s
 *        - At 30 cm: 0.0 cm/s
 *        Works up to 50 cm/s maximum velocity (deceleration starts at ~85.6 cm).
 */
float Ultrasonic_GetSafeSpeed(float requested_speed, float current_speed) {
    float dist = Obstacle_Dist_cm;

    /* Out of range, uninitialized, or open space (> 200 cm) -> proceed with requested speed */
    if (dist <= 2.0f || dist > 200.0f) {
        return requested_speed;
    }

    /* Target stop line reached -> stop immediately */
    if (dist <= TARGET_STOP_DIST) {
        return 0.0f;
    }

    /* Linear deceleration down to 30 cm */
    float dist_to_stop_line = dist - TARGET_STOP_DIST;
    float max_allowable_speed = DECEL_SLOPE * dist_to_stop_line;

    /* If braking limit is lower than requested speed, decelerate linearly */
    if (requested_speed > max_allowable_speed) {
        return max_allowable_speed;
    }

    return requested_speed;
}
