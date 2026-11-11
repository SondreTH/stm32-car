#ifndef INC_ULTRASONIC_H_
#define INC_ULTRASONIC_H_

#include "main.h"

/* Distance variable accessible by main.c and STM32CubeMonitor */
extern volatile float Obstacle_Dist_cm;

/* Function prototypes */
void Ultrasonic_Init(void);
void Ultrasonic_Trigger(void);
float Ultrasonic_GetDistance(void);
void Ultrasonic_EXTI_Callback(uint16_t GPIO_Pin);

/* Modular speed supervisor for deceleration and 30 cm stop */
float Ultrasonic_GetSafeSpeed(float requested_speed, float current_speed);
uint8_t Ultrasonic_IsObstacleConfirmed(void);

#endif /* INC_ULTRASONIC_H_ */
