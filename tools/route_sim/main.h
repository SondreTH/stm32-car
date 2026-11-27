/* main.h for route_sim: stands in for the CubeMX main.h and the HAL, so
 * Core/Src/car_route.c and car_drive.c compile unchanged on a PC. */
#ifndef SIM_MAIN_H
#define SIM_MAIN_H
#include <stdint.h>
#include <stdbool.h>

#define TIM3_IRQn 0
#define LD2_GPIO_Port 0
#define LD2_Pin 0
static inline void HAL_NVIC_DisableIRQ(int irq) { (void)irq; }
static inline void HAL_NVIC_EnableIRQ(int irq)  { (void)irq; }
static inline void HAL_GPIO_TogglePin(int port, int pin) { (void)port; (void)pin; }
uint32_t HAL_GetTick(void);   /* route_sim.c: simulated milliseconds */
#endif
