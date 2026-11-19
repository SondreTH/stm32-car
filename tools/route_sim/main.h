/* main.h for route_sim: stands in for the CubeMX main.h and the HAL, so
 * car_drive.c, car_heading.c and car_route.c compile unchanged on a PC.
 * The functions are in route_sim.c. */
#ifndef SIM_MAIN_H
#define SIM_MAIN_H
#include <stdint.h>
#include <stdbool.h>

#define TIM3_IRQn 0
#define LD2_GPIO_Port 0
#define LD2_Pin 0
#define HAL_OK 0
#define I2C_MEMADD_SIZE_8BIT 1
typedef struct { int dummy; } I2C_HandleTypeDef;
static inline void HAL_NVIC_DisableIRQ(int irq) { (void)irq; }
static inline void HAL_NVIC_EnableIRQ(int irq)  { (void)irq; }
static inline void HAL_GPIO_TogglePin(int port, int pin) { (void)port; (void)pin; }
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
int HAL_I2C_Mem_Write(I2C_HandleTypeDef *h, uint16_t a, uint16_t r, uint16_t s, uint8_t *d, uint16_t n, uint32_t t);
int HAL_I2C_Mem_Read(I2C_HandleTypeDef *h, uint16_t a, uint16_t r, uint16_t s, uint8_t *d, uint16_t n, uint32_t t);
#endif
