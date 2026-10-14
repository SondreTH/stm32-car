/* =====================================================================
 * car_hw.h - hardware layer on top of the CubeMX-generated HAL setup
 *   encoder (TIM5), motor (TIM1 PWM + IN1/IN2 GPIO), I2C1 bus scan
 * ===================================================================== */
#ifndef CAR_HW_H
#define CAR_HW_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void     hw_init(void);              /* start encoder and motor PWM */

/* encoder */
int32_t  encoder_count(void);        /* counts since last encoder_zero() */
void     encoder_zero(void);

/* motor: pct -100..100 (0 = coast), brake = both terminals high */
extern float motorPct;
void     motor_set(float pct);
void     motor_brake(void);

/* I2C1: put the 7-bit addresses that answer in found[] (up to max).
 * Returns how many answered, or -1 if the bus is stuck (SDA/SCL held low). */
int      i2c_scan(uint8_t *found, int max);

#ifdef __cplusplus
}
#endif
#endif
