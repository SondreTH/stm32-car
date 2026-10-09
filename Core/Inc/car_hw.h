/* =====================================================================
 * car_hw.h - hardware layer on top of the CubeMX-generated HAL setup
 *   encoder (TIM5), motor (TIM1 PWM + IN1/IN2 GPIO)
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

#ifdef __cplusplus
}
#endif
#endif
