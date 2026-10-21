/* =====================================================================
 * car_hw.h - hardware layer on top of the CubeMX-generated HAL setup
 *   encoder (TIM5), motor (TIM1 PWM + IN1/IN2 GPIO), steering servo (TIM2),
 *   battery voltage (ADC1), start buttons, I2C1 bus scan
 * ===================================================================== */
#ifndef CAR_HW_H
#define CAR_HW_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void     hw_init(void);              /* start encoder, motor and servo PWM */

/* encoder */
int32_t  encoder_count(void);        /* counts since last encoder_zero() */
void     encoder_zero(void);

/* motor: pct -100..100 (0 = coast), brake = both terminals high */
extern float motorPct;
void     motor_set(float pct);
void     motor_brake(void);

/* steering servo: pulse width in microseconds (SERVO_PULSE_* in car_config.h) */
extern uint32_t servoUs;
void     servo_set_us(uint32_t us);

float    battery_volts(void);        /* battery voltage from the PA4 divider */
bool     start_pressed(void);        /* blue button B1 or the start button on PB0 */

/* I2C1: put the 7-bit addresses that answer in found[] (up to max).
 * Returns how many answered, or -1 if the bus is stuck (SDA/SCL held low). */
int      i2c_scan(uint8_t *found, int max);

#ifdef __cplusplus
}
#endif
#endif
