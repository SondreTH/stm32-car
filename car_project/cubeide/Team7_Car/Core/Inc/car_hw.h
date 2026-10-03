/* =====================================================================
 * car_hw.h - hardware layer on top of the CubeMX-generated HAL setup
 *   time (SysTick), encoder (TIM5), motor (TIM1 + GPIO), servo (TIM2),
 *   HC-SR04 (GPIO + EXTI7), MPU-6050 (I2C1), battery (ADC1), buttons, LED
 * ===================================================================== */
#ifndef CAR_HW_H
#define CAR_HW_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void     hw_init(void);              /* start timers, PWM, encoder, sensors */

/* time */
uint32_t hw_micros(void);
uint32_t hw_millis(void);
void     hw_delay_us(uint32_t us);

/* encoder */
int32_t  encoder_count(void);
void     encoder_zero(void);

/* motor: pct -100..100 (0 = coast), brake = both terminals high */
extern float motorPct;
void     motor_set(float pct);
void     motor_brake(void);

/* steering: degrees, + = left */
extern float steerDeg;
extern int   steerUs;
void     steer_set_deg(float deg);
void     steer_set_us(int us, bool rawCalibration);

/* HC-SR04 */
extern float sonarCm;               /* median of last 3, SONAR_MAX_CM = nothing */
void     sonar_update(void);         /* call every loop */

/* MPU-6050 */
extern bool    imuOk;
extern uint8_t imuWhoAmI;
extern float   gyroBiasDps, gyroRateDps, headingDeg;
void     imu_begin(void);
void     imu_calibrate(uint16_t ms);
void     imu_update(float dt, bool still);

/* battery */
extern float battV;
void     battery_update(void);       /* call every loop, samples every 200 ms */

/* buttons and LED */
bool     button_pressed(void);       /* blue button OR external start button */
void     led_set(bool on);

#ifdef __cplusplus
}
#endif
#endif
