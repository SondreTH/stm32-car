/* =====================================================================
 * car_config.h - every tunable number of the car (values in car_config.c)
 *
 * Pins are NOT set here: they are set in Team7_Car.ioc (CubeMX) and
 * show up in main.h as e.g. MOTOR_IN1_Pin / MOTOR_IN1_GPIO_Port.
 *
 *   PA0  TIM5_CH1   ENC_A        encoder A        (CN7-28)
 *   PA1  TIM5_CH2   ENC_B        encoder B        (CN7-30)
 *   PA4  ADC1_IN4   BATT_SENSE   battery divider  (CN7-32)
 *   PA5  GPIO out   LD2          green LED
 *   PA7  EXTI7      SONAR_ECHO   HC-SR04 echo     (CN10-15, via divider)
 *   PA8  TIM1_CH1   MOTOR_ENA    L298N ENA PWM    (CN10-23)
 *   PA9  USART1_TX  BT_TX        -> BT module RXD (CN10-21)
 *   PA10 USART1_RX  BT_RX        <- BT module TXD (CN10-33)
 *   PB0  GPIO in    START_BTN    button to GND    (CN7-34)
 *   PB10 TIM2_CH3   SERVO_PWM    servo signal     (CN10-25)
 *   PB4  GPIO out   MOTOR_IN1    L298N IN1        (CN10-27)
 *   PB5  GPIO out   MOTOR_IN2    L298N IN2        (CN10-29)
 *   PB6  GPIO out   SONAR_TRIG   HC-SR04 trigger  (CN10-17)
 *   PB8  I2C1_SCL   IMU_SCL      MPU-6050 SCL     (CN10-3)
 *   PB9  I2C1_SDA   IMU_SDA      MPU-6050 SDA     (CN10-5)
 *   PC13 GPIO in    B1           blue user button
 *   PA2/PA3 USART2  ST-LINK virtual COM port (USB), 115200
 * ===================================================================== */
#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- direction fixes: flip one if a bring-up test goes the wrong way ---- */
#define ENCODER_INVERT  0   /* driving forward must INCREASE counts      */
#define MOTOR_INVERT    0   /* "P 30" must drive the car FORWARD         */
#define STEER_INVERT    0   /* "S 10" must turn the front wheels LEFT    */
#define GYRO_INVERT     0   /* turning the car LEFT must INCREASE heading */

#define CONTROL_HZ      100
#define SONAR_PERIOD_MS 60
#define SONAR_MAX_CM    400

/* ---- calibration (measure, then edit car_config.c) ---- */
extern float COUNTS_PER_METER;
extern int   SERVO_CENTER_US, SERVO_MIN_US, SERVO_MAX_US;
extern float SERVO_US_PER_DEG, STEER_MAX_DEG;
extern float GYRO_SCALE, GYRO_STILL_DPS, GYRO_BIAS_LEARN;

/* ---- speed control ---- */
extern float MOTOR_DEADBAND_PCT, SPEED_FF, SPEED_KP, SPEED_KI, MAX_ACCEL, SPEED_LPF;
extern float MOVE_DECEL, MOVE_CREEP, MOVE_STOP_TOL;

/* ---- autopilot ---- */
extern float WHEELBASE_M, SONAR_X_M;
extern float AP_CRUISE, AP_CREEP, AP_DECEL, AP_STOP_TOL_M, AP_LOOKAHEAD_M, AP_TURN_SLOWDOWN;
extern uint32_t AP_WAIT_MS;
extern float REC_SPACING_M;
extern float AP_OBST_STOP_CM, AP_OBST_CLEAR_CM, AP_OBST_IGNORE_M;
extern uint32_t AP_OBST_WAIT_MS;
extern bool  AP_REROUTE;
extern float AP_REVERSE_M, AP_REVERSE_SPEED, AP_DETOUR_OFFSET_M, AP_DETOUR_RAMP_M, AP_DETOUR_PASS_M, AP_DETOUR_SPEED;

/* ---- battery monitor: battery+ -> 10k -> PA4 -> 3.3k -> GND ---- */
extern float BATT_DIVIDER, BATT_LOW_V;

#ifdef __cplusplus
}
#endif
#endif
