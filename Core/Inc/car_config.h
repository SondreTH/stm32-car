/* =====================================================================
 * car_config.h - tunables for the Activity 2.5 base (values in car_config.c)
 *
 * Pins are set in stm32-car.ioc (CubeMX) and show up in main.h.
 *   PA0  TIM5_CH1   ENC_A        encoder A        (CN7-28)
 *   PA1  TIM5_CH2   ENC_B        encoder B        (CN7-30)
 *   PA5  GPIO out   LD2          green LED (free for you, the app never touches it)
 *   PA8  TIM1_CH1   MOTOR_ENA    L298N ENA PWM    (CN10-23)
 *   PA9  USART1_TX  BT_TX        -> BT module RXD (CN10-21)
 *   PA10 USART1_RX  BT_RX        <- BT module TXD (CN10-33)
 *   PB4  GPIO out   MOTOR_IN2    L298N IN2        (CN10-27)
 *   PB5  GPIO out   MOTOR_IN1    L298N IN1        (CN10-29)
 *   PC13 GPIO in    B1           blue user button
 *   PA2/PA3 USART2  ST-LINK virtual COM port (USB), 115200
 * ===================================================================== */
#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---- direction fixes: flip one if a bring-up test goes the wrong way ---- */
#define ENCODER_INVERT  0   /* driving forward must INCREASE counts */
#define MOTOR_INVERT    0   /* "P 30" must drive the car FORWARD    */

/* ---- calibration ---- */
extern float COUNTS_PER_METER;

#ifdef __cplusplus
}
#endif
#endif
