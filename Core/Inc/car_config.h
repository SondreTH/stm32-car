/* =====================================================================
 * car_config.h - tunables for the car (values in car_config.c)
 *
 * Pins are set in stm32-car.ioc (CubeMX) and show up in main.h.
 *   PA0  TIM5_CH1   ENC_A        encoder A        (CN7-28)
 *   PA1  TIM5_CH2   ENC_B        encoder B        (CN7-30)
 *   PA4  ADC1_IN4   BATT_SENSE   battery divider  (CN7-32)
 *   PA5  GPIO out   LD2          green LED: blinks every 100 ms step of a run
 *                                and on every Bluetooth byte
 *   PA7  GPIO in    SONAR_ECHO   HC-SR04 ECHO     (CN10-15, via 1k/2k divider)
 *   PA8  TIM1_CH1   MOTOR_ENA    L298N ENA PWM    (CN10-23)
 *   PA9  USART1_TX  BT_TX        -> BT module RXD (CN10-21)
 *   PA10 USART1_RX  BT_RX        <- BT module TXD (CN10-33), pull-up
 *   PB0  GPIO in    START_BTN    button to GND    (CN7-34), pull-up
 *   PB4  GPIO out   MOTOR_IN1    L298N IN1        (CN10-27)
 *   PB5  GPIO out   MOTOR_IN2    L298N IN2        (CN10-29)
 *   PB6  GPIO out   SONAR_TRIG   HC-SR04 TRIG     (CN10-17)
 *   PB8  I2C1_SCL   I2C_SCL      OLED SCK/SCL     (CN10-3, Arduino D15)
 *   PB9  I2C1_SDA   I2C_SDA      OLED SDA         (CN10-5, Arduino D14)
 *   PB10 TIM2_CH3   SERVO_PWM    servo signal     (CN10-25)
 *   PC13 GPIO in    B1           blue user button
 *   PA2/PA3 USART2  ST-LINK virtual COM port (USB), 115200
 * The sonar pins are set up but not used yet. The MPU-6050 (PC1 SCL,
 * PC0 SDA) is wired but not set up: those pins have no hardware I2C.
 * ===================================================================== */
#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---- direction fixes: flip one if a bring-up test goes the wrong way ---- */
#define ENCODER_INVERT  0   /* driving forward must INCREASE counts */
#define MOTOR_INVERT    0   /* "P 30" must drive the car FORWARD    */

/* ---- OLED (car_oled.c): match your module ---- */
#define OLED_HEIGHT     64  /* 64 = 0.96" 128x64, 32 = 0.91" 128x32             */
#define OLED_COL_OFFSET 0   /* 0 = SSD1306, 2 = SH1106 (most 1.3" modules)       */
#define OLED_FLIP       0   /* 1 = rotate 180 degrees if it reads upside down    */

/* ---- steering servo (TIM2, 1 us per tick), measured on the car ---- */
#define SERVO_PULSE_MIN     600   /* us, full left  */
#define SERVO_PULSE_CENTER 1100   /* us, straight   */
#define SERVO_PULSE_MAX    1500   /* us, full right */

/* ---- distance run (car_drive.c). PID gains are Kp/Ki/Kd in car_drive.c ---- */
#define DRIVE_PERIOD_MS   100     /* speed profile + PID step (M2.9 slide 20)  */
#define DRIVE_SPEED_CMS   18.0f   /* nominal speed Sn                          */
#define DRIVE_RAMP_S      2.0f    /* acceleration time = deceleration time     */
#define BUTTON_RUN_M      1.0f    /* B1 / start button: drive this far ...     */
#define BUTTON_DELAY_MS   1000    /* ... after this long, to let go of the car */

/* motor % for a speed in cm/s: an offset that gets the car moving, plus a slope.
 * Tuned on the car as 6500 + 85 per cm/s out of a 14400-count PWM period. */
#define MOTOR_MIN_PCT      (6500.0f / 144.0f)   /* 45.1 %          */
#define MOTOR_PCT_PER_CMS  (85.0f / 144.0f)     /* 0.59 % per cm/s */
#define MANUAL_PCT         60.0f                /* f / b commands  */

/* ---- battery: battery+ -> 10k -> PA4 -> 3.3k -> GND ---- */
#define BATT_DIVIDER    4.03f   /* (10k + 3.3k) / 3.3k, trim against a multimeter */

/* ---- calibration ---- */
extern float COUNTS_PER_METER;

#ifdef __cplusplus
}
#endif
#endif
