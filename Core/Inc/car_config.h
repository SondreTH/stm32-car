/* =====================================================================
 * car_config.h - tunables for the car (values in car_config.c)
 *
 * Pins are set in stm32-car.ioc (CubeMX) and show up in main.h.
 *   PA0  TIM5_CH1   ENC_A        encoder A        (CN7-28)
 *   PA1  TIM5_CH2   ENC_B        encoder B        (CN7-30)
 *   PA4  ADC1_IN4   BATT_SENSE   battery divider  (CN7-32)
 *   PA5  GPIO out   LD2          green LED: blinks every 100 ms step of a run
 *                                and on every Bluetooth byte
 *   PA7  EXTI7      SONAR_ECHO   HC-SR04 ECHO     (CN10-15, via 1k/2k divider)
 *   PA8  TIM1_CH1   MOTOR_ENA    L298N ENA PWM    (CN10-23)
 *   PA9  USART1_TX  BT_TX        -> BT module RXD (CN10-21)
 *   PA10 USART1_RX  BT_RX        <- BT module TXD (CN10-33), pull-up
 *   PB0  GPIO in    START_BTN    button to GND    (CN7-34), pull-up
 *   PB4  GPIO out   MOTOR_IN1    L298N IN1        (CN10-27)
 *   PB5  GPIO out   MOTOR_IN2    L298N IN2        (CN10-29)
 *   PB6  GPIO out   SONAR_TRIG   HC-SR04 TRIG     (CN10-17)
 *   PB8  I2C1_SCL   I2C_SCL      MPU-6050 + OLED  (CN10-3, Arduino D15)
 *   PB9  I2C1_SDA   I2C_SDA      MPU-6050 + OLED  (CN10-5, Arduino D14)
 *   PB10 TIM2_CH3   SERVO_PWM    servo signal     (CN10-25)
 *   PC13 GPIO in    B1           blue user button
 *   PA2/PA3 USART2  ST-LINK virtual COM port (USB), 115200
 *   TIM3 (no pin)                control loop interrupt every 100 ms
 * The OLED (0x3C) and the MPU-6050 (0x68) share I2C1, both used from the
 * main loop only.
 * ===================================================================== */
#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---- direction fixes: flip one if a bring-up test goes the wrong way ---- */
#define ENCODER_INVERT  1   /* driving forward must INCREASE counts (checked on the car 2026-11-12) */
#define MOTOR_INVERT    0   /* "P 30" must drive the car FORWARD    */

/* ---- OLED (car_oled.c): match your module ---- */
#define OLED_HEIGHT     64  /* 64 = 0.96" 128x64, 32 = 0.91" 128x32             */
#define OLED_COL_OFFSET 0   /* 0 = SSD1306, 2 = SH1106 (most 1.3" modules)       */
#define OLED_FLIP       0   /* 1 = rotate 180 degrees if it reads upside down    */

/* ---- steering servo (TIM2, 1 us per tick), measured on the car ---- */
#define SERVO_PULSE_MIN     600   /* us, full left  */
#define SERVO_PULSE_CENTER 1080   /* us, straight: default of SERVO_CENTER_US ("sc").
                                     1080 drove straight on 2026-11-12 (was 1100) */
#define SERVO_PULSE_MAX    1500   /* us, full right */

/* ---- MPU-6050 gyro (car_heading.c) ---- */
#define IMU_YAW_AXIS    2     /* which gyro axis points up: 0 = X, 1 = Y, 2 = Z (module lying flat) */
#define IMU_YAW_SIGN    1.0f  /* turning the car LEFT must make yaw go UP: else set -1.0f           */

/* ---- motor PWM ---- */
#define MOTOR_PWM_HZ      100     /* set at start-up, "pf" changes it. PWM is on the L298N enable
                                     pin: at CubeMX's 5 kHz the car needed 80-90 % to start, at
                                     100 Hz 30-40 % (tests of 2026-11-12). It buzzes; that is normal */

/* ---- distance run (car_drive.c) ---- */
#define DRIVE_PERIOD_MS   100     /* speed profile + PID step (M2.9 slide 20).
                                     Must match TIM3: 84 MHz / ((839+1) * (9999+1)) = 10 Hz */
#define DRIVE_RAMP_S      2.0f    /* acceleration time = deceleration time     */
#define BUTTON_RUN_M      1.0f    /* B1 / start button: drive this far ...     */
#define BUTTON_DELAY_MS   1000    /* ... after this long, to let go of the car */
#define DRIVE_MAX_CMS     80.0f   /* timed run: refuse if it needs a faster Sn */

/* ---- position loop (car_drive.c). Gains Kp/Ki/Kd are in car_drive.c ---- */
#define PID_I_MAX_CMS       10.0f /* anti-windup: the I term never adds more than this (cm/s) */
#define PID_OUT_MAX_CMS     12.0f /* the whole PID correction never adds more than this (cm/s) */
#define SPEED_I_MAX_PCT     80.0f /* speed loop: its I term stays within +- this (motor %)    */
#define DRIVE_CMD_MIN_CMS   1.0f  /* commands smaller than this coast instead of driving      */
#define DRIVE_MOVING_CMS    1.5f  /* slower than this (2 counts per step) = standing still    */
#define DRIVE_SETTLE_S      5.0f  /* after the profile: at most this long to reach the target */
#define DRIVE_MAX_REVERSALS 1     /* settling: change direction at most this often            */
#define KICK_RAMP_PCT_S     25.0f /* a standing car: add at least this much % per second       */
#define DRIVE_STALL_S       3.0f  /* motor driven this long without counts: give up           */
#define ENC_FAULT_CM        3.0f  /* this far the wrong way: encoder direction is wrong       */

/* ---- obstacle stop (HC-SR04) ---- */
#define SONAR_TICK_US     10      /* one TIM3 count = (839+1) / 84 MHz = 10 us  */
#define SONAR_MAX_CM      400.0f  /* no echo = nothing in range                 */
#define OBST_STOP_CM      30.0f   /* closer than this: brake and pause the run  */
#define OBST_CLEAR_CM     35.0f   /* further than this: continue the run        */

#define MANUAL_PCT        60.0f   /* f / b commands */

/* ---- battery warning (car_app.c). 3S LiPo: 12.6 V full, 11.1 V nominal ---- */
#define BATT_LOW_V        9.9f    /* 3.3 V per cell: warn, stop driving and charge.
                                     Below 9.0 V (3.0 V per cell) the cells are damaged */
#define BATT_PRESENT_V    5.0f    /* reading under this = no battery (USB power): no warning */

/* ---- values that commands can change (defaults in car_config.c) ---- */
extern float COUNTS_PER_METER;    /* M   encoder counts per metre                          */
extern float DRIVE_SPEED_CMS;     /* vn  nominal speed Sn of a run without a set time      */
extern float POS_TOL_CM;          /* tol a run is done within this distance of the target  */
/* motor % for a speed in cm/s: an offset that keeps the car rolling, plus a slope */
extern float MOTOR_START_PCT;     /* ms  needed to get the car moving from standstill      */
extern float MOTOR_MIN_PCT;       /* mo  offset once it rolls                              */
extern float MOTOR_PCT_PER_CMS;   /* mg  slope, % per cm/s                                 */
extern float BATT_DIVIDER;        /* vb  battery volts per volt on PA4                     */
extern float SERVO_CENTER_US;     /* sc  servo pulse that drives straight                  */
extern float STEER_KH;            /* kh  heading hold: us of steering per degree off, 0 = off */

#ifdef __cplusplus
}
#endif
#endif
