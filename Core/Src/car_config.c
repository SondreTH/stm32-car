/* car_config.c - default values (see car_config.h).
 * Values changed with commands are lost at reset: copy good ones here. */
#include "car_config.h"

/* Counts per metre, measured on the car: 10.35 counts per cm.
 * TIM5 counts every edge of A and B (x4). */
float COUNTS_PER_METER = 1035.0f;

float DRIVE_SPEED_CMS = 18.0f;
float POS_TOL_CM      = 1.0f;

/* Motor, at MOTOR_PWM_HZ = 100 Hz. From the open-loop steps of 2026-11-12
 * (tools/logs): 40 % gave about 30 cm/s, 50 % 54 cm/s, 70 % 93 cm/s, and
 * the car started at 30-40 %. As a line: 22 % + 0.5 % per cm/s.
 * These are only the first guess for the speed loop in car_drive.c, which
 * corrects the rest. Hakon's values for 5 kHz were 45.1 % + 0.59 % per cm/s. */
float MOTOR_START_PCT   = 30.0f;
float MOTOR_MIN_PCT     = 22.0f;
float MOTOR_PCT_PER_CMS = 0.5f;

/* Battery: battery+ -> R1 -> PA4 -> R2 -> GND, divider = (R1 + R2) / R2.
 * The wiring guide says 10k / 3.3k (4.03), but with 4.03 the car showed
 * 7.1 V on an 11.6 V battery: 1.76 V on PA4, so the real divider is 6.58.
 * That fits 10k / 1.8k (6.56). Check the resistors; "vb <volts>" sets this
 * from a multimeter reading. PA4 must stay below 3.3 V at 12.6 V. */
float BATT_DIVIDER = 6.58f;

/* Steering. SERVO_CENTER_US: trim it with "sc" until the car rolls straight
 * with the heading hold off ("kh 0"). STEER_KH: the heading hold steers
 * this many us per degree the car has turned since the run started
 * (car_heading.c). 15 us per degree = full lock at about 27 degrees.
 * If the car steers further away instead of back, see IMU_YAW_SIGN. */
float SERVO_CENTER_US = SERVO_PULSE_CENTER;
float STEER_KH        = 15.0f;
