/* car_config.c - default values (see car_config.h).
 * Values changed with commands are lost at reset: copy good ones here. */
#include "car_config.h"

/* Counts per metre, measured on the car: 10.35 counts per cm.
 * TIM5 counts every edge of A and B (x4). */
float COUNTS_PER_METER = 1035.0f;

float DRIVE_SPEED_CMS = 18.0f;
float POS_TOL_CM      = 1.0f;

/* Motor. From standstill the car needs about 60 % before it moves; once it
 * rolls it follows Hakon's tuning: 6500 + 85 per cm/s out of a 14400-count
 * PWM period, so 45.1 % + 0.59 % per cm/s. */
float MOTOR_START_PCT   = 60.0f;
float MOTOR_MIN_PCT     = 6500.0f / 144.0f;
float MOTOR_PCT_PER_CMS = 85.0f / 144.0f;

/* Battery: battery+ -> R1 -> PA4 -> R2 -> GND, divider = (R1 + R2) / R2.
 * The wiring guide says 10k / 3.3k (4.03), but with 4.03 the car showed
 * 7.1 V on an 11.6 V battery: 1.76 V on PA4, so the real divider is 6.58.
 * That fits 10k / 1.8k (6.56). Check the resistors; "vb <volts>" sets this
 * from a multimeter reading. PA4 must stay below 3.3 V at 12.6 V. */
float BATT_DIVIDER = 6.58f;
