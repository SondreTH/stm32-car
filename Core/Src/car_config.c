/* car_config.c - default values (see car_config.h).
 * Values changed with commands are lost at reset: copy good ones here. */
#include "car_config.h"

/* Counts per metre, measured on the car: 10.35 counts per cm.
 * TIM5 counts every edge of A and B (x4). */
float COUNTS_PER_METER = 1035.0f;

float DRIVE_SPEED_CMS = 30.0f;
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
 * 7.1 V on an 11.6 V battery: 1.76 V on PA4, so the divider is about 6.6
 * (10k / 1.8k = 6.56). 2026-11-25, new 3S pack: the car showed 11.8 V at
 * 12.04 V on the multimeter, so 6.58 -> 6.71. The factor also absorbs the
 * error of the 3.3 V ADC reference. "vb <volts>" sets this from a multimeter
 * reading. PA4 must stay below 3.3 V at 12.6 V (1.88 V with 6.71). */
float BATT_DIVIDER = 6.71f;

/* Steering. SERVO_CENTER_US: trim it with "sc" until the car rolls straight
 * with the heading hold off ("kh 0"). STEER_KH: the heading hold steers
 * this many us per degree the car has turned since the run started
 * (car_heading.c). 15 us per degree = full lock at about 27 degrees.
 * If the car steers further away instead of back, see IMU_YAW_SIGN. */
float SERVO_CENTER_US = SERVO_PULSE_CENTER;
float STEER_KH        = 15.0f;

/* I term of the heading hold, us of steering per degree x second. Against
 * the "heading creep" of 2026-11-12 (5 deg off with the servo 70-90 us to
 * one side). Only tried in tools/route_sim so far, not on the car. */
float STEER_KHI       = 10.0f;

/* Route turns (car_route.c) drive at this speed. Slower than a straight:
 * the car turns at full lock, and the speed loop holds it in the curve. */
float TURN_SPEED_CMS  = 15.0f;

/* Play in the steering (2026-11-22 and 2026-11-25): after a right turn the
 * centre pulse leaves the wheels a little right, after a left turn a little
 * left, so the next straight ends off to that side. The gyro steering adds
 * this many us in the direction the pulse moves, so the servo crosses the
 * gap at once. In open loop -50 us after a right turn fixed it. "sb" sets
 * it live, 0 = off. Too much: the car weaves on a straight. */
float STEER_PLAY_US   = 30.0f;
