// =====================================================================
//  config.h  -  every pin and every tunable number lives here
// =====================================================================
#pragma once
#include <stdint.h>

// ---------------- PIN MAP (Nucleo-F446RE) ----------------------------
// Arduino-header name in [brackets]. Give this list to whoever wires.
//
//  Encoder (25GA-370)  A -> PA0 [A0]   B -> PA1 [A1]   VCC -> 3V3  GND -> GND
//                      (TIM2 hardware encoder mode - these two pins are fixed)
//  L298N               ENA -> PA8 [D7] (PWM, remove the ENA jumper)
//                      IN1 -> PB4 [D5]   IN2 -> PB5 [D4]
//  Servo MG996R        signal -> PB10 [D6]   (power from a separate 5-6 V BEC!)
//  HC-SR04             TRIG -> PB6 [D10]  ECHO -> PA7 [D11] via 1k/2k divider
//  Bluetooth BT04/ZS-040  module TXD -> PA10 [D2]   module RXD -> PA9 [D8]
//  MPU-6050 (GY-521)   SDA -> PB9 [D14]  SCL -> PB8 [D15]  VCC -> 5V  GND -> GND
//                      mount FLAT, chip side up, screwed/taped firmly to the chassis
//  On-board            blue button B1 = PC13, green LED LD2 = PA5 [D13]
//  NOT usable          PA2/PA3 [D1/D0] = USB serial to the laptop
//  ALL GROUNDS TIED TOGETHER (battery, L298N, BEC, Nucleo, sensors)

#define PIN_MOTOR_EN    PA8
#define PIN_MOTOR_IN1   PB4
#define PIN_MOTOR_IN2   PB5
#define PIN_SERVO       PB10
#define PIN_SONAR_TRIG  PB6
#define PIN_SONAR_ECHO  PA7
#define PIN_BT_RX       PA10   // MCU receives here  (module TXD)
#define PIN_BT_TX       PA9    // MCU transmits here (module RXD)
#define PIN_IMU_SDA     PB9
#define PIN_IMU_SCL     PB8
#define PIN_LED         PA5
#define PIN_BUTTON      PC13   // on-board blue button, active LOW
#define PIN_BUTTON2     PB0    // external start button on the perfboard: PB0 [CN7-34] -> button -> GND

#define USB_BAUD        115200
#define BT_BAUD         9600   // BT04-A / HC-05 / HC-06 default

// ---------------- DIRECTION FIXES -------------------------------------
// Flip one of these if a bring-up test goes the wrong way.
#define ENCODER_INVERT  false  // pushing/driving forward must INCREASE counts
#define MOTOR_INVERT    false  // "P 30" must drive the car FORWARD
#define STEER_INVERT    false  // "S 10" must turn the front wheels LEFT
#define GYRO_INVERT     false  // turning the car LEFT must INCREASE heading

// ---------------- CALIBRATION (measure, then edit here) ---------------
// Counts per metre: drive a measured distance, counts / metres.
// Placeholder only - the real value depends on encoder PPR, gearbox and
// the gear between motor and axle.
static float COUNTS_PER_METER = 20000.0f;

// Servo: find these with the "U" command (raw microseconds).
static int   SERVO_CENTER_US  = 1500;  // wheels straight
static int   SERVO_MIN_US     = 1200;  // hard limit one side (linkage end)
static int   SERVO_MAX_US     = 1800;  // hard limit other side
static float SERVO_US_PER_DEG = 10.0f; // us per degree of WHEEL angle
static float STEER_MAX_DEG    = 25.0f;

// Gyro: turn the car 5 x 360 deg by hand, SCALE = 1800 / heading shown.
static float GYRO_SCALE      = 1.000f;
static float GYRO_STILL_DPS  = 1.0f;    // below this while stopped = noise, not turning
static float GYRO_BIAS_LEARN = 0.002f;  // how fast the bias is re-learned when still

// ---------------- MOTOR / SPEED CONTROL -------------------------------
#define PWM_FREQ_HZ     5000   // L298N is slow, keep this low-ish
#define PWM_BITS        10
#define PWM_MAX         1023

static float MOTOR_DEADBAND_PCT = 15.0f;  // min % that makes the car move (find with P)
static float SPEED_FF   = 300.0f;  // feed-forward, % per m/s  (~100 / top speed)
static float SPEED_KP   = 60.0f;   // % per (m/s) of error
static float SPEED_KI   = 150.0f;  // % per (m/s * s) of error
static float MAX_ACCEL  = 0.5f;    // m/s^2, setpoint ramp
static float SPEED_LPF  = 0.3f;    // 0..1, speed low-pass (higher = less filtering)

// Distance moves ("D" command)
static float MOVE_DECEL   = 0.3f;   // m/s^2 used to plan the slow-down
static float MOVE_CREEP   = 0.05f;  // m/s minimum speed in the final approach
static float MOVE_STOP_TOL= 0.005f; // m, stop when this close

// ---------------- AUTOPILOT (automatic mode) ----------------------------
static float WHEELBASE_M       = 0.17f;  // MEASURE: front axle to rear axle, metres
static float SONAR_X_M         = 0.22f;  // MEASURE: sonar distance in front of the rear axle
static float AP_CRUISE         = 0.30f;  // m/s normal driving speed
static float AP_CREEP          = 0.06f;  // m/s slowest speed (final approach)
static float AP_DECEL          = 0.25f;  // m/s^2 planned slow-down before a stop
static float AP_STOP_TOL_M     = 0.01f;  // stop when this close to the waypoint (along the path)
static float AP_LOOKAHEAD_M    = 0.60f;  // pure pursuit look-ahead. Bigger = smoother, cuts corners more
static float AP_TURN_SLOWDOWN  = 0.40f;  // 0..1, how much to slow down at full steering
static uint32_t AP_WAIT_MS     = 10500;  // waypoint stop time (10 s + margin)
static float REC_SPACING_M     = 0.25f;  // teach mode: one path point every ... m

// Obstacles (HC-SR04)
static float AP_OBST_STOP_CM   = 40.0f;  // stop if something is closer than this
static float AP_OBST_CLEAR_CM  = 60.0f;  // ... and continue when it is further than this
static float AP_OBST_IGNORE_M  = 0.30f;  // ignore obstacles in the last ... m before a stop
static uint32_t AP_OBST_WAIT_MS= 4000;   // wait this long before rerouting
static bool  AP_REROUTE        = true;   // false = only stop and wait (1 point instead of 2)
static float AP_REVERSE_M      = 0.50f;  // back up this far before the detour
static float AP_REVERSE_SPEED  = 0.12f;  // m/s
static float AP_DETOUR_OFFSET_M= 0.60f;  // sideways shift, + = pass on the LEFT, - = right
static float AP_DETOUR_RAMP_M  = 0.90f;  // distance used to move sideways (and back)
static float AP_DETOUR_PASS_M  = 0.70f;  // keep shifted this far past the obstacle
static float AP_DETOUR_SPEED   = 0.20f;  // m/s while detouring

// Battery monitor: battery+ -> 10k -> PA4 [A2] -> 3.3k -> GND
#define PIN_BATT        PA4
static float BATT_DIVIDER      = 4.03f;  // (10k + 3.3k) / 3.3k, trim so the reading matches a multimeter
static float BATT_LOW_V        = 10.5f;  // 3S LiPo: 3.5 V/cell -> warn

// ---------------- TIMING ------------------------------------------------
#define CONTROL_HZ      100
#define SONAR_PERIOD_MS 60
#define SONAR_MAX_CM    400    // "nothing seen" is reported as this value
