/* car_config.c - default values of every tunable (see car_config.h).
 * Values changed with commands are lost at reset: copy good ones here. */
#include "car_config.h"

/* Counts per metre: drive a measured distance, counts / metres. Placeholder. */
float COUNTS_PER_METER   = 20000.0f;

/* Servo, raw microseconds (find with the "U" command) */
int   SERVO_CENTER_US    = 1500;
int   SERVO_MIN_US       = 1200;
int   SERVO_MAX_US       = 1800;
float SERVO_US_PER_DEG   = 10.0f;   /* us per degree of WHEEL angle */
float STEER_MAX_DEG      = 25.0f;

/* Gyro: turn the car 5 x 360 deg by hand, SCALE = 1800 / heading shown */
float GYRO_SCALE         = 1.000f;
float GYRO_STILL_DPS     = 1.0f;
float GYRO_BIAS_LEARN    = 0.002f;

/* Speed control */
float MOTOR_DEADBAND_PCT = 15.0f;   /* min % that makes the car move */
float SPEED_FF           = 300.0f;  /* % per m/s  (~100 / top speed) */
float SPEED_KP           = 60.0f;
float SPEED_KI           = 150.0f;
float MAX_ACCEL          = 0.5f;    /* m/s^2 setpoint ramp */
float SPEED_LPF          = 0.3f;

float MOVE_DECEL         = 0.3f;
float MOVE_CREEP         = 0.05f;
float MOVE_STOP_TOL      = 0.005f;

/* Autopilot */
float WHEELBASE_M        = 0.17f;   /* MEASURE: front axle to rear axle */
float SONAR_X_M          = 0.22f;   /* MEASURE: rear axle to sonar face */
float AP_CRUISE          = 0.30f;
float AP_CREEP           = 0.06f;
float AP_DECEL           = 0.25f;
float AP_STOP_TOL_M      = 0.01f;
float AP_LOOKAHEAD_M     = 0.60f;
float AP_TURN_SLOWDOWN   = 0.40f;
uint32_t AP_WAIT_MS      = 10500;   /* waypoint stop: 10 s + margin */
float REC_SPACING_M      = 0.25f;

float AP_OBST_STOP_CM    = 40.0f;
float AP_OBST_CLEAR_CM   = 60.0f;
float AP_OBST_IGNORE_M   = 0.30f;
uint32_t AP_OBST_WAIT_MS = 4000;
bool  AP_REROUTE         = true;
float AP_REVERSE_M       = 0.50f;
float AP_REVERSE_SPEED   = 0.12f;
float AP_DETOUR_OFFSET_M = 0.60f;   /* + = pass on the left */
float AP_DETOUR_RAMP_M   = 0.90f;
float AP_DETOUR_PASS_M   = 0.70f;
float AP_DETOUR_SPEED    = 0.20f;

/* Battery */
float BATT_DIVIDER       = 4.03f;   /* (10k + 3.3k) / 3.3k, trim against a multimeter */
float BATT_LOW_V         = 10.5f;
