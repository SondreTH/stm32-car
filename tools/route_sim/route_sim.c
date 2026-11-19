/* =====================================================================
 * route_sim.c - drive the real car_route.c (laps), car_heading.c (gyro
 * steering) and car_drive.c (distance runs) against a model car on a PC.
 * Shows whether the route logic works: steps in the right order, turns
 * that end, the reports. It does not know the real car's steering, so
 * the radius and the end position are a guess, not a prediction.
 *
 * Build and run (from the repo root):
 *   gcc -O2 -Wall -Wextra -Itools/route_sim -ICore/Inc tools/route_sim/route_sim.c \
 *       Core/Src/car_route.c Core/Src/car_heading.c Core/Src/car_drive.c \
 *       Core/Src/car_config.c -lm -o tools/route_sim/route_sim
 *   tools/route_sim/route_sim                      1 lap of the 2 m test square
 *   tools/route_sim/route_sim laps=3 lap=S2,T90,S1,W10,T90,S2,T90,S1,T90
 *
 * Options (name=value):
 *   laps=1        laps to drive
 *   lap=...       the lap: S<m> straight, T<deg> turn (+ = left), W<s> wait
 *   lock=25       degrees the front wheels turn at full lock
 *   wb=20         wheelbase, cm
 *   trim=0        us: the real straight-ahead pulse is SERVO_CENTER_US + trim
 *                 (a car that pulls to one side)
 *   play=0        us: dead band in the steering linkage around the centre
 *   bias=0.5      deg/s: gyro bias (measured at start-up, as on the car)
 *   drift=0       deg/s per minute: the bias changes by this much
 *   obst=<s>      an obstacle in front from this time, for 2 s
 *   kh= khi=      STEER_KH, STEER_KHI,  tv= TURN_SPEED_CMS,  vn= DRIVE_SPEED_CMS
 * ===================================================================== */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
#include "main.h"
#include "car_drive.h"
#include "car_config.h"
#include "car_hw.h"
#include "car_heading.h"
#include "car_route.h"
#include "car_comms.h"

/* ---- the model car: speed along its axis like drive_sim, plus x/y/yaw ---- */
static double x = 0, y = 0, yaw = 0, v = 0, dist = 0;   /* cm, cm, deg, cm/s, cm along the path */
static double mStart = 33, mOff = 22, mSlope = 0.5, mTau = 0.8, mCoast = 90;
static double mLock = 25, mWb = 20, mTrim = 0, mPlay = 0, mBias = 0.5, mDrift = 0;
static int    braking = 0;
static int32_t encZero = 0;
static uint32_t nowMs = 0;
static double yawRate = 0;   /* deg/s, what the real gyro would measure */

/* ---- car_hw.c ---- */
float motorPct = 0;
uint32_t servoUs = SERVO_PULSE_CENTER;
volatile float sonarCm = SONAR_MAX_CM;
I2C_HandleTypeDef hi2c1;
void    motor_set(float pct) { motorPct = pct < -100 ? -100 : (pct > 100 ? 100 : pct); braking = 0; }
void    motor_brake(void)    { motorPct = 0; braking = 1; }
static int32_t enc_raw(void) { return (int32_t)floor(dist * COUNTS_PER_METER / 100.0); }
int32_t encoder_count(void)  { return enc_raw() - encZero; }
void    encoder_zero(void)   { encZero = enc_raw(); }
void    sonar_trigger(void)  {}
void    servo_set_us(uint32_t us) { servoUs = us; }
float   battery_volts(void)  { return 11.4f; }

/* ---- HAL ---- */
uint32_t HAL_GetTick(void) { return nowMs; }
void HAL_Delay(uint32_t ms) { nowMs += ms; }
int HAL_I2C_Mem_Write(I2C_HandleTypeDef *h, uint16_t a, uint16_t r, uint16_t s, uint8_t *d, uint16_t n, uint32_t t) {
  (void)h; (void)a; (void)r; (void)s; (void)d; (void)n; (void)t;
  return HAL_OK;
}
/* the MPU-6050 gyro registers: X, Y, Z rate, 131 counts per deg/s */
int HAL_I2C_Mem_Read(I2C_HandleTypeDef *h, uint16_t a, uint16_t r, uint16_t s, uint8_t *d, uint16_t n, uint32_t t) {
  (void)h; (void)a; (void)r; (void)s; (void)t;
  double bias = mBias + mDrift * nowMs / 60000.0;
  int16_t raw = (int16_t)lround((yawRate + bias) * 131.0 / IMU_YAW_SIGN);
  memset(d, 0, n);
  d[2 * IMU_YAW_AXIS] = (uint8_t)((uint16_t)raw >> 8);
  d[2 * IMU_YAW_AXIS + 1] = (uint8_t)(raw & 0xFF);
  return HAL_OK;
}

/* ---- car_comms.c: print to the terminal with the sim time in front ---- */
static char line[256];
static int  len = 0;
void lb_clear(void) { len = 0; line[0] = 0; }
static void lb_fmt(const char *f, ...) {
  va_list ap; va_start(ap, f);
  len += vsnprintf(line + len, sizeof line - len, f, ap);
  va_end(ap);
}
void lb_str(const char *s) { lb_fmt("%s", s); }
void lb_int(int32_t v) { lb_fmt("%ld", (long)v); }
void lb_uint(uint32_t v) { lb_fmt("%lu", (unsigned long)v); }
void lb_float(float v, int dec) { lb_fmt("%.*f", dec, v); }
void lb_hex(uint32_t v) { lb_fmt("%lX", (unsigned long)v); }
void lb_send(bool drop) { (void)drop; printf("%7.1f s  %s\n", nowMs / 1000.0, line); }
void say(const char *s) { lb_clear(); lb_str(s); lb_send(false); }

static void plant_step(double dt) {
  double a;
  double p = fabs(motorPct), dir = motorPct < 0 ? -1 : 1;
  double sgn = v > 0 ? 1 : -1;
  if (braking)                 a = (v == 0) ? 0 : -sgn * 150.0 - v / 0.1;
  else if (fabs(motorPct) < 1) a = (v == 0) ? 0 : -sgn * mCoast - v / 2.0;
  else if (v == 0 && p < mStart) a = 0;
  else {
    double vt = (p - mOff) / mSlope;
    a = (dir * (vt > 0 ? vt : 0) - v) / mTau;
    if (vt <= 0 && v != 0) a -= sgn * mCoast;
  }
  double vn = v + a * dt;
  if (v != 0 && vn * v < 0) vn = 0;
  if (fabs(vn) < 0.05 && (braking || fabs(motorPct) < 1.0 || p < mOff)) vn = 0;
  double ds = 0.5 * (v + vn) * dt;
  v = vn;

  /* steering: pulse -> front wheel angle, + = left. Larger us = right. */
  double off = (double)SERVO_CENTER_US + mTrim - (double)servoUs;   /* us to the left of real straight */
  if (fabs(off) <= mPlay) off = 0; else off -= (off > 0 ? mPlay : -mPlay);
  double range = off > 0 ? (SERVO_CENTER_US - SERVO_PULSE_MIN) : (SERVO_PULSE_MAX - SERVO_CENTER_US);
  double delta = mLock * off / range;
  if (delta > mLock) delta = mLock;
  if (delta < -mLock) delta = -mLock;
  double dyaw = ds / mWb * tan(delta * M_PI / 180.0) * 180.0 / M_PI;   /* bicycle model */
  yawRate = dyaw / dt;
  yaw += dyaw;
  x += ds * cos(yaw * M_PI / 180.0);
  y += ds * sin(yaw * M_PI / 180.0);
  dist += ds;
}

static const char *opts(int argc, char **argv, const char *name) {
  size_t n = strlen(name);
  for (int i = 1; i < argc; i++)
    if (!strncmp(argv[i], name, n) && argv[i][n] == '=') return argv[i] + n + 1;
  return NULL;
}
static double opt(int argc, char **argv, const char *name, double def) {
  const char *s = opts(argc, argv, name);
  return s ? atof(s) : def;
}

int main(int argc, char **argv) {
  int laps = (int)opt(argc, argv, "laps", 1);
  mLock  = opt(argc, argv, "lock", mLock);
  mWb    = opt(argc, argv, "wb", mWb);
  mTrim  = opt(argc, argv, "trim", mTrim);
  mPlay  = opt(argc, argv, "play", mPlay);
  mBias  = opt(argc, argv, "bias", mBias);
  mDrift = opt(argc, argv, "drift", mDrift);
  STEER_KH        = (float)opt(argc, argv, "kh", STEER_KH);
  STEER_KHI       = (float)opt(argc, argv, "khi", STEER_KHI);
  TURN_SPEED_CMS  = (float)opt(argc, argv, "tv", TURN_SPEED_CMS);
  DRIVE_SPEED_CMS = (float)opt(argc, argv, "vn", DRIVE_SPEED_CMS);
  double obst = opt(argc, argv, "obst", -1);

  route_init();
  const char *lapStr = opts(argc, argv, "lap");
  if (lapStr) {
    route_clear();
    for (const char *p = lapStr; *p; ) {
      char t = (char)(*p++ & ~0x20);
      char *end;
      float val = strtof(p, &end);
      if (end == p || (t != STEP_STRAIGHT && t != STEP_TURN && t != STEP_WAIT)) { printf("bad lap: %s\n", lapStr); return 1; }
      route_add(t, val);
      p = (*end == ',') ? end + 1 : end;
    }
  }
  route_list();

  if (!heading_init()) { printf("gyro init failed\n"); return 1; }
  if (!route_start(laps)) return 1;

  while (route_running() && nowMs < 3600000) {
    for (int i = 0; i < 5; i++) {     /* main loop every 5 ms */
      plant_step(0.001);
      nowMs++;
    }
    if (nowMs % DRIVE_PERIOD_MS < 5) {
      sonarCm = (obst >= 0 && nowMs / 1000.0 >= obst && nowMs / 1000.0 < obst + 2.0) ? 20.0f : SONAR_MAX_CM;
      drive_update();                 /* TIM3 interrupt */
    }
    heading_poll();
    int r = drive_result();
    if (r != RUN_NONE) route_run_ended(r);
    route_poll();
  }
  printf("model car ends at x %.0f cm, y %.0f cm from the start (straight-line %.0f cm), "
         "pointing %.1f deg, gyro says %.1f deg\n", x, y, hypot(x, y), fmod(yaw, 360.0), (double)yawDeg);
  return 0;
}
