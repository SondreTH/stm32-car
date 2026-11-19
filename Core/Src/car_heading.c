/* =====================================================================
 * car_heading.c - heading from the MPU-6050 gyro, and heading hold
 *
 * The gyro measures how fast the car turns (degrees per second) around
 * its vertical axis. Adding that up over time gives the heading:
 *
 *   yawDeg = yawDeg + (rate - bias) * dt
 *
 * bias is what the gyro reads while the car stands still. It is measured
 * at start-up and trimmed slowly whenever the car stands, because it
 * drifts with temperature.
 *
 * Heading hold: when a distance run starts, the heading at that moment is
 * remembered. During the run the servo steers against the difference:
 *
 *   servo (us) = SERVO_CENTER_US + STEER_KH * (yawDeg - target heading)
 *
 * so a car that drifts left steers right until it is straight again.
 * A car that pulls to one side (servo centre a bit off, play in the
 * steering) needs a steady steering offset, which P alone only gives with
 * a steady error. The I term (STEER_KHI, M2.10) adds up the error over
 * time and supplies that offset, so the error goes back to zero:
 *
 *   servo (us) = SERVO_CENTER_US + STEER_KH * e + STEER_KHI * integral(e dt)
 *
 * In a
 * reverse run the steering works the other way round, so the sign flips.
 *
 * A route (car_route.c) sets the target heading itself: +90 degrees for a
 * left turn. In a turn the servo is at full lock until the car is within
 * HEADING_TURN_TOL_DEG of the new heading; then the formula above takes
 * over again.
 *
 * Everything here runs in the main loop, not in an interrupt: the OLED
 * uses the same I2C bus from the main loop, so the two never collide. The
 * OLED update takes ~25 ms; the time since the last reading is measured,
 * so the heading stays right across that gap.
 * ===================================================================== */
#include "main.h"
#include "car_heading.h"
#include "car_config.h"
#include "car_hw.h"
#include "car_drive.h"
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

#define MPU_ADDR        (0x68 << 1)
#define MPU_PWR_MGMT_1  0x6B
#define MPU_CONFIG      0x1A
#define MPU_GYRO_CONFIG 0x1B
#define MPU_GYRO_XOUT_H 0x43
#define MPU_LSB_PER_DPS 131.0f      /* at the +-250 deg/s range */

bool  imuOk = false;
volatile float yawDeg = 0.0f;
volatile float yawRateDps = 0.0f;
volatile float headingErrDeg = 0.0f;
volatile float headingTargetDeg = 0.0f;
float gyroBiasDps = 0.0f;

static uint32_t lastMs = 0;
static int   failCount = 0;
static bool  wasRunning = false;
static bool  wasSteering = false;
static bool  targetFixed = false;     /* a route owns headingTargetDeg */
static bool  turning = false;         /* a route turn: steer although no distance run */
static float headingAcum = 0.0f;      /* I term of the heading hold: degrees x seconds */
static float dtSteer = 0.0f;          /* seconds since the last gyro reading */

static bool mpu_write(uint8_t reg, uint8_t value) {
  return HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 5) == HAL_OK;
}

/* turn rate around the vertical axis in degrees per second, bias NOT removed */
static bool mpu_read_rate(float *dps) {
  uint8_t b[6];
  if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, MPU_GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, b, 6, 3) != HAL_OK)
    return false;
  int16_t raw = (int16_t)((b[2 * IMU_YAW_AXIS] << 8) | b[2 * IMU_YAW_AXIS + 1]);
  *dps = IMU_YAW_SIGN * (float)raw / MPU_LSB_PER_DPS;
  return true;
}

bool heading_init(void) {
  imuOk = false;
  failCount = 0;
  if (!mpu_write(MPU_PWR_MGMT_1, 0x01)) return false;   /* wake up, clock from the gyro */
  HAL_Delay(30);
  mpu_write(MPU_CONFIG, 0x03);        /* low-pass filter 44 Hz: less vibration noise */
  mpu_write(MPU_GYRO_CONFIG, 0x00);   /* range +-250 deg/s */
  HAL_Delay(30);

  /* bias: average of 200 readings while the car stands still */
  float sum = 0.0f;
  int n = 0;
  for (int i = 0; i < 200; i++) {
    float r;
    if (mpu_read_rate(&r)) { sum += r; n++; }
    HAL_Delay(3);
  }
  if (n < 100) return false;
  gyroBiasDps = sum / n;
  yawDeg = 0.0f;
  yawRateDps = 0.0f;
  headingErrDeg = 0.0f;
  lastMs = HAL_GetTick();
  imuOk = true;
  return true;
}

void heading_poll(void) {
  bool running = (state != DRIVE_OFF);
  dtSteer = 0.0f;   /* set below only when there is a new gyro reading */

  if (imuOk) {
    uint32_t now = HAL_GetTick();
    if (now - lastMs >= 5) {
      float dt = (now - lastMs) / 1000.0f;
      if (dt > 0.1f) dt = 0.1f;
      lastMs = now;
      dtSteer = dt;
      float r;
      if (mpu_read_rate(&r)) {
        failCount = 0;
        /* standing still with the motor off: trim the bias slowly. Not
         * while someone turns the car by hand (more than 1 deg/s). */
        if (!running && motorPct == 0.0f && Speed_CM_SEC == 0.0f && fabsf(r - gyroBiasDps) < 1.0f)
          gyroBiasDps += 0.002f * (r - gyroBiasDps);
        yawRateDps = r - gyroBiasDps;
        yawDeg = yawDeg + yawRateDps * dt;
      } else if (++failCount >= 20) {
        imuOk = false;                 /* unplugged: stop steering by it. "I" tries again */
        servo_set_us((uint32_t)SERVO_CENTER_US);
      }
    }
  }

  /* Steering by the gyro: during a distance run, and during a route turn.
   * Normally the target is the heading the run starts with. A route sets
   * the target itself (heading_target), so a run does not overwrite it. */
  bool steering = running || turning;
  if (running && !wasRunning) {
    if (!targetFixed) headingTargetDeg = yawDeg;
    headingAcum = 0.0f;                                 /* every run starts its own I term */
  }
  if (steering) {
    headingErrDeg = yawDeg - headingTargetDeg;          /* + = the car points left of the target */
    if (imuOk && STEER_KH != 0.0f) {
      /* reverse run: steering acts the other way. A turn always drives forward. */
      float dir = (running && target_dist_cm < 0.0f) ? -1.0f : 1.0f;
      /* I term: only on a straight, while the car rolls and is close to
       * the heading. In a turn the error is large on purpose. */
      if (running && !turning && fabsf(Speed_CM_SEC) >= DRIVE_MOVING_CMS && fabsf(headingErrDeg) < 10.0f)
        headingAcum += headingErrDeg * dtSteer;
      float iUs = 0.0f;
      if (STEER_KHI > 0.0f) {
        iUs = STEER_KHI * headingAcum;
        if (iUs > HEADING_I_MAX_US)  { iUs = HEADING_I_MAX_US;  headingAcum = iUs / STEER_KHI; }
        if (iUs < -HEADING_I_MAX_US) { iUs = -HEADING_I_MAX_US; headingAcum = iUs / STEER_KHI; }
      } else headingAcum = 0.0f;
      float us = SERVO_CENTER_US + dir * (STEER_KH * headingErrDeg + iUs);   /* larger us = steer right */
      /* A turn steers at full lock until it is close to the new heading.
       * With P only, a car that pulls to one side would stop turning a few
       * degrees short: the small steering left would just keep it straight. */
      if (turning && headingErrDeg < -HEADING_TURN_TOL_DEG) us = SERVO_PULSE_MIN;   /* target is to the left */
      if (turning && headingErrDeg >  HEADING_TURN_TOL_DEG) us = SERVO_PULSE_MAX;   /* target is to the right */
      if (us < SERVO_PULSE_MIN) us = SERVO_PULSE_MIN;
      if (us > SERVO_PULSE_MAX) us = SERVO_PULSE_MAX;
      servo_set_us((uint32_t)us);
    }
  }
  if (!steering && wasSteering) servo_set_us((uint32_t)SERVO_CENTER_US);   /* done: wheels straight */
  wasRunning = running;
  wasSteering = steering;
}

void heading_target(float deg) {
  headingTargetDeg = deg;
  targetFixed = true;
}

void heading_turn(bool on) {
  turning = on;
}

void heading_release(void) {
  targetFixed = false;
  turning = false;
}
