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
 *   servo (us) = SERVO_CENTER_US + STEER_KH * (yawDeg - heading at start)
 *
 * so a car that drifts left steers right until it is straight again. In a
 * reverse run the steering works the other way round, so the sign flips.
 *
 * Everything here runs in the main loop, not in an interrupt: the OLED
 * uses the same I2C bus from the main loop, so the two never collide. The
 * OLED update takes ~25 ms; the time since the last reading is measured,
 * so the heading stays right across that gap.
 *
 * A failed reading does not lose time: the next good one is used for the
 * whole gap since the last good one (up to 0.1 s). Before 2026-11-22 the
 * gap was thrown away, so many failures made the heading too small.
 * gyroReadsFailed counts them ("G" and the open-loop report show it).
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
float gyroBiasDps = 0.0f;
uint32_t gyroReadsOk = 0, gyroReadsFailed = 0;
float gyroAxisDeg[3] = { 0.0f, 0.0f, 0.0f };   /* angle around X, Y, Z since "Y": which axis is up? */
static float axisDps[3], axisBias[3];

static uint32_t lastMs = 0, lastTryMs = 0;
static int   failCount = 0;
static bool  wasRunning = false;
static float yawAtStart = 0.0f;

static bool mpu_write(uint8_t reg, uint8_t value) {
  return HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 5) == HAL_OK;
}

/* turn rate around the vertical axis in degrees per second, bias NOT removed.
 * All three axes go to axisDps[], for the axis check in "G". */
static bool mpu_read_rate(float *dps) {
  uint8_t b[6];
  if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, MPU_GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, b, 6, 3) != HAL_OK)
    return false;
  for (int k = 0; k < 3; k++)
    axisDps[k] = (float)(int16_t)((b[2 * k] << 8) | b[2 * k + 1]) / MPU_LSB_PER_DPS;
  *dps = IMU_YAW_SIGN * axisDps[IMU_YAW_AXIS];
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
  float sum = 0.0f, sumAxis[3] = { 0.0f, 0.0f, 0.0f };
  int n = 0;
  for (int i = 0; i < 200; i++) {
    float r;
    if (mpu_read_rate(&r)) {
      sum += r; n++;
      for (int k = 0; k < 3; k++) sumAxis[k] += axisDps[k];
    }
    HAL_Delay(3);
  }
  if (n < 100) return false;
  gyroBiasDps = sum / n;
  for (int k = 0; k < 3; k++) { axisBias[k] = sumAxis[k] / n; gyroAxisDeg[k] = 0.0f; }
  yawDeg = 0.0f;
  yawRateDps = 0.0f;
  headingErrDeg = 0.0f;
  lastMs = lastTryMs = HAL_GetTick();
  imuOk = true;
  return true;
}

void heading_poll(void) {
  bool running = (state != DRIVE_OFF);

  if (imuOk) {
    uint32_t now = HAL_GetTick();
    if (now - lastTryMs >= 5) {
      lastTryMs = now;
      float r;
      if (mpu_read_rate(&r)) {
        float dt = (now - lastMs) / 1000.0f;   /* since the last GOOD reading */
        if (dt > 0.1f) dt = 0.1f;
        lastMs = now;
        failCount = 0;
        gyroReadsOk++;
        /* standing still with the motor off: trim the bias slowly. Not
         * while someone turns the car by hand (more than 1 deg/s). */
        if (!running && motorPct == 0.0f && Speed_CM_SEC == 0.0f && fabsf(r - gyroBiasDps) < 1.0f)
          gyroBiasDps += 0.002f * (r - gyroBiasDps);
        yawRateDps = r - gyroBiasDps;
        yawDeg = yawDeg + yawRateDps * dt;
        for (int k = 0; k < 3; k++) gyroAxisDeg[k] += (axisDps[k] - axisBias[k]) * dt;
      } else {
        gyroReadsFailed++;
        if (++failCount >= 20) {
          imuOk = false;               /* unplugged: stop steering by it. "I" tries again */
          servo_set_us((uint32_t)SERVO_CENTER_US);
        }
      }
    }
  }

  /* heading hold during a distance run */
  if (running && !wasRunning) yawAtStart = yawDeg;
  if (running && imuOk && STEER_KH != 0.0f) {
    headingErrDeg = yawDeg - yawAtStart;                /* + = the car has turned left */
    float dir = (target_dist_cm < 0.0f) ? -1.0f : 1.0f; /* reverse: steering acts the other way */
    float us = SERVO_CENTER_US + dir * STEER_KH * headingErrDeg;   /* larger us = steer right */
    if (us < SERVO_PULSE_MIN) us = SERVO_PULSE_MIN;
    if (us > SERVO_PULSE_MAX) us = SERVO_PULSE_MAX;
    servo_set_us((uint32_t)us);
  } else if (running) {
    headingErrDeg = yawDeg - yawAtStart;
  }
  if (!running && wasRunning) servo_set_us((uint32_t)SERVO_CENTER_US);   /* run over: wheels straight */
  wasRunning = running;
}
