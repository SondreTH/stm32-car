// =====================================================================
//  imu.h  -  MPU-6050 gyro Z  ->  heading
//
//  Only the Z gyro (yaw rate) is used. Heading = integral of yaw rate.
//  The gyro has a small offset (bias) that would make the heading drift,
//  so we:
//    1) measure the bias at power-up (car must be still ~1 s)
//    2) keep re-learning it whenever the car is standing still
//       (motor stopped + no encoder counts) - e.g. at every waypoint stop
//    3) ignore tiny rates while standing still ("zero-velocity update")
// =====================================================================
#pragma once
#include <Wire.h>

#define MPU_ADDR 0x68          // AD0 pin low / unconnected

static bool  imuOk        = false;
static float gyroBiasDps  = 0;     // deg/s
static float gyroRateDps  = 0;     // bias-corrected yaw rate, + = turning left
static float headingDeg   = 0;     // continuous (not wrapped), + = left of start
static uint8_t imuWhoAmI  = 0;

static void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

static bool mpuRead(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

// raw gyro Z in deg/s (no bias correction). Returns false on I2C error.
static bool gyroZRaw(float& dps) {
  uint8_t b[2];
  if (!mpuRead(0x47, b, 2)) return false;          // GYRO_ZOUT_H/L
  int16_t raw = (int16_t)((b[0] << 8) | b[1]);
  dps = raw / 65.5f;                               // +-500 dps range
  if (GYRO_INVERT) dps = -dps;
  return true;
}

// Average the gyro while the car is still. Blocks ~ms milliseconds.
static void imuCalibrate(uint16_t ms = 1000) {
  if (!imuOk) return;
  float sum = 0; uint16_t n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    float d;
    if (gyroZRaw(d)) { sum += d; n++; }
    delay(2);
  }
  if (n) gyroBiasDps = sum / n;
}

static void imuBegin() {
  Wire.setSDA(PIN_IMU_SDA);
  Wire.setSCL(PIN_IMU_SCL);
  Wire.begin();
  Wire.setClock(400000);
  delay(50);

  uint8_t who = 0;
  imuOk = mpuRead(0x75, &who, 1);                  // WHO_AM_I (0x68 on genuine, clones differ)
  imuWhoAmI = who;
  if (!imuOk) return;

  mpuWrite(0x6B, 0x80); delay(100);                // reset
  mpuWrite(0x6B, 0x01);                            // wake up, clock = gyro X PLL
  mpuWrite(0x1A, 0x03);                            // DLPF ~42 Hz (filters motor vibration)
  mpuWrite(0x19, 0x00);                            // sample rate 1 kHz
  mpuWrite(0x1B, 0x08);                            // gyro +-500 dps
  mpuWrite(0x1C, 0x00);                            // accel +-2 g (unused)
  delay(100);
  imuCalibrate(1000);
}

// Call at CONTROL_HZ. 'still' = car is certainly not moving.
static void imuUpdate(float dt, bool still) {
  if (!imuOk) return;
  float raw;
  if (!gyroZRaw(raw)) return;                      // skip a failed read
  float rate = raw - gyroBiasDps;

  if (still && fabsf(rate) < GYRO_STILL_DPS) {
    // standing still: learn the bias slowly, don't integrate noise
    gyroBiasDps += GYRO_BIAS_LEARN * (raw - gyroBiasDps);
    gyroRateDps = 0;
    return;
  }
  gyroRateDps = rate * GYRO_SCALE;
  headingDeg += gyroRateDps * dt;
}
