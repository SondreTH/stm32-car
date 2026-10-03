// =====================================================================
//  steering.h  -  front-wheel servo (MG996R)
//  Convention: positive angle = LEFT, negative = RIGHT (wheel angle, deg)
// =====================================================================
#pragma once
#include <Servo.h>

static Servo steerServo;
static float steerDeg = 0;
static int   steerUs  = 1500;

static void steerSetUs(int us, bool rawCalibration = false) {
  // normal use: clamp to the linkage limits.
  // raw calibration ("U" command): allow a wider range so you can FIND the limits.
  if (rawCalibration) us = constrain(us, 900, 2100);
  else                us = constrain(us, SERVO_MIN_US, SERVO_MAX_US);
  steerUs = us;
  steerServo.writeMicroseconds(us);
}

static void steerSetDeg(float deg) {
  deg = constrain(deg, -STEER_MAX_DEG, STEER_MAX_DEG);
  steerDeg = deg;
  float d = STEER_INVERT ? -deg : deg;
  steerSetUs(SERVO_CENTER_US + (int)lroundf(d * SERVO_US_PER_DEG));
}

static void steeringBegin() {
  steerServo.attach(PIN_SERVO);
  steerSetDeg(0);
}
