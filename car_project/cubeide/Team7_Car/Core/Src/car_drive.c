/* car_drive.c - odometry and PI speed control (see car_drive.h) */
#include "car_drive.h"
#include "car_hw.h"
#include "car_config.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

DriveState drv = { DM_STOP, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.2f, 0, false };

static float clampf_(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void drive_stop(void) {
  drv.mode = DM_STOP;
  drv.target = drv.setpoint = 0;
  drv.integ = 0;
  motor_brake();
}
void drive_open(float pct)  { drv.mode = DM_OPEN; drv.openPct = pct; drv.integ = 0; }
void drive_speed(float mps) { drv.mode = DM_SPEED; drv.target = mps; }

void drive_distance(float meters, float cruise) {
  drv.moveStart  = drv.distance;
  drv.moveGoal   = drv.distance + meters;
  drv.moveCruise = fabsf(cruise);
  drv.moveDone   = false;
  drv.mode       = DM_DISTANCE;
}

void drive_zero(void) {
  encoder_zero();
  drv.lastCount = 0;
  drv.distance = 0;
  drv.x = drv.y = 0;
}

/* certainly not moving: braked and no encoder counts for 0.3 s */
bool drive_is_still(void) { return drv.mode == DM_STOP && drv.stillTicks > CONTROL_HZ * 3 / 10; }

/* encoder gives the distance step, gyro the direction (midpoint rule) */
void odometry_update(float headingBeforeDeg, float headingAfterDeg) {
  float ds = drv.lastDelta / COUNTS_PER_METER;
  float th = 0.5f * (headingBeforeDeg + headingAfterDeg) * (float)M_PI / 180.0f;
  drv.x += ds * cosf(th);
  drv.y += ds * sinf(th);
}

/* PI with feed-forward, deadband compensation and anti-windup -> motor % */
static float speed_pi(float sp, float meas, float dt) {
  if (fabsf(sp) < 0.005f) { drv.integ = 0; return 0; }
  float err = sp - meas;
  float u = SPEED_FF * sp + (sp > 0 ? MOTOR_DEADBAND_PCT : -MOTOR_DEADBAND_PCT) + SPEED_KP * err + drv.integ;
  if (fabsf(u) < 100.0f || ((u > 0) != (err > 0))) {
    drv.integ += SPEED_KI * err * dt;
    drv.integ = clampf_(drv.integ, -60.0f, 60.0f);
  }
  return clampf_(u, -100.0f, 100.0f);
}

void drive_update(float dt) {
  /* odometry */
  int32_t c = encoder_count();
  int32_t dc = c - drv.lastCount;
  drv.lastCount = c;
  drv.lastDelta = dc;
  if (dc == 0) { if (drv.stillTicks < 60000) drv.stillTicks++; } else drv.stillTicks = 0;
  float vRaw = (dc / COUNTS_PER_METER) / dt;
  drv.speed += SPEED_LPF * (vRaw - drv.speed);
  drv.distance = c / COUNTS_PER_METER;

  /* distance move: plan the speed from the remaining distance */
  if (drv.mode == DM_DISTANCE) {
    float remaining = drv.moveGoal - drv.distance;
    float dir = (drv.moveGoal >= drv.moveStart) ? 1.0f : -1.0f;
    if (remaining * dir <= MOVE_STOP_TOL) { drive_stop(); drv.moveDone = true; return; }
    float v = sqrtf(2.0f * MOVE_DECEL * fabsf(remaining));
    v = clampf_(v, MOVE_CREEP, drv.moveCruise);
    drv.target = dir * v;
  }

  switch (drv.mode) {
    case DM_STOP: motor_brake(); break;
    case DM_OPEN: motor_set(drv.openPct); break;
    case DM_SPEED:
    case DM_DISTANCE: {
      float step = MAX_ACCEL * dt;
      drv.setpoint += clampf_(drv.target - drv.setpoint, -step, step);
      if (drv.mode == DM_DISTANCE && fabsf(drv.target) < fabsf(drv.setpoint)) drv.setpoint = drv.target;
      if (drv.mode == DM_SPEED && fabsf(drv.target) < 0.005f && fabsf(drv.setpoint) < 0.005f) {
        motor_brake(); drv.integ = 0;
      } else {
        motor_set(speed_pi(drv.setpoint, drv.speed, dt));
      }
      break;
    }
  }
}
