// =====================================================================
//  drive.h  -  odometry (distance + speed) and the PI speed loop
//
//  Modes:
//    STOP      motor braked
//    OPEN      raw % to the motor, no feedback   (bring-up, deadband test)
//    SPEED     closed-loop speed in m/s          (the required control loop)
//    DISTANCE  drive X metres with a slow-down, then brake
// =====================================================================
#pragma once

enum DriveMode { DM_STOP = 0, DM_OPEN = 1, DM_SPEED = 2, DM_DISTANCE = 3 };

struct DriveState {
  DriveMode mode     = DM_STOP;
  float openPct      = 0;     // OPEN mode command
  float target       = 0;     // requested speed, m/s
  float setpoint     = 0;     // ramped speed actually fed to the PI, m/s
  float speed        = 0;     // measured, filtered, m/s
  float distance     = 0;     // m since last zero
  float integ        = 0;     // PI integrator, %
  int32_t lastCount  = 0;
  int32_t lastDelta  = 0;     // counts moved in the last control tick
  uint16_t stillTicks= 0;     // consecutive ticks with no encoder movement
  // odometry (x forward from start, y to the LEFT of start), metres
  float x = 0, y = 0;
  // distance move
  float moveGoal     = 0;     // absolute distance to stop at, m
  float moveCruise   = 0.2f;  // m/s
  float moveStart    = 0;
  bool  moveDone     = false; // set once when a move finishes (main prints it)
} drv;

static void driveStop() {
  drv.mode = DM_STOP;
  drv.target = drv.setpoint = 0;
  drv.integ = 0;
  motorBrake();
}

static void driveOpen(float pct)  { drv.mode = DM_OPEN;  drv.openPct = pct; drv.integ = 0; }
static void driveSpeed(float mps) { drv.mode = DM_SPEED; drv.target = mps; }

static void driveDistance(float meters, float cruise) {
  drv.moveStart  = drv.distance;
  drv.moveGoal   = drv.distance + meters;
  drv.moveCruise = fabsf(cruise);
  drv.moveDone   = false;
  drv.mode       = DM_DISTANCE;
}

static void driveZero() {
  encoderZero();
  drv.lastCount = 0;
  drv.distance  = 0;
  drv.x = drv.y = 0;
}

// Car is certainly not moving: motor stopped and wheels still for 0.3 s.
static bool driveIsStill() {
  return drv.mode == DM_STOP && drv.stillTicks > CONTROL_HZ * 3 / 10;
}

// Dead reckoning: encoder gives the distance step, gyro gives the direction.
// Call right after driveUpdate() and imuUpdate(), with the heading from
// BEFORE and AFTER this tick (midpoint rule = less error in curves).
static void odometryUpdate(float headingBeforeDeg, float headingAfterDeg) {
  float ds = drv.lastDelta / COUNTS_PER_METER;
  float th = 0.5f * (headingBeforeDeg + headingAfterDeg) * DEG_TO_RAD;
  drv.x += ds * cosf(th);
  drv.y += ds * sinf(th);
}

// PI speed controller with feed-forward, deadband compensation and
// anti-windup. Returns motor %.
static float speedPI(float sp, float meas, float dt) {
  if (fabsf(sp) < 0.005f) { drv.integ = 0; return 0; }
  float err = sp - meas;
  float u = SPEED_FF * sp
          + (sp > 0 ? MOTOR_DEADBAND_PCT : -MOTOR_DEADBAND_PCT)
          + SPEED_KP * err
          + drv.integ;
  // only integrate when not saturated (or when the error pulls us back)
  if (fabsf(u) < 100.0f || ((u > 0) != (err > 0))) {
    drv.integ += SPEED_KI * err * dt;
    drv.integ = constrain(drv.integ, -60.0f, 60.0f);
  }
  return constrain(u, -100.0f, 100.0f);
}

// Call at CONTROL_HZ.
static void driveUpdate(float dt) {
  // ---- odometry ----
  int32_t c  = encoderCount();
  int32_t dc = c - drv.lastCount;
  drv.lastCount = c;
  drv.lastDelta = dc;
  if (dc == 0) { if (drv.stillTicks < 60000) drv.stillTicks++; }
  else drv.stillTicks = 0;
  float vRaw = (dc / COUNTS_PER_METER) / dt;
  drv.speed += SPEED_LPF * (vRaw - drv.speed);
  drv.distance = c / COUNTS_PER_METER;

  // ---- distance move: plan the speed from the remaining distance ----
  if (drv.mode == DM_DISTANCE) {
    float remaining = drv.moveGoal - drv.distance;
    float dir = (drv.moveGoal >= drv.moveStart) ? 1.0f : -1.0f;
    if (remaining * dir <= MOVE_STOP_TOL) {        // arrived (or overshot)
      driveStop();
      drv.moveDone = true;
      return;
    }
    float v = sqrtf(2.0f * MOVE_DECEL * fabsf(remaining));   // v^2 = 2*a*d
    v = constrain(v, MOVE_CREEP, drv.moveCruise);
    drv.target = dir * v;
  }

  // ---- motor output ----
  switch (drv.mode) {
    case DM_STOP:
      motorBrake();
      break;
    case DM_OPEN:
      motorSet(drv.openPct);
      break;
    case DM_SPEED:
    case DM_DISTANCE: {
      // ramp the setpoint (smooth starts, no wheel spin)
      float step = MAX_ACCEL * dt;
      float diff = drv.target - drv.setpoint;
      drv.setpoint += constrain(diff, -step, step);
      // in DISTANCE mode, slowing down must not be limited by the ramp
      if (drv.mode == DM_DISTANCE && fabsf(drv.target) < fabsf(drv.setpoint))
        drv.setpoint = drv.target;
      if (drv.mode == DM_SPEED && fabsf(drv.target) < 0.005f && fabsf(drv.setpoint) < 0.005f) {
        motorBrake();
        drv.integ = 0;
      } else {
        motorSet(speedPI(drv.setpoint, drv.speed, dt));
      }
      break;
    }
  }
}
