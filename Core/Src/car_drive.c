/* =====================================================================
 * car_drive.c - distance run. Started as Hakon's main.c on the
 * hakon-backup branch (Hakons_leg project); the position loop is from
 * Activity 2.6.
 *
 * A run follows a trapezoidal speed profile (M2.9): accelerate for t1,
 * cruise at Sn until t2, decelerate until T. The area under the profile so
 * far is the reference position Pos_Ref. Every 100 ms (TIM3 interrupt) the
 * PID (M2.10) compares Pos_Ref with the encoder distance Real_Dist and adds
 * its correction to SP_Vel, the feed-forward speed command.
 *
 *   command (cm/s) = SP_Vel + Kp*e + Ki*integral(e) + Kd*de/dt,  e = Pos_Ref - Real_Dist
 *   motor %        = MOTOR_MIN_PCT + command * MOTOR_PCT_PER_CMS
 *
 * What keeps it from stalling, winding up and hunting:
 *  - Kick-start. A standing car gets at least MOTOR_START_PCT until the
 *    encoder sees it move, so it never waits for the integral to grow.
 *  - Anti-windup. The integral only grows while the motor can follow: not
 *    while the car is still standing, not at 100 %, not during an obstacle
 *    pause. The I term is also limited to PID_I_MAX_CMS.
 *  - During the profile the motor never drives against the run. A car that
 *    is ahead coasts or brakes until the reference catches up.
 *  - Settle. When the profile ends the car closes the remaining error with
 *    P and D only, in either direction, and stops inside POS_TOL_CM. It
 *    changes direction at most DRIVE_MAX_REVERSALS times, so it cannot
 *    hunt around the target.
 * The HC-SR04 pauses a forward run while something is closer than OBST_STOP_CM.
 *
 * Names and types are kept from the original so the CubeMonitor setup
 * still finds them. Motor and encoder go through car_hw.c.
 * ===================================================================== */
#include "main.h"
#include "car_drive.h"
#include "car_config.h"
#include "car_hw.h"
#include <math.h>

/* Speed Profile Variables (M2.9 PDF). Sn, t1, t2, T are always positive;
 * the direction of the run is in runDir. */
volatile float time_ctl = 0.0f; // Current elapsed time tc (seconds)
volatile float Sn = 18.0f;   // Nominal Speed in cm/s
float t1 = 2.0f;             // Acceleration time 'a' (seconds)
float d_ramp = 2.0f;         // Deceleration time 'd' (seconds)
float t2 = 8.0f;             // Time when deceleration starts: T - d (seconds)
volatile float T = 10.0f;    // Total movement time (seconds)
volatile float SP_Vel = 0.0f;         // Speed setpoint command (cm/s), negative in reverse
volatile int state = 0; // 0 = Off, 1 = Area 1 (Accel), 2 = Area 2 (Cruise), 3 = Area 3 (Decel), 4 = Settle
static float runDir = 1.0f;           // +1 = forward run, -1 = reverse run

/* Encoder & Distance Measurement Variables */
volatile int32_t position = 0;        // Encoder counts since the start of the run, signed
int32_t last_pos = 0;
volatile float Real_Dist = 0.0f;      // Traveled distance in cm, negative = backwards
volatile float target_dist_cm = 0.0f; // Target distance in cm
volatile float Speed_CM_SEC = 0.0f;   // Raw speed in cm/s
volatile float AverageSpeed = 0.0f;   // Filtered speed (0.5 * prev + 0.5 * raw)

/* PID Control Variables (M2.10 PDF) */
float elapsedTime = 0.0f;
float error = 0.0f;
float lastError = 0.0f;
float ErrorAcum = 0.0f;
float ErrorRate = 0.0f;

/* Gains: position error (cm) -> speed correction (cm/s). Hakon's values
 * from the car; replace them with the MATLAB tuning. tools/drive_sim tries
 * other values on a PC. Volatile for CubeMonitor tuning. */
volatile float Kp = 3.0f;
volatile float Ki = 2.0f;
volatile float Kd = 0.0f;

/* Function to enter distance with computer (CubeMonitor trigger) */
volatile float test_dist_m = 0.0f;

/* Position loop and timed run (Activity 2.6) */
volatile float Pos_Ref = 0.0f;        // Reference position: area under SP_Vel so far (cm)
volatile float run_T  = 0.0f;         // Timed run: total time (s), 0 = nominal speed instead
volatile float run_ta = DRIVE_RAMP_S; // Timed run: acceleration time (s)
volatile float run_td = DRIVE_RAMP_S; // Timed run: deceleration time (s)
volatile bool  obstacle = false;      // Run paused: something closer than OBST_STOP_CM
volatile uint32_t driveSteps = 0;     // Control steps so far
static int obstHits = 0;              // Close readings in a row

/* Motor output and anti-windup */
static float openLoopCms = 0.0f;      // "v" command: hold this speed without the PID
static bool  intFreeze = false;       // The motor can't follow: don't integrate
static int   outSat = 0;              // +1 / -1 while the motor is at +100 % / -100 %
static float lastDriveDir = 1.0f;     // Direction the motor last drove in
static int   reversals = 0;           // Changes of direction while settling
static float settleTime = 0.0f;       // Seconds since the profile ended
static float stallTime = 0.0f;        // Seconds driven without the car moving
static volatile int runResult = RUN_NONE;

static float ticks_per_cm(void) { return COUNTS_PER_METER / 100.0f; }
static float signf_(float v) { return v < 0.0f ? -1.0f : 1.0f; }

/* Clear everything a run measures, before a new run starts */
static void reset_run(void) {
  /* Reset encoder and tracking variables */
  encoder_zero();
  position = 0;
  last_pos = 0;
  Real_Dist = 0.0f;
  Pos_Ref = 0.0f;
  SP_Vel = 0.0f;
  Speed_CM_SEC = 0.0f;
  AverageSpeed = 0.0f;
  obstacle = false;
  obstHits = 0;
  openLoopCms = 0.0f;

  /* Reset PID state (M2.10 Slide 11) */
  error = 0.0f;
  ErrorAcum = 0.0f;
  lastError = 0.0f;
  ErrorRate = 0.0f;
  intFreeze = false;
  outSat = 0;
  reversals = 0;
  settleTime = 0.0f;
  stallTime = 0.0f;
  runResult = RUN_NONE;
}

/**
 * @brief Configures the Trapezoidal Speed Profile for a target distance in meters
 *        Uses the formulas from M2.9_CREATING A SPEED PROFILE
 *        A negative distance is the same profile driven in reverse.
 */
void Start_Motion_Profile(float dist_m, float nominal_cm_s, float accel_s) {
  reset_run();
  runDir = signf_(dist_m);
  lastDriveDir = runDir;

  /* Convert target distance from meters to cm (PDF uses cm and cm/s) */
  target_dist_cm = dist_m * 100.0f;
  float D = fabsf(target_dist_cm);
  Sn = nominal_cm_s;
  t1 = accel_s;      // a = acceleration ramp (seconds)
  d_ramp = accel_s;  // d = deceleration ramp (seconds)

  /* Calculate areas A1 and A3 at nominal speed: A1 + A3 = Sn * (a + d) / 2 */
  float ramp_dist_cm = (Sn * (t1 + d_ramp)) / 2.0f;

  if (D <= ramp_dist_cm) {
    /* Short distance: triangular profile (no constant cruise phase) */
    t1 = sqrtf(D / (Sn / accel_s));
    d_ramp = t1;
    t2 = t1;
    T = t1 + d_ramp;
    /* Recalculate peak speed reached */
    Sn = (2.0f * D) / T;
  } else {
    /* Trapezoidal profile (M2.9 Slide 8 & 12):
     * D = A1 + A2 + A3  =>  A2 = D - (A1 + A3)
     * Cruise time (t2 - t1) = A2 / Sn
     */
    float cruise_dist_cm = D - ramp_dist_cm;
    float cruise_time = cruise_dist_cm / Sn;
    t2 = t1 + cruise_time;
    T = t2 + d_ramp;
  }

  time_ctl = 0.0f;
  state = DRIVE_ACCEL; // Start in Area 1 (Acceleration)
}

/**
 * @brief Trapezoidal profile that covers dist_m in exactly total_s seconds,
 *        with its own acceleration and deceleration times (Activity 2.6).
 *        D = A1 + A2 + A3 = Sn*a/2 + Sn*(T - a - d) + Sn*d/2
 *          => Sn = D / (T - (a + d)/2)
 * @return false (and nothing starts) if the times don't fit or Sn is too fast
 */
bool Start_Timed_Profile(float dist_m, float total_s, float accel_s, float decel_s) {
  float D = fabsf(dist_m) * 100.0f;
  if (D < 1.0f || accel_s < 0.1f || decel_s < 0.1f || total_s < accel_s + decel_s)
    return false;
  float sn = D / (total_s - 0.5f * (accel_s + decel_s));
  if (sn > DRIVE_MAX_CMS)
    return false;

  reset_run();
  runDir = signf_(dist_m);
  lastDriveDir = runDir;
  target_dist_cm = dist_m * 100.0f;
  Sn = sn;
  t1 = accel_s;
  d_ramp = decel_s;
  T = total_s;
  t2 = T - d_ramp;
  time_ctl = 0.0f;
  state = DRIVE_ACCEL; // Start in Area 1 (Acceleration)
  return true;
}

/* Every way of starting a run comes here. TIM3 is held off while the run
 * is set up, so the interrupt never sees a half-reset run. */
bool drive_start(float dist_m) {
  bool ok = true;
  if (fabsf(dist_m) < 0.01f) return false;
  HAL_NVIC_DisableIRQ(TIM3_IRQn);
  if (run_T > 0.0f) ok = Start_Timed_Profile(dist_m, run_T, run_ta, run_td);
  else Start_Motion_Profile(dist_m, DRIVE_SPEED_CMS, DRIVE_RAMP_S);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  return ok;
}

/**
 * @brief Discrete PID Controller (M2.10 Slide 11 & 21)
 * @return Control output (in cm/s command space)
 */
float ControlPID(float setpoint, float input, float dt) {
  /* 1. Obtain the error: Setpoint - process variable measure (sensor) */
  error = setpoint - input;

  /* 2. Obtain accumulated error (Integral term).
   * Anti-windup: only while the motor can follow the command. A standing
   * car or a motor at 100 % would only store up an overshoot for later.
   * Integrating is always allowed when it makes the integral smaller. */
  bool unwinding = (error * ErrorAcum < 0.0f);
  bool saturated = (outSat != 0 && error * (float) outSat > 0.0f);
  if (unwinding || (!intFreeze && !saturated))
    ErrorAcum = ErrorAcum + (error * dt);

  /* Limit the I term itself to PID_I_MAX_CMS, whatever Ki is */
  float ki = Ki;
  if (ki > 0.0f) {
    float acumMax = PID_I_MAX_CMS / ki;
    if (ErrorAcum > acumMax)
      ErrorAcum = acumMax;
    if (ErrorAcum < -acumMax)
      ErrorAcum = -acumMax;
  } else {
    ErrorAcum = 0.0f;
  }

  /* 3. Obtain derivative error: ErrorRate */
  if (dt > 0.0f) {
    ErrorRate = (error - lastError) / dt;
  } else {
    ErrorRate = 0.0f;
  }

  /* 4. Compute manipulation value according to PID equation */
  float output = (Kp * error) + (ki * ErrorAcum) + (Kd * ErrorRate);

  /* 5. Update variables for next iteration */
  lastError = error;

  return output;
}

/**
 * @brief Converts a speed command (cm/s, either direction) into motor %
 *        Includes the minimum motor voltage offset (M2.9 Slide 36-38 & M2.10 Slide 22)
 * @return true if the motor drives, false if it coasts or brakes
 */
static bool Apply_Motor_PWM(float control_cm_s) {
  float dir = signf_(control_cm_s);
  bool moving = fabsf(Speed_CM_SEC) >= DRIVE_MOVING_CMS;
  outSat = 0;
  intFreeze = false;

  if (fabsf(control_cm_s) < DRIVE_CMD_MIN_CMS) {
    motor_set(0.0f);   // coast: IN1 = IN2 = low, no PWM
    return false;
  }
  if (moving && Speed_CM_SEC * dir < 0.0f) {
    /* Still rolling the other way: stop first, drive when it stands */
    motor_brake();
    intFreeze = true;
    return false;
  }

  float pct = MOTOR_MIN_PCT + fabsf(control_cm_s) * MOTOR_PCT_PER_CMS;
  if (!moving) {
    /* Kick-start: below MOTOR_START_PCT a standing car does not move at all.
     * If it still stands (low battery, carpet), push harder every step. */
    float kick = MOTOR_START_PCT + KICK_RAMP_PCT_S * stallTime;
    if (pct < kick)
      pct = kick;
    intFreeze = true;
    stallTime += DRIVE_PERIOD_MS / 1000.0f;
  } else {
    stallTime = 0.0f;
  }
  if (pct >= 100.0f) {
    pct = 100.0f;
    outSat = (int) dir;
  }
  motor_set(dir * pct);
  return true;
}

void Set_Speed_cm_s(float target_cm_s) {
  drive_cancel();
  if (target_cm_s == 0.0f) {
    motor_set(0.0f);
    return;
  }
  stallTime = 0.0f;
  openLoopCms = target_cm_s;   // applied by drive_update(), which also kick-starts
}

void drive_cancel(void) {
  state = DRIVE_OFF;
  SP_Vel = 0.0f;
  openLoopCms = 0.0f;
}

void drive_stop(void) {
  drive_cancel();
  ErrorAcum = 0.0f;
  motor_set(0.0f);
}

int drive_result(void) {
  int r = runResult;
  if (r != RUN_NONE) runResult = RUN_NONE;
  return r;
}

/* The run is over: let go of the motor and tell the main loop why */
static void finish_run(int result) {
  state = DRIVE_OFF;
  SP_Vel = 0.0f;
  openLoopCms = 0.0f;
  ErrorAcum = 0.0f;
  motor_set(0.0f);
  runResult = result;
}

/* Speed and position of the profile at time_ctl, as positive numbers.
 * The position is the exact area under the speed curve (M2.9 Slide 8). */
static void profile_at(float tc, float *speed, float *dist) {
  float D = fabsf(target_dist_cm);
  if (tc < t1) {                 /* Area-1: First triangle (Acceleration) */
    *speed = (tc * Sn) / t1;
    *dist = 0.5f * (*speed) * tc;
    state = DRIVE_ACCEL;
  } else if (tc < t2) {          /* Area-2: Rectangle (Nominal Speed Cruise) */
    *speed = Sn;
    *dist = 0.5f * Sn * t1 + Sn * (tc - t1);
    state = DRIVE_CRUISE;
  } else if (tc < T) {           /* Area-3: Second triangle (Deceleration) */
    *speed = Sn * (T - tc) / (T - t2);
    *dist = D - 0.5f * (*speed) * (T - tc);
    state = DRIVE_DECEL;
  } else {                       /* Profile finished: hold the target */
    *speed = 0.0f;
    *dist = D;
    state = DRIVE_SETTLE;
  }
}

/* Called from HAL_TIM_PeriodElapsedCallback() (main.c) every 100 ms */
void drive_update(void) {
  /* Trigger Option 1: CubeMonitor direct variable input */
  if (test_dist_m > 0.01f) {
    float d = test_dist_m;
    test_dist_m = 0.0f; // Reset trigger
    drive_start(d);
  }

  /* Read wheel encoder counts (TIM5) and the distance driven */
  position = encoder_count();
  Real_Dist = (float) position / ticks_per_cm();

  /* Execute Speed Profile & PID every 100 ms (0.1 sec), per M2.9 Slide 20 & 23.
   * The TIM3 interrupt sets the timing, so the step is always the same. */
  driveSteps++;

  /* 1. Elapsed time in seconds (M2.10 Slide 11): the sampling period */
  elapsedTime = DRIVE_PERIOD_MS / 1000.0f;

  /* 2. Measure Process Variable (Speed_CM_SEC) & Filter (M2.10 Slide 19 & M2.9 Slide 43) */
  int32_t delta_ticks = position - last_pos;
  last_pos = position;

  Speed_CM_SEC = ((float) delta_ticks / ticks_per_cm()) / elapsedTime;
  AverageSpeed = (0.5f * AverageSpeed) + (0.5f * Speed_CM_SEC);
  bool moving = fabsf(Speed_CM_SEC) >= DRIVE_MOVING_CMS;

  /* Obstacle: two close readings in a row to stop, clear to continue.
   * Then start the next sonar measurement (its echo is read before next step). */
  if (sonarCm < OBST_STOP_CM) { if (obstHits < 2) obstHits++; }
  else obstHits = 0;
  if (obstHits >= 2) obstacle = true;
  else if (sonarCm > OBST_CLEAR_CM) obstacle = false;
  sonar_trigger();

  /* No run: only the open-loop "v" command drives from here */
  if (state == DRIVE_OFF) {
    if (openLoopCms != 0.0f) {
      Apply_Motor_PWM(openLoopCms);
      if (stallTime >= DRIVE_STALL_S) finish_run(RUN_STALL);
    }
    return;
  }

  /* Paused by an obstacle: brake, and freeze the profile clock and Pos_Ref,
   * so the run continues from the same point once the way is clear.
   * The sonar looks forward, so a reverse run does not stop for it. */
  if (obstacle && runDir > 0.0f) {
    motor_brake();
    stallTime = 0.0f;
    return;
  }

  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // Check 100ms timing

  /* Safety: counting the wrong way would turn the loop into a runaway */
  if (state != DRIVE_SETTLE && Real_Dist * runDir < -ENC_FAULT_CM) {
    finish_run(RUN_ENC_DIR);
    return;
  }
  /* Safety: wheel blocked or encoder unplugged */
  if (stallTime >= DRIVE_STALL_S) {
    finish_run(RUN_STALL);
    return;
  }

  /* 3. Speed Profile State Machine (M2.9 Slide 23) */
  bool wasSettling = (state == DRIVE_SETTLE);
  float speed, dist;
  time_ctl = time_ctl + elapsedTime;
  profile_at(time_ctl, &speed, &dist);
  SP_Vel = runDir * speed;
  Pos_Ref = runDir * dist;   /* Reference position = area under the speed profile so far */

  if (state != DRIVE_SETTLE) {
    /* Position loop: PID on Pos_Ref - Real_Dist, SP_Vel as feed-forward */
    float pid_correction = ControlPID(Pos_Ref, Real_Dist, elapsedTime);
    float total_command_cm_s = SP_Vel + pid_correction;

    if (total_command_cm_s * runDir > 0.0f) {
      Apply_Motor_PWM(total_command_cm_s);
    } else {
      /* At or ahead of the reference: never drive against the run. Brake if
       * it is far ahead and still rolling, else coast, until the reference
       * catches up. */
      outSat = 0;
      intFreeze = false;
      stallTime = 0.0f;
      if (moving && Speed_CM_SEC * runDir > 0.0f && fabsf(error) > POS_TOL_CM)
        motor_brake();
      else
        motor_set(0.0f);
    }
    return;
  }

  /* 4. Settle: the profile is done, close what is left of the error.
   * P and D only: an integral would push a standing car until it jumps
   * past the target, again and again. */
  if (!wasSettling) {
    ErrorAcum = 0.0f;
    settleTime = 0.0f;
  }
  settleTime += elapsedTime;
  intFreeze = true;
  float command_cm_s = ControlPID(Pos_Ref, Real_Dist, elapsedTime);
  ErrorAcum = 0.0f;

  if (fabsf(error) <= POS_TOL_CM) {
    /* On target: brake until it stands, then the run is done */
    if (moving) motor_brake();
    else finish_run(RUN_DONE);
    return;
  }
  if (settleTime >= DRIVE_SETTLE_S) {
    finish_run(RUN_TIMEOUT);
    return;
  }
  if (!moving) {
    /* Standing outside the tolerance: move towards the target, but only
     * change direction DRIVE_MAX_REVERSALS times */
    float dir = signf_(error);
    if (dir != lastDriveDir && ++reversals > DRIVE_MAX_REVERSALS) {
      finish_run(RUN_SWINGS);
      return;
    }
    lastDriveDir = dir;
    if (command_cm_s * dir < DRIVE_CMD_MIN_CMS)
      command_cm_s = dir * DRIVE_CMD_MIN_CMS;
  }
  Apply_Motor_PWM(command_cm_s);
}
