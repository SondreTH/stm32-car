/* =====================================================================
 * car_drive.c - Hakon's distance run, from main.c on the hakon-backup
 * branch (Hakons_leg project).
 *
 * A run follows a trapezoidal speed profile (M2.9): accelerate for t1,
 * cruise at Sn until t2, decelerate until T. The area under the profile so
 * far is the reference position Pos_Ref. Every 100 ms (TIM3 interrupt) the
 * PID (M2.10) compares Pos_Ref with the encoder distance Real_Dist and adds
 * its correction to SP_Vel, the feed-forward speed command.
 * The HC-SR04 pauses the run while something is closer than OBST_STOP_CM.
 *
 * Names and types are kept from the original so the CubeMonitor setup
 * still finds them. Motor and encoder go through car_hw.c.
 * ===================================================================== */
#include "main.h"
#include "car_drive.h"
#include "car_config.h"
#include "car_hw.h"
#include <math.h>

/* Speed Profile Variables (M2.9 PDF) */
float time_ctl = 0.0f;       // Current elapsed time tc (seconds)
float Sn = 18.0f;            // Nominal Speed in cm/s
float t1 = 2.0f;             // Acceleration time 'a' (seconds)
float d_ramp = 2.0f;         // Deceleration time 'd' (seconds)
float t2 = 8.0f;             // Time when deceleration starts: T - d (seconds)
float T = 10.0f;             // Total movement time (seconds)
volatile float SP_Vel = 0.0f;         // Speed setpoint command (cm/s)
int state = 0; // 0 = Off, 1 = Area 1 (Accel), 2 = Area 2 (Cruise), 3 = Area 3 (Decel)

/* Encoder & Distance Measurement Variables */
volatile int32_t position = 0;        // Encoder counts since the start, either direction
int32_t last_pos = 0;
volatile float Real_Dist = 0.0f;      // Traveled distance in cm
float target_dist_cm = 0.0f;          // Target distance in cm
volatile float Speed_CM_SEC = 0.0f;   // Raw speed in cm/s
volatile float AverageSpeed = 0.0f;   // Filtered speed (0.5 * prev + 0.5 * raw)

/* PID Control Variables (M2.10 PDF) */
float elapsedTime = 0.0f;
float error = 0.0f;
float lastError = 0.0f;
float ErrorAcum = 0.0f;
float ErrorRate = 0.0f;

/* Gains: position error (cm) -> speed correction (cm/s). Starting values,
 * replace them with the MATLAB tuning. Volatile for CubeMonitor tuning. */
volatile float Kp = 3.0f;
volatile float Ki = 2.0f;
volatile float Kd = 0.0f;

/* Function to enter distance with computer (CubeMonitor trigger) */
volatile float test_dist_m = 0.0f;

/* Position loop and timed run (Activity 2.6) */
volatile float Pos_Ref = 0.0f;        // Reference position: area under SP_Vel so far (cm)
volatile float run_T  = 0.0f;         // Timed run: total time (s), 0 = nominal speed Sn instead
volatile float run_ta = DRIVE_RAMP_S; // Timed run: acceleration time (s)
volatile float run_td = DRIVE_RAMP_S; // Timed run: deceleration time (s)
volatile bool  obstacle = false;      // Run paused: something closer than OBST_STOP_CM
volatile uint32_t driveSteps = 0;     // Control steps so far
static int obstHits = 0;              // Close readings in a row

static float ticks_per_cm(void) { return COUNTS_PER_METER / 100.0f; }

/* Clear everything a run measures, before a new run starts */
static void reset_run(void) {
  /* Reset encoder and tracking variables */
  encoder_zero();
  position = 0;
  last_pos = 0;
  Real_Dist = 0.0f;
  Pos_Ref = 0.0f;
  Speed_CM_SEC = 0.0f;
  AverageSpeed = 0.0f;
  obstacle = false;
  obstHits = 0;

  /* Reset PID state (M2.10 Slide 11) */
  ErrorAcum = 0.0f;
  lastError = 0.0f;
}

/**
 * @brief Configures the Trapezoidal Speed Profile for a target distance in meters
 *        Uses the formulas from M2.9_CREATING A SPEED PROFILE
 */
void Start_Motion_Profile(float dist_m, float nominal_cm_s, float accel_s) {
  reset_run();

  /* Convert target distance from meters to cm (PDF uses cm and cm/s) */
  target_dist_cm = dist_m * 100.0f;
  Sn = nominal_cm_s;
  t1 = accel_s;      // a = acceleration ramp (seconds)
  d_ramp = accel_s;  // d = deceleration ramp (seconds)

  /* Calculate areas A1 and A3 at nominal speed: A1 + A3 = Sn * (a + d) / 2 */
  float ramp_dist_cm = (Sn * (t1 + d_ramp)) / 2.0f;

  if (target_dist_cm <= ramp_dist_cm) {
    /* Short distance: triangular profile (no constant cruise phase) */
    t1 = sqrtf(target_dist_cm / (Sn / accel_s));
    d_ramp = t1;
    t2 = t1;
    T = t1 + d_ramp;
    /* Recalculate peak speed reached */
    Sn = (2.0f * target_dist_cm) / T;
  } else {
    /* Trapezoidal profile (M2.9 Slide 8 & 12):
     * D = A1 + A2 + A3  =>  A2 = D - (A1 + A3)
     * Cruise time (t2 - t1) = A2 / Sn
     */
    float cruise_dist_cm = target_dist_cm - ramp_dist_cm;
    float cruise_time = cruise_dist_cm / Sn;
    t2 = t1 + cruise_time;
    T = t2 + d_ramp;
  }

  time_ctl = 0.0f;
  state = 1; // Start in Area 1 (Acceleration)
}

/**
 * @brief Trapezoidal profile that covers dist_m in exactly total_s seconds,
 *        with its own acceleration and deceleration times (Activity 2.6).
 *        D = A1 + A2 + A3 = Sn*a/2 + Sn*(T - a - d) + Sn*d/2
 *          => Sn = D / (T - (a + d)/2)
 * @return false (and nothing starts) if the times don't fit or Sn is too fast
 */
bool Start_Timed_Profile(float dist_m, float total_s, float accel_s, float decel_s) {
  if (dist_m <= 0.0f || accel_s < 0.1f || decel_s < 0.1f || total_s < accel_s + decel_s)
    return false;
  float sn = (dist_m * 100.0f) / (total_s - 0.5f * (accel_s + decel_s));
  if (sn > DRIVE_MAX_CMS)
    return false;

  reset_run();
  target_dist_cm = dist_m * 100.0f;
  Sn = sn;
  t1 = accel_s;
  d_ramp = decel_s;
  T = total_s;
  t2 = T - d_ramp;
  time_ctl = 0.0f;
  state = 1; // Start in Area 1 (Acceleration)
  return true;
}

/* Every way of starting a run comes here. TIM3 is held off while the run
 * is set up, so the interrupt never sees a half-reset run. */
bool drive_start(float dist_m) {
  bool ok = true;
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

  /* 2. Obtain accumulated error (Integral term) */
  ErrorAcum = ErrorAcum + (error * dt);

  /* Anti-windup limit on integral term (tightened to prevent overshoot) */
  if (ErrorAcum > 15.0f)
    ErrorAcum = 15.0f;
  if (ErrorAcum < -15.0f)
    ErrorAcum = -15.0f;

  /* 3. Obtain derivative error: ErrorRate */
  if (dt > 0.0f) {
    ErrorRate = (error - lastError) / dt;
  } else {
    ErrorRate = 0.0f;
  }

  /* 4. Compute manipulation value according to PID equation */
  float output = (Kp * error) + (Ki * ErrorAcum) + (Kd * ErrorRate);

  /* 5. Update variables for next iteration */
  lastError = error;

  return output;
}

/**
 * @brief Converts control speed command (cm/s) + PID correction into motor %
 *        Includes the minimum motor voltage offset (M2.9 Slide 36-38 & M2.10 Slide 22)
 */
void Apply_Motor_PWM(float control_cm_s) {
  if (control_cm_s <= 0.2f) {
    motor_set(0.0f);   // coast: IN1 = IN2 = low, no PWM
    return;
  }
  /* Lower base offset (~45% duty) so PID can control slow 18 cm/s speeds.
   * motor_set() clamps to 100 %. */
  motor_set(MOTOR_MIN_PCT + control_cm_s * MOTOR_PCT_PER_CMS);
}

void Set_Speed_cm_s(float target_cm_s) {
  state = 0;
  if (target_cm_s <= 0.0f) {
    motor_set(0.0f);
    return;
  }
  Apply_Motor_PWM(target_cm_s);
}

void drive_cancel(void) {
  state = 0;
  SP_Vel = 0.0f;
}

void drive_stop(void) {
  drive_cancel();
  ErrorAcum = 0.0f;
  Apply_Motor_PWM(0.0f);
}

/* Called from HAL_TIM_PeriodElapsedCallback() (main.c) every 100 ms */
void drive_update(void) {
  /* Read wheel encoder counts (TIM5) and the distance driven */
  int32_t cnt = encoder_count();
  position = (cnt < 0) ? -cnt : cnt;
  Real_Dist = (float) position / ticks_per_cm();

  /* Trigger Option 1: CubeMonitor direct variable input */
  if (test_dist_m > 0.01f) {
    float d = test_dist_m;
    test_dist_m = 0.0f; // Reset trigger
    drive_start(d);
  }

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

  /* Obstacle: two close readings in a row to stop, clear to continue.
   * Then start the next sonar measurement (its echo is read before next step). */
  if (sonarCm < OBST_STOP_CM) { if (obstHits < 2) obstHits++; }
  else obstHits = 0;
  if (obstHits >= 2) obstacle = true;
  else if (sonarCm > OBST_CLEAR_CM) obstacle = false;
  sonar_trigger();

  /* Paused by an obstacle: brake, and freeze the profile clock and Pos_Ref,
   * so the run continues from the same point once the way is clear */
  if (state != 0 && obstacle) {
    motor_brake();
    return;
  }

  /* 3. Speed Profile State Machine (M2.9 Slide 23) */
  if (state != 0) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // Check 100ms timing
    time_ctl = time_ctl + elapsedTime;

    switch (state) {
    case 1: /* Area-1: First triangle (Acceleration) */
      SP_Vel = (time_ctl * Sn) / t1;
      if (time_ctl >= t1)
        state = 2;
      break;

    case 2: /* Area-2: Rectangle (Nominal Speed Cruise) */
      SP_Vel = Sn;
      if (time_ctl >= t2)
        state = 3;
      break;

    case 3: /* Area-3: Second triangle (Deceleration) */
      SP_Vel = Sn * (T - time_ctl) / (T - t2);
      if (time_ctl >= T) {
        state = 0;
        SP_Vel = 0.0f;
      }
      break;
    }

    /* Reference position = area under the speed profile so far */
    Pos_Ref = Pos_Ref + SP_Vel * elapsedTime;

    if (state == 0) {
      SP_Vel = 0.0f;
      Pos_Ref = target_dist_cm;
      Apply_Motor_PWM(0.0f);
    } else {
      /* Position loop: PID on Pos_Ref - Real_Dist, SP_Vel as feed-forward */
      float pid_correction = ControlPID(Pos_Ref, Real_Dist, elapsedTime);
      float total_command_cm_s = SP_Vel + pid_correction;
      if (total_command_cm_s < 0.5f)
        total_command_cm_s = 0.5f;
      Apply_Motor_PWM(total_command_cm_s);
    }
  }
}
