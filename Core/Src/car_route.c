/* =====================================================================
 * car_route.c - laps: a list of steps driven one after the other
 *
 * A lap is a list of steps:
 *   S <m>    straight: a normal distance run (speed profile + position PID,
 *            car_drive.c). The heading hold steers it along the route
 *            heading, not "whatever the car pointed at when it started".
 *   T <deg>  turn: + = left, - = right. The route heading moves by <deg>,
 *            and the car drives forward at TURN_SPEED_CMS (speed loop)
 *            while the gyro steering turns it to the new heading. Far from
 *            it the servo is at full lock, so the curve is the tightest the
 *            car can do. Done ROUTE_TURN_TOL_DEG before the new heading.
 *   W <s>    wait: stand still, e.g. 10 s at a waypoint. While the car
 *            stands, the gyro bias is trimmed again (car_heading.c).
 * Between steps the car stands still for ROUTE_PAUSE_MS.
 *
 * The route heading is the sum of the turns since "go". Because the gyro
 * measures the heading over the whole route, a turn that stops 3 degrees
 * short is corrected on the next straight; the errors do not add up from
 * lap to lap (only the gyro drift does).
 *
 * What it reports, for testing the car:
 *   every step:  distance, time, heading error (straights), the angle
 *                turned and the radius of the curve (turns)
 *   every lap:   time, distance, heading at the end, worst heading error
 *   at the end:  total and average lap time
 * Measure by hand how far from the start mark the car ends: the firmware
 * does not know x/y yet.
 *
 * Everything here runs in the main loop. It only starts and stops things
 * (drive_start, Set_Speed_cm_s, drive_stop); the motor control stays in
 * the TIM3 interrupt.
 * ===================================================================== */
#include "main.h"
#include "car_route.h"
#include "car_config.h"
#include "car_hw.h"
#include "car_comms.h"
#include "car_drive.h"
#include "car_heading.h"
#include <math.h>
#include <stddef.h>

typedef struct {
  char  type;    /* STEP_STRAIGHT, STEP_TURN, STEP_WAIT */
  float value;   /* metres, degrees or seconds          */
} RouteStep;

static RouteStep steps[ROUTE_MAX_STEPS];
static int nSteps = 0;

/* what the route is doing */
#define R_OFF       0
#define R_STRAIGHT  1   /* a distance run is going on                 */
#define R_TURN      2
#define R_WAIT      3
#define R_PAUSE     4   /* between two steps: wait until the car stands */
static int phase = R_OFF;

static int   stepIdx = 0, lap = 0, laps = 0;
static float routeHeading = 0.0f;   /* heading the route wants now, degrees      */
static int   pendingResult = RUN_NONE;
static uint32_t routeMs, lapMs, stepMs, pauseMs;
static float stepMaxErr;            /* worst |heading error| in this straight,
                                       after the first ROUTE_SETTLE_CM          */
static float stepStartErr;          /* heading error when the straight started  */
static float lapMaxErr, lapCm;
static float turnYaw0;              /* heading when the turn started            */
static int32_t turnCnt0;            /* encoder counts when the turn started     */
static bool  turnPaused;            /* the turn is held up by an obstacle       */

/* the first part of a straight takes out what the turn left over; the
 * "worst heading error" of a straight is only counted after it */
#define ROUTE_SETTLE_CM 30.0f

static float signf_(float v) { return v < 0.0f ? -1.0f : 1.0f; }

/* ------------------------------------------------------------ the lap */
void route_clear(void) { nSteps = 0; }

bool route_add(char type, float value) {
  if (nSteps >= ROUTE_MAX_STEPS) return false;
  steps[nSteps].type = type;
  steps[nSteps].value = value;
  nSteps++;
  return true;
}

/* A 2 m square, left turns. Needs a clear floor of about 3 x 3 m (the
 * curves cut the corners). Change it with rc / rs / rt / rw. */
void route_default(void) {
  route_clear();
  for (int i = 0; i < 4; i++) {
    route_add(STEP_STRAIGHT, 2.0f);
    route_add(STEP_TURN, 90.0f);
  }
}

void route_init(void) { route_default(); }

static void print_step(int i) {
  lb_str("#"); lb_int(i + 1); lb_str(" ");
  if (steps[i].type == STEP_STRAIGHT) { lb_str("straight "); lb_float(steps[i].value, 2); lb_str(" m"); }
  else if (steps[i].type == STEP_TURN) {
    lb_str("turn "); lb_float(fabsf(steps[i].value), 0); lb_str(steps[i].value > 0 ? " deg left" : " deg right");
  } else { lb_str("wait "); lb_float(steps[i].value, 1); lb_str(" s"); }
}

void route_list(void) {
  float m = 0.0f, deg = 0.0f, wait = 0.0f;
  for (int i = 0; i < nSteps; i++) {
    if (steps[i].type == STEP_STRAIGHT) m += steps[i].value;
    else if (steps[i].type == STEP_TURN) deg += steps[i].value;
    else wait += steps[i].value;
  }
  lb_clear(); lb_str("lap: "); lb_int(nSteps); lb_str(" steps, "); lb_float(m, 2); lb_str(" m straight, turns add up to ");
  lb_float(deg, 0); lb_str(" deg, "); lb_float(wait, 0); lb_str(" s waiting"); lb_send(false);
  for (int i = 0; i < nSteps; i++) { lb_clear(); lb_str("  "); print_step(i); lb_send(false); }
  if (nSteps > 0 && fabsf(fabsf(deg) - 360.0f) > 1.0f && fabsf(deg) > 1.0f)
    say("  note: the turns do not add up to 360 deg, so the lap does not end facing the start direction");
}

/* ------------------------------------------------------------ running */
bool route_running(void) { return phase != R_OFF; }
int  route_lap(void)     { return lap; }
int  route_step(void)    { return phase == R_OFF ? 0 : stepIdx + 1; }

const char *route_phase_name(void) {
  if (phase == R_TURN) return turnPaused ? "OBSTACLE" : "TURN";
  if (phase == R_WAIT) return "WAIT";
  return 0;
}

void route_run_ended(int result) {
  if (phase != R_OFF) pendingResult = result;
}

void route_stop(const char *why) {
  if (phase == R_OFF) return;
  phase = R_OFF;
  heading_release();
  drive_stop();
  servo_set_us((uint32_t)SERVO_CENTER_US);
  if (why) say(why);
}

static void begin_step(void) {
  RouteStep *s = &steps[stepIdx];
  pendingResult = RUN_NONE;
  stepMs = HAL_GetTick();
  stepMaxErr = 0.0f;
  stepStartErr = yawDeg - routeHeading;
  heading_turn(false);   /* a turn steers until the car stands; the next step decides */

  if (s->type == STEP_STRAIGHT) {
    heading_target(routeHeading);
    if (!drive_start(s->value)) { route_stop("ROUTE stopped: the straight does not fit the run settings"); return; }
    phase = R_STRAIGHT;
  } else if (s->type == STEP_TURN) {
    routeHeading += s->value;
    heading_target(routeHeading);
    heading_turn(true);
    turnYaw0 = yawDeg;
    turnCnt0 = encoder_count();
    turnPaused = false;
    Set_Speed_cm_s(TURN_SPEED_CMS);
    phase = R_TURN;
  } else {
    drive_stop();
    phase = R_WAIT;
  }
}

/* "L1 #3 " in front of every report line */
static void step_prefix(void) {
  lb_clear(); lb_str("L"); lb_int(lap); lb_str(" #"); lb_int(stepIdx + 1); lb_str(" ");
}

/* a step is over: report the lap if it was the last step, then pause */
static void end_step(float cm) {
  uint32_t now = HAL_GetTick();
  lapCm += cm;
  stepIdx++;
  if (stepIdx >= nSteps) {
    float off = yawDeg - routeHeading;
    lb_clear(); lb_str("LAP "); lb_int(lap); lb_str(" of "); lb_int(laps); lb_str(": ");
    lb_float((now - lapMs) / 1000.0f, 1); lb_str(" s, "); lb_float(lapCm, 0); lb_str(" cm, heading off by ");
    lb_float(off, 1); lb_str(" deg, worst on a straight "); lb_float(lapMaxErr, 1); lb_str(" deg, batt ");
    lb_float(battery_volts(), 1); lb_str(" V");
    lb_send(false);
    if (lap >= laps) {
      float total = (now - routeMs) / 1000.0f;
      lb_clear(); lb_str("ROUTE DONE: "); lb_int(laps); lb_str(laps == 1 ? " lap in " : " laps in ");
      lb_float(total, 1); lb_str(" s, "); lb_float(total / laps, 1); lb_str(" s per lap. Measure how far the car is from the start mark");
      lb_send(false);
      route_stop(NULL);
      return;
    }
    lap++;
    stepIdx = 0;
    lapMs = now;
    lapCm = 0.0f;
    lapMaxErr = 0.0f;
  }
  phase = R_PAUSE;
  pauseMs = now;
}

bool route_start(int nLaps) {
  if (nSteps == 0) { say("ERR the lap is empty: rd loads the test square, or rs / rt / rw add steps"); return false; }
  if (!imuOk)      { say("ERR a route needs the gyro: check wiring, then type I"); return false; }
  if (run_T > 0.0f) { say("ERR a route drives at the nominal speed: set tt 0 first"); return false; }
  if (STEER_KH == 0.0f) { say("ERR heading hold is off (kh 0): the turns need it"); return false; }
  if (nLaps < 1) nLaps = 1;
  laps = nLaps;
  lap = 1;
  stepIdx = 0;
  routeHeading = yawDeg;   /* the route starts in the direction the car points */
  routeMs = lapMs = HAL_GetTick();
  lapCm = 0.0f;
  lapMaxErr = 0.0f;
  begin_step();
  return phase != R_OFF;
}

void route_poll(void) {
  if (phase == R_OFF) return;
  uint32_t now = HAL_GetTick();
  RouteStep *s = &steps[stepIdx];

  if (phase == R_STRAIGHT) {
    if (state != DRIVE_OFF && fabsf(Real_Dist) > ROUTE_SETTLE_CM) {
      float e = fabsf(headingErrDeg);
      if (e > stepMaxErr) stepMaxErr = e;
      if (e > lapMaxErr) lapMaxErr = e;
    }
    if (pendingResult == RUN_NONE) return;
    if (pendingResult >= RUN_STALL) { route_stop("ROUTE stopped: the straight failed (see above)"); return; }
    step_prefix(); lb_str("straight "); lb_float(s->value, 2); lb_str(" m: "); lb_float(Real_Dist, 1);
    lb_str(" cm in "); lb_float((now - stepMs) / 1000.0f, 1); lb_str(" s, heading (deg) at start ");
    lb_float(stepStartErr, 1); lb_str(" worst "); lb_float(stepMaxErr, 1); lb_str(" end "); lb_float(yawDeg - routeHeading, 1);
    lb_send(false);
    end_step(Real_Dist);
    return;
  }

  if (phase == R_TURN) {
    float cm = (float)(encoder_count() - turnCnt0) * 100.0f / COUNTS_PER_METER;
    float dir = signf_(s->value);
    float left = (routeHeading - yawDeg) * dir;   /* degrees still to turn */
    if (pendingResult == RUN_STALL) { route_stop("ROUTE stopped: wheel blocked in the turn"); return; }
    if (cm > ROUTE_TURN_MAX_CM) { route_stop("ROUTE stopped: the turn never reached its heading. Steering or gyro sign?"); return; }
    if (left > fabsf(s->value) + 30.0f) { route_stop("ROUTE stopped: the car turns the wrong way. Check IMU_YAW_SIGN and kh"); return; }

    /* an obstacle holds up the turn the same way it pauses a straight */
    if (obstacle && !turnPaused) { drive_cancel(); motor_brake(); turnPaused = true; }
    else if (!obstacle && turnPaused) { Set_Speed_cm_s(TURN_SPEED_CMS); turnPaused = false; }

    if (left > ROUTE_TURN_TOL_DEG) return;
    drive_stop();   /* coast to a stop; the steering keeps working until the next step */
    float deg = yawDeg - turnYaw0;
    step_prefix(); lb_str("turn "); lb_float(s->value, 0); lb_str(": "); lb_float(deg, 1); lb_str(" deg in ");
    lb_float(cm, 0); lb_str(" cm, "); lb_float((now - stepMs) / 1000.0f, 1); lb_str(" s");
    if (fabsf(deg) >= 5.0f) { lb_str(", radius "); lb_float(57.3f * cm / fabsf(deg), 0); lb_str(" cm"); }
    lb_send(false);
    end_step(cm);
    return;
  }

  if (phase == R_WAIT) {
    if (now - stepMs < (uint32_t)(s->value * 1000.0f)) return;
    step_prefix(); lb_str("waited "); lb_float(s->value, 1); lb_str(" s, gyro bias now ");
    lb_float(gyroBiasDps, 3); lb_str(" deg/s"); lb_send(false);
    end_step(0.0f);
    return;
  }

  /* R_PAUSE: wait until the car has stood still for ROUTE_PAUSE_MS */
  if (fabsf(Speed_CM_SEC) >= DRIVE_MOVING_CMS) pauseMs = now;
  if (now - pauseMs >= ROUTE_PAUSE_MS) begin_step();
}
