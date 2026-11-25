/* =====================================================================
 * car_speedtest.c - "vs" speed steps, see car_speedtest.h
 *
 * Everything runs in the main loop. It only sets the speed the speed loop
 * holds (Set_Speed_cm_s) and the heading the steering holds; the motor
 * control stays in the TIM3 interrupt.
 *
 * Averages are taken once per control step (10 Hz) after VS_SETTLE_S of
 * each step, so the result shows the motor % the surface needs at that
 * speed, not the kick to get there. Stops for the sonar like a run does.
 * ===================================================================== */
#include "main.h"
#include "car_speedtest.h"
#include "car_config.h"
#include "car_hw.h"
#include "car_comms.h"
#include "car_drive.h"
#include "car_heading.h"
#include <math.h>

static bool     running = false;
static float    stepCms, topCms;
static int      stepNo;
static uint32_t stepMs, lastStep;
static float    sumSpeed, sumPct, sumBatt;
static int      nAvg;

bool vs_running(void) { return running; }

static void begin_step(void) {
  stepMs = HAL_GetTick();
  sumSpeed = sumPct = sumBatt = 0.0f;
  nAvg = 0;
  Set_Speed_cm_s(stepCms);
}

static void report_step(void) {
  lb_clear(); lb_str("VS step "); lb_int(stepNo); lb_str(" at "); lb_float(stepCms, 0); lb_str(" cm/s: ");
  if (nAvg == 0) lb_str("too short to average");
  else {
    lb_str("speed "); lb_float(sumSpeed / nAvg, 1); lb_str(" cm/s, motor ");
    lb_float(sumPct / nAvg, 1); lb_str(" %, battery "); lb_float(sumBatt / nAvg, 2); lb_str(" V");
  }
  lb_send(false);
}

bool vs_start(float top) {
  if (top <= 0.0f) top = VS_TOP_CMS;
  if (top < VS_FIRST_CMS || top > DRIVE_MAX_CMS) {
    lb_clear(); lb_str("ERR vs <top cm/s>, "); lb_float(VS_FIRST_CMS, 0); lb_str(".."); lb_float(DRIVE_MAX_CMS, 0);
    lb_send(false);
    return false;
  }
  if (state != DRIVE_OFF) { say("ERR a run is going: s first"); return false; }
  topCms = top;
  stepCms = VS_FIRST_CMS;
  stepNo = 1;
  running = true;
  heading_target(yawDeg);   /* hold the way the car points now */
  heading_turn(true);
  lastStep = driveSteps;
  begin_step();
  return true;
}

void vs_stop(const char *why) {
  if (!running) return;
  running = false;
  drive_stop();          /* coast */
  heading_release();
  if (why) { lb_clear(); lb_str("VS stopped: "); lb_str(why); lb_send(false); }
}

void vs_poll(void) {
  if (!running) return;
  if (obstacle) { report_step(); vs_stop("sonar: something in front"); return; }

  if (driveSteps != lastStep) {   /* one control step: add to the averages */
    lastStep = driveSteps;
    if (HAL_GetTick() - stepMs >= (uint32_t)(VS_SETTLE_S * 1000.0f)) {
      sumSpeed += Speed_CM_SEC;
      sumPct += motorPct;
      sumBatt += battery_volts();
      nAvg++;
    }
  }

  if (HAL_GetTick() - stepMs >= (uint32_t)(VS_STEP_S * 1000.0f)) {
    report_step();
    if (stepCms + VS_STEP_CMS > topCms + 0.01f) { vs_stop(NULL); say("VS done"); return; }
    stepCms += VS_STEP_CMS;
    stepNo++;
    begin_step();
  }
}
