/* =====================================================================
 * car_openloop.c - open-loop route (MR2006B activity "open loop control
 * for mobile robots")
 *
 * The plan is a list of steps. Each step holds a motor %, a steering pulse
 * and a time:
 *
 *   for each step i:  motor = pct[i]
 *                     servo = SERVO_CENTER_US + steer[i]
 *                     hold it for secs[i] seconds
 *   motor 0 % brakes (it does not coast)
 *   after the last:   brake OL_BRAKE_MS, coast, wheels straight
 *   "or <n>" plays the whole list n times (the square: one side + one turn, 4 times)
 *
 * Nothing the car measures changes what it does: no encoder, no gyro, no
 * speed loop. The plan is worked out in advance (motor % for the speed,
 * steering pulse for the curve, time for the distance and the angle) and
 * the car only plays it back. That is open-loop control. Whatever pushes
 * the car off (battery voltage, floor, slope, the tight spot in the
 * drivetrain, a hand) goes straight into where it ends up.
 *
 * The encoder and the gyro are only RECORDED, to compare the run with the
 * plan afterwards:
 *  - after the run, one line per step: distance driven and angle turned
 *  - an estimate of where the car stopped, from the encoder distance and
 *    the gyro heading (dead reckoning). x = forward at the start, y = left:
 *        x = x + d * cos(heading),   y = y + d * sin(heading)
 *    It is an estimate: wheel slip and gyro drift are in it too. Mark the
 *    start on the floor and measure the real end with a tape.
 *  - "od" prints the whole recording (10 samples a second) as CSV, and
 *    tools/car_console.py saves it to tools/logs/.
 *
 * Battery compensation ("ov <volts>", off by default). The motor gets on
 * average duty x battery voltage, so a full battery drives faster than a
 * tired one with the same plan. With ov the duty is scaled so the motor
 * always gets what it would get at that voltage:
 *     duty = pct x V_ref / V_battery
 * This is feedforward from a disturbance that can be measured. It is still
 * open loop: the car does not look at its speed or where it is.
 *
 * Two safety stops end the run (brake), they never steer or change the
 * speed: the sonar (something closer than OBST_STOP_CM; "os 0" turns it
 * off) and a blocked wheel (motor on, no encoder counts for DRIVE_STALL_S).
 * The start button and "s" stop it too.
 *
 * Runs in the main loop (HAL_GetTick), not in the TIM3 interrupt. Step
 * times are counted from the start of the run, so the OLED update (~25 ms)
 * makes a step change up to 25 ms late but the delays do not add up.
 * ===================================================================== */
#include "main.h"
#include "car_openloop.h"
#include "car_config.h"
#include "car_hw.h"
#include "car_comms.h"
#include "car_drive.h"
#include "car_heading.h"
#include <math.h>
#include <stdlib.h>
#include <ctype.h>

/* The plan. "ol" shows it, "ol <step> <motor %> <steer us> <s>" changes a
 * step. Copy a plan that works here: a reset brings this one back.
 *
 * The square of 2026-11-22 (.claude/docs/openloop.md has every run): four
 * sides and four right turns of 90 degrees, driven without stopping, back
 * to the start mark. The turns are arcs (Ackermann steering, radius ~65 cm
 * at +350 us), so the corners are round. Smooth floor, battery ~11.2 V.
 *  - kick: the car needs more % to start than to keep rolling
 *  - 46 % in the turns, 40 % on the sides: a turn drags (the tyres scrub)
 *  - -50 us on the sides after a turn: the steering has play, and back at
 *    the centre after a right turn the wheels stay a little right
 *  - the first side is short: the last side and the roll-out finish it
 * Stopping at every corner did not work: the L298N brake is weak, and from
 * standing with the wheels at full lock 35 % cannot move the car.
 * With step 10 at 0 s the car stopped 30 cm short of the mark, at 0.45 s
 * 30-40 cm past it, heading within 1 degree both times. 0.2 s is the
 * interpolation: not run yet. */
OlStep olPlan[OL_MAX_STEPS] = {
  { 55.0f,   0.0f, 0.3f },   /* kick                                  */
  { 40.0f,   0.0f, 0.6f },   /* first side (short)                    */
  { 46.0f, 350.0f, 1.5f },   /* right turn 1                          */
  { 40.0f, -50.0f, 1.3f },   /* side 2                                */
  { 46.0f, 350.0f, 1.5f },   /* right turn 2                          */
  { 40.0f, -50.0f, 1.3f },   /* side 3                                */
  { 46.0f, 350.0f, 1.5f },   /* right turn 3                          */
  { 40.0f, -50.0f, 1.3f },   /* side 4                                */
  { 46.0f, 350.0f, 1.5f },   /* right turn 4                          */
  { 40.0f, -50.0f, 0.2f },   /* end of side 1                         */
  {  0.0f, -50.0f, 1.0f },   /* brake to a stop                       */
};
int olRepeat = 1;
bool olButton    = true;
bool olSonarStop = true;
float olVref     = 0.0f;   /* ov: battery compensation to this voltage, 0 = off */

enum { OL_IDLE, OL_ARMED, OL_RUN, OL_END };
static int      phase = OL_IDLE;
static uint32_t armMs, startMs, endMs;

/* the run: a copy of the plan, so changing it afterwards does not change the report */
static OlStep   runPlan[OL_MAX_RUN];       /* the plan, repeated                   */
static int      runSteps, runPlanSteps, runReps;
static uint32_t stepEndMs[OL_MAX_RUN];     /* ms after the start when step i ends */
static int      stepNow;                   /* index of the step being played      */
static float    runSc;                     /* SERVO_CENTER_US at the start         */
static float    runVref;                   /* olVref at the start                  */

/* measured: only for the report and the recording, never for driving */
static int32_t  lastCnt, stallCnt;
static uint32_t stallMs;
static float    yaw0, distCm, xCm, yCm;
static float    stepDist[OL_MAX_RUN + 1], stepTurn[OL_MAX_RUN + 1];   /* [runSteps] = rolling out */
static float    markDist, markYaw;          /* at the start of the current step */
static int      stepsDone;                  /* steps played to the end          */
static int      standSteps;
static bool     braking;
static float    battStart, battF;
static uint32_t gyroOk0, gyroFail0;          /* gyro reading counters at the start */
static const char *endWhy = "";

/* recording, 10 samples a second */
typedef struct {
  uint16_t t10;       /* time since the start, in 10 ms        */
  uint8_t  step;      /* plan step 1.., 0 = after the plan     */
  int8_t   pct;       /* motor %                               */
  uint16_t servoUs;
  uint16_t sonarCm;
  int16_t  speed10;   /* cm/s x 10, encoder                    */
  int16_t  yaw10;     /* degrees x 10 since the start, + = left */
  float    dist, x, y;   /* cm                                 */
  float    batt;      /* V                                     */
} OlSample;
static OlSample rec[OL_REC_MAX];
static int      recN = 0;
static uint32_t lastDriveStep;
static int      dumpIdx = -1, dumpPort = 0;

static float clampf_(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static float yaw_now(void) { return imuOk ? yawDeg - yaw0 : 0.0f; }

static void servo_offset(float steerUs) {
  servo_set_us((uint32_t)clampf_(SERVO_CENTER_US + steerUs, SERVO_PULSE_MIN, SERVO_PULSE_MAX));
}

/* motor % of step i, with the battery compensation when it is on */
static float step_pct(int i) {
  float pct = runPlan[i].pct;
  if (runVref > 0.0f && battF > BATT_PRESENT_V) pct = pct * runVref / battF;
  return clampf_(pct, -100.0f, 100.0f);
}

/* drive step i: motor 0 % brakes (both motor pins high), it does not coast */
static void step_motor(int i) {
  float pct = step_pct(i);
  if (pct == 0.0f) motor_brake(); else motor_set(pct);
}

static void apply_step(int i) {
  step_motor(i);
  servo_offset(runPlan[i].steerUs);
  markDist = distCm;
  markYaw = yaw_now();
}

int ol_steps(void) {
  int n = 0;
  while (n < OL_MAX_STEPS && olPlan[n].secs > 0.0f) n++;
  return n;
}

bool    ol_busy(void)    { return phase != OL_IDLE; }
bool    ol_dumping(void) { return dumpIdx >= 0; }
int     ol_step_now(void) { return phase == OL_RUN ? stepNow + 1 : 0; }
int     ol_run_steps(void) { return runSteps; }
int32_t ol_armed_ms_left(void) {
  return phase == OL_ARMED ? (int32_t)(OL_DELAY_MS - (HAL_GetTick() - armMs)) : -1;
}
float ol_secs_left(void) {
  if (phase != OL_RUN || runSteps == 0) return 0.0f;
  return (float)(int32_t)(stepEndMs[runSteps - 1] - (HAL_GetTick() - startMs)) / 1000.0f;
}

/* ---------------------------------------------------------------- output */
/* Lines go to both ports, or only to the dump port while dumping */
static void out(bool toDump) { if (toDump) lb_send_to(dumpPort); else lb_send(false); }

static void step_text(const OlStep *s) {
  if (s->pct == 0.0f) lb_str("brake"); else { lb_float(s->pct, 0); lb_str(" %"); }
  lb_str(" steer "); lb_float(s->steerUs, 0); lb_str(" us for ");
  lb_float(s->secs, 2); lb_str(" s");
}

static void print_plan(void) {
  int n = ol_steps();
  float total = 0.0f;
  for (int i = 0; i < n; i++) total += olPlan[i].secs;
  lb_clear(); lb_str("plan: "); lb_int(n); lb_str(" steps, "); lb_float(total, 2); lb_str(" s, played ");
  lb_int(olRepeat); lb_str(olRepeat == 1 ? " time" : " times"); lb_str(". Button runs it: ");
  lb_str(olButton ? "yes" : "no (1 m run)"); lb_str(". Sonar stop: "); lb_str(olSonarStop ? "on" : "off");
  lb_str(". Battery compensation: ");
  if (olVref > 0.0f) { lb_str("to "); lb_float(olVref, 2); lb_str(" V"); } else lb_str("off");
  out(false);
  for (int i = 0; i < n; i++) { lb_clear(); lb_str(" "); lb_int(i + 1); lb_str(": "); step_text(&olPlan[i]); out(false); }
  say("change: ol <step> <motor %> <steer us, - = left> <s>   (s = 0 ends the plan there)");
}

/* the report after a run; prefix "# " when it goes into a dump */
static void report(bool toDump) {
  const char *pre = toDump ? "# " : "";
  float secs = (float)(endMs - startMs) / 1000.0f;
  lb_clear(); lb_str(pre); lb_str("OPEN LOOP done ("); lb_str(endWhy); lb_str("): ");
  lb_float(secs, 1); lb_str(" s, "); lb_float(distCm, 0); lb_str(" cm");
  if (imuOk) { lb_str(", turned "); lb_float(yaw_now(), 0); lb_str(" deg"); }
  lb_str(". Battery "); lb_float(battStart, 2); lb_str(" V, servo centre "); lb_float(runSc, 0); lb_str(" us");
  if (runVref > 0.0f) { lb_str(", battery compensation to "); lb_float(runVref, 2); lb_str(" V"); }
  out(toDump);
  for (int i = 0; i <= runSteps; i++) {
    if (i < runSteps && i > stepsDone) continue;     /* never started */
    lb_clear(); lb_str(pre);
    if (i < runSteps) {
      if (runReps > 1) { lb_str(" lap "); lb_int(i / runPlanSteps + 1); }
      lb_str(" step "); lb_int(i % runPlanSteps + 1); lb_str(": "); step_text(&runPlan[i]); lb_str(i == stepsDone ? " (cut short)" : ""); }
    else lb_str(" braking + rolling out");
    lb_str(" -> "); lb_float(stepDist[i], 0); lb_str(" cm");
    if (imuOk) {
      lb_str(", "); lb_float(stepTurn[i], 1); lb_str(" deg");
      if (i < runSteps && fabsf(stepTurn[i]) >= 5.0f && runPlan[i].secs > 0.0f) {
        lb_str(" ("); lb_float(stepTurn[i] / runPlan[i].secs, 1); lb_str(" deg/s, radius ");
        lb_float(57.3f * fabsf(stepDist[i] / stepTurn[i]), 0); lb_str(" cm)");
      }
    }
    out(toDump);
  }
  if (imuOk) {
    lb_clear(); lb_str(pre); lb_str(" end (encoder + gyro estimate): ");
    lb_float(fabsf(xCm), 0); lb_str(xCm >= 0 ? " cm ahead, " : " cm behind, ");
    lb_float(fabsf(yCm), 0); lb_str(yCm >= 0 ? " cm left" : " cm right");
    lb_str(" of the start = "); lb_float(sqrtf(xCm * xCm + yCm * yCm), 0); lb_str(" cm away");
    out(toDump);
    uint32_t bad = gyroReadsFailed - gyroFail0, all = bad + gyroReadsOk - gyroOk0;
    if (bad > 0) {   /* many failures = I2C trouble: the heading is not to be trusted */
      lb_clear(); lb_str(pre); lb_str(" gyro: "); lb_uint(bad); lb_str(" of "); lb_uint(all);
      lb_str(" readings failed"); out(toDump);
    }
  }
  if (!toDump) { lb_clear(); lb_str("od = print the recording ("); lb_int(recN); lb_str(" samples)"); out(false); }
}

/* ------------------------------------------------------------- recording */
static void record(void) {
  if (recN >= OL_REC_MAX) return;
  OlSample *s = &rec[recN++];
  uint32_t t = (HAL_GetTick() - startMs) / 10;
  s->t10 = (uint16_t)(t > 65535 ? 65535 : t);
  s->step = (uint8_t)ol_step_now();
  s->pct = (int8_t)motorPct;
  s->servoUs = (uint16_t)servoUs;
  s->sonarCm = (uint16_t)sonarCm;
  s->speed10 = (int16_t)clampf_(Speed_CM_SEC * 10.0f, -32000.0f, 32000.0f);
  s->yaw10 = (int16_t)clampf_(yaw_now() * 10.0f, -32000.0f, 32000.0f);
  s->dist = distCm; s->x = xCm; s->y = yCm;
  battF += 0.3f * (battery_volts() - battF);   /* the PWM makes single readings jump */
  s->batt = battF;
}

/* dead reckoning: add up the encoder distance along the gyro heading */
static void track(void) {
  int32_t c = encoder_count();
  if (c == lastCnt) return;
  float d = (float)(c - lastCnt) * 100.0f / COUNTS_PER_METER;
  float h = yaw_now() * 0.0174533f;
  xCm += d * cosf(h);
  yCm += d * sinf(h);
  distCm += d;
  lastCnt = c;
}

/* ----------------------------------------------------------- start, stop */
static void start_run(void) {
  runPlanSteps = ol_steps();
  runReps = olRepeat;
  runSteps = runPlanSteps * runReps;
  if (runSteps == 0) { phase = OL_IDLE; say("ERR the plan is empty: ol 1 <motor %> <steer us> <s>"); return; }
  if (runSteps > OL_MAX_RUN) { phase = OL_IDLE; say("ERR plan steps x repeats is more than 48: fewer repeats (or)"); return; }
  uint32_t t = 0;
  for (int i = 0; i < runSteps; i++) {
    runPlan[i] = olPlan[i % runPlanSteps];
    t += (uint32_t)(runPlan[i].secs * 1000.0f + 0.5f);
    stepEndMs[i] = t;
    stepDist[i] = stepTurn[i] = 0.0f;
  }
  stepDist[runSteps] = stepTurn[runSteps] = 0.0f;
  runSc = SERVO_CENTER_US;
  runVref = olVref;
  lastCnt = stallCnt = encoder_count();
  yaw0 = yawDeg;
  distCm = xCm = yCm = 0.0f;
  stepNow = 0;
  stepsDone = 0;
  standSteps = 0;
  battStart = battF = battery_volts();
  gyroOk0 = gyroReadsOk;
  gyroFail0 = gyroReadsFailed;
  recN = 0;
  dumpIdx = -1;
  startMs = stallMs = HAL_GetTick();
  lastDriveStep = driveSteps;
  phase = OL_RUN;
  apply_step(0);
  record();
  lb_clear(); lb_str("OPEN LOOP start: "); lb_int(runSteps); lb_str(" steps, ");
  lb_float(stepEndMs[runSteps - 1] / 1000.0f, 2); lb_str(" s"); lb_send(false);
}

/* stop playing the plan: brake, then record the roll-out (OL_END) */
static void begin_end(const char *why) {
  stepDist[stepNow] = distCm - markDist;
  stepTurn[stepNow] = yaw_now() - markYaw;
  markDist = distCm;
  markYaw = yaw_now();
  endWhy = why;
  endMs = HAL_GetTick();
  phase = OL_END;
  braking = true;
  motor_brake();
  servo_set_us((uint32_t)SERVO_CENTER_US);
}

static void finish(void) {
  stepDist[runSteps] = distCm - markDist;
  stepTurn[runSteps] = yaw_now() - markYaw;
  phase = OL_IDLE;
  servo_set_us((uint32_t)SERVO_CENTER_US);
  report(false);
}

void ol_arm(void) {
  if (phase != OL_IDLE) return;
  if (ol_steps() == 0) { say("ERR the plan is empty: ol 1 <motor %> <steer us> <s>"); return; }
  if (state != DRIVE_OFF) { say("ERR a distance run is on: s first"); return; }
  phase = OL_ARMED;
  armMs = HAL_GetTick();
  lb_clear(); lb_str("OK open loop starts in "); lb_float(OL_DELAY_MS / 1000.0f, 1);
  lb_str(" s: let go of the car"); lb_send(false);
}

void ol_stop(const char *why) {
  if (phase == OL_ARMED) { phase = OL_IDLE; say("open loop cancelled"); }
  else if (phase == OL_RUN) begin_end(why);
}

/* a manual command takes over: end at once, it sets the motor itself */
void ol_cancel(void) {
  if (phase == OL_ARMED) { phase = OL_IDLE; return; }
  if (phase == OL_RUN) { begin_end("a command took over"); }
  if (phase == OL_END) finish();
}

/* ------------------------------------------------------------- main loop */
void ol_poll(void) {
  uint32_t now = HAL_GetTick();

  if (phase == OL_ARMED && now - armMs >= OL_DELAY_MS) start_run();

  if (phase == OL_RUN || phase == OL_END) {
    track();
    if (driveSteps != lastDriveStep) {           /* a new encoder speed: one sample */
      lastDriveStep = driveSteps;
      record();
      standSteps = (Speed_CM_SEC == 0.0f) ? standSteps + 1 : 0;
      if (phase == OL_RUN && runVref > 0.0f) step_motor(stepNow);   /* follow the battery */
    }
  }

  if (phase == OL_RUN) {
    uint32_t t = now - startMs;
    while (stepNow < runSteps && t >= stepEndMs[stepNow]) {
      stepDist[stepNow] = distCm - markDist;
      stepTurn[stepNow] = yaw_now() - markYaw;
      stepsDone = ++stepNow;
      if (stepNow < runSteps) apply_step(stepNow);
    }
    if (stepNow >= runSteps) { stepNow = runSteps - 1; begin_end("end of the plan"); }
    /* safety stops */
    else if (state != DRIVE_OFF) begin_end("a distance run started");
    else if (olSonarStop && obstacle) begin_end("sonar: something in front");
    else {
      int32_t c = encoder_count();
      if (motorPct == 0.0f || c != stallCnt) { stallCnt = c; stallMs = now; }
      else if (now - stallMs >= (uint32_t)(DRIVE_STALL_S * 1000.0f)) begin_end("motor on but the wheel does not turn");
    }
  }

  if (phase == OL_END) {
    uint32_t t = now - endMs;
    if (braking && t >= OL_BRAKE_MS) { braking = false; motor_set(0.0f); }   /* brake over: coast */
    if ((t >= OL_BRAKE_MS && standSteps >= 3) || t >= 3000) finish();
  }

  /* "od": a few lines at a time, only while the port has room */
  while (dumpIdx >= 0 && comms_tx_free(dumpPort) > 140) {
    if (dumpIdx >= recN) {
      report(true);
      lb_clear(); lb_str("OK dump done: "); lb_int(recN); lb_str(" samples"); lb_send_to(dumpPort);
      dumpIdx = -1;
      break;
    }
    const OlSample *s = &rec[dumpIdx++];
    lb_clear();
    lb_uint(s->t10 * 10u);         lb_str(",");
    lb_uint(s->step);              lb_str(",");
    lb_int(s->pct);                lb_str(",");
    lb_uint(s->servoUs);           lb_str(",");
    lb_float(s->speed10 / 10.0f, 1); lb_str(",");
    lb_float(s->dist, 1);          lb_str(",");
    lb_float(s->yaw10 / 10.0f, 1); lb_str(",");
    lb_float(s->x, 1);             lb_str(",");
    lb_float(s->y, 1);             lb_str(",");
    lb_uint(s->sonarCm);           lb_str(",");
    lb_float(s->batt, 2);
    lb_send_to(dumpPort);
  }
}

/* ---------------------------------------------------------------- commands */
void ol_command(const char *line) {
  char c2 = (char)tolower((unsigned char)line[1]);
  const char *p = line;
  while (isalpha((unsigned char)*p)) p++;
  float v[4];
  int n = 0;
  while (n < 4) {
    char *end;
    float f = strtof(p, &end);
    if (end == p) break;
    v[n++] = f;
    p = end;
  }

  if (c2 == 'l') {                      /* ol: show, or change one step */
    if (n == 0) { print_plan(); return; }
    if (phase != OL_IDLE) { say("ERR not while the plan runs: s first"); return; }
    int i = (int)v[0] - 1;
    if (n != 4 || i < 0 || i >= OL_MAX_STEPS || fabsf(v[1]) > 100.0f || v[3] < 0.0f || v[3] > 60.0f) {
      say("ERR ol <step 1..12> <motor % -100..100> <steer us, - = left> <s 0..60>");
      return;
    }
    if (i > ol_steps()) { say("ERR fill the steps in order: no gap before this one"); return; }
    olPlan[i].pct = v[1];
    olPlan[i].steerUs = v[2];
    olPlan[i].secs = v[3];
    if (v[3] == 0.0f)                   /* the plan ends here: clear the rest */
      for (int k = i; k < OL_MAX_STEPS; k++) olPlan[k].secs = 0.0f;
    print_plan();
  } else if (c2 == 'g') {               /* og: go */
    ol_arm();
  } else if (c2 == 'd') {               /* od: print the recording */
    if (phase != OL_IDLE) { say("ERR not while the plan runs"); return; }
    if (recN == 0) { say("ERR nothing recorded yet: og first"); return; }
    dumpPort = comms_from;
    lb_clear(); lb_str("t_ms,step,motor_pct,servo_us,speed_cms,dist_cm,yaw_deg,x_cm,y_cm,sonar_cm,batt_v");
    lb_send_to(dumpPort);
    for (int i = 0; i < runPlanSteps; i++) {
      lb_clear(); lb_str("# plan step "); lb_int(i + 1); lb_str(": "); step_text(&runPlan[i]);
      lb_send_to(dumpPort);
    }
    lb_clear(); lb_str("# played "); lb_int(runReps); lb_str(" times, the step column counts all "); lb_int(runSteps);
    lb_send_to(dumpPort);
    dumpIdx = 0;
  } else if (c2 == 's') {               /* os 0/1: sonar stop */
    olSonarStop = (n > 0 && v[0] != 0.0f);
    print_plan();
  } else if (c2 == 'v') {               /* ov <V>: battery compensation, 0 = off */
    if (n == 0 || v[0] < 0.0f || (v[0] > 0.0f && v[0] < 6.0f) || v[0] > 13.0f) { say("ERR ov <volts 6..13>, ov 0 = off"); return; }
    olVref = v[0];
    print_plan();
  } else if (c2 == 'r') {               /* or <n>: play the plan n times */
    if (n == 0 || v[0] < 1.0f || v[0] > 10.0f) { say("ERR or <times 1..10>"); return; }
    if (phase != OL_IDLE) { say("ERR not while the plan runs: s first"); return; }
    olRepeat = (int)v[0];
    print_plan();
  } else if (c2 == 'b') {               /* ob 0/1: what the start button does */
    olButton = (n > 0 && v[0] != 0.0f);
    print_plan();
  } else {
    say("ERR open loop: ol (plan), ol <n> <%> <us> <s>, og (go), od (dump), os 0/1, ob 0/1, ov <V>, or <n>");
  }
}
