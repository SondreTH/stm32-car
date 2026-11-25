/* =====================================================================
 * car_route.c - closed-loop autonomous route sequencer
 *
 * Commands (all start with 'r' + a second letter; bare 'r' stays steer right):
 *   rp [s <m>, t <deg>, w <s> ...]  show or set the whole route
 *   rs <step> <s|t|w|0> [val]       edit one 1-based step (0 = end route here)
 *   rg                              arm and run the route (2 s countdown)
 *   rb [0|1]                        start button runs this route
 *   rtu <us>                        turn steering offset from centre (default 350)
 *   rtv <cm/s>                      turn speed (default 22)
 *   rtl <deg>                       turn lead angle before target (default 4)
 * ===================================================================== */
#include "main.h"
#include "car_route.h"
#include "car_config.h"
#include "car_hw.h"
#include "car_comms.h"
#include "car_drive.h"
#include "car_heading.h"
#include <math.h>
#include <stdlib.h>
#include <ctype.h>

RouteStep routePlan[ROUTE_MAX_STEPS] = {
  { ROUTE_STRAIGHT,  1.5f },
  { ROUTE_TURN,    -120.0f },
  { ROUTE_STRAIGHT,  1.5f },
  { ROUTE_TURN,    -120.0f },
  { ROUTE_STRAIGHT,  1.5f },
  { ROUTE_TURN,    -120.0f },
  { ROUTE_END,       0.0f }
};

bool  routeButton      = true;
float routeTurnUs      = 250.0f;
float routeTurnSpeed   = 22.0f;
float routeTurnLeadDeg = 4.0f;

typedef enum { RT_IDLE = 0, RT_ARMED, RT_STRAIGHT, RT_TURN, RT_WAIT } RouteState;
static RouteState rtState = RT_IDLE;
static int        rtIdx = 0;
static uint32_t   rtStepMs = 0;
static float      rtTargetYaw = 0.0f;

int route_steps(void) {
  int n = 0;
  while (n < ROUTE_MAX_STEPS && routePlan[n].type != ROUTE_END) n++;
  return n;
}

bool route_busy(void)     { return rtState != RT_IDLE; }
int  route_step_now(void) { return (rtState >= RT_STRAIGHT) ? rtIdx + 1 : 0; }

static void route_finish(const char *msg, bool brake) {
  rtState = RT_IDLE;
  heading_clear_target();
  heading_set_hold(true);
  drive_cancel();
  if (brake) motor_brake();
  else       motor_set(0.0f);
  servo_set_us((uint32_t)SERVO_CENTER_US);
  if (msg) say(msg);
}

void route_stop(const char *why) {
  if (rtState == RT_IDLE) return;
  route_finish(why, rtState != RT_ARMED);
}

void route_cancel(void) {
  if (rtState == RT_IDLE) return;
  rtState = RT_IDLE;
  heading_clear_target();
  heading_set_hold(true);
}

static void start_step(int idx) {
  int total = route_steps();
  if (idx >= total) {
    route_finish("ROUTE DONE", true);
    return;
  }

  rtIdx = idx;
  rtStepMs = HAL_GetTick();
  RouteStep *s = &routePlan[idx];

  if (s->type == ROUTE_STRAIGHT) {
    rtState = RT_STRAIGHT;
    heading_set_target(rtTargetYaw);
    heading_set_hold(true);
    if (!drive_start(s->val)) {
      route_finish("ERR route straight does not fit (check tt/vn)", false);
      return;
    }
    lb_clear(); lb_str("route step "); lb_int(idx + 1);
    lb_str(": straight "); lb_float(s->val, 2); lb_str(" m"); lb_send(false);
  } else if (s->type == ROUTE_TURN) {
    if (!imuOk) {
      route_finish("ERR route turn needs gyro (type I)", false);
      return;
    }
    rtState = RT_TURN;
    rtTargetYaw += s->val;
    heading_set_hold(false);
    /* + angle = left (smaller servo pulse), - angle = right (larger pulse) */
    float us = SERVO_CENTER_US + ((s->val > 0.0f) ? -routeTurnUs : routeTurnUs);
    if (us < SERVO_PULSE_MIN) us = SERVO_PULSE_MIN;
    if (us > SERVO_PULSE_MAX) us = SERVO_PULSE_MAX;
    servo_set_us((uint32_t)us);
    Set_Speed_cm_s(routeTurnSpeed);
    lb_clear(); lb_str("route step "); lb_int(idx + 1);
    lb_str(": turn "); lb_float(s->val, 1); lb_str(" deg -> target ");
    lb_float(rtTargetYaw, 1); lb_send(false);
  } else if (s->type == ROUTE_WAIT) {
    rtState = RT_WAIT;
    drive_stop();
    servo_set_us((uint32_t)SERVO_CENTER_US);
    lb_clear(); lb_str("route step "); lb_int(idx + 1);
    lb_str(": wait "); lb_float(s->val, 1); lb_str(" s"); lb_send(false);
  } else {
    route_finish("ROUTE DONE", true);
  }
}

void route_arm(void) {
  if (route_steps() == 0) { say("ERR route is empty"); return; }
  drive_stop();
  rtState = RT_ARMED;
  rtStepMs = HAL_GetTick();
  lb_clear(); lb_str("route armed: "); lb_int(route_steps());
  lb_str(" steps in "); lb_float(ROUTE_DELAY_MS / 1000.0f, 1); lb_str(" s"); lb_send(false);
}

void route_poll(int driveRes) {
  if (rtState == RT_IDLE) return;

  if (rtState == RT_ARMED) {
    if (HAL_GetTick() - rtStepMs >= ROUTE_DELAY_MS) {
      rtTargetYaw = yawDeg;
      start_step(0);
    }
    return;
  }

  if (rtState == RT_STRAIGHT) {
    if (driveRes == RUN_NONE) return;
    if (driveRes == RUN_STALL || driveRes == RUN_ENC_DIR) {
      route_finish("ERR route stopped: drive fault", true);
      return;
    }
    start_step(rtIdx + 1);
    return;
  }

  if (rtState == RT_TURN) {
    if (driveRes == RUN_STALL) {
      route_finish("ERR route stopped: stall during turn", true);
      return;
    }
    if (obstacle) {
      motor_brake();
      return;
    }
    if (motorPct == 0.0f && !obstacle) {
      Set_Speed_cm_s(routeTurnSpeed);   /* resume after obstacle cleared */
    }
    if (HAL_GetTick() - rtStepMs >= ROUTE_TURN_TIMEOUT_MS) {
      route_finish("ERR route turn timeout", true);
      return;
    }
    float ang = routePlan[rtIdx].val;
    bool done = (ang >= 0.0f) ? (yawDeg >= rtTargetYaw - routeTurnLeadDeg)
                              : (yawDeg <= rtTargetYaw + routeTurnLeadDeg);
    if (done) {
      servo_set_us((uint32_t)SERVO_CENTER_US);
      heading_set_hold(true);
      start_step(rtIdx + 1);
    }
    return;
  }

  if (rtState == RT_WAIT) {
    if (HAL_GetTick() - rtStepMs >= (uint32_t)(routePlan[rtIdx].val * 1000.0f)) {
      start_step(rtIdx + 1);
    }
  }
}

static void print_route(void) {
  int n = route_steps();
  lb_clear(); lb_str("route ("); lb_int(n); lb_str(" steps, rb=");
  lb_int(routeButton ? 1 : 0); lb_str(" rtu="); lb_float(routeTurnUs, 0);
  lb_str(" rtv="); lb_float(routeTurnSpeed, 0); lb_str(" rtl=");
  lb_float(routeTurnLeadDeg, 1); lb_str(")"); lb_send(false);
  for (int i = 0; i < n; i++) {
    lb_clear(); lb_str("  "); lb_int(i + 1); lb_str(": ");
    if (routePlan[i].type == ROUTE_STRAIGHT)      { lb_str("s "); lb_float(routePlan[i].val, 2); lb_str(" m"); }
    else if (routePlan[i].type == ROUTE_TURN)     { lb_str("t "); lb_float(routePlan[i].val, 1); lb_str(" deg"); }
    else if (routePlan[i].type == ROUTE_WAIT)     { lb_str("w "); lb_float(routePlan[i].val, 1); lb_str(" s"); }
    lb_send(false);
  }
}

static const char *skip_sep(const char *p) {
  while (*p == ' ' || *p == '\t' || *p == ',') p++;
  return p;
}

static RouteType parse_type(char ch) {
  ch = (char)tolower((unsigned char)ch);
  if (ch == 's') return ROUTE_STRAIGHT;
  if (ch == 't') return ROUTE_TURN;
  if (ch == 'w') return ROUTE_WAIT;
  return ROUTE_END;
}

void route_command(const char *line) {
  char c2 = (char)tolower((unsigned char)line[1]);
  char c3 = (char)tolower((unsigned char)line[2]);

  if (c2 == 'g') { route_arm(); return; }

  if (c2 == 'b') {
    const char *p = skip_sep(line + 2);
    if (*p) routeButton = (atoi(p) != 0);
    else    routeButton = !routeButton;
    print_route();
    return;
  }

  if (c2 == 't') {
    float v = strtof(line + 3, NULL);
    if (c3 == 'u') {
      if (v < 50.0f || v > 500.0f) { say("ERR rtu <us> 50..500"); return; }
      routeTurnUs = v;
    } else if (c3 == 'v') {
      if (v < 5.0f || v > DRIVE_MAX_CMS) { say("ERR rtv <cm/s>"); return; }
      routeTurnSpeed = v;
    } else if (c3 == 'l') {
      if (v < 0.0f || v > 30.0f) { say("ERR rtl <deg> 0..30"); return; }
      routeTurnLeadDeg = v;
    } else {
      say("ERR use rtu, rtv, or rtl");
      return;
    }
    print_route();
    return;
  }

  if (c2 == 's') {
    char *end = NULL;
    const char *p = skip_sep(line + 2);
    long idx = strtol(p, &end, 10);
    if (end == p || idx < 1 || idx > ROUTE_MAX_STEPS) {
      say("ERR rs <step 1..32> <s|t|w|0> [val]");
      return;
    }
    p = skip_sep(end);
    if (*p == '0' || !*p) {
      routePlan[idx - 1].type = ROUTE_END;
      routePlan[idx - 1].val = 0.0f;
      print_route();
      return;
    }
    RouteType t = parse_type(*p);
    if (t == ROUTE_END) { say("ERR type must be s, t, w, or 0"); return; }
    p = skip_sep(p + 1);
    float val = strtof(p, NULL);
    routePlan[idx - 1].type = t;
    routePlan[idx - 1].val = val;
    if (idx < ROUTE_MAX_STEPS && routePlan[idx].type == ROUTE_END) {
      /* keep terminator after new last step */
    }
    print_route();
    return;
  }

  if (c2 == 'p') {
    const char *p = skip_sep(line + 2);
    if (!*p) { print_route(); return; }
    int n = 0;
    while (*p && n < ROUTE_MAX_STEPS) {
      RouteType t = parse_type(*p);
      if (t == ROUTE_END) { say("ERR rp: expected s, t, or w"); return; }
      p = skip_sep(p + 1);
      char *end = NULL;
      float val = strtof(p, &end);
      if (end == p) { say("ERR rp: missing number"); return; }
      routePlan[n].type = t;
      routePlan[n].val = val;
      n++;
      p = skip_sep(end);
    }
    if (n < ROUTE_MAX_STEPS) routePlan[n].type = ROUTE_END;
    print_route();
    return;
  }

  say("ERR unknown r* command (rp, rs, rg, rb, rtu, rtv, rtl)");
}
