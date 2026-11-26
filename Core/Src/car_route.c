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
 *   ra                              show the settings for going around an obstacle
 *   raw <s>  ras <1|-1>  raa <deg>  rao <m>  rap <m>  rab <m>  rav <cm/s>
 *                                   wait, side (1 = left), angle out, diagonal leg,
 *                                   leg beside it, back up first, speed (see below)
 *
 * Going around an obstacle (added 2026-11-26). The sonar pauses a straight
 * (car_drive.c). If the way is still blocked after routeAvoidWaitS, the car
 * drives around it and then drives the rest of the straight:
 *
 *                 ___pass___
 *            out /          \ out           side: left (+1) or right (-1)
 *   ___ back <__/    [X]     \___ rest ->   angle: routeAvoidDeg
 *
 * back up, turn out, diagonal, turn back parallel, pass it, turn in,
 * diagonal, turn back onto the line. How far along the straight the detour
 * got is measured, not assumed: encoder distance x cos(heading off the
 * straight), added up while it drives. The sonar only looks forward, so
 * the car cannot see how wide or long the obstacle is: set out/pass to
 * what the course uses. During the detour an obstacle only pauses the car.
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

/* Going around an obstacle: defaults of the "ra.." commands */
float routeAvoidWaitS  = 10.0f;   /* raw: blocked this long, then go around (0 = only wait) */
float routeAvoidSide   = 1.0f;    /* ras: +1 = around on the left, -1 = on the right */
float routeAvoidDeg    = 45.0f;   /* raa: angle out of the straight */
float routeAvoidOutM   = 1.0f;    /* rao: diagonal leg out (and back in) */
float routeAvoidPassM  = 1.0f;    /* rap: leg parallel to the straight, beside the obstacle */
float routeAvoidBackM  = 0.5f;    /* rab: back up first, so the first turn has room */
float routeAvoidSpeed  = 30.0f;   /* rav: speed of the detour legs (cm/s) */

typedef enum { RT_IDLE = 0, RT_ARMED, RT_STRAIGHT, RT_TURN, RT_WAIT, RT_AVOID } RouteState;
static RouteState rtState = RT_IDLE;
static int        rtIdx = 0;
static uint32_t   rtStepMs = 0;
static float      rtTargetYaw = 0.0f;

/* A turn in progress (route turn or detour turn) */
static float    turnTarget = 0.0f;   /* gyro heading to reach (deg) */
static float    turnDir = 1.0f;      /* +1 = turning left, -1 = right */
static uint32_t turnRunMs = 0;       /* time spent turning, not counting obstacle pauses */
static uint32_t turnLastMs = 0;

/* The obstacle detour */
#define AVOID_MAX_STEPS 8
typedef struct {
  RouteType type;    /* ROUTE_STRAIGHT or ROUTE_TURN */
  float     val;     /* straight: metres (negative = back up); turn: direction +1 / -1 */
  float     yawOff;  /* heading of this piece, relative to the straight (deg) */
} AvoidStep;
static AvoidStep avSteps[AVOID_MAX_STEPS];
static int      avN = 0, avIdx = 0;
static bool     avBlocked = false;   /* the straight is paused by an obstacle */
static uint32_t avBlockedMs = 0;     /* since when */
static float    avYaw0 = 0.0f;       /* heading of the straight */
static float    avLeftCm = 0.0f;     /* straight still to drive when the detour began */
static float    avAlongCm = 0.0f;    /* detour progress along the straight */
static float    avSideCm = 0.0f;     /* detour distance to the side (+ = left) */
static float    avLastDist = 0.0f;   /* Real_Dist at the last update */
static bool     avLegBlocked = false;   /* a detour leg is paused by the sonar (told once) */

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

/* Start an arc towards the gyro heading `target`: dir +1 = left, -1 = right.
 * The speed hold (car_drive.c) pauses by itself for an obstacle. */
static void begin_turn(float target, float dir) {
  turnTarget = target;
  turnDir = dir;
  turnRunMs = 0;
  turnLastMs = HAL_GetTick();
  heading_set_hold(false);
  /* + = left (smaller servo pulse), - = right (larger pulse) */
  float us = SERVO_CENTER_US - dir * routeTurnUs;
  if (us < SERVO_PULSE_MIN) us = SERVO_PULSE_MIN;
  if (us > SERVO_PULSE_MAX) us = SERVO_PULSE_MAX;
  servo_set_us((uint32_t)us);
  Set_Speed_cm_s(routeTurnSpeed);
}

/* 1 = the turn is done (wheels straight again), 0 = still turning,
 * -1 = it failed and the route has been stopped */
static int poll_turn(int driveRes) {
  uint32_t now = HAL_GetTick();
  if (driveRes == RUN_STALL) {
    route_finish("ERR route stopped: stall during turn", true);
    return -1;
  }
  if (!obstacle) turnRunMs += now - turnLastMs;   /* waiting for an obstacle is not turning */
  turnLastMs = now;
  if (turnRunMs >= ROUTE_TURN_TIMEOUT_MS) {
    route_finish("ERR route turn timeout", true);
    return -1;
  }
  bool done = (turnDir > 0.0f) ? (yawDeg >= turnTarget - routeTurnLeadDeg)
                               : (yawDeg <= turnTarget + routeTurnLeadDeg);
  if (!done) return 0;
  servo_set_us((uint32_t)SERVO_CENTER_US);
  heading_set_hold(true);
  return 1;
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
    avBlocked = false;
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
    begin_turn(rtTargetYaw, (s->val >= 0.0f) ? 1.0f : -1.0f);
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

/* ---------------------------------------------------- going around an obstacle */

/* Add up how far the detour has come along the straight and to the side */
static void avoid_track(void) {
  float d = Real_Dist;
  float dd = d - avLastDist;
  avLastDist = d;
  float off = (yawDeg - avYaw0) * 0.0174533f;   /* degrees off the straight, in radians */
  avAlongCm += dd * cosf(off);
  avSideCm  += dd * sinf(off);
}

static void avoid_add(RouteType type, float val, float yawOff) {
  if (avN >= AVOID_MAX_STEPS) return;
  avSteps[avN].type = type;
  avSteps[avN].val = val;
  avSteps[avN].yawOff = yawOff;
  avN++;
}

static void start_avoid_step(void) {
  AvoidStep *a = &avSteps[avIdx];
  avLegBlocked = false;
  if (a->type == ROUTE_TURN) {
    begin_turn(avYaw0 + a->yawOff, a->val);
    return;
  }
  heading_set_target(avYaw0 + a->yawOff);
  heading_set_hold(true);
  if (!drive_start_at(a->val, routeAvoidSpeed)) {
    route_finish("ERR route: detour leg does not fit", false);
    return;
  }
  avLastDist = 0.0f;   /* drive_start_at() set Real_Dist back to 0 */
}

/* The straight has been blocked for routeAvoidWaitS: plan the detour */
static void begin_avoid(void) {
  float s = (routeAvoidSide >= 0.0f) ? 1.0f : -1.0f;   /* +1 = left */
  float a = routeAvoidDeg;
  avYaw0 = rtTargetYaw;
  avLeftCm = target_dist_cm - Real_Dist;
  avAlongCm = 0.0f;
  avSideCm = 0.0f;
  avLastDist = Real_Dist;
  drive_cancel();   /* end the paused run; the motor stays braked */

  avN = 0;
  avIdx = 0;
  if (routeAvoidBackM > 0.01f) avoid_add(ROUTE_STRAIGHT, -routeAvoidBackM, 0.0f);
  avoid_add(ROUTE_TURN, s, s * a);                     /* turn out */
  if (routeAvoidOutM > 0.01f) avoid_add(ROUTE_STRAIGHT, routeAvoidOutM, s * a);
  avoid_add(ROUTE_TURN, -s, 0.0f);                     /* parallel to the straight */
  if (routeAvoidPassM > 0.01f) avoid_add(ROUTE_STRAIGHT, routeAvoidPassM, 0.0f);
  avoid_add(ROUTE_TURN, -s, -s * a);                   /* turn in */
  if (routeAvoidOutM > 0.01f) avoid_add(ROUTE_STRAIGHT, routeAvoidOutM, -s * a);
  avoid_add(ROUTE_TURN, s, 0.0f);                      /* back onto the line */

  rtState = RT_AVOID;
  lb_clear(); lb_str("route step "); lb_int(rtIdx + 1); lb_str(": blocked for ");
  lb_float(routeAvoidWaitS, 0); lb_str(" s, going around it on the ");
  lb_str(s > 0.0f ? "left, " : "right, "); lb_float(avLeftCm / 100.0f, 2);
  lb_str(" m of the straight left"); lb_send(false);
  start_avoid_step();
}

/* The detour is done: drive the rest of the straight, at its heading */
static void end_avoid(void) {
  float left = avLeftCm - avAlongCm;
  lb_clear(); lb_str("route step "); lb_int(rtIdx + 1); lb_str(": around it, ");
  lb_float(avAlongCm / 100.0f, 2); lb_str(" m along, "); lb_float(avSideCm, 0);
  lb_str(" cm to the side, "); lb_float(left / 100.0f, 2); lb_str(" m left"); lb_send(false);
  heading_set_target(avYaw0);
  heading_set_hold(true);
  if (left < 5.0f) {
    /* The detour got past the end of the straight (the obstacle was close
     * to its end). Backing up along the line would drive towards the
     * obstacle with the sonar looking the other way, so go on from here. */
    if (left < -5.0f) {
      lb_clear(); lb_str("route step "); lb_int(rtIdx + 1); lb_str(": the detour went ");
      lb_float(-left / 100.0f, 2); lb_str(" m past the end of the straight"); lb_send(false);
    }
    start_step(rtIdx + 1);
    return;
  }
  rtState = RT_STRAIGHT;
  avBlocked = false;
  if (!drive_start(left / 100.0f))
    route_finish("ERR route straight does not fit (check tt/vn)", false);
}

static void poll_avoid(int driveRes) {
  avoid_track();
  AvoidStep *a = &avSteps[avIdx];
  if (a->type == ROUTE_TURN) {
    if (poll_turn(driveRes) <= 0) return;   /* still turning, or failed */
  } else {
    if (driveRes == RUN_NONE) {             /* still driving, or paused by the sonar */
      if (obstacle && !avLegBlocked && a->val > 0.0f) {
        /* Something is in the way of the detour itself (the obstacle is
         * wider than rao/raa reach, or something else): wait, no new detour */
        avLegBlocked = true;
        lb_clear(); lb_str("route step "); lb_int(rtIdx + 1); lb_str(": the detour is blocked at ");
        lb_float(sonarCm, 0); lb_str(" cm, waiting (s = stop)"); lb_send(false);
      }
      return;
    }
    if (driveRes == RUN_STALL || driveRes == RUN_ENC_DIR) {
      route_finish("ERR route stopped: drive fault while going around", true);
      return;
    }
  }
  if (++avIdx >= avN) end_avoid();
  else start_avoid_step();
}

static void print_avoid(void) {
  lb_clear(); lb_str("avoid: wait "); lb_float(routeAvoidWaitS, 0); lb_str(" s (raw, 0 = never), ");
  lb_str(routeAvoidSide >= 0.0f ? "left" : "right"); lb_str(" (ras 1/-1), out ");
  lb_float(routeAvoidDeg, 0); lb_str(" deg (raa)"); lb_send(false);
  lb_clear(); lb_str("  diagonal "); lb_float(routeAvoidOutM, 2); lb_str(" m (rao), beside it ");
  lb_float(routeAvoidPassM, 2); lb_str(" m (rap), back up "); lb_float(routeAvoidBackM, 2);
  lb_str(" m (rab), "); lb_float(routeAvoidSpeed, 0); lb_str(" cm/s (rav)"); lb_send(false);
  lb_clear(); lb_str("  stops "); lb_float(OBST_STOP_CM, 0); lb_str(" cm + ");
  lb_float(OBST_LOOK_S, 1); lb_str(" s x speed before an obstacle (so)"); lb_send(false);
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
    if (driveRes == RUN_NONE) {
      /* Paused by the sonar (car_drive.c brakes and waits). Still blocked
       * after routeAvoidWaitS: drive around it. */
      if (obstacle && state != DRIVE_OFF && target_dist_cm > 0.0f) {
        if (!avBlocked) {
          avBlocked = true;
          avBlockedMs = HAL_GetTick();
          lb_clear(); lb_str("route step "); lb_int(rtIdx + 1); lb_str(": obstacle at ");
          lb_float(sonarCm, 0); lb_str(" cm, waiting"); lb_send(false);
        } else if (routeAvoidWaitS > 0.0f &&
                   HAL_GetTick() - avBlockedMs >= (uint32_t)(routeAvoidWaitS * 1000.0f)) {
          begin_avoid();
        }
      } else {
        avBlocked = false;
      }
      return;
    }
    if (driveRes == RUN_STALL || driveRes == RUN_ENC_DIR) {
      route_finish("ERR route stopped: drive fault", true);
      return;
    }
    start_step(rtIdx + 1);
    return;
  }

  if (rtState == RT_TURN) {
    if (poll_turn(driveRes) > 0) start_step(rtIdx + 1);
    return;
  }

  if (rtState == RT_AVOID) {
    poll_avoid(driveRes);
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

  if (c2 == 'a') {   /* ra..: going around an obstacle */
    float v = strtof(line + 3, NULL);
    if (c3 == 'w') {
      if (v < 0.0f || v > 120.0f) { say("ERR raw <s> 0..120, 0 = never go around"); return; }
      routeAvoidWaitS = v;
    } else if (c3 == 's') {
      routeAvoidSide = (v >= 0.0f) ? 1.0f : -1.0f;
    } else if (c3 == 'a') {
      if (v < 10.0f || v > 90.0f) { say("ERR raa <deg> 10..90"); return; }
      routeAvoidDeg = v;
    } else if (c3 == 'o' || c3 == 'p') {
      if (v < 0.0f || v > 5.0f) { say("ERR rao / rap <m> 0..5"); return; }
      if (c3 == 'o') routeAvoidOutM = v; else routeAvoidPassM = v;
    } else if (c3 == 'b') {
      if (v < 0.0f || v > 2.0f) { say("ERR rab <m> 0..2"); return; }
      routeAvoidBackM = v;
    } else if (c3 == 'v') {
      if (v < 5.0f || v > DRIVE_MAX_CMS) { say("ERR rav <cm/s>"); return; }
      routeAvoidSpeed = v;
    } else if (c3 != '\0' && c3 != ' ') {
      say("ERR use ra, raw, ras, raa, rao, rap, rab or rav");
      return;
    }
    print_avoid();
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
