/* autopilot.c - automatic mode state machine (see autopilot.h) */
#include "autopilot.h"
#include "car_config.h"
#include <math.h>
#include <string.h>

Autopilot ap;

static int find_stop_after(int idx) {
  for (int i = idx + 1; i < (int)path.n; i++)
    if (path.p[i].flags & (PF_STOP | PF_FINISH)) return i;
  return -1;
}

static void ap_event(ApEvent e, float a, float b) { ap.ev = e; ap.evA = a; ap.evB = b; }
static void ap_set(ApState s, uint32_t ms) { ap.state = s; ap.t0 = ms; }

/* Call with the car at the start (pose zeroed, gyro calibrated) */
bool ap_start(uint32_t ms) {
  follower_reset();
  memset(&ap, 0, sizeof(ap));
  ap.stopIdx = -1;
  if (path.n < 2) { ap_event(EV_NO_PATH, 0, 0); return false; }
  ap.stopIdx = find_stop_after(0);
  if (ap.stopIdx < 0) ap.stopIdx = path.n - 1;      /* no flags: stop at the end */
  ap.stopNum = 1;
  ap_set(AP_RUN, ms);
  return true;
}

void ap_abort(void) { ap.state = AP_IDLE; fol.detour = false; }

void ap_update(const ApIn *in, ApOut *out) {
  out->brake = true; out->speed = 0; out->steerDeg = ap.steer;
  if (ap.state == AP_IDLE || ap.state == AP_DONE) return;

  follower_project(in->x, in->y);
  float remaining = path.s[ap.stopIdx] - fol.sNow;
  bool obstChecks = !(path.p[fol.seg].flags & PF_NOOBS) && remaining > AP_OBST_IGNORE_M;
  bool blocked = in->sonarCm < AP_OBST_STOP_CM;

  switch (ap.state) {
    case AP_RUN: {
      if (fol.detour && fol.sNow > fol.d3) { fol.detour = false; ap_event(EV_DETOUR_DONE, 0, 0); }
      if (remaining <= AP_STOP_TOL_M) {                       /* arrived */
        uint8_t f = path.p[ap.stopIdx].flags;
        ap_event((f & PF_FINISH) ? EV_FINISHED : EV_WAYPOINT, ap.stopNum, fol.cte);
        ap_set((f & PF_FINISH) ? AP_DONE : AP_WAIT_WP, in->ms);
        return;
      }
      if (obstChecks && blocked) {                            /* obstacle ahead */
        ap.obstS = fol.sNow + SONAR_X_M + in->sonarCm / 100.0f;
        ap.clearSince = 0;
        ap_event(EV_OBSTACLE, in->sonarCm, 0);
        ap_set(AP_OBST_WAIT, in->ms);
        return;
      }
      ap.steer = follower_steer(in->x, in->y, in->headingDeg, AP_LOOKAHEAD_M, WHEELBASE_M);
      float v = fminf(AP_CRUISE, sqrtf(2.0f * AP_DECEL * fmaxf(remaining, 0.0f)));   /* v^2 = 2 a d */
      v *= 1.0f - AP_TURN_SLOWDOWN * fminf(1.0f, fabsf(ap.steer) / STEER_MAX_DEG);
      if (fol.detour) v = fminf(v, AP_DETOUR_SPEED);
      v = fmaxf(v, AP_CREEP);
      ap.speed = v;
      out->brake = false; out->speed = v; out->steerDeg = ap.steer;
      return;
    }
    case AP_WAIT_WP:
      if (in->ms - ap.t0 >= AP_WAIT_MS) {
        ap.stopIdx = find_stop_after(ap.stopIdx);
        if (ap.stopIdx < 0) { ap_set(AP_DONE, in->ms); ap_event(EV_FINISHED, ap.stopNum, fol.cte); return; }
        ap.stopNum++;
        ap_event(EV_LEAVING, ap.stopNum, 0);
        ap_set(AP_RUN, in->ms);
      }
      return;
    case AP_OBST_WAIT:
      if (!blocked && in->sonarCm > AP_OBST_CLEAR_CM) {
        if (!ap.clearSince) ap.clearSince = in->ms ? in->ms : 1;
        if (in->ms - ap.clearSince >= 700) { ap_event(EV_CLEARED, 0, 0); ap_set(AP_RUN, in->ms); }
      } else ap.clearSince = 0;
      if (ap.state == AP_OBST_WAIT && AP_REROUTE && !fol.detour && in->ms - ap.t0 >= AP_OBST_WAIT_MS) {
        ap.revStart = in->odoDist;
        ap_event(EV_REROUTE, AP_DETOUR_OFFSET_M, 0);
        ap_set(AP_REVERSE, in->ms);
      }
      return;
    case AP_REVERSE:
      if (in->odoDist <= ap.revStart - AP_REVERSE_M || in->ms - ap.t0 > 6000) {
        follower_start_detour(ap.obstS, AP_DETOUR_OFFSET_M, AP_DETOUR_RAMP_M, AP_DETOUR_PASS_M);
        ap_set(AP_RUN, in->ms);
        return;
      }
      out->brake = false; out->speed = -AP_REVERSE_SPEED; out->steerDeg = 0;
      return;
    default: return;
  }
}
