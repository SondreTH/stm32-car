// =====================================================================
//  autopilot.h  -  automatic mode state machine
//
//   RUN        follow the path; slow down and stop exactly at the next
//              STOP / FINISH point
//   WAIT_WP    standing at a waypoint for AP_WAIT_MS (10 s + margin)
//   OBST_WAIT  something is in front: brake and wait for it to leave
//   REVERSE    obstacle did not leave: back up a little ...
//              ... then RUN again on a sideways-shifted path (detour)
//   DONE       finished at the start/finish point
//
//  Plain C++: inputs in (time, pose, sonar), speed/steer out. The
//  firmware applies the outputs; the PC simulator uses the same code.
// =====================================================================
#pragma once
#include "path.h"

enum ApState : uint8_t { AP_IDLE = 0, AP_RUN = 1, AP_WAIT_WP = 2, AP_OBST_WAIT = 3, AP_REVERSE = 4, AP_DONE = 5 };
enum ApEvent : uint8_t { EV_NONE = 0, EV_WAYPOINT, EV_FINISHED, EV_LEAVING, EV_OBSTACLE, EV_CLEARED,
                         EV_REROUTE, EV_DETOUR_DONE, EV_NO_PATH };

struct ApIn  { uint32_t ms; float x, y, headingDeg, odoDist, sonarCm; };
struct ApOut { bool brake; float speed; float steerDeg; };

struct Autopilot {
  ApState  state   = AP_IDLE;
  int      stopIdx = -1;      // path index of the stop we are driving to
  uint8_t  stopNum = 0;       // 1, 2, 3 ... for messages
  uint32_t t0 = 0;            // time the current state started
  uint32_t clearSince = 0;    // obstacle wait: time the way became clear (0 = not clear)
  float    obstS = 0;         // path distance where the obstacle is
  float    revStart = 0;      // odometer when reversing started
  float    steer = 0, speed = 0;
  // one-shot event for the firmware to print
  ApEvent  ev = EV_NONE;
  float    evA = 0, evB = 0;
};
static Autopilot ap;

static int apFindStopAfter(int idx) {
  for (int i = idx + 1; i < (int)path.n; i++)
    if (path.p[i].flags & (PF_STOP | PF_FINISH)) return i;
  return -1;
}

static void apEvent(ApEvent e, float a = 0, float b = 0) { ap.ev = e; ap.evA = a; ap.evB = b; }

// Call once the car is placed at the start (pose zeroed, gyro calibrated).
static bool apStart(uint32_t ms) {
  followerReset();
  ap = Autopilot();
  if (path.n < 2) { apEvent(EV_NO_PATH); return false; }
  ap.stopIdx = apFindStopAfter(0);
  if (ap.stopIdx < 0) ap.stopIdx = path.n - 1;       // no flags: stop at the end
  ap.stopNum = 1;
  ap.state = AP_RUN;
  ap.t0 = ms;
  return true;
}

static void apAbort() { ap.state = AP_IDLE; fol.detour = false; }

static void apSet(ApState s, uint32_t ms) { ap.state = s; ap.t0 = ms; }

static void apUpdate(const ApIn& in, ApOut& out) {
  out.brake = true; out.speed = 0; out.steerDeg = ap.steer;
  if (ap.state == AP_IDLE || ap.state == AP_DONE) return;

  followerProject(in.x, in.y);
  float remaining = path.s[ap.stopIdx] - fol.sNow;
  bool obstChecks = !(path.p[fol.seg].flags & PF_NOOBS) && remaining > AP_OBST_IGNORE_M;
  bool blocked = in.sonarCm < AP_OBST_STOP_CM;

  switch (ap.state) {

    case AP_RUN: {
      if (fol.detour && fol.sNow > fol.d3) { fol.detour = false; apEvent(EV_DETOUR_DONE); }

      // arrived at the stop point (or just past it)?
      if (remaining <= AP_STOP_TOL_M) {
        uint8_t f = path.p[ap.stopIdx].flags;
        apEvent(f & PF_FINISH ? EV_FINISHED : EV_WAYPOINT, ap.stopNum, fol.cte);
        apSet((f & PF_FINISH) ? AP_DONE : AP_WAIT_WP, in.ms);
        return;                                           // brake
      }
      // obstacle ahead?
      if (obstChecks && blocked) {
        ap.obstS = fol.sNow + SONAR_X_M + in.sonarCm / 100.0f;
        ap.clearSince = 0;
        apEvent(EV_OBSTACLE, in.sonarCm);
        apSet(AP_OBST_WAIT, in.ms);
        return;
      }
      // steer along the path
      ap.steer = followerSteer(in.x, in.y, in.headingDeg, AP_LOOKAHEAD_M, WHEELBASE_M);
      // speed: cruise, slow for the stop (v^2 = 2 a d), slower in turns/detours
      float v = fminf(AP_CRUISE, sqrtf(2.0f * AP_DECEL * fmaxf(remaining, 0.0f)));
      v *= 1.0f - AP_TURN_SLOWDOWN * fminf(1.0f, fabsf(ap.steer) / STEER_MAX_DEG);
      if (fol.detour) v = fminf(v, AP_DETOUR_SPEED);
      v = fmaxf(v, AP_CREEP);
      ap.speed = v;
      out.brake = false; out.speed = v; out.steerDeg = ap.steer;
      return;
    }

    case AP_WAIT_WP:
      if (in.ms - ap.t0 >= AP_WAIT_MS) {
        ap.stopIdx = apFindStopAfter(ap.stopIdx);
        if (ap.stopIdx < 0) { apSet(AP_DONE, in.ms); apEvent(EV_FINISHED, ap.stopNum, fol.cte); return; }
        ap.stopNum++;
        apEvent(EV_LEAVING, ap.stopNum);
        apSet(AP_RUN, in.ms);
      }
      return;

    case AP_OBST_WAIT:
      if (!blocked && in.sonarCm > AP_OBST_CLEAR_CM) {
        if (!ap.clearSince) ap.clearSince = in.ms ? in.ms : 1;
        if (in.ms - ap.clearSince >= 700) { apEvent(EV_CLEARED); apSet(AP_RUN, in.ms); }
      } else ap.clearSince = 0;
      // still blocked after the wait time -> reroute (once per obstacle)
      if (ap.state == AP_OBST_WAIT && AP_REROUTE && !fol.detour && in.ms - ap.t0 >= AP_OBST_WAIT_MS) {
        ap.revStart = in.odoDist;
        apEvent(EV_REROUTE, AP_DETOUR_OFFSET_M);
        apSet(AP_REVERSE, in.ms);
      }
      return;

    case AP_REVERSE:
      if (in.odoDist <= ap.revStart - AP_REVERSE_M || in.ms - ap.t0 > 6000) {
        followerStartDetour(ap.obstS, AP_DETOUR_OFFSET_M, AP_DETOUR_RAMP_M, AP_DETOUR_PASS_M);
        apSet(AP_RUN, in.ms);
        return;                                           // brake one tick
      }
      out.brake = false; out.speed = -AP_REVERSE_SPEED; out.steerDeg = 0;
      return;

    default: return;
  }
}
