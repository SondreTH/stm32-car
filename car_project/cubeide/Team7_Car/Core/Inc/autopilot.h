/* =====================================================================
 * autopilot.h - automatic mode state machine
 *   RUN        follow the path, stop exactly at the next STOP/FINISH
 *   WAIT_WP    standing at a waypoint for AP_WAIT_MS (10 s + margin)
 *   OBST_WAIT  something in front: brake and wait for it to leave
 *   REVERSE    obstacle stayed: back up, then RUN on a shifted path
 *   DONE       finished at the start/finish point
 * Inputs in (time, pose, sonar), speed/steer out. No hardware calls.
 * ===================================================================== */
#ifndef AUTOPILOT_H
#define AUTOPILOT_H
#include <stdint.h>
#include <stdbool.h>
#include "path.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { AP_IDLE = 0, AP_RUN = 1, AP_WAIT_WP = 2, AP_OBST_WAIT = 3, AP_REVERSE = 4, AP_DONE = 5 } ApState;
typedef enum { EV_NONE = 0, EV_WAYPOINT, EV_FINISHED, EV_LEAVING, EV_OBSTACLE, EV_CLEARED,
               EV_REROUTE, EV_DETOUR_DONE, EV_NO_PATH } ApEvent;

typedef struct { uint32_t ms; float x, y, headingDeg, odoDist, sonarCm; } ApIn;
typedef struct { bool brake; float speed; float steerDeg; } ApOut;

typedef struct {
  ApState  state;
  int      stopIdx;      /* path index of the stop we drive to */
  uint8_t  stopNum;      /* 1, 2, 3 ... for messages */
  uint32_t t0;           /* time the current state started */
  uint32_t clearSince;   /* obstacle wait: when the way became clear (0 = not) */
  float    obstS;        /* path distance of the obstacle */
  float    revStart;     /* odometer when reversing started */
  float    steer, speed;
  ApEvent  ev;           /* one-shot event for printing */
  float    evA, evB;
} Autopilot;

extern Autopilot ap;

bool ap_start(uint32_t ms);
void ap_abort(void);
void ap_update(const ApIn *in, ApOut *out);

#ifdef __cplusplus
}
#endif
#endif
