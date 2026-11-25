/* =====================================================================
 * car_route.h - closed-loop autonomous route sequencer
 *
 * Executes a list of straight runs (encoder PID + gyro heading hold),
 * gyro-guided turns (fixed steering offset until target yaw is reached),
 * and optional waypoint pauses. Up to ROUTE_MAX_STEPS segments (16+ corners).
 * ===================================================================== */
#ifndef CAR_ROUTE_H
#define CAR_ROUTE_H
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROUTE_MAX_STEPS       32
#define ROUTE_DELAY_MS        2000   /* countdown before starting ("rg" or button) */
#define ROUTE_TURN_TIMEOUT_MS 15000  /* safety timeout for a single turn */

typedef enum {
  ROUTE_END = 0,
  ROUTE_STRAIGHT,   /* val = distance in meters (positive = forward) */
  ROUTE_TURN,       /* val = turn angle in degrees (+ = left, - = right) */
  ROUTE_WAIT        /* val = pause in seconds */
} RouteType;

typedef struct {
  RouteType type;
  float     val;
} RouteStep;

extern RouteStep routePlan[ROUTE_MAX_STEPS];
extern bool  routeButton;       /* rb: start button runs the closed-loop route */
extern float routeTurnUs;       /* rtu: steering offset in us during turns */
extern float routeTurnSpeed;    /* rtv: speed in cm/s during turns */
extern float routeTurnLeadDeg;  /* rtl: stop turning this many deg early */

void route_command(const char *line);
void route_poll(int driveRes);  /* call from main loop after drive_result() */
void route_arm(void);           /* start route after ROUTE_DELAY_MS */
void route_stop(const char *why);
void route_cancel(void);        /* manual override: abort without touching motor */
bool route_busy(void);          /* armed or running */
int  route_step_now(void);      /* 1.. while running, 0 otherwise */
int  route_steps(void);         /* number of active steps */

#ifdef __cplusplus
}
#endif
#endif
