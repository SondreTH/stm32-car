/* =====================================================================
 * car_route.h - a lap as a list of steps (straight, turn, wait), driven
 * one or more times with one command, with a report after every step and
 * every lap. Runs in the main loop; the straights are normal distance
 * runs (car_drive.c), the turns steer by the gyro (car_heading.c).
 * ===================================================================== */
#ifndef CAR_ROUTE_H
#define CAR_ROUTE_H
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* kinds of step */
#define STEP_STRAIGHT 'S'   /* value = metres, forward                       */
#define STEP_TURN     'T'   /* value = degrees, + = left, - = right          */
#define STEP_WAIT     'W'   /* value = seconds standing still (a waypoint)   */

void route_init(void);                   /* load the default test lap           */
void route_default(void);                /* the default test lap: a 2 m square  */
void route_clear(void);
bool route_add(char type, float value);  /* false if the lap is full            */
void route_list(void);                   /* print the lap                       */

bool route_start(int laps);              /* false (and says why) if it can't start */
void route_stop(const char *why);        /* stop the route, if one runs; why may be NULL */
void route_poll(void);                   /* call from the main loop             */
void route_run_ended(int result);        /* the main loop got a RUN_... result  */

bool route_running(void);
int  route_lap(void);                    /* 1.. while running                   */
int  route_step(void);                   /* 1.. while running, 0 = no route     */
const char *route_phase_name(void);      /* "TURN", "WAIT", or 0 (use the run phase) */

#ifdef __cplusplus
}
#endif
#endif
