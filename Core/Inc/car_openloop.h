/* =====================================================================
 * car_openloop.h - open-loop route: a list of steps (motor %, steering,
 * time) that the car plays without looking at any sensor. See
 * car_openloop.c for how it works and why.
 * ===================================================================== */
#ifndef CAR_OPENLOOP_H
#define CAR_OPENLOOP_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OL_MAX_STEPS  12      /* steps in a plan                                  */
#define OL_MAX_RUN    48      /* steps in a run: plan steps x repeats ("or")      */
#define OL_DELAY_MS   2000    /* "og" / the button: start this long after, to step back */
#define OL_BRAKE_MS   500     /* after the last step: brake this long, then coast */
#define OL_REC_MAX    1200    /* samples recorded, 10 per second = 2 minutes      */

typedef struct {
  float pct;       /* motor %, -100..100, 0 = brake                  */
  float steerUs;   /* servo pulse added to SERVO_CENTER_US, - = left */
  float secs;      /* how long, 0 = the plan ends before this step   */
} OlStep;

extern OlStep olPlan[OL_MAX_STEPS];
extern bool   olButton;      /* ob: the start button runs the plan (else a 1 m run) */
extern bool   olSonarStop;   /* os: stop the plan when something is in front        */
extern float  olVref;        /* ov: battery compensation to this voltage, 0 = off   */
extern int    olRepeat;      /* or: play the plan this many times                   */

void    ol_command(const char *line);   /* "ol", "og", "od", "os", "ob", "ov", "or" (from car_app.c) */
void    ol_poll(void);                  /* main loop, right after heading_poll()   */
void    ol_arm(void);                   /* start the plan in OL_DELAY_MS           */
void    ol_stop(const char *why);       /* armed: cancel. Running: brake, roll out, print the report */
void    ol_cancel(void);                /* a manual command takes over: end at once, motor untouched */
bool    ol_busy(void);                  /* armed, running or rolling out           */
bool    ol_dumping(void);               /* "od" is still printing                  */
int32_t ol_armed_ms_left(void);         /* -1 = not armed                          */
int     ol_step_now(void);              /* 1.. while running, 0 otherwise          */
int     ol_steps(void);                 /* steps in the plan                       */
int     ol_run_steps(void);             /* steps in the run: plan x repeats        */
float   ol_secs_left(void);             /* time left in the plan                   */

#ifdef __cplusplus
}
#endif
#endif
