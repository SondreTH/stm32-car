/* =====================================================================
 * car_drive.h - odometry (distance, speed, x/y) and the PI speed loop
 *   STOP      motor braked
 *   OPEN      raw % to the motor (bring-up)
 *   SPEED     closed-loop speed in m/s  (the required control loop)
 *   DISTANCE  drive X metres with a slow-down, then brake
 * ===================================================================== */
#ifndef CAR_DRIVE_H
#define CAR_DRIVE_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { DM_STOP = 0, DM_OPEN = 1, DM_SPEED = 2, DM_DISTANCE = 3 } DriveMode;

typedef struct {
  DriveMode mode;
  float openPct;
  float target, setpoint, speed;    /* m/s */
  float distance;                   /* m since last zero */
  float integ;                      /* PI integrator, % */
  int32_t lastCount, lastDelta;
  uint16_t stillTicks;
  float x, y;                       /* m, x = forward at start, y = left */
  float moveGoal, moveCruise, moveStart;
  bool  moveDone;
} DriveState;

extern DriveState drv;

void drive_stop(void);
void drive_open(float pct);
void drive_speed(float mps);
void drive_distance(float meters, float cruise);
void drive_zero(void);
bool drive_is_still(void);
void drive_update(float dt);                                /* every control tick */
void odometry_update(float headingBeforeDeg, float headingAfterDeg);

#ifdef __cplusplus
}
#endif
#endif
