/* =====================================================================
 * car_drive.h - distance run: trapezoidal speed profile (M2.9) with a
 * PID position loop (M2.10). Speeds in cm/s, distances in cm, times in s.
 * Forward is positive; a negative distance is a run in reverse.
 * drive_update() runs in the TIM3 interrupt every 100 ms.
 *
 * The variables are global so STM32CubeMonitor can watch and set them.
 * Writing a distance in metres to test_dist_m starts a run.
 * ===================================================================== */
#ifndef CAR_DRIVE_H
#define CAR_DRIVE_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* state of a run */
#define DRIVE_OFF     0
#define DRIVE_ACCEL   1
#define DRIVE_CRUISE  2
#define DRIVE_DECEL   3
#define DRIVE_SETTLE  4   /* profile finished: close the last error to the target */

/* how the last run ended (drive_result) */
#define RUN_NONE      0
#define RUN_DONE      1   /* stopped within POS_TOL_CM of the target        */
#define RUN_TIMEOUT   2   /* not within tolerance after DRIVE_SETTLE_S      */
#define RUN_SWINGS    3   /* stopped: would need a second change of direction */
#define RUN_STALL     4   /* motor driven but no encoder counts             */
#define RUN_ENC_DIR   5   /* encoder counts the wrong way: ENCODER_INVERT   */

extern volatile int state;             /* DRIVE_OFF .. DRIVE_SETTLE */
extern volatile float time_ctl, T, Sn, target_dist_cm;   /* set in the TIM3 interrupt */
extern volatile float SP_Vel;          /* speed setpoint from the profile */
extern volatile float Real_Dist;       /* distance driven since the start of the run */
extern volatile float Speed_CM_SEC, AverageSpeed;
extern volatile float Kp, Ki, Kd;
extern volatile float test_dist_m;
extern volatile float Pos_Ref;         /* where the car should be now: area under SP_Vel */
extern volatile float run_T, run_ta, run_td;   /* timed run: total, accel, decel time (s) */
extern volatile bool  obstacle;        /* run paused by the sonar */
extern volatile uint32_t driveSteps;   /* counts control steps, for the main loop */

void  Start_Motion_Profile(float dist_m, float nominal_cm_s, float accel_s);
bool  Start_Timed_Profile(float dist_m, float total_s, float accel_s, float decel_s);
bool  drive_start(float dist_m);   /* timed run if run_T > 0, else Start_Motion_Profile */
float ControlPID(float setpoint, float input, float dt);
void  Set_Speed_cm_s(float target_cm_s);     /* end any run, then hold this speed open loop */

void  drive_stop(void);      /* end the run and let the motor coast */
void  drive_cancel(void);    /* end the run, leave the motor to the caller */
void  drive_update(void);    /* one control step: TIM3 interrupt, every 100 ms */
int   drive_result(void);    /* RUN_... of a run that just ended, once; else RUN_NONE */

#ifdef __cplusplus
}
#endif
#endif
