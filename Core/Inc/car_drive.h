/* =====================================================================
 * car_drive.h - distance run: trapezoidal speed profile (M2.9) with a
 * PID position loop (M2.10). Speeds in cm/s, distances in cm, times in s.
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

extern int state;                      /* 0 = off, 1 = accelerate, 2 = cruise, 3 = decelerate */
extern float time_ctl, T, Sn, target_dist_cm;
extern volatile float SP_Vel;          /* speed setpoint */
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
void  Apply_Motor_PWM(float control_cm_s);   /* forward at about this speed, open loop */
void  Set_Speed_cm_s(float target_cm_s);     /* end any run, then Apply_Motor_PWM */

void  drive_stop(void);      /* end the run and let the motor coast */
void  drive_cancel(void);    /* end the run, leave the motor to the caller */
void  drive_update(void);    /* one control step: TIM3 interrupt, every 100 ms */

#ifdef __cplusplus
}
#endif
#endif
