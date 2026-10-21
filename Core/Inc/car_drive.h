/* =====================================================================
 * car_drive.h - distance run: trapezoidal speed profile (M2.9) with a
 * PID speed loop (M2.10). Speeds in cm/s, distances in cm, times in s.
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
extern float time_ctl, T, target_dist_cm;
extern volatile float SP_Vel;          /* speed setpoint */
extern volatile float Real_Dist;       /* distance driven since the start of the run */
extern volatile float Speed_CM_SEC, AverageSpeed;
extern volatile float Kp, Ki, Kd;
extern volatile float test_dist_m;

void  Start_Motion_Profile(float dist_m, float nominal_cm_s, float accel_s);
float ControlPID(float setpoint, float input, float dt);
void  Apply_Motor_PWM(float control_cm_s);   /* forward at about this speed, open loop */
void  Set_Speed_cm_s(float target_cm_s);     /* end any run, then Apply_Motor_PWM */

void  drive_stop(void);      /* end the run and let the motor coast */
void  drive_cancel(void);    /* end the run, leave the motor to the caller */
bool  drive_update(void);    /* call every loop: true when a 100 ms step ran */

#ifdef __cplusplus
}
#endif
#endif
