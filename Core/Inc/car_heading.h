/* =====================================================================
 * car_heading.h - MPU-6050 gyro (I2C1, 0x68): heading of the car, and
 * steering that keeps the heading during a distance run
 * ===================================================================== */
#ifndef CAR_HEADING_H
#define CAR_HEADING_H
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

extern bool  imuOk;                  /* the MPU-6050 answers                              */
extern volatile float yawDeg;        /* heading since reset, degrees, + = turned LEFT     */
extern volatile float yawRateDps;    /* turn rate, degrees per second, bias removed       */
extern volatile float headingErrDeg; /* while steering: heading minus target heading      */
extern volatile float headingTargetDeg; /* the heading the steering holds                 */
extern float gyroBiasDps;            /* what the gyro reads while standing still          */
#include <stdint.h>
extern uint32_t gyroReadsOk, gyroReadsFailed;   /* I2C readings since start-up */
extern float gyroAxisDeg[3];                    /* angle around X, Y, Z since "Y" (IMU_YAW_AXIS check) */

bool heading_init(void);   /* wake the sensor and measure the bias: the car must stand still (0.6 s) */
void heading_poll(void);   /* call from the main loop: reads the gyro, steers during a run */

bool heading_dump(float acc[3], float gyr[3]);   /* "gy": raw accel (g) and gyro (deg/s), X Y Z */

/* for a route (car_route.c) */
void heading_target(float deg);   /* steer to this heading; runs no longer reset it      */
void heading_turn(bool on);       /* steer to the target also outside a distance run     */
void heading_release(void);       /* back to normal: a run holds the heading it starts at */

#ifdef __cplusplus
}
#endif
#endif
