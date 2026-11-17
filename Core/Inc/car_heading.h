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
extern volatile float headingErrDeg; /* during a run: heading minus heading at the start  */
extern float gyroBiasDps;            /* what the gyro reads while standing still          */

bool heading_init(void);   /* wake the sensor and measure the bias: the car must stand still (0.6 s) */
void heading_poll(void);   /* call from the main loop: reads the gyro, steers during a run */

#ifdef __cplusplus
}
#endif
#endif
