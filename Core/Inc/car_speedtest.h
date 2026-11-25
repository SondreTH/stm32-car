/* =====================================================================
 * car_speedtest.h - "vs": speed steps, to see how a surface loads the
 * motor. The car holds VS_FIRST_CMS, then VS_STEP_CMS more each step up to
 * the top speed, VS_STEP_S seconds each, with the speed loop (car_drive.c)
 * and the gyro heading hold keeping it straight. Per step it reports the
 * speed it reached and the motor % that needed. Recorded by car_log.c.
 *   vs            steps up to VS_TOP_CMS
 *   vs <cm/s>     steps up to this speed
 * ===================================================================== */
#ifndef CAR_SPEEDTEST_H
#define CAR_SPEEDTEST_H
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VS_FIRST_CMS  20.0f   /* first step                                  */
#define VS_STEP_CMS   10.0f   /* each next step this much faster             */
#define VS_TOP_CMS    60.0f   /* default top speed                           */
#define VS_STEP_S     3.0f    /* seconds per step                            */
#define VS_SETTLE_S   1.0f    /* not averaged: the speed loop settles first  */

bool vs_start(float topCms);       /* false (and says why) if it can't start */
void vs_stop(const char *why);     /* ends it, motor coasts; why may be NULL */
void vs_poll(void);                /* main loop                              */
bool vs_running(void);

#ifdef __cplusplus
}
#endif
#endif
