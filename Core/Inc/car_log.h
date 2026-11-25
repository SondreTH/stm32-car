/* =====================================================================
 * car_log.h - test recorder: every test ("d", a route with "go", "vt")
 * is recorded on the car at 10 Hz, with the report lines the car printed
 * during it. Downloaded later with "ld", so a test outside still has its
 * data when Bluetooth drops. Kept in RAM: a reset loses it.
 *   ll        list the recorded tests
 *   ld        download the last test (CSV, to the port that asked)
 *   ld <n>    download test n, "ld 0" = all of them
 *   lc        clear the recorder
 * ===================================================================== */
#ifndef CAR_LOG_H
#define CAR_LOG_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LOG_MAX_SAMPLES 2400   /* 10 per second: 4 minutes in all            */
#define LOG_MAX_TESTS   20
#define LOG_NOTES_BYTES 6144   /* report lines printed during the tests      */
#define LOG_IDLE_MS     1500   /* a test ends this long after the car stands */

void log_begin(const char *what);          /* a test starts: what = e.g. "d 2.00 m" */
void log_poll(bool busy);                  /* main loop; busy = a test is still going */
void log_line(const char *s, uint16_t n);  /* every line the car prints (car_comms.c) */
void log_command(const char *line);        /* ll, ld, lc                             */
bool log_recording(void);
bool log_dumping(void);

#ifdef __cplusplus
}
#endif
#endif
