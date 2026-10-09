/* =====================================================================
 * car_comms.h - text in/out on USART2 (USB virtual COM, 115200) and
 * USART1 (Bluetooth, 9600). Interrupt-driven, never blocks the loop.
 * ===================================================================== */
#ifndef CAR_COMMS_H
#define CAR_COMMS_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void comms_init(void);
void comms_pump(void);                          /* call every loop: starts transmissions */
void comms_poll(void (*onLine)(char *line));    /* call every loop: complete input lines */

/* Build a line, then send it to both ports */
void lb_clear(void);
void lb_str(const char *s);
void lb_int(int32_t v);
void lb_uint(uint32_t v);
void lb_float(float v, int decimals);
void lb_hex(uint32_t v);
void lb_send(bool dropIfBusy);                  /* true = drop if queue full (telemetry) */
void say(const char *s);

#ifdef __cplusplus
}
#endif
#endif
