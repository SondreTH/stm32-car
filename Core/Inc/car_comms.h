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

/* Bluetooth lock: when locked, lines from Bluetooth are ignored (and
 * reported). Only a command from USB should change it. */
bool comms_line_from_usb(void);   /* the line being handled came from the USB port */
void comms_bt_lock(bool locked);
bool comms_bt_locked(void);

/* Build a line, then send it to both ports */
void lb_clear(void);
void lb_str(const char *s);
void lb_int(int32_t v);
void lb_uint(uint32_t v);
void lb_float(float v, int decimals);
void lb_hex(uint32_t v);
void lb_send(bool dropIfBusy);                  /* true = drop if queue full (telemetry) */
void say(const char *s);

/* One port only (0 = USB, 1 = Bluetooth): for long outputs like the
 * open-loop recording, which would take a minute at Bluetooth speed */
extern int comms_from;                          /* port of the command line being handled */
void     lb_send_to(int port);                  /* waits if the queue is full */
uint16_t comms_tx_free(int port);               /* bytes free in that port's send queue */

#ifdef __cplusplus
}
#endif
#endif
