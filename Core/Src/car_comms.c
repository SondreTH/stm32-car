/* =====================================================================
 * car_comms.c - UART text I/O with software queues.
 *  RX: 1-byte interrupt receive into a ring buffer, re-armed every byte.
 *  TX: lines go into a ring buffer; comms_pump() starts an interrupt
 *      transmission of the next block whenever the UART is idle.
 * ===================================================================== */
#include "main.h"
#include "car_comms.h"
#include <string.h>

extern UART_HandleTypeDef huart1;   /* Bluetooth */
extern UART_HandleTypeDef huart2;   /* USB virtual COM (ST-LINK) */

#define TXQ 1024   /* power of two */
#define RXQ 256    /* power of two */

typedef struct {
  UART_HandleTypeDef *h;
  uint8_t  tx[TXQ];
  volatile uint16_t txHead, txTail, txBusy;   /* txBusy = bytes in flight */
  uint8_t  rx[RXQ];
  volatile uint16_t rxHead, rxTail;
  uint8_t  rxByte;
  char     line[64];
  uint8_t  lineLen;
} Port;

static Port ports[2];

static uint16_t tx_used(Port *p)  { return (uint16_t)(p->txHead - p->txTail) & (TXQ - 1); }
static uint16_t tx_space(Port *p) { return (uint16_t)(TXQ - 1 - tx_used(p)); }

static void port_pump(Port *p) {
  if (p->txBusy || p->txHead == p->txTail) return;
  if (p->h->gState != HAL_UART_STATE_READY) return;
  uint16_t tail = p->txTail, head = p->txHead;
  uint16_t len = (head > tail) ? (uint16_t)(head - tail) : (uint16_t)(TXQ - tail);  /* contiguous block */
  p->txBusy = len;
  if (HAL_UART_Transmit_IT(p->h, &p->tx[tail], len) != HAL_OK) p->txBusy = 0;
}

static void port_put(Port *p, const char *d, uint16_t n) {
  for (uint16_t i = 0; i < n; i++) { p->tx[p->txHead] = (uint8_t)d[i]; p->txHead = (uint16_t)((p->txHead + 1) & (TXQ - 1)); }
}

static void port_send(Port *p, const char *d, uint16_t n, bool dropIfBusy) {
  if ((uint16_t)(n + 2) > tx_space(p)) {
    if (dropIfBusy) return;
    uint32_t t0 = HAL_GetTick();
    while ((uint16_t)(n + 2) > tx_space(p) && HAL_GetTick() - t0 < 500) port_pump(p);
    if ((uint16_t)(n + 2) > tx_space(p)) return;   /* port stuck: give up rather than hang */
  }
  port_put(p, d, n);
  port_put(p, "\r\n", 2);
}

/* ---- HAL callbacks (called from the USART interrupts) ---- */
static Port *port_of(UART_HandleTypeDef *h) { return h == &huart2 ? &ports[0] : (h == &huart1 ? &ports[1] : 0); }

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h) {
  Port *p = port_of(h); if (!p) return;
  p->txTail = (uint16_t)((p->txTail + p->txBusy) & (TXQ - 1));
  p->txBusy = 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h) {
  Port *p = port_of(h); if (!p) return;
  uint16_t next = (uint16_t)((p->rxHead + 1) & (RXQ - 1));
  if (next != p->rxTail) { p->rx[p->rxHead] = p->rxByte; p->rxHead = next; }
  HAL_UART_Receive_IT(h, &p->rxByte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *h) {
  Port *p = port_of(h); if (!p) return;
  if (p->txBusy && h->gState == HAL_UART_STATE_READY) p->txBusy = 0;   /* aborted TX: resend later */
  HAL_UART_Receive_IT(h, &p->rxByte, 1);                               /* re-arm after overrun/noise */
}

void comms_init(void) {
  memset(ports, 0, sizeof(ports));
  ports[0].h = &huart2;
  ports[1].h = &huart1;
  for (int i = 0; i < 2; i++) HAL_UART_Receive_IT(ports[i].h, &ports[i].rxByte, 1);
}

void comms_pump(void) { port_pump(&ports[0]); port_pump(&ports[1]); }

void comms_poll(void (*onLine)(char *line)) {
  for (int i = 0; i < 2; i++) {
    Port *p = &ports[i];
    /* make sure reception is armed (e.g. after an error) */
    if (p->h->RxState == HAL_UART_STATE_READY) HAL_UART_Receive_IT(p->h, &p->rxByte, 1);
    while (p->rxTail != p->rxHead) {
      char ch = (char)p->rx[p->rxTail];
      p->rxTail = (uint16_t)((p->rxTail + 1) & (RXQ - 1));
      if (ch == '\n' || ch == '\r') {
        if (p->lineLen) { p->line[p->lineLen] = 0; p->lineLen = 0; onLine(p->line); }
      } else if (p->lineLen < sizeof(p->line) - 1) p->line[p->lineLen++] = ch;
    }
  }
}

/* ---- line builder ---- */
static char    lbBuf[180];
static uint16_t lbLen = 0;

void lb_clear(void) { lbLen = 0; }
static void lb_ch(char c) { if (lbLen < sizeof(lbBuf) - 1) lbBuf[lbLen++] = c; }
void lb_str(const char *s) { while (*s) lb_ch(*s++); }

void lb_uint(uint32_t v) {
  char t[11]; int n = 0;
  do { t[n++] = (char)('0' + v % 10); v /= 10; } while (v);
  while (n) lb_ch(t[--n]);
}
void lb_int(int32_t v) { if (v < 0) { lb_ch('-'); lb_uint((uint32_t)(-(int64_t)v)); } else lb_uint((uint32_t)v); }

void lb_hex(uint32_t v) {
  const char *d = "0123456789ABCDEF"; char t[8]; int n = 0;
  do { t[n++] = d[v & 15]; v >>= 4; } while (v);
  while (n) lb_ch(t[--n]);
}

/* float without printf (newlib-nano has no %f by default) */
void lb_float(float v, int decimals) {
  if (v != v) { lb_str("nan"); return; }
  if (v < 0) { lb_ch('-'); v = -v; }
  uint32_t scale = 1; for (int i = 0; i < decimals; i++) scale *= 10;
  if (v > 4.0e9f / scale) { lb_str("ovf"); return; }
  uint32_t whole = (uint32_t)(v * scale + 0.5f);
  lb_uint(whole / scale);
  if (decimals > 0) {
    lb_ch('.');
    uint32_t frac = whole % scale, div = scale / 10;
    for (int i = 0; i < decimals; i++) { lb_ch((char)('0' + (frac / div) % 10)); div /= 10; if (!div) div = 1; }
  }
}

void lb_send(bool dropIfBusy) {
  port_send(&ports[0], lbBuf, lbLen, dropIfBusy);
  port_send(&ports[1], lbBuf, lbLen, dropIfBusy);
  lbLen = 0;
}

void say(const char *s) { lb_clear(); lb_str(s); lb_send(false); }
