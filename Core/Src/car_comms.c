/* =====================================================================
 * car_comms.c - Simple, robust UART communications matching Hakons_leg.
 *
 * USART1: Bluetooth (PA9 TX, PA10 RX, 9600 baud)
 * USART2: USB ST-LINK Virtual COM port (PA2 TX, PA3 RX, 115200 baud)
 *
 * Uses Hakons_leg proven direct interrupt reception with ORE error
 * recovery and LD2 LED toggle on every byte received.
 * ===================================================================== */
#include "main.h"
#include "car_comms.h"
#include <string.h>

extern UART_HandleTypeDef huart1;   /* Bluetooth */
extern UART_HandleTypeDef huart2;   /* USB virtual COM (ST-LINK) */

/* Live Bluetooth diagnostics */
volatile uint32_t btRxBytes = 0;
volatile uint32_t btErrCount = 0;
volatile char     lastBtChar = ' ';

/* Bluetooth (USART1) receive buffer - identical architecture to Hakons_leg */
#define BT_BUF_SIZE 64
static uint8_t  bt_rx_byte = 0;
static char     bt_rx_buf[BT_BUF_SIZE];
static uint8_t  bt_rx_idx = 0;
static volatile uint8_t bt_cmd_ready = 0;

/* USB (USART2) receive buffer */
#define USB_BUF_SIZE 64
static uint8_t  usb_rx_byte = 0;
static char     usb_rx_buf[USB_BUF_SIZE];
static uint8_t  usb_rx_idx = 0;
static volatile uint8_t usb_cmd_ready = 0;

/* Check for standalone action commands that don't take numeric arguments */
static bool is_instant_cmd(char c) {
  switch (c) {
    case 'f': case 'F':
    case 'b': case 'B':
    case 's': case 'S':
    case 'l': case 'L':
    case 'c': case 'C':
    case 'r': case 'R':
    case 'x': case 'X':
    case 'z': case 'Z':
    case 'g': case 'G':
    case 'i': case 'I':
    case '?': case 'h': case 'H':
    case '1': case '2': case '3': case '4': case '5': case '6':
      return true;
    default:
      return false;
  }
}

/* ------------------------------------------------------------- HAL Callbacks */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    /* DIAGNOSTIC: Toggle on-board green LED on EVERY byte received (like Hakons_leg) */
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    btRxBytes++;
    lastBtChar = (char)bt_rx_byte;

    if (bt_rx_byte == '\r' || bt_rx_byte == '\n') {
      if (bt_rx_idx > 0) {
        bt_rx_buf[bt_rx_idx] = '\0';
        bt_cmd_ready = 1;
      }
    } else if (bt_rx_idx == 0 && is_instant_cmd((char)bt_rx_byte)) {
      /* Single-character action command: execute immediately without waiting for newline */
      bt_rx_buf[0] = (char)bt_rx_byte;
      bt_rx_buf[1] = '\0';
      bt_cmd_ready = 1;
    } else {
      if (bt_rx_idx < (BT_BUF_SIZE - 1)) {
        bt_rx_buf[bt_rx_idx++] = (char)bt_rx_byte;
      } else {
        bt_rx_idx = 0;
      }
    }

    /* Clear any overrun flag before re-arming receive interrupt */
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    HAL_UART_Receive_IT(&huart1, &bt_rx_byte, 1);
  }
  else if (huart->Instance == USART2) {
    if (usb_rx_byte == '\r' || usb_rx_byte == '\n') {
      if (usb_rx_idx > 0) {
        usb_rx_buf[usb_rx_idx] = '\0';
        usb_cmd_ready = 1;
      }
    } else if (usb_rx_idx == 0 && is_instant_cmd((char)usb_rx_byte)) {
      usb_rx_buf[0] = (char)usb_rx_byte;
      usb_rx_buf[1] = '\0';
      usb_cmd_ready = 1;
    } else {
      if (usb_rx_idx < (USB_BUF_SIZE - 1)) {
        usb_rx_buf[usb_rx_idx++] = (char)usb_rx_byte;
      } else {
        usb_rx_idx = 0;
      }
    }

    __HAL_UART_CLEAR_OREFLAG(&huart2);
    HAL_UART_Receive_IT(&huart2, &usb_rx_byte, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    btErrCount++;
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

    /* Clear error flags exactly like Hakons_leg */
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    /* Restart listening */
    HAL_UART_Receive_IT(&huart1, &bt_rx_byte, 1);
  }
  else if (huart->Instance == USART2) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    HAL_UART_Receive_IT(&huart2, &usb_rx_byte, 1);
  }
}

/* ----------------------------------------------------------------- Public API */
void comms_init(void) {
  btRxBytes = 0;
  btErrCount = 0;
  lastBtChar = ' ';
  bt_rx_idx = 0;
  bt_cmd_ready = 0;
  memset(bt_rx_buf, 0, sizeof(bt_rx_buf));

  usb_rx_idx = 0;
  usb_cmd_ready = 0;
  memset(usb_rx_buf, 0, sizeof(usb_rx_buf));

  /* Force enable USART1 NVIC interrupt like Hakons_leg */
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  /* Clear flags and start listening, exactly like Hakons_leg */
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  __HAL_UART_CLEAR_NEFLAG(&huart1);
  __HAL_UART_CLEAR_FEFLAG(&huart1);
  HAL_UART_Receive_IT(&huart1, &bt_rx_byte, 1);

  __HAL_UART_CLEAR_OREFLAG(&huart2);
  __HAL_UART_CLEAR_NEFLAG(&huart2);
  __HAL_UART_CLEAR_FEFLAG(&huart2);
  HAL_UART_Receive_IT(&huart2, &usb_rx_byte, 1);
}

void comms_pump(void) {
  /* Transmissions are sent cleanly in lb_send; pump is a no-op */
}

void comms_poll(void (*onLine)(char *line)) {
  /* 1. Process Bluetooth command if ready */
  if (bt_cmd_ready) {
    static char line[BT_BUF_SIZE];
    strncpy(line, bt_rx_buf, sizeof(line) - 1);
    line[sizeof(line) - 1] = '\0';
    bt_rx_idx = 0;
    bt_rx_buf[0] = '\0';
    bt_cmd_ready = 0;
    onLine(line);
  }

  /* 2. Process USB command if ready */
  if (usb_cmd_ready) {
    static char line[USB_BUF_SIZE];
    strncpy(line, usb_rx_buf, sizeof(line) - 1);
    line[sizeof(line) - 1] = '\0';
    usb_rx_idx = 0;
    usb_rx_buf[0] = '\0';
    usb_cmd_ready = 0;
    onLine(line);
  }

  /* Ensure reception stays armed if an unhandled error reset RxState (Watchdog like Hakons_leg) */
  if (huart1.RxState == HAL_UART_STATE_READY) {
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    HAL_UART_Receive_IT(&huart1, &bt_rx_byte, 1);
  }
  if (huart2.RxState == HAL_UART_STATE_READY) {
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_CLEAR_NEFLAG(&huart2);
    __HAL_UART_CLEAR_FEFLAG(&huart2);
    HAL_UART_Receive_IT(&huart2, &usb_rx_byte, 1);
  }
}

/* -------------------------------------------------------------- Line Builder */
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
  if (lbLen == 0) return;
  if (lbLen < sizeof(lbBuf) - 2) {
    lbBuf[lbLen++] = '\r';
    lbBuf[lbLen++] = '\n';
  }
  /* Send to USB Virtual COM port */
  HAL_UART_Transmit(&huart2, (uint8_t *)lbBuf, lbLen, 10);
  /* Send to Bluetooth */
  HAL_UART_Transmit(&huart1, (uint8_t *)lbBuf, lbLen, 10);
  lbLen = 0;
}

void say(const char *s) { lb_clear(); lb_str(s); lb_send(false); }
