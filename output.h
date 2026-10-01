// =====================================================================
//  output.h  -  printing to USB and Bluetooth without blocking
// =====================================================================
#pragma once

#if STM32_CORE_VERSION_MAJOR >= 3
Uart BT(PIN_BT_RX, PIN_BT_TX);             // STM32duino core 3.x
#else
HardwareSerial BT(PIN_BT_RX, PIN_BT_TX);   // STM32duino core 2.x
#endif

// ---------------------------------------------------------------------
//  Output: lines go into a software queue per port, and loop() feeds the
//  hardware UART only as fast as it can take bytes. Printing therefore
//  never blocks the control loop, even on 9600-baud Bluetooth.
//  Telemetry is dropped if the queue is full; normal messages wait.
// ---------------------------------------------------------------------
class LineBuf : public Print {
 public:
  char b[160]; size_t n = 0;
  size_t write(uint8_t c) override { if (n < sizeof(b) - 1) b[n++] = c; return 1; }
  void clear() { n = 0; }
};
static LineBuf lb;

struct TxQueue {
  static const uint16_t SIZE = 1024;           // power of two
  char q[SIZE]; uint16_t head = 0, tail = 0;
  Stream* port = nullptr;
  uint16_t used()  const { return (uint16_t)(head - tail) & (SIZE - 1); }
  uint16_t space() const { return SIZE - 1 - used(); }
  void put(const char* d, size_t n) { for (size_t i = 0; i < n; i++) { q[head] = d[i]; head = (head + 1) & (SIZE - 1); } }
  void pump() {
    int n = port->availableForWrite();
    while (n-- > 0 && tail != head) { port->write((uint8_t)q[tail]); tail = (tail + 1) & (SIZE - 1); }
  }
  void send(const char* d, size_t n, bool dropIfBusy) {
    if (n + 2 > space()) {
      if (dropIfBusy) return;
      while (n + 2 > space()) pump();          // wait (only for rare messages)
    }
    put(d, n); put("\r\n", 2);
  }
};
static TxQueue usbTx, btTx;

static void commsBegin() { usbTx.port = &Serial; btTx.port = &BT; }
static void commsPump()  { usbTx.pump(); btTx.pump(); }

static void sendLine(bool dropIfBusy) {
  usbTx.send(lb.b, lb.n, dropIfBusy);
  btTx.send(lb.b, lb.n, dropIfBusy);
  lb.clear();
}
static void say(const char* s) { lb.clear(); lb.print(s); sendLine(false); }
