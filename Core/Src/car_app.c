/* =====================================================================
 * car_app.c - Activity 2.5 base: move the motor and read the encoder
 *
 * Commands work on the USB virtual COM port (115200) and Bluetooth (9600):
 *   P <pct>   open-loop motor -100..100     X   stop + brake
 *   Z         zero the encoder              C <cpm>  counts per metre
 *   T <hz>    telemetry on (0 = off)        G   show settings
 *   I         I2C scan + restart the OLED   ?   help
 *
 * The OLED (I2C1, car_oled.c) shows encoder counts and motor % at 5 Hz.
 *
 * LD2 and the blue button (B1) are not used here: they are free for the
 * activity. The 2.5 work (time interrupt, speed) goes in main.c USER CODE 4.
 * ===================================================================== */
#include "main.h"
#include "car_app.h"
#include "car_config.h"
#include "car_hw.h"
#include "car_comms.h"
#include "car_oled.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

static uint32_t telemPeriodMs = 0, telemLastMs = 0;
static uint32_t oledLastMs = 0;
static bool     oledOn = false;

static void print_help(void) {
  say("P <pct>   open-loop motor -100..100   X  stop + brake");
  say("Z         zero the encoder            C <cpm>  counts per metre");
  say("T <hz>    telemetry (0 = off)         G  settings   ?  help");
  say("I         I2C scan, restart the OLED");
}

static void print_settings(void) {
  lb_clear(); lb_str("CPM="); lb_float(COUNTS_PER_METER, 1);
  lb_str(" counts="); lb_int(encoder_count());
  lb_send(false);
}

static void telemetry_header(void) {
  say("t_ms,counts,motor_pct");
}

static void telemetry_send(void) {
  lb_clear();
  lb_uint(HAL_GetTick());    lb_str(",");
  lb_int(encoder_count());   lb_str(",");
  lb_float(motorPct, 1);
  lb_send(true);
}

/* ------------------------------------------------------------------ OLED */
static void oled_start(void) {
  uint8_t a = oled_init();
  oledOn = (a != 0);
  lb_clear();
  if (a) { lb_str("OLED OK at 0x"); lb_hex(a); }
  else     lb_str("OLED not found: check wiring, then type I");
  lb_send(false);
}

/* What the screen shows. Redrawn from scratch at 5 Hz: change freely.
 * Two-colour modules: rows 0-15 are yellow, 16-63 blue, so nothing crosses y = 16. */
static void oled_draw_status(void) {
  oled_clear();
  oled_text(0, 3, "Ninjago", 1);
  int x = oled_text(64, 3, "up ", 1);
  x = oled_int(x, 3, (int32_t)(HAL_GetTick() / 1000), 1);
  oled_text(x, 3, "s", 1);
  oled_fill(0, 13, OLED_WIDTH, 1, true);

  oled_text(0, 18, "encoder counts", 1);
  oled_int(0, 28, encoder_count(), 2);

  x = oled_text(0, 45, "motor ", 1);
  x = oled_int(x, 45, (int32_t)motorPct, 1);
  oled_text(x, 45, " %", 1);
  /* bar: 0 in the middle, forward to the right, reverse to the left */
  oled_frame(0, 53, OLED_WIDTH, 11);
  oled_fill(OLED_WIDTH / 2, 52, 1, 12, true);
  int w = (int)(motorPct * (OLED_WIDTH / 2 - 2) / 100.0f);
  if (w > 0) oled_fill(OLED_WIDTH / 2, 55, w, 7, true);
  else       oled_fill(OLED_WIDTH / 2 + w, 55, -w, 7, true);
}

static void i2c_report(void) {
  uint8_t found[8];
  int n = i2c_scan(found, 8);
  lb_clear(); lb_str("I2C1:");
  if (n < 0)       lb_str(" bus stuck, SDA or SCL held low (no pull-ups? short?)");
  else if (n == 0) lb_str(" nothing answers, check wiring");
  for (int i = 0; i < n && i < 8; i++) { lb_str(" 0x"); lb_hex(found[i]); }
  lb_send(false);
  oled_start();
}

static void handle_command(char *line) {
  char *tok[2] = {0};
  int n = 0;
  for (char *p = strtok(line, " ,\t"); p && n < 2; p = strtok(NULL, " ,\t")) tok[n++] = p;
  if (n == 0) return;
  for (char *p = tok[0]; *p; ++p) *p = (char)toupper((unsigned char)*p);
  float a1 = (n > 1) ? strtof(tok[1], NULL) : 0;
  const char *c = tok[0];

  if      (!strcmp(c, "X")) { motor_brake(); say("OK stop"); }
  else if (!strcmp(c, "P")) { motor_set(a1); say("OK open-loop"); }
  else if (!strcmp(c, "Z")) { encoder_zero(); say("OK zeroed"); }
  else if (!strcmp(c, "C")) { if (a1 > 0) COUNTS_PER_METER = a1; print_settings(); }
  else if (!strcmp(c, "T")) { telemPeriodMs = (a1 > 0) ? (uint32_t)(1000.0f / a1) : 0; if (telemPeriodMs) telemetry_header(); }
  else if (!strcmp(c, "G")) print_settings();
  else if (!strcmp(c, "I")) i2c_report();
  else if (!strcmp(c, "?") || !strcmp(c, "H")) print_help();
  else say("ERR unknown command, type ?");
}

void App_Init(void) {
  comms_init();
  hw_init();
  HAL_Delay(200);
  say("MR2006B Team 7 - Activity 2.5 base ready. Type ? for commands.");
  print_settings();
  oled_start();
}

void App_Loop(void) {
  comms_poll(handle_command);
  if (telemPeriodMs && HAL_GetTick() - telemLastMs >= telemPeriodMs) {
    telemLastMs = HAL_GetTick();
    telemetry_send();
  }
  if (oledOn && HAL_GetTick() - oledLastMs >= 200) {
    oledLastMs = HAL_GetTick();
    oled_draw_status();
    if (!oled_update()) { oledOn = false; say("OLED lost: check wiring, then type I"); }
  }
  comms_pump();
}
