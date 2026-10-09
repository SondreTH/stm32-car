/* =====================================================================
 * car_app.c - Activity 2.5 base: move the motor and read the encoder
 *
 * Commands work on the USB virtual COM port (115200) and Bluetooth (9600):
 *   P <pct>   open-loop motor -100..100     X   stop + brake
 *   Z         zero the encoder              C <cpm>  counts per metre
 *   T <hz>    telemetry on (0 = off)        G   show settings
 *   ?         help
 *
 * LD2 and the blue button (B1) are not used here: they are free for the
 * activity. The 2.5 work (time interrupt, speed) goes in main.c USER CODE 4.
 * ===================================================================== */
#include "main.h"
#include "car_app.h"
#include "car_config.h"
#include "car_hw.h"
#include "car_comms.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

static uint32_t telemPeriodMs = 0, telemLastMs = 0;

static void print_help(void) {
  say("P <pct>   open-loop motor -100..100   X  stop + brake");
  say("Z         zero the encoder            C <cpm>  counts per metre");
  say("T <hz>    telemetry (0 = off)         G  settings   ?  help");
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
  else if (!strcmp(c, "?") || !strcmp(c, "H")) print_help();
  else say("ERR unknown command, type ?");
}

void App_Init(void) {
  comms_init();
  hw_init();
  HAL_Delay(200);
  say("MR2006B Team 7 - Activity 2.5 base ready. Type ? for commands.");
  print_settings();
}

void App_Loop(void) {
  comms_poll(handle_command);
  if (telemPeriodMs && HAL_GetTick() - telemLastMs >= telemPeriodMs) {
    telemLastMs = HAL_GetTick();
    telemetry_send();
  }
  comms_pump();
}
