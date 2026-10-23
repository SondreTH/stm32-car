/* =====================================================================
 * car_app.c - application layer.
 *  - parses incoming command lines (USB or Bluetooth)
 *  - redraws the OLED display at 5 Hz
 *  - sends periodic telemetry when enabled (T command)
 *  - watches the start buttons (B1 blue button, or PB0 external)
 * ===================================================================== */
#include "main.h"
#include "car_app.h"
#include "car_comms.h"
#include "car_config.h"
#include "car_drive.h"
#include "car_hw.h"
#include "car_oled.h"
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

static bool     oledOn = false;
static uint8_t  oledSteps = 0;
static uint32_t telemPeriodMs = 0;
static uint32_t telemLastMs = 0;
static bool     btnWas = false;
static bool     armed = false;
static uint32_t armedMs = 0;

/* ----------------------------------------------------------- telemetry */
static void telemetry_header(void) {
  lb_clear();
  lb_str("ms,cnt,dist_cm,spd_cms,sp_cms,mot_pct,ref_cm,sonar_cm");
  lb_send(false);
}

static void telemetry_send(void) {
  lb_clear();
  lb_uint(HAL_GetTick());      lb_str(",");
  lb_int(encoder_count());     lb_str(",");
  lb_float(Real_Dist, 1);      lb_str(",");
  lb_float(AverageSpeed, 1);   lb_str(",");
  lb_float(SP_Vel, 1);         lb_str(",");
  lb_float(motorPct, 1);       lb_str(",");
  lb_float(Pos_Ref, 1);        lb_str(",");
  lb_float(sonarCm, 0);
  lb_send(true);
}

/* ------------------------------------------------------------- replies */
static void print_help(void) {
  say("Commands: d<dist_m> | tt<sec> | ta<sec> | td<sec> | kp<val> | ki<val> | kd<val>");
  say("          v<cm/s> | f | b | 1..6 | s | x | l | c | r | p<pct> | z | m<cpm> | t<hz> | g | i | ?");
}

static void print_settings(void) {
  lb_clear();
  lb_str("cpm="); lb_float(COUNTS_PER_METER, 1);
  lb_str(" Kp="); lb_float(Kp, 2); lb_str(" Ki="); lb_float(Ki, 2); lb_str(" Kd="); lb_float(Kd, 2);
  lb_str(" tt="); lb_float(run_T, 1); lb_str(" ta="); lb_float(run_ta, 1); lb_str(" td="); lb_float(run_td, 1);
  lb_str(" sonar="); lb_float(sonarCm, 0); lb_str("cm");
  lb_str(" batt="); lb_float(battery_volts(), 2); lb_str("V");
  lb_str(" counts="); lb_int(encoder_count());
  lb_send(false);
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
  static const char *const phase[] = { "IDLE", "ACCEL", "CRUISE", "DECEL" };
  oled_clear();
  oled_text(0, 3, "Ninjago", 1);
  int x = oled_float(54, 3, battery_volts(), 1, 1);
  oled_text(x, 3, "V", 1);

  if (sonarCm >= 200.0f) {
    oled_text(88, 3, "---cm", 1);
  } else {
    x = oled_int(88, 3, (int32_t)sonarCm, 1);
    oled_text(x, 3, "cm", 1);
  }
  oled_fill(0, 13, OLED_WIDTH, 1, true);

  /* what the car is doing, and the distance of the run */
  if (armed) {
    x = oled_text(0, 18, "GO IN ", 1);
    x = oled_float(x, 18, (int32_t)(BUTTON_DELAY_MS - (HAL_GetTick() - armedMs)) / 1000.0f, 1, 1);
    oled_text(x, 18, "s", 1);
  } else if (state != 0 && obstacle) {
    oled_text(0, 18, "OBSTACLE", 1);
  } else if (state >= 1 && state <= 3) {
    oled_text(0, 18, phase[state], 1);
  } else if (btRxBytes > 0) {
    /* Show live Bluetooth bytes received and last character */
    x = oled_text(0, 18, "BT:", 1);
    x = oled_int(x, 18, (int32_t)btRxBytes, 1);
    char buf[5] = { ' ', '\'', (lastBtChar >= 32 && lastBtChar <= 126) ? lastBtChar : '?', '\'', 0 };
    oled_text(x, 18, buf, 1);
  } else {
    oled_text(0, 18, motorPct != 0 ? "MANUAL" : phase[0], 1);
  }

  if (armed || state != 0) {
    x = oled_float(72, 18, armed ? BUTTON_RUN_M : target_dist_cm / 100.0f, 2, 1);
    oled_text(x, 18, " m", 1);
  }

  /* distance since the run started (or Z), big; motor % on the right */
  x = oled_float(0, 28, Real_Dist, 1, 2);
  oled_text(x, 36, " cm", 1);
  x = oled_int(98, 36, (int32_t)motorPct, 1);
  oled_text(x, 36, "%", 1);

  /* measured speed and setpoint */
  x = oled_float(0, 45, AverageSpeed, 1, 1);
  oled_text(x, 45, " cm/s", 1);
  x = oled_text(66, 45, "set ", 1);
  oled_float(x, 45, SP_Vel, 1, 1);

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

/* ------------------------------------------------------- start button */
/* B1 or PB0: arm on the press, start the run BUTTON_DELAY_MS later */
static void button_poll(void) {
  bool b = start_pressed();
  if (b && !btnWas && state == 0 && !armed) {
    armed = true;
    armedMs = HAL_GetTick();
    lb_clear(); lb_str("button: "); lb_float(BUTTON_RUN_M, 2); lb_str(" m run in ");
    lb_float(BUTTON_DELAY_MS / 1000.0f, 1); lb_str(" s"); lb_send(false);
  }
  btnWas = b;
  if (armed && HAL_GetTick() - armedMs >= BUTTON_DELAY_MS) {
    armed = false;
    if (!drive_start(BUTTON_RUN_M)) say("ERR run does not fit: check tt, ta, td");
  }
}

/* any manual command ends a run and a pending button start */
static void manual(void) { armed = false; drive_cancel(); }

static void handle_command(char *line) {
  while (*line == ' ' || *line == '\t') line++;
  if (!*line) return;
  char c = (char)toupper((unsigned char)line[0]);
  float a1 = strtof(line + 1, NULL);   /* 0 when no number follows */

  /* two-letter settings: tt/ta/td (run times) and kp/ki/kd (PID gains) */
  char c2 = (char)tolower((unsigned char)line[1]);
  float a2 = c2 ? strtof(line + 2, NULL) : 0;
  if (c == 'T' && (c2 == 't' || c2 == 'a' || c2 == 'd')) {
    if (a2 < 0) { say("ERR time in seconds"); return; }
    if (c2 == 't') run_T = a2; else if (c2 == 'a') run_ta = a2; else run_td = a2;
    print_settings();
    return;
  }
  if (c == 'K' && (c2 == 'p' || c2 == 'i' || c2 == 'd')) {
    if (c2 == 'p') Kp = a2; else if (c2 == 'i') Ki = a2; else Kd = a2;
    print_settings();
    return;
  }

  if (c == 'D') {
    if (a1 > 0.01f) {
      armed = false;
      if (drive_start(a1)) {
        lb_clear(); lb_str("OK run "); lb_float(a1, 2); lb_str(" m in "); lb_float(T, 1); lb_str(" s at ");
        lb_float(Sn, 1); lb_str(" cm/s"); lb_send(false);
      } else {
        lb_clear(); lb_str("ERR does not fit: need tt >= ta + td and speed <= "); lb_float(DRIVE_MAX_CMS, 0);
        lb_str(" cm/s"); lb_send(false);
      }
    } else say("ERR distance in metres, e.g. d1.2");
  }
  else if (c == 'V') { manual(); Set_Speed_cm_s(a1); say("OK speed (open loop)"); }
  else if (c == 'F') { manual(); motor_set(MANUAL_PCT); say("OK forward"); }
  else if (c == 'B') { manual(); motor_set(-MANUAL_PCT); say("OK back"); }
  else if (c == 'S') { manual(); drive_stop(); say("OK stop"); }
  else if (c >= '1' && c <= '6') {   /* direct drive or change speed while moving */
    float pct = (float)((c - '0' + 4) * 10);
    manual();
    motor_set(motorPct < 0 ? -pct : pct);
    lb_clear(); lb_str("OK "); lb_float(pct, 0); lb_str(" %"); lb_send(false);
  }
  else if (c == 'L') { servo_set_us(SERVO_PULSE_MIN);    say("OK left"); }
  else if (c == 'C') { servo_set_us(SERVO_PULSE_CENTER); say("OK centre"); }
  else if (c == 'R') { servo_set_us(SERVO_PULSE_MAX);    say("OK right"); }
  else if (c == 'X') { manual(); motor_brake(); say("OK stop + brake"); }
  else if (c == 'P') { manual(); motor_set(a1); say("OK open-loop"); }
  else if (c == 'Z') { if (state) say("ERR not during a run"); else { encoder_zero(); say("OK zeroed"); } }
  else if (c == 'M') { if (a1 > 0) COUNTS_PER_METER = a1; print_settings(); }
  else if (c == 'T') { telemPeriodMs = (a1 > 0) ? (uint32_t)(1000.0f / a1) : 0; if (telemPeriodMs) telemetry_header(); }
  else if (c == 'G') print_settings();
  else if (c == 'I') i2c_report();
  else if (c == '?' || c == 'H') print_help();
  else say("ERR unknown command, type ?");
}

void App_Init(void) {
  comms_init();
  hw_init();
  HAL_Delay(200);
  say("MR2006B Team 7 (Ninjago) car ready. Type ? for commands.");
  print_settings();
  oled_start();
}

void App_Loop(void) {
  comms_poll(handle_command);
  button_poll();
  /* the control step runs in the TIM3 interrupt; redraw the OLED (~25 ms)
   * after every second step */
  static uint32_t lastStep = 0;
  bool stepped = (driveSteps != lastStep);
  lastStep = driveSteps;
  if (stepped && oledOn && ++oledSteps >= 2) {
    oledSteps = 0;
    oled_draw_status();
    if (!oled_update()) { oledOn = false; say("OLED lost: check wiring, then type I"); }
  }
  if (telemPeriodMs && HAL_GetTick() - telemLastMs >= telemPeriodMs) {
    telemLastMs = HAL_GetTick();
    telemetry_send();
  }
  comms_pump();
}
