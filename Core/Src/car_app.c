/* =====================================================================
 * car_app.c - the car: distance runs, manual driving, steering, OLED
 *
 * Commands work on the USB virtual COM port (115200) and Bluetooth (9600).
 * One letter, then an optional number ("d1.2" and "D 1.2" are the same):
 *   d <m>     distance run: speed profile + PID (car_drive.c)
 *   v <cm/s>  constant speed, open loop    s   stop (coast)
 *   f / b     forward / back at 60 %       1..6  50..100 %, same direction
 *   l c r     steer left / centre / right
 *   P <pct>   open-loop motor -100..100    X   stop + brake
 *   Z         zero the encoder             M <cpm>  counts per metre
 *   T <hz>    telemetry on (0 = off)       G   show settings
 *   I         I2C scan + restart the OLED  ?   help
 * B1 or the start button (PB0) starts a 1 m run after 1 s.
 *
 * The OLED (I2C1, car_oled.c) shows the run, speed, motor % and battery at 5 Hz.
 * ===================================================================== */
#include "main.h"
#include "car_app.h"
#include "car_config.h"
#include "car_hw.h"
#include "car_comms.h"
#include "car_drive.h"
#include "car_oled.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

static uint32_t telemPeriodMs = 0, telemLastMs = 0;
static bool     oledOn = false;
static int      oledSteps = 0;
static bool     armed = false, btnWas = false;   /* button pressed, run starts after BUTTON_DELAY_MS */
static uint32_t armedMs = 0;

static void print_help(void) {
  say("d <m>     distance run, e.g. d1.2     s  stop");
  say("v <cm/s>  constant speed, open loop   1..6  50..100 % same direction");
  lb_clear(); lb_str("f / b     forward / back at "); lb_float(MANUAL_PCT, 0);
  lb_str(" %   l c r  steer left / centre / right"); lb_send(false);
  say("P <pct>   open-loop motor -100..100   X  stop + brake   Z  zero encoder");
  say("M <cpm>   counts per metre            T <hz>  telemetry (0 = off)");
  say("G         settings   I  I2C scan, restart the OLED   ?  help");
  lb_clear(); lb_str("B1 / start button: "); lb_float(BUTTON_RUN_M, 2); lb_str(" m run after ");
  lb_float(BUTTON_DELAY_MS / 1000.0f, 1); lb_str(" s"); lb_send(false);
}

static void print_settings(void) {
  lb_clear(); lb_str("CPM="); lb_float(COUNTS_PER_METER, 1);
  lb_str(" Kp="); lb_float(Kp, 2); lb_str(" Ki="); lb_float(Ki, 2); lb_str(" Kd="); lb_float(Kd, 2);
  lb_str(" batt="); lb_float(battery_volts(), 2); lb_str("V");
  lb_str(" counts="); lb_int(encoder_count());
  lb_send(false);
}

static void telemetry_header(void) {
  say("t_ms,state,sp_cms,speed_cms,dist_cm,counts,motor_pct");
}

static void telemetry_send(void) {
  lb_clear();
  lb_uint(HAL_GetTick());      lb_str(",");
  lb_int(state);               lb_str(",");
  lb_float(SP_Vel, 2);         lb_str(",");
  lb_float(AverageSpeed, 2);   lb_str(",");
  lb_float(Real_Dist, 1);      lb_str(",");
  lb_int(encoder_count());     lb_str(",");
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
  static const char *const phase[] = { "IDLE", "ACCEL", "CRUISE", "DECEL" };
  oled_clear();
  oled_text(0, 3, "Ninjago", 1);
  int x = oled_float(54, 3, battery_volts(), 1, 1);
  oled_text(x, 3, "V", 1);
  x = oled_int(92, 3, (int32_t)(HAL_GetTick() / 1000), 1);
  oled_text(x, 3, "s", 1);
  oled_fill(0, 13, OLED_WIDTH, 1, true);

  /* what the car is doing, and the distance of the run */
  if (armed) {
    x = oled_text(0, 18, "GO IN ", 1);
    x = oled_float(x, 18, (int32_t)(BUTTON_DELAY_MS - (HAL_GetTick() - armedMs)) / 1000.0f, 1, 1);
    oled_text(x, 18, "s", 1);
  } else if (state >= 1 && state <= 3) oled_text(0, 18, phase[state], 1);
  else oled_text(0, 18, motorPct != 0 ? "MANUAL" : phase[0], 1);
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
    Start_Motion_Profile(BUTTON_RUN_M, DRIVE_SPEED_CMS, DRIVE_RAMP_S);
  }
}

/* any manual command ends a run and a pending button start */
static void manual(void) { armed = false; drive_cancel(); }

static void handle_command(char *line) {
  while (*line == ' ' || *line == '\t') line++;
  if (!*line) return;
  char c = (char)toupper((unsigned char)line[0]);
  float a1 = strtof(line + 1, NULL);   /* 0 when no number follows */

  if (c == 'D') {
    if (a1 > 0.01f) {
      armed = false;
      Start_Motion_Profile(a1, DRIVE_SPEED_CMS, DRIVE_RAMP_S);
      lb_clear(); lb_str("OK run "); lb_float(a1, 2); lb_str(" m in "); lb_float(T, 1); lb_str(" s");
      lb_send(false);
    } else say("ERR distance in metres, e.g. d1.2");
  }
  else if (c == 'V') { manual(); Set_Speed_cm_s(a1); say("OK speed (open loop)"); }
  else if (c == 'F') { manual(); motor_set(MANUAL_PCT); say("OK forward"); }
  else if (c == 'B') { manual(); motor_set(-MANUAL_PCT); say("OK back"); }
  else if (c == 'S') { manual(); drive_stop(); say("OK stop"); }
  else if (c >= '1' && c <= '6') {   /* only changes the speed while moving */
    float pct = (float)((c - '0' + 4) * 10);
    manual();
    if (motorPct == 0) say("ERR not moving: f or b first");
    else {
      motor_set(motorPct < 0 ? -pct : pct);
      lb_clear(); lb_str("OK "); lb_float(pct, 0); lb_str(" %"); lb_send(false);
    }
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
  /* the OLED write takes ~25 ms: do it right after a 100 ms control step,
   * every second one, so it never delays the next step */
  if (drive_update() && oledOn && ++oledSteps >= 2) {
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
