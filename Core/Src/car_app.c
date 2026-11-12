/* =====================================================================
 * car_app.c - the car: distance runs, manual driving, steering, OLED
 *
 * Commands work on the USB virtual COM port (115200) and Bluetooth (9600).
 * One letter, then an optional number ("d1.2" and "D 1.2" are the same):
 *   d <m>     distance run: speed profile + position PID (car_drive.c),
 *             negative = in reverse
 *   tt <s>    run time (0 = nominal speed)  ta / td <s>  accel / decel time
 *   vn <cm/s> nominal speed                 tol <cm>  how close counts as there
 *   kp ki kd <gain>  PID gains
 *   ms mo mg  motor: start %, rolling offset %, % per cm/s
 *   vb <V>    battery voltage from a multimeter: calibrates the reading
 *   v <cm/s>  constant speed, open loop    s   stop (coast)
 *   f / b     forward / back at 60 %       1..6  50..100 %, same direction
 *   l c r     steer left / centre / right
 *   P <pct>   open-loop motor -100..100    X   stop + brake
 *   Z         zero the encoder             M <cpm>  counts per metre
 *   T <hz>    telemetry on (0 = off)       G   show settings
 *   I         I2C scan + restart the OLED  ?   help
 * B1 or the start button (PB0) starts a 1 m run after 1 s.
 *
 * The OLED (I2C1, car_oled.c) shows the run, speed, motor %, battery and
 * sonar distance at 5 Hz.
 * ===================================================================== */
#include "main.h"
#include "car_app.h"
#include "car_config.h"
#include "car_hw.h"
#include "car_comms.h"
#include "car_drive.h"
#include "car_oled.h"
#include "car_heading.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

static uint32_t telemPeriodMs = 0, telemLastMs = 0;
static bool     oledOn = false;
static int      oledSteps = 0;
static bool     armed = false, btnWas = false;   /* button pressed, run starts after BUTTON_DELAY_MS */
static uint32_t armedMs = 0;

static void print_help(void) {
  say("d <m>     distance run, e.g. d1.2 (d-0.5 = reverse)   s  stop");
  say("v <cm/s>  hold a speed (speed loop)   1..6  50..100 % same direction");
  lb_clear(); lb_str("f / b     forward / back at "); lb_float(MANUAL_PCT, 0);
  lb_str(" %   l c r  steer left / centre / right"); lb_send(false);
  say("P <pct>   open-loop motor -100..100   X  stop + brake   Z  zero encoder");
  say("M <cpm>   counts per metre            T <hz>  telemetry (0 = off)");
  say("G         settings   I  I2C scan, restart the OLED   ?  help");
  say("tt <s>    run time, 0 = nominal speed  ta <s> / td <s>  accel / decel time");
  say("vn <cm/s> nominal speed               tol <cm>  stop this close to the target");
  say("kp ki kd <gain>  PID gains, e.g. kp 3");
  say("ms / mo <pct>  motor % to start / to keep rolling   mg <pct per cm/s>");
  say("vb <volts>  battery voltage from a multimeter, calibrates the reading");
  say("pf <Hz>   motor PWM frequency 50..5000 (lower = more torque at low %)");
  say("sp / si <gain>  speed loop gains     kh <us/deg>  heading hold, 0 = off");
  say("sc <us>   servo pulse that drives straight   Y  zero the heading");
  lb_clear(); lb_str("B1 / start button: "); lb_float(BUTTON_RUN_M, 2); lb_str(" m run after ");
  lb_float(BUTTON_DELAY_MS / 1000.0f, 1); lb_str(" s"); lb_send(false);
}

static void print_settings(void) {
  lb_clear(); lb_str("CPM="); lb_float(COUNTS_PER_METER, 1);
  lb_str(" Kp="); lb_float(Kp, 2); lb_str(" Ki="); lb_float(Ki, 2); lb_str(" Kd="); lb_float(Kd, 2);
  lb_str(" tt="); lb_float(run_T, 1); lb_str(" ta="); lb_float(run_ta, 1); lb_str(" td="); lb_float(run_td, 1);
  lb_str(" vn="); lb_float(DRIVE_SPEED_CMS, 1); lb_str(" tol="); lb_float(POS_TOL_CM, 1);
  lb_str(" sp="); lb_float(Kvp, 2); lb_str(" si="); lb_float(Kvi, 2);
  lb_send(false);
  lb_clear(); lb_str("steer sc="); lb_float(SERVO_CENTER_US, 0); lb_str(" kh="); lb_float(STEER_KH, 1);
  if (imuOk) { lb_str(" yaw="); lb_float(yawDeg, 1); lb_str("deg rate="); lb_float(yawRateDps, 1);
               lb_str("dps bias="); lb_float(gyroBiasDps, 2); }
  else lb_str(" gyro NOT found");
  lb_send(false);
  lb_clear(); lb_str("motor ms="); lb_float(MOTOR_START_PCT, 1); lb_str(" mo="); lb_float(MOTOR_MIN_PCT, 1);
  lb_str(" mg="); lb_float(MOTOR_PCT_PER_CMS, 2);
  lb_str(" pf="); lb_float(motor_pwm_hz_now(), 0);
  lb_str(" sonar="); lb_float(sonarCm, 0); lb_str("cm");
  lb_str(" batt="); lb_float(battery_volts(), 2); lb_str("V (PA4 "); lb_float(battery_pin_volts(), 2);
  lb_str("V x "); lb_float(BATT_DIVIDER, 2); lb_str(")");
  lb_str(" counts="); lb_int(encoder_count());
  lb_send(false);
}

static void telemetry_header(void) {
  say("t_ms,state,sp_cms,speed_cms,dist_cm,counts,motor_pct,ref_cm,sonar_cm,yaw_deg,servo_us");
}

static void telemetry_send(void) {
  lb_clear();
  lb_uint(HAL_GetTick());      lb_str(",");
  lb_int(state);               lb_str(",");
  lb_float(SP_Vel, 2);         lb_str(",");
  lb_float(AverageSpeed, 2);   lb_str(",");
  lb_float(Real_Dist, 1);      lb_str(",");
  lb_int(encoder_count());     lb_str(",");
  lb_float(motorPct, 1);       lb_str(",");
  lb_float(Pos_Ref, 1);        lb_str(",");
  lb_float(sonarCm, 0);        lb_str(",");
  lb_float(state != 0 ? headingErrDeg : yawDeg, 1); lb_str(",");
  lb_uint(servoUs);
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

static void gyro_start(void) {
  if (state != 0) return;   /* the bias needs a standing car */
  lb_clear();
  if (heading_init()) { lb_str("gyro OK, bias "); lb_float(gyroBiasDps, 2); lb_str(" deg/s (car must stand still at start-up)"); }
  else                  lb_str("gyro (MPU-6050) not found: no heading hold. Check wiring, then type I");
  lb_send(false);
}

/* What the screen shows. Redrawn from scratch at 5 Hz: change freely.
 * Two-colour modules: rows 0-15 are yellow, 16-63 blue, so nothing crosses y = 16. */
static void oled_draw_status(void) {
  static const char *const phase[] = { "IDLE", "ACCEL", "CRUISE", "DECEL", "SETTLE" };
  oled_clear();
  oled_text(0, 3, "Ninjago", 1);
  int x = oled_float(54, 3, battery_volts(), 1, 1);
  oled_text(x, 3, "V", 1);
  x = oled_int(92, 3, (int32_t)sonarCm, 1);
  oled_text(x, 3, "cm", 1);
  oled_fill(0, 13, OLED_WIDTH, 1, true);

  /* what the car is doing, and the distance of the run */
  if (armed) {
    x = oled_text(0, 18, "GO IN ", 1);
    x = oled_float(x, 18, (int32_t)(BUTTON_DELAY_MS - (HAL_GetTick() - armedMs)) / 1000.0f, 1, 1);
    oled_text(x, 18, "s", 1);
  } else if (state != 0 && obstacle) oled_text(0, 18, "OBSTACLE", 1);
  else if (state >= DRIVE_ACCEL && state <= DRIVE_SETTLE) oled_text(0, 18, phase[state], 1);
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
  if (imuOk) {   /* heading: off course during a run, total otherwise */
    x = oled_float(72, 45, state != 0 ? headingErrDeg : yawDeg, 1, 1);
    oled_text(x, 45, " deg", 1);
  } else {
    x = oled_text(66, 45, "set ", 1);
    oled_float(x, 45, SP_Vel, 1, 1);
  }

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
  gyro_start();
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

/* the number after the letters of a command: "tol 1.5" -> 1.5, 0 if there is none */
static float number_after(const char *s) {
  while (isalpha((unsigned char)*s)) s++;
  return strtof(s, NULL);
}

/* a run just ended: say how it went */
static void report_run(int result) {
  static const char *const why[] = { "", "DONE", "DONE, not within tol in time", "DONE, stopped after one swing back",
    "ERR motor on but no encoder counts: wheel blocked or encoder unplugged?",
    "ERR encoder counts the wrong way: change ENCODER_INVERT in car_config.h" };
  lb_clear(); lb_str(why[result]);
  if (result <= RUN_SWINGS) {
    float err = Real_Dist - target_dist_cm;
    lb_str(": "); lb_float(Real_Dist, 1); lb_str(" cm, ");
    lb_str(err < 0 ? "short by " : "past by "); lb_float(fabsf(err), 1); lb_str(" cm");
  }
  lb_send(false);
}

static void handle_command(char *line) {
  while (*line == ' ' || *line == '\t') line++;
  if (!*line) return;
  char c = (char)toupper((unsigned char)line[0]);
  float a1 = strtof(line + 1, NULL);   /* 0 when no number follows */

  /* two-letter settings: tt/ta/td (run times) and kp/ki/kd (PID gains) */
  char c2 = (char)tolower((unsigned char)line[1]);
  float a2 = number_after(line);
  if (c == 'T' && (c2 == 't' || c2 == 'a' || c2 == 'd')) {
    if (a2 < 0) { say("ERR time in seconds"); return; }
    if (c2 == 't') run_T = a2; else if (c2 == 'a') run_ta = a2; else run_td = a2;
    print_settings();
    return;
  }
  if (c == 'T' && c2 == 'o') {   /* tol */
    if (a2 < 0.2f) { say("ERR tolerance in cm, at least 0.2"); return; }
    POS_TOL_CM = a2;
    print_settings();
    return;
  }
  if (c == 'K' && (c2 == 'p' || c2 == 'i' || c2 == 'd')) {
    if (a2 < 0) { say("ERR gains are positive"); return; }
    if (c2 == 'p') Kp = a2; else if (c2 == 'i') Ki = a2; else Kd = a2;
    print_settings();
    return;
  }
  if (c == 'M' && (c2 == 's' || c2 == 'o' || c2 == 'g')) {
    if (a2 <= 0 || (c2 != 'g' && a2 > 100)) { say("ERR ms / mo in % (1..100), mg in % per cm/s"); return; }
    if (c2 == 's') MOTOR_START_PCT = a2; else if (c2 == 'o') MOTOR_MIN_PCT = a2; else MOTOR_PCT_PER_CMS = a2;
    print_settings();
    return;
  }
  if (c == 'V' && c2 == 'n') {
    if (a2 < 2 || a2 > DRIVE_MAX_CMS) { say("ERR nominal speed in cm/s"); return; }
    DRIVE_SPEED_CMS = a2;
    print_settings();
    return;
  }
  if (c == 'V' && c2 == 'b') {   /* the multimeter says a2 volts: work out the divider */
    float pin = battery_pin_volts();
    if (a2 < 5 || pin < 0.2f) { say("ERR vb <battery volts>, and PA4 must see the battery"); return; }
    BATT_DIVIDER = a2 / pin;
    lb_clear(); lb_str("OK BATT_DIVIDER = "); lb_float(BATT_DIVIDER, 2);
    lb_str(": copy it to car_config.c"); lb_send(false);
    return;
  }
  if (c == 'S' && (c2 == 'p' || c2 == 'i')) {   /* speed loop gains */
    if (a2 < 0) { say("ERR gains are positive"); return; }
    if (c2 == 'p') Kvp = a2; else Kvi = a2;
    print_settings();
    return;
  }
  if (c == 'S' && c2 == 'c') {   /* servo pulse for straight ahead */
    if (a2 < SERVO_PULSE_MIN || a2 > SERVO_PULSE_MAX) { say("ERR sc <us> between full left and full right"); return; }
    SERVO_CENTER_US = a2;
    servo_set_us((uint32_t)a2);
    print_settings();
    return;
  }
  if (c == 'K' && c2 == 'h') {   /* heading hold gain, may be negative to flip the direction */
    STEER_KH = a2;
    print_settings();
    return;
  }
  if (c == 'P' && c2 == 'f') {   /* motor PWM frequency, see motor_pwm_hz() */
    if (a2 < 50 || a2 > 5000) { say("ERR pf <Hz>, 50..5000"); return; }
    lb_clear(); lb_str("OK motor PWM "); lb_float(motor_pwm_hz(a2), 0); lb_str(" Hz"); lb_send(false);
    return;
  }

  if (c == 'D') {
    if (fabsf(a1) >= 0.01f) {
      armed = false;
      if (drive_start(a1)) {
        lb_clear(); lb_str("OK run "); lb_float(a1, 2); lb_str(" m in "); lb_float(T, 1); lb_str(" s at ");
        lb_float(Sn, 1); lb_str(" cm/s"); lb_send(false);
      } else {
        lb_clear(); lb_str("ERR does not fit: need tt >= ta + td and speed <= "); lb_float(DRIVE_MAX_CMS, 0);
        lb_str(" cm/s"); lb_send(false);
      }
    } else say("ERR distance in metres, e.g. d1.2 or d-0.5");
  }
  else if (c == 'V') { manual(); Set_Speed_cm_s(a1); say("OK speed"); }
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
  else if (c == 'C') { servo_set_us((uint32_t)SERVO_CENTER_US); say("OK centre"); }
  else if (c == 'R') { servo_set_us(SERVO_PULSE_MAX);    say("OK right"); }
  else if (c == 'X') { manual(); motor_brake(); say("OK stop + brake"); }
  else if (c == 'P') { manual(); motor_set(a1); say("OK open-loop"); }
  else if (c == 'Z') { if (state) say("ERR not during a run"); else { encoder_zero(); say("OK zeroed"); } }
  else if (c == 'M') { if (a1 > 0) COUNTS_PER_METER = a1; print_settings(); }
  else if (c == 'T') { telemPeriodMs = (a1 > 0) ? (uint32_t)(1000.0f / a1) : 0; if (telemPeriodMs) telemetry_header(); }
  else if (c == 'G') print_settings();
  else if (c == 'Y') { yawDeg = 0.0f; say("OK heading zeroed"); }
  else if (c == 'I') i2c_report();
  else if (c == '?' || c == 'H') print_help();
  else say("ERR unknown command, type ?");
}

void App_Init(void) {
  comms_init();
  hw_init();
  HAL_Delay(200);
  say("MR2006B Team 7 (Ninjago) car ready. Type ? for commands.");
  oled_start();
  gyro_start();
  print_settings();
}

void App_Loop(void) {
  comms_poll(handle_command);
  button_poll();
  heading_poll();
  int result = drive_result();
  if (result != RUN_NONE) report_run(result);
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
