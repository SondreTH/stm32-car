/* =====================================================================
 * car_app.c - MR2006B Team 7 car application (STM32CubeIDE version)
 *
 * MANUAL  drive with commands (V, S, P, D ...)
 * TEACH   drive the course once; the car records its path (REC/WP/END)
 * AUTO    the car drives the recorded path by itself (GO or button)
 *
 * Commands work on the USB virtual COM port (115200) and Bluetooth (9600).
 * ===================================================================== */
#include "main.h"
#include "car_app.h"
#include "car_config.h"
#include "car_hw.h"
#include "car_comms.h"
#include "car_drive.h"
#include "path.h"
#include "autopilot.h"
#include "path_data.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

typedef enum { CM_MANUAL = 0, CM_TEACH = 1, CM_AUTO = 2 } CarMode;
CarMode carMode = CM_MANUAL;     /* global: visible in Live Expressions */

static uint32_t watchdogMs = 0, lastCommandMs = 0;
static uint32_t telemPeriodMs = 0, telemLastMs = 0;
static uint32_t battWarnMs = 0;

/* ------------------------------------------------------------ printing */
static void print_settings(void) {
  lb_clear(); lb_str("CPM="); lb_float(COUNTS_PER_METER, 1);
  lb_str(" FF="); lb_float(SPEED_FF, 1); lb_str(" KP="); lb_float(SPEED_KP, 1);
  lb_str(" KI="); lb_float(SPEED_KI, 1); lb_str(" DB="); lb_float(MOTOR_DEADBAND_PCT, 1);
  lb_send(false);
  lb_clear(); lb_str("SERVO center="); lb_int(SERVO_CENTER_US);
  lb_str(" min="); lb_int(SERVO_MIN_US); lb_str(" max="); lb_int(SERVO_MAX_US);
  lb_str(" us/deg="); lb_float(SERVO_US_PER_DEG, 2);
  lb_send(false);
  lb_clear(); lb_str("IMU "); lb_str(imuOk ? "OK" : "NOT FOUND");
  lb_str(" whoami=0x"); lb_hex(imuWhoAmI);
  lb_str(" bias="); lb_float(gyroBiasDps, 3); lb_str(" dps scale="); lb_float(GYRO_SCALE, 4);
  lb_send(false);
  lb_clear(); lb_str("AUTO cruise="); lb_float(AP_CRUISE, 2); lb_str(" lookahead="); lb_float(AP_LOOKAHEAD_M, 2);
  lb_str(" wheelbase="); lb_float(WHEELBASE_M, 3); lb_str(" obst_stop="); lb_float(AP_OBST_STOP_CM, 0);
  lb_str("cm reroute="); lb_int(AP_REROUTE ? 1 : 0); lb_str(" detour="); lb_float(AP_DETOUR_OFFSET_M, 2);
  lb_send(false);
  lb_clear(); lb_str("BATT "); lb_float(battV, 2); lb_str(" V   mode=");
  lb_str(carMode == CM_TEACH ? "TEACH" : carMode == CM_AUTO ? "AUTO" : "MANUAL");
  lb_str("   path="); lb_uint(path.n); lb_str(" pts / "); lb_float(path.total, 1); lb_str(" m");
  lb_send(false);
}

static void print_help(void) {
  say("--- manual ---");
  say("X            stop + brake (also stops AUTO)");
  say("V <m/s>      closed-loop speed      S <deg>   steer, + = left");
  say("P <pct>      open-loop motor        D <m> [m/s]  drive distance");
  say("--- teach & auto ---");
  say("REC          start recording a path here (= START)");
  say("WP           mark a waypoint here   NOOBS  toggle obstacle-off zone");
  say("END          mark the finish here, stop recording");
  say("GO           run AUTO (button does the same)");
  say("PATH  DUMP   show / print the path   LOAD  reload built-in path");
  say("PC           clear path   PA <x> <y> [flags]  append point");
  say("AS <m/s>  LA <m>  OB <cm>  RR <0/1>  DO <m>   cruise, lookahead, obstacle stop, reroute, detour");
  say("--- setup ---");
  say("Z  zero pose   GC  gyro cal   GS <k>  gyro scale   C <cpm>  counts/m");
  say("K <kp> <ki> [ff]   B <pct> deadband   U <us> raw servo");
  say("SC/SL/SR <us> servo center/min/max   WB <m> wheelbase");
  say("T <hz> telemetry   W <ms> watchdog   G settings");
}

static void telemetry_header(void) {
  say("t_ms,drive,car,ap,counts,dist_m,v_mps,set_mps,motor_pct,steer_deg,sonar_cm,heading_deg,x_m,y_m,s_m,batt_v");
}

static void telemetry_send(void) {
  lb_clear();
  lb_uint(hw_millis());      lb_str(",");
  lb_int(drv.mode);          lb_str(",");
  lb_int(carMode);           lb_str(",");
  lb_int(ap.state);          lb_str(",");
  lb_int(encoder_count());   lb_str(",");
  lb_float(drv.distance, 3); lb_str(",");
  lb_float(drv.speed, 3);    lb_str(",");
  lb_float(drv.setpoint, 3); lb_str(",");
  lb_float(motorPct, 1);     lb_str(",");
  lb_float(steerDeg, 1);     lb_str(",");
  lb_float(sonarCm, 0);      lb_str(",");
  lb_float(headingDeg, 1);   lb_str(",");
  lb_float(drv.x, 3);        lb_str(",");
  lb_float(drv.y, 3);        lb_str(",");
  lb_float(fol.sNow, 2);     lb_str(",");
  lb_float(battV, 2);
  lb_send(true);
}

/* ------------------------------------------------------------- path */
static void print_path_info(void) {
  lb_clear(); lb_str("PATH "); lb_uint(path.n); lb_str(" points, "); lb_float(path.total, 2); lb_str(" m");
  lb_send(false);
  int k = 0;
  for (uint16_t i = 0; i < path.n; i++) {
    if (!(path.p[i].flags & (PF_STOP | PF_FINISH))) continue;
    lb_clear();
    if (path.p[i].flags & PF_FINISH) lb_str("  FINISH"); else { lb_str("  STOP "); lb_int(++k); }
    lb_str(" at s="); lb_float(path.s[i], 2);
    lb_str(" m  x="); lb_float(path.p[i].x, 2); lb_str(" y="); lb_float(path.p[i].y, 2);
    lb_send(false);
  }
}

/* prints the path as a complete path_data.h (car_console.py saves it) */
static void dump_path(void) {
  say("// ==== PATH BEGIN ====");
  say("// path_data.h - recorded with TEACH, saved with DUMP");
  say("// flags: 1 = STOP (waypoint), 2 = FINISH, 4 = obstacle detection off");
  say("#pragma once");
  say("#include \"path.h\"");
  say("static const PathPt BUILTIN_PATH[] = {");
  for (uint16_t i = 0; i < path.n; i++) {
    lb_clear(); lb_str("  {"); lb_float(path.p[i].x, 3); lb_str("f, ");
    lb_float(path.p[i].y, 3); lb_str("f, "); lb_uint(path.p[i].flags); lb_str("},");
    lb_send(false);
    comms_pump();
  }
  say("};");
  say("static const uint16_t BUILTIN_PATH_N = sizeof(BUILTIN_PATH) / sizeof(BUILTIN_PATH[0]);");
  say("// ==== PATH END ====");
}

/* ------------------------------------------------------------- teach */
static uint8_t recFlags = 0, recStops = 0;
static float   recX = 0, recY = 0;

static void teach_start(void) {
  ap_abort(); drive_stop(); steer_set_deg(0);
  say("TEACH: keep the car still at the START position...");
  imu_calibrate(1000);
  drive_zero(); headingDeg = 0;
  path_clear(); path_add(0, 0, 0);
  recX = recY = 0; recFlags = 0; recStops = 0;
  carMode = CM_TEACH;
  say("TEACH recording. Drive with V / S. At each waypoint: stop exactly on it, send WP.");
  say("NOOBS toggles an obstacle-free zone. At the finish: stop on it, send END.");
}

static void teach_update(void) {
  if (carMode != CM_TEACH) return;
  if (hypotf(drv.x - recX, drv.y - recY) >= REC_SPACING_M) {
    if (!path_add(drv.x, drv.y, recFlags)) {
      drive_stop(); carMode = CM_MANUAL;
      say("TEACH: path memory full - recording stopped");
      return;
    }
    recX = drv.x; recY = drv.y;
  }
}

static void teach_mark(uint8_t flag) {
  if (carMode != CM_TEACH) { say("ERR not recording - send REC first"); return; }
  drive_stop();
  path_add(drv.x, drv.y, (uint8_t)(flag | recFlags));
  recX = drv.x; recY = drv.y;
  lb_clear();
  if (flag & PF_STOP) { recStops++; lb_str("WAYPOINT "); lb_uint(recStops); lb_str(" recorded"); }
  else lb_str("FINISH recorded");
  lb_str(" at x="); lb_float(drv.x, 2); lb_str(" y="); lb_float(drv.y, 2);
  lb_str(" (path "); lb_float(path.total, 1); lb_str(" m)");
  lb_send(false);
}

static void teach_end(void) {
  if (carMode != CM_TEACH) { say("ERR not recording - send REC first"); return; }
  teach_mark(PF_FINISH);
  carMode = CM_MANUAL;
  print_path_info();
  lb_clear(); lb_str("Loop closure: odometry says finish is ");
  lb_float(hypotf(drv.x, drv.y) * 100, 0); lb_str(" cm from start (0 = perfect)");
  lb_send(false);
  say("TEACH done. Send DUMP to save it, or GO to replay it now.");
}

/* -------------------------------------------------------------- auto */
static uint32_t autoCountdownAt = 0, autoStartMs = 0;

static void auto_cancel(const char *why) {
  ap_abort(); autoCountdownAt = 0; carMode = CM_MANUAL; drive_stop(); say(why);
}

static void auto_arm(void) {
  if (path.n < 2) { say("ERR no path - REC a path or LOAD one"); return; }
  ap_abort(); drive_stop(); steer_set_deg(0);
  carMode = CM_AUTO;
  autoCountdownAt = hw_millis() + 3000;
  if (!autoCountdownAt) autoCountdownAt = 1;
  say("AUTO: starting in 3 s - hands off, car must be still at the START");
}

static void auto_print_event(void) {
  lb_clear(); lb_str("[AUTO "); lb_float((hw_millis() - autoStartMs) / 1000.0f, 1); lb_str(" s] ");
  switch (ap.ev) {
    case EV_WAYPOINT:    lb_str("WAYPOINT "); lb_int((int32_t)ap.evA); lb_str(" reached - waiting 10 s. Off-path ");
                         lb_float(ap.evB * 100, 1); lb_str(" cm"); break;
    case EV_FINISHED:    lb_str("FINISH reached. Off-path "); lb_float(ap.evB * 100, 1); lb_str(" cm"); break;
    case EV_LEAVING:     lb_str("driving to stop "); lb_int((int32_t)ap.evA); break;
    case EV_OBSTACLE:    lb_str("OBSTACLE at "); lb_float(ap.evA, 0); lb_str(" cm - waiting"); break;
    case EV_CLEARED:     lb_str("obstacle gone - continuing"); break;
    case EV_REROUTE:     lb_str("obstacle did not move - rerouting"); break;
    case EV_DETOUR_DONE: lb_str("detour done, back on the path"); break;
    case EV_NO_PATH:     lb_str("no path loaded"); break;
    default: break;
  }
  lb_send(false);
  ap.ev = EV_NONE;
}

static void auto_update(void) {
  if (carMode != CM_AUTO) return;
  if (autoCountdownAt) {
    led_set((hw_millis() / 100) & 1);
    if ((int32_t)(hw_millis() - autoCountdownAt) >= 0) {
      autoCountdownAt = 0;
      imu_calibrate(1000);                 /* car is still: best moment for the gyro */
      drive_zero(); headingDeg = 0;
      autoStartMs = hw_millis();
      if (!ap_start(autoStartMs)) { auto_cancel("AUTO: no path loaded"); return; }
      say("AUTO: GO");
    }
    return;
  }
  ApIn in = { hw_millis(), drv.x, drv.y, headingDeg, drv.distance, sonarCm };
  ApOut out;
  ap_update(&in, &out);
  if (out.brake) { if (drv.mode != DM_STOP) drive_stop(); }
  else           { drive_speed(out.speed); steer_set_deg(out.steerDeg); }
  if (ap.ev) auto_print_event();
  if (ap.state == AP_DONE) {
    carMode = CM_MANUAL; drive_stop();
    lb_clear(); lb_str("AUTO finished in "); lb_float((hw_millis() - autoStartMs) / 1000.0f, 1); lb_str(" s");
    lb_send(false);
  }
}

static void manual_takeover(void) { if (carMode == CM_AUTO) auto_cancel("AUTO aborted - manual command"); }

/* ---------------------------------------------------------- commands */
static void handle_command(char *line) {
  char *tok[4] = {0};
  int n = 0;
  for (char *p = strtok(line, " ,\t"); p && n < 4; p = strtok(NULL, " ,\t")) tok[n++] = p;
  if (n == 0) return;
  for (char *p = tok[0]; *p; ++p) *p = (char)toupper((unsigned char)*p);
  float a1 = (n > 1) ? strtof(tok[1], NULL) : 0;
  float a2 = (n > 2) ? strtof(tok[2], NULL) : 0;
  float a3 = (n > 3) ? strtof(tok[3], NULL) : 0;
  const char *c = tok[0];
  lastCommandMs = hw_millis();

  /* driving */
  if      (!strcmp(c, "X"))  { if (carMode == CM_AUTO) auto_cancel("AUTO stopped"); else { drive_stop(); say("OK stop"); } }
  else if (!strcmp(c, "P"))  { manual_takeover(); drive_open(a1); say("OK open-loop"); }
  else if (!strcmp(c, "V"))  { manual_takeover(); drive_speed(a1); say("OK speed"); }
  else if (!strcmp(c, "D"))  { manual_takeover(); drive_distance(a1, n > 2 ? a2 : 0.2f); say("OK distance move"); }
  else if (!strcmp(c, "S"))  { manual_takeover(); steer_set_deg(a1); }
  else if (!strcmp(c, "U"))  { manual_takeover(); steer_set_us((int)a1, true);
                               lb_clear(); lb_str("servo us="); lb_int(steerUs); lb_send(false); }
  /* teach & auto */
  else if (!strcmp(c, "REC"))   teach_start();
  else if (!strcmp(c, "WP"))    teach_mark(PF_STOP);
  else if (!strcmp(c, "END"))   teach_end();
  else if (!strcmp(c, "NOOBS")) { recFlags ^= PF_NOOBS;
                                  say((recFlags & PF_NOOBS) ? "obstacle detection OFF from here (NOOBS again to turn on)"
                                                            : "obstacle detection ON from here"); }
  else if (!strcmp(c, "GO"))    auto_arm();
  else if (!strcmp(c, "PATH"))  print_path_info();
  else if (!strcmp(c, "DUMP"))  { drive_stop(); dump_path(); }
  else if (!strcmp(c, "LOAD"))  { path_load(BUILTIN_PATH, BUILTIN_PATH_N); print_path_info(); }
  else if (!strcmp(c, "PC"))    { manual_takeover(); path_clear(); say("OK path cleared"); }
  else if (!strcmp(c, "PA"))    { if (!path_add(a1, a2, (uint8_t)a3)) say("ERR path full"); }
  else if (!strcmp(c, "AS"))    { AP_CRUISE = a1; print_settings(); }
  else if (!strcmp(c, "LA"))    { if (a1 > 0.1f) AP_LOOKAHEAD_M = a1; print_settings(); }
  else if (!strcmp(c, "OB"))    { AP_OBST_STOP_CM = a1; AP_OBST_CLEAR_CM = a1 + 20; print_settings(); }
  else if (!strcmp(c, "RR"))    { AP_REROUTE = a1 > 0.5f; print_settings(); }
  else if (!strcmp(c, "DO"))    { AP_DETOUR_OFFSET_M = a1; print_settings(); }
  else if (!strcmp(c, "WB"))    { if (a1 > 0.05f) WHEELBASE_M = a1; print_settings(); }
  /* setup / calibration */
  else if (!strcmp(c, "Z"))  { drive_zero(); headingDeg = 0; say("OK zeroed"); }
  else if (!strcmp(c, "GC")) { drive_stop(); say("gyro calibrating, keep still..."); imu_calibrate(1000); print_settings(); }
  else if (!strcmp(c, "GS")) { if (a1 > 0) GYRO_SCALE = a1; print_settings(); }
  else if (!strcmp(c, "T"))  { telemPeriodMs = (a1 > 0) ? (uint32_t)(1000.0f / a1) : 0; if (telemPeriodMs) telemetry_header(); }
  else if (!strcmp(c, "C"))  { if (a1 > 0) COUNTS_PER_METER = a1; print_settings(); }
  else if (!strcmp(c, "K"))  { SPEED_KP = a1; SPEED_KI = a2; if (n > 3) SPEED_FF = a3; print_settings(); }
  else if (!strcmp(c, "B"))  { MOTOR_DEADBAND_PCT = a1; print_settings(); }
  else if (!strcmp(c, "SC")) { SERVO_CENTER_US = (int)a1; steer_set_deg(0); print_settings(); }
  else if (!strcmp(c, "SL")) { SERVO_MIN_US = (int)a1; print_settings(); }
  else if (!strcmp(c, "SR")) { SERVO_MAX_US = (int)a1; print_settings(); }
  else if (!strcmp(c, "W"))  { watchdogMs = (uint32_t)a1; say("OK watchdog"); }
  else if (!strcmp(c, "G"))  print_settings();
  else if (!strcmp(c, "?") || !strcmp(c, "H")) print_help();
  else say("ERR unknown command, type ?");
}

/* ------------------------------------------------------------ button
 * AUTO running -> abort | moving -> stop | TEACH -> waypoint | else -> AUTO */
static bool     btnLast = false;
static uint32_t btnLastChangeMs = 0;

static void button_update(void) {
  bool pressed = button_pressed();
  if (pressed != btnLast && hw_millis() - btnLastChangeMs > 30) {
    btnLastChangeMs = hw_millis();
    btnLast = pressed;
    if (!pressed) return;
    if (carMode == CM_AUTO)       auto_cancel("BUTTON: AUTO aborted");
    else if (drv.mode != DM_STOP) { drive_stop(); say("BUTTON stop"); }
    else if (carMode == CM_TEACH) teach_mark(PF_STOP);
    else                          auto_arm();
  }
}

/* ------------------------------------------------------------- battery */
static void battery_warn(void) {
  if (battV > 5.0f && battV < BATT_LOW_V && hw_millis() - battWarnMs > 30000) {
    battWarnMs = hw_millis();
    lb_clear(); lb_str("WARNING battery low: "); lb_float(battV, 2); lb_str(" V - charge it");
    lb_send(false);
  }
}

/* ================================================================ main */
static uint32_t lastCtrlUs = 0;

void App_Init(void) {
  comms_init();
  hw_init();                     /* includes gyro calibration: car must be still */
  drive_zero();
  drive_stop();
  path_load(BUILTIN_PATH, BUILTIN_PATH_N);
  HAL_Delay(200);
  say("MR2006B Team 7 car - STM32CubeIDE firmware v1.3 ready. Type ? for commands.");
  if (!imuOk) say("WARNING: MPU-6050 not found - check SDA/SCL/VCC");
  print_settings();
  lastCtrlUs = hw_micros();
}

void App_Loop(void) {
  comms_poll(handle_command);

  /* fixed-rate control loop, 100 Hz */
  uint32_t nowUs = hw_micros();
  if (nowUs - lastCtrlUs >= 1000000u / CONTROL_HZ) {
    float dt = (nowUs - lastCtrlUs) * 1e-6f;
    if (dt > 0.05f) dt = 1.0f / CONTROL_HZ;      /* after a blocking call (gyro cal) */
    lastCtrlUs = nowUs;

    drive_update(dt);                           /* encoder -> speed, PI loop */
    float hBefore = headingDeg;
    imu_update(dt, drive_is_still());           /* gyro -> heading */
    odometry_update(hBefore, headingDeg);       /* -> x, y */
    teach_update();
    auto_update();

    if (watchdogMs && carMode != CM_AUTO && (drv.mode == DM_OPEN || drv.mode == DM_SPEED)
        && hw_millis() - lastCommandMs > watchdogMs) {
      drive_stop();
      say("WATCHDOG stop (no command received)");
    }
  }

  if (drv.moveDone) {
    drv.moveDone = false;
    lb_clear(); lb_str("DONE move, distance now "); lb_float(drv.distance, 3);
    lb_str(" m, counts "); lb_int(encoder_count());
    lb_send(false);
  }

  sonar_update();
  battery_update();
  battery_warn();
  button_update();
  if (!autoCountdownAt) led_set(drv.mode != DM_STOP || carMode == CM_TEACH);

  if (telemPeriodMs && hw_millis() - telemLastMs >= telemPeriodMs) {
    telemLastMs = hw_millis();
    telemetry_send();
  }
  comms_pump();
}
