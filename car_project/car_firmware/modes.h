// =====================================================================
//  modes.h  -  MANUAL / TEACH / AUTO modes, and the battery monitor
//
//  MANUAL  you drive with commands (V, S, P, D ...)
//  TEACH   you drive the course once; the car records its path, you mark
//          the waypoints (WP) and the finish (END). DUMP saves it.
//  AUTO    the car drives the recorded path by itself (GO or blue button)
// =====================================================================
#pragma once
#include "path.h"
#include "autopilot.h"
#include "path_data.h"

enum CarMode : uint8_t { CM_MANUAL = 0, CM_TEACH = 1, CM_AUTO = 2 };
static CarMode carMode = CM_MANUAL;

// ---------------------------------------------------------------------
//  Battery monitor (optional divider on PA4 [A2])
// ---------------------------------------------------------------------
static float    battV = 0;
static uint32_t battLastMs = 0, battWarnMs = 0;

static void batteryUpdate() {
  if (millis() - battLastMs < 200) return;
  battLastMs = millis();
  float v = analogRead(PIN_BATT) * 3.3f / 4095.0f * BATT_DIVIDER;
  battV = (battV < 0.1f) ? v : battV + 0.2f * (v - battV);
  // below 5 V = divider not connected -> stay quiet
  if (battV > 5.0f && battV < BATT_LOW_V && millis() - battWarnMs > 30000) {
    battWarnMs = millis();
    lb.clear(); lb.print("WARNING battery low: "); lb.print(battV, 2); lb.print(" V - charge it");
    sendLine(false);
  }
}

// ---------------------------------------------------------------------
//  Path helpers
// ---------------------------------------------------------------------
static void loadBuiltinPath() { pathLoad(BUILTIN_PATH, BUILTIN_PATH_N); }

static void printPathInfo() {
  lb.clear(); lb.print("PATH "); lb.print(path.n); lb.print(" points, ");
  lb.print(path.total, 2); lb.print(" m");
  sendLine(false);
  int k = 0;
  for (uint16_t i = 0; i < path.n; i++) {
    if (!(path.p[i].flags & (PF_STOP | PF_FINISH))) continue;
    lb.clear(); lb.print((path.p[i].flags & PF_FINISH) ? "  FINISH" : "  STOP ");
    if (!(path.p[i].flags & PF_FINISH)) lb.print(++k);
    lb.print(" at s="); lb.print(path.s[i], 2);
    lb.print(" m  x="); lb.print(path.p[i].x, 2); lb.print(" y="); lb.print(path.p[i].y, 2);
    sendLine(false);
  }
}

// Prints the path as a complete path_data.h file (car_console.py saves it)
static void dumpPath() {
  say("// ==== PATH BEGIN ====");
  say("// path_data.h - recorded with TEACH, saved with DUMP");
  say("// flags: 1 = STOP (waypoint), 2 = FINISH, 4 = obstacle detection off");
  say("#pragma once");
  say("const PathPt BUILTIN_PATH[] = {");
  for (uint16_t i = 0; i < path.n; i++) {
    lb.clear(); lb.print("  {"); lb.print(path.p[i].x, 3); lb.print("f, ");
    lb.print(path.p[i].y, 3); lb.print("f, "); lb.print(path.p[i].flags); lb.print("},");
    sendLine(false);
    commsPump();
  }
  say("};");
  say("const uint16_t BUILTIN_PATH_N = sizeof(BUILTIN_PATH) / sizeof(BUILTIN_PATH[0]);");
  say("// ==== PATH END ====");
}

// ---------------------------------------------------------------------
//  TEACH
// ---------------------------------------------------------------------
static uint8_t recFlags = 0;
static float   recX = 0, recY = 0;
static uint8_t recStops = 0;

static void teachStart() {
  apAbort();
  driveStop();
  steerSetDeg(0);
  say("TEACH: keep the car still at the START position...");
  imuCalibrate(1000);
  driveZero(); headingDeg = 0;
  pathClear(); pathAdd(0, 0, 0);
  recX = recY = 0; recFlags = 0; recStops = 0;
  carMode = CM_TEACH;
  say("TEACH recording. Drive with V / S. At each waypoint: stop exactly on it, send WP.");
  say("NOOBS toggles an obstacle-free zone. At the finish: stop on it, send END.");
}

// every control tick
static void teachUpdate() {
  if (carMode != CM_TEACH) return;
  if (hypotf(drv.x - recX, drv.y - recY) >= REC_SPACING_M) {
    if (!pathAdd(drv.x, drv.y, recFlags)) {
      driveStop(); carMode = CM_MANUAL;
      say("TEACH: path memory full - recording stopped");
      return;
    }
    recX = drv.x; recY = drv.y;
  }
}

static void teachMark(uint8_t flag) {
  if (carMode != CM_TEACH) { say("ERR not recording - send REC first"); return; }
  driveStop();
  pathAdd(drv.x, drv.y, flag | recFlags);
  recX = drv.x; recY = drv.y;
  lb.clear();
  if (flag & PF_STOP) { recStops++; lb.print("WAYPOINT "); lb.print(recStops); lb.print(" recorded"); }
  else lb.print("FINISH recorded");
  lb.print(" at x="); lb.print(drv.x, 2); lb.print(" y="); lb.print(drv.y, 2);
  lb.print(" (path "); lb.print(path.total, 1); lb.print(" m)");
  sendLine(false);
}

static void teachEnd() {
  teachMark(PF_FINISH);
  if (carMode != CM_TEACH) return;
  carMode = CM_MANUAL;
  printPathInfo();
  // If the finish IS the start, this is how far the odometry drifted in one lap
  lb.clear(); lb.print("Loop closure: odometry says finish is ");
  lb.print(hypotf(drv.x, drv.y) * 100, 0); lb.print(" cm from start (0 = perfect)");
  sendLine(false);
  say("TEACH done. Send DUMP to save it, or GO to replay it now.");
}

static void teachNoObs() {
  recFlags ^= PF_NOOBS;
  say((recFlags & PF_NOOBS) ? "obstacle detection OFF from here (NOOBS again to turn on)"
                            : "obstacle detection ON from here");
}

// ---------------------------------------------------------------------
//  AUTO
// ---------------------------------------------------------------------
static uint32_t autoCountdownAt = 0;   // start time after the countdown (0 = none)
static uint32_t autoStartMs = 0;

static void autoCancel(const char* why) {
  apAbort();
  autoCountdownAt = 0;
  carMode = CM_MANUAL;
  driveStop();
  say(why);
}

static void autoArm() {
  if (path.n < 2) { say("ERR no path - REC a path or load one"); return; }
  apAbort();
  driveStop();
  steerSetDeg(0);
  carMode = CM_AUTO;
  autoCountdownAt = millis() + 3000;
  say("AUTO: starting in 3 s - hands off, car must be still at the START");
}

static void autoPrintEvent() {
  lb.clear();
  lb.print("[AUTO "); lb.print((millis() - autoStartMs) / 1000.0f, 1); lb.print(" s] ");
  switch (ap.ev) {
    case EV_WAYPOINT:    lb.print("WAYPOINT "); lb.print((int)ap.evA); lb.print(" reached - waiting 10 s. Off-path ");
                         lb.print(ap.evB * 100, 1); lb.print(" cm"); break;
    case EV_FINISHED:    lb.print("FINISH reached. Off-path "); lb.print(ap.evB * 100, 1); lb.print(" cm"); break;
    case EV_LEAVING:     lb.print("driving to stop "); lb.print((int)ap.evA); break;
    case EV_OBSTACLE:    lb.print("OBSTACLE at "); lb.print(ap.evA, 0); lb.print(" cm - waiting"); break;
    case EV_CLEARED:     lb.print("obstacle gone - continuing"); break;
    case EV_REROUTE:     lb.print("obstacle did not move - rerouting"); break;
    case EV_DETOUR_DONE: lb.print("detour done, back on the path"); break;
    case EV_NO_PATH:     lb.print("no path loaded"); break;
    default: break;
  }
  sendLine(false);
  ap.ev = EV_NONE;
}

// every control tick
static void autoUpdate() {
  if (carMode != CM_AUTO) return;

  if (autoCountdownAt) {                       // counting down, car braked
    digitalWrite(PIN_LED, (millis() / 100) & 1);
    if ((int32_t)(millis() - autoCountdownAt) >= 0) {
      autoCountdownAt = 0;
      imuCalibrate(1000);                      // car is still: best moment for the gyro
      driveZero(); headingDeg = 0;
      autoStartMs = millis();
      if (!apStart(autoStartMs)) { autoCancel("AUTO: no path loaded"); return; }
      say("AUTO: GO");
    }
    return;
  }

  ApIn in{millis(), drv.x, drv.y, headingDeg, drv.distance, sonarCm};
  ApOut out;
  apUpdate(in, out);
  if (out.brake) { if (drv.mode != DM_STOP) driveStop(); }
  else           { driveSpeed(out.speed); steerSetDeg(out.steerDeg); }

  if (ap.ev) autoPrintEvent();
  if (ap.state == AP_DONE) {
    carMode = CM_MANUAL;
    driveStop();
    lb.clear(); lb.print("AUTO finished in "); lb.print((millis() - autoStartMs) / 1000.0f, 1); lb.print(" s");
    sendLine(false);
  }
}

// Any manual driving command takes over from AUTO
static void manualTakeover() {
  if (carMode == CM_AUTO) autoCancel("AUTO aborted - manual command");
}
