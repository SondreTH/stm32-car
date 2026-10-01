// =====================================================================
//  commands.h  -  telemetry, the command parser, the blue button
// =====================================================================
#pragma once

// ---------------------------------------------------------------------
//  Telemetry (CSV). Column names are used by tools/plot_run.py
// ---------------------------------------------------------------------
static uint32_t telemPeriodMs = 0;     // 0 = off
static uint32_t telemLastMs   = 0;

static void telemetryHeader() {
  say("t_ms,drive,car,ap,counts,dist_m,v_mps,set_mps,motor_pct,steer_deg,sonar_cm,heading_deg,x_m,y_m,s_m,batt_v");
}

static void telemetrySend() {
  lb.clear();
  lb.print(millis());          lb.print(',');
  lb.print((int)drv.mode);     lb.print(',');
  lb.print((int)carMode);      lb.print(',');
  lb.print((int)ap.state);     lb.print(',');
  lb.print(encoderCount());    lb.print(',');
  lb.print(drv.distance, 3);   lb.print(',');
  lb.print(drv.speed, 3);      lb.print(',');
  lb.print(drv.setpoint, 3);   lb.print(',');
  lb.print(motorPct, 1);       lb.print(',');
  lb.print(steerDeg, 1);       lb.print(',');
  lb.print(sonarCm, 0);        lb.print(',');
  lb.print(headingDeg, 1);     lb.print(',');
  lb.print(drv.x, 3);          lb.print(',');
  lb.print(drv.y, 3);          lb.print(',');
  lb.print(fol.sNow, 2);       lb.print(',');
  lb.print(battV, 2);
  sendLine(true);
}


// ---------------------------------------------------------------------
//  Tunable settings: SET <name> <value>, listed by SETTINGS
// ---------------------------------------------------------------------
static uint32_t watchdogMs     = 0;    // 0 = off
static uint32_t lastCommandMs  = 0;

enum ParamType : uint8_t { PT_FLOAT, PT_INT, PT_BOOL, PT_U32 };

struct Param {
  const char* name;
  ParamType   type;
  void*       ptr;
  float       lo, hi;      // allowed range
  const char* unit;
};

static const Param PARAMS[] = {
  // speed control
  {"cpm",              PT_FLOAT, &COUNTS_PER_METER,   1,     1e6f,   "counts/m"},
  {"kp",               PT_FLOAT, &SPEED_KP,           0,     1000,   "%/(m/s)"},
  {"ki",               PT_FLOAT, &SPEED_KI,           0,     5000,   "%/(m/s*s)"},
  {"ff",               PT_FLOAT, &SPEED_FF,           0,     2000,   "%/(m/s)"},
  {"deadband",         PT_FLOAT, &MOTOR_DEADBAND_PCT, 0,     100,    "%"},
  {"accel",            PT_FLOAT, &MAX_ACCEL,          0.05f, 5,      "m/s^2"},
  // steering
  {"servo_center",     PT_INT,   &SERVO_CENTER_US,    900,   2100,   "us"},
  {"servo_min",        PT_INT,   &SERVO_MIN_US,       900,   2100,   "us"},
  {"servo_max",        PT_INT,   &SERVO_MAX_US,       900,   2100,   "us"},
  {"servo_us_per_deg", PT_FLOAT, &SERVO_US_PER_DEG,   1,     50,     "us/deg"},
  {"steer_max",        PT_FLOAT, &STEER_MAX_DEG,      5,     45,     "deg"},
  // gyro
  {"gyro_scale",       PT_FLOAT, &GYRO_SCALE,         0.5f,  1.5f,   ""},
  // autopilot
  {"wheelbase",        PT_FLOAT, &WHEELBASE_M,        0.05f, 1,      "m"},
  {"cruise",           PT_FLOAT, &AP_CRUISE,          0.02f, 2,      "m/s"},
  {"lookahead",        PT_FLOAT, &AP_LOOKAHEAD_M,     0.1f,  3,      "m"},
  {"waypoint_wait",    PT_U32,   &AP_WAIT_MS,         0,     600000, "ms"},
  {"obstacle_stop",    PT_FLOAT, &AP_OBST_STOP_CM,    5,     300,    "cm"},
  {"obstacle_clear",   PT_FLOAT, &AP_OBST_CLEAR_CM,   5,     400,    "cm"},
  {"reroute",          PT_BOOL,  &AP_REROUTE,         0,     1,      ""},
  {"detour",           PT_FLOAT, &AP_DETOUR_OFFSET_M, -2,    2,      "m (+ = pass left)"},
  // safety
  {"watchdog",         PT_U32,   &watchdogMs,         0,     60000,  "ms (0 = off)"},
};
static const uint8_t PARAM_N = sizeof(PARAMS) / sizeof(PARAMS[0]);

static const Param* findParam(const char* name) {
  for (uint8_t i = 0; i < PARAM_N; i++)
    if (!strcasecmp(name, PARAMS[i].name)) return &PARAMS[i];
  return nullptr;
}

static void printParam(const Param& p) {
  lb.clear(); lb.print(p.name); lb.print(" = ");
  switch (p.type) {
    case PT_FLOAT: lb.print(*(float*)p.ptr, 3); break;
    case PT_INT:   lb.print(*(int*)p.ptr); break;
    case PT_BOOL:  lb.print(*(bool*)p.ptr ? "on" : "off"); break;
    case PT_U32:   lb.print(*(uint32_t*)p.ptr); break;
  }
  if (*p.unit) { lb.print(' '); lb.print(p.unit); }
  sendLine(false);
}

static void printSettings() {
  say("--- settings (change with SET <name> <value>) ---");
  for (uint8_t i = 0; i < PARAM_N; i++) { printParam(PARAMS[i]); commsPump(); }
}

static void printStatus() {
  lb.clear(); lb.print("MODE ");
  lb.print(carMode == CM_TEACH ? "RECORD" : carMode == CM_AUTO ? "AUTO" : "MANUAL");
  lb.print("   PATH "); lb.print(path.n); lb.print(" pts / "); lb.print(path.total, 2); lb.print(" m");
  lb.print("   BATT "); lb.print(battV, 2); lb.print(" V");
  sendLine(false);
  lb.clear(); lb.print("ENCODER "); lb.print(encoderCount()); lb.print(" counts = ");
  lb.print(drv.distance, 3); lb.print(" m   speed "); lb.print(drv.speed, 3); lb.print(" m/s");
  sendLine(false);
  lb.clear(); lb.print("POSE x="); lb.print(drv.x, 3); lb.print(" y="); lb.print(drv.y, 3);
  lb.print(" m   heading "); lb.print(headingDeg, 1); lb.print(" deg");
  lb.print("   STEER "); lb.print(steerDeg, 1); lb.print(" deg ("); lb.print(steerUs); lb.print(" us)");
  sendLine(false);
  lb.clear(); lb.print("SONAR "); lb.print(sonarCm, 0); lb.print(" cm   IMU ");
  lb.print(imuOk ? "OK" : "NOT FOUND"); lb.print(" whoami=0x"); lb.print(imuWhoAmI, HEX);
  lb.print(" bias="); lb.print(gyroBiasDps, 3); lb.print(" dps");
  sendLine(false);
}

static void printHelp() {
  say("--- driving ---");
  say("STOP (or X)          brake now, also aborts AUTO");
  say("SPEED <m/s>          closed-loop speed, negative = reverse");
  say("POWER <pct>          open-loop motor power, -100..100");
  say("MOVE <m> [m/s]       drive a distance, then brake");
  say("STEER <deg>          front wheels, + = left");
  say("SERVO <us>           raw servo pulse (calibration)");
  say("--- route ---");
  say("RECORD               start recording a route here (= START)");
  say("WAYPOINT             mark a waypoint here");
  say("FINISH               mark the finish here, stop recording");
  say("OBSTACLES ON|OFF     obstacle detection for the route recorded from here");
  say("AUTO                 drive the route by itself (= blue button)");
  say("PATH                 route summary");
  say("PATH DUMP | LOAD | CLEAR | ADD <x> <y> [flags]");
  say("--- setup ---");
  say("STATUS               encoder, pose, sonar, IMU, battery");
  say("ZERO                 reset distance, position and heading");
  say("CALIBRATE GYRO       measure gyro bias (keep the car still)");
  say("SETTINGS             list all settings");
  say("SET <name> <value>   change a setting, e.g. SET cruise 0.3");
  say("TELEMETRY <hz>|OFF   CSV stream");
  say("HELP (or ?)          this list");
}

// ---------------------------------------------------------------------
//  Command parser
// ---------------------------------------------------------------------
static void err(const char* msg) { lb.clear(); lb.print("ERR "); lb.print(msg); sendLine(false); }

// strict number: the whole token must be a number
static bool parseNum(const char* s, float& out) {
  if (!s) return false;
  char* end;
  out = strtof(s, &end);
  return end != s && *end == 0;
}

static bool parseOnOff(const char* s, bool& out) {
  if (!s) return false;
  if (!strcasecmp(s, "on")  || !strcmp(s, "1") || !strcasecmp(s, "true"))  { out = true;  return true; }
  if (!strcasecmp(s, "off") || !strcmp(s, "0") || !strcasecmp(s, "false")) { out = false; return true; }
  return false;
}

static void cmdSet(const char* name, const char* value) {
  if (!name) { printSettings(); return; }
  const Param* p = findParam(name);
  if (!p) { err("unknown setting - type SETTINGS for the list"); return; }
  if (!value) { printParam(*p); return; }

  float v;
  if (p->type == PT_BOOL) {
    bool b;
    if (!parseOnOff(value, b)) { err("expected on or off"); return; }
    *(bool*)p->ptr = b;
  } else {
    if (!parseNum(value, v)) { err("expected a number"); return; }
    if (v < p->lo || v > p->hi) {
      lb.clear(); lb.print("ERR "); lb.print(p->name); lb.print(" must be ");
      lb.print(p->lo, 3); lb.print(" .. "); lb.print(p->hi, 3); sendLine(false);
      return;
    }
    switch (p->type) {
      case PT_FLOAT: *(float*)p->ptr    = v; break;
      case PT_INT:   *(int*)p->ptr      = (int)lroundf(v); break;
      case PT_U32:   *(uint32_t*)p->ptr = (uint32_t)lroundf(v); break;
      default: break;
    }
  }
  printParam(*p);

  // keep dependent values consistent
  if (AP_OBST_CLEAR_CM < AP_OBST_STOP_CM + 10) {
    AP_OBST_CLEAR_CM = AP_OBST_STOP_CM + 20;
    printParam(*findParam("obstacle_clear"));
  }
  if (!strncasecmp(p->name, "servo_", 6) || !strcasecmp(p->name, "steer_max"))
    steerSetDeg(steerDeg);                    // apply the new servo limits now
}

static void cmdPath(const char* sub, const char* a, const char* b, const char* c) {
  if (!sub) { printPathInfo(); return; }
  if (!strcasecmp(sub, "DUMP"))  { driveStop(); dumpPath(); return; }
  if (!strcasecmp(sub, "LOAD"))  { manualTakeover(); loadBuiltinPath(); printPathInfo(); return; }
  if (!strcasecmp(sub, "CLEAR")) { manualTakeover(); pathClear(); say("OK path cleared"); return; }
  if (!strcasecmp(sub, "ADD")) {
    float x, y, f = 0;
    if (!parseNum(a, x) || !parseNum(b, y) || (c && !parseNum(c, f))) { err("usage: PATH ADD <x> <y> [flags]"); return; }
    manualTakeover();
    if (!pathAdd(x, y, (uint8_t)f)) err("path full");
    return;
  }
  err("usage: PATH [DUMP | LOAD | CLEAR | ADD <x> <y> [flags]]");
}

static void handleCommand(char* line) {
  char* tok[5] = {0};
  int n = 0;
  for (char* p = strtok(line, " ,\t"); p && n < 5; p = strtok(NULL, " ,\t")) tok[n++] = p;
  if (n == 0) return;
  const char* c = tok[0];
  float a, b;
  lastCommandMs = millis();

  // ---- driving ----
  if (!strcasecmp(c, "STOP") || !strcasecmp(c, "X")) {
    if (carMode == CM_AUTO) autoCancel("AUTO stopped");
    else { driveStop(); say("OK stop"); }
  }
  else if (!strcasecmp(c, "SPEED")) {
    if (!parseNum(tok[1], a)) { err("usage: SPEED <m/s>"); return; }
    manualTakeover(); driveSpeed(a); say("OK speed");
  }
  else if (!strcasecmp(c, "POWER")) {
    if (!parseNum(tok[1], a)) { err("usage: POWER <pct>"); return; }
    manualTakeover(); driveOpen(a); say("OK power");
  }
  else if (!strcasecmp(c, "MOVE")) {
    b = 0.2f;
    if (!parseNum(tok[1], a) || (tok[2] && (!parseNum(tok[2], b) || b <= 0))) { err("usage: MOVE <m> [m/s]"); return; }
    manualTakeover(); driveDistance(a, b); say("OK move");
  }
  else if (!strcasecmp(c, "STEER")) {
    if (!parseNum(tok[1], a)) { err("usage: STEER <deg>"); return; }
    manualTakeover(); steerSetDeg(a);
  }
  else if (!strcasecmp(c, "SERVO")) {
    if (!parseNum(tok[1], a)) { err("usage: SERVO <us>"); return; }
    manualTakeover(); steerSetUs((int)a, true);
    lb.clear(); lb.print("servo "); lb.print(steerUs); lb.print(" us"); sendLine(false);
  }
  // ---- route ----
  else if (!strcasecmp(c, "RECORD"))   { teachStart(); }
  else if (!strcasecmp(c, "WAYPOINT")) { teachMark(PF_STOP); }
  else if (!strcasecmp(c, "FINISH"))   { teachEnd(); }
  else if (!strcasecmp(c, "OBSTACLES")) {
    bool on;
    if (!parseOnOff(tok[1], on)) { err("usage: OBSTACLES ON|OFF"); return; }
    teachSetObstacles(on);
  }
  else if (!strcasecmp(c, "AUTO"))     { autoArm(); }
  else if (!strcasecmp(c, "PATH"))     { cmdPath(tok[1], tok[2], tok[3], tok[4]); }
  // ---- setup ----
  else if (!strcasecmp(c, "STATUS"))   { printStatus(); }
  else if (!strcasecmp(c, "ZERO"))     { driveZero(); headingDeg = 0; say("OK zeroed"); }
  else if (!strcasecmp(c, "CALIBRATE")) {
    if (!tok[1] || strcasecmp(tok[1], "GYRO")) { err("usage: CALIBRATE GYRO"); return; }
    if (carMode == CM_AUTO) { err("not while AUTO is running"); return; }
    driveStop(); say("gyro calibrating, keep the car still...");
    imuCalibrate(1000);
    lb.clear(); lb.print("OK gyro bias "); lb.print(gyroBiasDps, 3); lb.print(" dps"); sendLine(false);
  }
  else if (!strcasecmp(c, "SETTINGS")) { printSettings(); }
  else if (!strcasecmp(c, "SET"))      { cmdSet(tok[1], tok[2]); }
  else if (!strcasecmp(c, "TELEMETRY")) {
    bool on;
    if (tok[1] && parseOnOff(tok[1], on) && !on) { telemPeriodMs = 0; say("OK telemetry off"); return; }
    if (!parseNum(tok[1], a) || a <= 0 || a > CONTROL_HZ) { err("usage: TELEMETRY <hz 1..100> | OFF"); return; }
    telemPeriodMs = (uint32_t)(1000.0f / a);
    telemetryHeader();
  }
  else if (!strcasecmp(c, "HELP") || !strcmp(c, "?")) { printHelp(); }
  else err("unknown command, type HELP");
}

// Line reader for one serial port
struct LineReader {
  char buf[64]; uint8_t n = 0;
  void poll(Stream& s) {
    while (s.available()) {
      char ch = (char)s.read();
      if (ch == '\n' || ch == '\r') {
        if (n) { buf[n] = 0; handleCommand(buf); n = 0; }
      } else if (n < sizeof(buf) - 1) buf[n++] = ch;
    }
  }
};
static LineReader usbIn, btIn;

// ---------------------------------------------------------------------
//  Blue button
//    AUTO running        -> abort
//    car moving          -> stop
//    RECORD, standing    -> mark a waypoint here
//    MANUAL, standing    -> start AUTO (3 s countdown)
// ---------------------------------------------------------------------
static bool     btnLast = true;
static uint32_t btnLastChangeMs = 0;

static void buttonUpdate() {
  bool b = digitalRead(PIN_BUTTON);            // HIGH = released
  if (b != btnLast && millis() - btnLastChangeMs > 30) {
    btnLastChangeMs = millis();
    btnLast = b;
    if (b) return;                             // act on press only
    if (carMode == CM_AUTO)          autoCancel("BUTTON: AUTO aborted");
    else if (drv.mode != DM_STOP)    { driveStop(); say("BUTTON stop"); }
    else if (carMode == CM_TEACH)    teachMark(PF_STOP);
    else                             autoArm();
  }
}
