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

static void printSettings() {
  lb.clear(); lb.print("CPM=");    lb.print(COUNTS_PER_METER, 1);
  lb.print(" FF=");  lb.print(SPEED_FF, 1);  lb.print(" KP="); lb.print(SPEED_KP, 1);
  lb.print(" KI=");  lb.print(SPEED_KI, 1);  lb.print(" DB="); lb.print(MOTOR_DEADBAND_PCT, 1);
  sendLine(false);
  lb.clear(); lb.print("SERVO center="); lb.print(SERVO_CENTER_US);
  lb.print(" min="); lb.print(SERVO_MIN_US); lb.print(" max="); lb.print(SERVO_MAX_US);
  lb.print(" us/deg="); lb.print(SERVO_US_PER_DEG, 2);
  sendLine(false);
  lb.clear(); lb.print("IMU ");  lb.print(imuOk ? "OK" : "NOT FOUND");
  lb.print(" whoami=0x"); lb.print(imuWhoAmI, HEX);
  lb.print(" bias="); lb.print(gyroBiasDps, 3); lb.print(" dps scale="); lb.print(GYRO_SCALE, 4);
  sendLine(false);
  lb.clear(); lb.print("AUTO cruise="); lb.print(AP_CRUISE, 2); lb.print(" lookahead="); lb.print(AP_LOOKAHEAD_M, 2);
  lb.print(" wheelbase="); lb.print(WHEELBASE_M, 3); lb.print(" obst_stop="); lb.print(AP_OBST_STOP_CM, 0);
  lb.print("cm reroute="); lb.print(AP_REROUTE ? 1 : 0); lb.print(" detour="); lb.print(AP_DETOUR_OFFSET_M, 2);
  sendLine(false);
  lb.clear(); lb.print("BATT "); lb.print(battV, 2); lb.print(" V   mode=");
  lb.print(carMode == CM_TEACH ? "TEACH" : carMode == CM_AUTO ? "AUTO" : "MANUAL");
  lb.print("   path="); lb.print(path.n); lb.print(" pts / "); lb.print(path.total, 1); lb.print(" m");
  sendLine(false);
}

static void printHelp() {
  say("--- manual ---");
  say("X            stop + brake (also stops AUTO)");
  say("V <m/s>      closed-loop speed      S <deg>   steer, + = left");
  say("P <pct>      open-loop motor        D <m> [m/s]  drive distance");
  say("--- teach & auto ---");
  say("REC          start recording a path here (= START)");
  say("WP           mark a waypoint here   NOOBS  toggle obstacle-off zone");
  say("END          mark the finish here, stop recording");
  say("GO           run AUTO (blue button does the same)");
  say("PATH  DUMP   show / print the path   LOAD  reload built-in path");
  say("PC           clear path   PA <x> <y> [flags]  append point");
  say("AS <m/s>  LA <m>  OB <cm>  RR <0/1>  DO <m>   cruise, lookahead, obstacle stop, reroute, detour side/size");
  say("--- setup ---");
  say("Z  zero pose   GC  gyro cal   GS <k>  gyro scale   C <cpm>  counts/m");
  say("K <kp> <ki> [ff]   B <pct> deadband   U <us> raw servo");
  say("SC/SL/SR <us> servo center/min/max   WB <m> wheelbase");
  say("T <hz> telemetry   W <ms> watchdog   G settings");
}

// ---------------------------------------------------------------------
//  Command parser
// ---------------------------------------------------------------------
static uint32_t watchdogMs     = 0;    // 0 = off
static uint32_t lastCommandMs  = 0;

static void handleCommand(char* line) {
  char* tok[4] = {0};
  int n = 0;
  for (char* p = strtok(line, " ,\t"); p && n < 4; p = strtok(NULL, " ,\t")) tok[n++] = p;
  if (n == 0) return;
  for (char* p = tok[0]; *p; ++p) *p = toupper(*p);
  float a1 = (n > 1) ? strtof(tok[1], NULL) : 0;
  float a2 = (n > 2) ? strtof(tok[2], NULL) : 0;
  float a3 = (n > 3) ? strtof(tok[3], NULL) : 0;
  const char* c = tok[0];
  lastCommandMs = millis();

  // ---- driving ----
  if      (!strcmp(c, "X"))  { if (carMode == CM_AUTO) autoCancel("AUTO stopped"); else { driveStop(); say("OK stop"); } }
  else if (!strcmp(c, "P"))  { manualTakeover(); driveOpen(a1); say("OK open-loop"); }
  else if (!strcmp(c, "V"))  { manualTakeover(); driveSpeed(a1); say("OK speed"); }
  else if (!strcmp(c, "D"))  { manualTakeover(); driveDistance(a1, n > 2 ? a2 : 0.2f); say("OK distance move"); }
  else if (!strcmp(c, "S"))  { manualTakeover(); steerSetDeg(a1); }
  else if (!strcmp(c, "U"))  { manualTakeover(); steerSetUs((int)a1, true);
                               lb.clear(); lb.print("servo us="); lb.print(steerUs); sendLine(false); }
  // ---- teach & auto ----
  else if (!strcmp(c, "REC"))   { teachStart(); }
  else if (!strcmp(c, "WP"))    { teachMark(PF_STOP); }
  else if (!strcmp(c, "END"))   { teachEnd(); }
  else if (!strcmp(c, "NOOBS")) { teachNoObs(); }
  else if (!strcmp(c, "GO"))    { autoArm(); }
  else if (!strcmp(c, "PATH"))  { printPathInfo(); }
  else if (!strcmp(c, "DUMP"))  { driveStop(); dumpPath(); }
  else if (!strcmp(c, "LOAD"))  { loadBuiltinPath(); printPathInfo(); }
  else if (!strcmp(c, "PC"))    { manualTakeover(); pathClear(); say("OK path cleared"); }
  else if (!strcmp(c, "PA"))    { if (!pathAdd(a1, a2, (uint8_t)a3)) say("ERR path full"); }
  else if (!strcmp(c, "AS"))    { AP_CRUISE = a1; printSettings(); }
  else if (!strcmp(c, "LA"))    { if (a1 > 0.1f) AP_LOOKAHEAD_M = a1; printSettings(); }
  else if (!strcmp(c, "OB"))    { AP_OBST_STOP_CM = a1; AP_OBST_CLEAR_CM = a1 + 20; printSettings(); }
  else if (!strcmp(c, "RR"))    { AP_REROUTE = a1 > 0.5f; printSettings(); }
  else if (!strcmp(c, "DO"))    { AP_DETOUR_OFFSET_M = a1; printSettings(); }
  else if (!strcmp(c, "WB"))    { if (a1 > 0.05f) WHEELBASE_M = a1; printSettings(); }
  // ---- setup / calibration ----
  else if (!strcmp(c, "Z"))  { driveZero(); headingDeg = 0; say("OK zeroed"); }
  else if (!strcmp(c, "GC")) { driveStop(); say("gyro calibrating, keep still...");
                               imuCalibrate(1000); printSettings(); }
  else if (!strcmp(c, "GS")) { if (a1 > 0) GYRO_SCALE = a1; printSettings(); }
  else if (!strcmp(c, "T"))  { telemPeriodMs = (a1 > 0) ? (uint32_t)(1000.0f / a1) : 0;
                               if (telemPeriodMs) telemetryHeader(); }
  else if (!strcmp(c, "C"))  { if (a1 > 0) COUNTS_PER_METER = a1; printSettings(); }
  else if (!strcmp(c, "K"))  { SPEED_KP = a1; SPEED_KI = a2; if (n > 3) SPEED_FF = a3; printSettings(); }
  else if (!strcmp(c, "B"))  { MOTOR_DEADBAND_PCT = a1; printSettings(); }
  else if (!strcmp(c, "SC")) { SERVO_CENTER_US = (int)a1; steerSetDeg(0); printSettings(); }
  else if (!strcmp(c, "SL")) { SERVO_MIN_US = (int)a1; printSettings(); }
  else if (!strcmp(c, "SR")) { SERVO_MAX_US = (int)a1; printSettings(); }
  else if (!strcmp(c, "W"))  { watchdogMs = (uint32_t)a1; say("OK watchdog"); }
  else if (!strcmp(c, "G"))  { printSettings(); }
  else if (!strcmp(c, "?") || !strcmp(c, "H")) { printHelp(); }
  else say("ERR unknown command, type ?");
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
//    TEACH, standing     -> mark a waypoint here
//    MANUAL, standing    -> start AUTO (3 s countdown)
// ---------------------------------------------------------------------
static bool     btnLast = true;
static uint32_t btnLastChangeMs = 0;

static void buttonUpdate() {
  bool b = digitalRead(PIN_BUTTON) && digitalRead(PIN_BUTTON2);   // HIGH = both released
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
