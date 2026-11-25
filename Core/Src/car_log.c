/* =====================================================================
 * car_log.c - test recorder, see car_log.h
 *
 * A test starts with log_begin() (car_app.c calls it when "d", "go" or
 * "vt" starts) and ends LOG_IDLE_MS after the main loop says the car is
 * no longer busy. While it runs, one sample is taken per control step
 * (TIM3, 10 Hz), and every line the car prints is kept as a note.
 *
 * "ld" sends a test as a CSV: the header line first (car_console.py starts
 * a new file in tools/logs/ on a line that begins with "t_ms,"), then the
 * notes as "# " lines, then the samples. It only sends to the port that
 * asked, a few lines at a time while that port has room, so the loop
 * never waits. Over Bluetooth a 20 s test takes about half a minute.
 * ===================================================================== */
#include "main.h"
#include "car_log.h"
#include "car_config.h"
#include "car_hw.h"
#include "car_comms.h"
#include "car_drive.h"
#include "car_heading.h"
#include "car_route.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

typedef struct {
  uint16_t t10;       /* 10 ms since the test started         */
  int8_t   state;     /* DRIVE_OFF .. DRIVE_SETTLE            */
  int8_t   step;      /* route step, 0 = no route             */
  int16_t  pct10;     /* motor % x 10                         */
  uint16_t servoUs;
  uint16_t sonarCm;
  int16_t  sp10;      /* speed setpoint, cm/s x 10            */
  int16_t  speed10;   /* encoder speed, cm/s x 10             */
  int16_t  herr10;    /* heading error, degrees x 10          */
  int16_t  yaw10;     /* heading since the test started, x 10 */
  uint16_t battMv;
  float    dist;      /* cm since the test started            */
} LogSample;

typedef struct {
  char     what[28];
  uint32_t startMs, endMs;
  uint16_t first, n;          /* samples                       */
  uint16_t noteFirst, noteLen;
  float    batt0, kh, khi, sc, sb;
} LogTest;

static LogSample smp[LOG_MAX_SAMPLES];
static uint16_t  nSmp = 0;
static LogTest   tests[LOG_MAX_TESTS];
static int       nTests = 0;
static char      notes[LOG_NOTES_BYTES];
static uint16_t  nNotes = 0;

static bool      rec = false;
static uint32_t  idleMs, lastStep;
static int32_t   cnt0;
static float     yaw0, battF;
static bool      fullSaid;

/* download state */
static int       dumpTest = -1, dumpLast = -1, dumpPort = 0;
static int       dumpPart;     /* 0 header, 1 title, 2 notes, 3 samples, 4 end */
static uint16_t  dumpIdx;

static int16_t i16(float v) { return (int16_t)(v > 32000.0f ? 32000.0f : (v < -32000.0f ? -32000.0f : v)); }

bool log_recording(void) { return rec; }
bool log_dumping(void)   { return dumpTest >= 0; }

static void end_test(const char *why) {
  if (!rec) return;
  rec = false;
  LogTest *t = &tests[nTests - 1];
  t->endMs = HAL_GetTick();
  t->n = (uint16_t)(nSmp - t->first);
  lb_clear(); lb_str("LOG test "); lb_int(nTests); lb_str(" saved: "); lb_uint(t->n); lb_str(" samples");
  if (why) { lb_str(" ("); lb_str(why); lb_str(")"); }
  lb_str(". ld = download");
  lb_send(false);
}

void log_begin(const char *what) {
  if (rec) end_test("next test started");
  if (nTests >= LOG_MAX_TESTS || nSmp >= LOG_MAX_SAMPLES - 20) {
    if (!fullSaid) say("LOG full, this test is NOT recorded: ld 0 to download, then lc");
    fullSaid = true;
    return;
  }
  LogTest *t = &tests[nTests++];
  memset(t, 0, sizeof *t);
  strncpy(t->what, what, sizeof t->what - 1);
  t->startMs = HAL_GetTick();
  t->first = nSmp;
  t->noteFirst = nNotes;
  battF = battery_volts();
  t->batt0 = battF;
  t->kh = STEER_KH; t->khi = STEER_KHI; t->sc = SERVO_CENTER_US; t->sb = STEER_PLAY_US;
  cnt0 = encoder_count();
  yaw0 = yawDeg;
  lastStep = driveSteps;
  idleMs = HAL_GetTick();
  rec = true;
  lb_clear(); lb_str("LOG test "); lb_int(nTests); lb_str(" recording: "); lb_str(what); lb_send(false);
}

void log_line(const char *s, uint16_t n) {
  if (!rec || nTests == 0) return;
  if (n > 0 && s[n - 1] == '\n') n--;
  if (n > 0 && s[n - 1] == '\r') n--;
  if (n == 0 || nNotes + n + 1 > LOG_NOTES_BYTES) return;
  if (n >= 4 && memcmp(s, "LOG ", 4) == 0) return;   /* our own messages */
  memcpy(&notes[nNotes], s, n);
  nNotes += n;
  notes[nNotes++] = '\n';
  tests[nTests - 1].noteLen = (uint16_t)(nNotes - tests[nTests - 1].noteFirst);
}

static void sample(void) {
  if (nSmp >= LOG_MAX_SAMPLES) { end_test("recorder full"); return; }
  LogSample *s = &smp[nSmp++];
  uint32_t t = (HAL_GetTick() - tests[nTests - 1].startMs) / 10;
  s->t10 = (uint16_t)(t > 65535 ? 65535 : t);
  s->state = (int8_t)state;
  s->step = (int8_t)route_step();
  s->pct10 = i16(motorPct * 10.0f);
  s->servoUs = (uint16_t)servoUs;
  s->sonarCm = (uint16_t)sonarCm;
  s->sp10 = i16(SP_Vel * 10.0f);
  s->speed10 = i16(Speed_CM_SEC * 10.0f);
  s->herr10 = i16(headingErrDeg * 10.0f);
  s->yaw10 = i16((yawDeg - yaw0) * 10.0f);
  battF += 0.3f * (battery_volts() - battF);   /* the PWM makes single readings jump */
  s->battMv = (uint16_t)(battF * 1000.0f);
  s->dist = (float)(encoder_count() - cnt0) * 100.0f / COUNTS_PER_METER;
}

/* ------------------------------------------------------------ download */
static void dump_start(int from, int to, int port) {
  dumpTest = from; dumpLast = to; dumpPort = port;
  dumpPart = 0; dumpIdx = 0;
}

static void dump_step(void) {
  while (dumpTest >= 0 && comms_tx_free(dumpPort) > 160) {
    const LogTest *t = &tests[dumpTest];
    lb_clear();
    if (dumpPart == 0) {
      lb_str("t_ms,state,step,motor_pct,servo_us,sonar_cm,sp_cms,speed_cms,dist_cm,head_err_deg,yaw_deg,batt_v");
      dumpPart = 1;
    } else if (dumpPart == 1) {
      lb_str("# TEST "); lb_int(dumpTest + 1); lb_str(": "); lb_str(t->what);
      lb_str(", "); lb_float((float)(t->endMs - t->startMs) / 1000.0f, 1); lb_str(" s, battery ");
      lb_float(t->batt0, 2); lb_str(" V, kh "); lb_float(t->kh, 1); lb_str(", khi "); lb_float(t->khi, 1);
      lb_str(", sc "); lb_float(t->sc, 0); lb_str(", sb "); lb_float(t->sb, 0);
      dumpPart = 2; dumpIdx = 0;
    } else if (dumpPart == 2) {
      if (dumpIdx >= t->noteLen) { dumpPart = 3; dumpIdx = 0; continue; }
      const char *p = &notes[t->noteFirst + dumpIdx];
      uint16_t n = 0;
      while (dumpIdx + n < t->noteLen && p[n] != '\n' && n < 120) n++;
      char line[124];
      memcpy(line, p, n); line[n] = 0;
      dumpIdx += n + 1;
      lb_str("# "); lb_str(line);
    } else if (dumpPart == 3) {
      if (dumpIdx >= t->n) { dumpPart = 4; continue; }
      const LogSample *s = &smp[t->first + dumpIdx++];
      lb_uint(s->t10 * 10u);              lb_str(",");
      lb_int(s->state);                   lb_str(",");
      lb_int(s->step);                    lb_str(",");
      lb_float(s->pct10 / 10.0f, 1);      lb_str(",");
      lb_uint(s->servoUs);                lb_str(",");
      lb_uint(s->sonarCm);                lb_str(",");
      lb_float(s->sp10 / 10.0f, 1);       lb_str(",");
      lb_float(s->speed10 / 10.0f, 1);    lb_str(",");
      lb_float(s->dist, 1);               lb_str(",");
      lb_float(s->herr10 / 10.0f, 1);     lb_str(",");
      lb_float(s->yaw10 / 10.0f, 1);      lb_str(",");
      lb_float(s->battMv / 1000.0f, 2);
    } else {
      lb_str("LOG test "); lb_int(dumpTest + 1); lb_str(" sent: "); lb_uint(t->n); lb_str(" samples");
      if (dumpTest < dumpLast) { dumpTest++; dumpPart = 0; }
      else dumpTest = -1;
    }
    lb_send_to(dumpPort);
  }
}

void log_poll(bool busy) {
  if (rec) {
    if (driveSteps != lastStep) { lastStep = driveSteps; sample(); }
    if (busy) idleMs = HAL_GetTick();
    else if (HAL_GetTick() - idleMs >= LOG_IDLE_MS) end_test(NULL);
  }
  if (dumpTest >= 0) dump_step();
}

/* ------------------------------------------------------------ commands */
void log_command(const char *line) {
  char c2 = (char)tolower((unsigned char)line[1]);
  const char *p = line;
  while (isalpha((unsigned char)*p)) p++;
  char *end;
  long n = strtol(p, &end, 10);
  bool hasN = (end != p);

  if (c2 == 'l') {                         /* ll: list */
    if (nTests == 0) { say("LOG empty: d, go and vs record a test"); return; }
    for (int i = 0; i < nTests; i++) {
      const LogTest *t = &tests[i];
      lb_clear(); lb_str("LOG "); lb_int(i + 1); lb_str(": "); lb_str(t->what); lb_str(", ");
      if (rec && i == nTests - 1) lb_str("recording");
      else { lb_float((float)(t->endMs - t->startMs) / 1000.0f, 1); lb_str(" s, "); lb_uint(t->n); lb_str(" samples"); }
      lb_send(false);
    }
    lb_clear(); lb_str("LOG used "); lb_uint(nSmp); lb_str(" of "); lb_uint(LOG_MAX_SAMPLES);
    lb_str(" samples, "); lb_int(nTests); lb_str(" of "); lb_int(LOG_MAX_TESTS); lb_str(" tests"); lb_send(false);
  } else if (c2 == 'd') {                  /* ld [n] */
    if (rec) { say("ERR the test is still recording: wait until the car stands"); return; }
    if (dumpTest >= 0) { say("ERR a download is still going"); return; }
    if (nTests == 0) { say("ERR nothing recorded"); return; }
    if (hasN && n == 0) dump_start(0, nTests - 1, comms_from);
    else {
      int i = hasN ? (int)n - 1 : nTests - 1;
      if (i < 0 || i >= nTests) { say("ERR no such test: ll lists them"); return; }
      dump_start(i, i, comms_from);
    }
  } else if (c2 == 'c') {                  /* lc: clear */
    if (rec || dumpTest >= 0) { say("ERR not while recording or downloading"); return; }
    nSmp = 0; nTests = 0; nNotes = 0; fullSaid = false;
    say("OK log cleared");
  } else {
    say("ERR log: ll (list), ld [n] (download, 0 = all), lc (clear)");
  }
}
