/* =====================================================================
 * drive_sim.c - run the real car_drive.c against a simple model of the
 * car on a PC, to try gains and settings before going to the car.
 *
 * Build and run (from the repo root):
 *   gcc -O2 -Itools/drive_sim -ICore/Inc tools/drive_sim/drive_sim.c \
 *       Core/Src/car_drive.c Core/Src/car_config.c -lm -o tools/drive_sim/drive_sim
 *   tools/drive_sim/drive_sim d=1.0                  1 m run, prints a summary
 *   tools/drive_sim/drive_sim d=0.1 csv=run.csv      also write telemetry
 *   python tools/plot_run.py run.csv                 plot it
 *
 * Options (name=value):
 *   d=<m>  tt= ta= td=<s>  kp= ki= kd=     the run, as with the car commands
 *   start=60   %: the model car does not move from standstill below this
 *   off=45.1   %: once rolling, speed = (pct - off) / slope
 *   slope=0.59 % per cm/s
 *   tau=0.25   s: how fast the speed follows the motor
 *   crawl=0    cm/s: the model car cannot roll slower than this, it stops
 *   obst=<s>   put an obstacle in front from this time, for 2 s
 *
 * The model is a guess, not a measurement of our car: stiction, a first
 * order speed response, friction when coasting and a strong brake. Use it
 * to compare settings, not to predict centimetres.
 * ===================================================================== */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "car_drive.h"
#include "car_config.h"
#include "car_hw.h"

/* ---- the model car ---- */
static double x = 0, v = 0;            /* cm, cm/s */
static double mStart = 60, mOff = 6500.0 / 144.0, mSlope = 85.0 / 144.0, mTau = 0.25, mCrawl = 0;
static int    braking = 0;
static int32_t encZero = 0;

/* ---- car_hw.c, as far as car_drive.c uses it ---- */
float motorPct = 0;
volatile float sonarCm = SONAR_MAX_CM;
void    motor_set(float pct) { motorPct = pct < -100 ? -100 : (pct > 100 ? 100 : pct); braking = 0; }
void    motor_brake(void)    { motorPct = 0; braking = 1; }
static int32_t enc_raw(void) { return (int32_t)floor(x * COUNTS_PER_METER / 100.0); }
int32_t encoder_count(void)  { return enc_raw() - encZero; }
void    encoder_zero(void)   { encZero = enc_raw(); }
void    sonar_trigger(void)  {}

static void plant_step(double dt) {
  double a;
  double p = fabs(motorPct), dir = motorPct < 0 ? -1 : 1;
  if (braking)        a = (v == 0) ? 0 : -(v > 0 ? 1 : -1) * 150.0 - v / 0.1;
  else if (p < 1.0)   a = (v == 0) ? 0 : -(v > 0 ? 1 : -1) * 60.0 - v / 1.0;
  else if (v == 0 && p < mStart) a = 0;                       /* stiction */
  else {
    double vt = (p - mOff) / mSlope;
    if (vt < mCrawl) vt = 0;
    a = (dir * (vt > 0 ? vt : 0) - v) / mTau;
    if (vt <= 0 && v != 0) a -= (v > 0 ? 1 : -1) * 60.0;
  }
  double vn = v + a * dt;
  if (v != 0 && vn * v < 0) vn = 0;                           /* friction stops, it does not reverse */
  if (fabs(vn) < 0.05 && (braking || p < 1.0)) vn = 0;
  x += 0.5 * (v + vn) * dt;
  v = vn;
}

static double opt(int argc, char **argv, const char *name, double def) {
  size_t n = strlen(name);
  for (int i = 1; i < argc; i++)
    if (!strncmp(argv[i], name, n) && argv[i][n] == '=') return atof(argv[i] + n + 1);
  return def;
}
static const char *opts(int argc, char **argv, const char *name) {
  size_t n = strlen(name);
  for (int i = 1; i < argc; i++)
    if (!strncmp(argv[i], name, n) && argv[i][n] == '=') return argv[i] + n + 1;
  return NULL;
}

int main(int argc, char **argv) {
  static const char *const why[] = { "none", "DONE", "TIMEOUT", "SWINGS", "STALL", "ENC_DIR" };
  double d = opt(argc, argv, "d", 1.0);
  run_T  = (float)opt(argc, argv, "tt", 0);
  run_ta = (float)opt(argc, argv, "ta", DRIVE_RAMP_S);
  run_td = (float)opt(argc, argv, "td", DRIVE_RAMP_S);
  Kp = (float)opt(argc, argv, "kp", Kp);
  Ki = (float)opt(argc, argv, "ki", Ki);
  Kd = (float)opt(argc, argv, "kd", Kd);
  mStart = opt(argc, argv, "start", mStart);
  mOff   = opt(argc, argv, "off", mOff);
  mSlope = opt(argc, argv, "slope", mSlope);
  mTau   = opt(argc, argv, "tau", mTau);
  mCrawl = opt(argc, argv, "crawl", mCrawl);
  double obst = opt(argc, argv, "obst", -1);
  const char *csvName = opts(argc, argv, "csv");
  FILE *csv = csvName ? fopen(csvName, "w") : NULL;
  if (csv) fprintf(csv, "t_ms,state,sp_cms,speed_cms,dist_cm,counts,motor_pct,ref_cm,sonar_cm\n");

  if (!drive_start((float)d)) { printf("run does not fit\n"); return 1; }

  double target = d * 100, dir = d < 0 ? -1 : 1;
  double maxPast = 0, maxLag = 0, tMove = -1, tEnd = 0;
  int result = RUN_NONE, flips = 0, lastSign = 0;
  for (int step = 1; step <= 3000 && result == RUN_NONE; step++) {
    double t = step * DRIVE_PERIOD_MS / 1000.0;
    for (int i = 0; i < DRIVE_PERIOD_MS; i++) {       /* the car moves, 1 ms at a time */
      plant_step(0.001);
      double past = (x - target) * dir;
      if (past > maxPast) maxPast = past;
    }
    sonarCm = (obst >= 0 && t >= obst && t < obst + 2.0) ? 20.0f : SONAR_MAX_CM;
    drive_update();                                    /* the 100 ms control step */
    int sign = motorPct > 0 ? 1 : (motorPct < 0 ? -1 : 0);
    if (sign && lastSign && sign != lastSign) flips++;
    if (sign) lastSign = sign;
    if (tMove < 0 && fabs(v) > 0) tMove = t;
    if (state >= DRIVE_ACCEL && state <= DRIVE_DECEL && fabs(Pos_Ref - Real_Dist) > maxLag)
      maxLag = fabs(Pos_Ref - Real_Dist);
    if (csv) fprintf(csv, "%d,%d,%.2f,%.2f,%.1f,%d,%.1f,%.1f,%.0f\n", step * DRIVE_PERIOD_MS, state,
                     SP_Vel, AverageSpeed, Real_Dist, (int)encoder_count(), motorPct, Pos_Ref, sonarCm);
    result = drive_result();
    tEnd = t;
  }
  for (int i = 0; i < 2000; i++) plant_step(0.001);    /* let it roll out */
  if (csv) fclose(csv);
  printf("%-7s end %7.2f cm  err %+5.2f cm  past target %4.2f cm  max lag %5.2f cm  "
         "moved after %.1f s  took %5.1f s (profile %.1f)  direction changes %d\n",
         why[result], x, x - target, maxPast, maxLag, tMove, tEnd, T, flips);
  return 0;
}
