/* =====================================================================
 * route_sim.c - run the real car_route.c and car_drive.c on a PC against
 * a model car on a flat floor, with a box in the way, to check the route
 * logic (going around an obstacle) before trying it on the car.
 *
 * Build and run (from the repo root):
 *   gcc -O2 -Wall -Wextra -Itools/route_sim -ICore/Inc tools/route_sim/route_sim.c \
 *       Core/Src/car_route.c Core/Src/car_drive.c Core/Src/car_config.c -lm \
 *       -o tools/route_sim/route_sim
 *   tools/route_sim/route_sim                    10 m straight, box at 5 m
 *   tools/route_sim/route_sim then=1 csv=p.csv   + a turn and 3 m, write the path
 *
 * Options (name=value):
 *   len=10     m, the route straight          then=0/1  add "t -90, s 3" after it
 *   at=5       m from the start to the front of the box
 *   bw=0.5 bd=0.5   box width and depth (m)   bx=0  box centre, m to the right
 *   gone=-1    s: the box is taken away at this time (-1 = never)
 *   raw= ras= raa= rao= rap= rab= rav=  the route's detour settings
 *   so=        obstacle look-ahead (s)        scale=1  gyro reads scale x the turn
 *
 * The model: speed from car_drive.c's motor % as in drive_sim (no slip),
 * curvature from the servo pulse (73 cm radius at 250 us, measured on the
 * car), the heading hold as in car_heading.c, a sonar with three rays
 * (-8, 0, +8 degrees) from the front of the car. The car is a 30 x 18 cm
 * box; touching the obstacle ends the run with "HIT". It checks the logic
 * (sides, signs, distances), not real centimetres.
 * ===================================================================== */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include "main.h"
#include "car_drive.h"
#include "car_config.h"
#include "car_hw.h"
#include "car_heading.h"
#include "car_comms.h"
#include "car_route.h"

/* ---- time ---- */
static uint32_t nowMs = 0;
uint32_t HAL_GetTick(void) { return nowMs; }

/* ---- car_hw.c, as far as the route and the drive use it ---- */
float motorPct = 0;
uint32_t servoUs = SERVO_PULSE_CENTER;
volatile float sonarCm = SONAR_MAX_CM;
static int braking = 0;
static double sPath = 0, v = 0;         /* distance along the path (cm), speed (cm/s) */
static int32_t encZero = 0;
void    motor_set(float pct) { motorPct = pct < -100 ? -100 : (pct > 100 ? 100 : pct); braking = 0; }
void    motor_brake(void)    { motorPct = 0; braking = 1; }
static int32_t enc_raw(void) { return (int32_t)floor(sPath * COUNTS_PER_METER / 100.0); }
int32_t encoder_count(void)  { return enc_raw() - encZero; }
void    encoder_zero(void)   { encZero = enc_raw(); }
void    sonar_trigger(void)  {}
void    servo_set_us(uint32_t us) { servoUs = us; }

/* ---- car_heading.c: the gyro and the heading hold, as in the firmware ---- */
bool  imuOk = true;
volatile float yawDeg = 0, yawRateDps = 0, headingErrDeg = 0;
float gyroBiasDps = 0;
static bool  useTarget = false, holdOn = true, wasRunning = false;
static float targetYaw = 0, yawAtStart = 0;
void heading_set_target(float deg) { targetYaw = deg; useTarget = true; }
void heading_clear_target(void)    { useTarget = false; }
void heading_set_hold(bool on)     { holdOn = on; }
static void heading_hold(void) {
  bool running = (state != DRIVE_OFF);
  if (running && !wasRunning) yawAtStart = yawDeg;
  float goal = useTarget ? targetYaw : yawAtStart;
  if (running && holdOn && STEER_KH != 0.0f) {
    headingErrDeg = yawDeg - goal;
    float dir = (target_dist_cm < 0.0f) ? -1.0f : 1.0f;
    float us = SERVO_CENTER_US + dir * STEER_KH * headingErrDeg;
    if (us < SERVO_PULSE_MIN) us = SERVO_PULSE_MIN;
    if (us > SERVO_PULSE_MAX) us = SERVO_PULSE_MAX;
    servo_set_us((uint32_t)us);
  }
  if (!running && wasRunning && holdOn) servo_set_us((uint32_t)SERVO_CENTER_US);
  wasRunning = running;
}

/* ---- car_comms.c: print what the car would send ---- */
static char lbBuf[256];
static int  lbLen = 0;
static int  routeOver = 0;
void lb_clear(void) { lbLen = 0; lbBuf[0] = 0; }
static void lb_add(const char *fmt, ...) {
  va_list ap; va_start(ap, fmt);
  lbLen += vsnprintf(lbBuf + lbLen, sizeof lbBuf - (size_t)lbLen, fmt, ap);
  va_end(ap);
  if (lbLen > (int)sizeof lbBuf - 1) lbLen = (int)sizeof lbBuf - 1;
}
void lb_str(const char *s)        { lb_add("%s", s); }
void lb_int(int32_t x)            { lb_add("%d", (int)x); }
void lb_uint(uint32_t x)          { lb_add("%u", (unsigned)x); }
void lb_float(float x, int dec)   { lb_add("%.*f", dec, (double)x); }
void lb_hex(uint32_t x)           { lb_add("%X", (unsigned)x); }
void lb_send(bool dropIfBusy) {
  (void)dropIfBusy;
  printf("%7.1f s  %s\n", nowMs / 1000.0, lbBuf);
  if (strstr(lbBuf, "ROUTE DONE") || strstr(lbBuf, "ERR")) routeOver = 1;
  lb_clear();
}
void say(const char *s) { lb_clear(); lb_str(s); lb_send(false); }

/* ---- the model car and the floor ---- */
static double px = 0, py = 0, head = 0;   /* m, m, deg (+ = left of the start direction) */
static double boxX0, boxX1, boxY0, boxY1;
static int boxThere = 1;

static void plant_step(double dt) {     /* speed: the drive_sim model, defaults */
  const double mStart = 33, mOff = 22, mSlope = 0.5, mTau = 0.8, mCoast = 90;
  double a, p = fabs(motorPct), dir = motorPct < 0 ? -1 : 1, sgn = v > 0 ? 1 : -1;
  if (braking)                 a = (v == 0) ? 0 : -sgn * 150.0 - v / 0.1;
  else if (fabs(motorPct) < 1) a = (v == 0) ? 0 : -sgn * mCoast - v / 2.0;
  else if (v == 0 && p < mStart) a = 0;
  else {
    double vt = (p - mOff) / mSlope;
    a = (dir * (vt > 0 ? vt : 0) - v) / mTau;
    if (vt <= 0 && v != 0) a -= sgn * mCoast;
  }
  double vn = v + a * dt;
  if (v != 0 && vn * v < 0) vn = 0;
  if (fabs(vn) < 0.05 && (braking || fabs(motorPct) < 1.0 || p < mOff)) vn = 0;
  double ds = 0.5 * (v + vn) * dt / 100.0;     /* m */
  v = vn;
  sPath += ds * 100.0;
  /* curvature from the servo: + pulse = right, 73 cm radius at 250 us */
  double k = ((double)servoUs - SERVO_PULSE_CENTER) / 250.0 / 0.73;
  double h0 = head * M_PI / 180.0;
  px += -sin(h0) * ds;
  py +=  cos(h0) * ds;
  head -= ds * k * 180.0 / M_PI;
}

/* distance from (x,y) along direction h (deg) to the box, or -1 */
static double ray_box(double x, double y, double h) {
  double dx = -sin(h * M_PI / 180.0), dy = cos(h * M_PI / 180.0);
  double t0 = 0, t1 = 1e9;
  double lo[2] = { boxX0, boxY0 }, hi[2] = { boxX1, boxY1 }, o[2] = { x, y }, d[2] = { dx, dy };
  for (int i = 0; i < 2; i++) {
    if (fabs(d[i]) < 1e-9) { if (o[i] < lo[i] || o[i] > hi[i]) return -1; continue; }
    double a = (lo[i] - o[i]) / d[i], b = (hi[i] - o[i]) / d[i];
    if (a > b) { double t = a; a = b; b = t; }
    if (a > t0) t0 = a;
    if (b < t1) t1 = b;
    if (t0 > t1) return -1;
  }
  return t0;
}

/* the car's corners (30 x 18 cm around px, py) inside the box? and the gap */
static double car_gap(int *hit) {
  double h = head * M_PI / 180.0, fx = -sin(h), fy = cos(h), rx = cos(h), ry = sin(h);
  double gap = 1e9;
  *hit = 0;
  for (int i = 0; i < 4; i++) {
    double a = (i < 2 ? 0.15 : -0.15), b = (i % 2 ? 0.09 : -0.09);
    double cx = px + fx * a + rx * b, cy = py + fy * a + ry * b;
    double gx = cx < boxX0 ? boxX0 - cx : (cx > boxX1 ? cx - boxX1 : 0);
    double gy = cy < boxY0 ? boxY0 - cy : (cy > boxY1 ? cy - boxY1 : 0);
    double g = sqrt(gx * gx + gy * gy);
    if (g == 0) *hit = 1;
    if (g < gap) gap = g;
  }
  return gap;
}

static double opt(int argc, char **argv, const char *name, double def) {
  size_t n = strlen(name);
  for (int i = 1; i < argc; i++)
    if (!strncmp(argv[i], name, n) && argv[i][n] == '=') return atof(argv[i] + n + 1);
  return def;
}

int main(int argc, char **argv) {
  double len = opt(argc, argv, "len", 10), at = opt(argc, argv, "at", 5);
  double bw = opt(argc, argv, "bw", 0.5), bd = opt(argc, argv, "bd", 0.5), bx = opt(argc, argv, "bx", 0);
  double gone = opt(argc, argv, "gone", -1), scale = opt(argc, argv, "scale", 1);
  int then = (int)opt(argc, argv, "then", 0);
  routeAvoidWaitS = (float)opt(argc, argv, "raw", routeAvoidWaitS);
  routeAvoidSide  = (float)opt(argc, argv, "ras", routeAvoidSide);
  routeAvoidDeg   = (float)opt(argc, argv, "raa", routeAvoidDeg);
  routeAvoidOutM  = (float)opt(argc, argv, "rao", routeAvoidOutM);
  routeAvoidPassM = (float)opt(argc, argv, "rap", routeAvoidPassM);
  routeAvoidBackM = (float)opt(argc, argv, "rab", routeAvoidBackM);
  routeAvoidSpeed = (float)opt(argc, argv, "rav", routeAvoidSpeed);
  OBST_LOOK_S     = (float)opt(argc, argv, "so", OBST_LOOK_S);
  routeTurnUs = 250.0f;
  boxX0 = bx - bw / 2; boxX1 = bx + bw / 2; boxY0 = at; boxY1 = at + bd;
  if (at <= 0) boxThere = 0;
  const char *csvName = NULL;
  for (int i = 1; i < argc; i++) if (!strncmp(argv[i], "csv=", 4)) csvName = argv[i] + 4;
  FILE *csv = csvName ? fopen(csvName, "w") : NULL;
  if (csv) fprintf(csv, "t_s,x_m,y_m,heading_deg,speed_cms,servo_us,sonar_cm\n");

  int n = 0;
  routePlan[n++] = (RouteStep){ ROUTE_STRAIGHT, (float)len };
  if (then) {
    routePlan[n++] = (RouteStep){ ROUTE_TURN, -90.0f };
    routePlan[n++] = (RouteStep){ ROUTE_STRAIGHT, 3.0f };
  }
  routePlan[n] = (RouteStep){ ROUTE_END, 0.0f };
  route_arm();

  double minGap = 1e9;
  int hit = 0;
  for (nowMs = 1; nowMs < 400000 && !routeOver && !hit; nowMs++) {
    plant_step(0.001);
    yawDeg = (float)(head * scale);
    if (boxThere && gone >= 0 && nowMs / 1000.0 >= gone) { boxThere = 0; printf("%7.1f s  (the box is taken away)\n", nowMs / 1000.0); }
    if (boxThere) {
      double g = car_gap(&hit);
      if (g < minGap) minGap = g;
      if (hit) printf("%7.1f s  HIT the box at x %.2f y %.2f\n", nowMs / 1000.0, px, py);
    }
    if (nowMs % 5 == 0) heading_hold();
    if (nowMs % DRIVE_PERIOD_MS == 0) {
      double best = SONAR_MAX_CM;
      if (boxThere) {
        double h = head * M_PI / 180.0, fx = px - sin(h) * 0.15, fy = py + cos(h) * 0.15;
        for (int r = -1; r <= 1; r++) {
          double d = ray_box(fx, fy, head + 8.0 * r);
          if (d >= 0 && d * 100 < best) best = d * 100;
        }
      }
      sonarCm = (float)best;
      drive_update();
      if (csv) fprintf(csv, "%.1f,%.3f,%.3f,%.1f,%.1f,%u,%.0f\n", nowMs / 1000.0, px, py, head, v, (unsigned)servoUs, best);
    }
    if (nowMs % 10 == 0) route_poll(drive_result());
  }
  if (csv) fclose(csv);
  double ex = then ? 3.0 + 0.73 : 0.0, ey = then ? len + 0.73 : len;   /* where the route should end */
  printf("end: x %.2f m, y %.2f m, heading %.1f deg (route plan: x %.2f, y %.2f, %d deg); closest to the box %.0f cm%s\n",
         px, py, head, ex, ey, then ? -90 : 0, minGap * 100, hit ? "  ** HIT **" : "");
  return hit ? 2 : 0;
}
