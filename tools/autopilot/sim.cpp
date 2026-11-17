// =====================================================================
//  sim.cpp - PC test of the autopilot (the real path.c + autopilot.c)
//
//  Simulates the car on a loop shaped like the challenge course with
//  3 waypoints, one temporary obstacle and one permanent obstacle.
//  Build (from tools/):
//    C=../cubeide/Team7_Car/Core
//    gcc -O2 -c -I$C/Inc $C/Src/path.c $C/Src/autopilot.c $C/Src/car_config.c
//    g++ -O2 -std=c++17 -I$C/Inc sim.cpp path.o autopilot.o car_config.o -o sim
//    ./sim [gyro_scale_error] [t = teach-and-repeat]
//  Writes sim_out.csv (t,x,y,state,...) for plot_run.py.
// =====================================================================
#include <cstdio>
#include <cmath>
#include <vector>
#include <cstdlib>
#include "car_config.h"
#include "autopilot.h"
static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

struct Obst { float x, y, r; float removeAfterWaitS; bool present = true; float seenSince = -1; };

static float frand() { return (float)rand() / RAND_MAX * 2.0f - 1.0f; }

// ---- build a test course: rounded polygon, clockwise like the real map ----
static void addStraight(std::vector<PathPt>& v, float& x, float& y, float h, float len) {
  int n = (int)ceilf(len / 0.25f);
  for (int i = 1; i <= n; i++) v.push_back({x + cosf(h) * len * i / n, y + sinf(h) * len * i / n, 0});
  x += cosf(h) * len; y += sinf(h) * len;
}
static void addArc(std::vector<PathPt>& v, float& x, float& y, float& h, float r, float turnRad) {
  float cx = x - sinf(h) * r * (turnRad > 0 ? 1 : -1), cy = y + cosf(h) * r * (turnRad > 0 ? 1 : -1);
  int n = (int)ceilf(fabsf(turnRad) * r / 0.25f);
  float a0 = atan2f(y - cy, x - cx);
  for (int i = 1; i <= n; i++) {
    float a = a0 + turnRad * i / n;
    v.push_back({cx + r * cosf(a), cy + r * sinf(a), 0});
  }
  h += turnRad; x = v.back().x; y = v.back().y;
}

int main(int argc, char** argv) {
  float gyroScaleErr = argc > 1 ? atof(argv[1]) : 0.0f;   // e.g. 0.01 = 1 % gyro scale error
  srand(1);
  // Course (metres): start X, SW leg to WP1, west, north leg, WP2, east leg, WP3, back to X
  std::vector<PathPt> pts; float x = 0, y = 0, h = 0;
  pts.push_back({0, 0, 0});
  addStraight(pts, x, y, h, 9);           pts.back().flags = PF_STOP;        // WP1
  addArc(pts, x, y, h, 1.2f, -M_PI / 2);  addStraight(pts, x, y, h, 3);
  addArc(pts, x, y, h, 1.2f, -M_PI / 2);  addStraight(pts, x, y, h, 16);
  pts.back().flags = PF_STOP;                                                   // WP2
  addArc(pts, x, y, h, 1.2f, -M_PI / 2);  addStraight(pts, x, y, h, 11);
  pts.back().flags = PF_STOP;                                                   // WP3
  // back to the start: aim straight at (0,0) after one more turn
  addArc(pts, x, y, h, 1.2f, -M_PI / 2);
  { float hh = atan2f(0 - y, 0 - x); float d = hypotf(x, y);
    // turn towards start with an arc, then straight
    float turn = remainderf(hh - h, 2 * M_PI); addArc(pts, x, y, h, 1.2f, turn);
    hh = atan2f(-y, -x); d = hypotf(x, y); addStraight(pts, x, y, hh, d); pts.back() = {0, 0, PF_FINISH}; }
  // "teach" mode: the path is what the car's OWN odometry measured while
  // being driven along the true course (same gyro error as during replay)
  bool teach = argc > 2 && argv[2][0] == 't';
  std::vector<PathPt> truth = pts;
  if (teach) {
    float th = 0, px = 0, py = 0, prevTrue = 0;
    for (size_t i = 1; i < pts.size(); i++) {
      float dx = truth[i].x - truth[i-1].x, dy = truth[i].y - truth[i-1].y, d = hypotf(dx, dy);
      if (d < 1e-6f) { pts[i].x = px; pts[i].y = py; continue; }
      float ht = atan2f(dy, dx);
      th += remainderf(ht - prevTrue, 2 * M_PI) * (1 + gyroScaleErr); prevTrue = ht;
      px += d * cosf(th); py += d * sinf(th); pts[i].x = px; pts[i].y = py;
    }
  }
  path_load(pts.data(), (uint16_t)pts.size());
  printf("course: %d points, %.1f m\n", path.n, path.total);

  // obstacles: one temporary (a person) on leg 1, one permanent box on the long leg
  std::vector<Obst> obs;   // placed on the TRUE course
  auto onTruth = [&](float sWant, float& ox, float& oy) { float acc = 0;
    for (size_t i = 1; i < truth.size(); i++) { float d = hypotf(truth[i].x - truth[i-1].x, truth[i].y - truth[i-1].y);
      if (acc + d >= sWant) { float t = (sWant - acc) / d; ox = truth[i-1].x + t * (truth[i].x - truth[i-1].x); oy = truth[i-1].y + t * (truth[i].y - truth[i-1].y); return; } acc += d; } };
  float ox, oy; onTruth(4.0f, ox, oy);  obs.push_back({ox, oy, 0.15f, 3.0f});
  onTruth(22.0f, ox, oy);               obs.push_back({ox, oy, 0.15f, 1e9f});

  // true car state (rear axle), estimated state (odometry)
  float X = 0, Y = 0, TH = 0, V = 0, DEL = 0;
  float ex = 0, ey = 0, eth = 0, odo = 0;
  ap_start(0);
  FILE* f = fopen("sim_out.csv", "w");
  fprintf(f, "t_ms,state,x_m,y_m,true_x,true_y,v,steer,sonar_cm,s,cte\n");
  const float dt = 0.01f; float minClear = 1e9f; float maxCte = 0;
  int wp = 0;
  for (int k = 0; k < 15 * 60 * 100; k++) {
    uint32_t ms = k * 10; float t = k * dt;
    // --- sonar (from true pose), cone +-12 deg ---
    float sx = X + cosf(TH) * SONAR_X_M, sy = Y + sinf(TH) * SONAR_X_M, son = SONAR_MAX_CM;
    for (auto& o : obs) {
      if (!o.present) continue;
      float dx = o.x - sx, dy = o.y - sy, d = hypotf(dx, dy);
      float ang = fabsf(remainderf(atan2f(dy, dx) - TH, 2 * M_PI));
      if (ang < 12 * M_PI / 180 && d - o.r < 4.0f) son = fminf(son, (d - o.r) * 100 + frand());
      // car body clearance (circle around car centre)
      float cx = X + cosf(TH) * 0.12f, cy = Y + sinf(TH) * 0.12f;
      minClear = fminf(minClear, hypotf(o.x - cx, o.y - cy) - o.r - 0.10f);
    }
    // temporary obstacle leaves after being "seen" for its time
    for (auto& o : obs)
      if (o.present && ap.state == AP_OBST_WAIT) { if (o.seenSince < 0) o.seenSince = t; if (t - o.seenSince > o.removeAfterWaitS) o.present = false; }

    ApIn in{ms, ex, ey, eth * 180 / (float)M_PI, odo, son};
    ApOut out; ApState before = ap.state;
    ap_update(&in, &out);
    if (ap.ev) {
      if (ap.ev == EV_WAYPOINT || ap.ev == EV_FINISHED) {
        float px = truth[ap.stopIdx].x, py = truth[ap.stopIdx].y;     // real waypoint on the ground
        printf("t=%6.1fs  %s %d  true error %.1f cm\n", t, ap.ev == EV_FINISHED ? "FINISH" : "WAYPOINT", (int)ap.evA, hypotf(X - px, Y - py) * 100);
        wp++;
      } else {
        const char* names[] = {"", "", "", "leaving", "OBSTACLE", "cleared", "REROUTE", "detour done", "no path"};
        printf("t=%6.1fs  %s\n", t, names[ap.ev]);
      }
      ap.ev = EV_NONE;
    }
    (void)before;
    if (ap.state == AP_DONE) { printf("done in %.0f s, min obstacle clearance %.2f m, max |cte| %.2f m\n", t, minClear, maxCte); break; }
    if (ap.state == AP_RUN && !fol.detour) maxCte = fmaxf(maxCte, fabsf(fol.cte));

    // --- actuators: speed lag, servo slew, clamp ---
    float vCmd = out.brake ? 0 : out.speed;
    float tau = out.brake ? 0.05f : 0.2f;
    V += (vCmd - V) * dt / tau;
    float dCmd = clampf(out.steerDeg, -STEER_MAX_DEG, STEER_MAX_DEG) * (float)M_PI / 180;
    float slew = 300 * (float)M_PI / 180 * dt;
    DEL += clampf(dCmd - DEL, -slew, slew);
    // --- true motion (bicycle model, rear axle) ---
    float dTH = V / WHEELBASE_M * tanf(DEL) * dt;
    X += V * cosf(TH + dTH / 2) * dt; Y += V * sinf(TH + dTH / 2) * dt; TH += dTH;
    // --- odometry like the firmware (encoder + gyro with scale error) ---
    float dsE = V * dt, dthE = dTH * (1 + gyroScaleErr);
    ex += dsE * cosf(eth + dthE / 2); ey += dsE * sinf(eth + dthE / 2); eth += dthE; odo += dsE;
    if (k % 10 == 0) fprintf(f, "%u,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.0f,%.2f,%.3f\n", ms, ap.state, ex, ey, X, Y, V, DEL * 180 / M_PI, son, fol.sNow, fol.cte);
  }
  fclose(f);
  return 0;
}
