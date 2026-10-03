/* path.c - route storage and pure-pursuit follower (see path.h) */
#include "path.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

Path path;
Follower fol;

static float clampf_(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void path_clear(void) { path.n = 0; path.total = 0; }

bool path_add(float x, float y, uint8_t flags) {
  if (path.n >= PATH_MAX_PTS) return false;
  uint16_t i = path.n;
  path.p[i].x = x; path.p[i].y = y; path.p[i].flags = flags;
  path.s[i] = (i == 0) ? 0 : path.s[i - 1] + hypotf(x - path.p[i - 1].x, y - path.p[i - 1].y);
  path.total = path.s[i];
  path.n++;
  return true;
}

void path_load(const PathPt *src, uint16_t n) {
  path_clear();
  for (uint16_t i = 0; i < n; i++) path_add(src[i].x, src[i].y, src[i].flags);
}

/* Point at distance s along the path and the path direction there (rad).
 * Beyond the end the last segment is extended straight. */
void path_point_at(float s, float *x, float *y, float *dir) {
  if (path.n == 0) { *x = *y = *dir = 0; return; }
  if (path.n == 1) { *x = path.p[0].x; *y = path.p[0].y; *dir = 0; return; }
  uint16_t lo = 0, hi = (uint16_t)(path.n - 1);
  if (s <= 0) lo = 0;
  else if (s >= path.total) lo = (uint16_t)(path.n - 2);
  else while (hi - lo > 1) { uint16_t mid = (uint16_t)((lo + hi) / 2); if (path.s[mid] <= s) lo = mid; else hi = mid; }
  uint16_t i = lo;
  while (i > 0 && (path.s[i + 1] - path.s[i]) < 1e-4f) i--;   /* skip zero-length segments */
  float ax = path.p[i].x, ay = path.p[i].y, bx = path.p[i + 1].x, by = path.p[i + 1].y;
  float len = hypotf(bx - ax, by - ay);
  *dir = (len > 1e-4f) ? atan2f(by - ay, bx - ax) : 0;
  float t = (s < 0) ? 0 : s - path.s[i];
  *x = ax + cosf(*dir) * t;
  *y = ay + sinf(*dir) * t;
}

void follower_reset(void) { memset(&fol, 0, sizeof(fol)); }

/* Project the car onto the path, searching only just behind/ahead of the
 * current segment, so start and finish (same place) are never confused. */
void follower_project(float x, float y) {
  if (path.n < 2) return;
  float best = 1e18f, bs = fol.sNow, bc = 0;
  uint16_t bi = fol.seg;
  uint16_t first = (fol.seg > 2) ? (uint16_t)(fol.seg - 2) : 0;
  uint16_t last = (uint16_t)(fol.seg + 40);
  if (last > path.n - 2) last = (uint16_t)(path.n - 2);
  for (uint16_t i = first; i <= last; i++) {
    float ax = path.p[i].x, ay = path.p[i].y;
    float vx = path.p[i + 1].x - ax, vy = path.p[i + 1].y - ay;
    float L2 = vx * vx + vy * vy;
    if (L2 < 1e-8f) continue;
    float t = clampf_(((x - ax) * vx + (y - ay) * vy) / L2, 0.0f, 1.0f);
    float px = ax + t * vx, py = ay + t * vy;
    float d2 = (x - px) * (x - px) + (y - py) * (y - py);
    if (d2 < best) {
      float L = sqrtf(L2);
      best = d2; bi = i; bs = path.s[i] + t * L;
      bc = (vx * (y - ay) - vy * (x - ax)) / L;
    }
  }
  fol.seg = bi; fol.sNow = bs; fol.cte = bc;
}

static float detour_offset_at(float s) {
  if (!fol.detour || s <= fol.d0 || s >= fol.d3) return 0;
  if (s < fol.d1) return fol.dOff * (s - fol.d0) / (fol.d1 - fol.d0);
  if (s < fol.d2) return fol.dOff;
  return fol.dOff * (fol.d3 - s) / (fol.d3 - fol.d2);
}

/* Shift the path sideways by 'offset' (+ = left) around an obstacle at sObs */
void follower_start_detour(float sObs, float offset, float ramp, float pass) {
  fol.d0 = fol.sNow;
  fol.d1 = fol.d0 + ramp;
  if (fol.d1 > sObs - 0.15f) fol.d1 = fol.d0 + fmaxf(0.3f, sObs - 0.15f - fol.d0);
  fol.d2 = fmaxf(fol.d1, sObs) + pass;
  fol.d3 = fol.d2 + ramp;
  fol.dOff = offset;
  fol.detour = true;
}

/* Pure pursuit: aim at a point 'lookahead' metres ahead on the (shifted)
 * path; return the front-wheel angle in degrees (+ = left). */
float follower_steer(float x, float y, float headingDeg, float lookahead, float wheelbase) {
  float px, py, dir;
  float sL = fol.sNow + lookahead;
  path_point_at(sL, &px, &py, &dir);
  float d = detour_offset_at(sL);
  px += -sinf(dir) * d;
  py +=  cosf(dir) * d;
  fol.tx = px; fol.ty = py;
  float th = headingDeg * (float)M_PI / 180.0f;
  float dx = px - x, dy = py - y;
  float lx =  cosf(th) * dx + sinf(th) * dy;
  float ly = -sinf(th) * dx + cosf(th) * dy;
  float L2 = lx * lx + ly * ly;
  if (L2 < 1e-4f) return 0;
  float kappa = 2.0f * ly / L2;
  return atanf(kappa * wheelbase) * 180.0f / (float)M_PI;
}
