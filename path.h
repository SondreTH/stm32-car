// =====================================================================
//  path.h  -  the route, and how to follow it (pure pursuit)
//
//  A path is a list of points (x, y) in metres, in the same frame as the
//  odometry: start = (0,0), x = forward at start, y = left at start.
//  Some points are flagged: STOP (waypoint, wait 10 s), FINISH (end of run),
//  NOOBS (obstacle detection off from here, e.g. near a wall).
//
//  Plain C++ (no Arduino calls) so it can be tested on a PC.
// =====================================================================
#pragma once
#include <math.h>
#include <stdint.h>

#define PATH_MAX_PTS 800            // 800 x 0.25 m = 200 m of route

enum : uint8_t { PF_NONE = 0, PF_STOP = 1, PF_FINISH = 2, PF_NOOBS = 4 };

struct PathPt { float x, y; uint8_t flags; };

struct Path {
  PathPt   p[PATH_MAX_PTS];
  float    s[PATH_MAX_PTS];         // distance along the path to each point
  uint16_t n = 0;
  float    total = 0;
};
static Path path;

static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static void pathClear() { path.n = 0; path.total = 0; }

static bool pathAdd(float x, float y, uint8_t flags) {
  if (path.n >= PATH_MAX_PTS) return false;
  uint16_t i = path.n;
  path.p[i].x = x; path.p[i].y = y; path.p[i].flags = flags;
  path.s[i] = (i == 0) ? 0 : path.s[i - 1] + hypotf(x - path.p[i - 1].x, y - path.p[i - 1].y);
  path.total = path.s[i];
  path.n++;
  return true;
}

static void pathLoad(const PathPt* src, uint16_t n) {
  pathClear();
  for (uint16_t i = 0; i < n; i++) pathAdd(src[i].x, src[i].y, src[i].flags);
}

// Point at distance s along the path, and the path direction there (rad).
// Beyond the end, the last segment is extended straight.
static void pathPointAt(float s, float& x, float& y, float& dir) {
  if (path.n == 0) { x = y = dir = 0; return; }
  if (path.n == 1) { x = path.p[0].x; y = path.p[0].y; dir = 0; return; }
  // find segment i with s[i] <= s < s[i+1] (skip zero-length segments)
  uint16_t lo = 0, hi = path.n - 1;
  if (s <= 0) { lo = 0; }
  else if (s >= path.total) { lo = path.n - 2; }
  else {
    while (hi - lo > 1) { uint16_t mid = (lo + hi) / 2; if (path.s[mid] <= s) lo = mid; else hi = mid; }
  }
  // back up over zero-length segments to get a usable direction
  uint16_t i = lo;
  while (i > 0 && (path.s[i + 1] - path.s[i]) < 1e-4f) i--;
  float ax = path.p[i].x, ay = path.p[i].y;
  float bx = path.p[i + 1].x, by = path.p[i + 1].y;
  float len = hypotf(bx - ax, by - ay);
  dir = (len > 1e-4f) ? atan2f(by - ay, bx - ax) : 0;
  float t = s - path.s[i];                      // may be > len at the end (extrapolate)
  if (s < 0) t = 0;
  x = ax + cosf(dir) * t;
  y = ay + sinf(dir) * t;
}

// ---------------------------------------------------------------------
//  Follower: where are we on the path, and where to steer
// ---------------------------------------------------------------------
struct Follower {
  uint16_t seg  = 0;     // current segment (p[seg] -> p[seg+1])
  float    sNow = 0;     // progress along the path, m
  float    cte  = 0;     // cross-track error, m (+ = car is LEFT of path)
  float    tx = 0, ty = 0;   // current pursuit target (for telemetry/debug)
  // obstacle detour: lateral shift of the path between d0..d3
  bool  detour = false;
  float d0 = 0, d1 = 0, d2 = 0, d3 = 0, dOff = 0;
};
static Follower fol;

static void followerReset() { fol = Follower(); }

// Project the car position onto the path. Only searches a window just
// behind/ahead of the current segment, so on a loop the start and the
// finish (same place!) can never be confused.
static void followerProject(float x, float y) {
  if (path.n < 2) return;
  float best = 1e18f; uint16_t bi = fol.seg; float bs = fol.sNow, bc = 0;
  uint16_t first = (fol.seg > 2) ? fol.seg - 2 : 0;
  uint16_t last  = fol.seg + 40; if (last > path.n - 2) last = path.n - 2;
  for (uint16_t i = first; i <= last; i++) {
    float ax = path.p[i].x, ay = path.p[i].y;
    float vx = path.p[i + 1].x - ax, vy = path.p[i + 1].y - ay;
    float L2 = vx * vx + vy * vy;
    if (L2 < 1e-8f) continue;
    float t = clampf(((x - ax) * vx + (y - ay) * vy) / L2, 0.0f, 1.0f);
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

static float detourOffsetAt(float s) {
  if (!fol.detour || s <= fol.d0 || s >= fol.d3) return 0;
  if (s < fol.d1) return fol.dOff * (s - fol.d0) / (fol.d1 - fol.d0);
  if (s < fol.d2) return fol.dOff;
  return fol.dOff * (fol.d3 - s) / (fol.d3 - fol.d2);
}

// Start a detour: shift the path sideways by 'offset' (+ = left) so the
// car passes an obstacle sitting at path distance sObs.
static void followerStartDetour(float sObs, float offset, float ramp, float pass) {
  fol.d0 = fol.sNow;
  fol.d1 = fol.d0 + ramp;
  if (fol.d1 > sObs - 0.15f) fol.d1 = fol.d0 + fmaxf(0.3f, sObs - 0.15f - fol.d0);
  fol.d2 = fmaxf(fol.d1, sObs) + pass;
  fol.d3 = fol.d2 + ramp;
  fol.dOff = offset;
  fol.detour = true;
}

// Pure pursuit: aim at a point 'lookahead' metres further along the
// (possibly shifted) path and compute the steering angle that drives an
// arc through it.  Returns front-wheel angle in degrees, + = left.
static float followerSteer(float x, float y, float headingDeg, float lookahead, float wheelbase) {
  float px, py, dir;
  float sL = fol.sNow + lookahead;
  pathPointAt(sL, px, py, dir);
  float d = detourOffsetAt(sL);
  px += -sinf(dir) * d;
  py +=  cosf(dir) * d;
  fol.tx = px; fol.ty = py;

  float th = headingDeg * (float)M_PI / 180.0f;
  float dx = px - x, dy = py - y;
  float lx =  cosf(th) * dx + sinf(th) * dy;    // target in car frame
  float ly = -sinf(th) * dx + cosf(th) * dy;
  float L2 = lx * lx + ly * ly;
  if (L2 < 1e-4f) return 0;
  float kappa = 2.0f * ly / L2;                  // curvature of the arc
  return atanf(kappa * wheelbase) * 180.0f / (float)M_PI;
}
