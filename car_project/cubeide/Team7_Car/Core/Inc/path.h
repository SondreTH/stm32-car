/* =====================================================================
 * path.h - the route, and how to follow it (pure pursuit)
 *
 * A path is a list of points (x, y) in metres in the odometry frame:
 * start = (0,0), x = forward at start, y = left at start. Flags mark
 * STOP (waypoint, wait 10 s), FINISH, NOOBS (obstacle detection off).
 * Plain C, no hardware: the PC simulator (tools/sim.cpp) uses it too.
 * ===================================================================== */
#ifndef PATH_H
#define PATH_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PATH_MAX_PTS 800            /* 800 x 0.25 m = 200 m of route */

enum { PF_NONE = 0, PF_STOP = 1, PF_FINISH = 2, PF_NOOBS = 4 };

typedef struct { float x, y; uint8_t flags; } PathPt;

typedef struct {
  PathPt   p[PATH_MAX_PTS];
  float    s[PATH_MAX_PTS];         /* distance along the path to each point */
  uint16_t n;
  float    total;
} Path;

typedef struct {
  uint16_t seg;                     /* current segment p[seg] -> p[seg+1] */
  float    sNow;                    /* progress along the path, m */
  float    cte;                     /* cross-track error, m, + = car LEFT of path */
  float    tx, ty;                  /* current pursuit target */
  bool     detour;                  /* obstacle detour: path shifted between d0..d3 */
  float    d0, d1, d2, d3, dOff;
} Follower;

extern Path path;
extern Follower fol;

void  path_clear(void);
bool  path_add(float x, float y, uint8_t flags);
void  path_load(const PathPt *src, uint16_t n);
void  path_point_at(float s, float *x, float *y, float *dir);

void  follower_reset(void);
void  follower_project(float x, float y);
void  follower_start_detour(float sObs, float offset, float ramp, float pass);
float follower_steer(float x, float y, float headingDeg, float lookahead, float wheelbase);

#ifdef __cplusplus
}
#endif
#endif
