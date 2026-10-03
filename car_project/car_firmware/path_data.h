// path_data.h - the built-in route loaded at power-on.
// DEMO PATH for an indoor test: 2 m straight -> STOP, 90 deg left turn
// (radius 1 m), 1 m straight -> FINISH.  Needs ~2 x 3 m of floor.
// Replace this file with the one saved from DUMP after teaching the course.
// flags: 1 = STOP (waypoint), 2 = FINISH, 4 = obstacle detection off
#pragma once
const PathPt BUILTIN_PATH[] = {
  {0.000f, 0.000f, 0},
  {0.250f, 0.000f, 0},
  {0.500f, 0.000f, 0},
  {0.750f, 0.000f, 0},
  {1.000f, 0.000f, 0},
  {1.250f, 0.000f, 0},
  {1.500f, 0.000f, 0},
  {1.750f, 0.000f, 0},
  {2.000f, 0.000f, 1},
  {2.223f, 0.025f, 0},
  {2.434f, 0.099f, 0},
  {2.623f, 0.218f, 0},
  {2.782f, 0.377f, 0},
  {2.901f, 0.566f, 0},
  {2.975f, 0.777f, 0},
  {3.000f, 1.000f, 0},
  {3.000f, 1.250f, 0},
  {3.000f, 1.500f, 0},
  {3.000f, 1.750f, 0},
  {3.000f, 2.000f, 2},
};
const uint16_t BUILTIN_PATH_N = sizeof(BUILTIN_PATH) / sizeof(BUILTIN_PATH[0]);
