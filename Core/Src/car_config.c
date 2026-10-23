/* car_config.c - default values (see car_config.h).
 * Values changed with commands are lost at reset: copy good ones here. */
#include "car_config.h"

/* Counts per metre, tuned on the car: 11.70 counts per cm = 1170 counts per metre.
 * TIM5 counts every edge of A and B (x4). Calibrated for exactly 1 meter drive. */
float COUNTS_PER_METER = 1170.0f;
