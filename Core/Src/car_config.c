/* car_config.c - default values (see car_config.h).
 * Values changed with commands are lost at reset: copy good ones here. */
#include "car_config.h"

/* Counts per metre, measured on the car: 10.35 counts per cm.
 * TIM5 counts every edge of A and B (x4). */
float COUNTS_PER_METER = 1035.0f;
