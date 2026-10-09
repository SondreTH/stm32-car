/* car_config.c - default values (see car_config.h).
 * Values changed with commands are lost at reset: copy good ones here. */
#include "car_config.h"

/* Counts per metre: drive a measured distance, counts / metres. Placeholder.
 * TIM5 counts every edge of A and B (x4). */
float COUNTS_PER_METER = 20000.0f;
