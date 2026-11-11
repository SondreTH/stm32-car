/* =====================================================================
 * car_oled.h - small I2C OLED on I2C1 (PB8 SCL, PB9 SDA)
 *   128x64 or 128x32, SSD1306 or SH1106 controller
 *
 * Drawing only changes a framebuffer in RAM; oled_update() sends it to the
 * screen. Pixels: (0,0) is top left, x to the right, y down.
 * Typical frame: oled_clear(); draw...; oled_update();
 * ===================================================================== */
#ifndef CAR_OLED_H_
#define CAR_OLED_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OLED_WIDTH      128
#define OLED_HEIGHT     64  /* 64 = 0.96" 128x64, 32 = 0.91" 128x32             */
#define OLED_COL_OFFSET 0   /* 0 = SSD1306, 2 = SH1106 (most 1.3" modules)       */
#define OLED_FLIP       0   /* 1 = rotate 180 degrees if it reads upside down    */

uint8_t oled_init(void);     /* find (0x3C/0x3D) and set up: returns the address, 0 = not found */
bool    oled_update(void);   /* framebuffer -> screen, blocking ~25 ms. false = no display */
uint8_t oled_get_addr(void); /* returns active 7-bit I2C address (0 if none) */

void oled_clear(void);
void oled_pixel(int x, int y, bool on);
void oled_fill(int x, int y, int w, int h, bool on);   /* filled rectangle */
void oled_frame(int x, int y, int w, int h);           /* rectangle outline */

/* text: 5x7 font in a 6x8 cell, times scale (scale 1 = 21 chars x 8 lines).
 * Background pixels are drawn too. Returns the x just after the text. */
int  oled_text(int x, int y, const char *s, int scale);
int  oled_int(int x, int y, int32_t v, int scale);
int  oled_float(int x, int y, float v, int decimals, int scale);

/* Pre-formatted status screen for the car */
void oled_draw_car_status(float dist_cm, float speed_cms, float set_speed_cms,
                          int drive_state, float target_m, float sonar_cm,
                          float mot_pct, float batt_volts);

#ifdef __cplusplus
}
#endif

#endif /* CAR_OLED_H_ */
