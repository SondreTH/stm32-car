/* car_app.h - the car application. main.c calls App_Init() once and
 * App_Loop() forever (inside the USER CODE sections). */
#ifndef CAR_APP_H
#define CAR_APP_H

#ifdef __cplusplus
extern "C" {
#endif

void App_Init(void);
void App_Loop(void);

#ifdef __cplusplus
}
#endif
#endif
