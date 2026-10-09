/* =====================================================================
 * car_hw.c - hardware layer (HAL). Peripheral setup itself is generated
 * by CubeMX from stm32-car.ioc; this file only starts and uses it.
 * ===================================================================== */
#include "main.h"
#include "car_hw.h"
#include "car_config.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;   /* motor PWM, PA8   */
extern TIM_HandleTypeDef htim5;   /* encoder, PA0/PA1 */

static float clampf_(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* --------------------------------------------------------------- encoder */
/* TIM5 counts every edge of A and B in hardware (x4), 32-bit, input filter 6 */
static int32_t encOffset = 0;
static int32_t encoder_raw(void) {
  int32_t c = (int32_t)TIM5->CNT;
  return ENCODER_INVERT ? -c : c;
}
void    encoder_zero(void)  { encOffset = encoder_raw(); }
int32_t encoder_count(void) { return encoder_raw() - encOffset; }

/* ----------------------------------------------------------------- motor */
float motorPct = 0;

void motor_set(float pct) {
  pct = clampf_(pct, -100.0f, 100.0f);
  motorPct = pct;
  if (MOTOR_INVERT) pct = -pct;
  GPIO_PinState in1 = GPIO_PIN_RESET, in2 = GPIO_PIN_RESET;
  if (pct > 0)      { in1 = GPIO_PIN_SET; }
  else if (pct < 0) { in2 = GPIO_PIN_SET; }
  HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, in1);
  HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, in2);
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(fabsf(pct) * (arr + 1) / 100.0f));
}

void motor_brake(void) {
  motorPct = 0;
  HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, __HAL_TIM_GET_AUTORELOAD(&htim1) + 1);
}

/* ------------------------------------------------------------------ init */
void hw_init(void) {
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);    /* encoder */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);          /* motor PWM (sets MOE on TIM1) */
  motor_brake();
  encoder_zero();
}
