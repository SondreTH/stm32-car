/* =====================================================================
 * car_hw.c - hardware layer (HAL). Peripheral setup itself is generated
 * by CubeMX from Team7_Car.ioc; this file only starts and uses it.
 * ===================================================================== */
#include "main.h"
#include "car_hw.h"
#include "car_config.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;   /* motor PWM, PA8       */
extern TIM_HandleTypeDef htim2;   /* servo PWM, PB10 CH3  */
extern TIM_HandleTypeDef htim5;   /* encoder, PA0/PA1     */
extern I2C_HandleTypeDef hi2c1;   /* MPU-6050             */
extern ADC_HandleTypeDef hadc1;   /* battery, PA4         */

static float clampf_(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* ------------------------------------------------------------------ time */
/* Microseconds from SysTick (1 ms tick + the counter inside the tick).
 * Safe in interrupts too: if SysTick has wrapped but its interrupt has not
 * run yet (pending), the missing millisecond is added. */
uint32_t hw_micros(void) {
  uint32_t ms, val, pend;
  do {
    ms   = HAL_GetTick();
    val  = SysTick->VAL;
    pend = SCB->ICSR & SCB_ICSR_PENDSTSET_Msk;
  } while (ms != HAL_GetTick());
  uint32_t load = SysTick->LOAD + 1u;
  if (pend && val > load / 2u) ms++;
  return ms * 1000u + (load - 1u - val) / (load / 1000u);
}
uint32_t hw_millis(void) { return HAL_GetTick(); }
void hw_delay_us(uint32_t us) { uint32_t t0 = hw_micros(); while (hw_micros() - t0 < us) { } }

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

/* -------------------------------------------------------------- steering */
/* TIM2 CH3 on PB10: 1 tick = 1 us, period 20 ms -> compare value = pulse in us */
float steerDeg = 0;
int   steerUs  = 1500;

void steer_set_us(int us, bool rawCalibration) {
  if (rawCalibration) us = (int)clampf_((float)us, 900, 2100);
  else                us = (int)clampf_((float)us, (float)SERVO_MIN_US, (float)SERVO_MAX_US);
  steerUs = us;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)us);
}

void steer_set_deg(float deg) {
  deg = clampf_(deg, -STEER_MAX_DEG, STEER_MAX_DEG);
  steerDeg = deg;
  float d = STEER_INVERT ? -deg : deg;
  steer_set_us(SERVO_CENTER_US + (int)lroundf(d * SERVO_US_PER_DEG), false);
}

/* ----------------------------------------------------------------- sonar */
/* ECHO pulse is timed by the EXTI7 interrupt (both edges), never blocking */
static volatile uint32_t sonarRiseUs = 0, sonarWidthUs = 0;
static volatile bool     sonarNew = false;
static uint32_t sonarLastTrigMs = 0;
static float    sonarHist[3] = {SONAR_MAX_CM, SONAR_MAX_CM, SONAR_MAX_CM};
static uint8_t  sonarIdx = 0;
float sonarCm = SONAR_MAX_CM;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin != SONAR_ECHO_Pin) return;
  uint32_t now = hw_micros();
  if (HAL_GPIO_ReadPin(SONAR_ECHO_GPIO_Port, SONAR_ECHO_Pin) == GPIO_PIN_SET) sonarRiseUs = now;
  else { sonarWidthUs = now - sonarRiseUs; sonarNew = true; }
}

static float median3(float a, float b, float c) {
  float t;
  if (a > b) { t = a; a = b; b = t; }
  if (b > c) { t = b; b = c; c = t; }
  if (a > b) { t = a; a = b; b = t; }
  return b;
}

void sonar_update(void) {
  uint32_t now = hw_millis();
  if (now - sonarLastTrigMs < SONAR_PERIOD_MS) return;
  /* 1) result of the previous ping */
  __disable_irq();
  bool got = sonarNew; uint32_t w = sonarWidthUs; sonarNew = false;
  __enable_irq();
  float cm = SONAR_MAX_CM;
  if (got && w > 100 && w < (uint32_t)SONAR_MAX_CM * 58u) cm = w / 58.0f;
  sonarHist[sonarIdx] = cm;
  sonarIdx = (uint8_t)((sonarIdx + 1) % 3);
  sonarCm = median3(sonarHist[0], sonarHist[1], sonarHist[2]);
  /* 2) next ping: 10 us trigger pulse */
  HAL_GPIO_WritePin(SONAR_TRIG_GPIO_Port, SONAR_TRIG_Pin, GPIO_PIN_SET);
  hw_delay_us(10);
  HAL_GPIO_WritePin(SONAR_TRIG_GPIO_Port, SONAR_TRIG_Pin, GPIO_PIN_RESET);
  sonarLastTrigMs = now;
}

/* -------------------------------------------------------------- MPU-6050 */
#define MPU_ADDR (0x68 << 1)
bool    imuOk = false;
uint8_t imuWhoAmI = 0;
float   gyroBiasDps = 0, gyroRateDps = 0, headingDeg = 0;

static bool mpu_write(uint8_t reg, uint8_t val) {
  return HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 5) == HAL_OK;
}
static bool mpu_read(uint8_t reg, uint8_t *buf, uint16_t n) {
  return HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, n, 5) == HAL_OK;
}

/* raw yaw rate in deg/s (no bias correction) */
static bool gyro_z_raw(float *dps) {
  uint8_t b[2];
  if (!mpu_read(0x47, b, 2)) return false;           /* GYRO_ZOUT_H/L */
  int16_t raw = (int16_t)((b[0] << 8) | b[1]);
  float v = raw / 65.5f;                              /* +-500 dps range */
  *dps = GYRO_INVERT ? -v : v;
  return true;
}

void imu_calibrate(uint16_t ms) {
  if (!imuOk) return;
  float sum = 0; uint32_t n = 0, t0 = hw_millis();
  while (hw_millis() - t0 < ms) {
    float d;
    if (gyro_z_raw(&d)) { sum += d; n++; }
    HAL_Delay(2);
  }
  if (n) gyroBiasDps = sum / n;
}

void imu_begin(void) {
  uint8_t who = 0;
  imuOk = mpu_read(0x75, &who, 1);                   /* WHO_AM_I */
  imuWhoAmI = who;
  if (!imuOk) return;
  mpu_write(0x6B, 0x80); HAL_Delay(100);             /* reset */
  mpu_write(0x6B, 0x01);                             /* wake, PLL clock */
  mpu_write(0x1A, 0x03);                             /* DLPF ~42 Hz */
  mpu_write(0x19, 0x00);                             /* 1 kHz sample rate */
  mpu_write(0x1B, 0x08);                             /* gyro +-500 dps */
  mpu_write(0x1C, 0x00);                             /* accel +-2 g (unused) */
  HAL_Delay(100);
  imu_calibrate(1000);
}

/* Heading = integral of yaw rate. While the car is certainly still, the
 * bias is re-learned and small rates are ignored (drift correction). */
void imu_update(float dt, bool still) {
  if (!imuOk) return;
  float raw;
  if (!gyro_z_raw(&raw)) return;
  float rate = raw - gyroBiasDps;
  if (still && fabsf(rate) < GYRO_STILL_DPS) {
    gyroBiasDps += GYRO_BIAS_LEARN * (raw - gyroBiasDps);
    gyroRateDps = 0;
    return;
  }
  gyroRateDps = rate * GYRO_SCALE;
  headingDeg += gyroRateDps * dt;
}

/* --------------------------------------------------------------- battery */
float battV = 0;
static uint32_t battLastMs = 0;

void battery_update(void) {
  if (hw_millis() - battLastMs < 200) return;
  battLastMs = hw_millis();
  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 2) != HAL_OK) return;
  float v = HAL_ADC_GetValue(&hadc1) * 3.3f / 4095.0f * BATT_DIVIDER;
  battV = (battV < 0.1f) ? v : battV + 0.2f * (v - battV);
}

/* ------------------------------------------------------- buttons and LED */
bool button_pressed(void) {
  return HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET ||
         HAL_GPIO_ReadPin(START_BTN_GPIO_Port, START_BTN_Pin) == GPIO_PIN_RESET;
}
void led_set(bool on) { HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }

/* ------------------------------------------------------------------ init */
void hw_init(void) {
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);    /* encoder */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);          /* motor PWM (sets MOE on TIM1) */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);          /* servo */
  motor_brake();
  steer_set_deg(0);
  encoder_zero();
  imu_begin();                                       /* car must be still: gyro calibration */
}
