// =====================================================================
//  encoder.h  -  quadrature encoder on TIM2 (hardware counting, x4)
//  The timer counts every edge of A and B by itself, so no counts are
//  lost even at full motor speed, and it costs zero CPU time.
// =====================================================================
#pragma once

static int32_t encOffset = 0;

static void encoderBegin() {
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef g = {};
  g.Pin       = GPIO_PIN_0 | GPIO_PIN_1;   // PA0 = TIM2_CH1, PA1 = TIM2_CH2
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_PULLUP;               // hall outputs are often open-collector
  g.Speed     = GPIO_SPEED_FREQ_LOW;
  g.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &g);

  TIM2->CR1   = 0;
  TIM2->SMCR  = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;           // encoder mode 3: count A and B edges
  TIM2->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0        // CH1->TI1, CH2->TI2
              | (0x6 << TIM_CCMR1_IC1F_Pos)                // input noise filter
              | (0x6 << TIM_CCMR1_IC2F_Pos);
  TIM2->CCER  = 0;                                          // no inversion
  TIM2->ARR   = 0xFFFFFFFF;                                 // 32-bit counter
  TIM2->CNT   = 0;
  TIM2->CR1   = TIM_CR1_CEN;
}

static inline int32_t encoderRaw() {
  int32_t c = (int32_t)TIM2->CNT;
  return ENCODER_INVERT ? -c : c;
}

static void encoderZero()            { encOffset = encoderRaw(); }
static inline int32_t encoderCount() { return encoderRaw() - encOffset; }
