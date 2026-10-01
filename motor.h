// =====================================================================
//  motor.h  -  rear DC motor through the L298N
// =====================================================================
#pragma once

static float motorPct = 0;   // last command, -100..100 (for telemetry)

static void motorBegin() {
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  analogWriteResolution(PWM_BITS);
  analogWriteFrequency(PWM_FREQ_HZ);
  analogWrite(PIN_MOTOR_EN, 0);
}

// pct: -100 (full reverse) .. +100 (full forward), 0 = coast
static void motorSet(float pct) {
  pct = constrain(pct, -100.0f, 100.0f);
  motorPct = pct;
  if (MOTOR_INVERT) pct = -pct;

  if (pct > 0)      { digitalWrite(PIN_MOTOR_IN1, HIGH); digitalWrite(PIN_MOTOR_IN2, LOW);  }
  else if (pct < 0) { digitalWrite(PIN_MOTOR_IN1, LOW);  digitalWrite(PIN_MOTOR_IN2, HIGH); }
  else              { digitalWrite(PIN_MOTOR_IN1, LOW);  digitalWrite(PIN_MOTOR_IN2, LOW);  }

  analogWrite(PIN_MOTOR_EN, (int)(fabsf(pct) * PWM_MAX / 100.0f + 0.5f));
}

// Active brake: both motor terminals tied together -> stops fast and
// holds position much better than coasting (important for waypoints).
static void motorBrake() {
  motorPct = 0;
  digitalWrite(PIN_MOTOR_IN1, HIGH);
  digitalWrite(PIN_MOTOR_IN2, HIGH);
  analogWrite(PIN_MOTOR_EN, PWM_MAX);
}
