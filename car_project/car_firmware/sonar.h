// =====================================================================
//  sonar.h  -  HC-SR04, non-blocking (echo timed by interrupt)
//  pulseIn() would freeze the control loop for up to 25 ms, so the echo
//  pin is timed with an interrupt instead. Median of the last 3 readings.
//  ECHO is a 5 V signal: use a divider (1k from ECHO, 2k to GND).
// =====================================================================
#pragma once

static volatile uint32_t sonarRiseUs  = 0;
static volatile uint32_t sonarWidthUs = 0;
static volatile bool     sonarNew     = false;
static uint32_t sonarLastTrigMs = 0;
static float    sonarHist[3] = {SONAR_MAX_CM, SONAR_MAX_CM, SONAR_MAX_CM};
static uint8_t  sonarIdx = 0;
static float    sonarCm  = SONAR_MAX_CM;   // filtered result

static void sonarEchoISR() {
  uint32_t now = micros();
  if (digitalRead(PIN_SONAR_ECHO)) sonarRiseUs = now;
  else { sonarWidthUs = now - sonarRiseUs; sonarNew = true; }
}

static float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

static void sonarBegin() {
  pinMode(PIN_SONAR_TRIG, OUTPUT);
  digitalWrite(PIN_SONAR_TRIG, LOW);
  pinMode(PIN_SONAR_ECHO, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_SONAR_ECHO), sonarEchoISR, CHANGE);
}

// Call every loop(); it only does work every SONAR_PERIOD_MS.
static void sonarUpdate() {
  uint32_t now = millis();
  if (now - sonarLastTrigMs < SONAR_PERIOD_MS) return;

  // 1) collect the result of the previous ping
  noInterrupts();
  bool got = sonarNew;  uint32_t w = sonarWidthUs;  sonarNew = false;
  interrupts();
  float cm = SONAR_MAX_CM;                         // no echo = nothing seen
  if (got && w > 100 && w < (uint32_t)SONAR_MAX_CM * 58) cm = w / 58.0f;
  sonarHist[sonarIdx] = cm;
  sonarIdx = (sonarIdx + 1) % 3;
  sonarCm = median3(sonarHist[0], sonarHist[1], sonarHist[2]);

  // 2) fire the next ping
  digitalWrite(PIN_SONAR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_SONAR_TRIG, LOW);
  sonarLastTrigMs = now;
}
