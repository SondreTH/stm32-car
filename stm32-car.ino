// =====================================================================
//  MR2006B Team car - firmware v1.0
//  Board: NUCLEO-F446RE, Arduino IDE 2 + STM32duino core
//
//  Manual driving, teach-and-repeat autonomous mode with waypoint stops,
//  obstacle stop / wait / reroute, telemetry over USB and Bluetooth.
//  Commands: USB Serial Monitor (115200) or Bluetooth (9600), one per
//  line (newline-terminated). Type HELP
//
//  Files:
//    config.h      all pins and tunable numbers      <- edit this one
//    path_data.h   the built-in route (PATH DUMP)    <- and this one
//    encoder/motor/drive/steering/sonar/imu.h   hardware + control loops
//    path.h, autopilot.h                          route following (PC-tested)
//    output.h, modes.h, commands.h                printing, modes, commands
// =====================================================================
#include "config.h"
#include "encoder.h"
#include "motor.h"
#include "drive.h"
#include "steering.h"
#include "sonar.h"
#include "imu.h"
#include "output.h"
#include "modes.h"
#include "commands.h"

void setup() {
  Serial.begin(USB_BAUD);
  BT.begin(BT_BAUD);
  commsBegin();
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT);
  analogReadResolution(12);

  motorBegin();
  encoderBegin();
  steeringBegin();
  sonarBegin();
  imuBegin();              // car must be still here (gyro calibration)
  driveZero();
  driveStop();
  loadBuiltinPath();

  delay(200);
  say("MR2006B car firmware v1.0 ready. Type HELP for commands.");
  if (!imuOk) say("WARNING: MPU-6050 not found - check SDA/SCL/VCC");
  printStatus();
}

static uint32_t lastCtrlUs = 0;

void loop() {
  usbIn.poll(Serial);
  btIn.poll(BT);

  // ---- fixed-rate control loop (100 Hz) ----
  uint32_t nowUs = micros();
  if (nowUs - lastCtrlUs >= 1000000UL / CONTROL_HZ) {
    float dt = (lastCtrlUs == 0) ? 1.0f / CONTROL_HZ : (nowUs - lastCtrlUs) * 1e-6f;
    if (dt > 0.05f) dt = 1.0f / CONTROL_HZ;     // after a blocking call (gyro cal)
    lastCtrlUs = nowUs;

    driveUpdate(dt);                            // encoder -> speed, PI speed loop
    float hBefore = headingDeg;
    imuUpdate(dt, driveIsStill());              // gyro -> heading
    odometryUpdate(hBefore, headingDeg);        // -> x, y
    teachUpdate();                              // record path points
    autoUpdate();                               // autopilot

    // comms watchdog: manual driving stops if the phone/laptop goes silent
    if (watchdogMs && carMode != CM_AUTO && (drv.mode == DM_OPEN || drv.mode == DM_SPEED)
        && millis() - lastCommandMs > watchdogMs) {
      driveStop();
      say("WATCHDOG stop (no command received)");
    }
  }

  if (drv.moveDone) {
    drv.moveDone = false;
    lb.clear(); lb.print("DONE move, distance now "); lb.print(drv.distance, 3);
    lb.print(" m, counts "); lb.print(encoderCount());
    sendLine(false);
  }

  sonarUpdate();
  batteryUpdate();
  buttonUpdate();
  if (!autoCountdownAt) digitalWrite(PIN_LED, drv.mode != DM_STOP || carMode == CM_TEACH);

  if (telemPeriodMs && millis() - telemLastMs >= telemPeriodMs) {
    telemLastMs = millis();
    telemetrySend();
  }

  commsPump();
}
