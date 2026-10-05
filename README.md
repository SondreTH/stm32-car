# Team 7 Autonomous Car (MR2006B)

This is the code for our autonomous car in the MR2006B Industrial Automation course at
Tecnológico de Monterrey. The car has to drive a circuit of about 150 m by itself, stop
for 10 seconds at three waypoints, avoid obstacles and end where it started. Everything
is controlled by an STM32 Nucleo F446RE board.

## How it works

- **Position:** An encoder on the rear motor measures how far the car has driven, and an
  MPU6050 gyro measures which direction it is pointing. From this the car calculates its
  x and y position 100 times per second.
- **Recording and replaying the route:** First we drive the course manually over Bluetooth
  while the car saves its own path and the waypoints. In automatic mode the car follows
  the saved path using pure pursuit, and slows down to stop on each waypoint.
- **Speed control:** A PI controller uses the encoder to keep the car at the speed we ask for.
- **Obstacles:** An HC SR04 ultrasonic sensor makes the car stop if something is in front of it.
  If the obstacle does not move, the car backs up and drives around it.
- **Communication:** Commands and telemetry (CSV) over Bluetooth and over USB.

## Hardware

| Part | Model |
|---|---|
| Microcontroller | STM32 Nucleo F446RE |
| Drive motor | 25GA370 12 V DC motor with gearbox and encoder |
| Motor driver | L298N |
| Steering | MG996R servo |
| Gyro | MPU6050 |
| Distance sensor | HC SR04 |
| Bluetooth | BT04A (ZS040) |
| Power | 3S LiPo battery, XL4015 buck converter for the servo |

## Folders

| Folder | What is inside |
|---|---|
| `cubeide/` | STM32CubeIDE project (`Team7_Car.ioc` and the C code). This is the main version. |
| `car_firmware/` | The same program for Arduino IDE (backup) |
| `tools/` | Python programs for logging, plotting runs and making paths, plus a simulator |
| `cad/` | 3D printed parts (top plate and sensor holder) |
| `wiring/` | Wiring guide for the circuit board |

How to set up the CubeIDE project is explained in `cubeide/README_CubeIDE.md`.
