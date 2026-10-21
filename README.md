# Team 7 (Ninjago) car: OLED + distance runs

This branch, `oled`, is the most up-to-date car firmware. It combines:

- **The OLED display** (from `activity2.6`) on I2C1, showing the run,
  speed, motor % and battery voltage at 5 Hz.
- **Hakon's car code** (from `hakon-backup`, the `Hakons_leg` project):
  - distance runs with a trapezoidal speed profile (M2.9) and a PID speed
    loop (M2.10);
  - the Bluetooth commands;
  - the steering servo;
  - the start buttons.

  It now lives in `Core/Src/car_drive.c`. The variable names are the same as
  in his `main.c`, so the CubeMonitor setup still works: `Kp`, `Ki`, `Kd`,
  `test_dist_m`, `SP_Vel`, `AverageSpeed`, `Real_Dist`, `state`, …
- **Everything from the Activity 2.5 base**: USB and Bluetooth commands at
  the same time, telemetry, encoder on TIM5.

## Get it

You need the normal team setup first. If you haven't done it, follow
**First-time setup** in the
[README on the `stm32-car` branch](https://github.com/SondreTH/stm32-car/tree/stm32-car#first-time-setup).

Then, in your `stm32-car` folder:

1. Commit or push your changes first. Git won't switch branches if they
   would be overwritten.
2. Run:
   ```
   git fetch
   git switch oled
   ```
3. In CubeIDE, right-click `stm32-car` and choose **Refresh** (F5).
4. Go to **Project → Clean…**, select `stm32-car`, then click **Clean**.
5. Build and run as usual.

In generated files, only write code between `USER CODE BEGIN` and
`USER CODE END`. The pins are set in `stm32-car.ioc`.

## Pins

| Pin | Morpho | Name in code | Goes to |
|---|---|---|---|
| PA0 | CN7-28 | `ENC_A` | Encoder A (yellow) |
| PA1 | CN7-30 | `ENC_B` | Encoder B (green) |
| PA4 | CN7-32 | `BATT_SENSE` | Battery divider (10k from battery +, 3.3k to GND) |
| PB0 | CN7-34 | `START_BTN` | Start button (other leg to GND) |
| PA7 | CN10-15 | `SONAR_ECHO` | HC-SR04 ECHO, through the 1k/2k divider |
| PB6 | CN10-17 | `SONAR_TRIG` | HC-SR04 TRIG |
| PA9 | CN10-21 | `BT_TX` | Bluetooth RXD |
| PA8 | CN10-23 | `MOTOR_ENA` | L298N ENA (jumper removed) |
| PB10 | CN10-25 | `SERVO_PWM` | Servo signal (orange) |
| PB4 | CN10-27 | `MOTOR_IN1` | L298N IN1 |
| PB5 | CN10-29 | `MOTOR_IN2` | L298N IN2 |
| PA10 | CN10-33 | `BT_RX` | Bluetooth TXD |
| PB8 | CN10-3 | `I2C_SCL` | OLED SCK/SCL |
| PB9 | CN10-5 | `I2C_SDA` | OLED SDA |

Power the OLED from 3.3 V (VDD/VCC) and GND.

The sonar pins are set up but no code reads the sonar yet. The MPU-6050
(PC1 SCL, PC0 SDA) is wired but not used: PC0/PC1 have no hardware I2C.

## Commands (USB 115200 / Bluetooth 9600, line ending LF)

Each command is one letter, followed by an optional number. `d1.2` and
`D 1.2` mean the same thing.

| Command | Does |
|---|---|
| `d1.2` | Distance run of 1.2 m: 18 cm/s, 2 s ramps, PID |
| `v18` | Constant speed in cm/s, open loop |
| `f` / `b` | Forward / back at 60 % |
| `1`…`6` | 50…100 %, same direction (only while moving) |
| `s` | Stop (coast) |
| `l` / `c` / `r` | Steer left / centre / right |
| `P 40` | Open-loop motor %, −100..100 |
| `X` | Stop and brake |
| `Z` | Zero the encoder |
| `M 1035` | Set counts per metre |
| `T 5` | Telemetry at 5 Hz (`T 0` = off) |
| `G` / `?` | Settings / help |
| `I` | Scan I2C1, restart the OLED |

**Start buttons.** Press the blue button (B1) or the start button on PB0.
The car does a 1 m run 1 s later. Any other drive command cancels it.

**CubeMonitor.** Write a distance in metres to `test_dist_m` to start a run.
You can also tune `Kp`, `Ki` and `Kd` live.

## Settings

Every tunable number is in `Core/Inc/car_config.h`, apart from the PID
gains, which are at the top of `car_drive.c`. The values that came from
Hakon's tuning on the car:

- 1035 counts per metre;
- servo 600 / 1100 / 1500 µs for left / straight / right;
- motor offset 45 % plus 0.59 % per cm/s;
- Kp 3, Ki 2, Kd 0.

## Changes compared with Hakon's `main.c`

- **`v` now drives.** Before, it always stopped the motor.
- **The start button no longer blocks for 1 s,** so commands and the OLED
  keep working during the countdown.
- **Holding the button no longer starts a new run** when the last one
  finishes.
- **The clock is 84 MHz instead of 72 MHz.** The timer settings are
  scaled to match, so PWM, servo and timing are unchanged.
- **USB serial runs at 115200** (Bluetooth is still 9600).

## Check after flashing

1. `f` must drive forward. It sets the same pins as Hakon's code did. If it
   goes backwards, set `MOTOR_INVERT` to 1 in `car_config.h`.
2. While driving forward, `G` must show the counts going up. If they go
   down, set `ENCODER_INVERT` to 1. Runs work either way, but the OLED
   counts and telemetry would be negative.
3. `G` shows the battery voltage. Compare it with a multimeter and trim
   `BATT_DIVIDER` if needed.
4. Run `d1` and measure how far the car actually went.
