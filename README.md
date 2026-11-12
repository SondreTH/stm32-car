# Team 7 (Ninjago) autonomous car

Firmware and tools for our car in MR2006B Industrial Automation at
Tecnológico de Monterrey. The car has to drive a circuit of about 150 m by
itself, stop for 10 seconds at three waypoints, avoid obstacles and end
where it started. It is controlled by an STM32 NUCLEO-F446RE.

This is one STM32CubeIDE project, `stm32-car`, at the root of the repository.

## What the car can do now

- **Distance runs.** `d1.2` drives 1.2 m and stops there. The car follows a
  trapezoidal speed profile, and a PID on the position (encoder) keeps it on
  the profile. `d-0.5` does the same in reverse.
- **Distance in a set time.** You choose the distance, the total time and
  the acceleration and deceleration times.
- **Obstacle stop.** The HC-SR04 pauses a run when something is closer than
  30 cm. The run continues once the way is clear.
- **Manual driving** over Bluetooth or USB: forward, back, speed steps,
  steering.
- **OLED display** with the run phase, distance, speed, motor %, battery
  voltage and sonar distance.
- **Telemetry** as CSV on both serial ports, for plotting and for comparing
  with Simulink.

- **Heading hold.** During a run the MPU-6050 gyro measures how far the car
  has turned, and the servo steers it back. New on 2026-11-12, not tested
  on the car yet.

Not in the firmware yet: x/y position, following
a recorded route, and driving around obstacles. An earlier version of the
firmware had these; see [Old autopilot code](#old-autopilot-code).

## Folders

| Folder | What is inside |
|---|---|
| `Core/Src`, `Core/Inc` | Our code (`car_*.c`) and the CubeMX-generated `main.c` |
| `Drivers/` | ST HAL and CMSIS libraries (generated, leave alone) |
| `stm32-car.ioc` | Pins, clocks and peripherals (CubeMX) |
| `tools/` | Laptop tools: serial console with logging, run plots, a PC simulation of the position loop |
| `cad/` | 3D-printed deck plate and sonar bracket (`.scad` + `.stl`) |
| `wiring/` | `team7_wiring.html`: what to solder where on the perfboard (open in a browser) |
| `docs/` | [Setup and team workflow](docs/setup.md) |
| `.claude/` | Detailed project documentation, written for Claude Code and for us |

Our code:

| File | Does |
|---|---|
| `car_app.c` | Commands, start button, OLED screen, telemetry. `App_Init()` and `App_Loop()` are called from `main.c` |
| `car_drive.c` | Distance run: speed profile, position PID, obstacle pause. Runs in the TIM3 interrupt every 100 ms |
| `car_hw.c` | Motor, encoder, servo, battery, sonar, I2C scan |
| `car_oled.c` | OLED driver (SSD1306 / SH1106) with a small font |
| `car_comms.c` | Text in and out on USB and Bluetooth without blocking |
| `car_config.h`, `car_config.c` | Every number you might want to tune |

## Get it running

First time on this computer: follow [docs/setup.md](docs/setup.md). It
covers installing CubeIDE 1.17.0, cloning, importing, building and flashing.

After that:

1. `git pull`
2. In CubeIDE, right-click `stm32-car` and choose **Refresh** (F5).
3. Build and run as usual.

In generated files (`main.c`, `main.h`, `stm32f4xx_it.c`,
`stm32f4xx_hal_msp.c`), only write code between `USER CODE BEGIN` and
`USER CODE END`. The pins are set in `stm32-car.ioc`.

## Pins

| Pin | Morpho | Name in code | Goes to |
|---|---|---|---|
| PA0 | CN7-28 | `ENC_A` | Encoder A (yellow) |
| PA1 | CN7-30 | `ENC_B` | Encoder B (green) |
| PA4 | CN7-32 | `BATT_SENSE` | Battery divider (see [Battery reading](#battery-reading)) |
| PB0 | CN7-34 | `START_BTN` | Start button (other leg to GND) |
| PA7 | CN10-15 | `SONAR_ECHO` | HC-SR04 ECHO, through the 1k/2k divider |
| PB6 | CN10-17 | `SONAR_TRIG` | HC-SR04 TRIG |
| PA9 | CN10-21 | `BT_TX` | Bluetooth RXD |
| PA8 | CN10-23 | `MOTOR_ENA` | L298N ENA (jumper removed) |
| PB10 | CN10-25 | `SERVO_PWM` | Servo signal (orange) |
| PB4 | CN10-27 | `MOTOR_IN1` | L298N IN1 |
| PB5 | CN10-29 | `MOTOR_IN2` | L298N IN2 |
| PA10 | CN10-33 | `BT_RX` | Bluetooth TXD |
| PB8 | CN10-3 | `I2C_SCL` | MPU-6050 SCL and OLED SCK/SCL |
| PB9 | CN10-5 | `I2C_SDA` | MPU-6050 SDA and OLED SDA |

Power the OLED from 3.3 V (VDD/VCC) and GND. The OLED (0x3C) and the
MPU-6050 (0x68) share the I2C1 wires, so `I` should list both. The gyro
is read for the heading hold; the car must stand still for a second after
power-on while it measures the gyro's zero.

Hardware: 25GA370 12 V motor with gearbox and encoder, L298N driver, MG996R
steering servo, HC-SR04, MPU-6050, BT04-A Bluetooth module, 3S LiPo, an
XL4015 buck converter for the servo.

## Commands (USB 115200 / Bluetooth 9600, line ending LF)

Each command is one or two letters, followed by an optional number. `d1.2`
and `D 1.2` mean the same thing.

| Command | Does |
|---|---|
| `d1.2` | Distance run of 1.2 m: 18 cm/s, 2 s ramps, position PID. With `tt` set: in that time. `d-0.5` runs in reverse |
| `tt 8` | Run time 8 s (`tt 0` = back to the nominal speed) |
| `ta 2` / `td 2` | Acceleration / deceleration time in s |
| `vn 18` | Nominal speed in cm/s, for runs without a set time |
| `tol 1` | A run is done when the car stands within this many cm of the target |
| `kp 3` / `ki 2` / `kd 0` | PID gains |
| `ms 60` / `mo 45` / `mg 0.59` | Motor: % needed to start moving / offset once rolling / % per cm/s |
| `vb 11.6` | The battery voltage on a multimeter: calibrates the battery reading |
| `v18` | Hold a speed in cm/s (speed loop) |
| `sp 1` / `si 1.5` | Speed loop gains |
| `pf 100` | Motor PWM frequency in Hz (100 is the default; 5000 barely moves the car) |
| `kh 15` | Heading hold: µs of steering per degree off course, `kh 0` = off, negative flips it |
| `sc 1080` | Servo pulse that drives straight |
| `Y` | Zero the heading |
| `cal` | Steering calibration: the car drives about 30 s in gentle to tight curves and prints the turn radius for nine servo pulses. Needs a clear strip 5 m long and 2 m wide. `s` stops it |
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

Values set with commands are lost at reset. Copy the good ones into
`car_config.c` (the PID gains are at the top of `car_drive.c`).

**Start buttons.** Press the blue button (B1) or the start button on PB0.
The car does a 1 m run 1 s later. Any other drive command cancels it.

**CubeMonitor.** Write a distance in metres to `test_dist_m` to start a run.
You can also tune `Kp`, `Ki` and `Kd` live, and watch `Pos_Ref`,
`Real_Dist`, `SP_Vel`, `AverageSpeed`, `state`, `sonarCm` and `obstacle`.

## How a distance run works

Every 100 ms the TIM3 interrupt runs one control step (`drive_update()` in
`car_drive.c`):

1. **Speed profile.** Accelerate, cruise at the nominal speed Sn,
   decelerate. `SP_Vel` is the speed the car should have now. `Pos_Ref` is
   the area under the profile so far: where the car should be now.
2. **Position PID.** The error is `Pos_Ref − Real_Dist` (encoder). The
   command in cm/s is `SP_Vel + Kp·e + Ki·∫e + Kd·de/dt`. `SP_Vel` is the
   feed-forward, so the PID only corrects what is left.
3. **Speed loop.** Motor % = `MOTOR_MIN_PCT` + command × `MOTOR_PCT_PER_CMS`
   as a first guess, then corrected by a PI on the measured speed (`sp`,
   `si`). On the car the same motor % gave anything from 2 to 20 cm/s, so
   the first guess alone cannot hold a speed.

Four things keep the car from stalling, overshooting and hunting:

- **Start.** A standing car needs more power than a rolling one. So a
  standing car gets at least `MOTOR_START_PCT` (30 % at 100 Hz PWM) and
  25 % more for every second it still stands. Before, the car waited for the integral to grow,
  which took seconds on short runs.
- **Anti-windup.** The integral only grows while the motor can follow the
  command: not while the car still stands, not at 100 %, not during an
  obstacle pause. The I term is also limited to `PID_I_MAX_CMS` (10 cm/s).
- **No pushing against the run.** During the profile, a car that is ahead
  of the reference coasts or brakes until the reference catches up.
- **Settle.** When the profile ends, the car closes the remaining error
  with P and D only, in either direction, and stops within `POS_TOL_CM`
  (1 cm). It changes direction at most once, so it cannot jitter around
  the target.

When a run ends the car prints how it went, for example
`DONE: 100.3 cm, past by 0.3 cm`. The run also stops by itself if the motor
is on for 2.5 s without encoder counts, or if the encoder counts the wrong
way.

**Timed run.** With `tt` > 0, `d` covers the distance in exactly `tt`
seconds with ramps `ta` and `td`: Sn = D / (T − (ta + td)/2). The car
refuses runs that need more than 80 cm/s or where tt < ta + td.

**Obstacle stop.** TRIG gets a 10 µs pulse every step. ECHO (PA7) has an
interrupt on both edges, timed with TIM3's 10 µs counter: distance = echo
µs / 58. Two readings under 30 cm in a row brake the car and freeze the
profile clock and `Pos_Ref`. The run continues once the reading is over
35 cm. The sonar looks forward, so reverse runs ignore it.

## Tuning on the car

1. Send `T 10` for telemetry every 100 ms. With `tools/car_console.py` it
   is saved to `tools/logs/` automatically.
2. Run `d1`, then plot the log: `python tools/plot_run.py tools/logs/run_….csv`.
   The plot shows reference and measured distance, the error, the speed and
   the motor %.
3. **Start.** If the car jumps at the start, lower `ms`. If the motor hums
   for a moment before the car moves, raise it.
4. **Cruise.** If the measured distance stays behind the reference during
   the cruise, raise `mo` or `mg`. If it runs ahead, lower them. A good
   feed-forward leaves little for the PID to do.
5. **Gains.** Raise `kp` until the error stays small without the speed
   starting to swing, then set `ki` to remove what is left during the
   cruise. Add a little `kd` (0.1–0.3) if the stop overshoots.
6. Copy the values into `car_config.c` and `car_drive.c`.

You can try settings on a PC first. `tools/drive_sim` runs the real
`car_drive.c` against a simple model of the car:

```
gcc -O2 -Itools/drive_sim -ICore/Inc tools/drive_sim/drive_sim.c \
    Core/Src/car_drive.c Core/Src/car_config.c -lm -o tools/drive_sim/drive_sim
tools/drive_sim/drive_sim d=0.1 kp=3 ki=2 csv=run.csv
python tools/plot_run.py run.csv
```

The model is a guess, not a measurement of our car. Use it to compare
settings, not to predict centimetres.

## Battery reading

The car showed 7.1 V when a multimeter on the battery showed 11.6 V. The
firmware assumed the divider from the wiring guide: 10k from battery + to
PA4 and 3.3k from PA4 to GND, a factor of 4.03. A reading of 7.1 V means
1.76 V on PA4, so the real factor is 11.6 / 1.76 = 6.58. That fits a 1.8k
resistor in the place of the 3.3k (factor 6.56).

`BATT_DIVIDER` is now 6.58, so the reading should be right. To be sure:

1. Measure PA4 (CN7-32) to GND with the multimeter. About 1.76 V at 11.6 V
   confirms the resistors are 10k / 1.8k. About 2.9 V would mean the
   resistors are as drawn and the problem is somewhere else.
2. `G` shows the PA4 voltage and the factor: `batt=11.60V (PA4 1.76V x 6.58)`.
3. `vb 11.6` (the multimeter value) sets the factor exactly. Copy it into
   `car_config.c`.

PA4 must stay below 3.3 V at a full battery (12.6 V). With 6.58 it reaches
1.9 V, which is safe.

**Low battery.** The battery is a 3S LiPo: 12.6 V full, and it must not go
below 9.0 V (3.0 V per cell), or the cells are damaged. On 2026-11-12 it
was run down to 7.9 V without anyone noticing. The car now warns below
9.9 V: "LOW BAT" blinks on the OLED and a `WARNING battery low` line is
printed every 30 s. It does not stop by itself, so stop and charge when
you see it. At 7.9 V the display showed 6.7 V, so at low voltage the
reading is still not right (or the battery sagged under load); the warning
comes early rather than late.

## Check after flashing

1. `f` must drive forward. If it goes backwards, set `MOTOR_INVERT` to 1 in
   `car_config.h`.
2. While driving forward, `G` must show the counts going up. If they go
   down, set `ENCODER_INVERT` to 1. This matters now: with the wrong sign a
   run stops after 3 cm with `ERR encoder counts the wrong way`.
3. `G` shows the battery voltage. Compare it with a multimeter (`vb`).
4. Run `d1` and measure how far the car actually went. If the car is right
   about its own distance but the tape disagrees, correct the counts per
   metre with `M`.
5. Run `d0.1` and `d0.05`. The car should start at once and stop within
   about 1 cm.

## Laptop tools

```
pip install pyserial matplotlib
python tools/car_console.py                 # lists ports
python tools/car_console.py COM5            # USB (115200)
python tools/car_console.py COM7 9600       # Bluetooth COM port
python tools/plot_run.py tools/logs/run_XXXX.csv
tools/build_check.sh                        # compile from a terminal (Linux/macOS)
```

Over Bluetooth, use `T 5` rather than `T 10`: 9600 baud is slow.

`tools/autopilot/` holds the path builder and the simulator of the old
autopilot. They need the old firmware sources (below) and do not work with
the current firmware.

## Old autopilot code

Before October the repository held a different firmware (`Team7_Car` in
`car_project/`), with gyro odometry, a recorded route followed by pure
pursuit, waypoint stops and driving around obstacles. It was replaced by
this project, which we built up step by step through the class activities.
The old code was never tuned on the car, but it is the starting point for
the parts we still need. It is in the Git history:

```
git show 77118af:car_project/cubeide/Team7_Car/Core/Src/autopilot.c
git show 77118af:car_project/cubeide/Team7_Car/Core/Src/path.c
git show 77118af:car_project/cubeide/Team7_Car/Core/Src/car_hw.c     # MPU-6050 driver
git show 77118af:car_project/README.md                               # how it was used
```

## Where the code came from

- **`hakon-backup`** (Hakon): the speed profile, the first PID, the
  Bluetooth commands, steering and start buttons, tuned on the car. The
  variable names are still his, so his CubeMonitor setup works.
- **`activity2.5`, `activity2.6`, `oled`** (Filip): the base project with
  motor and encoder, the OLED, and Hakon's code moved into `car_drive.c`.
- **`oled-timer-pid-timed-obstacle-bluetooth`** (Sondre): the TIM3 control
  interrupt, the position loop, timed runs and the obstacle stop.
- **The old `main`** (Sondre): the wiring guide, CAD and laptop tools.
