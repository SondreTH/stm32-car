# Open problems and what is next

Last updated 2026-11-22.

## State after the open-loop session of 2026-11-22

Done and seen working on the car (details in [openloop.md](openloop.md)):

- **Open-loop route** (`car_openloop.c`): a square of four ~1 m sides and
  four right turns drove back to within 30-40 cm of the start, heading
  within 1 degree, from a plan of timed steps with no feedback.
- **Gyro axis fixed.** The MPU-6050 stands on its side; the code read Z,
  which never saw a turn. Now X (`IMU_YAW_AXIS 0`). Checked by hand (90
  deg left = +90) and in every open-loop run (four turns add up to the
  -359 deg the eye saw).
- **Bluetooth works both ways.** Output to the module now goes in small
  pieces (it lost the end of every line before). The laptop console
  connects by address on Linux. Runs can be started and read without the
  USB cable, which matters because unplugging the USB resets the STM32.
- Gyro readings: none failed during the runs, also with the motor on.

Learned about the car (all in [hardware.md](hardware.md) and
[openloop.md](openloop.md)):

- The L298N brake barely brakes. Stopping takes 1-2 s and 40-80 cm.
- On a full pack the car is much faster than the 2026-11-12 numbers
  (taken on a nearly empty pack): 45 % up to 140 cm/s.
- The steering has play: about +50 µs after a right turn.
- +350 µs gives a turn radius of about 65 cm.
- From standstill with the wheels at full lock the car does not start at
  35 %; a hard kick spins the rear wheel.
- The sonar stop (30 cm) is too late above about 60 cm/s: the car rolled
  into a hand at ~120 cm/s.

**What this means for the 2026-11-12 results:** the heading hold never saw the
heading (wrong axis). It steered on pitch/roll drift instead, so "four
runs stayed within 1-2.5 degrees" and the "heading creep" (servo 70-90 µs
right) are not about the heading at all. The straight runs were straight because
`sc 1080` was close. The speed-loop and distance results still hold.

Not run on the car yet:

- The heading hold with the right axis (`d2` first, at 18 cm/s; see
  [control.md](control.md#heading-hold-car_headingc)).
- `cal` (it needs the gyro, which only works since 2026-11-22).
- The default plan in `car_openloop.c` (the working square with the last
  step at 0.2 s, interpolated between 0 s and 0.45 s). Battery compensation
  (`ov`). The open-loop scenarios for the video (list at the end of
  [openloop.md](openloop.md)).
- From the 2026-11-12 list: speed swings in cruise, battery reading at low
  voltage.

The checklist for the next sessions is in [todo.md](todo.md).

## Plan towards the circuit

Straight runs work. Next is turning, in this order:

1. Steering table from `cal`: servo pulse -> turn radius, real centre,
   play.
2. Heading as a target that code can change (now it is "whatever the
   heading was when the run started"), with an integral term.
3. A turn: drive an arc until the gyro reaches the new heading.
4. A route: a list of straights, turns and 10 s waypoint stops, run with
   one command. The gyro bias can be trimmed again at every stop.
5. x/y position and obstacle avoidance, as far as the course needs them.

Not known yet (asked the team on 2026-11-12): the shape of the circuit,
time limit, how the waypoints are marked, what the obstacles are, how
close to the start the car must end.

## Not verified on the car (list from 2026-11-11)

The `integration` branch was built and checked without the car.

- **It compiles** (`tools/build_check.sh`, no warnings in `car_*.c`).
- **The position loop was only run in `tools/drive_sim`**, against a
  guessed model of the car. Kick-start, anti-windup, settle, reverse runs
  and the safety stops have never run on the real motor.
- The OLED, commands and telemetry changes are small but also unflashed.

First session on the car, in this order:

1. `G`: check the battery line. Measure PA4 (see below).
2. `f`, then `G`: counts must go **up** when driving forward. If not, set
   `ENCODER_INVERT 1`. With the wrong sign, runs now stop after 3 cm.
   (Done 2026-11-12: the encoder did count backwards, `ENCODER_INVERT` is now 1.)
3. `T 10`, `d1`, plot with `tools/plot_run.py`. Look at the start (kick),
   the cruise (does the distance follow the reference?) and the stop.
4. `d0.1`, `d0.05`: the short runs that used to stall.
5. `d-0.3`: a reverse run.
6. Tune `ms`, `mo`, `mg`, then `kp`, `ki`, `kd` (README, "Tuning on the
   car"), and copy the values into `car_config.c` / `car_drive.c`.

Things that may need changing after that:

- `MOTOR_START_PCT` 60 is one measurement. If the kick makes the car jump,
  lower it; the kick grows by itself if it is too low.
- `POS_TOL_CM` 1 cm may be tighter than the smallest step the car can make
  from standstill. If runs end with "stopped after one swing back" a lot,
  raise it to 1.5–2.
- Braking (`motor_brake`) was assumed to stop the car quickly. It does
  not (2026-11-22, [hardware.md](hardware.md)): it is close to coasting.

## Battery reading

`BATT_DIVIDER` was changed from 4.03 to 6.58 from one pair of readings
(7.1 V shown, 11.6 V measured). The cause is not confirmed. Measure PA4 to
GND and read the two resistors on the perfboard. Details in
[hardware.md](hardware.md). If the resistors get replaced with the 10k /
3.3k from the wiring guide, set `BATT_DIVIDER` back to 4.03.

A low-battery warning for the 3S LiPo (below about 9.9 V) would be worth
adding once the reading can be trusted.

## Missing for the final challenge

The challenge: about 150 m circuit, three 10 s waypoint stops, obstacles,
finish at the start. What the firmware lacks, roughly in order:

1. **MPU-6050 gyro**: done in `car_heading.c` on 2026-11-12 (heading and
   heading hold); the axis was wrong until 2026-11-22, the heading is right
   now, the hold is untested. Was: read Z rate, calibrate bias at standstill, integrate
   to a heading. The sensor is wired on I2C1 and answers at 0x68. Note the
   OLED update blocks the bus for ~25 ms; reads must fit around it (or the
   OLED moves to a slower, split update).
2. **Odometry**: x/y from encoder distance and heading.
3. **Steering as an angle**: the heading hold and the open-loop plan set any
   pulse, but there is no pulse -> angle table. One point is known: +350 µs
   = radius ~65 cm. `cal` measures nine.
4. **Speed control while steering**: runs are straight lines with a fixed
   profile. Route following needs a speed setpoint that can change (a
   speed loop, or chained position profiles).
5. **Route**: record by driving (teach) or build from measurements; follow
   it (pure pursuit); stop at waypoints for 10 s.
6. **Obstacle avoidance**: now the car only stops and waits. Driving around
   needs reversing (Ackermann steering), which the position loop supports
   since 2026-11-11 (`d-0.3`).

The old `Team7_Car` firmware had a version of all six. See
[history.md](history.md) for how to read it. Port it piece by piece into
the current structure and units (cm), testing each piece on the car, as the
team did with the activities.

## Smaller things

- Manual drive commands (`f`, `b`, `P`, `v`) have no timeout. If Bluetooth
  drops, the car keeps driving. A watchdog ("stop if no command for N s")
  existed in the old firmware (`W`).
- Settings are lost at reset, and unplugging the USB resets the board.
  Saving them (and the open-loop plan) to flash would spare retyping.
- The sonar stop distance is fixed at 30 cm; at higher speeds it should
  grow with the speed (the brake is weak).
- `tools/autopilot/` does not build against the current firmware.
- The wiring guide still says "firmware v1.3" and lists the battery
  divider as 10k / 3.3k.
