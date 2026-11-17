# Open problems and what is next

Last updated 2026-11-12.

## State after the car sessions of 2026-11-12

Seen working on the car (logs `run_20261112_1632*` and `_1634*`):

- Speed loop + 100 Hz PWM + heading hold: 2 m and 5 m runs ended within
  1 cm of the target, started within 0.2-0.4 s, average cruise speed
  17.2-18.0 cm/s for a plan of 18.
- Heading hold: the sign is right (`IMU_YAW_SIGN 1`, axis Z). Four runs
  stayed within 1-2.5 degrees.
- Battery reading at 11.6 V, commands, telemetry, encoder direction.

Open problems seen in the same logs:

- **Speed swings.** In cruise the speed swings between about 10 and 27
  cm/s with a steady rhythm of about 40 cm of travel (2.1-2.4 s). Not known
  yet whether that is the tight spot in the drivetrain or the two loops
  hunting. Tests that tell: the `cal` drive (speed loop alone), and a run
  at another speed (`vn 40`): a rhythm that stays 40 cm is mechanical, one
  that stays 2.3 s is the controller (then lower `kp` / `ki`).
- **Heading creep.** In two of the 5 m runs the heading error grew slowly
  to +5 degrees while the servo went 70-90 us to the right. A P-only hold
  cannot remove an error that needs a constant steering offset. It looks
  like play in the steering (the servo moves, the wheels follow late); the
  `cal` drive measures that. An integral term in the heading hold is the
  likely fix.
- **Battery.** The 3S LiPo was run down to 7.9 V (multimeter) during the
  session, far below the 9.0 V limit. The OLED showed 6.7 V at the time, so
  the reading is off at low voltage or the pack sagged under load; see
  [hardware.md](hardware.md). A low-battery warning now exists.
- The tight spot in the drivetrain is still there (mechanical).
- `sc 1080` drove straight by eye (default now). The logs cannot confirm
  it: the heading hold was on in every logged run, and it hides the trim.

Written after those sessions, compiled only, **not yet on the car**: the
`cal` steering calibration drive, the low-battery warning, `batt_v` in the
telemetry.

Next session on the car (battery charged):

1. `G` with a multimeter on the battery and on PA4 at the same moment.
2. `T 10`, `cal`: needs a clear strip 5 m long and 2 m wide. Prints a turn
   radius for nine servo pulses. `s` stops it.
3. `vn 40`, `d5`: a faster run. 150 m at 18 cm/s would take 14 minutes.

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
- Braking (`motor_brake`) is assumed to stop the car quickly. If the car
  skids or the gearbox complains, replace the brake in settle with coasting.

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
   heading hold), untested. Was: read Z rate, calibrate bias at standstill, integrate
   to a heading. The sensor is wired on I2C1 and answers at 0x68. Note the
   OLED update blocks the bus for ~25 ms; reads must fit around it (or the
   OLED moves to a slower, split update).
2. **Odometry**: x/y from encoder distance and heading.
3. **Steering as an angle**: the servo only has left / centre / right now.
   Needs µs per degree and the wheelbase.
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
- Settings are lost at reset. Saving them to flash would spare retyping.
- `tools/autopilot/` does not build against the current firmware.
- The wiring guide still says "firmware v1.3" and lists the battery
  divider as 10k / 3.3k.
