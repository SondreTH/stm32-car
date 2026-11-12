# Open problems and what is next

Last updated 2026-11-11.

## State on 2026-11-12 (evening)

Flashed and seen working on the car: battery reading (11.5 V), commands,
telemetry, encoder direction (`ENCODER_INVERT 1`), `pf`.

Seen NOT working: distance runs at 5 kHz PWM (see "What the car showed" in
[control.md](control.md)).

Written after that, only compiled and simulated, **not yet on the car**:
100 Hz PWM default, the speed loop, the gyro heading hold. First things to
do with it:

1. Power on with the car standing still. The start message must say
   `gyro OK`. `G` shows `yaw=`; turn the car left by hand: yaw must go up.
   If it goes down set `IMU_YAW_SIGN -1.0f`; if it does not change, the
   module is not lying flat: change `IMU_YAW_AXIS`.
2. `kh 0`, `T 10`, `v18` for a few metres, `s`. The speed should stay near
   18. Tune `sp` / `si` if it surges or crawls.
3. `kh 15`, `d2`. Watch the front wheels: if the car drifts and they steer
   further the same way, use `kh -15` (and then flip `IMU_YAW_SIGN`).
4. `d0.1`, `d0.05`, `d-0.3`.

Open: the tight spot in the drivetrain (mechanical).

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
