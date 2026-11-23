# To do: next sessions

Written 2026-11-22, after the open-loop session. Tick things off here and
move what was learned into the other docs. Background for every item is in
[openloop.md](openloop.md), [roadmap.md](roadmap.md) and
[hardware.md](hardware.md).

## 1. Before the car drives again

- [ ] Charge the battery (3S LiPo, never below 9.9 V; the car warns).
- [ ] `git pull`, Refresh in CubeIDE, build, **flash**. The car still has the
      test build of 2026-11-22 (stop-and-go plan as default); the repo has
      the square that worked.
- [ ] `G`: battery about right, `reads ok` counting, `failed=0`.
- [ ] `Y`, turn the car 90 degrees left by hand, `G`: `x=` about +90. If
      not, the gyro has been moved (see [hardware.md](hardware.md#i2c1)).
- [ ] Bluetooth from the laptop: `python tools/car_console.py
      98:DA:20:0C:1A:63` (paired already on Filip's laptop, PIN 1234).
      Remember: unplugging the USB resets the car, so unplug first, then
      send commands over Bluetooth.

## 2. Open-loop video (individual, 2-3 minutes)

The assignment: explain open loop, nonlinear behaviour and disturbances,
with several scenarios. The car must drive a geometric shape and get back
to where it started. Footage of the tuning runs exists (the last one,
`rec8`, was filmed).

- [ ] Check the default square (`og` or the blue button). It ends with a
      0.2 s straight, a guess between 0 s (30 cm short) and 0.45 s (30-40 cm
      past). Adjust step 10 (`ol 10 40 -50 <s>`) until it stops on the
      mark, then copy the time into `olPlan[]` in `car_openloop.c`.
- [ ] Film the scenarios. Same plan every time, change one thing, measure
      the end point with a tape, `od` after each run (the console saves it):
  - [ ] the plan 3 times as it is (how far apart do the end points land?)
  - [ ] `ov 10`: behaves like the same plan on a 10 V battery
  - [ ] extra weight on the car (a phone, a bottle)
  - [ ] a push sideways during a side
  - [ ] start rotated about 10 degrees, or 20 cm off the mark
  - [ ] no kick (`ol 1 0 0 0.3`) or 35 % instead of 40 % (dead zone)
  - [ ] another floor (rug, tiles), if there is one
- [ ] Plots: `python tools/plot_openloop.py --title="..." "name=tools/logs/ol_....csv" ...`
      gives the paths from above and motor % / speed / heading over time,
      1920x1080. The path is an estimate (25-50 cm off after 8-9 m); say
      the tape numbers in the video.
- [ ] Script: what open loop is (a plan decided in advance, no sensor in
      the loop), how the plan was built (kick, more % in turns, −50 µs for
      the steering play, timing from measured runs), the nonlinear
      behaviour (dead zone, static vs rolling friction, wheel spin, weak
      brake, slow speed response, radius that grows with speed, steering
      play), the disturbances (battery, floor, tight spot, pushes), the
      scenarios, and why the real circuit needs feedback. All numbers are
      in [openloop.md](openloop.md#what-the-car-taught-us-material-for-the-video).

## 3. Check on the car what changed on 2026-11-22

- [ ] **Heading hold with the right gyro axis.** It never ran before.
      `kh 15`, `d2` at the normal 18 cm/s, watch it. If it weaves, lower
      `kh` (try 8). If it steers away instead of back, stop and check the
      signs (`IMU_YAW_SIGN`).
- [ ] `T 5`, `cal`: the steering table, turn radius for nine servo pulses.
      Needs 5 m x 2 m. Only possible since the gyro reads the right axis.
- [ ] A longer straight with the hold (`d5`): does it end straight, or does
      the steering play leave a constant offset? (Then the hold needs an
      integral term.)

## 4. Firmware for the full circuit

Open loop cannot do 150 m: the same timed turn gave 73 to 109 degrees.
Feedback on the heading and the distance is needed. In this order:

- [ ] **Turn by gyro.** Drive an arc at a set steering pulse and speed
      (speed loop) until the heading has changed by N degrees, then
      straighten. Start straightening a little early: the car keeps turning
      while the servo moves back, and the steering play leaves the wheels
      off centre after a turn (−50 µs fixed it in open loop). Filip saw the
      car overturn slightly and then curve back on the straights, so check
      the angle on video too.
- [ ] **Heading as a target** that code can change (now it is "the heading
      when the run started"), with an integral term against the play.
- [ ] **A route**: a list of straights (distance runs with the heading
      hold), gyro turns and 10 s waypoint stops, run with one command or
      the button. Trim the gyro bias at every stop. The open-loop plan
      (`car_openloop.c`) is a model for the list, the commands and the
      recording.
- [ ] **Sonar stop that depends on the speed.** 30 cm is too late above
      about 60 cm/s, and the brake is weak (40-80 cm to stop).
- [ ] Obstacle avoidance, x/y position: once the course is known.

If the circuit test comes before the gyro turns work, the fallback is an
open-loop plan tuned on the spot (`ol` steps over Bluetooth). Expect it to
drift on a long course.

## 5. Questions for the course

Still not known (first asked on 2026-11-12):

- [ ] Shape of the circuit, and how long the straights and turns are.
- [ ] Time limit.
- [ ] How the three waypoints are marked, and how exact the stop must be.
- [ ] What the obstacles are, and where.
- [ ] How close to the start the car must end.

## 6. Smaller things

- [ ] Battery reading: multimeter on the battery and on PA4 (CN7-32) at the
      same time as `G`, at two voltages ([hardware.md](hardware.md#battery-reading-known-problem)).
- [ ] Save settings and the open-loop plan to flash, so a reset (or
      unplugging the USB) does not lose them.
- [ ] A watchdog for manual driving over Bluetooth (stop if no command for
      a few seconds).
- [ ] Push `integration` and make the PR (Filip). Commits of 2026-11-22:
      `7cd75bf` (firmware, tools), `d10a5fd` (docs, runs), and this file.
