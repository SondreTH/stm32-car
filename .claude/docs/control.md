# The distance run (`car_drive.c`)

Everything here runs in `drive_update()`, called from the TIM3 interrupt
every 100 ms (`DRIVE_PERIOD_MS`). Units: cm, cm/s, s. Forward is positive.

## States

`state` is watched from CubeMonitor and shown on the OLED.

| Value | Name | Meaning |
|---|---|---|
| 0 | `DRIVE_OFF` | no run |
| 1 | `DRIVE_ACCEL` | profile: speed ramps up to Sn (until `t1`) |
| 2 | `DRIVE_CRUISE` | profile: Sn (until `t2`) |
| 3 | `DRIVE_DECEL` | profile: speed ramps down to 0 (until `T`) |
| 4 | `DRIVE_SETTLE` | profile finished, the car closes the last error |

## Profile

Two ways to set it up, both from `drive_start(dist_m)`:

- `Start_Motion_Profile(dist, Sn, ramp)`: used when `run_T == 0`. Nominal
  speed `DRIVE_SPEED_CMS` (18 cm/s), ramps `DRIVE_RAMP_S` (2 s). If the
  distance is shorter than the two ramps it becomes a triangle with a lower
  peak speed.
- `Start_Timed_Profile(dist, T, ta, td)`: used when `run_T > 0` (the `tt`
  command). Sn = D / (T − (ta + td)/2). Refused if `T < ta + td` or
  Sn > `DRIVE_MAX_CMS` (80).

`Sn`, `t1`, `t2`, `T` are always positive. The direction is in `runDir`
(+1 / −1, from the sign of the distance). `profile_at(time_ctl)` returns the
speed and the exact area under the curve so far; `SP_Vel` and `Pos_Ref` are
those times `runDir`. `Pos_Ref` ends exactly on `target_dist_cm`.

## Control law

During states 1–3:

```
error   = Pos_Ref - Real_Dist
command = SP_Vel + Kp*error + Ki*ErrorAcum + Kd*ErrorRate        (cm/s)
          (the PID part is limited to +-PID_OUT_MAX_CMS, 12 cm/s)

speed loop, in Apply_Motor_PWM (since 2026-11-12):
ev      = |command| - speed in that direction
motor % = MOTOR_MIN_PCT + |command| * MOTOR_PCT_PER_CMS + Kvp*ev + SpeedAcum
SpeedAcum += Kvi * ev * dt
```

`SP_Vel` is feed-forward for the position loop, and the motor line
(`MOTOR_MIN_PCT`, `MOTOR_PCT_PER_CMS`) is feed-forward for the speed loop.
Defaults: Kp 3, Ki 2, Kd 0 (Hakon's values), Kvp 1.0, Kvi 1.5, motor line
22 % + 0.5 %/(cm/s) at 100 Hz PWM.

## What the car showed on 2026-11-12 (logs in `tools/logs/`, not in git)

First runs on the real car. They are the reason for the speed loop.

- At the CubeMX PWM frequency of 5 kHz the car did nothing up to 70-80 %
  and then ran away to 80-90 cm/s. PWM is on the L298N enable pin, so the
  motor coasts between pulses; short pulses give almost no torque at
  standstill. Hakon's line (45.1 % + 0.59 %/(cm/s), start at 60 %) did not
  hold: a 1 m run stood for 2 s, shot to 62 cm/s and ended 6.4 cm too far.
- Open-loop steps at other frequencies (`pf`): 500 Hz was as bad as 5 kHz.
  At 100 Hz: 40 % gave 25-40 cm/s, 50 % 54 cm/s, 70 % 93 cm/s, started
  within 0.1 s. At 200 Hz in between. 100 Hz is now the default
  (`MOTOR_PWM_HZ`).
- Even at constant % the speed surged (30 % at 50 Hz: 2..20 cm/s and back).
  The team confirmed a tight spot in the drivetrain by hand. Not fixed yet.
- With the motor off the car stops by friction at roughly 90 cm/s2 (rolled
  16 cm from 54 cm/s, 32 cm from 93 cm/s).

So the same % gives very different speeds: only a loop on the measured
speed can hold 18 cm/s. **The speed loop and the 100 Hz default have only
run in `drive_sim` so far, not on the car.**

## Why the extra logic exists

The motor has stiction: measured on the car, about **60 %** is needed
before it moves, but once rolling it keeps moving with less (the 45 %
offset). The old loop had only the 45 % map, so at the start of a run the
command stayed under 60 % and the car stood still until the P and I terms
had grown enough. On short runs that took most of the profile, and the
stored integral then pushed the car past the target; very short runs never
moved at all. (In `drive_sim` with the default model: the old code started
after 1.1 s, a 10 cm run ended 6 cm too far, and a 5 cm run did not move.)

The paragraph above is the history of the first version (5 kHz PWM, 60 %
start). The numbers changed on 2026-11-12, the idea did not.

### Start and speed loop (`Apply_Motor_PWM`)

"Standing" means the last step's speed was under `DRIVE_MOVING_CMS`
(1.5 cm/s = fewer than 2 encoder counts in 100 ms). A standing car that is
commanded to move gets at least `MOTOR_START_PCT` (30 %). For every step it
still stands, `SpeedAcum` grows by at least `KICK_RAMP_PCT_S` (25 % per
second), which covers a tight spot, a low battery or a rough floor. Once
it moves, the PI on the speed error takes over: too fast lowers the motor
%, down to 0 (coast; it never drives backwards to slow down), too slow
raises it. `SpeedAcum` is limited to +-`SPEED_I_MAX_PCT`, stops growing at
0 % and 100 %, and is cleared whenever the motor is let go.

### Anti-windup (`ControlPID`)

The integral `ErrorAcum` only grows when the motor can act on it:

- not while the car stands and is being kicked (`intFreeze`),
- not while the motor is at ±100 % and the error asks for more (`outSat`),
- not during an obstacle pause (the whole step is skipped),
- not while it brakes to change direction.

It may always shrink (error and integral of opposite sign). The I term
`Ki*ErrorAcum` is limited to ±`PID_I_MAX_CMS` (10 cm/s) whatever `Ki` is.
The integral is cleared at the start of a run, on entering settle, and when
a run ends.

### Never drive against the run (states 1–3)

If the command points against `runDir` (the car is ahead of the reference),
the motor is not reversed. It brakes if the car is more than `POS_TOL_CM`
ahead and still rolling, otherwise it coasts, until the reference catches
up. Without this, the offset in the motor map would make the output jump
between +45 % and −45 % around zero command.

Commands smaller than `DRIVE_CMD_MIN_CMS` (1 cm/s) coast for the same reason.

### Settle (state 4)

After `T` the reference stays on the target. The loop becomes P + D only
(the integral is held at zero: with stiction an integral makes the car
creep, jump past, creep back, forever). Each step:

1. `|error| <= POS_TOL_CM` (1 cm): brake while still moving; when it
   stands, the run ends with `RUN_DONE`.
2. Longer than `DRIVE_SETTLE_S` (3 s) in settle: end with `RUN_TIMEOUT`.
3. Standing outside the tolerance: drive towards the target (kick-start
   applies). Each change of drive direction is counted; the second one
   ends the run with `RUN_SWINGS` (`DRIVE_MAX_REVERSALS` = 1). So after an
   overshoot the car backs up once and then stays, whatever happens.
4. Rolling outside the tolerance: P + D command; if it points against the
   motion the motor brakes first (`Apply_Motor_PWM` never reverses a
   rolling car).

The team asked for this behaviour: reversing is allowed (and will be
needed for obstacle avoidance with Ackermann steering), but at most one
overshoot swing and no jitter around zero error.

## Obstacle pause

Every step: `sonarCm < OBST_STOP_CM` (30) twice in a row sets `obstacle`;
`sonarCm > OBST_CLEAR_CM` (35) clears it. While set, a **forward** run
brakes and returns before the profile clock advances, so `time_ctl` and
`Pos_Ref` freeze and the run continues where it was. Reverse runs ignore
the sonar (it looks forward).

## Safety stops

| Result | Condition | Message |
|---|---|---|
| `RUN_ENC_DIR` | in states 1–3 the car is more than `ENC_FAULT_CM` (3 cm) on the wrong side of the start | `ERR encoder counts the wrong way` |
| `RUN_STALL` | the motor was driven for `DRIVE_STALL_S` (2.5 s) while the car stood | `ERR motor on but no encoder counts` |

`RUN_ENC_DIR` exists because the loop is now signed: with an inverted
encoder the error would grow as the car drives and the motor would go to
100 %. `RUN_STALL` also covers an unplugged encoder. Both also apply to the
open-loop `v` command (stall only).

## Holding a speed (`v` command)

`Set_Speed_cm_s(v)` stores `openLoopCms` (the name is from before the
speed loop). While no run is active, `drive_update()` applies it through
`Apply_Motor_PWM` every step, so `v18` holds 18 cm/s with the speed loop.
It is the easiest way to tune `sp` / `si`: `T 10`, `v18`, look at the speed.

## Heading hold (`car_heading.c`)

Runs in the main loop, not in the interrupt (the OLED uses the same I2C
bus from the main loop). Every 5 ms or more it reads the MPU-6050 gyro and
adds up `yawDeg += (rate - bias) * dt`, with dt measured, so the 25 ms OLED
update does not matter. Positive yaw = turned left.

- Bias: 200 readings at start-up (0.6 s, car must stand still), then
  trimmed slowly while the car stands with the motor off and nobody turns
  it (under 1 deg/s).
- When a run starts (`state` leaves 0) the heading is remembered. During
  the run: `servo = SERVO_CENTER_US + dir * STEER_KH * headingErrDeg`,
  limited to the servo range; `dir` is -1 for a reverse run. When the run
  ends the servo goes back to centre.
- `kh 0` turns it off. `sc <us>` trims the centre pulse. If the gyro stops
  answering, the hold stops and the servo centres; `I` tries again.
- **Not run on the car yet.** Unknown until then: the mounting of the
  module (`IMU_YAW_AXIS`, `IMU_YAW_SIGN`) and whether 15 us/deg is a good
  gain. `drive_sim` does not model steering.

## Checking a change: `tools/drive_sim`

`drive_sim.c` compiles the real `car_drive.c` and `car_config.c` with a
stand-in `main.h` and a model car: stiction below `start` %, speed following
`(pct − off) / slope` with time constant `tau`, friction when coasting, a
strong brake, encoder quantised to counts.

Sweep used when the loop was written (all must end `DONE` within 1 cm,
with at most one direction change):

```
for a in "d=1" "d=0.3" "d=0.1" "d=0.05" "d=0.02" "d=-0.5" "d=2 tt=8" "d=1 obst=3" \
         "d=1 off=38" "d=1 off=52" "d=1 slope=0.4" "d=1 slope=0.8" "d=1 tau=0.12" "d=1 tau=0.45" \
         "d=1 crawl=5" "d=0.1 crawl=5" "d=1 start=70" "d=0.1 start=70 tau=0.45" \
         "d=1 start=50 off=52" "d=0.1 off=38 slope=0.4 tau=0.45"; do
  tools/drive_sim/drive_sim $a
done
```

Results with the default gains: starts moving after 0.2 s (0.7 s when the
model needs 70 % to start), ends within 1 cm in every case, one swing back
in two of the mismatched cases (`slope=0.8`, `start=70 tau=0.45`). Gain
sets 3/2/0, 4/2/0.2 and 1.5/0.5/0 all pass, so the structure, not the exact
gains, is what makes it behave.

**This is a model, not the car.** As of 2026-11-11 the new loop has not
been run on the real car. See [roadmap.md](roadmap.md).

## Ideas not done

- A faster control step (20–50 Hz) would make the stop finer. The course
  material asks for 100 ms, and TIM3 is shared with the sonar timing.
- Scale the motor map with the battery voltage (the map was tuned at one
  voltage; a 3S LiPo goes from 12.6 V to about 10 V). Needs a trustworthy
  battery reading first.
- A separate speed loop inside the position loop (cascade). The
  feed-forward plus position PID is simpler and matches Activity 2.6.
