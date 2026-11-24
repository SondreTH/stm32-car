# Open-loop route (`car_openloop.c`)

Written 2026-11-22, after the first evening of tests on the car (Filip and
the team, for the MR2006B activity "Open loop control for mobile robots":
a 2-3 minute video, the car must drive a geometric shape and get back to
where it started).

## What it does

A plan is a list of up to 12 steps. Each step is a motor %, a steering
offset in µs (added to `SERVO_CENTER_US`, negative = left) and a time. The
car plays the list and does not look at any sensor while it drives: no
encoder, no gyro, no speed loop. That is the point of the activity.

- Motor 0 % in a step **brakes** (both L298N inputs high). It does not
  coast. After the last step the car brakes 0.5 s, then coasts, wheels
  straight.
- `or <n>` plays the whole list n times (for a shape made of identical
  laps). The run can have up to 48 steps.
- `ov <volts>` (off by default) scales the duty with the battery:
  `duty = pct x V_ref / V_battery`. The motor gets on average
  duty x battery voltage, so this keeps it the same as the battery drains.
  Feedforward from a measured disturbance, still open loop. `ov 10` on a
  full battery makes the car behave as if the plan ran on a 10 V battery
  (a way to show the battery disturbance without draining it).
- Safety stops end the run with a brake, and never steer or change the
  speed: the sonar (closer than 30 cm, `os 0` turns it off), a blocked
  wheel (motor on, no counts for 3 s), the start button, `s`.
- Steps change on `HAL_GetTick()` counted from the start, in the main loop.
  The OLED update can make a step change up to 25 ms late; delays do not
  add up.

The encoder and the gyro are only **recorded** (10 samples a second, up to
1200), for comparing the run with the plan:

- After a run the car prints one line per step: distance, angle turned,
  deg/s and turn radius. Then an estimate of where it stopped, from the
  encoder distance along the gyro heading (dead reckoning, x = forward at
  the start, y = left). Against the tape it was off by 25-50 cm after
  8-9 m of driving (estimate 14 / 17 / 84 cm from the start, tape about
  40 / 30 / 35 cm): kicks spin the wheel, and the encoder counts that as
  distance. Good for the shape of the path and for comparing runs; the
  tape is the real result.
- `od` prints the recording as CSV with the plan and the report as `#`
  lines. `tools/car_console.py` saves it to `tools/logs/`.
  `tools/plot_openloop.py` draws the paths of several runs on top of each
  other and motor % / speed / heading over time (1920x1080, for a video).

Commands: see [commands.md](commands.md). The default plan is at the top of
`car_openloop.c`.

## How to run it without a cable

Unplugging the USB cable **resets the STM32** (seen twice on 2026-11-22).
Anything sent over USB is gone after that, so:

1. Unplug the USB, keep the car still for a second (gyro bias at start-up).
2. Connect over Bluetooth: `python tools/car_console.py 98:DA:20:0C:1A:63`
   (Linux, see [hardware.md](hardware.md#bluetooth)).
3. Send the plan (`ol ...`, `or ...`), put the car on the start mark, `og`
   (starts 2 s later) or press the blue button.
4. After the run: `od` saves the recording. The report also comes over
   Bluetooth.

During the session the plan was typed fresh after each reset; a plan that
works belongs in `olPlan[]` in `car_openloop.c`.

## The square, 2026-11-22

Shape: four straight sides and four right turns of 90 degrees, back to the
start mark. The turns are arcs (Ackermann steering), so the corners are
round. Floor: smooth indoor floor at Filip's place. Battery 11.4 → 11.1 V
over the evening (3S LiPo, freshly charged).

The plan that worked (run `cont7` and `rec8`):

| Step | Motor | Steer | Time | Why |
|---|---|---|---|---|
| 1 | 55 % | 0 | 0.3 s | kick: the car needs more % to start than to keep rolling |
| 2 | 40 % | 0 | 0.6 s | first side, short: the roll-out at the end finishes it |
| 3, 5, 7, 9 | 46 % | +350 µs | 1.5 s | right turn. More % than the sides, to make up for the drag of the turn |
| 4, 6, 8 | 40 % | −50 µs | 1.3 s | side. −50 µs makes up for the play in the steering after a right turn |
| 10 | 40 % | −50 µs | 0–0.45 s | end of the last side (see below) |
| 11 | brake | −50 µs | 1.0 s | stop |

Results with that plan: the heading at the end was within 1 degree of the
start (−359 deg in total, gyro) in both runs. Without step 10 the car
stopped about 30 cm short of the start mark; with step 10 at 0.45 s it
stopped 30-40 cm past it (tape, Filip). Somewhere around 0.2 s should end
on the mark: **not run yet**. The default in `car_openloop.c` is that
interpolated 0.2 s.

The turn angles of one plan still varied from run to run and from corner
to corner: 84 / 87 / 85 / 97 deg in one run, 93.5 / 85 / 87 / 96 in the
next, same command. The fourth corner came out the largest in every run.

Filip's observation (by eye, `rec8`): the car overturns a little in each
corner and then curves gently left on the straights (the −50 µs), so the
corners need to be about 93 degrees to come out right. The gyro showed the
straights within 1 degree of straight, so this needs checking with a video
from above before it is used for turning by gyro.

### The runs of the evening

Files in `tools/openloop_runs/2026-11-22/` (copied from `tools/logs/`,
which git ignores), with `tuning_paths.png` and `tuning_signals.png`
(the last four runs, from `tools/plot_openloop.py`). Times are local.

| File | Plan | What happened | What it shows |
|---|---|---|---|
| `ol_..._210918_cal1` | continuous: 45 % sides 2.5 s, turns +350 µs 3.0 s | up to 140 cm/s, about 270 deg per corner (by eye), sonar stop at 18 s into Filip's hand | battery: 45 % on a full pack is much faster than 50 % was on 2026-11-12 (54 cm/s, nearly empty pack). The gyro recorded nothing: wrong axis (fixed after this run) |
| `..._212039_sq2` | continuous: 35 %, sides 2.0 s, turns 1.3 s | turns 99 / 62 / 38 / 53 deg, 270 deg in total, ended about 1.5 m from the start (by eye; estimate 161 cm) | steering is repeatable (radius 62-69 cm every turn), the speed is not. The motor needs ~1.5 s to settle and each turn slows the car, so every segment starts at another speed |
| `..._212436_lap1` | stop-and-go, one lap (`or 1`): kick 60 %, side 1.5 s, brake 0.6 s, steer, kick, turn 1.3 s | −87 deg | the brake barely brakes: 87 → 62 cm/s in 0.6 s, so the car was still rolling while it steered. The 60 % kick spins the wheel (137 cm/s on the encoder, then 35 cm/s) |
| `..._212709_lap2` | one lap: kick 50 %, side 1.0 s, brake 1.5 s, turn 1.9 s | −66 deg; rolled 78 cm while "braking" | the same weak brake |
| `..._213147_square3` | stop-and-go x 4, turn 2.8 s | lap 1 turned 82 deg, laps 2-4 only 11 / 2 / 3 deg | from a standstill with the wheels at full lock, 35 % cannot move the car; the 50 % kick spins the rear wheel in place (encoder +30 cm, gyro 0.3 deg). Lap 1 only worked because the car was still rolling |
| `..._213620_cont4` | continuous: kick 55 %, sides 40 %, turns 46 % 1.4 s | turns 109 / 76 / 87 / 109; the sides after a turn curved 10-14 deg right, the first side only 3 deg | play in the steering: back at 1080 µs after a right turn, the wheels stay a little right |
| `..._213754_cont5` | + sides after turns at −50 µs, turns 1.3 s, first side shorter | straights within 1.2 deg; turns 83 / 77 / 73 / 82; sonar stop at the end | the −50 µs fixes the play |
| `..._213852_cont6` | turns 1.45 s | turns 82 / 82 / 84 / 89, −348 deg in total; estimate 14 cm from the start, tape about 3x that (~15 cm behind, ~40 cm right) | first run that closed |
| `..._214202_cont7` | turns 1.50 s | turns 84 / 87 / 85 / 97, −359 deg; heading spot on, 30 cm short of the start (tape) | |
| `..._214411_rec8` | + 0.45 s straight before the brake (filmed) | turns 93.5 / 85 / 87 / 96, −359 deg; 30-40 cm past the start (tape) | the extra straight also made the brake roll further (51 cm instead of 43) |

## What the car taught us (material for the video)

Nonlinear behaviour:

- **Dead zone.** Below about 30 % the car does not move. 35 % is right at
  the edge: in a turn the speed fell from 27 to 7 cm/s and the car nearly
  stalled.
- **Static vs rolling friction.** Starting needs more than rolling: a kick
  (55-60 % for 0.3 s) gets it going. With the wheels at full lock and the
  car standing, even the kick does not move it.
- **Wheel spin.** A hard kick spins the rear wheel: the encoder says the
  car moved 30 cm, the gyro says it did not turn at all.
- **The brake is weak.** The L298N brakes the motor through its own
  transistors and diodes (about 2-2.5 V in total). Braking current only
  flows while the motor's back-EMF is higher than that, so at these speeds
  braking is nearly the same as coasting (17-40 cm/s per second).
- **Slow speed response.** At a constant % the speed takes about 1.5 s to
  settle (the 1.53 s time constant from the system identification), so the
  distance in a segment depends on the speed it started with.
- **Speed depends on the load.** A turn slows the car (the tyres scrub),
  the wheels in the air spin at 290 cm/s at 45 %, the floor gives about
  40-140.
- **The radius depends on the speed.** About 65 cm at 40 cm/s, 150-250 cm
  during a kick (the tyres slide).
- **Play in the steering.** The wheels do not return to the same place
  from left and from right.

Disturbances:

- **Battery voltage.** 45 % gave up to 140 cm/s on a full pack; 50 % gave
  54 cm/s on 2026-11-12 with a nearly empty one. The battery compensation
  (`ov`) is the open-loop answer.
- **The floor and the tight spot in the drivetrain**: the same command
  turned 84 or 97 degrees in different corners.
- **Anything that pushes the car** (a hand, a cable): nothing corrects it.

What made the open-loop plan work, all of it decided in advance: a kick
for the dead zone, more % in the turns for their drag, −50 µs on the
straights for the play, and timing from measured runs (calibration). The
plan only works on this floor, at this battery voltage, from this start
position. That is the limit of open loop, and the reason for closed-loop
turns (turn until the gyro says 90 degrees) on the real circuit.

## Scenarios not run yet (planned for the video)

Same plan, one thing changed: repeat runs (spread of end points), `ov 10`
(lower battery), extra weight on the car, a push during a side, a start
rotated 10 degrees or 20 cm off the mark, no kick or 35 % instead of 40 %
(dead zone), another floor.
