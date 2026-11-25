# The challenge track

The task: drive about 150 m alone, stop 10 s at three waypoints, avoid
obstacles, end at the start. This file holds what we know about the track
and the plan for driving it.

## The map (first rough version, 2026-11-25)

From one walk around the track with a phone compass. Lengths are rough;
the team will walk it again for better numbers.

The car starts at S pointing 353 degrees (nearly north). Bearings are
compass degrees. x = east, y = north, in metres from S.

| Leg | Length | Bearing | Turn before it | Ends at (x, y) |
|---|---|---|---|---|
| 1 | 45 m | 353 | start | (-5.5, 44.7) |
| 2 | 45 m | 84 | 91 right | (39.3, 49.4) |
| 3 | 17 m | 171 | 87 right | (41.9, 32.6) |
| 4 | 30 m | 218 | 47 right | (23.5, 8.9) |
| 5 | 15 m | 264 | 46 right | (8.5, 7.4) |

152 m in all. **The loop does not close:** after leg 5 the map is about
11 m north-east of S. Some lengths or bearings are off, or a piece back to
S is missing.

The (x, y) of each corner: x += length x sin(bearing), y += length x
cos(bearing). The turn at a corner is the change in bearing (here all to
the right).

### As a route

```
rc
rs 45
rt -91
rs 45
rt -87
rs 17
rt -47
rs 30
rt -46
rs 15
go
```

Minus = right. At 30 cm/s (`vn`) this takes about 8.5 minutes, at 60 cm/s
about 4.3 minutes.

### Still to measure on the next walk

- [ ] The length of every leg with a tape or a measuring wheel (legs 3-5
      most of all).
- [ ] The bearing of every leg: phone flat, away from metal.
- [ ] What comes after leg 5 (the map ends 11 m from S).
- [ ] Where the three 10 s waypoint stops are.
- [ ] Obstacles: what and where.
- [ ] Time limit.
- [ ] Does the path bend inside a leg? The car drives straight lines; a
      bend must be split into steps (`rs 20`, `rt 10`, `rs 25`).

## What the car needs before it can drive the track

Most important first.

1. **Gyro drift.** The heading comes from the gyro, and a small error
   grows over time: 0.05 deg/s over 5 minutes is 15 degrees, and 15
   degrees on a 30 m leg is 8 m to the side. The bias is only trimmed
   while the car stands, so put a stop (`rw 10`) at the corners; the three
   waypoint stops can do this. First measure the drift: car standing still
   for 5 minutes, watch `yaw` in `G`.
2. **Start without Bluetooth.** Bluetooth reaches about 10 m. The route
   must be in the code (`route_default` in `car_route.c`), and the blue
   button must start it (today it starts the open-loop plan).
3. **Go home.** On the last leg the car should work out the heading and
   distance to S from its own x/y (encoder distance along the gyro
   heading; `car_openloop.c` already adds this up). Then errors in the
   map and in the earlier legs do not decide where the car ends.
4. **Counts per metre on the real surface.** `M` was set indoors. 1 %
   wrong is 1.5 m over the track. Drive `d 10` outside, measure with a
   tape, set `M`.
5. **Recorder size.** It holds 4 minutes at 10 Hz; the track takes 4-8.
   Record less often on long runs.
6. **Steering play** (`sb`, see below): check on the track that the
   straights after the turns stay straight.
7. **Obstacles.** The car only stops and waits; it does not drive around.
8. **Speed.** Pick `vn` once the time limit is known.

## Compass (planned)

A magnetometer does not drift. The plan: the gyro steers as now, and the
compass slowly pulls the gyro heading to the true bearing (complementary
filter). Then route steps can use the bearings from the map directly
("drive 45 m at 84 degrees") instead of turns, so errors do not carry from
one leg to the next.

- **Module:** GY-271 (QMC5883L, I2C address 0x0D, or HMC5883L at 0x1E).
  Also fine: GY-273, GY-511 (LSM303), GY-91 / GY-9250 (MPU-9250). Not the
  GY-302: that is a light sensor (BH1750).
- **Wiring:** 3.3 V, GND, SDA, SCL in parallel with the OLED and the gyro
  on I2C1. `I` shows whether it answers.
- **Mounting:** as far from the motor and the battery as possible, on a
  10-15 cm post of plastic or wood, flat, with the board's arrow pointing
  forward.
- **Firmware to write:** driver, a calibration drive (`cc`, slow circles
  to measure the offset from the car's own metal), fusion with the gyro,
  route steps with absolute bearings.

## Findings on 2026-11-25

- **The car drove off at full speed by itself** at every start-up and on
  `X`. Cause: a loose wire between the Nucleo and the L298N (IN1 / IN2).
  The firmware brakes at start-up (both inputs high); with one input
  missing the L298N reads that as full speed. Fixed by pushing the plugs
  in. Check them before every test outside. The "weak brake" in
  [openloop.md](openloop.md) may have been the same wire.
- **Steering play, seen standing still:** after full lock to one side the
  servo goes back to the centre pulse, but the wheels stay a few degrees
  to that side. Small steps end straight; a big jump does not. So after a
  turn the next straight drifted to the turn side. Fix: `sb` (default
  30 us), the gyro steering adds this in the direction the pulse moves.
  Not yet checked on the track.
- **A turn followed by a separate `d`** holds whatever heading the car
  points at when `d` starts, so a turn that ends 4 degrees off gives a
  straight 4 degrees off. Inside a route (`rs`, `rt`, `go`) the straight
  holds the route heading and corrects it. Use routes on the track.
- **Battery:** new 3S 1200 mAh pack. `BATT_DIVIDER` 6.71 (11.8 V shown
  at 12.04 V on the multimeter).
- **Test recorder:** every `d`, `go` and `vs` is recorded on the car;
  `ld` downloads it (see [commands.md](commands.md)). A reset or
  unplugging the USB loses it, so download first.
