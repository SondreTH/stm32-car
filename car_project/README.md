# MR2006B car – complete software v1.3

```
car_project/
  cubeide/          STM32CubeIDE project (Team7_Car.ioc + C code)  <- main version, see cubeide/README_CubeIDE.md
  car_firmware/     Arduino IDE version of the same firmware (open car_firmware.ino)
    config.h        every pin and tunable number             <- you edit this
    path_data.h     the route the car drives in AUTO          <- replaced by DUMP
  tools/            laptop tools (Python 3: pip install pyserial matplotlib)
    car_console.py  terminal + automatic telemetry log + path saving
    plot_run.py     plot a run (where the car went vs. the path)
    path_builder.py make a path from measured legs (alternative to TEACH)
    sim.cpp         PC simulation of the autopilot (optional)
  cad/              3D-printable deck plate + HC-SR04 bracket (.scad + .stl)
  wiring/           team7_wiring.html - perfboard wiring guide (open in a browser)
```

Both firmware versions use the same pins and the same commands, and both match the soldered perfboard
(servo on PB10 / CN10-25). Use **one** of them: CubeIDE for the course, Arduino as a backup.

**How the car works**
- **Distance** comes from the encoder. **Heading** comes from the MPU-6050 gyro. Together they give the car's **x/y position**.
- In **TEACH** mode you drive the course once over Bluetooth. The car records its own x/y track, and you mark the 3 waypoints and the finish.
- In **AUTO** the car replays that track by steering toward a point 0.6 m ahead on the path (pure pursuit).
  - It slows down and stops exactly at each waypoint, waits 10 s, and ends at the start.
- **Obstacles:**
  - If something is closer than 40 cm, the car stops and waits.
  - If it's still there after 4 s, the car backs up and drives around it (worth 2 points).
- **Why teach-and-repeat:** errors like a slightly wrong gyro scale are the same during teaching and replay, so they cancel out.
  - In the simulation, a 3 % gyro error gave 1–2 cm waypoint error with a taught path.
  - The same error gave 50–220 cm with a path measured from the map.
  - Real runs will be worse than 1–2 cm, because wheel slip and random drift don't cancel.

---

## 1. Arduino IDE setup (one time)

1. **File → Preferences → Additional boards manager URLs**, add:
   `https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json`
2. **Boards Manager** → install **STM32 MCU based boards**.
3. **Tools**:
   - Board → **Nucleo-64**
   - Part number → **Nucleo F446RE**
   - Upload method → **Mass Storage**
4. Open `car_firmware/car_firmware.ino` and upload.
5. Talk to the car with the Serial Monitor at **115200** baud, line ending **Newline**, or with `tools/car_console.py` (better: it logs).

## 2. Wiring

| Part | Part pin | Nucleo pin | Header |
|---|---|---|---|
| Encoder | A / B | PA0 / PA1 | A0 / A1 |
| Encoder | VCC / GND | 3V3 / GND | |
| L298N | ENA (remove jumper) / IN1 / IN2 | PA8 / PB5 / PB4 | D7 / D4 / D5 |
| Servo | signal | PB10 | D6 |
| HC-SR04 | TRIG / ECHO (ECHO via 1k–2k divider) | PB6 / PA7 | D10 / D11 |
| Bluetooth | TXD / RXD | PA10 / PA9 | D2 / D8 |
| MPU-6050 | SDA / SCL | PB9 / PB8 | D14 / D15 |
| Battery monitor | battery+ → 10k → pin → 3.3k → GND | PA4 | A2 |

**Power:**
- Battery → switch → fuse feeds three things:
  - the L298N (motor)
  - a 6 V buck converter (servo only)
  - the 5 V module (Nucleo E5V + sensors)
- All grounds are common.
- Keep the car still for 2 s after power-on while the gyro calibrates.

## 3. This week: sensor bring-up (encoder, gyro, distance sensor)

Send `T 10` to get telemetry. With `car_console.py` it's saved to `tools/logs/` automatically. Record the results in the table in section 9.

**Encoder**
1. **Direction.** Lift the rear wheels and turn one forward by hand: `counts` must go up. If not, set `ENCODER_INVERT`.
2. **Motor direction.** Send `P 30`: the wheels must turn forward. If not, set `MOTOR_INVERT`. `X` stops.
3. **Deadband.** Send `P 5`, `P 10`, `P 15`, … The lowest value that reliably moves the car is the deadband. Set it with `B`.
4. **Counts per metre (most important).**
   - Send `Z`, then `P 40` along a tape. Send `X` after about 5 m.
   - Counts per metre = counts ÷ metres. Do it 3 times and average. Set it with `C` and in `config.h`.
5. **Top speed.** Send `P 100` and read `v_mps`. Set `SPEED_FF ≈ (100 − deadband) / top_speed`.
6. **Speed loop.** Send `V 0.3`: `v_mps` should follow `set_mps` without oscillating. If it oscillates, lower KP. If it's slow to reach the setpoint, raise KI.
7. **Distance.** Send `Z` then `D 5`, 5 times. Stops within a few cm of each other means the encoder is good.

**Steering**

8. **Limits.** Step `U 1500` by 25 µs (`U 1525`, `U 1550`, …) to each end stop. Back off 25 µs if the servo strains. Set `SL`, `SR`, `SC`.
9. **Angle scale.** Send `S 20` and measure the front-wheel angle (a phone protractor app works).
   - Set `SERVO_US_PER_DEG` so that `S 20` really gives 20°.
   - The autopilot depends on this.
10. **Wheelbase and sonar position.**
    - Measure from the front axle to the rear axle and put it in `WHEELBASE_M`.
    - Measure from the rear axle to the sonar face and put it in `SONAR_X_M`.

**Gyro**

11. **Found.** `G` must show `IMU OK`.
12. **Direction.** Send `Z` and rotate the car left by hand: `heading_deg` must go up. If not, set `GYRO_INVERT`.
13. **Scale (don't skip).**
    - Put the car against a wall edge, send `Z`, and turn it 5 full turns left. Put it back against the edge.
    - Set `GYRO_SCALE = 1800 / heading shown`.
    - Check it with 5 turns right.
14. **Drift.**
    - Wheels lifted: send `Z`, then `P 30`. After 3 minutes, `heading_deg` should be within about 2°.
    - Standing still, the car corrects its own drift, so test with the motor running.

**Distance sensor**

15. Check `sonar_cm` against a tape at 20, 50 and 100 cm. 400 means nothing is seen.
16. Walk around the car outdoors. It must NOT see the ground, a curb or the step. If it does, mount it higher or tilt it up (the printed bracket has a 3° tilt).

**Put it together**

17. **Odometry.** Send `Z`, drive an L-shape with `V`/`S`, stop, and compare `x_m`, `y_m` with a tape.
18. **Indoor autopilot test.** The built-in demo path is 2 m straight → stop → 90° left turn → 1 m → finish. It needs about 2 × 3 m of floor.
    - Put the car down and press the blue button (or send `GO`).
    - It stops after 2 m for 10 s, then turns and finishes.
19. **Obstacle test.** Run the demo again and step in front of the car.
    - It should stop, then continue when you step away.
    - Put a box in front instead: after 4 s it should back up and drive around the box.

## 4. Commands

| Command | Does |
|---|---|
| `X` | Stop. Also aborts AUTO |
| `V 0.3` / `S 15` | Speed (m/s) / steering (° , + = left) |
| `P 40` / `D 5` | Open-loop motor % / drive 5 m and stop |
| `REC` | Start recording a path. The car's position now = START |
| `WP` | Record a waypoint here (car stopped exactly on it) |
| `NOOBS` | Toggle an obstacle-free zone (e.g. passing close to a wall) |
| `END` | Record the finish here and stop recording |
| `GO` | AUTO: 3 s countdown, gyro calibration, then drive the path |
| `PATH` / `DUMP` / `LOAD` | Show path / print it as `path_data.h` / reload the built-in one |
| `AS 0.3` `LA 0.6` `OB 40` `RR 1` `DO 0.6` | Cruise speed, look-ahead, obstacle stop distance, reroute on/off, detour size (+ = pass on the left) |
| `Z` `GC` `GS` `C` `K` `B` `U` `SC/SL/SR` `WB` | Calibration (see section 3) |
| `T 10` `G` `W 1000` `?` | Telemetry rate, settings, watchdog, help |

**Blue button:**
- In AUTO: abort.
- While moving: stop.
- In TEACH, standing still: record a waypoint.
- Otherwise: start AUTO.

## 5. Teaching the course (the "training drive")

1. **Make a start jig.** Mark the START on the ground at the bell: tape where the two rear wheels go, plus a line for the heading.
   The start heading must be identical every time. It's the biggest single error source.
2. **Set up the phone.** In Serial Bluetooth Terminal, create macro buttons:
   `V 0.2`, `X`, `S 25`, `S 12`, `S 0`, `S -12`, `S -25`, `WP`, `END`.
3. **Start recording.** Put the car in the jig and send `REC`. It calibrates the gyro for 1 s: don't touch it.
4. **Drive the course slowly** (`V 0.2`), in the middle of the sidewalk.
   - Stay away from edges: the detour needs about 0.6 m of room on one side.
   - Smooth driving gives a smooth path.
5. **At each waypoint:** stop so the car is exactly on the marker, then send `WP` (or press the blue button).
   - Use the same point of the car every time (e.g. the rear axle over the mark), because that's the point AUTO will stop on.
6. **At the finish** (back at the bell): stop exactly in the start position and send `END`.
   - It prints the **loop closure**: how far the odometry thinks the finish is from the start. This is your drift over one lap.
7. **Save it.** Send `DUMP`. `car_console.py` saves it to `tools/saved_paths/`.
   - Copy that file over `car_firmware/path_data.h` and upload the sketch. The path is now permanent.
   - To test without re-uploading, `!upload <file>` in the console sends a path into the car's RAM.
8. **Replay.** Put the car in the jig and press the button. Check the waypoint errors and plot the run: `python plot_run.py logs/run_….csv --path ../car_firmware/path_data.h`.

**Improving accuracy:**
- Run 3 times and measure where the car stops at each waypoint.
- If an error is the same every run, it can be corrected. Nudge that waypoint's x/y in `path_data.h`, or re-teach.
- If the car cuts corners, lower `LA` (look-ahead). If it wobbles on straights, raise it.
- **Time check:** 150 m at 0.3 m/s plus 3 × 10.5 s ≈ 9 min, inside the 15 min limit.
  - Don't go below about 0.2 m/s average.
  - `AS` sets the cruise speed.

## 6. Laptop tools

```
pip install pyserial matplotlib
python tools/car_console.py                 # lists ports
python tools/car_console.py COM5            # USB (115200)
python tools/car_console.py COM7 9600       # Bluetooth COM port
python tools/plot_run.py tools/logs/run_XXXX.csv --path car_firmware/path_data.h
python tools/path_builder.py tools/course_example.txt -o car_firmware/path_data.h
```

- **Over Bluetooth:** use `T 5` rather than `T 10`, because 9600 baud is slow.
- **The simulator (optional):** runs the real autopilot code (`cubeide/.../path.c`, `autopilot.c`) on a simulated course.
  Build instructions are at the top of `tools/sim.cpp`, then `./sim 0.01 t`.

## 7. 3D-printed parts (`cad/`)

- **`deck_plate`** – an extra top layer, 180 × 110 × 3 mm.
  - It has a 10 mm grid of M3 holes (screws or zip ties anywhere) and two cable slots.
  - It mounts on four M3 brass standoffs.
  - **Before printing:** measure the spacing of the 4 holes you'll use in the existing upper plate. Set `mount_x` / `mount_y` in the `.scad` file and export a new STL in OpenSCAD (free).
- **`sonar_bracket`** – holds the HC-SR04 at 45 mm height, tilted 3° up. Two slots screw it to the front plate.
  - Check the eye spacing (26 mm) with calipers first.
- **Print:** PLA or PETG, 0.2 mm layers, 20 % infill, no supports.

## 8. Race-day checklist

- [ ] Battery charged, and the voltage shows in `G`
- [ ] The correct `path_data.h` uploaded (`PATH` shows 3 STOPs + FINISH)
- [ ] Car in the start jig, heading exact
- [ ] Hands off during the 3 s countdown (gyro calibration)
- [ ] Laptop or phone connected, `T 5` running, to show the telemetry to the professor

## 9. Results table

| Value | Measured |
|---|---|
| ENCODER / MOTOR / GYRO invert | |
| Deadband % | |
| Counts per metre (3 runs) | |
| Top speed at P 100 (m/s) | |
| Servo min / centre / max (µs), µs per degree | |
| Wheelbase, sonar offset (m) | |
| GYRO_SCALE | |
| D 5 result (5 runs) | |
| Loop closure after TEACH (cm) | |
| Waypoint errors, AUTO run 1 / 2 / 3 (cm) | |
