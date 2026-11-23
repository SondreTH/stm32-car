# Hardware

Board: NUCLEO-F446RE, with a perfboard on the inner morpho rows
(`wiring/team7_wiring.html` shows what is soldered where).

| Part | Model |
|---|---|
| Drive motor | 25GA370, 12 V, gearbox, quadrature encoder |
| Motor driver | L298N (ENA jumper removed, PWM on ENA) |
| Steering | MG996R servo, own 6 V supply from an XL4015 buck converter |
| Distance sensor | HC-SR04, ECHO through a 1k/2k divider |
| IMU | MPU-6050 at 0x68, **mounted on its side: the X axis points up**. Gyro read by `car_heading.c` |
| Display | 0.96" 128x64 I2C OLED, SSD1306, at 0x3C |
| Bluetooth | BT04-A (ZS-040), 9600 baud, shows up as `=HakonsDevice` (98:DA:20:0C:1A:63) |
| Power | 3S LiPo (11.1 V nominal, 12.6 V full), switch, 5 A fuse, 5 V module for the Nucleo (E5V) and sensors |

**Unplugging the USB cable resets the STM32** (seen twice on 2026-11-22,
from the uptime over Bluetooth right after), and plugging it back in did
too once. Everything set by commands is lost, the gyro measures its bias
again (keep the car still for a second). For runs without the cable: unplug,
then send commands over Bluetooth.

Flashing from a terminal works without CubeIDE (Linux, `st-flash` from
stlink):

```
tools/build_check.sh
arm-none-eabi-objcopy -O binary build/stm32-car.elf build/stm32-car.bin
st-flash --reset write build/stm32-car.bin 0x08000000
```

That build uses -O2 instead of CubeIDE's Debug -O0. Every run of
2026-11-22 was on such a build.

## Pins (from `stm32-car.ioc`, names in `main.h`)

| Pin | Function | Name | Connected to |
|---|---|---|---|
| PA0 / PA1 | TIM5_CH1 / CH2 | `ENC_A` / `ENC_B` | encoder A / B |
| PA2 / PA3 | USART2 | – | ST-LINK virtual COM port, 115200 |
| PA4 | ADC1_IN4 | `BATT_SENSE` | battery divider |
| PA5 | GPIO out | `LD2` | green LED on the Nucleo |
| PA7 | EXTI7, both edges | `SONAR_ECHO` | HC-SR04 ECHO via divider |
| PA8 | TIM1_CH1 | `MOTOR_ENA` | L298N ENA |
| PA9 / PA10 | USART1 TX / RX | `BT_TX` / `BT_RX` | Bluetooth RXD / TXD, 9600 |
| PB0 | GPIO in, pull-up | `START_BTN` | button to GND |
| PB4 / PB5 | GPIO out | `MOTOR_IN1` / `MOTOR_IN2` | L298N IN1 / IN2 |
| PB6 | GPIO out | `SONAR_TRIG` | HC-SR04 TRIG |
| PB8 / PB9 | I2C1 SCL / SDA | `I2C_SCL` / `I2C_SDA` | OLED and MPU-6050, 400 kHz |
| PB10 | TIM2_CH3 | `SERVO_PWM` | servo signal |
| PC13 | GPIO in | `B1` | blue user button |

## Clock and timers

System clock 84 MHz from the HSI through the PLL (M 16, N 336, P 4).
APB1 timers and APB2 timers both run at 84 MHz.

| Timer | Settings | Use |
|---|---|---|
| TIM1 | PSC 0, ARR 16799 → 5 kHz in CubeMX; `hw_init()` sets PSC 49 → **100 Hz** (`MOTOR_PWM_HZ`) | motor PWM on CH1 (PA8). Duty = motor % |
| TIM2 | PSC 83 → 1 MHz, ARR 19999 → 50 Hz | servo PWM on CH3. Compare value = pulse in µs |
| TIM3 | PSC 839 → 10 µs per count, ARR 9999 → 100 ms | update interrupt = control step; counter = sonar echo timing |
| TIM5 | encoder mode TI1+TI2 (x4), 32-bit, input filter 6 | wheel encoder |

Hakon's original project ran at 72 MHz with a 14400-count PWM period, which
is why the motor map is written as "6500 + 85 per cm/s out of 14400".

## Motor (`motor_set`, `motor_brake` in `car_hw.c`)

- `motor_set(pct)`: −100..100. Positive: IN1 high, IN2 low. Negative: the
  other way. 0: both low and no PWM, the motor coasts.
- `motor_brake()`: IN1 and IN2 high with full PWM: the L298N shorts the
  motor, it brakes. **In practice it barely brakes** (2026-11-22): 87 to
  62 cm/s in 0.6 s, 66 to 40 cm/s in 1.5 s, about the same as coasting.
  The braking current goes through the L298N's bipolar transistors and
  diodes, about 2-2.5 V in total, so it only flows while the motor's
  back-EMF is higher than that. PWM on IN1/IN2 with ENA high, or a
  driver with MOSFETs, would brake properly. A short reverse pulse would
  too, but its length depends on the speed.
- `MOTOR_INVERT` in `car_config.h` swaps the direction if the wiring is
  the other way round.
- PWM frequency matters a lot, because the PWM is on the enable pin (the
  motor coasts between pulses). Measured 2026-11-12: at 5 kHz and 500 Hz
  nothing moves below 70-80 % and then the car runs away; at 100 Hz it
  starts at 30-40 % and 40 / 50 / 70 % give about 30 / 54 / 93 cm/s.
  Those numbers were on a nearly empty pack. On a full pack (11.5 V,
  2026-11-22) 45 % reached 140 cm/s, 40 % about 70-100 cm/s, 35 % 25-100
  cm/s depending on what came before. The speed takes about 1.5 s to
  settle at a constant %. With the wheels in the air 45 % spins the wheel
  at 290 cm/s.
- From a standstill, 35 % does not move the car with the front wheels at
  full lock, and a 50-60 % kick spins the rear wheel on a smooth floor
  (encoder counts, the gyro sees no turn). See
  [openloop.md](openloop.md).
  `motor_pwm_hz()` (command `pf`) changes the TIM1 prescaler; the default
  is 100 Hz. The motor buzzes at that frequency.
- `MOTOR_START_PCT` 30, `MOTOR_MIN_PCT` 22, `MOTOR_PCT_PER_CMS` 0.5 in
  `car_config.c` are a rough line through those points. The speed loop in
  `car_drive.c` corrects the rest. Older values (5 kHz): 60, 45.1, 0.59.
- The drivetrain has a tight spot (felt by hand, seen as speed surges at
  constant %). Not fixed as of 2026-11-12.
- Wiring idea not tried: PWM on IN1 with ENA held high makes the L298N
  brake between pulses, so speed follows the duty much more closely.

## Encoder

TIM5 counts every edge of A and B. `COUNTS_PER_METER` = 1035 (10.35 counts
per cm, measured by Hakon), so one count is about 1 mm, and at the 100 ms
control step the speed resolution is about 1 cm/s. `encoder_count()` is
relative to the last `encoder_zero()`. `ENCODER_INVERT` must be set so that
forward counts **up**; the position loop depends on it and stops a run with
an error if it is wrong.

## Servo

Pulse widths measured on the car (`car_config.h`): 600 µs full left,
1080 µs straight (`SERVO_CENTER_US`, was 1100), 1500 µs full right. The
heading hold and the open-loop plan use any pulse in between.

Measured in the open-loop runs of 2026-11-22:

- +350 µs (1430) turns right with a radius of 62-73 cm at 40-70 cm/s,
  very repeatable. During a hard kick the radius grows to 150-250 cm (the
  tyres slide).
- **Play in the steering.** Back at 1080 µs after a right turn the car
  curves right (radius about 4 m, 10-14 deg per 1 m side). Steering 1030
  µs (−50) after a right turn drives straight. Probably the same in the
  other direction after a left turn (not measured).
- `cal` (steering table for nine pulses) has still not been run. Before
  2026-11-22 it could not have worked: the gyro read the wrong axis.

## Sonar

`sonar_trigger()` (every control step) sends a 10 µs TRIG pulse.
`sonar_echo_edge()` (EXTI on PA7) times the echo with TIM3's counter:
distance in cm = echo µs / 58. Resolution 10 µs ≈ 1.7 mm. No echo before
the next trigger = `SONAR_MAX_CM` (400). One reading per 100 ms.

## Battery reading: known problem

Intended circuit (wiring guide, J9): battery + → 10k → PA4 → 3.3k → GND,
factor (10 + 3.3) / 3.3 = 4.03.

Observed on 2026-11-11 (reported by Filip): the OLED showed 7.1 V while a
multimeter on the battery showed 11.6 V. With the factor 4.03 that means
1.76 V on PA4, so the real factor is 11.6 / 1.76 = **6.58**. A 10k / 1.8k
divider gives 6.56, so the most likely cause is a 1.8k resistor where the
3.3k should be (or another pair with the same ratio, such as 5.6k / 1k).
Not confirmed by measuring the resistors.

What was done: `BATT_DIVIDER` is now a variable, default 6.58. `G` prints
the PA4 voltage and the factor. `vb <volts>` computes the factor from a
multimeter reading.

Still to do on the car: measure PA4 to GND. About 1.76 V at 11.6 V confirms
the divider theory. About 2.9 V would mean the divider is as drawn and the
ADC side is wrong (then look at VDDA / the 3.3 V rail and the ADC setup).
Either way PA4 is far below 3.3 V, so nothing is at risk.

Second pair of readings, 2026-11-12:

| Multimeter on the battery | Shown (factor 6.58) | PA4 from the reading |
|---|---|---|
| 11.6 V | 11.49 V | 1.75 V |
| 7.9 V | 6.7 V | 1.02 V |

One factor cannot fit both (6.63 and 7.76). Two explanations, not told
apart yet:

- The two 7.9 / 6.7 values were not taken at the same moment. A nearly
  empty LiPo sags a lot under load and recovers when the load is gone.
- Something in the sense path drops a fixed voltage: a straight line
  through both points is battery = 5.05 x PA4 + 2.75 V.

To settle it: multimeter on the battery and on PA4 (CN7-32) at the same
moment as `G`, at two different battery voltages. Telemetry now has
`batt_v`, which shows the sag while driving.

**7.9 V is far too low for a 3S LiPo** (2.6 V per cell; the cells are
damaged below about 3.0 V per cell, 9.0 V). `battery_poll()` in
`car_app.c` now warns below `BATT_LOW_V` (9.9 V, 3.3 V per cell): "LOW BAT"
blinks on the OLED and a line is printed every 30 s. It only warns; it
does not stop the car. With the reading as it is, the warning comes early
rather than late.

ADC setup: ADC1, channel 4, 12 bit, 480-cycle sampling, software start.
`battery_pin_volts()` averages 8 conversions and assumes VDDA = 3.3 V.

## I2C1

400 kHz on PB8/PB9. The OLED (0x3C) and the MPU-6050 (0x68) share the bus.
`i2c_scan()` (the `I` command) lists what answers and re-initialises the
peripheral, which clears a stuck BUSY flag after hot-plugging.

Both devices are only used from the main loop, so they never interrupt
each other. `car_heading.c` reads the gyro (6 bytes, about 0.3 ms) every
5 ms or more; the OLED update blocks the bus for ~25 ms five times a
second, and the heading integration measures the real time between
readings. Gyro setup: ±250 deg/s, 44 Hz low-pass. `IMU_YAW_AXIS` /
`IMU_YAW_SIGN` in `car_config.h` describe how the module is mounted.

**The module stands on its side: X points up** (`IMU_YAW_AXIS 0`, sign
+1; checked 2026-11-22 by turning the car 90 degrees left by hand: X =
+90, Y = 0, Z = 1). Until then the code read Z, which only sees pitch and
roll here, so the heading never saw a turn: every heading result from
before 2026-11-22 (the heading hold, "within 1-2.5 degrees") measured
nothing. The standing bias of X is about −4.4 deg/s (measured at
start-up and subtracted). `G` shows the angle around all three axes since
`Y`, for checking the mounting again.

A failed gyro reading does not lose time any more (the next good one
covers the gap), and `G` counts failures. None failed on 2026-11-22,
also with the motor running.

## Bluetooth

BT04-A on USART1 at 9600 baud, name `=HakonsDevice`, address
98:DA:20:0C:1A:63, PIN 1234. Pairing on Linux:

```
bluetoothctl     # then: agent KeyboardOnly, default-agent,
                 # pair 98:DA:20:0C:1A:63 (PIN 1234), trust 98:DA:20:0C:1A:63
python tools/car_console.py 98:DA:20:0C:1A:63
```

`car_console.py` opens an RFCOMM socket itself (no `rfcomm` tool, which
newer BlueZ no longer ships). On Windows, use the COM port.

**The module loses data in long bursts.** Until 2026-11-22 every line from
the car longer than about 43 characters arrived cut: the module passed on
the first ~43 bytes of a burst and lost the next ~20 while it was sending
over the radio. `car_comms.c` now sends to the Bluetooth port in pieces of
20 bytes with 20 ms between them (`BT_CHUNK`, `BT_GAP_MS`), about 400
bytes/s; at `T 5` all lines arrive whole. Commands to the car were never
affected. When a connection opens, the module sends something the car
answers with `ERR unknown command` once or twice: harmless.

## Printed parts (`cad/`)

- `deck_plate`: 180 × 110 × 3 mm top plate with a 10 mm grid of M3 holes,
  on four M3 standoffs. Set `mount_x` / `mount_y` in the `.scad` to the
  hole spacing of the car before printing.
- `sonar_bracket`: holds the HC-SR04 at 45 mm height, tilted 3° up.
