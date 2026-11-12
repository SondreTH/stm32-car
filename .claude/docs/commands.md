# Commands, telemetry, CubeMonitor

## Serial commands (`handle_command` in `car_app.c`)

Both ports accept the same commands: USB (ST-LINK virtual COM, 115200) and
Bluetooth (9600). A line ends with LF or CR. The first letter is not case
sensitive. A number may follow directly or after a space.

The parser checks the two-letter settings first (`tt ta td tol kp ki kd ms
mo mg vn vb sp si sc kh pf`), then the single letters. So `T 5` is telemetry, `tt 5` is
the run time; `M 1035` is counts per metre, `ms 60` is the start power;
`v18` is open-loop speed, `vn 18` the nominal speed.

| Command | Does | Variable |
|---|---|---|
| `d <m>` | distance run, negative = reverse | – |
| `tt <s>` | total time of a run, 0 = use the nominal speed | `run_T` |
| `ta <s>` / `td <s>` | acceleration / deceleration time of a timed run | `run_ta` / `run_td` |
| `vn <cm/s>` | nominal speed for runs without `tt` (2..80) | `DRIVE_SPEED_CMS` |
| `tol <cm>` | tolerance for "arrived" (min 0.2) | `POS_TOL_CM` |
| `kp` / `ki` / `kd <gain>` | PID gains | `Kp` / `Ki` / `Kd` |
| `ms <pct>` | motor % to start from standstill | `MOTOR_START_PCT` |
| `mo <pct>` | motor offset once rolling | `MOTOR_MIN_PCT` |
| `mg <pct per cm/s>` | motor slope | `MOTOR_PCT_PER_CMS` |
| `vb <volts>` | calibrate the battery reading to this multimeter value | `BATT_DIVIDER` |
| `sp` / `si <gain>` | speed loop gains | `Kvp` / `Kvi` |
| `pf <Hz>` | motor PWM frequency, 50..5000, default 100 | TIM1 prescaler |
| `kh <us/deg>` | heading hold gain, 0 = off, negative flips the direction | `STEER_KH` |
| `sc <us>` | servo pulse that drives straight | `SERVO_CENTER_US` |
| `Y` | zero the heading | `yawDeg` |
| `v <cm/s>` | hold a speed with the speed loop (negative = reverse), `v0` stops | – |
| `f` / `b` | forward / back at `MANUAL_PCT` (60 %) | – |
| `1`..`6` | 50..100 % in the current direction, only while moving | – |
| `s` | stop: end the run, coast | – |
| `X` | stop and brake | – |
| `P <pct>` | open-loop motor −100..100 | – |
| `l` / `c` / `r` | steer left / centre / right | – |
| `Z` | zero the encoder (not during a run) | – |
| `M <counts>` | counts per metre | `COUNTS_PER_METER` |
| `T <hz>` | telemetry rate, 0 = off | – |
| `G` | print the settings (two lines) | – |
| `I` | scan I2C1 and restart the OLED | – |
| `?` or `H` | help | – |

Nothing is stored in flash. After a reset the defaults from `car_config.c`
and the top of `car_drive.c` apply.

Replies start with `OK` or `ERR`. When a run ends, one line is printed:

```
DONE: 100.3 cm, past by 0.3 cm
DONE, not within tol in time: 98.2 cm, short by 1.8 cm
DONE, stopped after one swing back: 101.4 cm, past by 1.4 cm
ERR motor on but no encoder counts: wheel blocked or encoder unplugged?
ERR encoder counts the wrong way: change ENCODER_INVERT in car_config.h
```

`G` prints:

```
CPM=1035.0 Kp=3.00 Ki=2.00 Kd=0.00 tt=0.0 ta=2.0 td=2.0 vn=18.0 tol=1.0
motor ms=60.0 mo=45.1 mg=0.59 sonar=400cm batt=11.60V (PA4 1.76V x 6.58) counts=0
```

## Telemetry (`T <hz>`)

A header line, then CSV lines at the chosen rate on both ports:

```
t_ms,state,sp_cms,speed_cms,dist_cm,counts,motor_pct,ref_cm,sonar_cm,yaw_deg,servo_us
```

| Column | Source |
|---|---|
| `t_ms` | `HAL_GetTick()` |
| `state` | 0 off, 1 accel, 2 cruise, 3 decel, 4 settle |
| `sp_cms` | `SP_Vel`, the profile speed |
| `speed_cms` | `AverageSpeed`, filtered encoder speed |
| `dist_cm` | `Real_Dist`, distance since the run started |
| `counts` | encoder counts since the last zero |
| `motor_pct` | last commanded motor % (0 while braking or coasting) |
| `ref_cm` | `Pos_Ref`, where the car should be |
| `sonar_cm` | `sonarCm`, 400 = nothing seen |
| `yaw_deg` | during a run: degrees off the heading at the start (+ = left). Otherwise the total heading |
| `servo_us` | servo pulse, 1100 = centre |

Telemetry lines are dropped when the send buffer is full. Over Bluetooth
use `T 5`. The control loop runs at 10 Hz, so more than `T 10` only repeats
values.

`tools/car_console.py` starts a new file in `tools/logs/` at every header
line. `tools/plot_run.py` plots it. `tools/drive_sim` writes the same
columns (`csv=<file>`).

## STM32CubeMonitor

The globals in `car_drive.c` keep their names from Hakon's project so his
CubeMonitor flow still finds them.

- Write: `test_dist_m` (metres, > 0.01 starts a run, then resets to 0),
  `Kp`, `Ki`, `Kd`, `run_T`, `run_ta`, `run_td`.
- Watch: `state`, `time_ctl`, `SP_Vel`, `Pos_Ref`, `Real_Dist`,
  `Speed_CM_SEC`, `AverageSpeed`, `error`, `ErrorAcum`, `ErrorRate`,
  `position`, `sonarCm`, `obstacle`, `motorPct`.

Changed on 2026-11-11: `Real_Dist`, `position` and the speeds are now
signed (negative when the car moves backwards), and `state` can be 4.
