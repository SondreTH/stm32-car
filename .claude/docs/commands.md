# Commands, telemetry, CubeMonitor

## Serial commands (`handle_command` in `car_app.c`)

Both ports accept the same commands: USB (ST-LINK virtual COM, 115200) and
Bluetooth (9600). A line ends with LF or CR. The first letter is not case
sensitive. A number may follow directly or after a space.

The parser checks the two-letter settings first (`tt ta td tol kp ki kd ms
mo mg vn vb sp si sc kh pf`), then everything starting with `o` (the
open-loop commands, handled in `car_openloop.c`), then the single letters. So `T 5` is telemetry, `tt 5` is
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
| `Y` | zero the heading (and the three axis angles in `G`) | `yawDeg` |
| `cal` | steering calibration drive: holds the nominal speed and nine servo pulses for 3 s each, prints the turn radius of each. Needs the gyro and a clear strip 5 m x 2 m. `s` stops it | – |
| `ol` | show the open-loop plan | `olPlan[]` |
| `ol <n> <pct> <us> <s>` | set step n (1..12): motor % (0 = brake), steering offset from `sc` in µs (negative = left), time. Time 0 ends the plan before step n. Steps must be filled in order | `olPlan[]` |
| `or <n>` | play the plan n times (1..10, at most 48 steps in all) | `olRepeat` |
| `og` | run the plan 2 s later (`OL_DELAY_MS`) | – |
| `od` | print the recording of the last plan run as CSV, to the port that asked | – |
| `os 0/1` | sonar stop during the plan off / on (default on) | `olSonarStop` |
| `ob 0/1` | the start button runs a 1 m distance run / the plan (default the plan) | `olButton` |
| `ov <V>` | battery compensation of the plan: duty = pct x V / battery. `ov 0` = off (default) | `olVref` |
| `v <cm/s>` | hold a speed with the speed loop (negative = reverse), `v0` stops | – |
| `vs [cm/s]` | speed steps (`car_speedtest.c`): `VS_FIRST_CMS` 20, +10 each step up to `VS_TOP_CMS` 60 or the number, `VS_STEP_S` 3 s each, speed loop + gyro heading hold. Per step: mean speed, motor % and battery after `VS_SETTLE_S`. Sonar stop | – |
| `ll` | list the tests in the recorder (`car_log.c`). `d`, `go` and `vs` start a recording; it ends `LOG_IDLE_MS` after the car stands | – |
| `ld [n]` | download the last test, test n, or all (`ld 0`) to the port that asked: header, `# ` notes (the report lines), then samples. Columns: `t_ms` (since the test started), `state`, `step`, `motor_pct`, `servo_us`, `sonar_cm`, `sp_cms`, `speed_cms`, `dist_cm`, `head_err_deg`, `yaw_deg` (since the test started), `batt_v` | – |
| `lc` | clear the recorder | – |
| `f` / `b` | forward / back at `MANUAL_PCT` (60 %) | – |
| `1`..`6` | 50..100 % in the current direction, only while moving | – |
| `s` | stop: end the run, coast. During an open-loop plan: brake, and the report is still printed | – |
| `X` | stop and brake | – |
| `P <pct>` | open-loop motor −100..100 | – |
| `l` / `c` / `r` | steer left / centre / right | – |
| `Z` | zero the encoder (not during a run) | – |
| `M <counts>` | counts per metre | `COUNTS_PER_METER` |
| `T <hz>` | telemetry rate, 0 = off | – |
| `G` | print the settings (three lines) | – |
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

`G` prints (` LOW!` after the battery when it is under `BATT_LOW_V`):

```
CPM=1035.0 Kp=3.00 Ki=2.00 Kd=0.00 tt=0.0 ta=2.0 td=2.0 vn=18.0 tol=1.0 sp=1.00 si=1.50
steer sc=1080 kh=15.0 yaw=0.3deg rate=0.0dps bias=-4.42 reads ok=5655 failed=0 axes since Y: x=90 y=0 z=1
motor ms=30.0 mo=22.0 mg=0.50 pf=100 sonar=400cm batt=11.60V (PA4 1.76V x 6.58) counts=0
```

`reads ok` / `failed` count gyro readings over I2C since start-up. `axes
since Y` is the angle around each gyro axis since the last `Y`: turn the
car 90 degrees by hand and the axis that shows ±90 is the one pointing up
(`IMU_YAW_AXIS`). On 2026-11-22 that was X: the module stands on its side.

`cal` prints one line per servo pulse, then `cal done`:

```
cal 1140 us: -9.8 deg in 44 cm, radius 257 cm right
```

Below `BATT_LOW_V` (9.9 V) for 3 s, every 30 s:

```
WARNING battery low: 9.6 V. Stop and charge it (3S LiPo: not below 9.9 V)
```

## Open-loop report and recording

After a plan (end of the plan, a safety stop, `s` or the button):

```
OPEN LOOP done (end of the plan): 11.8 s, 831 cm, turned -359 deg. Battery 11.18 V, servo centre 1080 us
 step 3: 46 % steer 350 us for 1.50 s -> 105 cm, -84.4 deg (-56.3 deg/s, radius 71 cm)
 ...
 end (encoder + gyro estimate): 9 cm behind, 14 cm right of the start = 17 cm away
od = print the recording (132 samples)
```

With `or` above 1 the step lines start with `lap n`. `(cut short)` marks the
step a safety stop ended. A `gyro: N of M readings failed` line appears if
any gyro reading failed. Reasons in the first line: `end of the plan`,
`sonar: something in front`, `motor on but the wheel does not turn`,
`stopped by s`, `stopped by the button`, `a command took over`, `a distance
run started`.

`od` prints, without dropping lines (it waits for room in the send queue):

```
t_ms,step,motor_pct,servo_us,speed_cms,dist_cm,yaw_deg,x_cm,y_cm,sonar_cm,batt_v
# plan step 1: 55 % steer 0 us for 0.30 s
# played 1 times, the step column counts all 11
0,1,55,1080,0.0,0.0,0.0,0.0,0.0,400,11.18
...
# OPEN LOOP done (...)            (the report again)
OK dump done: 132 samples
```

`t_ms` since the start of the plan, `step` 1.. (0 = after the plan),
`yaw_deg` since the start (+ = left), `x_cm` forward at the start, `y_cm`
left, `batt_v` smoothed. Telemetry (`T`) pauses during a dump. Over
Bluetooth a 30 s run takes about 30 s to dump.

## Telemetry (`T <hz>`)

A header line, then CSV lines at the chosen rate on both ports:

```
t_ms,state,sp_cms,speed_cms,dist_cm,counts,motor_pct,ref_cm,sonar_cm,yaw_deg,servo_us,batt_v
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
| `servo_us` | servo pulse, 1080 = centre |
| `batt_v` | battery voltage, smoothed over about half a second |

Telemetry lines are dropped when the send buffer is full. Over Bluetooth
use `T 5` (the Bluetooth port sends about 400 bytes/s, see
[hardware.md](hardware.md#bluetooth)). The control loop runs at 10 Hz, so more than `T 10` only repeats
values.

`tools/car_console.py` starts a new file in `tools/logs/` at every header
line (also the `od` header; `#` lines go into the file too). On Linux it
connects to Bluetooth by address: `python tools/car_console.py
98:DA:20:0C:1A:63`. `tools/plot_openloop.py` plots `od` recordings. `tools/plot_run.py` plots it. `tools/drive_sim` writes the same
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
