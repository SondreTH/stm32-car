# Activity 2.6, part 2: MATLAB / Simulink

Model identification, continuous and discrete simulation, and PID tuning for the
position loop in `Core/Src/car_drive.c`. The scripts use the same equations and
constants as the firmware (`car_params.m`), so the simulation matches the car.

**Needs:** MATLAB with Simulink, Control System Toolbox and System Identification
Toolbox (all in the Tec licence; MATLAB Online works too: upload this folder).

## 0. Simulated data (what the hand-in uses)

The real tests could not be recorded: the battery was flat (9.4 V of 12.6 V) and
the labs were closed until after the deadline. With the professor's agreement the
tests were simulated instead. `car_params.m` therefore points at `data_sim/`.

- `python/virtual_car.py` is a stand-in for the car: the plant
  `speed = K/(tau s + 1) * (motor% - p0)` with p0 = 50 % and K = 1.8 cm/s per %
  taken from the feed-forward Hakon tuned on the real car (commit b95dbe3), and an
  assumed tau = 0.25 s. It runs the firmware's own loop (100 ms interrupt, whole
  encoder counts, the 0.5/0.5 speed filter, ControlPID) and writes the same
  telemetry CSVs as the car, marked SIMULATED in their first lines.
- `python/analysis.py` does steps 1-4 in Python and writes the report figures
  (`report/figures`) and `python/results.json`.
- The MATLAB scripts below run unchanged on `data_sim/`, for the toolbox
  screenshots. step2 uses the same tuning target (phase margin 35 deg at 1.85 rad/s).

```sh
cd matlab/python
python virtual_car.py    # regenerate data_sim/ (needs nothing but Python)
python analysis.py       # needs numpy, scipy, matplotlib
```

To use real logs later: record them as below and set `p.data_dir = 'data'`.

## 1. Record the data on the car

Each log is one file in `matlab/data/`. Telemetry at 10 Hz (`T 10`) gives the
columns `ms,cnt,dist_cm,spd_cms,sp_cms,mot_pct,ref_cm,sonar_cm`, one row per
control period. Don't use `T 20` over Bluetooth: at 9600 baud a row takes
~47 ms to send, so 20 Hz keeps the main loop busy almost all the time.

**Logging over USB** (laptop next to the car, long cable):
```sh
picocom -b 115200 --logfile matlab/data/openloop_P70.csv /dev/ttyACM0
```
**Logging over Bluetooth from a Linux laptop:** pair the car's HC-05 (PIN 1234),
then use a small RFCOMM terminal that writes everything the car sends to the
file (Python `socket.AF_BLUETOOTH` works without sudo).

**Logging over Bluetooth from a phone:** the phone app (e.g. *Serial Bluetooth Terminal*) can
save the log to a file. Copy it here with the same name. Timestamps the app adds
are ignored.

### Open-loop steps (for step 1): one file per motor %
Car on the floor, at least 2 m of free space. For each of `P 60`, `P 70`, `P 80`, `P 90`:
```
T 10
(wait 1 s, car standing still)
P 70
(wait 3 s, it must reach a constant speed)
X
T 0
```
Save as `data/openloop_P60.csv`, `data/openloop_P70.csv`, ... At least 3 steps.
The car needs about 50 % to start moving, so `P 50` and lower show little.

### A closed-loop run (for step 4)
```
tt 8
ta 2
td 2
T 10
d1
(wait until it stops)
T 0
```
Save as `data/closedloop_run1.csv`, one run per file. Write down the gains (`G`
prints them) and the measured distance.

## 2. Run the scripts, in this order
Open MATLAB in this `matlab/` folder.

| Script | Does | Report / video |
|---|---|---|
| `step1_identify` | Finds the dead zone `p0` and static gain from the steps, then fits `G(s)` = speed / (motor % - p0) with `tfest` (1st order, 1st order + delay, 2nd order) | Open-loop plots, static characteristic, `compare` plot, the transfer function, K and tau. For screenshots of the System Identification app, run `systemIdentification` and import `dat` |
| `step2_tune_pid` | `pidtune` on P(s) = slope * G(s) / s, continuous and discrete (Ts = 0.1 s, backward Euler like `ControlPID()`) | Kp, Ki (Kd) table, phase margin, step response, Bode. Bandwidth vs sampling frequency |
| `step3_simulate` | A timed run (trapezoid + feed-forward + PID + actuator + encoder) continuous vs discrete, and Ts = 0.02 ... 0.4 s | Continuous vs discrete plot; table of error vs Ts to justify Ts = 0.1 s |
| `build_simulink_model` | Builds and runs `car_position_loop.slx`: the same two loops as Simulink blocks | Screenshot of the diagram and the scope |
| `step4_validate` | Real run vs discrete simulation | Car vs sim plot, final error. Put the CubeMonitor plot of the same run next to it |

Then send the gains from step 2 to the car over Bluetooth (`kp ...`, `ki ...`,
`kd ...`), and copy good ones into `car_drive.c` and `car_params.m`.

## The model in one picture

```
Pos_Ref --(+)--e--> PID --(+)--u--> actuator ---motor %---> car -----speed----> 1/s ---+--> position
          ^ -               ^       50 + 0.556*u           G(s) on                       |
          |                 |       max 100 %              (motor % - p0)                |
          |              SP_Vel                                                          |
          |          (feed-forward)                                                      |
          +---- encoder: whole counts (11.7 per cm), sampled every Ts = 0.1 s <----------+
```

## Files
| File | |
|---|---|
| `car_params.m` | Constants copied from the firmware: Ts, counts per metre, motor offset and slope, gains, the run |
| `load_telemetry.m` | Reads a picocom or Bluetooth log into a table |
| `trapezoid_ref.m` | The speed / position reference, as `Start_Timed_Profile()` |
| `sim_loop.m` | Simulates one run, continuous or discrete |
| `sim_setup.m` | Variables for the Simulink model (it runs this before each simulation) |
