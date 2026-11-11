# History and branches

Remote: `https://github.com/SondreTH/stm32-car.git` (Sondre's repository).
Team: Filip (`filip-rs`), Sondre (`SondreTH`), Hakon (`hako0025`).

## Timeline

| Date | Commit | What |
|---|---|---|
| 2026-09-29 | `90d56a1` | Hakon: first upload |
| 2026-10-01/02 | `a07a23f`, `822daea` | Sondre: complete rewrite into `car_project/` with the `Team7_Car` CubeIDE project (gyro odometry, teach-and-repeat, pure pursuit, obstacle reroute), laptop tools, CAD, wiring guide |
| 2026-10-05..07 | `a701e07`..`77118af` | Sondre, on `main`: README, IN1/IN2 swapped to PB4/PB5, Arduino firmware removed. **`main` stopped here** |
| 2026-10-08 | `4163333` | Hakon: his own CubeIDE project `Hakons_leg` at the repo root (replaces `car_project/` on this line of history) |
| 2026-10-08 | `bfcafb8`..`6a1a9da` | branch `stm32-car`: build output untracked, project renamed to `stm32-car`, setup README |
| 2026-10-08 | `24bb1e2` | Filip, `activity2.5`: motor + encoder base in the `car_*` file structure |
| 2026-10-14 | `74a4926` | Filip, `activity2.6`: OLED |
| 2026-10-14..21 | `fee7235`..`e9cd009` | Hakon, `hakon-backup`: cm/s speed profile, PID speed loop, run without Bluetooth (CubeMonitor trigger, buttons), all in `main.c` |
| 2026-10-21 | `13bfc26` | Filip, `oled`: Hakon's `main.c` ported into `car_drive.c` / `car_app.c`, more OLED |
| 2026-10-21 | `cafffb9`, `debfe04` | Sondre, `oled-timer-pid-timed-obstacle-bluetooth` (Activity 2.6): TIM3 control interrupt, position PID, timed runs, sonar obstacle stop |
| 2026-11-11 | branch `integration` | everything merged for `main`, new position-loop logic, battery calibration, this documentation |

## Branches

| Branch | State |
|---|---|
| `main` | until `integration` is merged: the old `car_project/` tree from 2026-10-07 |
| `integration` | made 2026-11-11 from `oled`. Meant to be merged into `main` by pull request |
| `oled` | Filip's branch. Locally it was at `debfe04`, the same commit as Sondre's branch |
| `oled-timer-pid-timed-obstacle-bluetooth` | Sondre's Activity 2.6 work, fully contained in `integration` |
| `hakon-backup` | Hakon's single-file project. His code was ported by hand; the branch is recorded as merged |
| `stm32-car` | project setup + the team setup README (now `docs/setup.md`) |
| `activity2.5`, `activity2.6` | class hand-ins, ancestors of `oled` |

## How `integration` was built

1. Branched from `oled` (`debfe04`) and committed Filip's pending edits
   (I2C1 shared by OLED and MPU-6050 in the docs; run variables `volatile`).
2. `git merge -s ours origin/hakon-backup`. Hakon's latest `main.c`
   (`e9cd009`) had already been ported line by line into `car_drive.c`, and
   his branch tracks `Debug/` build output, so no files were taken. The
   merge only makes Git show the branch as merged.
3. `git merge origin/main`. The conflict was structural: `main` had
   `car_project/`, this line had deleted it. Decision (Filip, 2026-11-11):
   keep the root-level `stm32-car` project as the only firmware, drop the
   old `Team7_Car` firmware, and move `cad/`, `wiring/` and `tools/` to the
   root. The autopilot-only tools went to `tools/autopilot/`.
4. New work on top: position loop with kick-start, anti-windup and settle
   (see [control.md](control.md)), reverse runs, battery divider
   calibration, `tools/drive_sim`, `tools/build_check.sh`, updated
   `plot_run.py`, new README, `docs/setup.md`, `.claude/`.

After the pull request, `main` can fast-forward to `integration`: it
already contains `main`'s history.

## The old autopilot firmware (`Team7_Car`)

Removed from the tree, kept in history at `77118af` (the last `main`
commit before the merge). It is the reference for the features still
missing. It was written quickly and, as far as we know, never tuned on the
real car. It uses metres and m/s, not cm.

```
git ls-tree -r --name-only 77118af car_project/cubeide/Team7_Car/Core
git show 77118af:car_project/cubeide/Team7_Car/Core/Src/car_hw.c      # MPU-6050 driver, gyro calibration, odometry inputs
git show 77118af:car_project/cubeide/Team7_Car/Core/Src/car_drive.c   # PI speed loop, steering angle -> servo
git show 77118af:car_project/cubeide/Team7_Car/Core/Src/path.c        # path storage, TEACH recording
git show 77118af:car_project/cubeide/Team7_Car/Core/Src/autopilot.c   # pure pursuit, waypoint stops, obstacle reroute
git show 77118af:car_project/cubeide/Team7_Car/Core/Src/car_app.c     # commands REC / WP / END / GO / DUMP ...
git show 77118af:car_project/README.md                                # user guide: bring-up, teaching a course
```

`tools/autopilot/sim.cpp` and `path_builder.py` belong to that firmware:
the simulator compiles `path.c` and `autopilot.c` from it, and the path
builder writes its `path_data.h`.
