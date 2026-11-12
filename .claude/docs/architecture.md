# Architecture

## Layout

```
Core/Src/main.c            generated; calls App_Init() once, App_Loop() forever,
                           and holds the two HAL callbacks (USER CODE 4)
Core/Src/car_app.c         application: commands, start button, OLED screen, telemetry
Core/Src/car_drive.c       distance run: profile, position PID, obstacle pause (TIM3 interrupt)
Core/Src/car_hw.c          motor, encoder, servo, battery ADC, buttons, I2C scan, sonar
Core/Src/car_oled.c        SSD1306 / SH1106 driver with framebuffer and 5x7 font
Core/Src/car_comms.c       UART ring buffers, line input, line builder for output
Core/Src/car_config.c      default values of everything that commands can change
Core/Inc/car_config.h      #define constants and the externs for car_config.c
stm32-car.ioc              CubeMX: pins, clocks, timers, interrupts
tools/                     laptop side: console, plots, drive_sim, build_check.sh
cad/  wiring/  docs/       printed parts, perfboard wiring guide, team setup guide
```

Dependencies go one way: `car_app` → `car_drive` → `car_hw` → HAL.
`car_comms` and `car_oled` only depend on the HAL. Everything reads
`car_config.h`.

`car_heading.c` / `.h` (added 2026-11-12): MPU-6050 gyro, heading, and the
steering that holds the heading during a run. `heading_poll()` is called
from `App_Loop()`.

## What runs where

| Context | Priority | Code | Period |
|---|---|---|---|
| Main loop | – | `App_Loop()`: `comms_poll` → `handle_command`, `button_poll`, run report, OLED redraw, telemetry, `comms_pump` | as fast as it goes; OLED redraw ~25 ms, every second control step (5 Hz) |
| TIM3 update interrupt | 2 | `HAL_TIM_PeriodElapsedCallback` → `drive_update()` | 100 ms |
| EXTI9_5 (PA7, both edges) | 1 | `HAL_GPIO_EXTI_Callback` → `sonar_echo_edge()` | per echo |
| USART1 / USART2 interrupts | 1 | HAL RX/TX → callbacks in `car_comms.c` | per byte / block |
| SysTick | 15 (HAL default) | `HAL_GetTick()` | 1 ms |

A lower number is a higher priority, so the sonar echo and the UARTs can
interrupt the control step. The control step itself takes microseconds.

`drive_update()` does, in order: CubeMonitor trigger → read encoder →
speed → sonar/obstacle bookkeeping and next TRIG pulse → (if a run is
active) safety checks → profile → PID → motor. See [control.md](control.md).

## Shared data between the interrupt and the main loop

- The run variables in `car_drive.c` are `volatile` globals. The main loop
  only *reads* them (OLED, telemetry, replies), with two exceptions:
  - `drive_start()` rewrites all of them. It masks `TIM3_IRQn` while it does.
  - `drive_cancel()` / `drive_stop()` / `Set_Speed_cm_s()` write `state`,
    `SP_Vel`, `openLoopCms`. Each is a single 32-bit store, which is atomic
    on a Cortex-M4.
- The values in `car_config.c` (`MOTOR_START_PCT`, `POS_TOL_CM`, …) are
  written by commands in the main loop and read in the interrupt. Single
  float stores, no locking needed.
- `driveSteps` counts control steps; the main loop uses it to redraw the
  OLED in step with the control loop.
- `drive_result()` hands the end-of-run reason from the interrupt to the
  main loop exactly once.
- `sonarCm` is written in the EXTI interrupt and in `sonar_trigger()` (TIM3
  interrupt), read in `drive_update()` and the main loop.
- `motorPct` (in `car_hw.c`) is the last commanded motor %. It is written
  from both contexts (manual commands in the main loop, the run in the
  interrupt). A manual command always calls `drive_cancel()` first, so the
  two never fight over the motor.

## Motor ownership

- `state != DRIVE_OFF`: the run owns the motor (interrupt).
- `state == DRIVE_OFF` and `openLoopCms != 0` (the `v` command): the
  interrupt applies the open-loop speed every step, with kick-start.
- Otherwise the motor keeps whatever the last manual command set
  (`f`, `b`, `P`, `1`..`6`, `s`, `X`). There is no timeout on manual
  commands: `f` drives until another command arrives.

## Comms

- Two ports, treated the same: USART2 (ST-LINK USB virtual COM, 115200) and
  USART1 (Bluetooth, 9600). Every output line goes to both. Input lines
  from either are handled by the same `handle_command()`.
- RX: one byte per interrupt into a 256-byte ring. `comms_poll()` splits
  lines on CR or LF (max 63 characters).
- TX: 1024-byte ring per port, sent by interrupt in blocks. `lb_send(true)`
  drops the line if the ring is full (telemetry); `lb_send(false)` waits up
  to 500 ms. At 9600 baud a telemetry line of ~45 characters takes ~47 ms,
  so more than about 10 Hz cannot get through Bluetooth.
- LD2 toggles on every Bluetooth byte and on every control step of a run.

## OLED

`oled_draw_status()` in `car_app.c` redraws the whole 128x64 framebuffer at
5 Hz; `oled_update()` sends it (blocking I2C, ~25 ms at 400 kHz). Layout:
title, battery volts and sonar cm on the top line (yellow rows 0-15 on
two-colour modules), then run phase and target, distance (big), motor %,
speed and setpoint, and a motor bar. If a transfer fails the OLED is marked
lost; the `I` command rescans I2C1 and restarts it.

## Start button

`button_poll()`: a press of B1 (PC13) or the PB0 button arms a run of
`BUTTON_RUN_M` (1 m) that starts `BUTTON_DELAY_MS` (1 s) later, without
blocking. Any manual command disarms it.
