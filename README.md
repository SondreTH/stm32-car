# Activity 2.5: time interrupts and motor speed

This branch, `activity2.5`, holds a stripped copy of the Team 7 car firmware
(the `Team7_Car` project from the old `main` branch). It keeps only what this
activity needs. The full car code is on the `stm32-car` branch and is
untouched.

**Kept:**
- Motor: TIM1 PWM on PA8, IN1/IN2 on PB5/PB4.
- Encoder: TIM5 in encoder mode on PA0/PA1.
- USB and Bluetooth commands.
- Blue button (B1) and LD2.

**Removed:**
- Servo, sonar, MPU-6050, battery ADC, start button.
- Speed PI loop, autopilot, path.

The wiring is the same as the soldered perfboard.

## Get it

You need the normal team setup first. If you haven't done it, follow
**First-time setup** in the
[README on the `stm32-car` branch](https://github.com/SondreTH/stm32-car/tree/stm32-car#first-time-setup).

Then, in your `stm32-car` folder:

1. Commit or push your changes first. Git won't switch branches if they
   would be overwritten.
2. Run:
   ```
   git fetch
   git switch activity2.5
   ```
3. In CubeIDE, right-click `stm32-car` and choose **Refresh** (F5).
4. Go to **Project → Clean…**, select `stm32-car`, then click **Clean**.
5. Build and run as usual.

To go back to the full car code, run `git switch stm32-car` and repeat steps
3–5.

The project is the same `stm32-car` project on both branches, so there's
nothing new to import. The team rules in the `stm32-car` README apply here
too. Most importantly, in generated files only write code between
`USER CODE BEGIN` and `USER CODE END`.

## Commands (USB 115200 / Bluetooth 9600, line ending LF)

| Command | Does |
|---|---|
| `P 40` | Open-loop motor %, −100..100 |
| `X` | Stop and brake |
| `Z` | Zero the encoder |
| `C 20000` | Set counts per metre |
| `T 5` | Telemetry at 5 Hz (`T 0` = off) |
| `G` / `?` | Settings / help |

## What the activity adds (not done here)

- [ ] **Timer for the update interrupt.** TIM1 and TIM5 are used; TIM3 is free. APB1 timer clock = 84 MHz.
- [ ] **NVIC.** Enable the timer's global interrupt and take screenshots.
- [ ] **Start the timer.** `HAL_TIM_Base_Start_IT()` in USER CODE 2.
- [ ] **Callbacks.** `HAL_TIM_PeriodElapsedCallback()` in USER CODE 4: counts → position and speed in engineering units.
- [ ] **Blue button.** B1 (PC13) → EXTI13 → zero the counter, enabling EXTI line[15:10].
- [ ] **Measurements.** Minimum speed command (`P 5`, `P 10`, …) and maximum speed (`P 100`).
- [ ] **Monitoring.** Debug Live Expressions, then STM32CubeMonitor graphs.

## Numbers from the slides (M2.7–M2.9)

- **Timer frequency:** F = F_clk / (PSC × ARR).
  - 84 MHz / (8400 × 1000) = 10 Hz, so the period is 100 ms.
  - Enter `8400-1` and `1000-1` in CubeMX.
- **Validation:** toggle LD2 every interrupt. 100 ms high + 100 ms low gives 5 Hz at 50 % duty.
- **Speed:** RPM = (Δcounts / counts per rev) / Δt × 60.
  - Slide check: 30 pulses/rev, 100 ms, count 300 → 450.
- **Counting mode:** TIM5 counts ×4, on every edge of A and B. The slides' Option A counts ×1, on the falling edge of A only.
- **Counts per rev:** 4 × PPR × gear ratio. To measure it, send `Z`, turn the wheel 10 times by hand, then divide the count by 10.
