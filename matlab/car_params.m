function p = car_params()
%CAR_PARAMS Values copied from the firmware, so the simulation matches the car.
%   If someone changes these in the C code, change them here too.

% ---- control loop (main.c, car_config.h) ----
p.Ts = 0.100;                 % DRIVE_PERIOD_MS: TIM3 interrupt every 100 ms (s)

% ---- encoder (car_config.c) ----
p.cpm    = 1170;              % COUNTS_PER_METER
p.cnt_cm = p.cpm / 100;       % counts per cm: the encoder's resolution

% ---- actuator: motor % = MOTOR_MIN_PCT + MOTOR_PCT_PER_CMS * command (car_config.h) ----
p.motor_min   = 7200 / 144;   % MOTOR_MIN_PCT (%)
p.motor_slope = 80 / 144;     % MOTOR_PCT_PER_CMS (% per cm/s)
p.u_min       = 1.0;          % command is clamped to >= 1 cm/s during a run (car_drive.c)

% ---- PID (car_drive.c): output = Kp*e + Ki*ErrorAcum + Kd*(e - e_last)/Ts ----
p.Kp = 3.8;
p.Ki = 2.5;
p.Kd = 0.05;
p.i_limit = 25;               % anti-windup: ErrorAcum is clamped to +-25 cm*s

% ---- the run to simulate: the d, tt, ta, td commands ----
p.D_cm = 100;                 % d1   -> 100 cm
p.T    = 8;                   % tt 8 -> total time (s)
p.ta   = 2;                   % ta 2 -> acceleration time (s)
p.td   = 2;                   % td 2 -> deceleration time (s)
end
