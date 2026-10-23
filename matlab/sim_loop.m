function r = sim_loop(G, p0, p, gains, mode, Ts)
%SIM_LOOP Simulate one timed run of the position loop.
%   G      plant: (motor % - p0) -> speed (cm/s), from step1_identify
%   p0     motor % where the car starts to move, from step1_identify
%   p      car_params() (run settings, actuator, limits)
%   gains  struct with Kp, Ki, Kd
%   mode   'continuous': the PID runs all the time on the exact position
%          'discrete'  : the PID runs every Ts on the encoder count, like the car
%   Ts     sampling time for 'discrete' (s)
%
%   The controller is the one in car_drive.c:
%     e = Pos_Ref - Real_Dist
%     ErrorAcum = clamp(ErrorAcum + e*dt, +-i_limit)
%     u = SP_Vel + Kp*e + Ki*ErrorAcum + Kd*(e - e_last)/dt,  u >= u_min
%     motor % = MOTOR_MIN_PCT + MOTOR_PCT_PER_CMS*u   (max 100 %)
%   When the run time is over the motor is switched off (coast).

if nargin < 6, Ts = p.Ts; end
dt   = 1e-3;                                   % plant integration step (s)
t    = (0:dt:p.T + 2)';
[spd_ref, pos_ref] = trapezoid_ref(p.D_cm, p.T, p.ta, p.td, t);

plant = absorbDelay(c2d(ss(G), dt, 'zoh'));    % any dead time becomes states
x     = zeros(size(plant.A, 1), 1);
nTs   = max(1, round(Ts / dt));
discrete = strcmpi(mode, 'discrete');

N = numel(t);
[pos, spd, pct_log, err] = deal(zeros(N, 1));
p_now = 0; I = 0; e_last = 0; pct = 0; e = 0;
for k = 1:N
    if ~discrete || mod(k - 1, nTs) == 0       % controller update
        if discrete
            h = nTs * dt;
            meas = floor(p_now * p.cnt_cm) / p.cnt_cm;   % whole encoder counts only
        else
            h = dt;
            meas = p_now;
        end
        if t(k) <= p.T
            e = pos_ref(k) - meas;
            I = min(max(I + e * h, -p.i_limit), p.i_limit);
            D = (e - e_last) / h;
            e_last = e;
            u = max(spd_ref(k) + gains.Kp * e + gains.Ki * I + gains.Kd * D, p.u_min);
            pct = min(p.motor_min + p.motor_slope * u, 100);
        else
            pct = 0;                           % run over: motor off
        end
    end
    in = max(pct - p0, 0);                     % below p0 the car does not drive
    v  = plant.C * x + plant.D * in;
    x  = plant.A * x + plant.B * in;
    p_now = p_now + v * dt;

    pos(k) = p_now; spd(k) = v; pct_log(k) = pct; err(k) = e;
end

r = struct('t', t, 'pos', pos, 'spd', spd, 'pct', pct_log, 'err', err, ...
           'pos_ref', pos_ref, 'spd_ref', spd_ref, 'mode', mode, 'Ts', Ts);
run = t <= p.T;
r.rms_err   = sqrt(mean((pos_ref(run) - pos(run)).^2));   % tracking error during the run
r.max_err   = max(abs(pos_ref(run) - pos(run)));
r.final_err = p.D_cm - pos(end);                           % after the car has stopped
end
