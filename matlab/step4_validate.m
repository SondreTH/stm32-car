%% Step 4 - Validate: a real run on the car vs the discrete simulation
% Input : data/closedloop_*.csv (a "d" run with telemetry on), model.mat
% Output: figures and errors for the report
%
% Set run (D, T, ta, td) and gains below to what was on the car for that run.
% They default to car_params.m.

clear; close all; clc;
p = car_params();
load('model.mat', 'G', 'p0');

file = 'data/closedloop_run1.csv';   % <- the run to compare
% p.D_cm = 100; p.T = 8; p.ta = 2; p.td = 2;     % <- if the run used other settings
gains = struct('Kp', p.Kp, 'Ki', p.Ki, 'Kd', p.Kd);  % <- the gains on the car (kp/ki/kd)

tel = load_telemetry(file);

% the run starts when the reference starts to move
st = find(tel.ref_cm > 0, 1);
assert(~isempty(st), '%s: no run found (ref_cm stays 0). Was it a "d" run with T 10 on?', file);
st = max(st - 1, 1);
rl = tel(st:end, :);
rl.t = rl.t - rl.t(1);

fprintf('Log: run of %.1f cm (car_params: %.0f cm)\n', max(rl.ref_cm), p.D_cm);
if abs(max(rl.ref_cm) - p.D_cm) > 1
    warning('The logged run distance differs from p.D_cm: set the run settings above.');
end

rs = sim_loop(G, p0, p, gains, 'discrete', p.Ts);

final_real = rl.dist_cm(end);
fprintf('                     final position   error\n');
fprintf('Car                  %7.1f cm     %6.1f cm\n', final_real, p.D_cm - final_real);
fprintf('Simulation           %7.1f cm     %6.1f cm\n', rs.pos(end), rs.final_err);
inrun = rl.t <= p.T;
fprintf('Car: RMS tracking error %.2f cm, max %.2f cm\n', ...
        sqrt(mean((rl.ref_cm(inrun) - rl.dist_cm(inrun)).^2)), max(abs(rl.ref_cm(inrun) - rl.dist_cm(inrun))));
fprintf('Sim: RMS tracking error %.2f cm, max %.2f cm\n', rs.rms_err, rs.max_err);

figure('Name', 'Validation');
subplot(3, 1, 1); hold on; grid on;
plot(rs.t, rs.pos_ref, 'k--', rl.t, rl.dist_cm, 'o-', rs.t, rs.pos, 'MarkerSize', 3);
ylabel('position (cm)'); legend('reference', 'car', 'simulation', 'Location', 'southeast');
title(sprintf('Car vs simulation: %.0f cm in %.1f s, Kp %.2f Ki %.2f Kd %.2f', ...
      p.D_cm, p.T, gains.Kp, gains.Ki, gains.Kd));
subplot(3, 1, 2); hold on; grid on;
plot(rs.t, rs.spd_ref, 'k--', rl.t, rl.spd_cms, 'o-', rs.t, rs.spd, 'MarkerSize', 3);
ylabel('speed (cm/s)'); legend('SP\_Vel', 'car', 'simulation', 'Location', 'best');
subplot(3, 1, 3); hold on; grid on;
plot(rl.t, rl.mot_pct, 'o-', rs.t, rs.pct, 'MarkerSize', 3);
ylabel('motor %'); xlabel('t (s)'); legend('car', 'simulation', 'Location', 'best');
