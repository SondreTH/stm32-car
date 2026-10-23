%% Step 2 - Tune the PID (continuous and discrete) on the identified model
% Input : model.mat (step1_identify)
% Output: gains.mat with gc (continuous) and gd (discrete) Kp, Ki, Kd
%
% Plant seen by the PID (all in the units of car_drive.c):
%   command u (cm/s) -> motor % = MOTOR_MIN_PCT + slope*u -> speed = G(s)*(motor% - p0)
%   -> position = speed / s   (cm)
% The constant offsets only shift the operating point, so the linear plant is
%   P(s) = slope * G(s) / s
% The feed-forward SP_Vel is left out of the tuning: the PID only has to fix
% what the feed-forward gets wrong.

clear; close all; clc;
p = car_params();
load('model.mat', 'G');

use_kd = false;   % the assignment asks for Kp and Ki; set true to also tune Kd
wc     = [];      % target crossover (rad/s); [] lets pidtune choose. Larger = faster, less margin

Pc = p.motor_slope * G * tf(1, [1 0]);
Pc.InputName = 'u (cm/s)';  Pc.OutputName = 'position (cm)';
Pd = c2d(Pc, p.Ts, 'zoh');       % what the controller sees: ZOH on the motor, sampled every Ts

type = 'PI'; if use_kd, type = 'PID'; end

%% Continuous PID
if isempty(wc), Cc = pidtune(Pc, type); else, Cc = pidtune(Pc, type, wc); end

%% Discrete PID, same equations as ControlPID():
%   ErrorAcum += e*Ts          -> integral, backward Euler
%   (e - lastError)/Ts         -> derivative, backward Euler, no filter
C0 = pid(1, 1, double(use_kd), 0, p.Ts, 'IFormula', 'BackwardEuler', 'DFormula', 'BackwardEuler');
if isempty(wc), Cd = pidtune(Pd, C0); else, Cd = pidtune(Pd, C0, wc); end

%% The gains in the firmware now, for comparison
Cfw = pid(p.Kp, p.Ki, p.Kd, 0, p.Ts, 'IFormula', 'BackwardEuler', 'DFormula', 'BackwardEuler');

fprintf('\n--- PID gains (e in cm, output in cm/s) ---\n');
fprintf('                Kp       Ki       Kd    phase margin  crossover\n');
show = {'Continuous', Cc, Pc; 'Discrete', Cd, Pd; 'Firmware now', Cfw, Pd};
for i = 1:size(show, 1)
    [~, Pm, ~, Wcp] = margin(show{i, 2} * show{i, 3});
    fprintf('%-13s %7.3f  %7.3f  %7.3f    %6.1f deg   %5.2f rad/s\n', ...
        show{i, 1}, show{i, 2}.Kp, show{i, 2}.Ki, show{i, 2}.Kd, Pm, Wcp);
end

% Is Ts = 0.1 s fast enough? The sampling frequency should be 10-30x the
% closed-loop bandwidth.
wb = bandwidth(feedback(Cc * Pc, 1));
fprintf('\nClosed-loop bandwidth %.2f rad/s; sampling at %.1f rad/s is %.0fx faster\n', ...
        wb, 2 * pi / p.Ts, (2 * pi / p.Ts) / wb);

figure('Name', 'Closed-loop step response');
step(feedback(Cc * Pc, 1), feedback(Cd * Pd, 1), feedback(Cfw * Pd, 1), 6);
legend('continuous', sprintf('discrete Ts = %.2f s', p.Ts), 'firmware gains', 'Location', 'best');
title('Position step, 1 cm (no feed-forward)'); grid on;

figure('Name', 'Open-loop Bode');
margin(Cd * Pd); grid on;

gc = struct('Kp', Cc.Kp, 'Ki', Cc.Ki, 'Kd', Cc.Kd);
gd = struct('Kp', Cd.Kp, 'Ki', Cd.Ki, 'Kd', Cd.Kd);
save('gains.mat', 'gc', 'gd', 'Cc', 'Cd', 'Pc', 'Pd');

fprintf('\nSaved gains.mat. On the car, over Bluetooth:\n  kp %.2f\n  ki %.2f\n  kd %.2f\n', gd.Kp, gd.Ki, gd.Kd);
fprintf('Next: step3_simulate\n');
