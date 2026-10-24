%% Step 1 - Open-loop measurements and identification of the plant model
% Input : <p.data_dir>/openloop_P<pct>.csv, one motor step per file (see README.md)
% Output: model.mat with G (plant), p0, k_static. Figures for the report.
%
% The car is driven open loop with "P <pct>" (no PID). For each file this
% script finds the step, measures the steady-state speed, and then uses the
% System Identification Toolbox (tfest) to fit transfer functions
%       speed(s) / (motor%(s) - p0) = G(s)
% where p0 is the motor % below which the car does not move (friction).

clear; close all; clc;
p = car_params();

files = dir(fullfile(p.data_dir, 'openloop_*.csv'));
assert(~isempty(files), 'No open-loop logs: put them in %s/openloop_P<pct>.csv (see README.md)', p.data_dir);

%% 1. Cut out each step and resample it on the 100 ms control period
seg = {};
for i = 1:numel(files)
    tel = load_telemetry(fullfile(files(i).folder, files(i).name));
    on  = find(tel.mot_pct > 0, 1);                 % the "P" command
    assert(~isempty(on), '%s: the motor never turned on', files(i).name);
    P   = tel.mot_pct(on);
    off = find(tel.mot_pct(on:end) ~= P, 1) + on - 2;   % last row before X / s / next P
    if isempty(off), off = height(tel); end
    first = find(tel.t >= tel.t(on) - 1, 1);        % keep 1 s of standstill before it
    s = tel(first:off, :);
    assert(s.t(end) - tel.t(on) >= 2, '%s: the step is shorter than 2 s, drive longer', files(i).name);

    tg = (s.t(1):p.Ts:s.t(end))';
    seg{end+1} = struct('name', files(i).name, 'P', P, 't', tg - tg(1), ...
        'u', interp1(s.t, s.mot_pct, tg, 'previous'), ...
        'v', interp1(s.t, s.spd_cms, tg, 'linear'));      %#ok<SAGROW>
end

%% 2. Steady-state speed vs motor %: dead zone p0 and static gain
P   = cellfun(@(c) c.P, seg);
vss = cellfun(@(c) mean(c.v(c.t >= c.t(end) - 1)), seg);  % mean of the last second
if numel(unique(P)) >= 2
    c  = polyfit(P, vss, 1);              % vss = k*(P - p0)
    k_static = c(1);
    p0 = -c(2) / c(1);
else
    p0 = p.motor_min;                     % one step only: use the firmware's offset
    k_static = vss / (P - p0);
    warning('Only one motor %% measured: p0 taken from car_params. Record 3+ steps.');
end

fprintf('\n--- Open-loop measurements ---\n');
fprintf('  motor %%   steady speed (cm/s)\n');
fprintf('  %6.1f    %6.1f\n', [P; vss]);
fprintf('Dead zone p0 = %.1f %%   static gain = %.3f cm/s per %%\n', p0, k_static);
fprintf('Firmware equivalent: MOTOR_MIN_PCT = %.1f, MOTOR_PCT_PER_CMS = %.3f (now %.1f, %.3f)\n', ...
        p0, 1 / k_static, p.motor_min, p.motor_slope);

figure('Name', 'Open-loop steps');
subplot(1, 2, 1); hold on; grid on;
for i = 1:numel(seg)
    plot(seg{i}.t, seg{i}.v, 'DisplayName', sprintf('P %.0f', seg{i}.P));
end
xlabel('t (s)'); ylabel('speed (cm/s)'); title('Open-loop step responses'); legend('Location', 'best');
subplot(1, 2, 2); hold on; grid on;
plot(P, vss, 'o', 'DisplayName', 'measured');
Pf = linspace(min(p0, min(P)), max(P), 50);
plot(Pf, k_static * (Pf - p0), '-', 'DisplayName', sprintf('fit: %.2f (P - %.1f)', k_static, p0));
xlabel('motor %'); ylabel('steady speed (cm/s)'); title('Static characteristic'); legend('Location', 'best');

%% 3. Identification: speed / (motor % - p0)
dat = [];
for i = 1:numel(seg)
    d = iddata(seg{i}.v, max(seg{i}.u - p0, 0), p.Ts, 'ExperimentName', seg{i}.name);
    d.InputName = 'motor% - p0';  d.InputUnit  = '%';
    d.OutputName = 'speed';       d.OutputUnit = 'cm/s';
    if isempty(dat), dat = d; else, dat = merge(dat, d); end
end

opt = tfestOptions('InitialCondition', 'zero');   % every step starts at standstill
G1  = tfest(dat, 1, 0, opt);          % K / (tau s + 1)
G1d = tfest(dat, 1, 0, NaN, opt);     % K e^(-Ls) / (tau s + 1), dead time estimated
G2  = tfest(dat, 2, 0, opt);          % second order
models = {G1, G1d, G2};
names  = {'1st order', '1st order + delay', '2nd order'};

fits = zeros(1, 3);
for i = 1:3
    [~, f] = compare(dat, models{i});
    if iscell(f), f = cell2mat(f); end
    fits(i) = mean(f);
end
% the simplest model, unless another one fits clearly (3 %) better
best = 1;
for i = 2:3
    if fits(i) > fits(best) + 3, best = i; end
end
G = models{best};

fprintf('\n--- Identification (fit to the measured speed) ---\n');
for i = 1:3, fprintf('  %-18s fit = %5.1f %%\n', names{i}, fits(i)); end
fprintf('Chosen: %s\n', names{best});
G   %#ok<NOPTS> % show the transfer function

K   = dcgain(G);
tau = max(-1 ./ real(pole(G)));      % slowest time constant
fprintf('Gain K = %.3f cm/s per %%,  time constant tau = %.3f s,  dead time = %.3f s\n', K, tau, G.IODelay);
fprintf('Ts = %.2f s is tau/%.1f (rule of thumb: Ts <= tau/4 ... tau/10 for the speed,\n', p.Ts, tau / p.Ts);
fprintf('the position loop is slower, see step2/step3 for the check against the closed loop)\n');

figure('Name', 'Identification');
compare(dat, G1, G1d, G2);
title('Measured speed vs identified models');

% To do the same in the System Identification app (screenshots for the report):
%   systemIdentification   -> Import data -> "Data object" -> dat
%   Estimate -> Transfer Function Models -> 1 pole, 0 zeros
% systemIdentification

save('model.mat', 'G', 'G1', 'G1d', 'G2', 'p0', 'k_static', 'dat', 'P', 'vss');
fprintf('\nSaved model.mat. Next: step2_tune_pid\n');
