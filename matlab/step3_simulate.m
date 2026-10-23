%% Step 3 - Continuous vs discrete simulation of a timed run, and the choice of Ts
% Input : model.mat, gains.mat
% Output: figures. The run (D, T, ta, td) is set in car_params.m.
%
% Both simulations include the actuator (offset, slope, 100 % limit, dead zone)
% and the trapezoidal reference with the feed-forward SP_Vel. The discrete one
% also includes the sampling (ZOH) and the encoder resolution, so it is the
% closest to what the car does. build_simulink_model.m builds the same loops
% in Simulink.

clear; close all; clc;
p = car_params();
load('model.mat', 'G', 'p0');
load('gains.mat', 'gc', 'gd');

rc = sim_loop(G, p0, p, gc, 'continuous');
rd = sim_loop(G, p0, p, gd, 'discrete', p.Ts);

fprintf('Run: %.0f cm in %.1f s (ta %.1f s, td %.1f s), Sn = %.1f cm/s\n', ...
        p.D_cm, p.T, p.ta, p.td, p.D_cm / (p.T - (p.ta + p.td) / 2));
fprintf('                 RMS error   max error   final error\n');
fprintf('Continuous      %7.2f cm  %7.2f cm  %7.2f cm\n', rc.rms_err, rc.max_err, rc.final_err);
fprintf('Discrete %.2fs  %7.2f cm  %7.2f cm  %7.2f cm\n', p.Ts, rd.rms_err, rd.max_err, rd.final_err);

figure('Name', 'Continuous vs discrete');
subplot(3, 1, 1); hold on; grid on;
plot(rc.t, rc.pos_ref, 'k--', rc.t, rc.pos, rd.t, rd.pos);
ylabel('position (cm)'); legend('reference', 'continuous', 'discrete', 'Location', 'southeast');
title(sprintf('Timed run: %.0f cm in %.1f s', p.D_cm, p.T));
subplot(3, 1, 2); hold on; grid on;
plot(rc.t, rc.spd_ref, 'k--', rc.t, rc.spd, rd.t, rd.spd);
ylabel('speed (cm/s)'); legend('SP\_Vel', 'continuous', 'discrete', 'Location', 'best');
subplot(3, 1, 3); hold on; grid on;
plot(rc.t, rc.pct, rd.t, rd.pct);
ylabel('motor %'); xlabel('t (s)'); legend('continuous', 'discrete', 'Location', 'best');

%% Choice of the sampling time: same gains, different Ts
Ts_list = [0.02 0.05 0.1 0.2 0.4];
fprintf('\nSampling time check (discrete gains from step2):\n   Ts (s)   RMS error   max error\n');
figure('Name', 'Sampling time'); hold on; grid on;
plot(rc.t, rc.pos_ref - rc.pos, 'k', 'DisplayName', 'continuous');
for Ts = Ts_list
    r = sim_loop(G, p0, p, gd, 'discrete', Ts);
    fprintf('   %5.2f   %7.2f cm  %7.2f cm\n', Ts, r.rms_err, r.max_err);
    plot(r.t, r.pos_ref - r.pos, 'DisplayName', sprintf('Ts = %.2f s', Ts));
end
xlabel('t (s)'); ylabel('position error (cm)'); legend('Location', 'best');
title('Tracking error for different sampling times');
fprintf('Pick the largest Ts whose error is close to the continuous one.\n');
fprintf('Next: build_simulink_model (Simulink diagram), then step4_validate with a real run\n');
