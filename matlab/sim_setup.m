%% Variables for car_position_loop.slx. The model runs this before every
% simulation (InitFcn), so you can change gains here or in car_params.m and
% press Run in Simulink.

p = car_params();
load('model.mat', 'G', 'p0');
load('gains.mat', 'gc', 'gd');

% plant: (motor % - p0) -> speed (cm/s)
[G_num, G_den] = tfdata(G, 'v');
G_delay = G.IODelay;

% PID gains: continuous loop (_c) and discrete loop (_d)
Kp_c = gc.Kp;  Ki_c = gc.Ki;  Kd_c = gc.Kd;
Kp_d = gd.Kp;  Ki_d = gd.Ki;  Kd_d = gd.Kd;
Ts   = p.Ts;

% trapezoidal reference: position (cm) and speed SP_Vel (cm/s)
t_ref = (0:1e-3:p.T)';
[v_ref, x_ref] = trapezoid_ref(p.D_cm, p.T, p.ta, p.td, t_ref);
ref_pos = timeseries(x_ref, t_ref);
ref_spd = timeseries(v_ref, t_ref);
