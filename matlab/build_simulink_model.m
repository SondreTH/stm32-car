%% Build car_position_loop.slx: the continuous and the discrete position loop
% Input : model.mat, gains.mat (step1, step2)
% Output: car_position_loop.slx, simulated once, with a plot
%
% Top row    = continuous PID on the exact position.
% Bottom row = discrete PID every Ts on the encoder count (ZOH + quantizer),
%              the same as ControlPID() in car_drive.c.
% Both rows:   e -> PID (integral clamped to +-25) -> + SP_Vel (feed-forward)
%              -> u >= 1 cm/s -> actuator: motor % = MOTOR_MIN_PCT + slope*u (max 100)
%              -> car: speed = G(s)*(motor % - p0) -> position = integral of speed
% Run this once; after that, open the .slx and press Run. The model runs
% sim_setup.m before each simulation to load the gains and the reference.

clear; close all; clc;
sim_setup;                       % puts every variable the blocks use in the workspace

mdl = 'car_position_loop';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl);
set_param(mdl, 'InitFcn', 'sim_setup', 'SolverType', 'Fixed-step', 'Solver', 'ode4', ...
          'FixedStep', '1e-3', 'StopTime', 'p.T');

%% Reference (shared by both loops)
blk(mdl, 'Ref position', 'simulink/Sources/From Workspace', [30 290], ...
    'VariableName', 'ref_pos', 'OutputAfterFinalValue', 'Holding final value');
blk(mdl, 'Ref speed SP_Vel', 'simulink/Sources/From Workspace', [30 370], ...
    'VariableName', 'ref_spd', 'OutputAfterFinalValue', 'Holding final value');

%% The two loops
for row = 1:2
    if row == 1, s = 'C '; y = 150; else, s = 'D '; y = 470; end
    disc = row == 2;
    k = @(n) [s n];                                  % block name in this row

    % error = reference - measured position
    blk(mdl, k('Error'), 'simulink/Math Operations/Sum', [280 y], 'Inputs', '+-', 'IconShape', 'rectangular');

    % PID: Kp*e + Ki*integral(e) + Kd*de/dt
    if ~disc
        blk(mdl, k('Kp'), 'simulink/Math Operations/Gain', [430 y-70], 'Gain', 'Kp_c');
        blk(mdl, k('Integral e'), 'simulink/Continuous/Integrator', [380 y], ...
            'LimitOutput', 'on', 'UpperSaturationLimit', 'p.i_limit', 'LowerSaturationLimit', '-p.i_limit');
        blk(mdl, k('Ki'), 'simulink/Math Operations/Gain', [480 y], 'Gain', 'Ki_c');
        blk(mdl, k('derivative'), 'simulink/Continuous/Derivative', [380 y+70]);
        blk(mdl, k('Kd'), 'simulink/Math Operations/Gain', [480 y+70], 'Gain', 'Kd_c');
    else
        blk(mdl, k('Kp'), 'simulink/Math Operations/Gain', [430 y-70], 'Gain', 'Kp_d');
        blk(mdl, k('Integral e'), 'simulink/Discrete/Discrete-Time Integrator', [380 y], ...
            'IntegratorMethod', 'Integration: Backward Euler', 'SampleTime', 'Ts', ...
            'LimitOutput', 'on', 'UpperSaturationLimit', 'p.i_limit', 'LowerSaturationLimit', '-p.i_limit');
        blk(mdl, k('Ki'), 'simulink/Math Operations/Gain', [480 y], 'Gain', 'Ki_d');
        blk(mdl, k('derivative'), 'simulink/Discrete/Discrete Transfer Fcn', [380 y+70], ...
            'Numerator', '[1 -1]/Ts', 'Denominator', '[1 0]', 'SampleTime', 'Ts');
        blk(mdl, k('Kd'), 'simulink/Math Operations/Gain', [480 y+70], 'Gain', 'Kd_d');
    end
    blk(mdl, k('PID'), 'simulink/Math Operations/Sum', [580 y], 'Inputs', '+++', 'IconShape', 'rectangular');
    blk(mdl, k('+ feed-forward'), 'simulink/Math Operations/Sum', [680 y], 'Inputs', '++', 'IconShape', 'rectangular');
    blk(mdl, k('u min 1 cm per s'), 'simulink/Discontinuities/Saturation', [780 y], ...
        'UpperLimit', 'inf', 'LowerLimit', 'p.u_min');

    % actuator: command (cm/s) -> motor %
    blk(mdl, k('MOTOR_PCT_PER_CMS'), 'simulink/Math Operations/Gain', [880 y], 'Gain', 'p.motor_slope');
    blk(mdl, k('MOTOR_MIN_PCT'), 'simulink/Math Operations/Bias', [980 y], 'Bias', 'p.motor_min');
    blk(mdl, k('motor % <= 100'), 'simulink/Discontinuities/Saturation', [1080 y], ...
        'UpperLimit', '100', 'LowerLimit', '0');

    % the car: dead zone p0, then the identified G(s), then position
    blk(mdl, k('- p0'), 'simulink/Math Operations/Bias', [1180 y], 'Bias', '-p0');
    blk(mdl, k('dead zone'), 'simulink/Discontinuities/Saturation', [1280 y], ...
        'UpperLimit', 'inf', 'LowerLimit', '0');
    blk(mdl, k('G(s)'), 'simulink/Continuous/Transfer Fcn', [1380 y], ...
        'Numerator', 'G_num', 'Denominator', 'G_den');
    last = k('G(s)');
    if G.IODelay > 0
        blk(mdl, k('dead time'), 'simulink/Continuous/Transport Delay', [1480 y], 'DelayTime', 'G_delay');
        add_line(mdl, [last '/1'], [k('dead time') '/1'], 'autorouting', 'on');
        last = k('dead time');
    end
    blk(mdl, k('position'), 'simulink/Continuous/Integrator', [1580 y]);

    % sensor: the exact position (continuous) or whole encoder counts every Ts (discrete)
    if disc
        blk(mdl, k('sample Ts'), 'simulink/Discrete/Zero-Order Hold', [1680 y+90], 'SampleTime', 'Ts');
        blk(mdl, k('encoder counts'), 'simulink/Discontinuities/Quantizer', [1580 y+90], ...
            'QuantizationInterval', '1/p.cnt_cm');
        blk(mdl, k('ref sample'), 'simulink/Discrete/Zero-Order Hold', [160 y], 'SampleTime', 'Ts');
        blk(mdl, k('SP_Vel sample'), 'simulink/Discrete/Zero-Order Hold', [560 y+150], 'SampleTime', 'Ts');
    end

    % logging
    blk(mdl, k('log pos'), 'simulink/Sinks/To Workspace', [1780 y], ...
        'VariableName', ['pos_' lower(s(1))], 'SaveFormat', 'Timeseries');
    blk(mdl, k('log pct'), 'simulink/Sinks/To Workspace', [1180 y-80], ...
        'VariableName', ['pct_' lower(s(1))], 'SaveFormat', 'Timeseries');

    % wires
    w = @(a, b) add_line(mdl, a, b, 'autorouting', 'on');
    if disc
        w('Ref position/1', [k('ref sample') '/1']);
        w([k('ref sample') '/1'], [k('Error') '/1']);
        w('Ref speed SP_Vel/1', [k('SP_Vel sample') '/1']);
        w([k('SP_Vel sample') '/1'], [k('+ feed-forward') '/2']);
        w([k('position') '/1'], [k('sample Ts') '/1']);
        w([k('sample Ts') '/1'], [k('encoder counts') '/1']);
        w([k('encoder counts') '/1'], [k('Error') '/2']);
    else
        w('Ref position/1', [k('Error') '/1']);
        w('Ref speed SP_Vel/1', [k('+ feed-forward') '/2']);
        w([k('position') '/1'], [k('Error') '/2']);
    end
    w([k('Error') '/1'], [k('Kp') '/1']);
    w([k('Error') '/1'], [k('Integral e') '/1']);
    w([k('Error') '/1'], [k('derivative') '/1']);
    w([k('Integral e') '/1'], [k('Ki') '/1']);
    w([k('derivative') '/1'], [k('Kd') '/1']);
    w([k('Kp') '/1'], [k('PID') '/1']);
    w([k('Ki') '/1'], [k('PID') '/2']);
    w([k('Kd') '/1'], [k('PID') '/3']);
    w([k('PID') '/1'], [k('+ feed-forward') '/1']);
    w([k('+ feed-forward') '/1'], [k('u min 1 cm per s') '/1']);
    w([k('u min 1 cm per s') '/1'], [k('MOTOR_PCT_PER_CMS') '/1']);
    w([k('MOTOR_PCT_PER_CMS') '/1'], [k('MOTOR_MIN_PCT') '/1']);
    w([k('MOTOR_MIN_PCT') '/1'], [k('motor % <= 100') '/1']);
    w([k('motor % <= 100') '/1'], [k('- p0') '/1']);
    w([k('motor % <= 100') '/1'], [k('log pct') '/1']);
    w([k('- p0') '/1'], [k('dead zone') '/1']);
    w([k('dead zone') '/1'], [k('G(s)') '/1']);
    w([last '/1'], [k('position') '/1']);
    w([k('position') '/1'], [k('log pos') '/1']);
end

blk(mdl, 'Position scope', 'simulink/Sinks/Scope', [1880 300], 'NumInputPorts', '3');
add_line(mdl, 'Ref position/1', 'Position scope/1', 'autorouting', 'on');
add_line(mdl, 'C position/1', 'Position scope/2', 'autorouting', 'on');
add_line(mdl, 'D position/1', 'Position scope/3', 'autorouting', 'on');
blk(mdl, 'log ref', 'simulink/Sinks/To Workspace', [160 220], 'VariableName', 'pos_ref', 'SaveFormat', 'Timeseries');
add_line(mdl, 'Ref position/1', 'log ref/1', 'autorouting', 'on');

save_system(mdl);
open_system(mdl);
fprintf('Built %s.slx\n', mdl);

%% Run it once and plot
out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
ref = out.get('pos_ref'); pc = out.get('pos_c'); pd = out.get('pos_d');
mc = out.get('pct_c');    md = out.get('pct_d');

figure('Name', 'Simulink: continuous vs discrete');
subplot(2, 1, 1); hold on; grid on;
plot(ref.Time, ref.Data, 'k--', pc.Time, pc.Data, pd.Time, pd.Data);
ylabel('position (cm)'); legend('reference', 'continuous', sprintf('discrete Ts = %.2f s', Ts), 'Location', 'southeast');
title(sprintf('Simulink: %.0f cm in %.1f s', p.D_cm, p.T));
subplot(2, 1, 2); hold on; grid on;
plot(mc.Time, mc.Data, md.Time, md.Data);
ylabel('motor %'); xlabel('t (s)'); legend('continuous', 'discrete', 'Location', 'best');
fprintf('Final position: continuous %.1f cm, discrete %.1f cm (target %.0f cm)\n', ...
        pc.Data(end), pd.Data(end), p.D_cm);

%% helper: add a block at [x y] (50x30) and set its parameters
function blk(mdl, name, lib, xy, varargin)
    h = add_block(lib, [mdl '/' name], 'Position', [xy(1) xy(2) xy(1)+50 xy(2)+30]);
    if ~isempty(varargin), set_param(h, varargin{:}); end
end
