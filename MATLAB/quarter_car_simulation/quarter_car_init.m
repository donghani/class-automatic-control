%% quarter_car_init.m
% Quarter Car Active Suspension Simulation
% - 2-DOF model: Car body (Mc) + Wheel (Mw)
% - 2 x mass-spring-damper: Suspension (ks, cs) + Tire (kt, ct)
% - Active actuator force for vertical position control of Mc
% - Controllers: PID (pidtune) and State Feedback with integral (place)
% - Target: Overshoot < 5%, Rise Time < 0.5 s
%
% Usage:
%   >> quarter_car_init    % design controllers & plot comparison
%   >> build_qc_model      % then open Simulink model

clear; close all; clc;

fprintf('=================================================\n');
fprintf('  Quarter Car Active Suspension Simulation\n');
fprintf('=================================================\n\n');

%% ----------------------------------------------------------------
%  1. PHYSICAL PARAMETERS  (editable defaults)
% ----------------------------------------------------------------
Mc  = 300;      % [kg]      Car body (sprung) mass
Mw  = 50;       % [kg]      Wheel (unsprung) mass
ks  = 15000;    % [N/m]     Suspension spring
cs  = 1500;     % [N·s/m]  Suspension damper
kt  = 200000;   % [N/m]     Tire spring
ct  = 0;        % [N·s/m]  Tire damper

step_amp = 0.05;  % [m]  Step reference amplitude for yc

fprintf('System Parameters:\n');
fprintf('  Mc = %g kg,  Mw = %g kg\n', Mc, Mw);
fprintf('  ks = %g N/m, cs = %g N·s/m\n', ks, cs);
fprintf('  kt = %g N/m, ct = %g N·s/m\n', kt, ct);
fprintf('  Step amplitude = %g m\n\n', step_amp);

%% ----------------------------------------------------------------
%  2. STATE-SPACE MODEL
%
%  States:  x = [yc; dyc; yw; dyw]
%  Input:   u = F_act  (actuator force, positive = push Mc up)
%  Output:  y = yc
%
%  Mc*ddyc = -ks*(yc-yw) - cs*(dyc-dyw) + F_act
%  Mw*ddyw =  ks*(yc-yw) + cs*(dyc-dyw) - kt*yw  - ct*dyw - F_act
% ----------------------------------------------------------------
A = [0,       1,        0,              0;
    -ks/Mc,  -cs/Mc,   ks/Mc,          cs/Mc;
     0,       0,        0,              1;
     ks/Mw,   cs/Mw,  -(ks+kt)/Mw,  -(cs+ct)/Mw];

B_act  = [0;  1/Mc;  0; -1/Mw];   % Actuator input column
C_out  = [1,  0,  0,  0];         % Output: yc
D_out  = 0;

sys = ss(A, B_act, C_out, D_out);

fprintf('Checking controllability ... ');
if rank(ctrb(A, B_act)) == 4
    fprintf('OK\n\n');
else
    error('System is NOT controllable — check parameters.');
end

%% ----------------------------------------------------------------
%  3. PID CONTROLLER  (pidtune + bandwidth sweep to hit specs)
%
%  Target: Overshoot < 5%, Rise Time < 0.5 s
% ----------------------------------------------------------------
fprintf('--- PID Controller Design ---\n');

best_pid  = [];
best_score = inf;

for bw = [8, 10, 12, 15, 18, 20, 25, 30]
    try
        C_try = pidtune(sys, 'PID', bw);
        T_try = feedback(series(C_try, sys), 1);
        si    = stepinfo(T_try, 'SettlingTimeThreshold', 0.02);
        if ~isnan(si.Overshoot) && ~isnan(si.RiseTime)
            meets_os = (si.Overshoot < 5);
            meets_tr = (si.RiseTime  < 0.5);
            score = si.Overshoot + 10*max(0, si.RiseTime - 0.5);
            if meets_os && meets_tr && score < best_score
                best_score = score;
                best_pid   = C_try;
            end
        end
    catch
    end
end

% Fallback: relax overshoot slightly and pick lowest overshoot
if isempty(best_pid)
    fprintf('  [Warning] Could not fully meet specs — using best available.\n');
    best_score = inf;
    for bw = 5:2:40
        try
            C_try = pidtune(sys, 'PID', bw);
            T_try = feedback(series(C_try, sys), 1);
            si    = stepinfo(T_try);
            score = si.Overshoot;
            if score < best_score
                best_score = score;
                best_pid   = C_try;
            end
        catch
        end
    end
end

C_pid = best_pid;
Kp = C_pid.Kp;
Ki = C_pid.Ki;
Kd = C_pid.Kd;
Tf = C_pid.Tf;   % derivative filter time constant (1/N)

T_pid = feedback(series(C_pid, sys), 1);
si_pid = stepinfo(T_pid, 'SettlingTimeThreshold', 0.02);

fprintf('  Kp = %.4f\n',  Kp);
fprintf('  Ki = %.4f\n',  Ki);
fprintf('  Kd = %.4f\n',  Kd);
fprintf('  Tf = %.4f  (derivative filter)\n', Tf);
fprintf('  Overshoot  = %.2f %%\n',  si_pid.Overshoot);
fprintf('  Rise Time  = %.3f s\n',   si_pid.RiseTime);
fprintf('  Settle Time= %.3f s\n\n', si_pid.SettlingTime);

%% ----------------------------------------------------------------
%  4. STATE FEEDBACK + INTEGRAL CONTROLLER  (pole placement)
%
%  Augmented state: z = [x; e_int]  where e_int = integral(r - yc)
%  Control law:     F_act = -[K_sf | Ki_sf] * z
%  Plant input:     u = -K_sf*x - Ki_sf*e_int
%
%  Target poles chosen for OS < 5%, Tr < 0.5 s
% ----------------------------------------------------------------
fprintf('--- State Feedback + Integral Controller ---\n');

% Augmented system: 5th order
A_aug = [A,     zeros(4,1);
        -C_out,  0        ];
B_aug = [B_act; 0];

fprintf('  Checking augmented controllability ... ');
if rank(ctrb(A_aug, B_aug)) == 5
    fprintf('OK\n');
else
    error('Augmented system NOT controllable.');
end

% Dominant poles: zeta = 0.8, wn = 15 rad/s → OS ≈ 1.5%, Tr ≈ 0.25 s
zeta = 0.8;
wn   = 15;
p12  = [-zeta*wn + 1j*wn*sqrt(1-zeta^2), ...
        -zeta*wn - 1j*wn*sqrt(1-zeta^2)];

% Non-dominant poles: 5-8x further left
p3   = -4*wn;
p4   = -5*wn;
p5   = -3*wn;   % integral pole

K_full = place(A_aug, B_aug, [p12, p3, p4, p5]);
K_sf   = K_full(1:4);
Ki_sf  = K_full(5);

fprintf('  Desired poles: %.2f±%.2fj, %.1f, %.1f, %.1f\n', ...
    real(p12(1)), abs(imag(p12(1))), p3, p4, p5);
fprintf('  K_sf  = [%.4f  %.4f  %.4f  %.4f]\n', K_sf);
fprintf('  Ki_sf = %.4f\n', Ki_sf);

% Verify: closed-loop augmented system
% dz/dt = (A_aug - B_aug*K_full)*z + [0;0;0;0;1]*r
% y = C_aug * z
A_cl  = A_aug - B_aug * K_full;
B_cl  = [zeros(4,1); 1];
C_aug = [C_out, 0];
sys_sf_cl = ss(A_cl, B_cl, C_aug, 0);
si_sf = stepinfo(sys_sf_cl, 'SettlingTimeThreshold', 0.02);

fprintf('  Overshoot  = %.2f %%\n',  si_sf.Overshoot);
fprintf('  Rise Time  = %.3f s\n',   si_sf.RiseTime);
fprintf('  Settle Time= %.3f s\n\n', si_sf.SettlingTime);

%% ----------------------------------------------------------------
%  5. COMPARISON PLOT  (step response, t = 0..5 s)
% ----------------------------------------------------------------
t_end = 5;
t     = 0 : 0.001 : t_end;

% PID response
[y_pid, t_pid] = step(step_amp * T_pid, t);

% State Feedback response
[y_sf, t_sf]   = step(step_amp * sys_sf_cl, t);

figure('Name', 'Quarter Car Active Suspension — Step Response', ...
       'NumberTitle', 'off', 'Position', [100 80 1000 480]);

% ---- Full range ----
ax1 = subplot(1,2,1);
h1 = plot(t_pid, y_pid,   'b-',  'LineWidth', 2.0); hold on;
h2 = plot(t_sf,  y_sf,    'r--', 'LineWidth', 2.0);
yline(step_amp, 'k:', 'LineWidth', 1.5);
yline(step_amp*1.05, 'Color', [0.6 0 0], 'LineStyle', ':', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Car body displacement  y_c  (m)');
title('Step Response — 0 to 5 s');
legend('PID Controller', 'State Feedback + Integral', 'Reference', ...
       '5% overshoot limit', 'Location', 'best');
grid on;  xlim([0 t_end]);

% ---- First 2 s (zoom) ----
ax2 = subplot(1,2,2);
plot(t_pid, y_pid,   'b-',  'LineWidth', 2.0); hold on;
plot(t_sf,  y_sf,    'r--', 'LineWidth', 2.0);
yline(step_amp, 'k:', 'LineWidth', 1.5);
yline(step_amp*1.05, 'Color', [0.6 0 0], 'LineStyle', ':', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Car body displacement  y_c  (m)');
title('Step Response — 0 to 2 s (zoom)');
legend('PID Controller', 'State Feedback + Integral', 'Reference', ...
       '5% overshoot limit', 'Location', 'best');
grid on;  xlim([0 2]);

linkaxes([ax1 ax2], 'y');

% Print summary table
fprintf('=== Step Response Summary (reference = %.3f m) ===\n', step_amp);
fprintf('%-26s %12s %15s\n', 'Metric', 'PID', 'State Feedback');
fprintf('%s\n', repmat('-',1,53));
fprintf('%-26s %11.2f%% %14.2f%%\n', 'Overshoot', si_pid.Overshoot, si_sf.Overshoot);
fprintf('%-26s %12.3f s %13.3f s\n', 'Rise Time',     si_pid.RiseTime,    si_sf.RiseTime);
fprintf('%-26s %12.3f s %13.3f s\n', 'Settling Time', si_pid.SettlingTime, si_sf.SettlingTime);
fprintf('%s\n\n', repmat('-',1,53));

%% ----------------------------------------------------------------
%  6. SAVE WORKSPACE VARIABLES  (used by Simulink model)
% ----------------------------------------------------------------
assignin('base','Mc',       Mc);
assignin('base','Mw',       Mw);
assignin('base','ks',       ks);
assignin('base','cs',       cs);
assignin('base','kt',       kt);
assignin('base','ct',       ct);
assignin('base','A',        A);
assignin('base','B_act',    B_act);
assignin('base','C_out',    C_out);
assignin('base','Kp',       Kp);
assignin('base','Ki',       Ki);
assignin('base','Kd',       Kd);
assignin('base','Tf',       Tf);
assignin('base','K_sf',     K_sf);
assignin('base','Ki_sf',    Ki_sf);
assignin('base','step_amp', step_amp);
assignin('base','A_cl',     A_cl);
assignin('base','B_cl',     B_cl);
assignin('base','C_aug',    C_aug);

% Closed-loop SS matrices for Simulink State Space blocks
% PID: use a low-pass filtered version if Tf=0 (ideal derivative → unstable SS)
if Tf < 1e-6
    Tf_use = 1e-4;  % small filter to keep properness
else
    Tf_use = Tf;
end
C_pid_filt = pid(Kp, Ki, Kd, Tf_use);
T_pid_filt = feedback(series(C_pid_filt, sys), 1);
[A_pid_cl, B_pid_cl, C_pid_cl, D_pid_cl] = ssdata(T_pid_filt);
[A_sf_cl,  B_sf_cl,  C_sf_cl,  D_sf_cl]  = ssdata(sys_sf_cl);

assignin('base','A_pid_cl', A_pid_cl);
assignin('base','B_pid_cl', B_pid_cl);
assignin('base','C_pid_cl', C_pid_cl);
assignin('base','D_pid_cl', D_pid_cl);
assignin('base','A_sf_cl',  A_sf_cl);
assignin('base','B_sf_cl',  B_sf_cl);
assignin('base','C_sf_cl',  C_sf_cl);
assignin('base','D_sf_cl',  D_sf_cl);

fprintf('Workspace variables saved.\n');
fprintf('Run  >>  build_qc_model  to open Simulink model.\n\n');
