%% quarter_car_recalc_callback.m
% Called by Simulink PreSimFcn before each simulation run.
% Reads Mc, Mw from the Constant blocks in the model,
% recalculates the state-space model and redesigns both controllers,
% then updates the State Space blocks in the model.

mdl = bdroot;

%% Read Mc, Mw from Constant block values
Mc_str = get_param([mdl '/Mc_val'], 'Value');
Mw_str = get_param([mdl '/Mw_val'], 'Value');
Mc = str2double(Mc_str);
Mw = str2double(Mw_str);

if isnan(Mc) || Mc <= 0, Mc = 300; end
if isnan(Mw) || Mw <= 0, Mw = 50;  end

%% Fixed parameters
ks = 15000;  cs = 1500;  kt = 200000;  ct = 0;
step_amp = 0.05;

%% State-space model
A = [0,       1,        0,             0;
    -ks/Mc,  -cs/Mc,   ks/Mc,         cs/Mc;
     0,       0,        0,             1;
     ks/Mw,   cs/Mw,  -(ks+kt)/Mw,  -(cs+ct)/Mw];
B_act = [0; 1/Mc; 0; -1/Mw];
C_out = [1, 0, 0, 0];
sys   = ss(A, B_act, C_out, 0);

%% PID design
best_pid   = [];
best_score = inf;
for bw = [8, 10, 12, 15, 18, 20, 25]
    try
        C_try = pidtune(sys, 'PID', bw);
        T_try = feedback(series(C_try, sys), 1);
        si    = stepinfo(T_try, 'SettlingTimeThreshold', 0.02);
        if si.Overshoot < 5 && si.RiseTime < 0.5
            score = si.Overshoot + 10*max(0, si.RiseTime-0.5);
            if score < best_score
                best_score = score;  best_pid = C_try;
            end
        end
    catch; end
end
if isempty(best_pid)
    best_pid = pidtune(sys, 'PID', 12);
end
C_pid    = best_pid;
T_pid_ss = minreal(feedback(series(C_pid, sys), 1));

%% State Feedback design
A_aug = [A, zeros(4,1); -C_out, 0];
B_aug = [B_act; 0];
zeta = 0.8;  wn = 15;
p12 = [-zeta*wn + 1j*wn*sqrt(1-zeta^2), ...
       -zeta*wn - 1j*wn*sqrt(1-zeta^2)];
K_full = place(A_aug, B_aug, [p12, -4*wn, -5*wn, -3*wn]);
A_cl   = A_aug - B_aug * K_full;
B_cl   = [zeros(4,1); 1];
C_aug  = [C_out, 0];
sys_sf_cl = ss(A_cl, B_cl, C_aug, 0);

%% Convert to ss data
[A_pid_cl, B_pid_cl, C_pid_cl, D_pid_cl] = ssdata(T_pid_ss);
[A_sf_cl,  B_sf_cl,  C_sf_cl,  D_sf_cl]  = ssdata(sys_sf_cl);

%% Push to base workspace
assignin('base','Mc',Mc); assignin('base','Mw',Mw);
assignin('base','step_amp',step_amp);
assignin('base','A_pid_cl',A_pid_cl); assignin('base','B_pid_cl',B_pid_cl);
assignin('base','C_pid_cl',C_pid_cl); assignin('base','D_pid_cl',D_pid_cl);
assignin('base','A_sf_cl', A_sf_cl);  assignin('base','B_sf_cl', B_sf_cl);
assignin('base','C_sf_cl', C_sf_cl);  assignin('base','D_sf_cl', D_sf_cl);

%% Update State Space block initial conditions (state count may change)
n_pid = size(A_pid_cl,1);
n_sf  = size(A_sf_cl, 1);
set_param([mdl '/PID_CL'], 'X0', sprintf('zeros(%d,1)', n_pid));
set_param([mdl '/SF_CL'],  'X0', sprintf('zeros(%d,1)', n_sf));

fprintf('[PreSimFcn] Recalculated for Mc=%.0f kg, Mw=%.0f kg\n', Mc, Mw);
si_pid = stepinfo(T_pid_ss,   'SettlingTimeThreshold',0.02);
si_sf  = stepinfo(sys_sf_cl,  'SettlingTimeThreshold',0.02);
fprintf('  PID: OS=%.1f%%, Tr=%.3fs\n', si_pid.Overshoot, si_pid.RiseTime);
fprintf('  SF:  OS=%.1f%%, Tr=%.3fs\n', si_sf.Overshoot,  si_sf.RiseTime);
