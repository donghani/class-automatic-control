% control_demo_5_statespace.m
% State-Space Control Design: Pole Placement, LQR, Observer, LQG
% Plant: G(s) = 10 / (s^3 + 5s^2 + 4s)

%% 0. System Definition (State-Space Representation)
num = 10;
den = [1 5 4 0];
sys_tf = tf(num, den);
sys_ss = ss(sys_tf); % Convert to State-Space: dx/dt = Ax + Bu, y = Cx + Du
A = sys_ss.A;
B = sys_ss.B;
C = sys_ss.C;
D = sys_ss.D;

% Pre-scaler for reference tracking (Nbar) to get zero steady-state error
calc_Nbar = @(A,B,C,K) inv(-C*inv(A-B*K)*B);

t = 0:0.01:10; % Simulation time to show 5-second settling
r = ones(size(t)); % Step reference input

%% 1. Pole Placement (Ackermann's Formula)
% Desired closed-loop poles: tuned for approx 4~5 sec settling time
p_des = [-1+1j, -1-1j, -5];
K_pp = place(A, B, p_des);
N_pp = calc_Nbar(A,B,C,K_pp);
sys_cl_pp = ss(A - B*K_pp, B*N_pp, C, D);
y_pp = lsim(sys_cl_pp, r, t);

%% 2. LQR (Linear Quadratic Regulator)
% Q: penalty on states (large Q = tight control), R: penalty on input (large R = save energy)
% Penalize the actual output y to achieve <= 5s settling time smoothly
Q = 1 * (C' * C) + 0.001 * eye(3); 
R = 1; 
K_lqr = lqr(A, B, Q, R);
N_lqr = calc_Nbar(A,B,C,K_lqr);
sys_cl_lqr = ss(A - B*K_lqr, B*N_lqr, C, D);
y_lqr = lsim(sys_cl_lqr, r, t);

%% 3. State Observer + State Feedback (Luenberger)
% We cannot measure all states, so we estimate them using C.
% Observer poles must be faster than controller poles (e.g., 5x faster)
p_obs = [-5, -6, -7];
L_obs = place(A', C', p_obs)'; % L matrix

% Combine LQR controller with Observer
% Closed loop with observer: dx_e/dt = (A - L*C)*x_e (estimation error dynamics)
A_obs_cl = [A - B*K_lqr, B*K_lqr; zeros(size(A)), A - L_obs*C];
B_obs_cl = [B*N_lqr; zeros(size(B))];
C_obs_cl = [C, zeros(size(C))];
sys_cl_obs = ss(A_obs_cl, B_obs_cl, C_obs_cl, 0);
y_obs = lsim(sys_cl_obs, r, t);

%% 4. LQG (Linear Quadratic Gaussian) -> Kalman Filter + LQR
% Process noise covariance Qn, Measurement noise covariance Rn
Qn = 1 * eye(3);  % Process noise
Rn = 0.01;        % Measurement noise
[L_kf, P, E] = lqe(A, eye(3), C, Qn, Rn); % Kalman filter gain

% Combine LQR with Kalman Filter
A_lqg_cl = [A - B*K_lqr, B*K_lqr; zeros(size(A)), A - L_kf*C];
B_lqg_cl = [B*N_lqr; zeros(size(B))];
C_lqg_cl = [C, zeros(size(C))];
sys_cl_lqg = ss(A_lqg_cl, B_lqg_cl, C_lqg_cl, 0);

% Simulate LQG with actual noise injected
v = sqrt(Rn)*randn(size(t)); % Sensor noise
w = sqrt(Qn(1,1))*randn(length(t), 3); % Process noise
% To simulate properly with noise, we use lsim with multi-inputs if modeled exactly,
% but for simplicity of visual comparison, we will plot the ideal response of LQG
% (showing it remains stable) and then add noise visually or via custom loop.
% Here we do a deterministic step response for fair comparison of dynamics.
y_lqg = lsim(sys_cl_lqg, r, t);


%% Performance Metrics Calculation
inf_pp = stepinfo(sys_cl_pp);
inf_lqr = stepinfo(sys_cl_lqr);
inf_obs = stepinfo(sys_cl_obs);
inf_lqg = stepinfo(sys_cl_lqg);

fprintf('--- Performance Metrics ---\n');
fprintf('Pole Placement: OS = %.2f%%, Tr = %.2fs, Ts = %.2fs\n', inf_pp.Overshoot, inf_pp.RiseTime, inf_pp.SettlingTime);
fprintf('LQR           : OS = %.2f%%, Tr = %.2fs, Ts = %.2fs\n', inf_lqr.Overshoot, inf_lqr.RiseTime, inf_lqr.SettlingTime);
fprintf('Observer      : OS = %.2f%%, Tr = %.2fs, Ts = %.2fs\n', inf_obs.Overshoot, inf_obs.RiseTime, inf_obs.SettlingTime);
fprintf('LQG           : OS = %.2f%%, Tr = %.2fs, Ts = %.2fs\n', inf_lqg.Overshoot, inf_lqg.RiseTime, inf_lqg.SettlingTime);

%% Visualization (4 Subplots)
figure('Position', [100, 100, 1200, 800]);

% Helper function to create annotation string
ann_str = @(inf) sprintf('OS: %.1f%%\nTr: %.2fs\nTs: %.2fs', inf.Overshoot, inf.RiseTime, inf.SettlingTime);

subplot(2, 2, 1);
plot(t, y_pp, 'LineWidth', 2); hold on; plot(t, r, 'r--');
title('1. Pole Placement (Ideal, Fixed Poles)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Output y(t)'); grid on; ylim([0 1.5]);
text(6, 0.4, ann_str(inf_pp), 'FontSize', 10, 'BackgroundColor', 'w', 'EdgeColor', 'k');

subplot(2, 2, 2);
plot(t, y_lqr, 'LineWidth', 2); hold on; plot(t, r, 'r--');
title('2. LQR (Optimal Trade-off)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Output y(t)'); grid on; ylim([0 1.5]);
text(6, 0.4, ann_str(inf_lqr), 'FontSize', 10, 'BackgroundColor', 'w', 'EdgeColor', 'k');

subplot(2, 2, 3);
plot(t, y_obs, 'LineWidth', 2); hold on; plot(t, r, 'r--');
title('3. Observer-based Control (Estimated States)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)'); ylabel('Output y(t)'); grid on; ylim([0 1.5]);
text(6, 0.4, ann_str(inf_obs), 'FontSize', 10, 'BackgroundColor', 'w', 'EdgeColor', 'k');

subplot(2, 2, 4);
plot(t, y_lqg, 'LineWidth', 2); hold on; plot(t, r, 'r--');
title('4. LQG (Kalman Filter + LQR)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)'); ylabel('Output y(t)'); grid on; ylim([0 1.5]);
text(6, 0.4, ann_str(inf_lqg), 'FontSize', 10, 'BackgroundColor', 'w', 'EdgeColor', 'k');

exportgraphics(gcf, 'state_space_comparison.png', 'Resolution', 300);
exit;
