% lqr_5sec_design.m
% Design LQR controller to achieve Settling Time <= 5 seconds
% Plant: G(s) = 10 / (s^3 + 5s^2 + 4s)

%% 1. System Definition
num = 10;
den = [1 5 4 0];
sys_tf = tf(num, den);
sys_ss = ss(sys_tf);
A = sys_ss.A;
B = sys_ss.B;
C = sys_ss.C;
D = sys_ss.D;

% Pre-scaler for zero steady-state error
calc_Nbar = @(A,B,C,K) inv(-C*inv(A-B*K)*B);

%% 2. LQR Auto-Tuning Design for <= 5-second Settling Time
% To achieve t_s <= 5s, we iteratively increase the penalty on the actual output y.
% Since y = C*x, the penalty on y^2 is x'*C'*q_y*C*x.

q_y = 1; % Initial weight for the output y
R = 1;   % Fixed control effort penalty
ts = 100; % Initialize settling time

fprintf('Starting iterative LQR design to achieve Settling Time <= 5s...\n');

while ts > 4.8
    % Construct Q matrix to heavily penalize output error
    Q = q_y * (C' * C) + 0.001 * eye(3); 
    K = lqr(A, B, Q, R);
    Nbar = calc_Nbar(A,B,C,K);
    sys_cl = ss(A - B*K, B*Nbar, C, D);
    
    info = stepinfo(sys_cl);
    ts = info.SettlingTime;
    
    if ts <= 4.8
        break;
    end
    q_y = q_y * 1.5; % Increase penalty by 50% iteratively
end

os = info.Overshoot;

fprintf('=== LQR Design Results ===\n');
fprintf('Final Output Penalty q_y: %f\n', q_y);
fprintf('R matrix: %f\n', R);
fprintf('Calculated Gain K: [%f, %f, %f]\n', K);
fprintf('Settling Time (2%%): %.3f seconds\n', ts);
fprintf('Overshoot: %.2f %%\n', os);
fprintf('==========================\n');

if ts <= 5
    fprintf('SUCCESS: Settling time is within 5 seconds!\n');
else
    fprintf('FAILED: Settling time exceeds 5 seconds.\n');
end

% Plotting the Step Response
figure('Position', [200, 200, 600, 400]);
step(sys_cl, 10);
title(sprintf('LQR Step Response (Settling Time = %.2f s)', ts), 'FontSize', 14);
grid on;
ylabel('Amplitude');
xlabel('Time (seconds)');

% Save plot
exportgraphics(gcf, 'lqr_5sec_response.png', 'Resolution', 300);
exit;
