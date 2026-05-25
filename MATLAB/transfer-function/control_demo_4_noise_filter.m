% control_demo_4_noise_filter.m
% High-frequency noise amplification in ideal derivative vs Lead Compensator

% 1. Frequency Domain Comparison (Bode Plot)
s = tf('s');
K_p = 1;
K_d = 0.5;

% Ideal PD Controller (Pure Derivative)
C_PD = K_p + K_d * s;

% Lead Compensator (Practical PD with low-pass filter)
% Filter time constant tau = 0.05 (Pole at s = -20)
tau = 0.05;
C_Lead = K_p + (K_d * s) / (tau * s + 1);

figure('Position', [100, 100, 1000, 400]);

% Subplot 1: Bode Plot
subplot(1, 2, 1);
opt = bodeoptions;
opt.MagUnits = 'abs'; % Use absolute magnitude instead of dB for clearer intuition
opt.MagScale = 'log';
bodeplot(C_PD, C_Lead, opt);
title('Bode Plot: Ideal PD vs Lead Compensator', 'FontSize', 12, 'FontWeight', 'bold');
legend('Ideal PD (No Pole)', 'Lead Compensator (Added Pole)', 'Location', 'northwest');
grid on;

% Subplot 2: Time Domain Simulation with Noise
t = 0:0.001:10;
% Base signal (low frequency 1 rad/s) + Noise (high frequency 100 rad/s)
base_signal = sin(t);
noise = 0.05 * sin(100 * t); % 5% amplitude noise
e_t = base_signal + noise;

% Time derivatives
% Pure derivative: d/dt(sin(t) + 0.05*sin(100t)) = cos(t) + 5*cos(100t)
d_ideal = cos(t) + 5 * cos(100 * t);
u_PD = K_p * e_t + K_d * d_ideal;

% Filtered derivative (Lead) simulation using lsim
u_Lead = lsim(C_Lead, e_t, t);

subplot(1, 2, 2);
plot(t, e_t, 'k', 'DisplayName', 'Input Signal (Sensor Data with Noise)');
hold on;
plot(t, u_PD, 'r', 'DisplayName', 'Control Output (Ideal PD)');
plot(t, u_Lead, 'b', 'LineWidth', 1.5, 'DisplayName', 'Control Output (Lead/Practical PD)');
xlim([0, 2]); % Zoom in to see the noise
title('Time Domain: Noise Amplification', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)');
ylabel('Control Signal Amplitude');
legend('Location', 'southwest');
grid on;

% Save plot
exportgraphics(gcf, 'derivative_noise_comparison.png', 'Resolution', 300);
exit;
