% Parameter k Root Locus Analysis
% Characteristic equation: s^2 + k*s + 10 = 0

s = tf('s');

% 1. Convert characteristic equation to standard root locus form
% s^2 + 10 + k*s = 0
% 1 + k * (s / (s^2 + 10)) = 0
% Therefore, the equivalent open-loop transfer function L(s) is:
L = s / (s^2 + 10);

% 2. Plot the Root Locus
figure('Position', [100, 100, 800, 600]);
rlocus(L);

% Set formatting for better visualization
title('Root Locus with respect to parameter k', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Real Axis (seconds^{-1})', 'FontSize', 12);
ylabel('Imaginary Axis (seconds^{-1})', 'FontSize', 12);
grid on;

% Add annotations for the poles and zeros
text(0.2, 3.2, 'Pole: +j\surd10', 'FontSize', 11, 'Color', 'b');
text(0.2, -3.2, 'Pole: -j\surd10', 'FontSize', 11, 'Color', 'b');
text(0.2, 0.2, 'Zero: 0', 'FontSize', 11, 'Color', 'r');

% 3. Save the plot
exportgraphics(gcf, 'root_locus_k.png', 'Resolution', 300);
exit;
