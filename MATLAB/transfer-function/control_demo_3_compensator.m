%% 자동제어 강의용 예제 3: Lead-Lag 보상기 설계를 통한 시스템 성능 종합 개선
% 작성일: 2026-05-21
% 설명: 미보상 시스템(단순 P 제어), 리드(진상) 보상 시스템, 리드-래그(진지상) 보상 시스템의
%       주파수 응답 및 과도 응답을 설계하고 비교합니다.
%       이 코드는 고해상도 비교 이미지(compensator_design_comparison.png)를 자동 저장합니다.

clear; clc; close all;

%% 1. 플랜트 및 보상기 정의
% Gp(s) = 10 / (s*(s+1)*(s+4))
num = 10;
den = [1, 5, 4, 0];
Gp = tf(num, den);

% (1) 미보상 시스템 (Uncompensated: 단순 P 제어, Kp = 1.0 적용)
Gc_u = 1.0;
Lu = Gc_u * Gp;
Tu = feedback(Lu, 1);

% (2) 리드(진상) 보상기 설계 (Lead Compensator)
% 위상 마진을 대폭 확보하여 과도 응답(오버슈트 감소, 속도 향상) 개선
% G_lead(s) = Kc * (s + z_lead) / (s + p_lead) = 3 * (s + 1.2) / (s + 4.8)
Kc = 3.0;
z_lead = 1.2;
p_lead = 4.8;
Glead = Kc * tf([1, z_lead], [1, p_lead]);

L_lead = Glead * Gp;
T_lead = feedback(L_lead, 1);

% (3) 래그(지상) 보상기 추가 설계 (Lag Compensator)
% 고주파 이득을 깎아 내어 보드 선도의 교차주파수 영역을 건드리지 않으면서, 
% 저주파 루프 이득을 5배 증가시켜(beta = 0.2) 정상상태 램프 오차를 획기적으로 개선.
% G_lag(s) = (s + z_lag) / (s + p_lag) = (s + 0.1) / (s + 0.02)
z_lag = 0.1;
p_lag = 0.02;
Glag = tf([1, z_lag], [1, p_lag]);

% 리드-래그 통합 보상 시스템
G_ll = Glead * Glag;
L_ll = G_ll * Gp;
T_ll = feedback(L_ll, 1);

fprintf('====================================================\n');
fprintf('        [자동제어 예제 3] Lead-Lag 보상기 설계\n');
fprintf('====================================================\n');
disp('1. 설계된 리드 보상기 G_lead(s):');
disp(Glead);
disp('2. 설계된 래그 보상기 G_lag(s):');
disp(Glag);
disp('3. 리드-래그 통합 제어기 G_c(s):');
disp(G_ll);

%% 2. 과도 응답 및 주파수 영역 성능 정량 비교
t = 0:0.01:10;
y_step_u    = step(Tu, t);
y_step_lead = step(T_lead, t);
y_step_ll   = step(T_ll, t);

% 시간 영역 메트릭 수집
info_u    = stepinfo(Tu);
info_lead = stepinfo(T_lead);
info_ll   = stepinfo(T_ll);

% 주파수 마진 수집
[GM_u, PM_u, w_pc_u, w_gc_u] = margin(Lu);
[GM_lead, PM_lead, w_pc_lead, w_gc_lead] = margin(L_lead);
[GM_ll, PM_ll, w_pc_ll, w_gc_ll] = margin(L_ll);

% 경사(Ramp) 응답을 통한 정상상태 오차 계산 (Ramp Steady-State Error)
t_ramp = 0:0.01:30;
r_ramp = t_ramp';
y_ramp_u    = lsim(Tu, r_ramp, t_ramp);
y_ramp_lead = lsim(T_lead, r_ramp, t_ramp);
y_ramp_ll   = lsim(T_ll, r_ramp, t_ramp);

ess_ramp_u    = r_ramp(end) - y_ramp_u(end);
ess_ramp_lead = r_ramp(end) - y_ramp_lead(end);
ess_ramp_ll   = r_ramp(end) - y_ramp_ll(end);

fprintf('\n--------------------------------------------------------------------------------\n');
fprintf('                    미보상 시스템 vs 보상 시스템 성능 종합 비교\n');
fprintf('--------------------------------------------------------------------------------\n');
fprintf('성능 지표               | 미보상 (P제어, Kp=1) | Lead 보상 적용      | Lead-Lag 보상 적용\n');
fprintf('--------------------------------------------------------------------------------\n');
fprintf('이득 교차 주파수 (w_gc) |     %5.2f rad/s      |     %5.2f rad/s      |     %5.2f rad/s\n', w_gc_u, w_gc_lead, w_gc_ll);
fprintf('위상 여유 (Phase Margin)|     %5.2f deg        |     %5.2f deg        |     %5.2f deg\n', PM_u, PM_lead, PM_ll);
fprintf('이득 여유 (Gain Margin) |     %5.2f dB         |     %5.2f dB         |     %5.2f dB\n', 20*log10(GM_u), 20*log10(GM_lead), 20*log10(GM_ll));
fprintf('--------------------------------------------------------------------------------\n');
fprintf('상승 시간 (Rise Time)   |     %5.2f s          |     %5.2f s          |     %5.2f s\n', info_u.RiseTime, info_lead.RiseTime, info_ll.RiseTime);
fprintf('최대 오버슈트 (Overshoot)|     %5.2f %%         |     %5.2f %%         |     %5.2f %%\n', info_u.Overshoot, info_lead.Overshoot, info_ll.Overshoot);
fprintf('정착 시간 (Settling Time)|     %5.2f s          |     %5.2f s          |     %5.2f s\n', info_u.SettlingTime, info_lead.SettlingTime, info_ll.SettlingTime);
fprintf('정상상태 램프오차(ess)  |     %5.2f            |     %5.2f            |     %5.2f\n', ess_ramp_u, ess_ramp_lead, ess_ramp_ll);
fprintf('================================================================================\n');

%% 3. 고해상도 시각화 및 이미지 저장
colors = {
    [0.0, 0.4470, 0.7410], ... % Sleek Blue (Uncompensated)
    [0.8500, 0.3250, 0.0980], ... % Warm Orange (Lead)
    [0.4660, 0.6740, 0.1880]      % Soft Green (Lead-Lag)
};

fig = figure('Color', 'w', 'Position', [100, 100, 1200, 550]);

% --- Left Subplot: Closed-Loop Step Response Comparison ---
subplot(1, 2, 1);
hold on;
grid on;
box on;

% 목표값 기준선
plot(t, ones(size(t)), '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off');

h1 = plot(t, y_step_u, 'Color', colors{1}, 'LineWidth', 2.0);
h2 = plot(t, y_step_lead, 'Color', colors{2}, 'LineWidth', 2.0);
h3 = plot(t, y_step_ll, 'Color', colors{3}, 'LineWidth', 2.0);

% 그래프 데코레이션
set(gca, 'FontSize', 12, 'LineWidth', 1.2, 'GridAlpha', 0.15);
xlabel('시간 (초)', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('시스템 출력 y(t)', 'FontSize', 13, 'FontWeight', 'bold');
title('\bf(a) Closed-Loop Step Response Comparison', 'FontSize', 14);
legend([h1, h2, h3], ...
       {sprintf('미보상 P제어 (K_p = 1.0)'), ...
        sprintf('Lead 보상기 적용 (위상여유 대폭 개선)'), ...
        sprintf('Lead-Lag 보상기 적용 (위상여유 + 정상상태 개선)')}, ...
       'Location', 'southeast', 'FontSize', 10.5);
ylim([-0.2, 1.6]);

% --- Right Subplot: Open-Loop Bode Diagram Comparison ---
subplot(1, 2, 2);
w_freq = logspace(-2, 2, 1000);

% 크기 선도 비교
[mag_u, phase_u, w_u] = bode(Lu, w_freq);
[mag_lead, phase_lead, w_lead] = bode(L_lead, w_freq);
[mag_ll, phase_ll, w_ll] = bode(L_ll, w_freq);

% 2단 분할을 통해 크기 및 위상 동시 플로팅
subplot(2, 2, 2);
hold on; grid on; box on;
b1 = plot(w_u, 20*log10(squeeze(mag_u)), 'Color', colors{1}, 'LineWidth', 1.8);
b2 = plot(w_lead, 20*log10(squeeze(mag_lead)), 'Color', colors{2}, 'LineWidth', 1.8);
b3 = plot(w_ll, 20*log10(squeeze(mag_ll)), 'Color', colors{3}, 'LineWidth', 2.0);
plot([w_freq(1) w_freq(end)], [0 0], 'k:', 'LineWidth', 1.0); % 0dB 기준선

set(gca, 'XScale', 'log', 'FontSize', 11, 'GridAlpha', 0.15);
ylabel('크기 (Magnitude, dB)', 'FontSize', 11, 'FontWeight', 'bold');
title('\bf(b) Open-Loop Bode Diagram Comparison', 'FontSize', 13);
xlim([0.05, 50]);
ylim([-50, 50]);
legend([b1, b2, b3], {'미보상 L_u', 'Lead 보상 L_{lead}', 'Lead-Lag 보상 L_{ll}'}, ...
       'Location', 'southwest', 'FontSize', 9);

% 위상 선도 비교
subplot(2, 2, 4);
hold on; grid on; box on;
plot(w_u, squeeze(phase_u), 'Color', colors{1}, 'LineWidth', 1.8);
plot(w_lead, squeeze(phase_lead), 'Color', colors{2}, 'LineWidth', 1.8);
plot(w_ll, squeeze(phase_ll), 'Color', colors{3}, 'LineWidth', 2.0);
plot([w_freq(1) w_freq(end)], [-180 -180], 'k:', 'LineWidth', 1.0); % -180도 기준선

set(gca, 'XScale', 'log', 'FontSize', 11, 'GridAlpha', 0.15);
xlabel('주파수 (Frequency, rad/s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('위상 (Phase, deg)', 'FontSize', 11, 'FontWeight', 'bold');
xlim([0.05, 50]);
ylim([-270, -90]);

saveas(fig, 'compensator_design_comparison.png');
fprintf('\n시각화 완료: "compensator_design_comparison.png" 파일이 저장되었습니다.\n\n');
