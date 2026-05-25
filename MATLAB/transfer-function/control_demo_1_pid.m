%% 자동제어 강의용 예제 1: 피드백과 PID 제어, 그리고 시간영역 성능평가
% 작성일: 2026-05-21
% 설명: 3차 선형 시스템에 대해 P, PI, PID 제어기를 적용하고 시간영역 성능을 분석합니다.
%       이 코드는 고해상도 시각화 이미지(pid_comparison.png)를 자동으로 생성합니다.

clear; clc; close all;

%% 1. 시스템(Plant) 및 파라미터 정의
% Gp(s) = 10 / (s*(s+1)*(s+4)) = 10 / (s^3 + 5*s^2 + 4*s)
num = 10;
den = [1, 5, 4, 0];
Gp = tf(num, den);

fprintf('====================================================\n');
fprintf('        [자동제어 예제 1] 피드백 및 PID 제어 시스템\n');
fprintf('====================================================\n');
disp('제어 대상 플랜트 Gp(s):');
disp(Gp);

% 시뮬레이션 시간 설정
t = 0:0.01:15;

%% 2. P 제어기 이득(Kp) 변화에 따른 안정성 분석 (Step Response)
% Kp = 0.5 (안정), Kp = 2.0 (임계 안정 - 지속 진동), Kp = 3.0 (불안정 - 발산)
Kp_vals = [0.5, 2.0, 3.0];
sys_P = cell(1, 3);

for i = 1:length(Kp_vals)
    Gc = Kp_vals(i);
    sys_P{i} = feedback(Gc * Gp, 1);
end

% 각 Kp에 대한 계단 응답 계산
y_P1 = step(sys_P{1}, t);
y_P2 = step(sys_P{2}, t);
y_P3 = step(sys_P{3}, t);

%% 3. P, PI, PID 제어기 성능 비교
% - P 제어기 (Kp = 0.5): 단순 비례 제어 (상당한 과도 오버슈트 및 늦은 응답)
% - PI 제어기 (Kp = 0.5, Ki = 0.2): 정상상태 램프 에러를 줄이기 위한 적분기 추가
% - PID 제어기 (Kp = 1.5, Ki = 0.5, Kd = 1.0): 미분 이득을 추가하여 오버슈트를 줄이고 빠른 정착 유도
Kp = 0.5;
sys_cl_P = feedback(Kp * Gp, 1);

% PI 제어기: Gc(s) = Kp + Ki/s = (Kp*s + Ki)/s
Kp_pi = 0.5; Ki_pi = 0.2;
Gc_pi = tf([Kp_pi, Ki_pi], [1, 0]);
sys_cl_PI = feedback(Gc_pi * Gp, 1);

% PID 제어기: Gc(s) = Kp + Ki/s + Kd*s = (Kd*s^2 + Kp*s + Ki)/s
Kp_pid = 1.5; Ki_pid = 0.5; Kd_pid = 1.0;
Gc_pid = tf([Kd_pid, Kp_pid, Ki_pid], [1, 0]);
sys_cl_PID = feedback(Gc_pid * Gp, 1);

% 계단 응답 계산
y_step_P   = step(sys_cl_P, t);
y_step_PI  = step(sys_cl_PI, t);
y_step_PID = step(sys_cl_PID, t);

% 경사(Ramp) 입력에 대한 응답 계산 (정상상태 오차 비교용)
% Ramp 입력 r(t) = t 이므로, 1/s^2 입력을 넣은 것과 같음. 
% step(sys/s)로 계산할 수 있습니다.
y_ramp_input = t';
y_ramp_P   = step(sys_cl_P / tf([1, 0], 1), t);
y_ramp_PI  = step(sys_cl_PI / tf([1, 0], 1), t);
y_ramp_PID = step(sys_cl_PID / tf([1, 0], 1), t);

%% 4. 시간 영역 성능 지표 분석 (StepInfo 사용)
% 안정적인 시스템에 대해서만 시간 영역 성능 분석 수행
info_P   = stepinfo(sys_cl_P);
info_PI  = stepinfo(sys_cl_PI);
info_PID = stepinfo(sys_cl_PID);

% 정상상태 계단 오차 (Step Steady-State Error)
% Type 1 시스템이므로 계단 입력에 대한 정상상태 오차는 이론적으로 0입니다.
ess_step_P   = 1 - y_step_P(end);
ess_step_PI  = 1 - y_step_PI(end);
ess_step_PID = 1 - y_step_PID(end);

% 정상상태 경사 오차 (Ramp Steady-State Error)
% e_ss(ramp) = lim s->0 s * (1/s^2) * (1 / (1 + Gc*Gp)) = lim s->0 1 / (s * (1 + Gc*Gp))
% P 제어 (Type 1): Gc*Gp = Kp * 10 / (s(s+1)(s+4)). s*Gc*Gp -> Kp * 10/4 = 2.5 * Kp. e_ss = 1 / (2.5 * Kp)
% PI, PID 제어 (Type 2): Gc에 분모 s가 붙어 루프 전달함수가 Type 2가 됨. 따라서 e_ss(ramp) = 0.
ess_ramp_P   = y_ramp_input(end) - y_ramp_P(end);
ess_ramp_PI  = y_ramp_input(end) - y_ramp_PI(end);
ess_ramp_PID = y_ramp_input(end) - y_ramp_PID(end);

fprintf('\n----------------------------------------------------\n');
fprintf('         시간 영역 성능 평가 결과 (Step Response)\n');
fprintf('----------------------------------------------------\n');
fprintf('제어기 유형      | 상승시간(tr) | 오버슈트(Mp) | 정착시간(ts) | 계단오차(ess)\n');
fprintf('P 제어 (Kp=0.5)  |   %6.2f s   |   %6.2f %%   |   %6.2f s   |   %6.4f\n', info_P.RiseTime, info_P.Overshoot, info_P.SettlingTime, ess_step_P);
fprintf('PI 제어 (I추가)  |   %6.2f s   |   %6.2f %%   |   %6.2f s   |   %6.4f\n', info_PI.RiseTime, info_PI.Overshoot, info_PI.SettlingTime, ess_step_PI);
fprintf('PID 제어 (D추가) |   %6.2f s   |   %6.2f %%   |   %6.2f s   |   %6.4f\n', info_PID.RiseTime, info_PID.Overshoot, info_PID.SettlingTime, ess_step_PID);
fprintf('----------------------------------------------------\n');

fprintf('\n----------------------------------------------------\n');
fprintf('         경사(Ramp) 입력에 대한 정상상태 오차 비교\n');
fprintf('----------------------------------------------------\n');
fprintf('P 제어 (Type 1)  | 정상상태 경사 오차 (ess_ramp) = %6.4f (이론치: %6.4f)\n', ess_ramp_P, 1/(2.5*Kp));
fprintf('PI 제어 (Type 2) | 정상상태 경사 오차 (ess_ramp) = %6.4f (이론치: 0.0000)\n', ess_ramp_PI);
fprintf('PID 제어 (Type 2)| 정상상태 경사 오차 (ess_ramp) = %6.4f (이론치: 0.0000)\n', ess_ramp_PID);
fprintf('====================================================\n');

%% 5. 고해상도 시각화 및 이미지 저장
% 고급스러운 프리미엄 컬러 팔레트 설정
colors = {
    [0.0, 0.4470, 0.7410], ... % Sleek Blue
    [0.8500, 0.3250, 0.0980], ... % Warm Orange
    [0.9290, 0.6940, 0.1250], ... % Mustard Yellow
    [0.4660, 0.6740, 0.1880], ... % Soft Green
    [0.4940, 0.1840, 0.5560]      % Regal Purple
};

% 그래픽 창 생성 (DPI 세팅 및 크기 조절)
fig = figure('Color', 'w', 'Position', [100, 100, 1200, 550]);

% --- Left Subplot: 비례 제어 이득(Kp)에 따른 안정성 경계 ---
subplot(1, 2, 1);
hold on;
grid on;
box on;
plot(t, ones(size(t)), '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off'); % 목표값 점선

p1 = plot(t, y_P1, 'Color', colors{1}, 'LineWidth', 2.0);
p2 = plot(t, y_P2, 'Color', colors{3}, 'LineWidth', 2.0);
p3 = plot(t, y_P3, 'Color', colors{2}, 'LineWidth', 2.0);

% 그래프 꾸미기
set(gca, 'FontSize', 12, 'LineWidth', 1.2, 'GridAlpha', 0.15);
xlabel('시간 (초)', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('시스템 출력 y(t)', 'FontSize', 13, 'FontWeight', 'bold');
title('\bf(a) 비례 이득(K_p) 변화와 안정성 한계', 'FontSize', 14, 'Color', [0.1 0.1 0.1]);
legend([p1, p2, p3], ...
       {sprintf('안정 (K_p = %.1f)', Kp_vals(1)), ...
        sprintf('임계 안정 (K_p = %.1f - 지속 진동)', Kp_vals(2)), ...
        sprintf('불안정 (K_p = %.1f - 발산)', Kp_vals(3))}, ...
       'Location', 'northeast', 'FontSize', 11);
ylim([-0.5, 3.5]);

% --- Right Subplot: P, PI, PID 제어기 과도/정상상태 응답 비교 ---
subplot(1, 2, 2);
hold on;
grid on;
box on;
plot(t, ones(size(t)), '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off'); % 목표값 점선

r1 = plot(t, y_step_P, 'Color', colors{1}, 'LineWidth', 2.0);
r2 = plot(t, y_step_PI, 'Color', colors{2}, 'LineWidth', 2.0);
r3 = plot(t, y_step_PID, 'Color', colors{4}, 'LineWidth', 2.0);

% 성능 지표 텍스트 박스 추가 (PID의 우수성 강조)
info_str = sprintf(['\\bf[시간 영역 성능 비교]\\rm\n', ...
                    '\\color[rgb]{0.0,0.447,0.741}■ P 제어:\\rm M_p = %.1f%%, t_r = %.2fs, t_s = %.2fs\n', ...
                    '\\color[rgb]{0.494,0.184,0.556}■ PI 제어:\\rm M_p = %.1f%%, t_r = %.2fs, t_s = %.2fs\n', ...
                    '\\color[rgb]{0.466,0.674,0.188}■ PID 제어:\\rm M_p = %.1f%%, t_r = %.2fs, t_s = %.2fs'], ...
                    info_P.Overshoot, info_P.RiseTime, info_P.SettlingTime, ...
                    info_PI.Overshoot, info_PI.RiseTime, info_PI.SettlingTime, ...
                    info_PID.Overshoot, info_PID.RiseTime, info_PID.SettlingTime);
annotation('textbox', [0.65, 0.18, 0.23, 0.22], 'String', info_str, ...
           'FitBoxToText', 'on', 'BackgroundColor', [0.97 0.97 0.97], ...
           'EdgeColor', [0.8 0.8 0.8], 'LineWidth', 1.0, 'FontSize', 10.5);

% 그래프 꾸미기
set(gca, 'FontSize', 12, 'LineWidth', 1.2, 'GridAlpha', 0.15);
xlabel('시간 (초)', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('시스템 출력 y(t)', 'FontSize', 13, 'FontWeight', 'bold');
title('\bf(b) P vs PI vs PID 계단 응답 비교', 'FontSize', 14, 'Color', [0.1 0.1 0.1]);
legend([r1, r2, r3], ...
       {'P 제어 (K_p = 0.5)', 'PI 제어 (K_p = 0.5, K_i = 0.2)', 'PID 제어 (K_p = 1.5, K_i = 0.5, K_d = 1.0)'}, ...
       'Location', 'northeast', 'FontSize', 11);
ylim([-0.2, 1.8]);

% 이미지 파일로 자동 저장
saveas(fig, 'pid_comparison.png');
fprintf('\n시각화 완료: "pid_comparison.png" 파일이 저장되었습니다.\n\n');
