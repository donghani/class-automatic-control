%% 상태 피드백 제어 vs 적분 제어 시뮬링크 연동 시뮬레이션
% 이 스크립트는 모델 불확실성과 외란이 있을 때 상태 피드백 및 적분 제어기의 성능을 비교합니다.
% 시뮬링크 모델('sf_normal_model' 및 'sf_integral_model')을 실행하기 전 이 스크립트를 먼저 실행하세요.

clear; clc; close all;

%% 1. 시스템 파라미터 정의
% 제어기 설계용 '공칭 모델' (Nominal Model)
a0_nom = 2; a1_nom = 3; b_nom = 1;
A_nom = [0 1; -a0_nom -a1_nom];
B_nom = [0; b_nom];
C_nom = [1 0]; % 출력 y = x1
D_nom = 0;

% 실제 물리 시스템 (Actual Model - 파라미터 오차 반영)
a0_act = 1.5; a1_act = 2.5; b_act = 1.2;
A_act = [0 1; -a0_act -a1_act];
B_act = [0; b_act];
C_act = [1 0];
D_act = 0;

% 목표값 및 외란 조건 (시뮬링크 Step 블록에서 사용)
r = 1.0;          % 목표값
dist_val = 0.5;   % t = 5초에 인가될 입력단 상수 외란

%% 2. 제어기 설계 (극 배치법, Pole Placement)
% --- 방식 A: 일반 상태 피드백 ---
poles_sf = [-3, -4]; % 원하는 폐루프 극점
K_sf = acker(A_nom, B_nom, poles_sf);
% 피드포워드 이득 N 계산
N_ff = 1 / (-C_nom * ((A_nom - B_nom * K_sf) \ B_nom));

% --- 방식 B: 적분 제어가 포함된 상태 피드백 ---
A_aug = [A_nom, zeros(2,1); -C_nom, 0];
B_aug = [B_nom; 0];
poles_aug = [-3, -4, -5]; % 원하는 확장 폐루프 극점
K_aug = acker(A_aug, B_aug, poles_aug);

K = K_aug(1:2);
Ki = -K_aug(3); % u = -K*x + Ki*xi

%% 3. 시뮬링크 모델 실행
disp('시뮬링크 모델을 시뮬레이션하는 중...');

% Simulink 1: 일반 상태 피드백 모델 실행
try
    out_sf = sim('sf_normal_model');
    t_sf = out_sf.tout;
    y_sf = out_sf.yout;
    disp('일반 상태 피드백 모델 시뮬레이션 완료.');
catch
    warning('sf_normal_model.slx 파일이 없거나 오류가 발생했습니다. 수치 시뮬레이션으로 대체합니다.');
    tspan = 0:0.01:10;
    x0_sf = [0; 0];
    [t_sf, x_sf] = ode45(@(t, x) [0 1; -a0_act -a1_act]*x + [0; b_act]*(-K_sf*x + N_ff*r + (t>=5)*dist_val), tspan, x0_sf);
    y_sf = x_sf(:, 1);
end

% Simulink 2: 적분 제어 결합 상태 피드백 모델 실행
try
    out_int = sim('sf_integral_model');
    t_int = out_int.tout;
    y_int = out_int.yout;
    disp('적분 피드백 모델 시뮬레이션 완료.');
catch
    warning('sf_integral_model.slx 파일이 없거나 오류가 발생했습니다. 수치 시뮬레이션으로 대체합니다.');
    tspan = 0:0.01:10;
    x0_int = [0; 0; 0];
    [t_int, x_int] = ode45(@(t, x_aug) [ [0 1; -a0_act -a1_act], [0; 0]; -C_act, 0]*x_aug + [0; b_act; 0]*(-K*x_aug(1:2) + Ki*x_aug(3) + (t>=5)*dist_val) + [0; 0; 1]*r, tspan, x0_int);
    y_int = x_int(:, 1);
end

%% 4. 결과 시각화
figure('Position', [100, 100, 900, 500]);
plot(t_sf, y_sf, 'r--', 'LineWidth', 2.0); hold on;
plot(t_int, y_int, 'b-', 'LineWidth', 2.0);
yline(r, 'k:', 'LineWidth', 1.5);
xline(5, 'r:', 't = 5s (외란 유입)', 'LabelVerticalAlignment', 'bottom', 'LineWidth', 1.2);

grid on;
xlabel('시간 (Time, s)', 'FontSize', 12);
ylabel('출력 (Output, y)', 'FontSize', 12);
title('시뮬링크 연동: 모델 불확실성 및 외란 조건 제어 성능 비교', 'FontSize', 14);
legend('일반 상태 피드백 (u = -Kx + Nr)', '적분 피드백 (u = -Kx + Ki*xi)', '목표값 (r)', 'Location', 'SouthEast');
