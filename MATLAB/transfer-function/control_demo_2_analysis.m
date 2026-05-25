%% 자동제어 강의용 예제 2: 루트 로커스, 보드 선도, 나이퀴스트 선도 기반 안정성 분석
% 작성일: 2026-05-21
% 설명: 3차 시스템에 대해 근궤적, Bode 선도, Nyquist 선도를 그리고 주파수 영역 성능지표를 평가합니다.
%       이 코드는 두 개의 고해상도 이미지(root_locus_analysis.png, frequency_stability_analysis.png)를 자동 저장합니다.

clear; clc; close all;

%% 1. 시스템(Plant) 정의
% Gp(s) = 10 / (s*(s+1)*(s+4))
num = 10;
den = [1, 5, 4, 0];
Gp = tf(num, den);

% 기준 루프 전달 함수 L(s) = Kp * Gp(s) (Kp = 0.5 적용)
Kp = 0.5;
L = Kp * Gp;

fprintf('====================================================\n');
fprintf('  [자동제어 예제 2] 주파수 영역 안정성 및 성능 해석\n');
fprintf('====================================================\n');
disp('개루프 전달함수 Gp(s):');
disp(Gp);
fprintf('비례 이득 Kp = %.1f 적용 시 루프 전달함수 L(s) = Kp * Gp(s):\n', Kp);
disp(L);

%% 2. 주파수 영역 성능 지표 계산
% Gain Margin, Phase Margin, Crossover Frequencies 계산
[GM_abs, PM, w_pc, w_gc] = margin(L);
GM_dB = 20 * log10(GM_abs); % 절대값이므로 dB로 변환

% 폐루프 시스템 T(s) = L(s) / (1 + L(s))
T = feedback(L, 1);

% 폐루프 대역폭 (Bandwidth, omega_BW) 계산
omega_BW = bandwidth(T);

% 폐루프 공진 피크(M_pw) 및 공진 주파수(w_r) 계산
w_freq = logspace(-2, 2, 1000);
[mag, ~, w] = bode(T, w_freq);
[mag_peak, peak_idx] = max(mag(:));
Mpw_dB = 20 * log10(mag_peak);
w_r = w(peak_idx);

fprintf('----------------------------------------------------\n');
fprintf('         주파수 영역 주요 성능 및 안정성 지표\n');
fprintf('----------------------------------------------------\n');
fprintf('1. 상대적 안정성 지표 (Open-Loop Margins):\n');
fprintf('   - 이득 여유 (Gain Margin, GM)       = %6.2f dB (이론치: 12.04 dB)\n', GM_dB);
fprintf('   - 위상 여유 (Phase Margin, PM)     = %6.2f deg\n', PM);
fprintf('   - 이득 교차 주파수 (w_gc)           = %6.2f rad/s\n', w_gc);
fprintf('   - 위상 교차 주파수 (w_pc)           = %6.2f rad/s (이론치: 2.00 rad/s)\n', w_pc);
fprintf('\n2. 폐루프 성능 지표 (Closed-Loop Metrics):\n');
fprintf('   - 대역폭 (Bandwidth, w_BW)          = %6.2f rad/s\n', omega_BW);
fprintf('   - 공진 피크치 (Resonant Peak, M_pw) = %6.2f dB\n', Mpw_dB);
fprintf('   - 공진 주파수 (Resonant Freq, w_r)  = %6.2f rad/s\n', w_r);
fprintf('====================================================\n');

%% 3. 근궤적(Root Locus) 시각화 및 분석
fig1 = figure('Color', 'w', 'Position', [100, 100, 700, 600]);
hold on;
grid on;
box on;

% 기본 rlocus 그리기
[r, k_r] = rlocus(Gp);
plot(real(r)', imag(r)', 'LineWidth', 1.8, 'Color', [0.3 0.3 0.3]);

% 개루프 극점(Poles) 및 영점(Zeros) 표시
poles = pole(Gp);
p_plot = plot(real(poles), imag(poles), 'x', 'MarkerSize', 12, 'LineWidth', 2.5, 'Color', [0.85 0.325 0.098]);

% 특정 비례 이득에 따른 폐루프 극점 계산 및 표시
% - Kp = 0.5 (안정)
poles_cl_1 = roots([1, 5, 4, 10*0.5]);
cl1 = plot(real(poles_cl_1), imag(poles_cl_1), 'o', 'MarkerSize', 9, 'LineWidth', 2.0, ...
           'MarkerFaceColor', [0 0.447 0.741], 'MarkerEdgeColor', [0 0.447 0.741]);

% - Kp = 2.0 (임계 안정 - 허수축 교차)
poles_cl_2 = roots([1, 5, 4, 10*2.0]);
cl2 = plot(real(poles_cl_2), imag(poles_cl_2), 'd', 'MarkerSize', 9, 'LineWidth', 2.0, ...
           'MarkerFaceColor', [0.929 0.694 0.125], 'MarkerEdgeColor', [0.929 0.694 0.125]);

% - Kp = 3.0 (불안정 - 우반평면 진입)
poles_cl_3 = roots([1, 5, 4, 10*3.0]);
cl3 = plot(real(poles_cl_3), imag(poles_cl_3), 's', 'MarkerSize', 9, 'LineWidth', 2.0, ...
           'MarkerFaceColor', [0.85 0.325 0.098], 'MarkerEdgeColor', [0.85 0.325 0.098]);

% 허수축(s = jw) 및 실수축 가이드라인 강조
ax = xlim;
ay = ylim;
plot([0 0], ay, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
plot(ax, [0 0], 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');

% 임계 안정점 텍스트 주석 추가
text(0.1, 2.0, '\bf s = j2\newline(K_{crit} = 2.0)', 'FontSize', 11, 'Color', [0.7 0.4 0]);
text(0.1, -2.0, '\bf s = -j2\newline(K_{crit} = 2.0)', 'FontSize', 11, 'Color', [0.7 0.4 0]);

% 그래프 스타일 설정
set(gca, 'FontSize', 12, 'LineWidth', 1.2, 'GridAlpha', 0.15);
xlabel('실수축 (Real Axis)', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('허수축 (Imaginary Axis)', 'FontSize', 13, 'FontWeight', 'bold');
title('\bf 개루프 시스템 G_p(s)의 근궤적 및 이득별 폐루프 극점', 'FontSize', 14);
legend([p_plot, cl1, cl2, cl3], ...
       {'개루프 극점 (s = 0, -1, -4)', ...
        sprintf('K_p = 0.5의 극점 (안정)'), ...
        sprintf('K_p = 2.0의 극점 (임계 안정)'), ...
        sprintf('K_p = 3.0의 극점 (불안정)')}, ...
       'Location', 'southwest', 'FontSize', 11);
xlim([-6.5, 2.5]);
ylim([-4.5, 4.5]);

saveas(fig1, 'root_locus_analysis.png');
fprintf('시각화 완료: "root_locus_analysis.png" 파일이 저장되었습니다.\n');

%% 4. Bode 선도 및 Nyquist 선도 시각화 (안정성 여유 종합 해석)
fig2 = figure('Color', 'w', 'Position', [100, 100, 1200, 550]);

% --- Left Subplot: Bode Diagram with Margins ---
subplot(1, 2, 1);
[mag_L, phase_L, w_L] = bode(L, w_freq);
mag_L = squeeze(mag_L);
phase_L = squeeze(phase_L);

% 상단: Magnitude Plot
subplot(2, 2, 1);
hold on; grid on; box on;
plot(w_L, 20*log10(mag_L), 'Color', [0.0, 0.4470, 0.7410], 'LineWidth', 2.2);
plot([w_freq(1) w_freq(end)], [0 0], 'r--', 'LineWidth', 1.0); % 0dB 선

% 이득 교차점 표시 (GM 계산용)
plot(w_gc, 0, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
plot([w_pc w_pc], [-60 20], 'k:', 'LineWidth', 1.2);
plot([w_gc w_gc], [-60 20], 'k:', 'LineWidth', 1.2);

% 이득 여유 표시선
mag_at_wpc = 20*log10(squeeze(bode(L, w_pc)));
plot([w_pc w_pc], [mag_at_wpc 0], 'r', 'LineWidth', 2.0);
text(w_pc*1.2, mag_at_wpc/2, sprintf('\\bfGM = %.2f dB', GM_dB), 'Color', 'r', 'FontSize', 10);

set(gca, 'XScale', 'log', 'FontSize', 11, 'GridAlpha', 0.15);
ylabel('크기 (Magnitude, dB)', 'FontSize', 11, 'FontWeight', 'bold');
title('\bf(a) 루프 전달함수 L(s)의 보드선도 (K_p = 0.5)', 'FontSize', 13);
xlim([0.05, 50]);
ylim([-60, 40]);

% 하단: Phase Plot
subplot(2, 2, 3);
hold on; grid on; box on;
plot(w_L, phase_L, 'Color', [0.0, 0.4470, 0.7410], 'LineWidth', 2.2);
plot([w_freq(1) w_freq(end)], [-180 -180], 'r--', 'LineWidth', 1.0); % -180도 선

% 위상 교차점 및 위상 여유 표시
plot(w_pc, -180, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
plot([w_pc w_pc], [-270 -90], 'k:', 'LineWidth', 1.2);
plot([w_gc w_gc], [-270 -90], 'k:', 'LineWidth', 1.2);

phase_at_wgc = squeeze(bode(L, w_gc));
[~, phase_at_wgc] = bode(L, w_gc);
plot([w_gc w_gc], [-180 phase_at_wgc], 'r', 'LineWidth', 2.0);
text(w_gc*0.2, (phase_at_wgc-180)/2, sprintf('\\bfPM = %.1f^o', PM), 'Color', 'r', 'FontSize', 10);

set(gca, 'XScale', 'log', 'FontSize', 11, 'GridAlpha', 0.15);
xlabel('주파수 (Frequency, rad/s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('위상 (Phase, deg)', 'FontSize', 11, 'FontWeight', 'bold');
xlim([0.05, 50]);
ylim([-270, -90]);

% --- Right Subplot: Nyquist Plot with Stability Criterion ---
subplot(1, 2, 2);
hold on;
grid on;
box on;

% 임의의 Kp = 3.0 (불안정) 상태의 Nyquist 비교를 위해 같이 플롯
L_unstable = 3.0 * Gp;
[re_u, im_u] = nyquist(L_unstable, w_freq);
[re_s, im_s] = nyquist(L, w_freq);

% Nyquist 곡선 그리기
ny1 = plot(squeeze(re_s), squeeze(im_s), 'Color', [0 0.447 0.741], 'LineWidth', 2.0);
% 대칭인 음의 주파수 대역 점선 표시
plot(squeeze(re_s), -squeeze(im_s), 'Color', [0 0.447 0.741], 'LineStyle', ':', 'LineWidth', 1.5, 'HandleVisibility', 'off');

ny2 = plot(squeeze(re_u), squeeze(im_u), 'Color', [0.85 0.325 0.098], 'LineWidth', 1.5, 'LineStyle', '--');
plot(squeeze(re_u), -squeeze(im_u), 'Color', [0.85 0.325 0.098], 'LineStyle', ':', 'LineWidth', 1.2, 'HandleVisibility', 'off');

% 임계점 (-1, 0) 표시
plot(-1, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
text(-1.3, 0.2, '\bf(-1, j0)', 'Color', 'r', 'FontSize', 11);

% 그래프 꾸미기
set(gca, 'FontSize', 12, 'LineWidth', 1.2, 'GridAlpha', 0.15);
xlabel('실수축 (Real Axis)', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('허수축 (Imaginary Axis)', 'FontSize', 13, 'FontWeight', 'bold');
title('\bf(b) 복소평면에서의 Nyquist 선도', 'FontSize', 13);
legend([ny1, ny2], ...
       {sprintf('안정 조건: K_p = 0.5 (감싸지 않음)'), ...
        sprintf('불안정 조건: K_p = 3.0 (2회 시계방향 감쌈)')}, ...
       'Location', 'southwest', 'FontSize', 10.5);
xlim([-2.5, 0.5]);
ylim([-2.0, 2.0]);

saveas(fig2, 'frequency_stability_analysis.png');
fprintf('시각화 완료: "frequency_stability_analysis.png" 파일이 저장되었습니다.\n\n');
