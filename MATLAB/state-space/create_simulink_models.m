%% 시뮬링크 모델 자동 생성 스크립트
% 이 스크립트는 'sf_normal_model.slx' 및 'sf_integral_model.slx' 모델 파일을
% 프로그래밍 방식으로 자동 생성하고 저장합니다.
% 이 스크립트를 한 번 실행하고 나면 'state_feedback_simulink_run.m'에서
% 워닝(Warning) 없이 시뮬링크로 완벽하게 시뮬레이션을 수행할 수 있습니다.

clear; clc; close all;

%% 1. 모델 파라미터 임시 정의 (블록 유효성 검사용)
r = 1.0; dist_val = 0.5;
A_act = [0 1; -1.5 -2.5]; B_act = [0; 1.2]; C_act = [1 0];
N_ff = 12; K_sf = [10 4]; K = [45 9]; Ki = 60;

%% 2. sf_normal_model.slx 자동 생성
model1 = 'sf_normal_model';
if exist(model1, 'file') == 4
    bdclose(model1);
    delete([model1 '.slx']);
end

new_system(model1);
open_system(model1);

% 블록 추가
add_block('simulink/Sources/Step', [model1 '/Step_r'], 'Position', [50, 100, 80, 130]);
add_block('simulink/Math Operations/Gain', [model1 '/Pre-scaler N'], 'Position', [120, 100, 160, 130]);
add_block('simulink/Math Operations/Sum', [model1 '/Sum_u'], 'Position', [200, 105, 220, 125]);
add_block('simulink/Math Operations/Sum', [model1 '/Sum_dist'], 'Position', [260, 105, 280, 125]);
add_block('simulink/Sources/Step', [model1 '/Step_dist'], 'Position', [200, 30, 230, 60]);
add_block('simulink/Continuous/State-Space', [model1 '/Plant_SS'], 'Position', [320, 95, 400, 135]);
add_block('simulink/Math Operations/Gain', [model1 '/Feedback_K'], 'Position', [240, 180, 280, 210]);
add_block('simulink/Math Operations/Gain', [model1 '/C_matrix_gain'], 'Position', [440, 100, 480, 130]);
add_block('simulink/Sinks/To Workspace', [model1 '/yout'], 'Position', [520, 105, 560, 125]);

% 블록 파라미터 설정
set_param([model1 '/Step_r'], 'Time', '0', 'Before', '0', 'After', 'r');
set_param([model1 '/Pre-scaler N'], 'Gain', 'N_ff');
set_param([model1 '/Sum_u'], 'Inputs', '+-');
set_param([model1 '/Sum_dist'], 'Inputs', '++');
set_param([model1 '/Step_dist'], 'Time', '5', 'Before', '0', 'After', 'dist_val');
set_param([model1 '/Plant_SS'], 'A', 'A_act', 'B', 'B_act', 'C', 'eye(2)', 'D', 'zeros(2,1)');
set_param([model1 '/Feedback_K'], 'Gain', 'K_sf', 'Multiplication', 'Matrix(u*K)', 'Orientation', 'left');
set_param([model1 '/C_matrix_gain'], 'Gain', 'C_act', 'Multiplication', 'Matrix(u*K)');
set_param([model1 '/yout'], 'SaveFormat', 'Timeseries', 'VariableName', 'yout');

% 라인 연결
add_line(model1, 'Step_r/1', 'Pre-scaler N/1');
add_line(model1, 'Pre-scaler N/1', 'Sum_u/1');
add_line(model1, 'Sum_u/1', 'Sum_dist/2');
add_line(model1, 'Step_dist/1', 'Sum_dist/1');
add_line(model1, 'Sum_dist/1', 'Plant_SS/1');
add_line(model1, 'Plant_SS/1', 'Feedback_K/1');
add_line(model1, 'Plant_SS/1', 'C_matrix_gain/1');
add_line(model1, 'Feedback_K/1', 'Sum_u/2');
add_line(model1, 'C_matrix_gain/1', 'yout/1');

save_system(model1);
close_system(model1);
fprintf('성공: "%s.slx" 파일이 자동으로 생성 및 저장되었습니다.\n', model1);

%% 3. sf_integral_model.slx 자동 생성
model2 = 'sf_integral_model';
if exist(model2, 'file') == 4
    bdclose(model2);
    delete([model2 '.slx']);
end

new_system(model2);
open_system(model2);

% 블록 추가
add_block('simulink/Sources/Step', [model2 '/Step_r'], 'Position', [50, 100, 80, 130]);
add_block('simulink/Math Operations/Sum', [model2 '/Sum_err'], 'Position', [120, 105, 140, 125]);
add_block('simulink/Continuous/Integrator', [model2 '/Integrator'], 'Position', [180, 100, 210, 130]);
add_block('simulink/Math Operations/Gain', [model2 '/Gain_Ki'], 'Position', [250, 100, 290, 130]);
add_block('simulink/Math Operations/Sum', [model2 '/Sum_u'], 'Position', [330, 105, 350, 125]);
add_block('simulink/Math Operations/Sum', [model2 '/Sum_dist'], 'Position', [390, 105, 410, 125]);
add_block('simulink/Sources/Step', [model2 '/Step_dist'], 'Position', [330, 30, 360, 60]);
add_block('simulink/Continuous/State-Space', [model2 '/Plant_SS'], 'Position', [450, 95, 530, 135]);
add_block('simulink/Math Operations/Gain', [model2 '/Feedback_K'], 'Position', [370, 180, 410, 210]);
add_block('simulink/Math Operations/Gain', [model2 '/C_matrix_gain'], 'Position', [570, 100, 610, 130]);
add_block('simulink/Sinks/To Workspace', [model2 '/yout'], 'Position', [660, 105, 700, 125]);

% 블록 파라미터 설정
set_param([model2 '/Step_r'], 'Time', '0', 'Before', '0', 'After', 'r');
set_param([model2 '/Sum_err'], 'Inputs', '+-');
set_param([model2 '/Gain_Ki'], 'Gain', 'Ki');
set_param([model2 '/Sum_u'], 'Inputs', '+-');
set_param([model2 '/Sum_dist'], 'Inputs', '++');
set_param([model2 '/Step_dist'], 'Time', '5', 'Before', '0', 'After', 'dist_val');
set_param([model2 '/Plant_SS'], 'A', 'A_act', 'B', 'B_act', 'C', 'eye(2)', 'D', 'zeros(2,1)');
set_param([model2 '/Feedback_K'], 'Gain', 'K', 'Multiplication', 'Matrix(u*K)', 'Orientation', 'left');
set_param([model2 '/C_matrix_gain'], 'Gain', 'C_act', 'Multiplication', 'Matrix(u*K)');
set_param([model2 '/yout'], 'SaveFormat', 'Timeseries', 'VariableName', 'yout');

% 라인 연결
add_line(model2, 'Step_r/1', 'Sum_err/1');
add_line(model2, 'Sum_err/1', 'Integrator/1');
add_line(model2, 'Integrator/1', 'Gain_Ki/1');
add_line(model2, 'Gain_Ki/1', 'Sum_u/1');
add_line(model2, 'Sum_u/1', 'Sum_dist/2');
add_line(model2, 'Step_dist/1', 'Sum_dist/1');
add_line(model2, 'Sum_dist/1', 'Plant_SS/1');
add_line(model2, 'Plant_SS/1', 'Feedback_K/1');
add_line(model2, 'Plant_SS/1', 'C_matrix_gain/1');
add_line(model2, 'Feedback_K/1', 'Sum_u/2');
add_line(model2, 'C_matrix_gain/1', 'yout/1');
add_line(model2, 'C_matrix_gain/1', 'Sum_err/2');

save_system(model2);
close_system(model2);
fprintf('성공: "%s.slx" 파일이 자동으로 생성 및 저장되었습니다.\n', model2);
disp('이제 "state_feedback_simulink_run.m"을 실행하여 시뮬링크 연동 제어 시뮬레이션을 즐겨보세요!');
