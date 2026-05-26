%% build_qc_model.m
% Builds the Quarter Car Active Suspension Simulink model.
% Run AFTER quarter_car_init.m.
%
% Model layout:
%   [Step_Ref] → [PID_CL]  → [Mux] → [Scope]
%              → [SF_CL]   ↗
%   [Mc_val] [Mw_val]  (Constant blocks — double-click to change mass,
%                        then re-run; PreSimFcn redesigns controllers)

if ~exist('A_pid_cl','var') || ~exist('A_sf_cl','var')
    error('Run quarter_car_init.m first to populate workspace.');
end

mdl = 'quarter_car_active';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);
set_param(mdl, 'Solver','ode45', 'StopTime','5', 'RelTol','1e-4', 'AbsTol','1e-6');

%% ---- local layout helper ----
at = @(x,y,w,h) [x, y, x+w, y+h];

%% ---- Step Reference ----
add_block('simulink/Sources/Step', [mdl '/Step_Ref'], ...
    'Position', at(40,215,55,30), ...
    'Time','0', 'Before','0', 'After','step_amp');

%% ---- PID Closed-Loop (State Space) ----
add_block('simulink/Continuous/State-Space', [mdl '/PID_CL'], ...
    'Position', at(180,140,120,50), ...
    'A','A_pid_cl','B','B_pid_cl','C','C_pid_cl','D','D_pid_cl', ...
    'X0','zeros(size(A_pid_cl,1),1)');

%% ---- State Feedback Closed-Loop (State Space) ----
add_block('simulink/Continuous/State-Space', [mdl '/SF_CL'], ...
    'Position', at(180,285,120,50), ...
    'A','A_sf_cl','B','B_sf_cl','C','C_sf_cl','D','D_sf_cl', ...
    'X0','zeros(size(A_sf_cl,1),1)');

%% ---- Mux (2→1) → Scope ----
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_out'], ...
    'Position', at(370,170,15,145), 'Inputs','2');

add_block('simulink/Sinks/Scope', [mdl '/Scope_yc'], ...
    'Position', at(440,220,55,45), 'NumInputPorts','1');

%% ---- To Workspace ----
add_block('simulink/Sinks/To Workspace', [mdl '/yc_pid_ws'], ...
    'Position', at(370,130,90,25), ...
    'VariableName','yc_pid', 'SaveFormat','Array');

add_block('simulink/Sinks/To Workspace', [mdl '/yc_sf_ws'], ...
    'Position', at(370,320,90,25), ...
    'VariableName','yc_sf', 'SaveFormat','Array');

% Note: tout is automatically provided by sim() output object

%% ---- Constant blocks for Mc, Mw (user edits double-click) ----
add_block('simulink/Sources/Constant', [mdl '/Mc_val'], ...
    'Position', at(40,310,70,30), 'Value','Mc');
add_block('simulink/Sources/Constant', [mdl '/Mw_val'], ...
    'Position', at(40,360,70,30), 'Value','Mw');

add_block('simulink/Sinks/Terminator', [mdl '/Mc_term'], 'Position', at(160,310,20,20));
add_block('simulink/Sinks/Terminator', [mdl '/Mw_term'], 'Position', at(160,360,20,20));

%% ---- Connections ----
add_line(mdl, 'Step_Ref/1', 'PID_CL/1', 'autorouting','on');
add_line(mdl, 'Step_Ref/1', 'SF_CL/1',  'autorouting','on');

add_line(mdl, 'PID_CL/1', 'Mux_out/1',  'autorouting','on');
add_line(mdl, 'SF_CL/1',  'Mux_out/2',  'autorouting','on');
add_line(mdl, 'Mux_out/1','Scope_yc/1', 'autorouting','on');

add_line(mdl, 'PID_CL/1', 'yc_pid_ws/1','autorouting','on');
add_line(mdl, 'SF_CL/1',  'yc_sf_ws/1', 'autorouting','on');
add_line(mdl, 'Mc_val/1', 'Mc_term/1',  'autorouting','on');
add_line(mdl, 'Mw_val/1', 'Mw_term/1',  'autorouting','on');

%% ---- PreSimFcn: auto-redesign on Mc/Mw change ----
set_param(mdl, 'InitFcn', 'quarter_car_recalc_callback');

%% ---- Save ----
slx_path = fullfile(fileparts(which('build_qc_model')), [mdl '.slx']);
if isempty(slx_path) || strcmp(slx_path, ['./' mdl '.slx'])
    slx_path = fullfile(pwd, [mdl '.slx']);
end
save_system(mdl, slx_path);

fprintf('\n=======================================================\n');
fprintf('  Simulink model  "%s"  ready.\n', mdl);
fprintf('=======================================================\n');
fprintf('  Scope  → channel 1 = PID,  channel 2 = State FB\n');
fprintf('  Mc_val / Mw_val → double-click to change mass values\n');
fprintf('  PreSimFcn recalculates controllers automatically.\n');
fprintf('  Press Ctrl+T or the Run button to simulate.\n');
fprintf('  Afterwards: >> plot_qc_results\n\n');
