%% plot_qc_results.m
% Run after quarter_car_init + sim('quarter_car_active').
% Plots PID vs State Feedback step response comparison.
%
% Usage:
%   >> quarter_car_init
%   >> simOut = sim('quarter_car_active');
%   >> plot_qc_results(simOut)
%
% OR (if you ran in Simulink GUI):
%   >> plot_qc_results           % uses workspace yc_pid, yc_sf, tout

function plot_qc_results(simOut)

if nargin == 0
    % Try to use workspace variables from To Workspace blocks
    if ~exist('yc_pid','var') || ~exist('yc_sf','var')
        error(['No simulation data found.\n' ...
               'Run:  simOut = sim(''quarter_car_active'');\n' ...
               '      plot_qc_results(simOut)']);
    end
    yc_pid = evalin('base','yc_pid');
    yc_sf  = evalin('base','yc_sf');
    t      = evalin('base','tout');
else
    yc_pid = simOut.yc_pid;
    yc_sf  = simOut.yc_sf;
    t      = simOut.tout;
end

if ~exist('step_amp','var'), step_amp = evalin('base','step_amp'); end

%% Metrics (use array form: stepinfo(y, t, yf))
yf     = step_amp;
si_pid = stepinfo(yc_pid(:), t(:), yf, 'SettlingTimeThreshold', 0.02);
si_sf  = stepinfo(yc_sf(:),  t(:), yf, 'SettlingTimeThreshold', 0.02);

%% Plot
figure('Name','Quarter Car — PID vs State Feedback Comparison', ...
       'NumberTitle','off','Position',[80 80 1050 450]);

colors = struct('pid',[0.1 0.4 0.85], 'sf',[0.85 0.2 0.1], ...
                'ref',[0 0 0], 'lim',[0.7 0 0]);

for col = 1:2
    subplot(1,2,col);
    hold on; grid on;
    plot(t, yc_pid, '-',  'Color',colors.pid, 'LineWidth',2.0);
    plot(t, yc_sf,  '--', 'Color',colors.sf,  'LineWidth',2.0);
    yline(step_amp,      ':', 'Color',colors.ref, 'LineWidth',1.5);
    yline(step_amp*1.05, ':', 'Color',colors.lim, 'LineWidth',1.0);
    ylabel('Car body displacement  y_c  (m)');
    xlabel('Time  (s)');
    legend('PID Controller','State Feedback + Integral','Reference (0.05 m)', ...
           '5% overshoot limit','Location','best','FontSize',9);
    if col == 1
        title(sprintf('Full response  (0 – %.0f s)', t(end)));
        xlim([0 t(end)]);
    else
        title('Zoom: first 2 s');
        xlim([0 min(2, t(end))]);
    end
end

%% Summary table
fprintf('\n=== Simulink Simulation Results ===\n');
fprintf('%-22s %12s %14s\n','Metric','PID','State Feedback');
fprintf('%s\n', repmat('-',50,1));
fprintf('%-22s %10.2f %% %12.2f %%\n','Overshoot',   si_pid.Overshoot,    si_sf.Overshoot);
fprintf('%-22s %11.3f s %12.3f s\n', 'Rise Time',    si_pid.RiseTime,     si_sf.RiseTime);
fprintf('%-22s %11.3f s %12.3f s\n', 'Settling Time',si_pid.SettlingTime, si_sf.SettlingTime);
fprintf('%-22s %11.5f m %12.5f m\n', 'Final value',  yc_pid(end),         yc_sf(end));
fprintf('%s\n', repmat('-',50,1));
fprintf('Specs: Overshoot < 5%%, Rise Time < 0.5 s\n');
end
