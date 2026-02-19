%% Monte Carlo Postprocessing - Split Windows
clc; close all; clear;

% === USER CONFIGURATION ===
results_root = '\\wsl.localhost\Ubuntu\home\lat_avionics\Ardupilot_SITL_LATEST\Ardupilot_SILS\Tools\autotest\monte_carlo\mc_results_20260212_143407';
plot_count = 2;
R2D = 180/pi;
save_figures = true;

%% Discover cases
all_case_dirs = dir(fullfile(results_root,'case_*'));
all_case_dirs = all_case_dirs([all_case_dirs.isdir]);

if plot_count>0 && plot_count<numel(all_case_dirs)
    idx = randperm(numel(all_case_dirs),plot_count);
    case_dirs = all_case_dirs(idx);
else
    case_dirs = all_case_dirs;
end

%% Read Data
data = {};
servo = {};
labels = {};

num_cases = numel(case_dirs);

takeoff_times = zeros(num_cases,1);
takeoff_distances = zeros(num_cases,1);

for k=1:numel(case_dirs)
    cdir = fullfile(results_root,case_dirs(k).name);
    s1 = dir(fullfile(cdir,'*state_log.csv'));
    s2 = dir(fullfile(cdir,'*servo_cmds.csv'));
    if isempty(s1)||isempty(s2), continue; end

    T = sortrows(readtable(fullfile(cdir,s1(1).name)),'wall_time_s');
    S = sortrows(readtable(fullfile(cdir,s2(1).name)),'wall_time_s');

    % ensure servo columns
    for i=1:16
        v = "servo"+i;
        if ~ismember(v,S.Properties.VariableNames)
            S.(v)=NaN(height(S),1);
        end
    end

    data{end+1}=T;
    servo{end+1}=S;
    labels{end+1}=case_dirs(k).name;

        % === TAKEOFF DETECTION ===
    z=T.local_z_m;
    t=T.wall_time_s;
    v=T.airspeed_mps;

    idx_takeoff = find(z(1:end-1)>0 & z(2:end)<=0,1,'first')+1;

    if ~isempty(idx_takeoff)
        takeoff_times(k)=t(idx_takeoff);
        takeoff_distances(k)=trapz(t(1:idx_takeoff),v(1:idx_takeoff));
    end
end

num_cases=numel(data);
cmap=turbo(num_cases);

%% Overlay helper
function overlay(ax,data,cmap,x,y,scale,ylab,labels)
    hold(ax,'on'); grid(ax,'on'); box(ax,'on');
    for k=1:numel(data)
        T=data{k};
        if ~ismember(y,T.Properties.VariableNames), continue; end
        plot(ax,T.(x),T.(y)*scale,'Color',[cmap(k,:) 0.6],'LineWidth',1);
    end 
    ylabel(ax,ylab); xlabel(ax,'Time (s)');
    if numel(data)<=12, legend(ax,labels,'FontSize',7); end
end

%% ================= FIGURE 1 – STATES =================
fig1=figure('Name','Monte Carlo – States');
set(fig1,'Units','normalized','OuterPosition',[0 0 1 1]);
t1=tiledlayout(fig1,4,3,'TileSpacing','compact','Padding','compact');
title(t1,'Aircraft States','FontSize',16,'FontWeight','bold');

overlay(nexttile(t1),data,cmap,'wall_time_s','roll_rad',R2D,'Roll (deg)',labels);
overlay(nexttile(t1),data,cmap,'wall_time_s','pitch_rad',R2D,'Pitch (deg)',labels);
overlay(nexttile(t1),data,cmap,'wall_time_s','yaw_rad',R2D,'Yaw (deg)',labels);

overlay(nexttile(t1),data,cmap,'wall_time_s','local_x_m',1,'X (m)',labels);
overlay(nexttile(t1),data,cmap,'wall_time_s','local_y_m',1,'Y (m)',labels);
overlay(nexttile(t1),data,cmap,'wall_time_s','local_z_m',1,'Z (m)',labels);

overlay(nexttile(t1),data,cmap,'wall_time_s','local_vx_mps',1,'VX (m/s)',labels);
overlay(nexttile(t1),data,cmap,'wall_time_s','local_vy_mps',1,'VY (m/s)',labels);
overlay(nexttile(t1),data,cmap,'wall_time_s','local_vz_mps',1,'VZ (m/s)',labels);

overlay(nexttile(t1),data,cmap,'wall_time_s','airspeed_mps',1,'Airspeed',labels);
overlay(nexttile(t1),data,cmap,'wall_time_s','groundspeed_mps',1,'Groundspeed',labels);

if save_figures
    saveas(fig1,fullfile(results_root,'states.png'));
end

%% ================= FIGURE 2 – SERVOS (OPTIMIZED) =================
fig2=figure('Name','Monte Carlo – Servos');
set(fig2,'Units','normalized','OuterPosition',[0 0 1 1]);

t2=tiledlayout(fig2,2,3,'TileSpacing','compact','Padding','compact');
title(t2,'Primary Actuator Commands','FontSize',16,'FontWeight','bold');

% Motor - servo1
overlay(nexttile(t2),servo,cmap,'wall_time_s',"servo1",1,'Motor Throttle',labels);

% Rudder - servo6
overlay(nexttile(t2),servo,cmap,'wall_time_s',"servo6",1,'Rudder',labels);

% Aileron - servo11
overlay(nexttile(t2),servo,cmap,'wall_time_s',"servo11",1,'Aileron',labels);

% Flap - servo12
overlay(nexttile(t2),servo,cmap,'wall_time_s',"servo12",1,'Flap',labels);

% Elevator - servo13
overlay(nexttile(t2),servo,cmap,'wall_time_s',"servo13",1,'Elevator',labels);

if save_figures
    saveas(fig2,fullfile(results_root,'servos.png'));
end

%% ================= FIGURE 4 – TAKEOFF DISTANCE NORMAL DISTRIBUTION =================
valid_dist = takeoff_distances(~isnan(takeoff_distances));

mu = mean(valid_dist);
sigma = std(valid_dist);

fig4 = figure('Name','Takeoff Distance Distribution');
hold on; grid on; box on;

% Histogram normalized to PDF
histogram(valid_dist, 'Normalization','pdf');

% Normal curve
x = linspace(min(valid_dist), max(valid_dist), 200);
y = (1/(sigma*sqrt(2*pi))) * exp(-0.5*((x-mu)/sigma).^2);
plot(x, y, 'LineWidth', 2);

% Mean & Std lines
xline(mu, '--', 'Mean');
xline(mu+sigma, ':', '+1σ');
xline(mu-sigma, ':', '-1σ');

xlabel('Takeoff Distance (m)');
ylabel('Probability Density');
title('Normal Distribution – Takeoff Distance');

legend('Histogram','Normal Fit','Mean','±1σ');

hold off

if save_figures
    saveas(fig1,fullfile(results_root,'states.png'));
    saveas(fig2,fullfile(results_root,'servos.png'));
    saveas(fig4,fullfile(results_root,'takeoff_distance.png'));
end

%% ================= FIGURE – 2D XY TRAJECTORY WITH DIRECTION =================
figXY = figure('Name','2D Ground Track');
hold on; grid on; box on; axis equal;

for k = 1:num_cases
    T = data{k};
    if ~ismember('local_x_m',T.Properties.VariableNames) || ...
       ~ismember('local_y_m',T.Properties.VariableNames)
        continue;
    end

    x = T.local_x_m;
    y = T.local_y_m;

    % Shift so start point becomes (0,0)
    x = x - x(1);
    y = y - y(1);

    % Plot trajectory
    plot(x, y, 'LineWidth', 1.2);

    % Direction arrow near end
    if numel(x) > 5
        idx = round(numel(x)*0.9);
        quiver(x(idx), y(idx), ...
               x(idx+1)-x(idx), y(idx+1)-y(idx), ...
               0, 'MaxHeadSize', 2, 'LineWidth', 1);
    end

    % Start marker
    plot(0, 0, 'ko', 'MarkerFaceColor','k', 'MarkerSize',5);
end

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Vehicle 2D XY Ground Track (Start = 0,0)');
legend(case_labels,'Location','bestoutside');

hold off;

if save_figures
    saveas(figXY, fullfile(results_root,'xy_trajectory.png'));
end

disp('Visualization complete.');
