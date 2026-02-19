%% =========================================================================
%  Monte Carlo – Two Master Windows (States + Servos/Motors)
%  =========================================================================
clc; close all; clear;

%% === CONFIG =============================================================
results_root = '\\wsl.localhost\Ubuntu\home\lat_avionics\Ardupilot_SITL_LATEST\Ardupilot_SILS\Tools\autotest\monte_carlo\mc_results_20260214_064957';
R2D = 180/pi;
save_fig = true;

%% === LOAD DATA ==========================================================
case_dirs = dir(fullfile(results_root,'case_*'));
case_dirs = case_dirs([case_dirs.isdir]);
data = {}; labels = {};

for k = 1:numel(case_dirs)
    f = dir(fullfile(results_root, case_dirs(k).name, 'sim_output*.csv'));
    if isempty(f), continue; end
    T = readtable(fullfile(results_root, case_dirs(k).name, f(1).name));
    if height(T) < 5, continue; end
    data{end+1} = T;
    labels{end+1} = case_dirs(k).name;
end
N = numel(data);
if N==0, error('No CSV data found.'); end
cmap = turbo(max(N,2));
fprintf('Loaded %d cases.\n', N);

%% === DARK THEME DEFAULTS ================================================
bg  = [0.08 0.08 0.08];   % near-black background
fg  = [1 1 1];             % white text
gc  = [0.3 0.3 0.3];       % dark-grey grid lines

%% === HELPER =============================================================
function ov(ax,D,cm,xc,yc,sc,yl,ttl,bg,fg,gc)
    hold(ax,'on'); grid(ax,'on');
    for k=1:numel(D)
        T=D{k}; if ~ismember(yc,T.Properties.VariableNames), continue; end
        plot(ax, T.(xc), T.(yc)*sc, 'Color',[cm(k,:) .85], 'LineWidth',1.1);
    end
    ylabel(ax,yl,'FontSize',10,'FontWeight','bold','Color',fg);
    xlabel(ax,'Time (s)','FontSize',9,'Color',fg);
    title(ax,ttl,'FontSize',11,'FontWeight','bold','Color',fg);
    set(ax,'FontSize',9,'Color',bg,'XColor',fg,'YColor',fg,...
           'GridColor',gc,'GridAlpha',0.5,'MinorGridColor',gc);
end

%% ====================== WINDOW 1 — STATES ==============================
fig1 = figure('Name','States','Units','normalized',...
              'OuterPosition',[0 0 1 1],'Color',bg);
t1 = tiledlayout(fig1, 5, 4, 'TileSpacing','tight','Padding','compact');
title(t1, sprintf('Vehicle States  (%d cases)', N), ...
      'FontSize',16,'FontWeight','bold','Color',fg);

% Row 1: Euler angles + TAS
ov(nexttile(t1), data, cmap, 'Time_s','phi',   R2D, 'deg', '\phi  Roll',bg,fg,gc);
ov(nexttile(t1), data, cmap, 'Time_s','theta', R2D, 'deg', '\theta  Pitch',bg,fg,gc);
ov(nexttile(t1), data, cmap, 'Time_s','psi',   R2D, 'deg', '\psi  Yaw',bg,fg,gc);
ov(nexttile(t1), data, cmap, 'Time_s','TAS_mps', 1, 'm/s', 'TAS',bg,fg,gc);

% Row 2: Body rates + Alt AGL
ov(nexttile(t1), data, cmap, 'Time_s','p', R2D, 'deg/s', 'p  Roll rate',bg,fg,gc);
ov(nexttile(t1), data, cmap, 'Time_s','q', R2D, 'deg/s', 'q  Pitch rate',bg,fg,gc);
ov(nexttile(t1), data, cmap, 'Time_s','r', R2D, 'deg/s', 'r  Yaw rate',bg,fg,gc);
ov(nexttile(t1), data, cmap, 'Time_s','alt_agl_m', 1, 'm', 'Alt AGL',bg,fg,gc);

% Row 3: Body velocities + Height
ov(nexttile(t1), data, cmap, 'Time_s','V_b_tas_0', 1, 'm/s', 'u',bg,fg,gc);
ov(nexttile(t1), data, cmap, 'Time_s','V_b_tas_1', 1, 'm/s', 'v',bg,fg,gc);
ov(nexttile(t1), data, cmap, 'Time_s','V_b_tas_2', 1, 'm/s', 'w',bg,fg,gc);
ov(nexttile(t1), data, cmap, 'Time_s','pos_ned_2',-1, 'm', 'Height (-D)',bg,fg,gc);

% Row 4: Body accelerations + angular accel
ov(nexttile(t1), data, cmap, 'Time_s','Accel_b_0', 1, 'm/s^2', 'a_x',bg,fg,gc);
ov(nexttile(t1), data, cmap, 'Time_s','Accel_b_1', 1, 'm/s^2', 'a_y',bg,fg,gc);
ov(nexttile(t1), data, cmap, 'Time_s','Accel_b_2', 1, 'm/s^2', 'a_z',bg,fg,gc);
ov(nexttile(t1), data, cmap, 'Time_s','p_dot', R2D, 'deg/s^2', 'p_{dot}',bg,fg,gc);

% Row 5: NED N/E + Ground track (wide)
ov(nexttile(t1), data, cmap, 'Time_s','pos_ned_0', 1, 'm', 'North',bg,fg,gc);
ov(nexttile(t1), data, cmap, 'Time_s','pos_ned_1', 1, 'm', 'East',bg,fg,gc);
ax_gt = nexttile(t1,[1 2]);
hold(ax_gt,'on'); grid(ax_gt,'on'); axis(ax_gt,'equal');
for k=1:N
    T=data{k};
    plot(ax_gt, T.pos_ned_1, T.pos_ned_0, 'Color',[cmap(k,:) .85],'LineWidth',1.1);
end
xlabel(ax_gt,'East (m)','FontSize',10,'Color',fg);
ylabel(ax_gt,'North (m)','FontSize',10,'Color',fg);
title(ax_gt,'Ground Track','FontSize',11,'FontWeight','bold','Color',fg);
set(ax_gt,'FontSize',9,'Color',bg,'XColor',fg,'YColor',fg,...
    'GridColor',gc,'GridAlpha',0.5);

if save_fig
    exportgraphics(fig1, fullfile(results_root,'mc_states.png'),...
                  'Resolution',250,'BackgroundColor',bg);
    fprintf('Saved: mc_states.png\n');
end

%% ================ WINDOW 2 — SERVOS & MOTORS ===========================
fig2 = figure('Name','Servos & Motors','Units','normalized',...
              'OuterPosition',[0 0 1 1],'Color',bg);
t2 = tiledlayout(fig2, 4, 4, 'TileSpacing','tight','Padding','compact');
title(t2, sprintf('Servo Commands & Motors  (%d cases)', N), ...
      'FontSize',16,'FontWeight','bold','Color',fg);

% Row 1: Control surface deflections (deg)
ov(nexttile(t2), data, cmap, 'Time_s','delta_e',  R2D, 'deg', '\delta_e  Elevator',bg,fg,gc);
ov(nexttile(t2), data, cmap, 'Time_s','delta_aL', R2D, 'deg', '\delta_{aL}  Ail-L',bg,fg,gc);
ov(nexttile(t2), data, cmap, 'Time_s','delta_aR', R2D, 'deg', '\delta_{aR}  Ail-R',bg,fg,gc);
ov(nexttile(t2), data, cmap, 'Time_s','delta_r',  R2D, 'deg', '\delta_r  Rudder',bg,fg,gc);

% Row 2: Flap + aileron + PWMs
ov(nexttile(t2), data, cmap, 'Time_s','delta_f', R2D, 'deg', '\delta_f  Flap',bg,fg,gc);
ov(nexttile(t2), data, cmap, 'Time_s','delta_a', R2D, 'deg', '\delta_a  Aileron',bg,fg,gc);
ov(nexttile(t2), data, cmap, 'Time_s','pwm_elev', 1, 'PWM', 'Elevator PWM',bg,fg,gc);
ov(nexttile(t2), data, cmap, 'Time_s','pwm_rud',  1, 'PWM', 'Rudder PWM',bg,fg,gc);

% Row 3: Servo PWMs
ov(nexttile(t2), data, cmap, 'Time_s','pwm_ailL', 1, 'PWM', 'Ail-L PWM',bg,fg,gc);
ov(nexttile(t2), data, cmap, 'Time_s','pwm_ailR', 1, 'PWM', 'Ail-R PWM',bg,fg,gc);
ov(nexttile(t2), data, cmap, 'Time_s','pwm_flap', 1, 'PWM', 'Flap PWM',bg,fg,gc);
ov(nexttile(t2), data, cmap, 'Time_s','pwm_nlg',  1, 'PWM', 'NLG PWM',bg,fg,gc);

% Row 4: Motor PWM + throttle cmd + rotor force + landing gear
ov(nexttile(t2), data, cmap, 'Time_s','pwm_mot0', 1, 'PWM', 'Motor PWM',bg,fg,gc);
ov(nexttile(t2), data, cmap, 'Time_s','total_rotor_force', 1, 'N', 'Rotor Force',bg,fg,gc);
ov(nexttile(t2), data, cmap, 'Time_s','MLG_NR', 1, 'N', 'MLG NR',bg,fg,gc);

if save_fig
    exportgraphics(fig2, fullfile(results_root,'mc_servos.png'),...
                  'Resolution',250,'BackgroundColor',bg);
    fprintf('Saved: mc_servos.png\n');
end

%% ================ WINDOW 3 — TAKEOFF DISTANCE ==========================
% Detect liftoff: first time alt_agl_m rises above a small threshold
% Integrate |V| = sqrt(u^2 + v^2 + w^2) from t=0 to t_liftoff

liftoff_thresh = 0.11;  % metres — just above ground
to_dist  = NaN(N,1);
to_time  = NaN(N,1);
to_speed = NaN(N,1);

for k = 1:N
    T = data{k};
    t   = T.Time_s;
    alt = T.alt_agl_m;
    u   = T.V_b_tas_0;
    v   = T.V_b_tas_1;
    w   = T.V_b_tas_2;
    Vmag = sqrt(u.^2 + v.^2 + w.^2);

    % Find first sample where altitude exceeds threshold
    idx = find(alt > liftoff_thresh, 1, 'first');
    if isempty(idx) || idx < 2, continue; end

    to_time(k)  = t(idx);
    to_dist(k)  = trapz(t(1:idx), Vmag(1:idx));
    to_speed(k) = T.TAS_mps(idx);   % TAS at liftoff
end

valid = ~isnan(to_dist);
fprintf('Takeoff distances computed for %d / %d cases.\n', sum(valid), N);

fig3 = figure('Name','Takeoff Distance & Speed','Units','normalized',...
              'OuterPosition',[0.10 0.10 0.8 0.8],'Color',bg);
t3 = tiledlayout(fig3, 2, 1, 'TileSpacing','compact','Padding','compact');
title(t3, sprintf('Takeoff Performance  (%d cases)', sum(valid)), ...
      'FontSize',16,'FontWeight','bold','Color',fg);

if sum(valid) >= 2
    % --- Top: Takeoff distance bar chart ---
    ax3a = nexttile(t3);
    hold(ax3a,'on'); grid(ax3a,'on');

    bar(ax3a, find(valid), to_dist(valid), 'FaceColor',[0.1 0.7 0.9],...
        'EdgeColor','none','FaceAlpha',0.85);

    mu_d  = mean(to_dist(valid));
    sig_d = std(to_dist(valid));
    yline(ax3a, mu_d, '--', sprintf('\\mu = %.1f m', mu_d),...
          'Color','r','LineWidth',1.5,'FontSize',11,...
          'LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom');
    yline(ax3a, mu_d + sig_d, ':', sprintf('+1\\sigma = %.1f m', mu_d+sig_d),...
          'Color',[1 .6 .2],'LineWidth',1.2,'FontSize',10,...
          'LabelHorizontalAlignment','left');
    yline(ax3a, max(0, mu_d - sig_d), ':', sprintf('-1\\sigma = %.1f m', mu_d-sig_d),...
          'Color',[1 .6 .2],'LineWidth',1.2,'FontSize',10,...
          'LabelHorizontalAlignment','left');

    xlabel(ax3a, 'Case #','FontSize',12,'Color',fg,'FontWeight','bold');
    ylabel(ax3a, 'Takeoff Ground Roll (m)','FontSize',12,'Color',fg,'FontWeight','bold');
    title(ax3a, sprintf('Takeoff Distance  (\\mu=%.1f m,  \\sigma=%.1f m)', ...
          mu_d, sig_d), 'FontSize',14,'FontWeight','bold','Color',fg);
    set(ax3a,'FontSize',10,'Color',bg,'XColor',fg,'YColor',fg,...
            'GridColor',gc,'GridAlpha',0.5);

    % --- Bottom: Takeoff speed bar chart ---
    ax3b = nexttile(t3);
    hold(ax3b,'on'); grid(ax3b,'on');

    bar(ax3b, find(valid), to_speed(valid), 'FaceColor',[0.9 0.4 0.1],...
        'EdgeColor','none','FaceAlpha',0.85);

    mu_v  = mean(to_speed(valid));
    sig_v = std(to_speed(valid));
    yline(ax3b, mu_v, '--', sprintf('\\mu = %.1f m/s', mu_v),...
          'Color','r','LineWidth',1.5,'FontSize',11,...
          'LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom');
    yline(ax3b, mu_v + sig_v, ':', sprintf('+1\\sigma = %.1f m/s', mu_v+sig_v),...
          'Color',[1 .6 .2],'LineWidth',1.2,'FontSize',10,...
          'LabelHorizontalAlignment','left');
    yline(ax3b, max(0, mu_v - sig_v), ':', sprintf('-1\\sigma = %.1f m/s', mu_v-sig_v),...
          'Color',[1 .6 .2],'LineWidth',1.2,'FontSize',10,...
          'LabelHorizontalAlignment','left');

    xlabel(ax3b, 'Case #','FontSize',12,'Color',fg,'FontWeight','bold');
    ylabel(ax3b, 'Liftoff TAS (m/s)','FontSize',12,'Color',fg,'FontWeight','bold');
    title(ax3b, sprintf('Takeoff Speed  (\\mu=%.1f m/s,  \\sigma=%.1f m/s)', ...
          mu_v, sig_v), 'FontSize',14,'FontWeight','bold','Color',fg);
    set(ax3b,'FontSize',10,'Color',bg,'XColor',fg,'YColor',fg,...
            'GridColor',gc,'GridAlpha',0.5);

elseif sum(valid) == 1
    ax3a = nexttile(t3);
    bar(ax3a, 1, to_dist(valid), 'FaceColor',[0.1 0.7 0.9],'EdgeColor','none');
    xlabel(ax3a,'Case','FontSize',12,'Color',fg);
    ylabel(ax3a,'Takeoff Distance (m)','FontSize',12,'Color',fg);
    title(ax3a, sprintf('Takeoff Distance = %.1f m', to_dist(valid)),...
          'FontSize',14,'FontWeight','bold','Color',fg);
    set(ax3a,'FontSize',10,'Color',bg,'XColor',fg,'YColor',fg,...
            'GridColor',gc,'GridAlpha',0.5);

    ax3b = nexttile(t3);
    bar(ax3b, 1, to_speed(valid), 'FaceColor',[0.9 0.4 0.1],'EdgeColor','none');
    xlabel(ax3b,'Case','FontSize',12,'Color',fg);
    ylabel(ax3b,'Liftoff TAS (m/s)','FontSize',12,'Color',fg);
    title(ax3b, sprintf('Liftoff Speed = %.1f m/s', to_speed(valid)),...
          'FontSize',14,'FontWeight','bold','Color',fg);
    set(ax3b,'FontSize',10,'Color',bg,'XColor',fg,'YColor',fg,...
            'GridColor',gc,'GridAlpha',0.5);
else
    ax3a = nexttile(t3);
    text(ax3a, 0.5, 0.5, 'No liftoff detected', 'Units','normalized',...
         'HorizontalAlignment','center','FontSize',14,'Color',fg);
    set(ax3a,'Color',bg,'XColor',fg,'YColor',fg);
end

if save_fig
    exportgraphics(fig3, fullfile(results_root,'mc_takeoff_distance.png'),...
                  'Resolution',250,'BackgroundColor',bg);
    fprintf('Saved: mc_takeoff_distance.png\n');
end

fprintf('Done.\n');
