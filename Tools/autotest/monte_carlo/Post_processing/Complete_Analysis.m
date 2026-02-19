%% =========================================================================
%  Monte Carlo Post-Processing & Visualization
%  =========================================================================
%  Reads sim_output_*.csv files (from Output.cpp) and summary.csv produced
%  by the LAT Monte Carlo runner.
%
%  CSV columns (sim_output):
%    Time_s, plane_moving_state, TAS_mps, alt_agl_m, MLG_NR, FLG_NR,
%    lat, lon, alt_msl, phi, theta, psi, p, q, r,
%    V_b_tas_0, V_b_tas_1, V_b_tas_2, pos_ned_0, pos_ned_1, pos_ned_2,
%    p_dot, q_dot, r_dot, Accel_b_0, Accel_b_1, Accel_b_2,
%    ArduPlane_Mode, delta_e, delta_aL, delta_aR, delta_r, delta_f, delta_a,
%    pwm_ailL, pwm_ailR, pwm_elev, pwm_rud, pwm_flap, pwm_nlg,
%    pwm_mot0, pwm_mot1, pwm_mot2, pwm_mot3,
%    mot0_thr_cmd, mot1_thr_cmd, mot2_thr_cmd, mot3_thr_cmd,
%    total_rotor_force
%  =========================================================================
clc; close all; clear;

%% ======================== USER CONFIGURATION ============================
results_root = '\\wsl.localhost\Ubuntu\home\lat_avionics\Ardupilot_SITL_LATEST\Ardupilot_SILS\Tools\autotest\monte_carlo\mc_results_20260217_074641';

max_cases_to_plot = 1clos0;       % 0 = plot ALL cases; >0 = randomly pick N
R2D              = 180/pi;
save_figures     = true;
fig_format       = '-dpng';  % '-dpng', '-dsvg', '-dpdf'
fig_dpi          = '-r200';

%% ======================== STYLE DEFAULTS ================================
set(0, 'DefaultAxesFontSize', 11);
set(0, 'DefaultAxesFontName', 'Segoe UI');
set(0, 'DefaultLineLineWidth', 1.1);
set(0, 'DefaultAxesBox', 'on');
set(0, 'DefaultAxesGridAlpha', 0.2);
set(0, 'DefaultAxesLineWidth', 0.8);

%% ======================== DISCOVER CASES ================================
all_case_dirs = dir(fullfile(results_root, 'case_*'));
all_case_dirs = all_case_dirs([all_case_dirs.isdir]);
fprintf('Found %d case directories.\n', numel(all_case_dirs));

if max_cases_to_plot > 0 && max_cases_to_plot < numel(all_case_dirs)
    idx = sort(randperm(numel(all_case_dirs), max_cases_to_plot));
    case_dirs = all_case_dirs(idx);
else
    case_dirs = all_case_dirs;
end

%% ======================== READ sim_output CSVs ==========================
data   = {};
labels = {};

for k = 1:numel(case_dirs)
    cdir = fullfile(results_root, case_dirs(k).name);
    csv_files = dir(fullfile(cdir, 'sim_output*.csv'));
    if isempty(csv_files)
        fprintf('  [SKIP] %s – no sim_output CSV\n', case_dirs(k).name);
        continue;
    end
    T = readtable(fullfile(cdir, csv_files(1).name));
    if isempty(T) || height(T) < 3
        fprintf('  [SKIP] %s – CSV too short (%d rows)\n', case_dirs(k).name, height(T));
        continue;
    end
    data{end+1}   = T; %#ok<SAGROW>
    labels{end+1} = case_dirs(k).name; %#ok<SAGROW>
end

num_cases = numel(data);
fprintf('Loaded %d cases for plotting.\n', num_cases);

if num_cases == 0
    error('No valid sim_output CSV files found in %s', results_root);
end

cmap = turbo(max(num_cases, 2));

%% ======================== READ summary.csv ==============================
summary_file = fullfile(results_root, 'summary.csv');
has_summary = isfile(summary_file);
if has_summary
    summary = readtable(summary_file);
    fprintf('Summary: %d rows loaded.\n', height(summary));
end

%% ======================== HELPER FUNCTIONS ==============================

function overlay(ax, data, cmap, xcol, ycol, scale, ylab, labels)
    hold(ax,'on'); grid(ax,'on'); box(ax,'on');
    for k = 1:numel(data)
        T = data{k};
        if ~ismember(ycol, T.Properties.VariableNames), continue; end
        plot(ax, T.(xcol), T.(ycol) * scale, ...
             'Color', [cmap(k,:) 0.55], 'LineWidth', 1.0);
    end
    ylabel(ax, ylab, 'FontWeight','bold');
    xlabel(ax, 'Time (s)');
    if numel(data) <= 15
        legend(ax, labels, 'FontSize', 7, 'Location', 'best');
    end
end

function overlay_multi(ax, data, cmap, xcol, ycols, scale, ylab, leg_names, labels)
    % Plot multiple columns on same axes with different line styles
    hold(ax,'on'); grid(ax,'on'); box(ax,'on');
    styles = {'-', '--', ':', '-.'};
    for k = 1:numel(data)
        T = data{k};
        for j = 1:numel(ycols)
            if ~ismember(ycols{j}, T.Properties.VariableNames), continue; end
            ls = styles{mod(j-1, numel(styles))+1};
            plot(ax, T.(xcol), T.(ycols{j}) * scale, ls, ...
                 'Color', [cmap(k,:) 0.6], 'LineWidth', 1.0);
        end
    end
    ylabel(ax, ylab, 'FontWeight','bold');
    xlabel(ax, 'Time (s)');
    if numel(ycols) <= 6
        legend(ax, leg_names, 'FontSize', 7, 'Location','best');
    end
end

function savefig_custom(fig, results_root, name, fmt, dpi)
    exportgraphics(fig, fullfile(results_root, [name '.png']), 'Resolution', 200);
    fprintf('  Saved %s.png\n', name);
end

%% =========================================================================
%  FIGURE 1 — Euler Angles & Body Rates
%  =========================================================================
fig1 = figure('Name','Euler Angles & Body Rates', ...
              'Units','normalized','OuterPosition',[0 0 1 1]);
t1 = tiledlayout(fig1, 2, 3, 'TileSpacing','compact','Padding','compact');
title(t1, 'Euler Angles & Body Rates', 'FontSize', 16, 'FontWeight','bold');

overlay(nexttile(t1), data, cmap, 'Time_s','phi',   R2D, '\phi  Roll (deg)',   labels);
overlay(nexttile(t1), data, cmap, 'Time_s','theta', R2D, '\theta  Pitch (deg)', labels);
overlay(nexttile(t1), data, cmap, 'Time_s','psi',   R2D, '\psi  Yaw (deg)',     labels);
overlay(nexttile(t1), data, cmap, 'Time_s','p',     R2D, 'p  Roll rate (deg/s)',  labels);
overlay(nexttile(t1), data, cmap, 'Time_s','q',     R2D, 'q  Pitch rate (deg/s)', labels);
overlay(nexttile(t1), data, cmap, 'Time_s','r',     R2D, 'r  Yaw rate (deg/s)',   labels);

if save_figures, savefig_custom(fig1, results_root, 'fig1_euler_rates', fig_format, fig_dpi); end

%% =========================================================================
%  FIGURE 2 — NED Position
%  =========================================================================
fig2 = figure('Name','NED Position', ...
              'Units','normalized','OuterPosition',[0 0 1 1]);
t2 = tiledlayout(fig2, 2, 2, 'TileSpacing','compact','Padding','compact');
title(t2, 'NED Position & Altitude', 'FontSize', 16, 'FontWeight','bold');

overlay(nexttile(t2), data, cmap, 'Time_s','pos_ned_0', 1, 'North (m)', labels);
overlay(nexttile(t2), data, cmap, 'Time_s','pos_ned_1', 1, 'East (m)',  labels);
overlay(nexttile(t2), data, cmap, 'Time_s','pos_ned_2',-1, 'Height -D (m)', labels);
overlay(nexttile(t2), data, cmap, 'Time_s','alt_agl_m', 1, 'Alt AGL (m)',   labels);

if save_figures, savefig_custom(fig2, results_root, 'fig2_ned_position', fig_format, fig_dpi); end

%% =========================================================================
%  FIGURE 3 — Body Velocities & Airspeed
%  =========================================================================
fig3 = figure('Name','Body Velocities & Airspeed', ...
              'Units','normalized','OuterPosition',[0 0 1 1]);
t3 = tiledlayout(fig3, 2, 2, 'TileSpacing','compact','Padding','compact');
title(t3, 'Body Velocities & Airspeed', 'FontSize', 16, 'FontWeight','bold');

overlay(nexttile(t3), data, cmap, 'Time_s','V_b_tas_0', 1, 'u  (m/s)',  labels);
overlay(nexttile(t3), data, cmap, 'Time_s','V_b_tas_1', 1, 'v  (m/s)',  labels);
overlay(nexttile(t3), data, cmap, 'Time_s','V_b_tas_2', 1, 'w  (m/s)',  labels);
overlay(nexttile(t3), data, cmap, 'Time_s','TAS_mps',   1, 'TAS (m/s)', labels);

if save_figures, savefig_custom(fig3, results_root, 'fig3_velocities', fig_format, fig_dpi); end

%% =========================================================================
%  FIGURE 4 — Body Accelerations & Rate Derivatives
%  =========================================================================
fig4 = figure('Name','Accelerations & Rate Dots', ...
              'Units','normalized','OuterPosition',[0 0 1 1]);
t4 = tiledlayout(fig4, 2, 3, 'TileSpacing','compact','Padding','compact');
title(t4, 'Body Accelerations & Angular Acceleration', ...
      'FontSize', 16, 'FontWeight','bold');

overlay(nexttile(t4), data, cmap, 'Time_s','Accel_b_0', 1, 'a_x  (m/s^2)', labels);
overlay(nexttile(t4), data, cmap, 'Time_s','Accel_b_1', 1, 'a_y  (m/s^2)', labels);
overlay(nexttile(t4), data, cmap, 'Time_s','Accel_b_2', 1, 'a_z  (m/s^2)', labels);
overlay(nexttile(t4), data, cmap, 'Time_s','p_dot', R2D, 'p_{dot} (deg/s^2)', labels);
overlay(nexttile(t4), data, cmap, 'Time_s','q_dot', R2D, 'q_{dot} (deg/s^2)', labels);
overlay(nexttile(t4), data, cmap, 'Time_s','r_dot', R2D, 'r_{dot} (deg/s^2)', labels);

if save_figures, savefig_custom(fig4, results_root, 'fig4_accelerations', fig_format, fig_dpi); end

%% =========================================================================
%  FIGURE 5 — Control Surface Deflections (rad → deg)
%  =========================================================================
fig5 = figure('Name','Control Surface Deflections', ...
              'Units','normalized','OuterPosition',[0 0 1 1]);
t5 = tiledlayout(fig5, 2, 3, 'TileSpacing','compact','Padding','compact');
title(t5, 'Control Surface Deflections', 'FontSize', 16, 'FontWeight','bold');

overlay(nexttile(t5), data, cmap, 'Time_s','delta_e',  R2D, '\delta_e  Elevator (deg)', labels);
overlay(nexttile(t5), data, cmap, 'Time_s','delta_aL', R2D, '\delta_{aL}  Ail-L (deg)',  labels);
overlay(nexttile(t5), data, cmap, 'Time_s','delta_aR', R2D, '\delta_{aR}  Ail-R (deg)',  labels);
overlay(nexttile(t5), data, cmap, 'Time_s','delta_r',  R2D, '\delta_r  Rudder (deg)',    labels);
overlay(nexttile(t5), data, cmap, 'Time_s','delta_f',  R2D, '\delta_f  Flap (deg)',      labels);
overlay(nexttile(t5), data, cmap, 'Time_s','delta_a',  R2D, '\delta_a  Aileron (deg)',   labels);

if save_figures, savefig_custom(fig5, results_root, 'fig5_control_surfaces', fig_format, fig_dpi); end

%% =========================================================================
%  FIGURE 6 — PWM Inputs (Servo & Motor)
%  =========================================================================
fig6 = figure('Name','PWM Inputs', ...
              'Units','normalized','OuterPosition',[0 0 1 1]);
t6 = tiledlayout(fig6, 2, 3, 'TileSpacing','compact','Padding','compact');
title(t6, 'PWM Servo & Motor Inputs', 'FontSize', 16, 'FontWeight','bold');

overlay(nexttile(t6), data, cmap, 'Time_s','pwm_elev', 1, 'Elevator PWM',  labels);
overlay(nexttile(t6), data, cmap, 'Time_s','pwm_ailL', 1, 'Aileron-L PWM', labels);
overlay(nexttile(t6), data, cmap, 'Time_s','pwm_ailR', 1, 'Aileron-R PWM', labels);
overlay(nexttile(t6), data, cmap, 'Time_s','pwm_rud',  1, 'Rudder PWM',    labels);
overlay(nexttile(t6), data, cmap, 'Time_s','pwm_flap', 1, 'Flap PWM',      labels);
overlay(nexttile(t6), data, cmap, 'Time_s','pwm_nlg',  1, 'NLG PWM',       labels);

if save_figures, savefig_custom(fig6, results_root, 'fig6_pwm_servo', fig_format, fig_dpi); end

%% =========================================================================
%  FIGURE 7 — Motor Throttle Commands & Rotor Force
%  =========================================================================
fig7 = figure('Name','Motor Throttle & Rotor Force', ...
              'Units','normalized','OuterPosition',[0 0 1 1]);
t7 = tiledlayout(fig7, 2, 3, 'TileSpacing','compact','Padding','compact');
title(t7, 'Motor Throttle Commands & Total Rotor Force', ...
      'FontSize', 16, 'FontWeight','bold');

overlay(nexttile(t7), data, cmap, 'Time_s','mot0_thr_cmd', 1, 'Motor 0 Cmd', labels);
overlay(nexttile(t7), data, cmap, 'Time_s','mot1_thr_cmd', 1, 'Motor 1 Cmd', labels);
overlay(nexttile(t7), data, cmap, 'Time_s','mot2_thr_cmd', 1, 'Motor 2 Cmd', labels);
overlay(nexttile(t7), data, cmap, 'Time_s','mot3_thr_cmd', 1, 'Motor 3 Cmd', labels);
overlay(nexttile(t7), data, cmap, 'Time_s','total_rotor_force', 1, 'Total Rotor Force (N)', labels);

% Motor PWMs on last tile
ax_mpwm = nexttile(t7);
overlay_multi(ax_mpwm, data, cmap, 'Time_s', ...
    {'pwm_mot0','pwm_mot1','pwm_mot2','pwm_mot3'}, 1, ...
    'Motor PWMs', {'Mot0','Mot1','Mot2','Mot3'}, labels);

if save_figures, savefig_custom(fig7, results_root, 'fig7_motors', fig_format, fig_dpi); end

%% =========================================================================
%  FIGURE 8 — Landing Gear Normal Reactions
%  =========================================================================
fig8 = figure('Name','Landing Gear Loads', ...
              'Units','normalized','OuterPosition',[0 0 1 1]);
t8 = tiledlayout(fig8, 1, 2, 'TileSpacing','compact','Padding','compact');
title(t8, 'Landing Gear Normal Reactions', 'FontSize', 16, 'FontWeight','bold');

overlay(nexttile(t8), data, cmap, 'Time_s','MLG_NR', 1, 'MLG Normal Reaction (N)', labels);
overlay(nexttile(t8), data, cmap, 'Time_s','FLG_NR', 1, 'FLG Normal Reaction (N)', labels);

if save_figures, savefig_custom(fig8, results_root, 'fig8_landing_gear', fig_format, fig_dpi); end

%% =========================================================================
%  FIGURE 9 — 2D Ground Track (NED X-Y)
%  =========================================================================
fig9 = figure('Name','2D Ground Track', ...
              'Units','normalized','OuterPosition',[0 0.1 0.6 0.85]);
ax9 = axes(fig9);
hold(ax9,'on'); grid(ax9,'on'); box(ax9,'on'); axis(ax9,'equal');
title(ax9, '2D Ground Track  (North vs East)', 'FontSize', 14, 'FontWeight','bold');

for k = 1:num_cases
    T = data{k};
    if ~ismember('pos_ned_0', T.Properties.VariableNames), continue; end
    x = T.pos_ned_0;
    y = T.pos_ned_1;

    plot(ax9, y, x, 'Color', [cmap(k,:) 0.6], 'LineWidth', 1.2);

    % Start marker
    plot(ax9, y(1), x(1), 'o', 'Color', cmap(k,:), ...
         'MarkerFaceColor', cmap(k,:), 'MarkerSize', 6);

    % Direction arrow near 90% of trajectory
    if numel(x) > 10
        idx = round(numel(x)*0.85);
        quiver(ax9, y(idx), x(idx), ...
               y(idx+1)-y(idx), x(idx+1)-x(idx), ...
               15, 'Color', cmap(k,:), 'MaxHeadSize', 2, ...
               'LineWidth', 1.5);
    end
end
xlabel(ax9, 'East (m)'); ylabel(ax9, 'North (m)');
if num_cases <= 15
    legend(ax9, labels, 'Location','bestoutside', 'FontSize', 8);
end

if save_figures, savefig_custom(fig9, results_root, 'fig9_ground_track', fig_format, fig_dpi); end

%% =========================================================================
%  FIGURE 10 — 3D Trajectory
%  =========================================================================
fig10 = figure('Name','3D Trajectory', ...
               'Units','normalized','OuterPosition',[0 0 0.7 0.9]);
ax10 = axes(fig10);
hold(ax10,'on'); grid(ax10,'on'); box(ax10,'on');
title(ax10, '3D Flight Trajectory', 'FontSize', 14, 'FontWeight','bold');

for k = 1:num_cases
    T = data{k};
    if ~ismember('pos_ned_0', T.Properties.VariableNames), continue; end
    plot3(ax10, T.pos_ned_1, T.pos_ned_0, -T.pos_ned_2, ...
          'Color', [cmap(k,:) 0.6], 'LineWidth', 1.0);
end
xlabel(ax10, 'East (m)'); ylabel(ax10, 'North (m)'); zlabel(ax10, 'Height (m)');
view(ax10, 35, 25);
if num_cases <= 15
    legend(ax10, labels, 'Location','bestoutside', 'FontSize', 8);
end

if save_figures, savefig_custom(fig10, results_root, 'fig10_3d_trajectory', fig_format, fig_dpi); end

%% =========================================================================
%  FIGURE 11 — Lat/Lon Map View
%  =========================================================================
fig11 = figure('Name','Lat/Lon Ground Track', ...
               'Units','normalized','OuterPosition',[0 0 0.6 0.85]);
ax11 = axes(fig11);
hold(ax11,'on'); grid(ax11,'on'); box(ax11,'on'); axis(ax11,'equal');
title(ax11, 'Geo Ground Track  (Lat vs Lon)', 'FontSize', 14, 'FontWeight','bold');

for k = 1:num_cases
    T = data{k};
    if ~ismember('lat', T.Properties.VariableNames), continue; end
    % Skip if lat/lon are all zero (not populated)
    if all(T.lat == 0), continue; end
    plot(ax11, T.lon, T.lat, 'Color', [cmap(k,:) 0.6], 'LineWidth', 1.0);
    plot(ax11, T.lon(1), T.lat(1), 'o', 'Color', cmap(k,:), ...
         'MarkerFaceColor', cmap(k,:), 'MarkerSize', 5);
end
xlabel(ax11, 'Longitude (deg)'); ylabel(ax11, 'Latitude (deg)');
if num_cases <= 15
    legend(ax11, labels, 'Location','bestoutside', 'FontSize', 8);
end

if save_figures, savefig_custom(fig11, results_root, 'fig11_latlon', fig_format, fig_dpi); end

%% =========================================================================
%  FIGURE 12 — Flight Phase / Mode Timeline
%  =========================================================================
fig12 = figure('Name','Flight Phase Timeline', ...
               'Units','normalized','OuterPosition',[0 0 1 0.5]);
t12 = tiledlayout(fig12, 1, 2, 'TileSpacing','compact','Padding','compact');
title(t12, 'Flight Phase & ArduPlane Mode', 'FontSize', 16, 'FontWeight','bold');

overlay(nexttile(t12), data, cmap, 'Time_s','plane_moving_state', 1, ...
        'Plane Moving State', labels);
overlay(nexttile(t12), data, cmap, 'Time_s','ArduPlane_Mode', 1, ...
        'ArduPlane Mode Number', labels);

if save_figures, savefig_custom(fig12, results_root, 'fig12_flight_phase', fig_format, fig_dpi); end

%% =========================================================================
%  FIGURE 13 — Summary Statistics  (requires summary.csv)
%  =========================================================================
if has_summary && height(summary) >= 2
    fig13 = figure('Name','Monte Carlo Summary Statistics', ...
                   'Units','normalized','OuterPosition',[0 0 1 1]);
    t13 = tiledlayout(fig13, 2, 3, 'TileSpacing','compact','Padding','compact');
    title(t13, 'Monte Carlo Summary Statistics', ...
          'FontSize', 16, 'FontWeight','bold');

    % --- Max altitude distribution ---
    ax = nexttile(t13);
    histogram(ax, summary.max_alt_m, 'FaceColor', [0.2 0.6 0.9], ...
              'EdgeColor','w', 'FaceAlpha', 0.85);
    xlabel(ax, 'Max Altitude (m)'); ylabel(ax, 'Count');
    title(ax, 'Max Altitude Distribution');
    grid(ax,'on');
    xline(ax, mean(summary.max_alt_m), 'r--', ...
          sprintf('\\mu=%.1f', mean(summary.max_alt_m)), 'LineWidth', 1.5);

    % --- Duration distribution ---
    ax = nexttile(t13);
    histogram(ax, summary.duration_s, 'FaceColor', [0.9 0.5 0.2], ...
              'EdgeColor','w', 'FaceAlpha', 0.85);
    xlabel(ax, 'Duration (s)'); ylabel(ax, 'Count');
    title(ax, 'Mission Duration Distribution');
    grid(ax,'on');
    xline(ax, mean(summary.duration_s), 'r--', ...
          sprintf('\\mu=%.1f', mean(summary.duration_s)), 'LineWidth', 1.5);

    % --- Takeoff success pie chart ---
    ax = nexttile(t13);
    if iscell(summary.takeoff_success) || isstring(summary.takeoff_success)
        n_ok = sum(strcmpi(string(summary.takeoff_success), 'True'));
    else
        n_ok = sum(summary.takeoff_success == 1);
    end
    n_fail = height(summary) - n_ok;
    if n_fail > 0
        pie(ax, [n_ok n_fail], {sprintf('Success (%d)', n_ok), ...
                                sprintf('Fail (%d)', n_fail)});
    else
        pie(ax, [n_ok 0.001], {sprintf('All Success (%d)', n_ok), ''});
    end
    title(ax, 'Takeoff Success Rate');
    colormap(ax, [0.3 0.8 0.4; 0.9 0.3 0.3]);

    % --- Exit reason bar chart ---
    ax = nexttile(t13);
    if ismember('exit_reason', summary.Properties.VariableNames)
        reasons = categorical(summary.exit_reason);
        cats = categories(reasons);
        counts = countcats(reasons);
        barh(ax, categorical(cats), counts, 'FaceColor', [0.5 0.4 0.8]);
        xlabel(ax, 'Count'); title(ax, 'Exit Reasons');
        grid(ax,'on');
    end

    % --- Mass vs max altitude scatter ---
    if ismember('p_mass', summary.Properties.VariableNames)
        ax = nexttile(t13);
        scatter(ax, summary.p_mass, summary.max_alt_m, 30, ...
                summary.duration_s, 'filled', 'MarkerFaceAlpha', 0.7);
        xlabel(ax, 'Mass (kg)'); ylabel(ax, 'Max Altitude (m)');
        title(ax, 'Mass vs Max Alt (color = duration)');
        colorbar(ax); grid(ax,'on');
    end

    % --- Ixx vs max altitude scatter ---
    if ismember('p_Ixx', summary.Properties.VariableNames)
        ax = nexttile(t13);
        scatter(ax, summary.p_Ixx, summary.max_alt_m, 30, ...
                [0.2 0.7 0.5], 'filled', 'MarkerFaceAlpha', 0.7);
        xlabel(ax, 'I_{xx} (kg m^2)'); ylabel(ax, 'Max Altitude (m)');
        title(ax, 'I_{xx} vs Max Altitude');
        grid(ax,'on');
    end

    if save_figures, savefig_custom(fig13, results_root, 'fig13_summary_stats', fig_format, fig_dpi); end
end

%% =========================================================================
%  FIGURE 14 — Envelope: Altitude vs Airspeed
%  =========================================================================
fig14 = figure('Name','Alt vs TAS Envelope', ...
               'Units','normalized','OuterPosition',[0 0 0.6 0.7]);
ax14 = axes(fig14);
hold(ax14,'on'); grid(ax14,'on'); box(ax14,'on');
title(ax14, 'Flight Envelope — Altitude vs TAS', ...
      'FontSize', 14, 'FontWeight','bold');

for k = 1:num_cases
    T = data{k};
    plot(ax14, T.TAS_mps, T.alt_agl_m, ...
         'Color', [cmap(k,:) 0.45], 'LineWidth', 0.8);
end
xlabel(ax14, 'TAS (m/s)'); ylabel(ax14, 'Altitude AGL (m)');
if num_cases <= 15
    legend(ax14, labels, 'Location','bestoutside', 'FontSize', 8);
end

if save_figures, savefig_custom(fig14, results_root, 'fig14_envelope', fig_format, fig_dpi); end

%% =========================================================================
%  FIGURE 15 — Parameter Correlation Matrix  (summary)
%  =========================================================================
if has_summary && height(summary) >= 5
    param_cols = summary.Properties.VariableNames( ...
                     startsWith(summary.Properties.VariableNames, 'p_'));
    if numel(param_cols) >= 2
        fig15 = figure('Name','Parameter Correlation', ...
                       'Units','normalized','OuterPosition',[0 0 0.7 0.85]);
        ax15 = axes(fig15);

        P = summary{:, param_cols};
        R = corrcoef(P);
        imagesc(ax15, R);
        colormap(ax15, parula);
        colorbar(ax15);
        caxis(ax15, [-1 1]);
        ax15.XTick = 1:numel(param_cols);
        ax15.YTick = 1:numel(param_cols);
        short_names = strrep(param_cols, 'p_', '');
        ax15.XTickLabel = short_names;
        ax15.YTickLabel = short_names;
        ax15.XTickLabelRotation = 45;
        title(ax15, 'Parameter Correlation Matrix', ...
              'FontSize', 14, 'FontWeight','bold');
        axis(ax15, 'tight');

        if save_figures, savefig_custom(fig15, results_root, 'fig15_correlation', fig_format, fig_dpi); end
    end
end

%% =========================================================================
fprintf('\n=== Visualization Complete ===\n');
fprintf('Figures saved to: %s\n', results_root);
