%% MC_POSTPROCESS  Ultra-fast Monte Carlo post-processing for LAT SITL
%
%   Outputs (written into results folder):
%     mc_performance_report_matlab.csv  - per-case metrics
%     mc_flagged_cases_matlab.csv       - failed cases only
%     mc_summary_matlab.txt             - human-readable console report
%
%   Author: LAT Avionics  |.
% 000  Date: 2026-02-17

clc; clear;
t_wall = tic;

%% ══════════════════════════════════════════════════════════════════════
%  PASTE YOUR RESULTS PATH HERE
%  ══════════════════════════════════════════════════════════════════════
results_path = '\\wsl.localhost\Ubuntu\home\lag\SITL_Workspace\Ardupilot_SILS\Tools\autotest\monte_carlo\mc_results_20260219_152436';

%% ══════════════════════════════════════════════════════════════════════
%  PASS/FAIL THRESHOLDS (edit these to match your requirements)
%  ══════════════════════════════════════════════════════════════════════
LIM.liftoff_alt_m       = 0.11;     % alt_agl to detect liftoff
LIM.commanded_alt_m     = 100.0;    % target altitude AGL (m)
LIM.alt_ss_error_tol_m  = 10.0;     % ±10 m acceptable SS error
LIM.ss_start_frac       = 0.60;     % last 40% of flight = steady state

LIM.max_roll_rate_dps   = 150.0;     % max |p| allowed (deg/s)
LIM.max_pitch_rate_dps  = 150.0;     % max |q| allowed (deg/s)

LIM.roll_limit_deg      = 35.0;     % ROLL_LIMIT_DEG
LIM.pitch_limit_max_deg = 25.0;     % PTCH_LIM_MAX_DEG
LIM.pitch_limit_min_deg = -25.0;    % PTCH_LIM_MIN_DEG

LIM.commanded_radius_m  = 200.0;    % CIRCLE_RADIUS / WP_LOITER_RAD
LIM.radius_tol_pct      = 20.0;     % ±20% tolerance

LIM.stall_aoa_deg       = 30.0;     % AoA stall threshold (deg)

LIM.airspeed_min_mps    = 24.0;     % AIRSPEED_MIN
LIM.airspeed_max_mps    = 40.0;     % AIRSPEED_MAX
LIM.airspeed_cruise_mps = 30.0;     % AIRSPEED_CRUISE

fprintf('══════════════════════════════════════════════════════════════════════════════\n');
fprintf('  LAT Monte Carlo — Ultra-Fast MATLAB Post-Processing\n');
fprintf('══════════════════════════════════════════════════════════════════════════════\n');
fprintf('  Results: %s\n', results_path);

case_listing = dir(fullfile(results_path, 'case_*'));
case_listing = case_listing([case_listing.isdir]);
N = numel(case_listing);
if N == 0
    error('No case_XXXX directories found in %s', results_path);
end
case_dirs = sort({case_listing.name});
fprintf('  Cases found: %d\n', N);

%% ══════════════════════════════════════════════════════════════════════
%  COLUMN INDEX MAPS (built once, reused per case — avoids repeated header parsing)
%  ══════════════════════════════════════════════════════════════════════
%  sim_output columns (0-indexed in comment, 1-indexed in MATLAB)
SIM_COLS = struct( ...
    'Time_s',1, 'TAS_mps',3, 'alt_agl_m',4, ...
    'phi',10, 'theta',11, 'psi',12, ...
    'p',13, 'q',14, 'r',15, ...
    'V_b_tas_0',16, 'V_b_tas_1',17, 'V_b_tas_2',18, ...
    'V_ned_gnd_0',19, 'V_ned_gnd_1',20, 'V_ned_gnd_2',21, ...
    'pos_ned_0',22, 'pos_ned_1',23, 'pos_ned_2',24, ...
    'Accel_b_2',30, 'ArduPlane_Mode',31, ...
    'delta_e',32, 'delta_a',37, 'delta_r',35, ...
    'mot0_thr_cmd',44);

%  state_log columns
SL_COLS = struct('local_x_m',16, 'local_y_m',17);

%% ══════════════════════════════════════════════════════════════════════
%  PRE-ALLOCATE RESULTS TABLE (struct of arrays — cache-friendly)
%  ══════════════════════════════════════════════════════════════════════
nan_vec  = nan(N,1);
zero_vec = zeros(N,1);
false_vec = false(N,1);

R = struct();
R.case_id              = (0:N-1)';

% 1) Takeoff
R.takeoff_distance_m   = nan_vec;
R.takeoff_speed_mps    = nan_vec;
R.takeoff_time_s       = nan_vec;

% 2) AoA at liftoff
R.aoa_at_liftoff_deg   = nan_vec;

% 3) Altitude SS error
R.alt_ss_mean_m        = nan_vec;
R.alt_ss_std_m         = nan_vec;
R.alt_ss_error_m       = nan_vec;

% 4) Max rates & attitudes
R.max_roll_rate_dps    = nan_vec;
R.max_pitch_rate_dps   = nan_vec;
R.max_yaw_rate_dps     = nan_vec;
R.max_roll_deg         = nan_vec;
R.max_roll_ss_deg      = nan_vec;
R.max_pitch_deg        = nan_vec;
R.min_pitch_deg        = nan_vec;

% 5) Circle/Loiter radius
R.circle_radius_mean_m = nan_vec;
R.circle_radius_std_m  = nan_vec;
R.circle_radius_err_pct= nan_vec;

% 6) AoA variation
R.aoa_max_deg          = nan_vec;
R.aoa_min_deg          = nan_vec;
R.aoa_ss_mean_deg      = nan_vec;
R.aoa_ss_std_deg       = nan_vec;
R.stall_event_count    = zero_vec;

% 7) Actuator frequencies
R.freq_elevator_hz     = nan_vec;
R.freq_aileron_hz      = nan_vec;
R.freq_rudder_hz       = nan_vec;
R.freq_throttle_hz     = nan_vec;

% Extra
R.max_load_factor_g    = nan_vec;
R.airspeed_ss_mean_mps = nan_vec;
R.airspeed_ss_std_mps  = nan_vec;
R.max_airspeed_mps     = nan_vec;
R.min_airspeed_mps     = nan_vec;
R.flight_duration_s    = nan_vec;
R.max_climb_rate_mps   = nan_vec;
R.max_sink_rate_mps    = nan_vec;

% Flags
R.flag_no_liftoff      = false_vec;
R.flag_alt_error       = false_vec;
R.flag_roll_rate       = false_vec;
R.flag_pitch_rate      = false_vec;
R.flag_roll_angle      = false_vec;
R.flag_pitch_angle     = false_vec;
R.flag_circle_radius   = false_vec;
R.flag_stall_aoa       = false_vec;
R.flag_airspeed_low    = false_vec;
R.flag_airspeed_high   = false_vec;

%% ══════════════════════════════════════════════════════════════════════
%  MAIN LOOP — serial (scripts cannot parfor with local functions)
%  ══════════════════════════════════════════════════════════════════════
fprintf('  Processing %d cases...\n', N);
t_proc = tic;

lim = LIM;
sc  = SIM_COLS;
slc = SL_COLS;

res_cell = cell(N, 1);
for ii = 1:N
    res_cell{ii} = analyze_one_case(results_path, case_dirs{ii}, ii, lim, sc, slc);
    if mod(ii, max(1, floor(N/10))) == 0
        fprintf('    [%d/%d]  (%.1fs)\n', ii, N, toc(t_proc));
    end
end

% Unpack cell array into struct of arrays
fnames = fieldnames(res_cell{1});
for f = 1:numel(fnames)
    fn = fnames{f};
    if islogical(res_cell{1}.(fn))
        R.(fn) = false(N,1);
    elseif isnumeric(res_cell{1}.(fn))
        R.(fn) = nan(N,1);
    end
end
for ii = 1:N
    rc = res_cell{ii};
    for f = 1:numel(fnames)
        fn = fnames{f};
        R.(fn)(ii) = rc.(fn);
    end
end

elapsed_proc = toc(t_proc);
fprintf('  Analysis done in %.2fs  (%.1f ms/case)\n', elapsed_proc, elapsed_proc/N*1000);

%% ══════════════════════════════════════════════════════════════════════
%  PASS/FAIL DETERMINATION
%  ══════════════════════════════════════════════════════════════════════
R.passed = ~(R.flag_no_liftoff | R.flag_alt_error | R.flag_roll_rate | ...
             R.flag_pitch_rate | R.flag_roll_angle | ...
             R.flag_pitch_angle | ...
             R.flag_circle_radius | R.flag_stall_aoa | ...
             R.flag_airspeed_low | R.flag_airspeed_high);

%% ══════════════════════════════════════════════════════════════════════
%  WRITE CSV REPORTS
%  ══════════════════════════════════════════════════════════════════════
csv_path  = fullfile(results_path, 'mc_performance_report_matlab.csv');
flag_path = fullfile(results_path, 'mc_flagged_cases_matlab.csv');
sum_path  = fullfile(results_path, 'mc_summary_matlab.txt');

write_full_csv(csv_path, R, N);
write_flagged_csv(flag_path, R, N, lim);

%% ══════════════════════════════════════════════════════════════════════
%  CONSOLE SUMMARY + SAVE TO FILE
%  ══════════════════════════════════════════════════════════════════════
report = generate_summary(R, N, lim, results_path);
fprintf('%s', report);

fid = fopen(sum_path, 'w');
fprintf(fid, '%s', report);
fclose(fid);

elapsed_total = toc(t_wall);
fprintf('\n  Total wall-clock time : %.2fs\n', elapsed_total);
fprintf('  Report CSV            : %s\n', csv_path);
fprintf('  Flagged CSV           : %s\n', flag_path);
fprintf('  Summary text          : %s\n', sum_path);
fprintf('══════════════════════════════════════════════════════════════════════════════\n');

%% ══════════════════════════════════════════════════════════════════════
%  DISTRIBUTION PLOTS — saved as PNGs into results folder
%  ══════════════════════════════════════════════════════════════════════
fprintf('\n  Generating distribution plots...\n');
valid = ~R.flag_no_liftoff;
fig_dir = fullfile(results_path, 'figures');
if ~isfolder(fig_dir), mkdir(fig_dir); end

% ── Style defaults ──
set(0, 'DefaultAxesFontSize', 11);
set(0, 'DefaultAxesFontName', 'Segoe UI');

% ── Helper: histogram with mean/std lines ──
% (defined as local function at bottom: plot_hist_ax)

% ─────────────────────────────────────────────────────────────────────
%  FIG A — Altitude & Circle Radius Distributions
% ─────────────────────────────────────────────────────────────────────
figA = figure('Name','Altitude & Radius', 'Units','normalized', ...
              'OuterPosition',[0 0 1 1], 'Visible','off');
tA = tiledlayout(figA, 2, 3, 'TileSpacing','compact','Padding','compact');
title(tA, 'Altitude & Circle Radius Distributions (1000 cases)', ...
      'FontSize',15,'FontWeight','bold');

v = R.alt_ss_mean_m(valid);
plot_hist_ax(nexttile(tA), v, 'Alt SS Mean (m)', [0.2 0.6 0.9], lim.commanded_alt_m);
v = R.alt_ss_error_m(valid);
plot_hist_ax(nexttile(tA), v, 'Alt SS Error (m)', [0.9 0.4 0.3], 0);
xline(gca, lim.alt_ss_error_tol_m, 'r--','LineWidth',1.2);
xline(gca, -lim.alt_ss_error_tol_m, 'r--','LineWidth',1.2);
v = R.alt_ss_std_m(valid);
plot_hist_ax(nexttile(tA), v, 'Alt SS Std (m)', [0.3 0.7 0.5], NaN);
v = R.circle_radius_mean_m(valid);
plot_hist_ax(nexttile(tA), v, 'Circle Radius Mean (m)', [0.5 0.4 0.8], lim.commanded_radius_m);
v = R.circle_radius_err_pct(valid);
plot_hist_ax(nexttile(tA), v, 'Circle Radius Error (%)', [0.8 0.5 0.2], 0);
xline(gca, lim.radius_tol_pct, 'r--','LineWidth',1.2);
xline(gca, -lim.radius_tol_pct, 'r--','LineWidth',1.2);
v = R.circle_radius_std_m(valid);
plot_hist_ax(nexttile(tA), v, 'Circle Radius Std (m)', [0.4 0.6 0.7], NaN);

exportgraphics(figA, fullfile(fig_dir, 'dist_altitude_radius.png'), 'Resolution', 200);
fprintf('    Saved dist_altitude_radius.png\n');

% ─────────────────────────────────────────────────────────────────────
%  FIG B — Rates & Attitudes
% ─────────────────────────────────────────────────────────────────────
figB = figure('Name','Rates & Attitudes', 'Units','normalized', ...
              'OuterPosition',[0 0 1 1], 'Visible','off');
tB = tiledlayout(figB, 2, 3, 'TileSpacing','compact','Padding','compact');
title(tB, 'Body Rates & Attitude Distributions', ...
      'FontSize',15,'FontWeight','bold');

plot_hist_ax(nexttile(tB), R.max_roll_rate_dps(valid), 'Max |p| (dps)', [0.2 0.6 0.9], lim.max_roll_rate_dps);
plot_hist_ax(nexttile(tB), R.max_pitch_rate_dps(valid), 'Max |q| (dps)', [0.9 0.5 0.2], lim.max_pitch_rate_dps);
plot_hist_ax(nexttile(tB), R.max_yaw_rate_dps(valid), 'Max |r| (dps)', [0.3 0.7 0.5], NaN);
plot_hist_ax(nexttile(tB), R.max_roll_ss_deg(valid), 'Max |phi| SS (deg)', [0.2 0.6 0.9], lim.roll_limit_deg);
plot_hist_ax(nexttile(tB), R.max_pitch_deg(valid), 'Max theta (deg)', [0.9 0.4 0.3], lim.pitch_limit_max_deg);
plot_hist_ax(nexttile(tB), R.min_pitch_deg(valid), 'Min theta (deg)', [0.5 0.4 0.8], lim.pitch_limit_min_deg);

exportgraphics(figB, fullfile(fig_dir, 'dist_rates_attitudes.png'), 'Resolution', 200);
fprintf('    Saved dist_rates_attitudes.png\n');

% ─────────────────────────────────────────────────────────────────────
%  FIG C — Airspeed, AoA, Load Factor
% ─────────────────────────────────────────────────────────────────────
figC = figure('Name','Airspeed & AoA', 'Units','normalized', ...
              'OuterPosition',[0 0 1 1], 'Visible','off');
tC = tiledlayout(figC, 2, 3, 'TileSpacing','compact','Padding','compact');
title(tC, 'Airspeed, AoA & Load Factor Distributions', ...
      'FontSize',15,'FontWeight','bold');

plot_hist_ax(nexttile(tC), R.airspeed_ss_mean_mps(valid), 'Airspeed SS Mean (m/s)', [0.2 0.6 0.9], lim.airspeed_cruise_mps);
plot_hist_ax(nexttile(tC), R.max_airspeed_mps(valid), 'Max Airspeed SS (m/s)', [0.9 0.4 0.3], lim.airspeed_max_mps);
plot_hist_ax(nexttile(tC), R.min_airspeed_mps(valid), 'Min Airspeed SS (m/s)', [0.5 0.4 0.8], lim.airspeed_min_mps);
plot_hist_ax(nexttile(tC), R.aoa_max_deg(valid), 'AoA Max (deg)', [0.8 0.5 0.2], lim.stall_aoa_deg);
plot_hist_ax(nexttile(tC), R.aoa_ss_mean_deg(valid), 'AoA SS Mean (deg)', [0.3 0.7 0.5], NaN);
plot_hist_ax(nexttile(tC), R.max_load_factor_g(valid), 'Max Load Factor (g)', [0.4 0.6 0.7], NaN);

exportgraphics(figC, fullfile(fig_dir, 'dist_airspeed_aoa.png'), 'Resolution', 200);
fprintf('    Saved dist_airspeed_aoa.png\n');

% ─────────────────────────────────────────────────────────────────────
%  FIG D — Takeoff Performance
% ─────────────────────────────────────────────────────────────────────
figD = figure('Name','Takeoff Performance', 'Units','normalized', ...
              'OuterPosition',[0 0 1 1], 'Visible','off');
tD = tiledlayout(figD, 1, 3, 'TileSpacing','compact','Padding','compact');
title(tD, 'Takeoff Performance Distributions', ...
      'FontSize',15,'FontWeight','bold');

plot_hist_ax(nexttile(tD), R.takeoff_distance_m(valid), 'Takeoff Distance (m)', [0.2 0.6 0.9], NaN);
plot_hist_ax(nexttile(tD), R.takeoff_speed_mps(valid), 'Takeoff Speed (m/s)', [0.9 0.5 0.2], NaN);
plot_hist_ax(nexttile(tD), R.aoa_at_liftoff_deg(valid), 'AoA at Liftoff (deg)', [0.3 0.7 0.5], NaN);

exportgraphics(figD, fullfile(fig_dir, 'dist_takeoff.png'), 'Resolution', 200);
fprintf('    Saved dist_takeoff.png\n');

% ─────────────────────────────────────────────────────────────────────
%  FIG E — Actuator Frequencies
% ─────────────────────────────────────────────────────────────────────
figE = figure('Name','Actuator Frequencies', 'Units','normalized', ...
              'OuterPosition',[0 0 1 1], 'Visible','off');
tE = tiledlayout(figE, 2, 2, 'TileSpacing','compact','Padding','compact');
title(tE, 'Actuator Dominant Frequency Distributions (SS)', ...
      'FontSize',15,'FontWeight','bold');

plot_hist_ax(nexttile(tE), R.freq_elevator_hz(valid), 'Elevator (Hz)', [0.2 0.6 0.9], NaN);
plot_hist_ax(nexttile(tE), R.freq_aileron_hz(valid), 'Aileron (Hz)', [0.9 0.5 0.2], NaN);
plot_hist_ax(nexttile(tE), R.freq_rudder_hz(valid), 'Rudder (Hz)', [0.3 0.7 0.5], NaN);
plot_hist_ax(nexttile(tE), R.freq_throttle_hz(valid), 'Throttle (Hz)', [0.5 0.4 0.8], NaN);

exportgraphics(figE, fullfile(fig_dir, 'dist_actuator_freq.png'), 'Resolution', 200);
fprintf('    Saved dist_actuator_freq.png\n');

% ─────────────────────────────────────────────────────────────────────
%  FIG F — Flag Summary Bar Chart
% ─────────────────────────────────────────────────────────────────────
figF = figure('Name','Flag Summary', 'Units','pixels', ...
              'Position',[100 100 1400 900], 'Visible','off');
flag_labels_plot = {'ALT\_ERROR','ROLL\_RATE','PITCH\_RATE','ROLL\_ANG\_SS', ...
              'PITCH\_ANG','CIRCLE\_R','STALL\_AOA','ASPD\_LOW','ASPD\_HIGH'};
flag_vals = [sum(R.flag_alt_error), sum(R.flag_roll_rate), sum(R.flag_pitch_rate), ...
             sum(R.flag_roll_angle), sum(R.flag_pitch_angle), ...
             sum(R.flag_circle_radius), sum(R.flag_stall_aoa), ...
             sum(R.flag_airspeed_low), sum(R.flag_airspeed_high)];
barh(categorical(flag_labels_plot, flag_labels_plot), flag_vals, ...
     'FaceColor', [0.85 0.33 0.10], 'EdgeColor','w');
xlabel('Number of Cases Flagged'); ylabel('Criterion');
title(sprintf('Pass/Fail Flag Summary  (N=%d)', N), 'FontSize',14,'FontWeight','bold');
grid on; box on;
for kk = 1:numel(flag_vals)
    if flag_vals(kk) > 0
        text(flag_vals(kk)+0.3, kk, sprintf('%d', flag_vals(kk)), ...
             'FontSize',10,'FontWeight','bold');
    end
end

exportgraphics(figF, fullfile(fig_dir, 'flag_summary_bar.png'), 'Resolution', 200);
fprintf('    Saved flag_summary_bar.png\n');

% ─────────────────────────────────────────────────────────────────────
%  FIG G — Pass/Fail Pie Chart
% ─────────────────────────────────────────────────────────────────────
figG = figure('Name','Pass/Fail', 'Units','pixels', ...
              'Position',[100 100 1000 900], 'Visible','off');
n_pass = sum(R.passed);
n_fail = N - n_pass;
pie([n_pass, n_fail], {sprintf('PASS (%d)', n_pass), sprintf('FAIL (%d)', n_fail)});
colormap([0.3 0.8 0.4; 0.9 0.3 0.3]);
title(sprintf('Overall Pass/Fail  (N=%d)', N), 'FontSize',14,'FontWeight','bold');

exportgraphics(figG, fullfile(fig_dir, 'pass_fail_pie.png'), 'Resolution', 200);
fprintf('    Saved pass_fail_pie.png\n');

close all;
fprintf('  All distribution plots saved to: %s\n', fig_dir);


%% ════════════════════════════════════════════════════════════════════════
%  SINGLE CASE ANALYSIS (called from parfor)
%  ════════════════════════════════════════════════════════════════════════
function S = analyze_one_case(results_path, case_name, idx, lim, sc, slc)

S = init_case_struct(idx - 1);  % 0-based case_id
case_dir = fullfile(results_path, case_name);

% ── Find sim_output CSV ──
sim_files = dir(fullfile(case_dir, 'sim_output_*.csv'));
if isempty(sim_files)
    S.flag_no_liftoff = true;
    return
end

% ── Load sim_output (readmatrix is fastest for numeric CSV) ──
try
    D = readmatrix(fullfile(case_dir, sim_files(1).name));
catch
    S.flag_no_liftoff = true;
    return
end

[Nr, Nc] = size(D);
if Nr < 50 || Nc < 40
    S.flag_no_liftoff = true;
    return
end

% ── Extract columns by index ──
t   = D(:, sc.Time_s);
alt = D(:, sc.alt_agl_m);
tas = D(:, sc.TAS_mps);
phi = D(:, sc.phi);         % rad
th  = D(:, sc.theta);       % rad
p   = D(:, sc.p);           % rad/s
q   = D(:, sc.q);           % rad/s
r   = D(:, sc.r);           % rad/s
vn  = D(:, sc.V_ned_gnd_0);
ve  = D(:, sc.V_ned_gnd_1);
vd  = D(:, sc.V_ned_gnd_2);
vb0 = D(:, sc.V_b_tas_0);
vb2 = D(:, sc.V_b_tas_2);
az  = D(:, sc.Accel_b_2);

de_col = min(sc.delta_e, Nc);
da_col = min(sc.delta_a, Nc);
dr_col = min(sc.delta_r, Nc);
thr_col = min(sc.mot0_thr_cmd, Nc);

de  = D(:, de_col);
da  = D(:, da_col);
dr  = D(:, dr_col);
thr = D(:, thr_col);

S.flight_duration_s = t(end) - t(1);

% ═══════════════════════════════════════════════════════════════════════
%  1) TAKEOFF DISTANCE & SPEED
% ═══════════════════════════════════════════════════════════════════════
lo_idx = find(alt > lim.liftoff_alt_m, 1, 'first');
if isempty(lo_idx) || lo_idx < 3
    S.flag_no_liftoff = true;
    return
end

Vmag = sqrt(vn(1:lo_idx).^2 + ve(1:lo_idx).^2 + vd(1:lo_idx).^2);
S.takeoff_distance_m = trapz(t(1:lo_idx), Vmag);
S.takeoff_speed_mps  = tas(lo_idx);
S.takeoff_time_s     = t(lo_idx);

% ═══════════════════════════════════════════════════════════════════════
%  2) AOA AT LIFTOFF
% ═══════════════════════════════════════════════════════════════════════
S.aoa_at_liftoff_deg = rad2deg(th(lo_idx));

% ═══════════════════════════════════════════════════════════════════════
%  3) ALTITUDE STEADY-STATE ERROR
% ═══════════════════════════════════════════════════════════════════════
ss_idx = max(lo_idx + 1, round(Nr * lim.ss_start_frac));
alt_ss = alt(ss_idx:end);
alt_ss = alt_ss(isfinite(alt_ss));
if numel(alt_ss) > 10
    S.alt_ss_mean_m  = mean(alt_ss);
    S.alt_ss_std_m   = std(alt_ss);
    S.alt_ss_error_m = mean(alt_ss) - lim.commanded_alt_m;
    if abs(S.alt_ss_error_m) > lim.alt_ss_error_tol_m
        S.flag_alt_error = true;
    end
end

% ═══════════════════════════════════════════════════════════════════════
%  4) MAX RATES & ATTITUDES (after liftoff)
% ═══════════════════════════════════════════════════════════════════════
fly = lo_idx:Nr;

p_dps = abs(rad2deg(p(fly)));
q_dps = abs(rad2deg(q(fly)));
r_dps = abs(rad2deg(r(fly)));
phi_d = rad2deg(phi(fly));
th_d  = rad2deg(th(fly));

S.max_roll_rate_dps  = max(p_dps);
S.max_pitch_rate_dps = max(q_dps);
S.max_yaw_rate_dps   = max(r_dps);
S.max_roll_deg       = max(abs(phi_d));
S.max_pitch_deg      = max(th_d);
S.min_pitch_deg      = min(th_d);

if S.max_roll_rate_dps  > lim.max_roll_rate_dps;  S.flag_roll_rate  = true; end
if S.max_pitch_rate_dps > lim.max_pitch_rate_dps;  S.flag_pitch_rate = true; end
if S.max_pitch_deg > lim.pitch_limit_max_deg || S.min_pitch_deg < lim.pitch_limit_min_deg
    S.flag_pitch_angle = true;
end

% Steady-state roll angle (last 40% of flight) — flag if exceeds limit
phi_ss_deg = abs(rad2deg(phi(ss_idx:end)));
phi_ss_deg = phi_ss_deg(isfinite(phi_ss_deg));
if ~isempty(phi_ss_deg)
    S.max_roll_ss_deg = max(phi_ss_deg);
    if S.max_roll_ss_deg > lim.roll_limit_deg
        S.flag_roll_angle = true;
    end
end

% ═══════════════════════════════════════════════════════════════════════
%  5) CIRCLE / LOITER RADIUS (from state_log local_x_m, local_y_m)
% ═══════════════════════════════════════════════════════════════════════
sl_files = dir(fullfile(case_dir, '*state_log*.csv'));
if ~isempty(sl_files)
    try
        SL = readmatrix(fullfile(case_dir, sl_files(1).name));
        if size(SL,1) > 50 && size(SL,2) >= max(slc.local_x_m, slc.local_y_m)
            lx = SL(:, slc.local_x_m);
            ly = SL(:, slc.local_y_m);
            sl_ss = max(1, round(size(SL,1) * lim.ss_start_frac));
            [r_mean, r_std] = kasa_circle_fit(lx(sl_ss:end), ly(sl_ss:end));
            S.circle_radius_mean_m  = r_mean;
            S.circle_radius_std_m   = r_std;
            if isfinite(r_mean)
                S.circle_radius_err_pct = (r_mean - lim.commanded_radius_m) / lim.commanded_radius_m * 100;
                if abs(S.circle_radius_err_pct) > lim.radius_tol_pct
                    S.flag_circle_radius = true;
                end
            end
        end
    catch
        % silently skip if state_log parse fails
    end
end

% ═══════════════════════════════════════════════════════════════════════
%  6) ANGLE OF ATTACK
% ═══════════════════════════════════════════════════════════════════════
if all(isfinite(vb0)) && all(isfinite(vb2))
    aoa_deg = rad2deg(atan2(vb2, vb0));
    aoa_fly = aoa_deg(fly);
    aoa_fly = aoa_fly(isfinite(aoa_fly));
    if ~isempty(aoa_fly)
        S.aoa_max_deg = max(aoa_fly);
        S.aoa_min_deg = min(aoa_fly);
    end
    aoa_ss = aoa_deg(ss_idx:end);
    aoa_ss = aoa_ss(isfinite(aoa_ss));
    if numel(aoa_ss) > 10
        S.aoa_ss_mean_deg = mean(aoa_ss);
        S.aoa_ss_std_deg  = std(aoa_ss);
    end
    stall_mask = aoa_fly > lim.stall_aoa_deg;
    if any(stall_mask)
        S.flag_stall_aoa = true;
        d_stall = diff([0; stall_mask(:); 0]);
        S.stall_event_count = sum(d_stall == 1);
    end
else
    % Fallback: theta ≈ AoA
    aoa_deg = rad2deg(th);
    aoa_fly = aoa_deg(fly);
    S.aoa_max_deg = max(aoa_fly);
    S.aoa_min_deg = min(aoa_fly);
    aoa_ss = aoa_deg(ss_idx:end);
    if numel(aoa_ss) > 10
        S.aoa_ss_mean_deg = mean(aoa_ss);
        S.aoa_ss_std_deg  = std(aoa_ss);
    end
    if any(aoa_fly > lim.stall_aoa_deg)
        S.flag_stall_aoa = true;
    end
end

% ═══════════════════════════════════════════════════════════════════════
%  7) ACTUATOR DOMINANT FREQUENCIES (steady-state region, FFT)
% ═══════════════════════════════════════════════════════════════════════
t_ss = t(ss_idx:end);
S.freq_elevator_hz = dominant_freq(t_ss, de(ss_idx:end));
S.freq_aileron_hz  = dominant_freq(t_ss, da(ss_idx:end));
S.freq_rudder_hz   = dominant_freq(t_ss, dr(ss_idx:end));
S.freq_throttle_hz = dominant_freq(t_ss, thr(ss_idx:end));

% ═══════════════════════════════════════════════════════════════════════
%  EXTRA METRICS
% ═══════════════════════════════════════════════════════════════════════
% Load factor
az_fly = az(fly);
az_fly = az_fly(isfinite(az_fly));
if ~isempty(az_fly)
    S.max_load_factor_g = max(abs(az_fly)) / 9.81;
end

% Airspeed SS
tas_ss = tas(ss_idx:end);
tas_ss = tas_ss(isfinite(tas_ss));
if numel(tas_ss) > 10
    S.airspeed_ss_mean_mps = mean(tas_ss);
    S.airspeed_ss_std_mps  = std(tas_ss);
end

% Max/min airspeed in steady-state (loiter)
% Apply 1-s moving average (~10 samples at 10 Hz) to filter
% single-sample simulation glitches before checking limits.
tas_ss_chk = tas(ss_idx:end);
tas_ss_chk = tas_ss_chk(isfinite(tas_ss_chk) & tas_ss_chk > 0.5);
if numel(tas_ss_chk) > 10
    tas_ss_filt = movmean(tas_ss_chk, 10);
    S.max_airspeed_mps = max(tas_ss_filt);
    S.min_airspeed_mps = min(tas_ss_filt);
    if S.min_airspeed_mps < lim.airspeed_min_mps
        S.flag_airspeed_low = true;
    end
    if S.max_airspeed_mps > lim.airspeed_max_mps
        S.flag_airspeed_high = true;
    end
end

% Climb / Sink rate
vd_fly = vd(fly);
vd_fly = vd_fly(isfinite(vd_fly));
if ~isempty(vd_fly)
    S.max_climb_rate_mps = max(-vd_fly);   % NED: -vd = climb
    S.max_sink_rate_mps  = max(vd_fly);
end

end % analyze_one_case


%% ════════════════════════════════════════════════════════════════════════
%  HELPER: Kasa circle fit (algebraic least-squares)
%  ════════════════════════════════════════════════════════════════════════
function [R_mean, R_std] = kasa_circle_fit(x, y)
    good = isfinite(x) & isfinite(y);
    if sum(good) < 20
        R_mean = NaN; R_std = NaN;
        return
    end
    x = x(good); y = y(good);
    n = numel(x);
    A = [x, y, ones(n,1)];
    b = x.^2 + y.^2;
    c = A \ b;          % fast backslash solve
    cx = c(1)/2;  cy = c(2)/2;
    R_sq = c(3) + cx^2 + cy^2;
    if R_sq <= 0
        R_mean = NaN; R_std = NaN;
        return
    end
    radii = sqrt((x - cx).^2 + (y - cy).^2);
    R_mean = mean(radii);
    R_std  = std(radii);
end


%% ════════════════════════════════════════════════════════════════════════
%  HELPER: Dominant frequency via FFT
%  ════════════════════════════════════════════════════════════════════════
function f_dom = dominant_freq(t, sig)
    good = isfinite(sig) & isfinite(t);
    if sum(good) < 64
        f_dom = NaN;
        return
    end
    sig = sig(good);
    t   = t(good);
    dt  = median(diff(t));
    if dt <= 0 || ~isfinite(dt)
        f_dom = NaN;
        return
    end
    fs  = 1 / dt;
    sig = sig - mean(sig);   % de-mean
    n   = numel(sig);
    nfft = 2^nextpow2(n);   % zero-pad to power of 2 for speed
    Y = abs(fft(sig, nfft));
    Y = Y(1:nfft/2+1);      % single-sided
    freqs = (0:nfft/2) * fs / nfft;
    % Ignore DC and < 0.05 Hz
    mask = freqs > 0.05;
    if ~any(mask)
        f_dom = NaN;
        return
    end
    Y_mask = Y(mask);
    f_mask = freqs(mask);
    [~, idx] = max(Y_mask);
    f_dom = f_mask(idx);
end


%% ════════════════════════════════════════════════════════════════════════
%  HELPER: Initialize empty case struct
%  ════════════════════════════════════════════════════════════════════════
function S = init_case_struct(case_id)
    S.case_id              = case_id;
    S.takeoff_distance_m   = NaN;
    S.takeoff_speed_mps    = NaN;
    S.takeoff_time_s       = NaN;
    S.aoa_at_liftoff_deg   = NaN;
    S.alt_ss_mean_m        = NaN;
    S.alt_ss_std_m         = NaN;
    S.alt_ss_error_m       = NaN;
    S.max_roll_rate_dps    = NaN;
    S.max_pitch_rate_dps   = NaN;
    S.max_yaw_rate_dps     = NaN;
    S.max_roll_deg         = NaN;
    S.max_roll_ss_deg      = NaN;
    S.max_pitch_deg        = NaN;
    S.min_pitch_deg        = NaN;
    S.circle_radius_mean_m = NaN;
    S.circle_radius_std_m  = NaN;
    S.circle_radius_err_pct= NaN;
    S.aoa_max_deg          = NaN;
    S.aoa_min_deg          = NaN;
    S.aoa_ss_mean_deg      = NaN;
    S.aoa_ss_std_deg       = NaN;
    S.stall_event_count    = 0;
    S.freq_elevator_hz     = NaN;
    S.freq_aileron_hz      = NaN;
    S.freq_rudder_hz       = NaN;
    S.freq_throttle_hz     = NaN;
    S.max_load_factor_g    = NaN;
    S.airspeed_ss_mean_mps = NaN;
    S.airspeed_ss_std_mps  = NaN;
    S.max_airspeed_mps     = NaN;
    S.min_airspeed_mps     = NaN;
    S.flight_duration_s    = NaN;
    S.max_climb_rate_mps   = NaN;
    S.max_sink_rate_mps    = NaN;
    S.flag_no_liftoff      = false;
    S.flag_alt_error       = false;
    S.flag_roll_rate       = false;
    S.flag_pitch_rate      = false;
    S.flag_roll_angle      = false;
    S.flag_pitch_angle     = false;
    S.flag_circle_radius   = false;
    S.flag_stall_aoa       = false;
    S.flag_airspeed_low    = false;
    S.flag_airspeed_high   = false;
end


%% ════════════════════════════════════════════════════════════════════════
%  OUTPUT: Full performance CSV
%  ════════════════════════════════════════════════════════════════════════
function write_full_csv(filepath, R, N)
    fid = fopen(filepath, 'w');
    % Header
    fprintf(fid, ['case_id,takeoff_distance_m,takeoff_speed_mps,takeoff_time_s,' ...
        'aoa_at_liftoff_deg,alt_ss_mean_m,alt_ss_std_m,alt_ss_error_m,' ...
        'max_roll_rate_dps,max_pitch_rate_dps,max_yaw_rate_dps,' ...
        'max_roll_deg,max_roll_ss_deg,max_pitch_deg,min_pitch_deg,' ...
        'circle_radius_mean_m,circle_radius_std_m,circle_radius_err_pct,' ...
        'aoa_max_deg,aoa_min_deg,aoa_ss_mean_deg,aoa_ss_std_deg,stall_event_count,' ...
        'freq_elevator_hz,freq_aileron_hz,freq_rudder_hz,freq_throttle_hz,' ...
        'max_load_factor_g,airspeed_ss_mean_mps,airspeed_ss_std_mps,' ...
        'max_airspeed_mps,min_airspeed_mps,flight_duration_s,' ...
        'max_climb_rate_mps,max_sink_rate_mps,' ...
        'flag_no_liftoff,flag_alt_error,flag_roll_rate,flag_pitch_rate,' ...
        'flag_roll_angle,flag_pitch_angle,flag_circle_radius,flag_stall_aoa,' ...
        'flag_airspeed_low,flag_airspeed_high,passed,fail_reasons\n']);
    for ii = 1:N
        reasons = build_fail_reasons(R, ii);
        fprintf(fid, ['%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,' ...
            '%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,' ...
            '%.4f,%.4f,%.4f,' ...
            '%.4f,%.4f,%.4f,%.4f,%d,' ...
            '%.4f,%.4f,%.4f,%.4f,' ...
            '%.4f,%.4f,%.4f,' ...
            '%.4f,%.4f,%.4f,' ...
            '%.4f,%.4f,' ...
            '%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s\n'], ...
            R.case_id(ii), R.takeoff_distance_m(ii), R.takeoff_speed_mps(ii), R.takeoff_time_s(ii), ...
            R.aoa_at_liftoff_deg(ii), R.alt_ss_mean_m(ii), R.alt_ss_std_m(ii), R.alt_ss_error_m(ii), ...
            R.max_roll_rate_dps(ii), R.max_pitch_rate_dps(ii), R.max_yaw_rate_dps(ii), ...
            R.max_roll_deg(ii), R.max_roll_ss_deg(ii), R.max_pitch_deg(ii), R.min_pitch_deg(ii), ...
            R.circle_radius_mean_m(ii), R.circle_radius_std_m(ii), R.circle_radius_err_pct(ii), ...
            R.aoa_max_deg(ii), R.aoa_min_deg(ii), R.aoa_ss_mean_deg(ii), R.aoa_ss_std_deg(ii), R.stall_event_count(ii), ...
            R.freq_elevator_hz(ii), R.freq_aileron_hz(ii), R.freq_rudder_hz(ii), R.freq_throttle_hz(ii), ...
            R.max_load_factor_g(ii), R.airspeed_ss_mean_mps(ii), R.airspeed_ss_std_mps(ii), ...
            R.max_airspeed_mps(ii), R.min_airspeed_mps(ii), R.flight_duration_s(ii), ...
            R.max_climb_rate_mps(ii), R.max_sink_rate_mps(ii), ...
            R.flag_no_liftoff(ii), R.flag_alt_error(ii), R.flag_roll_rate(ii), R.flag_pitch_rate(ii), ...
            R.flag_roll_angle(ii), R.flag_pitch_angle(ii), R.flag_circle_radius(ii), R.flag_stall_aoa(ii), ...
            R.flag_airspeed_low(ii), R.flag_airspeed_high(ii), R.passed(ii), reasons);
    end
    fclose(fid);
end


%% ════════════════════════════════════════════════════════════════════════
%  OUTPUT: Flagged cases CSV
%  ════════════════════════════════════════════════════════════════════════
function write_flagged_csv(filepath, R, N, ~)
    fid = fopen(filepath, 'w');
    fprintf(fid, ['case_id,fail_reasons,takeoff_distance_m,takeoff_speed_mps,' ...
        'alt_ss_error_m,max_roll_rate_dps,max_pitch_rate_dps,' ...
        'max_roll_deg,max_roll_ss_deg,max_pitch_deg,min_pitch_deg,' ...
        'circle_radius_mean_m,circle_radius_err_pct,' ...
        'aoa_max_deg,stall_event_count,' ...
        'max_airspeed_mps,min_airspeed_mps\n']);
    for ii = 1:N
        if ~R.passed(ii)
            reasons = build_fail_reasons(R, ii);
            fprintf(fid, '%d,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.4f,%.4f\n', ...
                R.case_id(ii), reasons, ...
                R.takeoff_distance_m(ii), R.takeoff_speed_mps(ii), ...
                R.alt_ss_error_m(ii), R.max_roll_rate_dps(ii), R.max_pitch_rate_dps(ii), ...
                R.max_roll_deg(ii), R.max_roll_ss_deg(ii), R.max_pitch_deg(ii), R.min_pitch_deg(ii), ...
                R.circle_radius_mean_m(ii), R.circle_radius_err_pct(ii), ...
                R.aoa_max_deg(ii), R.stall_event_count(ii), ...
                R.max_airspeed_mps(ii), R.min_airspeed_mps(ii));
        end
    end
    fclose(fid);
end


%% ════════════════════════════════════════════════════════════════════════
%  HELPER: Build fail reason string
%  ════════════════════════════════════════════════════════════════════════
function s = build_fail_reasons(R, ii)
    parts = {};
    if R.flag_no_liftoff(ii);  parts{end+1} = 'NO_LIFTOFF'; end
    if R.flag_alt_error(ii);   parts{end+1} = sprintf('ALT_ERR(%+.1fm)', R.alt_ss_error_m(ii)); end
    if R.flag_roll_rate(ii);   parts{end+1} = sprintf('ROLL_RATE(%.1fdps)', R.max_roll_rate_dps(ii)); end
    if R.flag_pitch_rate(ii);  parts{end+1} = sprintf('PITCH_RATE(%.1fdps)', R.max_pitch_rate_dps(ii)); end
    if R.flag_roll_angle(ii);  parts{end+1} = sprintf('ROLL_ANG_SS(%.1fdeg)', R.max_roll_ss_deg(ii)); end
    if R.flag_pitch_angle(ii); parts{end+1} = sprintf('PITCH_ANG(+%.1f/%.1f)', R.max_pitch_deg(ii), R.min_pitch_deg(ii)); end
    if R.flag_circle_radius(ii); parts{end+1} = sprintf('CIRCLE_R(%.0fm)', R.circle_radius_mean_m(ii)); end
    if R.flag_stall_aoa(ii);   parts{end+1} = sprintf('STALL(AoA=%.1f)', R.aoa_max_deg(ii)); end
    if R.flag_airspeed_low(ii);  parts{end+1} = sprintf('ASPD_LOW(%.1f)', R.min_airspeed_mps(ii)); end
    if R.flag_airspeed_high(ii); parts{end+1} = sprintf('ASPD_HIGH(%.1f)', R.max_airspeed_mps(ii)); end
    if isempty(parts)
        s = 'PASS';
    else
        s = strjoin(parts, '; ');
    end
end


%% ════════════════════════════════════════════════════════════════════════
%  SUMMARY REPORT GENERATOR
%  ════════════════════════════════════════════════════════════════════════
function report = generate_summary(R, N, lim, results_path)

    valid_mask = ~R.flag_no_liftoff;
    Nv = sum(valid_mask);
    Np = sum(R.passed);
    Nf = N - Np;

    sep  = repmat(char(9552), 1, 78);   % ═
    line = repmat(char(9472), 1, 78);   % ─

    c = {};
    c{end+1} = '';
    c{end+1} = sep;
    c{end+1} = '  LAT MONTE CARLO — POST-PROCESSING REPORT (MATLAB)';
    c{end+1} = sep;
    c{end+1} = sprintf('  Results path  : %s', results_path);
    c{end+1} = sprintf('  Total cases   : %d', N);
    c{end+1} = sprintf('  Valid liftoff : %d/%d  (%.1f%%)', Nv, N, 100*Nv/max(N,1));
    c{end+1} = sprintf('  PASSED        : %d/%d  (%.1f%%)', Np, N, 100*Np/max(N,1));
    c{end+1} = sprintf('  FLAGGED/FAIL  : %d/%d  (%.1f%%)', Nf, N, 100*Nf/max(N,1));
    c{end+1} = sep;

    % Criteria
    c{end+1} = '';
    c{end+1} = '  PASS/FAIL CRITERIA USED:';
    c{end+1} = line;
    c{end+1} = sprintf('  Commanded altitude      : %.0f m AGL', lim.commanded_alt_m);
    c{end+1} = sprintf('  Alt SS error tolerance  : +/-%.0f m', lim.alt_ss_error_tol_m);
    c{end+1} = sprintf('  Max roll rate           : %.0f deg/s', lim.max_roll_rate_dps);
    c{end+1} = sprintf('  Max pitch rate          : %.0f deg/s', lim.max_pitch_rate_dps);
    c{end+1} = sprintf('  Roll angle limit        : +/-%.0f deg', lim.roll_limit_deg);
    c{end+1} = sprintf('  Pitch limit             : %.0f to +%.0f deg', lim.pitch_limit_min_deg, lim.pitch_limit_max_deg);
    c{end+1} = sprintf('  Circle radius command   : %.0f m  (+/-%.0f%%)', lim.commanded_radius_m, lim.radius_tol_pct);
    c{end+1} = sprintf('  Stall AoA threshold     : %.0f deg', lim.stall_aoa_deg);
    c{end+1} = sprintf('  Airspeed band (SS)      : [%.1f, %.1f] m/s', lim.airspeed_min_mps, lim.airspeed_max_mps);
    c{end+1} = line;

    % Helper for stats line
    stat = @(v) stat_str(v(valid_mask));

    % 1) Takeoff
    c{end+1} = '';
    c{end+1} = '  1) TAKEOFF PERFORMANCE';
    c{end+1} = line;
    c{end+1} = sprintf('  Distance (m)  :%s', stat(R.takeoff_distance_m));
    c{end+1} = sprintf('  Speed (m/s)   :%s', stat(R.takeoff_speed_mps));
    c{end+1} = sprintf('  Time (s)      :%s', stat(R.takeoff_time_s));

    % 2) AoA at liftoff
    c{end+1} = '';
    c{end+1} = '  2) ANGLE OF ATTACK AT LIFTOFF';
    c{end+1} = line;
    c{end+1} = sprintf('  AoA (deg)     :%s', stat(R.aoa_at_liftoff_deg));

    % 3) Altitude error
    c{end+1} = '';
    c{end+1} = '  3) ALTITUDE STEADY-STATE ERROR';
    c{end+1} = line;
    c{end+1} = sprintf('  Alt SS mean (m):%s', stat(R.alt_ss_mean_m));
    c{end+1} = sprintf('  Alt SS err (m) :%s', stat(R.alt_ss_error_m));
    c{end+1} = sprintf('  Alt SS std (m) :%s', stat(R.alt_ss_std_m));
    c{end+1} = sprintf('  Cases FLAGGED  : %d/%d', sum(R.flag_alt_error(valid_mask)), Nv);

    % 4) Rates & Attitudes
    c{end+1} = '';
    c{end+1} = '  4) MAX RATES & ATTITUDES (in-flight)';
    c{end+1} = line;
    c{end+1} = sprintf('  Max |p| (dps) :%s', stat(R.max_roll_rate_dps));
    c{end+1} = sprintf('  Max |q| (dps) :%s', stat(R.max_pitch_rate_dps));
    c{end+1} = sprintf('  Max |r| (dps) :%s', stat(R.max_yaw_rate_dps));
    c{end+1} = sprintf('  Max |phi| (deg):%s', stat(R.max_roll_deg));
    c{end+1} = sprintf('  Max theta (deg):%s', stat(R.max_pitch_deg));
    c{end+1} = sprintf('  Min theta (deg):%s', stat(R.min_pitch_deg));
    c{end+1} = sprintf('  Roll rate FLAGGED  : %d/%d', sum(R.flag_roll_rate(valid_mask)), Nv);
    c{end+1} = sprintf('  Pitch rate FLAGGED : %d/%d', sum(R.flag_pitch_rate(valid_mask)), Nv);
    c{end+1} = sprintf('  Max |phi| SS (deg) :%s', stat(R.max_roll_ss_deg));
    c{end+1} = sprintf('  Roll angle (SS) FLAGGED : %d/%d', sum(R.flag_roll_angle(valid_mask)), Nv);
    c{end+1} = sprintf('  Pitch angle FLAGGED: %d/%d', sum(R.flag_pitch_angle(valid_mask)), Nv);

    % 5) Circle radius
    c{end+1} = '';
    c{end+1} = '  5) CIRCLE / LOITER RADIUS TRACKING';
    c{end+1} = line;
    c{end+1} = sprintf('  Radius mean (m)  :%s', stat(R.circle_radius_mean_m));
    c{end+1} = sprintf('  Radius std  (m)  :%s', stat(R.circle_radius_std_m));
    c{end+1} = sprintf('  Radius err  (%%)  :%s', stat(R.circle_radius_err_pct));
    c{end+1} = sprintf('  Cases FLAGGED    : %d/%d', sum(R.flag_circle_radius(valid_mask)), Nv);

    % 6) AoA variation
    c{end+1} = '';
    c{end+1} = '  6) ANGLE OF ATTACK VARIATION (in-flight)';
    c{end+1} = line;
    c{end+1} = sprintf('  AoA max  (deg) :%s', stat(R.aoa_max_deg));
    c{end+1} = sprintf('  AoA min  (deg) :%s', stat(R.aoa_min_deg));
    c{end+1} = sprintf('  AoA SS mu (deg):%s', stat(R.aoa_ss_mean_deg));
    c{end+1} = sprintf('  AoA SS sd (deg):%s', stat(R.aoa_ss_std_deg));
    n_st = sum(R.flag_stall_aoa(valid_mask));
    total_stalls = sum(R.stall_event_count(valid_mask));
    c{end+1} = sprintf('  Cases with stall (>%.0fdeg): %d/%d  (%d total events)', lim.stall_aoa_deg, n_st, Nv, total_stalls);

    % 7) Actuator frequencies
    c{end+1} = '';
    c{end+1} = '  7) ACTUATOR DOMINANT FREQUENCY (steady-state)';
    c{end+1} = line;
    c{end+1} = sprintf('  Elevator (Hz) :%s', stat(R.freq_elevator_hz));
    c{end+1} = sprintf('  Aileron  (Hz) :%s', stat(R.freq_aileron_hz));
    c{end+1} = sprintf('  Rudder   (Hz) :%s', stat(R.freq_rudder_hz));
    c{end+1} = sprintf('  Throttle (Hz) :%s', stat(R.freq_throttle_hz));

    % Extra
    c{end+1} = '';
    c{end+1} = '  EXTRA METRICS';
    c{end+1} = line;
    c{end+1} = sprintf('  Airspeed SS mu (m/s) :%s', stat(R.airspeed_ss_mean_mps));
    c{end+1} = sprintf('  Max airspeed  (m/s)  :%s', stat(R.max_airspeed_mps));
    c{end+1} = sprintf('  Min airspeed  (m/s)  :%s', stat(R.min_airspeed_mps));
    c{end+1} = sprintf('  Max load factor (g)  :%s', stat(R.max_load_factor_g));
    c{end+1} = sprintf('  Max climb rate (m/s) :%s', stat(R.max_climb_rate_mps));
    c{end+1} = sprintf('  Max sink rate  (m/s) :%s', stat(R.max_sink_rate_mps));
    c{end+1} = sprintf('  Airspeed LOW  FLAGGED : %d/%d', sum(R.flag_airspeed_low(valid_mask)), Nv);
    c{end+1} = sprintf('  Airspeed HIGH FLAGGED : %d/%d', sum(R.flag_airspeed_high(valid_mask)), Nv);

    % Flagged cases detail
    c{end+1} = '';
    c{end+1} = sep;
    c{end+1} = '  FLAGGED CASES DETAIL';
    c{end+1} = sep;
    fail_idx = find(~R.passed);
    if isempty(fail_idx)
        c{end+1} = '  ALL CASES PASSED';
    else
        for jj = 1:numel(fail_idx)
            ii = fail_idx(jj);
            c{end+1} = sprintf('  Case %04d: %s', R.case_id(ii), build_fail_reasons(R, ii));
        end
    end
    c{end+1} = sep;

    % Flag summary bar chart
    c{end+1} = '';
    c{end+1} = '  FLAG SUMMARY';
    c{end+1} = line;
    flag_names = {'NO_LIFTOFF','ALT_ERROR','ROLL_RATE','PITCH_RATE','ROLL_ANG_SS', ...
                  'PITCH_ANGLE','CIRCLE_RADIUS','STALL_AOA','AIRSPEED_LOW','AIRSPEED_HIGH'};
    flag_counts = [sum(R.flag_no_liftoff), sum(R.flag_alt_error), sum(R.flag_roll_rate), ...
                   sum(R.flag_pitch_rate), sum(R.flag_roll_angle), ...
                   sum(R.flag_pitch_angle), ...
                   sum(R.flag_circle_radius), sum(R.flag_stall_aoa), ...
                   sum(R.flag_airspeed_low), sum(R.flag_airspeed_high)];
    for jj = 1:numel(flag_names)
        bar_len = min(flag_counts(jj), 50);
        bar_str = [repmat('#', 1, bar_len), sprintf('  %d/%d', flag_counts(jj), N)];
        c{end+1} = sprintf('  %-16s %s', flag_names{jj}, bar_str);
    end
    c{end+1} = sep;

    % ── Per-criterion case lists (for further analysis) ──
    c{end+1} = '';
    c{end+1} = sep;
    c{end+1} = '  CASES FAILING EACH CRITERION (case IDs for further analysis)';
    c{end+1} = sep;
    flag_fields = {'flag_no_liftoff','flag_alt_error','flag_roll_rate','flag_pitch_rate', ...
                   'flag_roll_angle','flag_pitch_angle','flag_circle_radius', ...
                   'flag_stall_aoa','flag_airspeed_low','flag_airspeed_high'};
    flag_labels = {'NO_LIFTOFF','ALT_ERROR','ROLL_RATE','PITCH_RATE','ROLL_ANG_SS', ...
                   'PITCH_ANGLE','CIRCLE_RADIUS','STALL_AOA','AIRSPEED_LOW','AIRSPEED_HIGH'};
    for jj = 1:numel(flag_fields)
        ids = R.case_id(R.(flag_fields{jj}));
        if isempty(ids)
            c{end+1} = sprintf('  %-16s  (none)', flag_labels{jj});
        else
            id_strs = arrayfun(@(x) sprintf('%04d', x), ids, 'UniformOutput', false);
            % Print in rows of 20 for readability
            for kk = 1:20:numel(id_strs)
                chunk = strjoin(id_strs(kk:min(kk+19, end)), ', ');
                if kk == 1
                    c{end+1} = sprintf('  %-16s  [%s', flag_labels{jj}, chunk);
                else
                    c{end+1} = sprintf('  %-16s   %s', '', chunk);
                end
            end
            c{end} = [c{end}, sprintf(']  (%d cases)', numel(ids))];
        end
    end
    c{end+1} = sep;

    report = strjoin(c, '\n');
    report = [report, '\n'];
end


%% ════════════════════════════════════════════════════════════════════════
%  HELPER: Statistics string
%  ════════════════════════════════════════════════════════════════════════
function s = stat_str(v)
    a = v(isfinite(v));
    if isempty(a)
        s = '  N/A';
    else
        s = sprintf('  mu=%.2f  sd=%.2f  med=%.2f  [%.2f, %.2f]', ...
            mean(a), std(a), median(a), min(a), max(a));
    end
end


%% ════════════════════════════════════════════════════════════════════════
%  HELPER: Histogram subplot for distribution plots
%  ════════════════════════════════════════════════════════════════════════
function plot_hist_ax(ax, data, x_label, face_clr, limit_val)
%PLOT_HIST_AX  Draw a histogram with mean/std annotations and optional
%              threshold reference line on the given axes handle.
%
%  ax        – target axes (e.g. from nexttile)
%  data      – numeric vector (NaNs are removed internally)
%  x_label   – string for the x-axis label
%  face_clr  – 1×3 RGB colour for the histogram bars
%  limit_val – scalar threshold (drawn as red dashed line); pass NaN / []
%              to skip

    d = data(isfinite(data));
    if isempty(d); title(ax, x_label); return; end

    histogram(ax, d, 'FaceColor', face_clr, 'FaceAlpha', 0.85, ...
              'EdgeColor', 'w', 'LineWidth', 0.4);
    hold(ax, 'on');

    mu = mean(d);  sd = std(d);

    % Mean line
    xline(ax, mu, '-r', sprintf('\\mu=%.1f', mu), ...
        'LineWidth', 1.6, 'LabelOrientation', 'aligned', ...
        'LabelHorizontalAlignment', 'left', 'FontSize', 7);

    % ±1σ lines
    xline(ax, mu - sd, ':b', sprintf('-1\\sigma'), ...
        'LineWidth', 1.0, 'LabelOrientation', 'aligned', 'FontSize', 7);
    xline(ax, mu + sd, ':b', sprintf('+1\\sigma'), ...
        'LineWidth', 1.0, 'LabelOrientation', 'aligned', 'FontSize', 7);

    % Optional limit reference
    if nargin >= 5 && ~isempty(limit_val) && isfinite(limit_val)
        xline(ax, limit_val, '--r', 'Limit', ...
            'LineWidth', 1.4, 'LabelOrientation', 'aligned', ...
            'LabelHorizontalAlignment', 'right', 'FontSize', 7);
    end

    hold(ax, 'off');
    xlabel(ax, x_label, 'FontSize', 8);
    ylabel(ax, 'Count', 'FontSize', 8);
    grid(ax, 'on'); box(ax, 'on');
end
