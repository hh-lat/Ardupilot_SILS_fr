%% =========================================================================
%  Takeoff Performance — Distribution Analysis (1000 cases)
%  Lightweight: no time-series plots, only histograms
%  =========================================================================
clc; close all; clear;

%% === CONFIG =============================================================
results_root = '\\wsl.localhost\Ubuntu\home\lat_avionics\Ardupilot_SITL_LATEST\Ardupilot_SILS\Tools\autotest\monte_carlo\mc_results_20260216_071033';
save_fig     = true;
liftoff_thresh = 0.11;   % metres — same as mc_analysis.m

%% === DARK THEME =========================================================
bg  = [0.08 0.08 0.08];
fg  = [1 1 1];
gc  = [0.3 0.3 0.3];

%% === LOAD & COMPUTE =====================================================
case_dirs = dir(fullfile(results_root, 'case_*'));
case_dirs = case_dirs([case_dirs.isdir]);
Ncases    = numel(case_dirs);

to_dist  = NaN(Ncases, 1);
to_speed = NaN(Ncases, 1);

fprintf('Processing %d cases ...\n', Ncases);
for k = 1:Ncases
    f = dir(fullfile(results_root, case_dirs(k).name, 'sim_output*.csv'));
    if isempty(f), continue; end
    T = readtable(fullfile(results_root, case_dirs(k).name, f(1).name));
    if height(T) < 5, continue; end

    t   = T.Time_s;
    alt = T.alt_agl_m;
    u   = T.V_ned_gnd_0;
    v   = T.V_ned_gnd_1;
    w   = T.V_ned_gnd_0;
    Vmag = sqrt(u.^2 + v.^2 + w.^2);

    idx = find(alt > liftoff_thresh, 1, 'first');
    if isempty(idx) || idx < 2, continue; end

    to_dist(k)  = trapz(t(1:idx), Vmag(1:idx));
    to_speed(k) = T.TAS_mps(idx);
end

valid = ~isnan(to_dist);
Nvalid = sum(valid);
fprintf('Liftoff detected in %d / %d cases.\n', Nvalid, Ncases);
if Nvalid < 2, error('Not enough valid cases for distribution analysis.'); end

d = to_dist(valid);
v = to_speed(valid);

%% === FIGURE — TWO DISTRIBUTION PLOTS ====================================
fig = figure('Name','Takeoff Distributions','Units','normalized',...
             'OuterPosition',[0.05 0.05 0.9 0.9],'Color',bg);
tl  = tiledlayout(fig, 1, 2, 'TileSpacing','compact','Padding','compact');
title(tl, sprintf('Takeoff Performance Distribution  (N = %d)', Nvalid),...
      'FontSize',18,'FontWeight','bold','Color',fg);

% ---- colour palette ----
hist_col_d = [0.1  0.70 0.90];   % cyan  — distance
hist_col_v = [0.95 0.45 0.10];   % orange — speed
line_mu    = [1 0.15 0.15];      % red
line_sig   = [1 0.6  0.2];       % amber

nBins = max(15, round(sqrt(Nvalid)));   % reasonable bin count

% ===================== LEFT: Takeoff Distance ============================
ax1 = nexttile(tl);
hold(ax1,'on'); grid(ax1,'on');

histogram(ax1, d, nBins, ...
    'FaceColor', hist_col_d, 'EdgeColor','none', 'FaceAlpha',0.80);

mu_d  = mean(d);
sig_d = std(d);
med_d = median(d);

xline(ax1, mu_d, '-', sprintf('  \\mu = %.1f m', mu_d), ...
      'Color',line_mu,'LineWidth',2,'FontSize',12,...
      'LabelOrientation','horizontal','LabelVerticalAlignment','bottom');
xline(ax1, mu_d + sig_d, '--', sprintf('  +1\\sigma = %.1f m', mu_d+sig_d), ...
      'Color',line_sig,'LineWidth',1.5,'FontSize',10,...
      'LabelOrientation','horizontal','LabelVerticalAlignment','top');
xline(ax1, max(0, mu_d - sig_d), '--', sprintf('  -1\\sigma = %.1f m', mu_d-sig_d), ...
      'Color',line_sig,'LineWidth',1.5,'FontSize',10,...
      'LabelOrientation','horizontal','LabelVerticalAlignment','top');
xline(ax1, med_d, ':', sprintf('  med = %.1f m', med_d), ...
      'Color',[0.4 1 0.4],'LineWidth',1.5,'FontSize',10,...
      'LabelOrientation','horizontal','LabelVerticalAlignment','bottom');

xlabel(ax1, 'Takeoff Ground Roll (m)', 'FontSize',13,'Color',fg,'FontWeight','bold');
ylabel(ax1, 'Count', 'FontSize',13,'Color',fg,'FontWeight','bold');
title(ax1, sprintf('Takeoff Distance\n\\mu=%.1f m   \\sigma=%.1f m   med=%.1f m   [%.1f – %.1f] m', ...
      mu_d, sig_d, med_d, min(d), max(d)), ...
      'FontSize',13,'FontWeight','bold','Color',fg);
set(ax1,'FontSize',11,'Color',bg,'XColor',fg,'YColor',fg,...
        'GridColor',gc,'GridAlpha',0.5);

% ===================== RIGHT: Liftoff Speed ==============================
ax2 = nexttile(tl);
hold(ax2,'on'); grid(ax2,'on');

histogram(ax2, v, nBins, ...
    'FaceColor', hist_col_v, 'EdgeColor','none', 'FaceAlpha',0.80);

mu_v  = mean(v);
sig_v = std(v);
med_v = median(v);

xline(ax2, mu_v, '-', sprintf('  \\mu = %.1f m/s', mu_v), ...
      'Color',line_mu,'LineWidth',2,'FontSize',12,...
      'LabelOrientation','horizontal','LabelVerticalAlignment','bottom');
xline(ax2, mu_v + sig_v, '--', sprintf('  +1\\sigma = %.1f m/s', mu_v+sig_v), ...
      'Color',line_sig,'LineWidth',1.5,'FontSize',10,...
      'LabelOrientation','horizontal','LabelVerticalAlignment','top');
xline(ax2, max(0, mu_v - sig_v), '--', sprintf('  -1\\sigma = %.1f m/s', mu_v-sig_v), ...
      'Color',line_sig,'LineWidth',1.5,'FontSize',10,...
      'LabelOrientation','horizontal','LabelVerticalAlignment','top');
xline(ax2, med_v, ':', sprintf('  med = %.1f m/s', med_v), ...
      'Color',[0.4 1 0.4],'LineWidth',1.5,'FontSize',10,...
      'LabelOrientation','horizontal','LabelVerticalAlignment','bottom');

xlabel(ax2, 'Liftoff TAS (m/s)', 'FontSize',13,'Color',fg,'FontWeight','bold');
ylabel(ax2, 'Count', 'FontSize',13,'Color',fg,'FontWeight','bold');
title(ax2, sprintf('Liftoff Speed\n\\mu=%.1f m/s   \\sigma=%.1f m/s   med=%.1f m/s   [%.1f – %.1f] m/s', ...
      mu_v, sig_v, med_v, min(v), max(v)), ...
      'FontSize',13,'FontWeight','bold','Color',fg);
set(ax2,'FontSize',11,'Color',bg,'XColor',fg,'YColor',fg,...
        'GridColor',gc,'GridAlpha',0.5);

%% === SAVE ===============================================================
if save_fig
    exportgraphics(fig, fullfile(results_root,'takeoff_distributions.png'),...
                  'Resolution',300,'BackgroundColor',bg);
    fprintf('Saved: takeoff_distributions.png\n');
end

%% === PRINT SUMMARY ======================================================
fprintf('\n=== TAKEOFF SUMMARY (N=%d) ===\n', Nvalid);
fprintf('Distance : mu=%.1f m,  sigma=%.1f m,  med=%.1f m,  range=[%.1f, %.1f] m\n', ...
        mu_d, sig_d, med_d, min(d), max(d));
fprintf('Speed    : mu=%.1f m/s, sigma=%.1f m/s, med=%.1f m/s, range=[%.1f, %.1f] m/s\n', ...
        mu_v, sig_v, med_v, min(v), max(v));
fprintf('Done.\n');
