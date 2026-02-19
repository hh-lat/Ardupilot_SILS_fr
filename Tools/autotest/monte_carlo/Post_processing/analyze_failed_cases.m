%% ANALYZE_FAILED_CASES  Perturbation Analysis of Failed MC Cases
% =========================================================================
%  Identifies the failed cases (passed = 0) and analyses the perturbation
%  magnitude (% from nominal) for the Top-10 most important parameters.
%
%  Pass/fail is read directly from the mc_performance_report_matlab.csv
%  "passed" column, which is computed in mc_postprocess.m from 10 binary
%  flags (stall AoA threshold = 30 deg).
%    FAILED  = passed == 0  (any flag triggered)
%    PASSED  = passed == 1  (no flags triggered)
%
%  Analysis outputs:
%    failed_case_perturbations.csv   — per-case perturbation % for top-10
%    failure_perturbation_summary.csv — statistical comparison (failed vs passed)
%    fig_failed_vs_passed_boxplot.png — side-by-side boxplots
%    fig_failed_perturbation_heatmap.png — heatmap of failed-case perturbations
%    failure_analysis_report.txt     — console-style summary
%
%  Author : LAT Avionics
%  Date   : 2026-02-19
% =========================================================================

clc; clear; close all;
t_wall = tic;

%% ═══════════════════════════════════════════════════════════════════════
%  0.  CONFIGURATION
%  ═══════════════════════════════════════════════════════════════════════
results_path = '\\wsl.localhost\Ubuntu\home\lat_avionics\Ardupilot_SITL_LATEST\Ardupilot_SILS\Tools\autotest\monte_carlo\mc_results_20260217_074641';

out_dir = fullfile(results_path, 'failure_analysis');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

% --- Top-10 parameters (from Sensitivity.m composite ranking) -----------
top10_names = {'CL_alpha', 'Cm_delta_e', 'Cm_alpha', 'mass', 'cg_x', ...
               'CL_delta_e', 'CL_0', 'Cl_delta_aR', 'Ixx', 'Cl_delta_aL'};
top10_scores = [1.0, 0.633, 0.523, 0.440, 0.293, ...
                0.223, 0.207, 0.186, 0.164, 0.140];

% --- Nominal values (from monte_carlo_config.json) ----------------------
nominals = struct( ...
    'CL_alpha',     5.718000, ...
    'Cm_delta_e',  -3.303274, ...
    'Cm_alpha',    -4.205166, ...
    'mass',        65.000000, ...
    'cg_x',         1.030000, ...
    'CL_delta_e',   0.690000, ...
    'CL_0',         0.180000, ...
    'Cl_delta_aR', -0.062853, ...
    'Ixx',         14.658000, ...
    'Cl_delta_aL',  0.062338);

% --- 3-sigma bounds (%) -------------------------------------------------
sigma3_pct = struct( ...
    'CL_alpha',    50.0, ...
    'Cm_delta_e',  50.0, ...
    'Cm_alpha',    50.0, ...
    'mass',        10.0, ...
    'cg_x',        10.0, ...
    'CL_delta_e',  50.0, ...
    'CL_0',        50.0, ...
    'Cl_delta_aR', 50.0, ...
    'Ixx',         20.0, ...
    'Cl_delta_aL', 50.0);

% --- Pretty labels for plots -------------------------------------------
pretty = struct( ...
    'CL_alpha',    '$C_{L_\alpha}$', ...
    'Cm_delta_e',  '$C_{m_{\delta_e}}$', ...
    'Cm_alpha',    '$C_{m_\alpha}$', ...
    'mass',        'Mass', ...
    'cg_x',        '$x_{cg}$', ...
    'CL_delta_e',  '$C_{L_{\delta_e}}$', ...
    'CL_0',        '$C_{L_0}$', ...
    'Cl_delta_aR', '$C_{\ell_{\delta_{a_R}}}$', ...
    'Ixx',         '$I_{xx}$', ...
    'Cl_delta_aL', '$C_{\ell_{\delta_{a_L}}}$');

% --- Units map ----------------------------------------------------------
units = struct( ...
    'CL_alpha',    '1/rad', ...
    'Cm_delta_e',  '1/rad', ...
    'Cm_alpha',    '1/rad', ...
    'mass',        'kg', ...
    'cg_x',        'm', ...
    'CL_delta_e',  '1/rad', ...
    'CL_0',        '–', ...
    'Cl_delta_aR', '1/rad', ...
    'Ixx',         'kg·m²', ...
    'Cl_delta_aL', '1/rad');

nTop = numel(top10_names);

%% ═══════════════════════════════════════════════════════════════════════
%  1.  DATA INGESTION
%  ═══════════════════════════════════════════════════════════════════════
fprintf('══════════════════════════════════════════════════════════════════\n');
fprintf('  Failed-Case Perturbation Analysis\n');
fprintf('══════════════════════════════════════════════════════════════════\n\n');

% --- 1a. summary.csv  (perturbed parameter values per case) -------------
fprintf('  Loading summary.csv  ...');
T_sum = readtable(fullfile(results_path, 'summary.csv'), ...
    'Delimiter', ',', 'TextType', 'string');
fprintf(' %d cases\n', height(T_sum));

% --- 1b. mc_performance_report_matlab.csv (flags + passed) --------------
fprintf('  Loading mc_performance_report_matlab.csv  ...');
T_met = readtable(fullfile(results_path, 'mc_performance_report_matlab.csv'), ...
    'Delimiter', ',', 'TextType', 'string');
fprintf(' %d cases\n', height(T_met));

assert(height(T_sum) == height(T_met), 'Row count mismatch');
assert(all(T_sum.case_id == T_met.case_id), 'case_id mismatch');
N = height(T_sum);

%% ═══════════════════════════════════════════════════════════════════════
%  2.  PASS / FAIL  (from mc_postprocess.m "passed" column)
%  ═══════════════════════════════════════════════════════════════════════
flag_cols = {'flag_no_liftoff', 'flag_alt_error', 'flag_roll_rate', ...
             'flag_pitch_rate', 'flag_roll_angle', 'flag_pitch_angle', ...
             'flag_circle_radius', 'flag_stall_aoa', ...
             'flag_airspeed_low', 'flag_airspeed_high'};
flag_labels = {'No Liftoff', 'Alt Error', 'Roll Rate', 'Pitch Rate', ...
               'Roll Angle', 'Pitch Angle', 'Circle Radius', ...
               'Stall AoA', 'Airspeed Low', 'Airspeed High'};
nFlags = numel(flag_cols);

% Build flag matrix  (N × nFlags)
F = zeros(N, nFlags);
for jj = 1:nFlags
    F(:, jj) = T_met.(flag_cols{jj});
end
total_flags = sum(F, 2);   % total flags triggered per case

% Use the passed column directly from mc_postprocess.m
passed = logical(T_met.passed);    % 1 = pass, 0 = fail
failed = ~passed;
n_fail = sum(failed);
n_pass = sum(passed);

fprintf('\n  Pass/fail from mc_postprocess.m (stall AoA threshold = 30 deg)\n');
fprintf('  ──────────────────────────────────────────────────\n');
fprintf('    PASSED:  %4d  (%.1f%%)\n', n_pass, 100*n_pass/N);
fprintf('    FAILED:  %4d  (%.1f%%)\n', n_fail, 100*n_fail/N);
fprintf('    Total:   %4d\n', N);

% --- Flag breakdown across failed cases ---
fprintf('\n  Flag breakdown (failed cases only):\n');
for jj = 1:nFlags
    fc = sum(F(failed, jj));
    if fc > 0
        triggered_ids = T_met.case_id(failed & F(:,jj)==1);
        id_str = strjoin(arrayfun(@(c) sprintf('%04d', c), triggered_ids, 'UniformOutput', false), ', ');
        fprintf('    %-16s  %2d cases  [%s]\n', flag_labels{jj}, fc, id_str);
    else
        fprintf('    %-16s  (none)\n', flag_labels{jj});
    end
end
fprintf('\n');

%% ═══════════════════════════════════════════════════════════════════════
%  3.  COMPUTE PERTURBATION PERCENTAGES FOR TOP-10 PARAMETERS
%  ═══════════════════════════════════════════════════════════════════════
% Perturbation matrix:  pct_delta(i,j) = (perturbed_ij - nominal_j) / |nominal_j| × 100
pct_delta = zeros(N, nTop);
nom_vec   = zeros(1, nTop);

for jj = 1:nTop
    col_name = ['p_' top10_names{jj}];
    vals     = T_sum.(col_name);
    nom      = nominals.(top10_names{jj});
    nom_vec(jj) = nom;
    pct_delta(:, jj) = (vals - nom) ./ abs(nom) * 100;
end

%% ═══════════════════════════════════════════════════════════════════════
%  4.  PER-CASE OUTPUT  — failed_case_perturbations.csv
%  ═══════════════════════════════════════════════════════════════════════
fprintf('  [1] Generating per-case perturbation table for %d failed cases ...\n', n_fail);

fail_idx    = find(failed);
fail_ids    = T_sum.case_id(fail_idx);
fail_reasons = T_met.fail_reasons(fail_idx);

% Build table
T_fail = table();
T_fail.case_id = fail_ids;
for jj = 1:nTop
    raw_col_name = ['p_' top10_names{jj}];
    T_fail.(['val_' top10_names{jj}]) = T_sum.(raw_col_name)(fail_idx);
    T_fail.(['pct_' top10_names{jj}]) = pct_delta(fail_idx, jj);
end
T_fail.total_flags  = total_flags(fail_idx);
T_fail.fail_reasons = fail_reasons;

% Sort by total_flags descending, then case_id ascending
[~, si] = sortrows([T_fail.total_flags, T_fail.case_id], [-1, 2]);
T_fail = T_fail(si, :);

csv_per_case = fullfile(out_dir, 'failed_case_perturbations.csv');
writetable(T_fail, csv_per_case);
fprintf('    Saved → %s\n', csv_per_case);

%% ═══════════════════════════════════════════════════════════════════════
%  5.  STATISTICAL COMPARISON  — failed vs passed
%  ═══════════════════════════════════════════════════════════════════════
fprintf('  [2] Statistical comparison (failed vs passed) ...\n');

% Pre-allocate summary table
summary_param       = cell(nTop, 1);
summary_nominal     = zeros(nTop, 1);
summary_unit        = cell(nTop, 1);
summary_composite   = zeros(nTop, 1);
summary_sigma3pct   = zeros(nTop, 1);
% Failed stats
summary_fail_mean   = zeros(nTop, 1);
summary_fail_std    = zeros(nTop, 1);
summary_fail_min    = zeros(nTop, 1);
summary_fail_max    = zeros(nTop, 1);
summary_fail_median = zeros(nTop, 1);
% Passed stats
summary_pass_mean   = zeros(nTop, 1);
summary_pass_std    = zeros(nTop, 1);
summary_pass_median = zeros(nTop, 1);
% Comparison
summary_mean_diff   = zeros(nTop, 1);  % fail_mean - pass_mean
summary_abs_ratio   = zeros(nTop, 1);  % |fail_mean| / |pass_mean|
summary_ranksum_p   = zeros(nTop, 1);  % Wilcoxon rank-sum p-value
summary_significant = cell(nTop, 1);

for jj = 1:nTop
    pf = pct_delta(failed, jj);   % perturbation % for failed
    pp = pct_delta(passed, jj);   % perturbation % for passed

    summary_param{jj}       = top10_names{jj};
    summary_nominal(jj)     = nom_vec(jj);
    summary_unit{jj}        = units.(top10_names{jj});
    summary_composite(jj)   = top10_scores(jj);
    summary_sigma3pct(jj)   = sigma3_pct.(top10_names{jj});

    summary_fail_mean(jj)   = mean(pf);
    summary_fail_std(jj)    = std(pf);
    summary_fail_min(jj)    = min(pf);
    summary_fail_max(jj)    = max(pf);
    summary_fail_median(jj) = median(pf);

    summary_pass_mean(jj)   = mean(pp);
    summary_pass_std(jj)    = std(pp);
    summary_pass_median(jj) = median(pp);

    summary_mean_diff(jj)   = summary_fail_mean(jj) - summary_pass_mean(jj);
    if abs(summary_pass_mean(jj)) > 1e-10
        summary_abs_ratio(jj) = abs(summary_fail_mean(jj)) / abs(summary_pass_mean(jj));
    else
        summary_abs_ratio(jj) = NaN;
    end

    % Wilcoxon rank-sum test (Mann-Whitney U)
    summary_ranksum_p(jj) = ranksum_test(pf, pp);
    if summary_ranksum_p(jj) < 0.01
        summary_significant{jj} = '***';
    elseif summary_ranksum_p(jj) < 0.05
        summary_significant{jj} = '**';
    elseif summary_ranksum_p(jj) < 0.10
        summary_significant{jj} = '*';
    else
        summary_significant{jj} = '';
    end
end

T_summary = table( ...
    summary_param, summary_nominal, summary_unit, summary_composite, summary_sigma3pct, ...
    summary_fail_mean, summary_fail_std, summary_fail_median, summary_fail_min, summary_fail_max, ...
    summary_pass_mean, summary_pass_std, summary_pass_median, ...
    summary_mean_diff, summary_abs_ratio, summary_ranksum_p, summary_significant, ...
    'VariableNames', {'Parameter', 'Nominal', 'Unit', 'Composite_Score', 'Sigma3_pct', ...
                      'Fail_Mean_pct', 'Fail_Std_pct', 'Fail_Median_pct', 'Fail_Min_pct', 'Fail_Max_pct', ...
                      'Pass_Mean_pct', 'Pass_Std_pct', 'Pass_Median_pct', ...
                      'MeanDiff_pct', 'AbsMeanRatio', 'Ranksum_pval', 'Significance'});

csv_summary = fullfile(out_dir, 'failure_perturbation_summary.csv');
writetable(T_summary, csv_summary);
fprintf('    Saved → %s\n', csv_summary);

%% ═══════════════════════════════════════════════════════════════════════
%  6.  WHICH FLAGS DID EACH FAILED CASE TRIGGER?
%  ═══════════════════════════════════════════════════════════════════════
fprintf('  [3] Flag breakdown for failed cases ...\n');

% Build flag breakdown table
T_flags = table();
T_flags.case_id = fail_ids;
for jj = 1:nFlags
    col = flag_cols{jj};
    T_flags.(col) = F(fail_idx, jj);
end
T_flags.total_flags  = total_flags(fail_idx);
T_flags.fail_reasons = fail_reasons;

% Reorder same as T_fail
T_flags = T_flags(si, :);

csv_flags = fullfile(out_dir, 'failed_case_flags.csv');
writetable(T_flags, csv_flags);
fprintf('    Saved → %s\n', csv_flags);

%% ═══════════════════════════════════════════════════════════════════════
%  7.  FIGURE 1:  Failed vs Passed Box-Whisker Plots
%  ═══════════════════════════════════════════════════════════════════════
fprintf('  [4] Generating boxplot figure ...\n');

fig_bg  = [0.94 0.94 0.96];   % cool grey background
ax_bg   = [1 1 1];
c_pass  = [0.20 0.55 0.78];   % deeper steel blue
c_fail  = [0.80 0.15 0.15];   % deeper crimson red
c_nom   = [0.05 0.05 0.05];   % near-black for nominal line
c_txt   = [0.05 0.05 0.05];   % near-black for all text/labels

fig1 = figure('Position', [50 50 1600 900], 'Color', fig_bg, ...
    'Name', 'Failed vs Passed Perturbations');
tiledlayout(2, 5, 'TileSpacing', 'compact', 'Padding', 'compact');

for jj = 1:nTop
    ax = nexttile;
    set(ax, 'Color', ax_bg);
    hold(ax, 'on');

    pp = pct_delta(passed, jj);
    pf = pct_delta(failed, jj);

    % Passed group (position 1)
    bp1 = boxplot_manual(ax, pp, 1, c_pass, 0.35);
    % Failed group (position 2)
    bp2 = boxplot_manual(ax, pf, 2, c_fail, 0.35);

    % Nominal line at 0%
    yline(ax, 0, ':', 'Color', c_nom, 'LineWidth', 1.2, 'Alpha', 0.7);

    % 3-sigma band
    s3 = sigma3_pct.(top10_names{jj});
    fill(ax, [0.4 2.6 2.6 0.4], [-s3 -s3 s3 s3], ...
        [0.5 0.5 0.5], 'FaceAlpha', 0.07, 'EdgeColor', 'none');

    % Overlay individual failed points (few enough to show)
    scatter(ax, 2*ones(size(pf)), pf, 30, c_fail, 'filled', ...
        'MarkerFaceAlpha', 0.7, 'MarkerEdgeColor', [0.5 0.1 0.1]);

    % Labels
    title(ax, pretty.(top10_names{jj}), 'Interpreter', 'latex', 'FontSize', 13, ...
        'Color', c_txt, 'FontWeight', 'bold');
    ylabel(ax, '\Delta from nominal (%)', 'FontSize', 10, 'Color', c_txt, 'FontWeight', 'bold');
    set(ax, 'XTick', [1 2], 'XTickLabel', {'Pass', 'Fail'}, 'FontSize', 10, ...
        'XColor', c_txt, 'YColor', c_txt, 'FontWeight', 'bold');
    xlim(ax, [0.4 2.6]);
    grid(ax, 'on');
    set(ax, 'GridAlpha', 0.15);

    % Annotation: p-value
    pval = summary_ranksum_p(jj);
    if pval < 0.001
        pstr = 'p < 0.001';
    else
        pstr = sprintf('p = %.3f', pval);
    end
    yl = ylim(ax);
    text(ax, 1.5, yl(2)*0.92, pstr, 'FontSize', 9, ...
        'HorizontalAlignment', 'center', 'Color', [0.0 0.0 0.0], ...
        'FontWeight', 'bold', 'BackgroundColor', [1 1 1 0.85], ...
        'EdgeColor', [0.3 0.3 0.3], 'Margin', 3);

    hold(ax, 'off');
end

sgtitle('Perturbation (% from Nominal) — Failed vs Passed Cases', ...
    'FontSize', 16, 'FontWeight', 'bold', 'Color', c_txt);
exportgraphics(fig1, fullfile(out_dir, 'fig_failed_vs_passed_boxplot.png'), ...
    'Resolution', 300, 'BackgroundColor', fig_bg);
fprintf('    Saved → fig_failed_vs_passed_boxplot.png\n');

%% ═══════════════════════════════════════════════════════════════════════
%  8.  FIGURE 2:  Heatmap of Failed-Case Perturbations
%  ═══════════════════════════════════════════════════════════════════════
fprintf('  [5] Generating perturbation heatmap for failed cases ...\n');

% Failed-case perturbation matrix (n_fail × nTop), same sort order as CSV
pct_fail_mat = pct_delta(fail_idx(si), :);

% Build Y-axis labels: "Case XXXX  [reason]"
case_labels = cell(n_fail, 1);
for ii = 1:n_fail
    reason_str = char(T_fail.fail_reasons(ii));
    % Truncate long reasons to keep labels readable
    if numel(reason_str) > 55
        reason_str = [reason_str(1:52) '...'];
    end
    case_labels{ii} = sprintf('Case %04d   %s', T_fail.case_id(ii), reason_str);
end

% Build X-axis labels: "Param (rank #N, score)"
x_labels = cell(nTop, 1);
for jj = 1:nTop
    x_labels{jj} = sprintf('#%d  %s', jj, top10_names{jj});
end

% --- Figure sizing: scale height with number of failed cases ---
row_h   = 38;   % pixels per row
fig_h   = max(550, 180 + n_fail * row_h);
fig_w   = 1700;

fig2 = figure('Position', [60 60 fig_w fig_h], 'Color', fig_bg, ...
    'Name', 'Failed Case Perturbation Heatmap');

% Leave generous margins for long Y-tick labels
ax2 = axes(fig2, 'Position', [0.30 0.12 0.58 0.78], 'Color', ax_bg);

imagesc(ax2, pct_fail_mat);
colormap(ax2, bluewhitered_dark(512));

% Symmetric colour limits — clamp to ±60% to avoid washing out moderate values
clim_val = min(max(abs(pct_fail_mat(:))), 60);
clim(ax2, [-clim_val clim_val]);

% --- Colourbar ---
cb = colorbar(ax2, 'Location', 'eastoutside');
cb.Label.String = 'Perturbation from Nominal (%)';
cb.Label.FontSize = 12;
cb.Label.FontWeight = 'bold';
cb.Label.Color = [0.05 0.05 0.05];
cb.FontSize = 10;
cb.TickDirection = 'out';
cb.Color = [0.05 0.05 0.05];

% --- Cell grid lines ---
hold(ax2, 'on');
for ii = 0.5 : 1 : (n_fail + 0.5)
    plot(ax2, [0.5 nTop+0.5], [ii ii], '-', 'Color', [0.5 0.5 0.5], ...
        'LineWidth', 0.4);
end
for jj = 0.5 : 1 : (nTop + 0.5)
    plot(ax2, [jj jj], [0.5 n_fail+0.5], '-', 'Color', [0.5 0.5 0.5], ...
        'LineWidth', 0.4);
end

% --- Overlay text values with sign + colour coding ---
for ii = 1:n_fail
    for jj = 1:nTop
        val = pct_fail_mat(ii, jj);

        % Adaptive text colour: white on dark cells, black on light
        frac = abs(val) / clim_val;  % 0..1 saturation
        if frac > 0.45
            tc = [1 1 1];
        else
            tc = [0.08 0.08 0.08];
        end

        % Show sign explicitly and use % suffix
        txt = sprintf('%+.1f%%', val);
        text(ax2, jj, ii, txt, ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'FontSize', 11, 'Color', tc, 'FontWeight', 'bold');
    end
end

% --- ±1σ and ±2σ threshold markers per column ---
for jj = 1:nTop
    s1 = sigma3_pct.(top10_names{jj}) / 3;   % 1σ in %
    s2 = 2 * s1;                              % 2σ in %
    for ii = 1:n_fail
        val = abs(pct_fail_mat(ii, jj));
        if val > s2
            % Red diamond for > 2σ
            plot(ax2, jj + 0.38, ii - 0.38, 'p', 'MarkerSize', 6, ...
                'MarkerFaceColor', [1 0.2 0.2], 'MarkerEdgeColor', 'none');
        elseif val > s1
            % Orange triangle for > 1σ
            plot(ax2, jj + 0.38, ii - 0.38, '^', 'MarkerSize', 5, ...
                'MarkerFaceColor', [1 0.6 0.1], 'MarkerEdgeColor', 'none');
        end
    end
end
hold(ax2, 'off');

% --- Axis tick labels ---
set(ax2, 'XTick', 1:nTop, 'XTickLabel', x_labels, ...
    'XTickLabelRotation', 35, 'FontSize', 11, 'FontWeight', 'bold', ...
    'TickLabelInterpreter', 'none', 'XColor', [0.05 0.05 0.05], 'YColor', [0.05 0.05 0.05]);
set(ax2, 'YTick', 1:n_fail, 'YTickLabel', case_labels, 'FontSize', 10, ...
    'TickLabelInterpreter', 'none');
set(ax2, 'TickDir', 'out', 'Box', 'on', 'LineWidth', 1.2);

ylabel(ax2, 'Failed Case', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.05 0.05 0.05]);
xlabel(ax2, 'Parameter (ranked by Composite Importance Score)', ...
    'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.05 0.05 0.05]);

% --- Title + subtitle ---
title(ax2, 'Parameter Perturbation from Nominal — All Failed Cases', ...
    'FontSize', 15, 'FontWeight', 'bold', 'Color', [0.05 0.05 0.05]);

% --- Legend for σ markers ---
annotation(fig2, 'textbox', [0.30 0.005 0.58 0.04], ...
    'String', [char(9650) ' > 1\sigma     ' char(9733) ' > 2\sigma     ' ...
               'Colour: blue = negative, red = positive perturbation'], ...
    'FontSize', 9, 'HorizontalAlignment', 'center', ...
    'EdgeColor', 'none', 'FitBoxToText', 'off', ...
    'FontWeight', 'bold', 'Color', [0.05 0.05 0.05]);

exportgraphics(fig2, fullfile(out_dir, 'fig_failed_perturbation_heatmap.png'), ...
    'Resolution', 300, 'BackgroundColor', fig_bg);
fprintf('    Saved → fig_failed_perturbation_heatmap.png\n');

%% ═══════════════════════════════════════════════════════════════════════
%  9.  FIGURE 3:  Absolute Perturbation Comparison (bar chart)
%  ═══════════════════════════════════════════════════════════════════════
fprintf('  [6] Generating absolute perturbation bar chart ...\n');

fig3 = figure('Position', [70 70 1100 550], 'Color', fig_bg, ...
    'Name', 'Mean |Perturbation| Comparison');
ax3 = axes(fig3, 'Color', ax_bg);
hold(ax3, 'on');

abs_pass_mean = zeros(nTop, 1);
abs_fail_mean = zeros(nTop, 1);
for jj = 1:nTop
    abs_pass_mean(jj) = mean(abs(pct_delta(passed, jj)));
    abs_fail_mean(jj) = mean(abs(pct_delta(failed, jj)));
end

x = 1:nTop;
bw = 0.35;
bar(ax3, x - bw/2, abs_pass_mean, bw, 'FaceColor', c_pass, 'EdgeColor', 'none', ...
    'FaceAlpha', 0.85, 'DisplayName', sprintf('Passed (N=%d)', n_pass));
bar(ax3, x + bw/2, abs_fail_mean, bw, 'FaceColor', c_fail, 'EdgeColor', 'none', ...
    'FaceAlpha', 0.85, 'DisplayName', sprintf('Failed (N=%d)', n_fail));

% 1-sigma reference line (σ = 3σ_bound / 3)
for jj = 1:nTop
    s1 = sigma3_pct.(top10_names{jj}) / 3;
    plot(ax3, [jj-0.5 jj+0.5], [s1 s1], '--', 'Color', [0.15 0.15 0.15], ...
        'LineWidth', 1.2, 'HandleVisibility', 'off');
end
% Dummy for legend
plot(ax3, NaN, NaN, '--', 'Color', [0.15 0.15 0.15], 'LineWidth', 1.2, ...
    'DisplayName', '1\sigma bound');

set(ax3, 'XTick', x, 'XTickLabel', cellfun(@(n) pretty.(n), top10_names, 'UniformOutput', false));
set(ax3, 'TickLabelInterpreter', 'latex', 'FontSize', 11, ...
    'XColor', c_txt, 'YColor', c_txt, 'FontWeight', 'bold');
ylabel(ax3, 'Mean |perturbation| (%)', 'FontSize', 12, 'Color', c_txt, 'FontWeight', 'bold');
title(ax3, 'Mean Absolute Perturbation — Failed vs Passed (Top-10 Parameters)', ...
    'FontSize', 14, 'FontWeight', 'bold', 'Color', c_txt);
legend(ax3, 'Location', 'northwest', 'FontSize', 10, 'TextColor', c_txt);
grid(ax3, 'on');
set(ax3, 'GridAlpha', 0.15);
hold(ax3, 'off');

exportgraphics(fig3, fullfile(out_dir, 'fig_abs_perturbation_comparison.png'), ...
    'Resolution', 300, 'BackgroundColor', fig_bg);
fprintf('    Saved → fig_abs_perturbation_comparison.png\n');

%% ═══════════════════════════════════════════════════════════════════════
%  10.  TEXT REPORT
%  ═══════════════════════════════════════════════════════════════════════
fprintf('  [7] Writing summary report ...\n');

rpt_path = fullfile(out_dir, 'failure_analysis_report.txt');
fid = fopen(rpt_path, 'w');

fprintf(fid, '══════════════════════════════════════════════════════════════════\n');
fprintf(fid, '  FAILED-CASE PERTURBATION ANALYSIS\n');
fprintf(fid, '  Generated: %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf(fid, '══════════════════════════════════════════════════════════════════\n\n');

fprintf(fid, '  Dataset:     %s\n', results_path);
fprintf(fid, '  Total cases: %d\n', N);
fprintf(fid, '  Passed:  %d  (%.1f%%)\n', n_pass, 100*n_pass/N);
fprintf(fid, '  Failed:  %d  (%.1f%%)\n\n', n_fail, 100*n_fail/N);

fprintf(fid, '  Stall AoA threshold: 30 deg (mc_postprocess.m)\n');
fprintf(fid, '  Pass = no flags triggered;  Fail = any flag triggered.\n\n');

fprintf(fid, '  FLAG BREAKDOWN (failed cases only):\n');
for jj = 1:nFlags
    fc = sum(F(failed, jj));
    if fc > 0
        triggered_ids = T_met.case_id(failed & F(:,jj)==1);
        id_str = strjoin(arrayfun(@(c) sprintf('%04d', c), triggered_ids, 'UniformOutput', false), ', ');
        fprintf(fid, '    %-16s  %2d cases  [%s]\n', flag_labels{jj}, fc, id_str);
    else
        fprintf(fid, '    %-16s  (none)\n', flag_labels{jj});
    end
end
fprintf(fid, '\n');

fprintf(fid, '──────────────────────────────────────────────────────────────────\n');
fprintf(fid, '  FAILED CASES & THEIR TRIGGER FLAGS\n');
fprintf(fid, '──────────────────────────────────────────────────────────────────\n');
for ii = 1:n_fail
    fprintf(fid, '  Case %04d  [%d flags]:  %s\n', ...
        T_fail.case_id(ii), T_fail.total_flags(ii), T_fail.fail_reasons{ii});
end
fprintf(fid, '\n');

fprintf(fid, '──────────────────────────────────────────────────────────────────\n');
fprintf(fid, '  TOP-10 PARAMETER PERTURBATION COMPARISON  (failed vs passed)\n');
fprintf(fid, '──────────────────────────────────────────────────────────────────\n');
fprintf(fid, '  %-16s  %8s  %8s  %8s  %8s  %8s  %s\n', ...
    'Parameter', 'Fail_μ%', 'Pass_μ%', 'Diff%', '|Fail|μ', 'p-value', 'Sig');
fprintf(fid, '  %-16s  %8s  %8s  %8s  %8s  %8s  %s\n', ...
    '────────────────', '────────', '────────', '────────', '────────', '────────', '───');
for jj = 1:nTop
    fprintf(fid, '  %-16s  %+8.2f  %+8.2f  %+8.2f  %8.2f  %8.4f  %s\n', ...
        top10_names{jj}, ...
        summary_fail_mean(jj), summary_pass_mean(jj), summary_mean_diff(jj), ...
        abs_fail_mean(jj), summary_ranksum_p(jj), summary_significant{jj});
end
fprintf(fid, '\n');

fprintf(fid, '──────────────────────────────────────────────────────────────────\n');
fprintf(fid, '  KEY FINDINGS\n');
fprintf(fid, '──────────────────────────────────────────────────────────────────\n');

% Find parameters with significant differences
sig_params = find(summary_ranksum_p < 0.05);
if ~isempty(sig_params)
    fprintf(fid, '  Parameters with statistically significant (p<0.05) mean\n');
    fprintf(fid, '  perturbation differences between failed and passed cases:\n');
    for kk = sig_params(:)'
        fprintf(fid, '    • %s  (p = %.4f, Δμ = %+.2f%%)\n', ...
            top10_names{kk}, summary_ranksum_p(kk), summary_mean_diff(kk));
    end
else
    fprintf(fid, '  No parameters show statistically significant (p<0.05) mean\n');
    fprintf(fid, '  perturbation differences between failed and passed cases.\n');
    fprintf(fid, '  This suggests failures arise from COMBINATIONS of moderate\n');
    fprintf(fid, '  perturbations rather than extreme single-parameter shifts.\n');
end
fprintf(fid, '\n');

% Which parameters have the largest absolute perturbations in failed cases?
[~, abs_sort] = sort(abs_fail_mean, 'descend');
fprintf(fid, '  Parameters with largest mean |perturbation| in failed cases:\n');
for kk = abs_sort(1:min(5, nTop))'
    fprintf(fid, '    • %s  (mean |Δ| = %.2f%%,  3σ bound = %.0f%%)\n', ...
        top10_names{kk}, abs_fail_mean(kk), sigma3_pct.(top10_names{kk}));
end
fprintf(fid, '\n');

% Check if any failed case has a parameter perturbation exceeding 2σ
fprintf(fid, '  Failed cases with perturbation exceeding 2σ (extreme outliers):\n');
any_extreme = false;
for ii = 1:n_fail
    for jj = 1:nTop
        s2 = 2 * sigma3_pct.(top10_names{jj}) / 3;  % 2σ in %
        val = abs(pct_delta(fail_idx(si(ii)), jj));
        if val > s2
            fprintf(fid, '    Case %04d:  %s = %+.1f%%  (2σ = %.1f%%)\n', ...
                T_fail.case_id(ii), top10_names{jj}, ...
                pct_delta(fail_idx(si(ii)), jj), s2);
            any_extreme = true;
        end
    end
end
if ~any_extreme
    fprintf(fid, '    (none)\n');
end

fprintf(fid, '\n══════════════════════════════════════════════════════════════════\n');
fprintf(fid, '  Wall time: %.1f s\n', toc(t_wall));
fprintf(fid, '══════════════════════════════════════════════════════════════════\n');
fclose(fid);
fprintf('    Saved → %s\n', rpt_path);

%% ═══════════════════════════════════════════════════════════════════════
%  11.  CONSOLE SUMMARY
%  ═══════════════════════════════════════════════════════════════════════
fprintf('\n══════════════════════════════════════════════════════════════════\n');
fprintf('  ANALYSIS COMPLETE\n');
fprintf('══════════════════════════════════════════════════════════════════\n');
fprintf('  Output directory: %s\n', out_dir);
fprintf('  Files generated:\n');
fprintf('    1. failed_case_perturbations.csv    (per-case perturbation %%)\n');
fprintf('    2. failure_perturbation_summary.csv  (statistical comparison)\n');
fprintf('    3. failed_case_flags.csv             (flag breakdown)\n');
fprintf('    4. fig_failed_vs_passed_boxplot.png  (boxplots)\n');
fprintf('    5. fig_failed_perturbation_heatmap.png (heatmap)\n');
fprintf('    6. fig_abs_perturbation_comparison.png (bar chart)\n');
fprintf('    7. failure_analysis_report.txt       (text summary)\n');
fprintf('  Wall time: %.1f s\n', toc(t_wall));
fprintf('══════════════════════════════════════════════════════════════════\n');


%% ═══════════════════════════════════════════════════════════════════════
%  LOCAL FUNCTIONS
%  ═══════════════════════════════════════════════════════════════════════

function bp = boxplot_manual(ax, data, pos, col, width)
%BOXPLOT_MANUAL  Draw a single box-and-whisker without Statistics Toolbox.
    q1 = prctile_approx(data, 25);
    q2 = prctile_approx(data, 50);  % median
    q3 = prctile_approx(data, 75);
    iqr_val = q3 - q1;
    whi = min(max(data), q3 + 1.5*iqr_val);
    wlo = max(min(data), q1 - 1.5*iqr_val);

    hw = width / 2;

    % Box
    fill(ax, pos + [-hw -hw hw hw], [q1 q3 q3 q1], col, ...
        'FaceAlpha', 0.3, 'EdgeColor', col, 'LineWidth', 1.5);
    % Median line
    plot(ax, pos + [-hw hw], [q2 q2], '-', 'Color', col, 'LineWidth', 2.5);
    % Whiskers
    plot(ax, [pos pos], [wlo q1], '-', 'Color', col, 'LineWidth', 1.2);
    plot(ax, [pos pos], [q3 whi], '-', 'Color', col, 'LineWidth', 1.2);
    % Whisker caps
    plot(ax, pos + [-hw/2 hw/2], [wlo wlo], '-', 'Color', col, 'LineWidth', 1.2);
    plot(ax, pos + [-hw/2 hw/2], [whi whi], '-', 'Color', col, 'LineWidth', 1.2);
    % Mean marker
    mu = mean(data);
    plot(ax, pos, mu, 'd', 'MarkerSize', 7, 'MarkerFaceColor', col, ...
        'MarkerEdgeColor', col*0.6, 'LineWidth', 1.0);

    bp = struct('q1', q1, 'q2', q2, 'q3', q3, 'whi', whi, 'wlo', wlo, 'mu', mu);
end


function p = prctile_approx(x, pct)
%PRCTILE_APPROX  Percentile without Statistics Toolbox.
    x = sort(x(:));
    n = numel(x);
    r = pct/100 * (n-1) + 1;
    lo = floor(r);
    hi = ceil(r);
    if lo < 1, lo = 1; end
    if hi > n, hi = n; end
    if lo == hi
        p = x(lo);
    else
        p = x(lo) + (r - lo) * (x(hi) - x(lo));
    end
end


function p = ranksum_test(x, y)
%RANKSUM_TEST  Two-sided Wilcoxon rank-sum test (normal approximation).
%  Equivalent to MATLAB's ranksum() but without Statistics Toolbox.
    nx = numel(x);
    ny = numel(y);
    combined = [x(:); y(:)];
    [~, si] = sort(combined);
    ranks = zeros(size(combined));
    ranks(si) = 1:(nx+ny);

    % Handle ties: average ranks
    [uc, ~, ic] = unique(combined);
    for kk = 1:numel(uc)
        mask = (ic == kk);
        if sum(mask) > 1
            ranks(mask) = mean(ranks(mask));
        end
    end

    W  = sum(ranks(1:nx));          % rank sum of first sample
    mu_W  = nx * (nx + ny + 1) / 2;
    sig_W = sqrt(nx * ny * (nx + ny + 1) / 12);

    % Continuity correction
    z = (abs(W - mu_W) - 0.5) / sig_W;

    % Two-sided p-value from standard normal
    p = 2 * (1 - normcdf_approx(z));
    p = min(p, 1);
end


function p = normcdf_approx(z)
%NORMCDF_APPROX  Standard normal CDF approximation (Abramowitz & Stegun).
    z = abs(z);
    t = 1 ./ (1 + 0.2316419 * z);
    d = 0.3989422804014327;   % 1/sqrt(2*pi)
    poly = ((((1.330274429*t - 1.821255978).*t + 1.781477937).*t ...
             - 0.356563782).*t + 0.319381530).*t;
    p_tail = d * exp(-0.5*z.^2) .* poly;
    p = 1 - p_tail;
end


function cmap = bluewhitered_dark(n)
%BLUEWHITERED_DARK  High-contrast blue-white-red diverging colormap.
    if nargin < 1, n = 512; end
    half = floor(n/2);
    t1 = linspace(0, 1, half)';
    gamma1 = t1.^0.7;
    r1 = 0.05 + gamma1 * (1 - 0.05);
    g1 = 0.10 + gamma1 * (1 - 0.10);
    b1 = 0.55 + gamma1 * (1 - 0.55);
    t2 = linspace(0, 1, n - half)';
    gamma2 = t2.^0.7;
    r2 = 1 - gamma2 * (1 - 0.70);
    g2 = 1 - gamma2 * (1 - 0.02);
    b2 = 1 - gamma2 * (1 - 0.02);
    cmap = [r1 g1 b1; r2 g2 b2];
end
