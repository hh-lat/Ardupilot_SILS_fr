%% MC_SENSITIVITY  Monte Carlo Sensitivity Analysis & Failure Prediction
% =========================================================================
%  Determines which aerodynamic/inertial parameter perturbations most
%  influence each flight-performance metric and predicts pass/fail.
%
%  Methods:
%    1. Pearson Correlation Coefficients  (PCC)  — linear dependence
%    2. Spearman Rank Correlation         (SRCC) — monotonic dependence
%    3. Standardised Regression Coeffs    (SRC)  — multivariate linear
%    4. Partial Rank Correlation Coeffs   (PRCC) — multivariate monotonic
%    5. L2-regularised Logistic Regression (IRLS) — pass/fail classifier
%    6. k-Fold cross-validated metrics    (accuracy, precision, recall,
%       F1-score, balanced accuracy, AUC-ROC)
%
%  Outputs (saved to <results_path>/sensitivity/):
%    sensitivity_src.csv      — SRC values (params × metrics)
%    sensitivity_prcc.csv     — PRCC values (params × metrics)
%    prediction_report.txt    — classifier performance
%    fig_src_heatmap.png      — SRC heatmap
%    fig_prcc_heatmap.png     — PRCC heatmap
%    fig_tornado_*.png        — tornado charts per critical metric
%    fig_r2_model_fit.png     — R² bar chart
%    fig_logreg_importance.png — logistic-regression odds-ratio chart
%    fig_roc_curve.png        — ROC curve
%    fig_fail_vs_pass.png     — box-plots for failed vs passed cases
%
%  Requirements: Base MATLAB ≥ R2019a (no toolboxes needed)
%
%  Author : LAT Avionics
%  Date   : 2026-02-18
% =========================================================================

clc; clear; close all;
t_wall = tic;

%% ═══════════════════════════════════════════════════════════════════════
%  0.  CONFIGURATION
%  ═══════════════════════════════════════════════════════════════════════
results_path = '\\wsl.localhost\Ubuntu\home\lat_avionics\Ardupilot_SITL_LATEST\Ardupilot_SILS\Tools\autotest\monte_carlo\mc_results_20260217_074641';

k_folds    = 10;        % cross-validation folds
lambda_log = 0.1;       % L2 regularisation (moderate: balances bias/variance)
rng_seed   = 42;        % for reproducible fold splits

% Output directory
out_dir = fullfile(results_path, 'sensitivity');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

%% ═══════════════════════════════════════════════════════════════════════
%  1.  DATA INGESTION
%  ═══════════════════════════════════════════════════════════════════════
fprintf('══════════════════════════════════════════════════════════════════\n');
fprintf('  MC Sensitivity Analysis & Failure Prediction\n');
fprintf('══════════════════════════════════════════════════════════════════\n');

% --- 1a. Read summary.csv  (parameter perturbation values) --------------
fprintf('  Loading summary.csv  ...');
sum_csv = fullfile(results_path, 'summary.csv');
T_sum   = readtable(sum_csv, 'Delimiter', ',', 'TextType', 'string');
fprintf(' %d cases, %d columns\n', height(T_sum), width(T_sum));

% --- 1b. Read mc_performance_report_matlab.csv  (metrics + flags) -------
fprintf('  Loading mc_performance_report_matlab.csv  ...');
met_csv = fullfile(results_path, 'mc_performance_report_matlab.csv');
T_met   = readtable(met_csv, 'Delimiter', ',', 'TextType', 'string');
fprintf(' %d cases, %d columns\n', height(T_met), width(T_met));

% --- 1c. Align on case_id -----------------------------------------------
assert(height(T_sum) == height(T_met), 'Row count mismatch between CSVs');
assert(all(T_sum.case_id == T_met.case_id), 'case_id mismatch');
N = height(T_sum);

% --- 1d. Extract parameter matrix X (N × nP) ----------------------------
param_cols = T_sum.Properties.VariableNames( ...
    startsWith(T_sum.Properties.VariableNames, 'p_'));
nP = numel(param_cols);
X  = table2array(T_sum(:, param_cols));      % N × nP

% Clean parameter labels (drop 'p_' prefix)
param_labels = cellfun(@(s) strrep(s, 'p_', ''), param_cols, ...
    'UniformOutput', false);
% Pretty labels for plots
param_pretty = strrep(param_labels, '_', '\_');

% --- 1e. Extract metric matrix Y (N × nM) for focus metrics -------------
focus_metrics = {
    'takeoff_distance_m',     'Takeoff Distance (m)';
    'takeoff_speed_mps',      'Takeoff Speed (m/s)';
    'takeoff_time_s',         'Takeoff Time (s)';
    'aoa_at_liftoff_deg',     'AoA at Liftoff (deg)';
    'alt_ss_error_m',         'Alt SS Error (m)';
    'alt_ss_std_m',           'Alt SS Std (m)';
    'max_roll_rate_dps',      'Max Roll Rate (dps)';
    'max_pitch_rate_dps',     'Max Pitch Rate (dps)';
    'max_roll_deg',           'Max |Roll| (deg)';
    'max_roll_ss_deg',        'Max |Roll| SS (deg)';
    'max_pitch_deg',          'Max Pitch (deg)';
    'min_pitch_deg',          'Min Pitch (deg)';
    'circle_radius_err_pct',  'Circle Radius Err (%)';
    'aoa_max_deg',            'Max AoA (deg)';
    'aoa_min_deg',            'Min AoA (deg)';
    'max_airspeed_mps',       'Max Airspeed (m/s)';
    'min_airspeed_mps',       'Min Airspeed (m/s)';
    'max_load_factor_g',      'Max Load Factor (g)';
};
metric_names  = focus_metrics(:,1);
metric_labels = focus_metrics(:,2);
nM = size(focus_metrics, 1);
Y  = zeros(N, nM);
for jj = 1:nM
    Y(:,jj) = T_met.(metric_names{jj});
end

% --- 1f. Extract pass/fail vector ----------------------------------------
passed = T_met.passed;   % 1=pass, 0=fail
failed = ~passed;
n_fail = sum(failed);
n_pass = sum(passed);
fprintf('  Cases: %d total  |  %d PASS (%.1f%%)  |  %d FAIL (%.1f%%)\n', ...
    N, n_pass, 100*n_pass/N, n_fail, 100*n_fail/N);

% --- 1g. Extract individual flag vectors ---------------------------------
flag_names = {'flag_alt_error','flag_roll_rate','flag_pitch_rate', ...
    'flag_roll_angle','flag_pitch_angle','flag_circle_radius', ...
    'flag_stall_aoa','flag_airspeed_low','flag_airspeed_high'};
flag_labels = {'Alt Error','Roll Rate','Pitch Rate','Roll Angle', ...
    'Pitch Angle','Circle Radius','Stall AoA','Low Airspeed','High Airspeed'};
nF = numel(flag_names);
F  = zeros(N, nF);
for jj = 1:nF
    F(:,jj) = T_met.(flag_names{jj});
end

fprintf('══════════════════════════════════════════════════════════════════\n\n');

%% ═══════════════════════════════════════════════════════════════════════
%  2.  PEARSON CORRELATION COEFFICIENTS  (PCC)
%  ═══════════════════════════════════════════════════════════════════════
fprintf('  [2] Computing Pearson Correlation Coefficients ...\n');
PCC   = zeros(nP, nM);
PCC_p = zeros(nP, nM);   % p-values
for jj = 1:nM
    for ii = 1:nP
        [PCC(ii,jj), PCC_p(ii,jj)] = pearson_corr(X(:,ii), Y(:,jj));
    end
end

%% ═══════════════════════════════════════════════════════════════════════
%  3.  SPEARMAN RANK CORRELATION COEFFICIENTS  (SRCC)
%  ═══════════════════════════════════════════════════════════════════════
fprintf('  [3] Computing Spearman Rank Correlation Coefficients ...\n');
X_rank = simple_rank_matrix(X);
Y_rank = simple_rank_matrix(Y);

SRCC   = zeros(nP, nM);
SRCC_p = zeros(nP, nM);
for jj = 1:nM
    for ii = 1:nP
        [SRCC(ii,jj), SRCC_p(ii,jj)] = pearson_corr(X_rank(:,ii), Y_rank(:,jj));
    end
end

%% ═══════════════════════════════════════════════════════════════════════
%  4.  STANDARDISED REGRESSION COEFFICIENTS  (SRC)  +  R²
%  ═══════════════════════════════════════════════════════════════════════
fprintf('  [4] Computing Standardised Regression Coefficients ...\n');
SRC  = zeros(nP, nM);
R2   = zeros(nM, 1);

% Standardise parameters (z-score)
X_mu  = mean(X, 1);
X_sd  = std(X, 0, 1);
X_std = (X - X_mu) ./ X_sd;

for jj = 1:nM
    y_j   = Y(:, jj);
    y_mu  = mean(y_j);
    y_sd  = std(y_j);
    if y_sd < 1e-12, continue; end          % constant metric — skip
    y_std = (y_j - y_mu) / y_sd;

    % OLS on standardised data (intercept absorbed by standardisation)
    beta = (X_std' * X_std) \ (X_std' * y_std);
    SRC(:, jj) = beta;

    % Model fit
    y_hat  = X_std * beta;
    SS_res = sum((y_std - y_hat).^2);
    SS_tot = sum(y_std.^2);                 % mean is zero
    R2(jj) = 1 - SS_res / SS_tot;
end

%% ═══════════════════════════════════════════════════════════════════════
%  5.  PARTIAL RANK CORRELATION COEFFICIENTS  (PRCC)
%  ═══════════════════════════════════════════════════════════════════════
fprintf('  [5] Computing Partial Rank Correlation Coefficients ...\n');
PRCC   = zeros(nP, nM);
PRCC_p = zeros(nP, nM);

for jj = 1:nM
    y_r = Y_rank(:, jj);
    for ii = 1:nP
        % Other parameters (all except ii)
        other_idx = [1:ii-1, ii+1:nP];
        Z = X_rank(:, other_idx);

        % Partial out the other parameters
        res_x = X_rank(:, ii) - Z * (Z \ X_rank(:, ii));
        res_y = y_r           - Z * (Z \ y_r);

        [PRCC(ii,jj), PRCC_p(ii,jj)] = pearson_corr(res_x, res_y);
    end
end
fprintf('  [5] Done (nP×nM = %d evaluations)\n', nP*nM);

%% ═══════════════════════════════════════════════════════════════════════
%  6.  LOGISTIC REGRESSION  — PASS/FAIL CLASSIFIER
%  ═══════════════════════════════════════════════════════════════════════
fprintf('  [6] Training L2-regularised logistic regression ...\n');

% Target: y=1 means FAIL (minority class, what we want to detect)
y_fail = double(failed);

% Standardise inputs (re-use X_std from Section 4)
% Add intercept column
X_lr = [ones(N,1), X_std];

% Class weights (balance: weight failures by n_pass/n_fail)
w_class = ones(N, 1);
if n_fail > 0
    w_class(y_fail == 1) = n_pass / n_fail;
end

% Full-data fit (with improved solver)
[beta_lr, lr_conv, lr_iterations] = logreg_irls(X_lr, y_fail, w_class, ...
    lambda_log, 500, 1e-8);
if ~lr_conv
    fprintf('    ⚠ WARNING: IRLS did NOT converge in %d iterations!\n', lr_iterations);
else
    fprintf('    Converged in %d iterations (lambda=%.3f)\n', lr_iterations, lambda_log);
end

% Predicted probabilities on training data (for ROC / full-model evaluation)
p_fail_train = sigmoid(X_lr * beta_lr);

% Odds-ratio interpretation: exp(beta) for one-SD change in parameter
odds_ratio = exp(beta_lr(2:end));   % exclude intercept

%% ═══════════════════════════════════════════════════════════════════════
%  7.  k-FOLD STRATIFIED CROSS-VALIDATION
%  ═══════════════════════════════════════════════════════════════════════
fprintf('  [7] Running %d-fold stratified cross-validation ...\n', k_folds);
rng_state = rng(rng_seed);

% Stratified fold assignment (preserve class ratio in each fold)
fold_idx = zeros(N, 1);
idx_fail = find(y_fail == 1);
idx_pass = find(y_fail == 0);

% Shuffle within each class
idx_fail = idx_fail(randperm(numel(idx_fail)));
idx_pass = idx_pass(randperm(numel(idx_pass)));

% Distribute to folds
for ii = 1:numel(idx_fail)
    fold_idx(idx_fail(ii)) = mod(ii-1, k_folds) + 1;
end
for ii = 1:numel(idx_pass)
    fold_idx(idx_pass(ii)) = mod(ii-1, k_folds) + 1;
end

% Cross-validate  (also accumulate betas for stable importance estimate)
cv_y_true  = zeros(N, 1);
cv_y_prob  = zeros(N, 1);
cv_y_pred  = zeros(N, 1);
threshold  = 0.5;
cv_betas   = zeros(nP + 1, k_folds);   % accumulate per-fold betas

for kk = 1:k_folds
    test_mask  = (fold_idx == kk);
    train_mask = ~test_mask;

    X_train = X_lr(train_mask, :);
    y_train = y_fail(train_mask);
    w_train = w_class(train_mask);

    X_test  = X_lr(test_mask, :);
    y_test  = y_fail(test_mask);

    [beta_k, ~, ~] = logreg_irls(X_train, y_train, w_train, ...
        lambda_log, 500, 1e-8);

    cv_betas(:, kk) = beta_k;
    p_test = sigmoid(X_test * beta_k);

    cv_y_true(test_mask)  = y_test;
    cv_y_prob(test_mask)  = p_test;
    cv_y_pred(test_mask)  = double(p_test >= threshold);
end

% ---- CV-averaged beta (more stable than full-data beta) ----
beta_lr_cv = mean(cv_betas, 2);          % (nP+1) × 1
beta_lr_cv_std = std(cv_betas, 0, 2);    % variability across folds
fprintf('    CV-averaged beta computed (mean across %d folds)\n', k_folds);

% Print beta stability diagnostic
beta_params = beta_lr_cv(2:end);         % exclude intercept
beta_params_std = beta_lr_cv_std(2:end);
coeff_var = abs(beta_params_std) ./ max(abs(beta_params), 1e-12);
n_unstable = sum(coeff_var > 1.0);       % CV > 100% means very unstable
if n_unstable > 0
    fprintf('    ⚠ %d/%d betas have high variability across folds (CV > 1)\n', ...
        n_unstable, nP);
end

% Confusion matrix
TP = sum(cv_y_pred == 1 & cv_y_true == 1);
TN = sum(cv_y_pred == 0 & cv_y_true == 0);
FP = sum(cv_y_pred == 1 & cv_y_true == 0);
FN = sum(cv_y_pred == 0 & cv_y_true == 1);

accuracy   = (TP + TN) / N;
precision_ = TP / max(TP + FP, 1);
recall_    = TP / max(TP + FN, 1);
f1_score   = 2 * precision_ * recall_ / max(precision_ + recall_, 1e-12);
specificity_ = TN / max(TN + FP, 1);
bal_acc    = (recall_ + specificity_) / 2;

% ROC curve and AUC
[fpr_vec, tpr_vec, auc_val] = compute_roc(cv_y_true, cv_y_prob);

fprintf('    ──── %d-Fold CV Results ────\n', k_folds);
fprintf('    Accuracy       : %.2f%%\n', 100*accuracy);
fprintf('    Balanced Acc   : %.2f%%\n', 100*bal_acc);
fprintf('    Precision      : %.2f%%\n', 100*precision_);
fprintf('    Recall (TPR)   : %.2f%%\n', 100*recall_);
fprintf('    Specificity    : %.2f%%\n', 100*specificity_);
fprintf('    F1 Score       : %.4f\n',   f1_score);
fprintf('    AUC-ROC        : %.4f\n',   auc_val);
fprintf('    Confusion: TP=%d  TN=%d  FP=%d  FN=%d\n', TP, TN, FP, FN);

%% ═══════════════════════════════════════════════════════════════════════
%  8.  PARAMETER IMPORTANCE RANKING  (composite score)
%  ═══════════════════════════════════════════════════════════════════════
fprintf('\n  [8] Computing composite parameter importance ...\n');

% Composite importance: average of normalised |SRC| and |PRCC| across
% all focus metrics.

% Mean absolute SRC across metrics (for each parameter)
mean_abs_SRC  = mean(abs(SRC), 2);            % nP × 1
mean_abs_PRCC = mean(abs(PRCC), 2);           % nP × 1

% Normalise each to [0,1]  (add eps to avoid 0/0 if all values are zero)
norm_SRC  = mean_abs_SRC  / max(mean_abs_SRC  + 1e-30);
norm_PRCC = mean_abs_PRCC / max(mean_abs_PRCC + 1e-30);

% Composite (equal weighting of 2 methods)
composite = (norm_SRC + norm_PRCC) / 2;

% Sort descending
[~, sort_idx] = sort(composite, 'descend');

fprintf('\n    %-25s  %8s  %8s  %10s\n', ...
    'Parameter', '|SRC|', '|PRCC|', 'Composite');
fprintf('    %s\n', repmat('─', 1, 56));
for ii = 1:nP
    jj = sort_idx(ii);
    fprintf('    %-25s  %8.4f  %8.4f  %10.4f\n', ...
        param_labels{jj}, norm_SRC(jj), norm_PRCC(jj), composite(jj));
end

%% ═══════════════════════════════════════════════════════════════════════
%  9.  VISUALISATION  (PRCC Heatmap + Composite Importance only)
%  ═══════════════════════════════════════════════════════════════════════
fprintf('\n  [9] Generating figures ...\n');

% ---- 9a. PRCC Heatmap  (publication-quality, extreme clarity) -----------
figB = figure('Units','pixels','Position',[20 20 2000 1100], ...
              'Color','w','Name','PRCC Heatmap');

ax_prcc = axes('Position', [0.20 0.20 0.68 0.72]);   % generous margin for labels
imagesc(PRCC');
hold on;

% --- Dark, saturated blue-white-red colormap ---
cmap_prcc = bluewhitered_dark(512);
colormap(ax_prcc, cmap_prcc);
caxis([-1 1]);

% --- Colour bar ---
cb = colorbar('FontSize', 13, 'FontWeight', 'bold', 'TickDirection', 'out', ...
              'LineWidth', 1);
cb.Label.String = 'PRCC';
cb.Label.FontSize = 15;
cb.Label.FontWeight = 'bold';
cb.Label.Color = [0 0 0];

% --- Axis labels (maximum contrast) ---
set(ax_prcc, ...
    'YTick', 1:nM, 'YTickLabel', metric_labels, ...
    'XTick', 1:nP, 'XTickLabel', param_pretty, ...
    'XTickLabelRotation', 50, ...
    'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'Helvetica', ...
    'TickDir', 'out', 'TickLength', [0.003 0.003], ...
    'Box', 'on', 'LineWidth', 1.5, ...
    'XColor', [0 0 0], 'YColor', [0 0 0]);
ax_prcc.XAxis.Label.Color = [0 0 0];
ax_prcc.YAxis.Label.Color = [0 0 0];
ax_prcc.XAxis.Color = [0 0 0];
ax_prcc.YAxis.Color = [0 0 0];
xlabel('Perturbed Parameters', 'FontSize', 16, 'FontWeight', 'bold', ...
       'Color', [0 0 0], 'FontName', 'Helvetica');
ylabel('Performance Metrics',  'FontSize', 16, 'FontWeight', 'bold', ...
       'Color', [0 0 0], 'FontName', 'Helvetica');
title('Partial Rank Correlation Coefficients  (PRCC)', ...
      'FontSize', 20, 'FontWeight', 'bold', 'Color', [0 0 0]);

% --- Grid lines ---
for gg = 0.5:1:(nP+0.5)
    plot([gg gg], [0.5 nM+0.5], 'k-', 'LineWidth', 0.5);
end
for gg = 0.5:1:(nM+0.5)
    plot([0.5 nP+0.5], [gg gg], 'k-', 'LineWidth', 0.5);
end

% --- Cell text (bold, high-contrast) ---
for ii = 1:nP
    for jj = 1:nM
        val = PRCC(ii, jj);
        if abs(val) >= 0.03
            if abs(val) > 0.40
                clr = [1 1 1];
            else
                clr = [0 0 0];
            end
            text(ii, jj, sprintf('%.2f', val), ...
                'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'middle', ...
                'FontSize', 8, 'FontWeight', 'bold', ...
                'Color', clr);
        end
    end
end

exportgraphics(figB, fullfile(out_dir, 'fig_prcc_heatmap.png'), 'Resolution', 300);
fprintf('    saved fig_prcc_heatmap.png\n');

% ---- 9b. Parameter Importance Comparison — Grouped Bar Chart ------------
%  Side-by-side |SRC| and |PRCC| for every parameter, sorted by composite.
%  This makes it crystal clear which parameters dominate and how each
%  method contributes.

figI = figure('Units','pixels','Position',[20 20 1600 1100], ...
              'Color','w','Name','Parameter Importance Comparison');

ax_comp = axes('Position', [0.22 0.07 0.68 0.84]);   % generous margins

% Grouped horizontal bar data (sorted by composite, rank-1 at top)
grouped_data = [norm_SRC(sort_idx), norm_PRCC(sort_idx)];
bh = barh(grouped_data, 'grouped');

% Vivid, distinct colours
bh(1).FaceColor = [0.12 0.47 0.71];   % SRC  — strong blue
bh(2).FaceColor = [0.96 0.51 0.19];   % PRCC — vivid orange
bh(1).EdgeColor = [0.08 0.33 0.53];
bh(2).EdgeColor = [0.72 0.38 0.14];
bh(1).LineWidth = 0.8;
bh(2).LineWidth = 0.8;

set(ax_comp, 'YTick', 1:nP, 'YTickLabel', param_pretty(sort_idx), ...
    'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'Helvetica', ...
    'YColor', [0 0 0], 'XColor', [0 0 0], ...
    'TickDir', 'out', 'LineWidth', 1.2, 'Box', 'on', ...
    'YDir', 'reverse');                       % rank 1 at top
xlabel('Normalised Importance  [0 – 1]', 'FontSize', 16, 'FontWeight', 'bold', ...
       'Color', [0 0 0]);
title({'Parameter Importance Ranking', ...
       'Composite = ( |SRC| + |PRCC| ) / 2,  sorted most \rightarrow least important'}, ...
    'FontSize', 17, 'FontWeight', 'bold', 'Color', [0 0 0]);
xlim([0 1.30]);
grid on;
set(ax_comp, 'GridAlpha', 0.25, 'GridLineStyle', '-');

lgd = legend({'|SRC|  — Standardised Regression Coefficient', ...
              '|PRCC| — Partial Rank Correlation Coefficient'}, ...
    'Location', 'southeast', 'FontSize', 12, 'FontWeight', 'bold', ...
    'EdgeColor', [0 0 0], 'LineWidth', 1);

% Annotate each parameter row with rank # and composite score
hold on;
for ii = 1:nP
    jj = sort_idx(ii);
    x_end = max(norm_SRC(jj), norm_PRCC(jj));
    text(x_end + 0.02, ii, ...
        sprintf('#%d   Comp = %.3f', ii, composite(jj)), ...
        'FontSize', 10, 'FontWeight', 'bold', ...
        'VerticalAlignment', 'middle', 'Color', [0.1 0.1 0.1]);
end

% Draw a thin vertical reference line at 0.5
plot([0.5 0.5], [0.3 nP+0.7], '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 1);

exportgraphics(figI, fullfile(out_dir, 'fig_parameter_importance.png'), 'Resolution', 300);
fprintf('    saved fig_parameter_importance.png\n');

%% ═══════════════════════════════════════════════════════════════════════
%  10. SAVE NUMERICAL RESULTS
%  ═══════════════════════════════════════════════════════════════════════
fprintf('\n  [10] Writing output files ...\n');

% ---- 10a. SRC CSV -------------------------------------------------------
src_tbl = array2table(SRC, 'VariableNames', metric_names, ...
    'RowNames', param_labels);
writetable(src_tbl, fullfile(out_dir, 'sensitivity_src.csv'), ...
    'WriteRowNames', true);

% ---- 10b. PRCC CSV ------------------------------------------------------
prcc_tbl = array2table(PRCC, 'VariableNames', metric_names, ...
    'RowNames', param_labels);
writetable(prcc_tbl, fullfile(out_dir, 'sensitivity_prcc.csv'), ...
    'WriteRowNames', true);

% ---- 10c. R² CSV --------------------------------------------------------
r2_tbl = table(metric_names, R2, 'VariableNames', {'Metric','R2'});
writetable(r2_tbl, fullfile(out_dir, 'model_fit_r2.csv'));

% ---- 10d. Composite Importance CSV --------------------------------------
imp_tbl = table(param_labels(sort_idx)', ...
    norm_SRC(sort_idx), norm_PRCC(sort_idx), ...
    composite(sort_idx), ...
    'VariableNames', {'Parameter','norm_SRC','norm_PRCC','Composite'});
writetable(imp_tbl, fullfile(out_dir, 'parameter_importance.csv'));
fprintf('    saved parameter_importance.csv  (%d parameters, ranked)\n', nP);

% ---- 10e. Prediction report (text) --------------------------------------
fid = fopen(fullfile(out_dir, 'prediction_report.txt'), 'w');
fprintf(fid, 'Monte Carlo Failure Prediction — Report\n');
fprintf(fid, '=========================================\n\n');
fprintf(fid, 'Date        : %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf(fid, 'Cases       : %d  (Pass=%d, Fail=%d)\n', N, n_pass, n_fail);
fprintf(fid, 'Parameters  : %d\n', nP);
fprintf(fid, 'Classifier  : L2-regularised Logistic Regression (IRLS)\n');
fprintf(fid, 'Lambda      : %.4f\n', lambda_log);
fprintf(fid, 'CV Folds    : %d (stratified)\n\n', k_folds);
fprintf(fid, '──── Cross-Validated Performance ────\n');
fprintf(fid, '  Accuracy       : %.2f%%\n', 100*accuracy);
fprintf(fid, '  Balanced Acc   : %.2f%%\n', 100*bal_acc);
fprintf(fid, '  Precision      : %.2f%%\n', 100*precision_);
fprintf(fid, '  Recall (TPR)   : %.2f%%\n', 100*recall_);
fprintf(fid, '  Specificity    : %.2f%%\n', 100*specificity_);
fprintf(fid, '  F1 Score       : %.4f\n',   f1_score);
fprintf(fid, '  AUC-ROC        : %.4f\n\n', auc_val);
fprintf(fid, '  Confusion Matrix:\n');
fprintf(fid, '              Pred Pass  Pred Fail\n');
fprintf(fid, '  Act Pass    %5d      %5d\n', TN, FP);
fprintf(fid, '  Act Fail    %5d      %5d\n\n', FN, TP);
fprintf(fid, '──── All Parameters — Composite Importance Ranking ────\n');
fprintf(fid, '  %-25s  %8s  %8s  %10s\n', ...
    'Parameter', '|SRC|', '|PRCC|', 'Composite');
for ii = 1:nP
    jj = sort_idx(ii);
    fprintf(fid, '  %-25s  %8.4f  %8.4f  %10.4f\n', ...
        param_labels{jj}, norm_SRC(jj), norm_PRCC(jj), composite(jj));
end
fprintf(fid, '\n──── Per-Metric R² (Linear Model Fit) ────\n');
for jj = 1:nM
    fprintf(fid, '  %-30s  R² = %.4f\n', metric_labels{jj}, R2(jj));
end
fprintf(fid, '\n──── Per-Flag: Top-3 Most Influential Parameters (PRCC) ────\n');
flag_metric_map = {
    'flag_alt_error',      'alt_ss_error_m';
    'flag_roll_angle',     'max_roll_ss_deg';
    'flag_circle_radius',  'circle_radius_err_pct';
    'flag_stall_aoa',      'aoa_max_deg';
    'flag_airspeed_low',   'min_airspeed_mps';
    'flag_airspeed_high',  'max_airspeed_mps';
    'flag_pitch_angle',    'max_pitch_deg';
};
for ff = 1:size(flag_metric_map,1)
    midx = find(strcmp(metric_names, flag_metric_map{ff,2}), 1);
    if isempty(midx), continue; end
    [~, si] = sort(abs(PRCC(:,midx)), 'descend');
    fprintf(fid, '  %s → %s\n', flag_metric_map{ff,1}, flag_metric_map{ff,2});
    for kk = 1:3
        fprintf(fid, '    #%d  %-20s  PRCC = %+.4f\n', ...
            kk, param_labels{si(kk)}, PRCC(si(kk), midx));
    end
end
fclose(fid);
fprintf('    saved prediction_report.txt\n');

%% ═══════════════════════════════════════════════════════════════════════
%  DONE
%  ═══════════════════════════════════════════════════════════════════════
elapsed = toc(t_wall);
fprintf('\n══════════════════════════════════════════════════════════════════\n');
fprintf('  Sensitivity analysis complete.  %.1f s elapsed.\n', elapsed);
fprintf('  Output directory: %s\n', out_dir);
fprintf('  Files saved:\n');
d = dir(out_dir);
for ii = 1:numel(d)
    if ~d(ii).isdir
        fprintf('    • %s\n', d(ii).name);
    end
end
fprintf('══════════════════════════════════════════════════════════════════\n');


%% ═══════════════════════════════════════════════════════════════════════
%  LOCAL FUNCTIONS
%  ═══════════════════════════════════════════════════════════════════════

function [r, p_val] = pearson_corr(x, y)
%PEARSON_CORR  Pearson correlation coefficient with p-value.
%  Uses t-test approximation: t = r*sqrt(N-2)/sqrt(1-r^2), df = N-2.
    n  = numel(x);
    mx = mean(x); my = mean(y);
    dx = x - mx;  dy = y - my;
    r  = sum(dx .* dy) / sqrt(sum(dx.^2) * sum(dy.^2) + 1e-30);
    r  = max(min(r, 1), -1);   % numerical safety

    % p-value via t-distribution approximation (for large n, ≈ normal)
    if abs(r) > 1 - 1e-10
        p_val = 0;
    else
        t_stat = r * sqrt(n - 2) / sqrt(1 - r^2);
        % Large-n normal approximation for p-value
        p_val = 2 * normcdf_local(-abs(t_stat));
    end
end


function R = simple_rank_matrix(M)
%SIMPLE_RANK_MATRIX  Rank-transform each column of M (no tie handling).
    [n, p] = size(M);
    R = zeros(n, p);
    for jj = 1:p
        [~, idx] = sort(M(:,jj));
        R(idx, jj) = (1:n)';
    end
end


function p = normcdf_local(x)
%NORMCDF_LOCAL  Standard normal CDF via erfc (no toolbox needed).
    p = 0.5 * erfc(-x / sqrt(2));
end


function s = sigmoid(z)
%SIGMOID  Logistic sigmoid, numerically stable.
    s = 1 ./ (1 + exp(-z));
    s = max(min(s, 1 - 1e-15), 1e-15);
end


function [beta, converged, iters] = logreg_irls(X, y, w_class, lambda, ...
                                                 max_iter, tol)
%LOGREG_IRLS  L2-regularised logistic regression via IRLS.
%
%  Inputs:
%    X        — N × p design matrix (column 1 = intercept = ones)
%    y        — N × 1 binary target (0 or 1)
%    w_class  — N × 1 per-sample class weights
%    lambda   — L2 regularisation strength (0 = unregularised)
%    max_iter — maximum IRLS iterations
%    tol      — convergence tolerance on ||Δβ||
%
%  Outputs:
%    beta      — p × 1 coefficient vector
%    converged — true if converged before max_iter
%    iters     — number of iterations used

    [N_lr, p] = size(X);
    beta = zeros(p, 1);
    converged = false;
    iters = max_iter;

    % ---- Initialise intercept using WEIGHTED log-odds ----
    %  With class weights, the effective failure count is sum(w * y)
    %  and the effective pass count is sum(w * (1-y)).
    %  Using unweighted log(n1/n0) gives -4.0 when the correct
    %  weighted answer is ~0, causing the solver to waste iterations.
    w_fail = sum(w_class .* y);
    w_pass = sum(w_class .* (1 - y));
    if w_fail > 0 && w_pass > 0
        beta(1) = log(w_fail / w_pass);   % ≈ 0 when class weights balance
    end

    % Regularisation matrix (don't penalise intercept)
    L = lambda * eye(p);
    L(1,1) = 0;

    for it = 1:max_iter
        eta = X * beta;
        mu  = sigmoid(eta);

        % Weighted diagonal
        W = mu .* (1 - mu) .* w_class;
        W = max(W, 1e-12);

        % Gradient of penalised log-likelihood
        grad = X' * (w_class .* (y - mu)) - L * beta;

        % Hessian (positive-definite)
        H = X' * (W .* X) + L;

        % Newton step
        delta = H \ grad;

        % ---- Step-size cap (per-component, not global norm) ----
        %  Cap each |delta_j| individually to prevent wild jumps
        %  while allowing other components to move freely.
        max_step_component = 5.0;
        delta = max(min(delta, max_step_component), -max_step_component);

        % Update
        beta = beta + delta;

        if norm(delta) < tol
            converged = true;
            iters = it;
            break;
        end
    end
end


function [fpr, tpr, auc_val] = compute_roc(y_true, y_score)
%COMPUTE_ROC  ROC curve and AUC from true labels and predicted scores.
    thresholds = sort(unique(y_score), 'descend');
    thresholds = [max(y_score)+1e-6; thresholds; min(y_score)-1e-6];

    nT  = numel(thresholds);
    fpr = zeros(nT, 1);
    tpr = zeros(nT, 1);

    P = sum(y_true == 1);
    Neg = sum(y_true == 0);

    for ii = 1:nT
        pred = double(y_score >= thresholds(ii));
        tp = sum(pred == 1 & y_true == 1);
        fp = sum(pred == 1 & y_true == 0);
        tpr(ii) = tp / max(P, 1);
        fpr(ii) = fp / max(Neg, 1);
    end

    % Sort by FPR for proper curve
    [fpr, si] = sort(fpr);
    tpr = tpr(si);

    % Remove duplicate FPR points (keep max TPR)
    [fpr, ui] = unique(fpr, 'last');
    tpr = tpr(ui);

    % AUC via trapezoidal integration
    auc_val = trapz(fpr, tpr);
end


function cmap = bluewhitered(n)
%BLUEWHITERED  Blue-white-red diverging colormap.
    if nargin < 1, n = 256; end
    half = floor(n/2);
    % Blue → White
    r1 = linspace(0.1, 1, half)';
    g1 = linspace(0.2, 1, half)';
    b1 = linspace(0.7, 1, half)';
    % White → Red
    r2 = linspace(1, 0.8, n - half)';
    g2 = linspace(1, 0.1, n - half)';
    b2 = linspace(1, 0.1, n - half)';
    cmap = [r1 g1 b1; r2 g2 b2];
end


function cmap = bluewhitered_dark(n)
%BLUEWHITERED_DARK  High-contrast blue-white-red colormap.
%  Deep navy blue at -1, pure white at 0, dark crimson red at +1.
%  Much more saturated and visible than the default version.
    if nargin < 1, n = 512; end
    half = floor(n/2);

    % --- Deep navy [0.05 0.10 0.55] → White [1 1 1] ---
    r1 = linspace(0.05, 1, half)';
    g1 = linspace(0.10, 1, half)';
    b1 = linspace(0.55, 1, half)';
    % Apply gamma curve to keep mid-tones darker / more saturated
    t1 = linspace(0, 1, half)';
    gamma1 = t1.^0.7;   % <1 makes mid-tones brighter (less dark)
    r1 = 0.05 + gamma1 * (1 - 0.05);
    g1 = 0.10 + gamma1 * (1 - 0.10);
    b1 = 0.55 + gamma1 * (1 - 0.55);

    % --- White [1 1 1] → Dark crimson [0.70 0.02 0.02] ---
    t2 = linspace(0, 1, n - half)';
    gamma2 = t2.^0.7;
    r2 = 1 - gamma2 * (1 - 0.70);
    g2 = 1 - gamma2 * (1 - 0.02);
    b2 = 1 - gamma2 * (1 - 0.02);

    cmap = [r1 g1 b1; r2 g2 b2];
end
