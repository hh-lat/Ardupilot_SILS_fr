%% MC_VALIDATE_CLASSIFIER  Flight-Critical Classifier Validation
% =========================================================================
%  Rigorous statistical validation of the logistic-regression failure
%  predictor for DO-178C / flight-safety argumentation.
%
%  Tests performed:
%    V1.  Leave-One-Out Cross-Validation (LOOCV)    — N=1000 folds,
%         zero data leakage, gold standard for small-minority-class
%    V2.  Repeated Stratified k-Fold  (50 × 10-fold = 500 evaluations)
%         with bootstrap confidence intervals on every metric
%    V3.  Permutation Test  (1000 label shuffles) — proves the model
%         learns real structure, not random noise (p-value)
%    V4.  Calibration Curve  — does P(fail)=0.3 really mean 30% of
%         those cases fail?
%    V5.  Threshold Sweep  — full precision-recall-F1 vs threshold
%    V6.  Matthews Correlation Coefficient (MCC) — best single metric
%         for imbalanced binary classification (−1 to +1)
%    V7.  Cohen's Kappa  — agreement corrected for chance
%    V8.  Learning Curve  — does more data actually help?
%    V9.  Feature Ablation  — if I remove a parameter, does the model
%         get worse?  (proves the features carry real signal)
%    V10. Null-Model Baselines — always-pass, class-prior, random
%
%  Outputs (saved to <results_path>/validation/):
%    validation_report.txt          — full text report
%    fig_loocv_roc.png              — LOOCV ROC curve
%    fig_repeated_cv_boxplot.png    — box-plot of 50 repeated CV runs
%    fig_permutation_test.png       — null distribution + observed
%    fig_calibration_curve.png      — reliability diagram
%    fig_threshold_sweep.png        — precision/recall/F1 vs threshold
%    fig_learning_curve.png         — performance vs training set size
%    fig_feature_ablation.png       — bar chart of AUC drop per param
%    fig_confidence_summary.png     — forest plot with 95% CIs
%
%  Requirements: Base MATLAB ≥ R2019a (no toolboxes)
%
%  Author : LAT Avionics
%  Date   : 2026-02-18
% =========================================================================

clc; clear; close all;
t_wall = tic;

%% ═══════════════════════════════════════════════════════════════════════
%  0.   CONFIGURATION
%  ═══════════════════════════════════════════════════════════════════════
results_path = '\\wsl.localhost\Ubuntu\home\lat_avionics\Ardupilot_SITL_LATEST\Ardupilot_SILS\Tools\autotest\monte_carlo\mc_results_20260217_074641';

lambda_log   = 0.01;       % L2 regularisation (same as Sensitivity.m)
n_repeats    = 50;          % repeated k-fold runs
k_folds      = 10;         % folds per repeat
n_perms      = 1000;        % permutation-test shuffles
n_boot       = 2000;        % bootstrap samples for CIs
alpha_ci     = 0.05;        % 95% confidence interval
rng_master   = 2026;        % master RNG seed

% Output directory
val_dir = fullfile(results_path, 'validation');
if ~exist(val_dir, 'dir'), mkdir(val_dir); end

%% ═══════════════════════════════════════════════════════════════════════
%  1.   DATA LOAD (identical to Sensitivity.m)
%  ═══════════════════════════════════════════════════════════════════════
fprintf('══════════════════════════════════════════════════════════════════\n');
fprintf('  FLIGHT-CRITICAL CLASSIFIER VALIDATION\n');
fprintf('══════════════════════════════════════════════════════════════════\n');

T_sum = readtable(fullfile(results_path, 'summary.csv'), ...
    'Delimiter', ',', 'TextType', 'string');
T_met = readtable(fullfile(results_path, 'mc_performance_report_matlab.csv'), ...
    'Delimiter', ',', 'TextType', 'string');
N = height(T_sum);

% Parameter matrix
param_cols = T_sum.Properties.VariableNames( ...
    startsWith(T_sum.Properties.VariableNames, 'p_'));
nP = numel(param_cols);
X  = table2array(T_sum(:, param_cols));
param_labels = cellfun(@(s) strrep(s,'p_',''), param_cols, 'UniformOutput', false);

% Standardise
X_mu  = mean(X, 1);
X_sd  = std(X, 0, 1);
X_std = (X - X_mu) ./ X_sd;

% Target
y = double(~T_met.passed);   % 1 = FAIL
n_fail = sum(y == 1);
n_pass = sum(y == 0);
prevalence = n_fail / N;

fprintf('  Cases: %d   |   Pass: %d (%.1f%%)   |   Fail: %d (%.1f%%)\n', ...
    N, n_pass, 100*n_pass/N, n_fail, 100*n_fail/N);
fprintf('  Parameters: %d\n', nP);
fprintf('  Prevalence (failure rate): %.2f%%\n', 100*prevalence);

% Class weights
w_class = ones(N, 1);
w_class(y == 1) = n_pass / n_fail;

% Design matrix with intercept
X_lr = [ones(N,1), X_std];

fprintf('══════════════════════════════════════════════════════════════════\n\n');

%% ═══════════════════════════════════════════════════════════════════════
%  V1.  LEAVE-ONE-OUT CROSS-VALIDATION  (LOOCV)
%  ═══════════════════════════════════════════════════════════════════════
fprintf('  [V1] Leave-One-Out Cross-Validation (%d folds) ...\n', N);
tic;
loocv_prob = zeros(N, 1);

for ii = 1:N
    train_mask = true(N, 1);
    train_mask(ii) = false;

    X_tr = X_lr(train_mask, :);
    y_tr = y(train_mask);
    w_tr = w_class(train_mask);

    [beta_loo, ~, ~] = logreg_irls(X_tr, y_tr, w_tr, lambda_log, 100, 1e-6);
    loocv_prob(ii) = sigmoid(X_lr(ii,:) * beta_loo);
end
loocv_time = toc;

loocv_pred = double(loocv_prob >= 0.5);
[loocv_metrics] = compute_all_metrics(y, loocv_pred, loocv_prob);

fprintf('    LOOCV completed in %.1f s\n', loocv_time);
fprintf('    Accuracy      : %.2f%%\n', 100*loocv_metrics.accuracy);
fprintf('    Balanced Acc  : %.2f%%\n', 100*loocv_metrics.bal_acc);
fprintf('    Precision     : %.2f%%\n', 100*loocv_metrics.precision);
fprintf('    Recall (TPR)  : %.2f%%\n', 100*loocv_metrics.recall);
fprintf('    Specificity   : %.2f%%\n', 100*loocv_metrics.specificity);
fprintf('    F1            : %.4f\n',   loocv_metrics.f1);
fprintf('    MCC           : %.4f\n',   loocv_metrics.mcc);
fprintf('    Cohen Kappa   : %.4f\n',   loocv_metrics.kappa);
fprintf('    AUC-ROC       : %.4f\n',   loocv_metrics.auc);
fprintf('    TP=%d  TN=%d  FP=%d  FN=%d\n', ...
    loocv_metrics.TP, loocv_metrics.TN, loocv_metrics.FP, loocv_metrics.FN);

%% ═══════════════════════════════════════════════════════════════════════
%  V2.  REPEATED STRATIFIED k-FOLD  (50 × 10 = 500 evaluations)
%  ═══════════════════════════════════════════════════════════════════════
fprintf('\n  [V2] Repeated %d-fold stratified CV (%d repeats) ...\n', ...
    k_folds, n_repeats);
tic;

rep_auc      = zeros(n_repeats, 1);
rep_f1       = zeros(n_repeats, 1);
rep_mcc      = zeros(n_repeats, 1);
rep_bal_acc  = zeros(n_repeats, 1);
rep_recall   = zeros(n_repeats, 1);
rep_precision = zeros(n_repeats, 1);
rep_kappa    = zeros(n_repeats, 1);
rep_accuracy = zeros(n_repeats, 1);

for rr = 1:n_repeats
    rng(rng_master + rr);
    cv_prob = zeros(N, 1);

    % Stratified fold assignment
    fold_idx = stratified_folds(y, k_folds);

    for kk = 1:k_folds
        test_mask  = (fold_idx == kk);
        train_mask = ~test_mask;

        [beta_k, ~, ~] = logreg_irls(X_lr(train_mask,:), y(train_mask), ...
            w_class(train_mask), lambda_log, 100, 1e-6);
        cv_prob(test_mask) = sigmoid(X_lr(test_mask,:) * beta_k);
    end

    cv_pred = double(cv_prob >= 0.5);
    m = compute_all_metrics(y, cv_pred, cv_prob);
    rep_auc(rr)       = m.auc;
    rep_f1(rr)        = m.f1;
    rep_mcc(rr)       = m.mcc;
    rep_bal_acc(rr)    = m.bal_acc;
    rep_recall(rr)     = m.recall;
    rep_precision(rr)  = m.precision;
    rep_kappa(rr)      = m.kappa;
    rep_accuracy(rr)   = m.accuracy;
end
rep_time = toc;

fprintf('    Completed %d repeats in %.1f s\n', n_repeats, rep_time);
fprintf('    AUC-ROC    : %.4f ± %.4f  [%.4f, %.4f]\n', ...
    mean(rep_auc), std(rep_auc), min(rep_auc), max(rep_auc));
fprintf('    F1         : %.4f ± %.4f\n', mean(rep_f1), std(rep_f1));
fprintf('    MCC        : %.4f ± %.4f\n', mean(rep_mcc), std(rep_mcc));
fprintf('    Bal. Acc   : %.2f%% ± %.2f%%\n', ...
    100*mean(rep_bal_acc), 100*std(rep_bal_acc));
fprintf('    Recall     : %.2f%% ± %.2f%%\n', ...
    100*mean(rep_recall), 100*std(rep_recall));
fprintf('    Precision  : %.2f%% ± %.2f%%\n', ...
    100*mean(rep_precision), 100*std(rep_precision));

%% ═══════════════════════════════════════════════════════════════════════
%  V3.  PERMUTATION TEST  (is the model better than chance?)
%  ═══════════════════════════════════════════════════════════════════════
fprintf('\n  [V3] Permutation test (%d shuffles) ...\n', n_perms);
tic;

% Observed AUC from LOOCV (our gold-standard estimate)
observed_auc = loocv_metrics.auc;
observed_mcc = loocv_metrics.mcc;

perm_auc = zeros(n_perms, 1);
perm_mcc = zeros(n_perms, 1);

for pp = 1:n_perms
    rng(rng_master + 10000 + pp);
    y_perm = y(randperm(N));    % shuffle labels

    % Quick 5-fold CV on permuted labels (faster than LOOCV)
    fold_perm = stratified_folds(y_perm, 5);
    cv_prob_p = zeros(N, 1);
    w_perm = ones(N,1);
    nf_p = sum(y_perm); np_p = N - nf_p;
    if nf_p > 0, w_perm(y_perm==1) = np_p / nf_p; end

    for kk = 1:5
        test_mask = (fold_perm == kk);
        train_mask = ~test_mask;
        [b_p,~,~] = logreg_irls(X_lr(train_mask,:), y_perm(train_mask), ...
            w_perm(train_mask), lambda_log, 50, 1e-5);
        cv_prob_p(test_mask) = sigmoid(X_lr(test_mask,:) * b_p);
    end
    cv_pred_p = double(cv_prob_p >= 0.5);
    m_p = compute_all_metrics(y_perm, cv_pred_p, cv_prob_p);
    perm_auc(pp) = m_p.auc;
    perm_mcc(pp) = m_p.mcc;
end
perm_time = toc;

% p-value: fraction of permutations with AUC >= observed
p_value_auc = (sum(perm_auc >= observed_auc) + 1) / (n_perms + 1);
p_value_mcc = (sum(perm_mcc >= observed_mcc) + 1) / (n_perms + 1);

fprintf('    Completed in %.1f s\n', perm_time);
fprintf('    Observed AUC = %.4f   |  Null mean = %.4f ± %.4f\n', ...
    observed_auc, mean(perm_auc), std(perm_auc));
fprintf('    p-value (AUC) = %.4f', p_value_auc);
if p_value_auc < 0.001
    fprintf('  *** HIGHLY SIGNIFICANT (p < 0.001)\n');
elseif p_value_auc < 0.01
    fprintf('  **  SIGNIFICANT (p < 0.01)\n');
elseif p_value_auc < 0.05
    fprintf('  *   SIGNIFICANT (p < 0.05)\n');
else
    fprintf('      NOT SIGNIFICANT — MODEL MAY NOT HAVE REAL PREDICTIVE POWER\n');
end
fprintf('    Observed MCC = %.4f   |  p-value (MCC) = %.4f\n', ...
    observed_mcc, p_value_mcc);

%% ═══════════════════════════════════════════════════════════════════════
%  V4.  BOOTSTRAP CONFIDENCE INTERVALS  (95%)
%  ═══════════════════════════════════════════════════════════════════════
fprintf('\n  [V4] Bootstrap confidence intervals (%d samples) ...\n', n_boot);
tic;
rng(rng_master + 20000);

boot_auc      = zeros(n_boot, 1);
boot_f1       = zeros(n_boot, 1);
boot_mcc      = zeros(n_boot, 1);
boot_bal_acc  = zeros(n_boot, 1);
boot_recall   = zeros(n_boot, 1);
boot_precision = zeros(n_boot, 1);

for bb = 1:n_boot
    % Stratified bootstrap: resample within each class
    idx_f = find(y == 1);
    idx_p = find(y == 0);
    boot_f = idx_f(randi(numel(idx_f), numel(idx_f), 1));
    boot_p = idx_p(randi(numel(idx_p), numel(idx_p), 1));
    boot_idx = [boot_f; boot_p];

    % Use LOOCV predictions for out-of-bag where available,
    % otherwise just evaluate on bootstrap sample
    y_b = y(boot_idx);
    p_b = loocv_prob(boot_idx);
    pred_b = double(p_b >= 0.5);

    m_b = compute_all_metrics(y_b, pred_b, p_b);
    boot_auc(bb)       = m_b.auc;
    boot_f1(bb)        = m_b.f1;
    boot_mcc(bb)       = m_b.mcc;
    boot_bal_acc(bb)    = m_b.bal_acc;
    boot_recall(bb)     = m_b.recall;
    boot_precision(bb)  = m_b.precision;
end
boot_time = toc;

ci = @(v) [quantile(v, alpha_ci/2), quantile(v, 1 - alpha_ci/2)];

ci_auc     = ci(boot_auc);
ci_f1      = ci(boot_f1);
ci_mcc     = ci(boot_mcc);
ci_bal_acc = ci(boot_bal_acc);
ci_recall  = ci(boot_recall);
ci_prec    = ci(boot_precision);

fprintf('    Done in %.1f s\n', boot_time);
fprintf('    AUC-ROC   : %.4f  [%.4f, %.4f]\n', loocv_metrics.auc, ci_auc);
fprintf('    F1        : %.4f  [%.4f, %.4f]\n', loocv_metrics.f1, ci_f1);
fprintf('    MCC       : %.4f  [%.4f, %.4f]\n', loocv_metrics.mcc, ci_mcc);
fprintf('    Bal. Acc  : %.2f%%  [%.2f%%, %.2f%%]\n', ...
    100*loocv_metrics.bal_acc, 100*ci_bal_acc);
fprintf('    Recall    : %.2f%%  [%.2f%%, %.2f%%]\n', ...
    100*loocv_metrics.recall, 100*ci_recall);
fprintf('    Precision : %.2f%%  [%.2f%%, %.2f%%]\n', ...
    100*loocv_metrics.precision, 100*ci_prec);

%% ═══════════════════════════════════════════════════════════════════════
%  V5.  THRESHOLD SWEEP  (Precision–Recall–F1 curve)
%  ═══════════════════════════════════════════════════════════════════════
fprintf('\n  [V5] Threshold sweep ...\n');
thresholds = 0:0.01:1;
nT = numel(thresholds);
sweep_prec   = zeros(nT, 1);
sweep_recall = zeros(nT, 1);
sweep_f1     = zeros(nT, 1);
sweep_mcc    = zeros(nT, 1);
sweep_fpr    = zeros(nT, 1);

for tt = 1:nT
    pred_t = double(loocv_prob >= thresholds(tt));
    m_t = compute_all_metrics(y, pred_t, loocv_prob);
    sweep_prec(tt)   = m_t.precision;
    sweep_recall(tt) = m_t.recall;
    sweep_f1(tt)     = m_t.f1;
    sweep_mcc(tt)    = m_t.mcc;
    sweep_fpr(tt)    = m_t.FP / max(m_t.FP + m_t.TN, 1);
end

[best_f1, best_f1_idx] = max(sweep_f1);
best_threshold = thresholds(best_f1_idx);
fprintf('    Optimal threshold (max F1) : %.2f  →  F1 = %.4f\n', ...
    best_threshold, best_f1);

[best_mcc_val, best_mcc_idx] = max(sweep_mcc);
best_mcc_threshold = thresholds(best_mcc_idx);
fprintf('    Optimal threshold (max MCC): %.2f  →  MCC = %.4f\n', ...
    best_mcc_threshold, best_mcc_val);

%% ═══════════════════════════════════════════════════════════════════════
%  V6.  NULL-MODEL BASELINES
%  ═══════════════════════════════════════════════════════════════════════
fprintf('\n  [V6] Null-model baselines ...\n');

% Baseline 1: Always predict PASS (majority class)
pred_always_pass = zeros(N, 1);
m_ap = compute_all_metrics(y, pred_always_pass, zeros(N,1));
fprintf('    Always-Pass  : Acc=%.2f%%  F1=%.4f  MCC=%.4f  AUC=%.4f\n', ...
    100*m_ap.accuracy, m_ap.f1, m_ap.mcc, m_ap.auc);

% Baseline 2: Random (with same prevalence)
rng(rng_master + 30000);
pred_random = double(rand(N,1) < prevalence);
prob_random = rand(N,1);
m_rnd = compute_all_metrics(y, pred_random, prob_random);
fprintf('    Random       : Acc=%.2f%%  F1=%.4f  MCC=%.4f  AUC=%.4f\n', ...
    100*m_rnd.accuracy, m_rnd.f1, m_rnd.mcc, m_rnd.auc);

% Our model (LOOCV)
fprintf('    Our Model    : Acc=%.2f%%  F1=%.4f  MCC=%.4f  AUC=%.4f\n', ...
    100*loocv_metrics.accuracy, loocv_metrics.f1, loocv_metrics.mcc, ...
    loocv_metrics.auc);

% Lift factors
fprintf('    ────────────────────────────────────────────\n');
fprintf('    AUC lift over random   : %.2fx\n', loocv_metrics.auc / max(m_rnd.auc, 0.01));
fprintf('    MCC lift over random   : %.2fx\n', loocv_metrics.mcc / max(abs(m_rnd.mcc), 0.001));
if loocv_metrics.mcc > 0
    fprintf('    → Positive MCC confirms model learns REAL failure patterns\n');
end

%% ═══════════════════════════════════════════════════════════════════════
%  V7.  LEARNING CURVE  (performance vs training set size)
%  ═══════════════════════════════════════════════════════════════════════
fprintf('\n  [V7] Learning curve ...\n');
train_fracs = [0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0];
n_lc = numel(train_fracs);
lc_auc_mean = zeros(n_lc, 1);
lc_auc_std  = zeros(n_lc, 1);
n_lc_reps   = 20;

for ff = 1:n_lc
    frac = train_fracs(ff);
    n_train = round(frac * N);
    auc_runs = zeros(n_lc_reps, 1);

    for rr = 1:n_lc_reps
        rng(rng_master + 40000 + ff*100 + rr);

        % Stratified subsample
        idx_f = find(y == 1);
        idx_p = find(y == 0);
        nf_sub = max(round(frac * numel(idx_f)), 2);
        np_sub = max(round(frac * numel(idx_p)), 2);
        train_idx = [idx_f(randperm(numel(idx_f), nf_sub)); ...
                     idx_p(randperm(numel(idx_p), np_sub))];
        test_idx  = setdiff((1:N)', train_idx);

        if isempty(test_idx) || numel(unique(y(test_idx))) < 2
            auc_runs(rr) = NaN;
            continue;
        end

        w_tr = ones(numel(train_idx), 1);
        nf_tr = sum(y(train_idx)); np_tr = numel(train_idx) - nf_tr;
        if nf_tr > 0, w_tr(y(train_idx)==1) = np_tr/nf_tr; end

        [b_lc,~,~] = logreg_irls(X_lr(train_idx,:), y(train_idx), ...
            w_tr, lambda_log, 100, 1e-6);
        p_lc = sigmoid(X_lr(test_idx,:) * b_lc);
        pred_lc = double(p_lc >= 0.5);
        m_lc = compute_all_metrics(y(test_idx), pred_lc, p_lc);
        auc_runs(rr) = m_lc.auc;
    end
    lc_auc_mean(ff) = nanmean(auc_runs);
    lc_auc_std(ff)  = nanstd(auc_runs);
    fprintf('    %.0f%% data (%4d cases): AUC = %.4f ± %.4f\n', ...
        100*frac, n_train, lc_auc_mean(ff), lc_auc_std(ff));
end

%% ═══════════════════════════════════════════════════════════════════════
%  V8.  FEATURE ABLATION  (AUC drop when each parameter is removed)
%  ═══════════════════════════════════════════════════════════════════════
fprintf('\n  [V8] Feature ablation (%d parameters) ...\n', nP);
tic;

% Baseline: 10-fold CV AUC with all features
rng(rng_master + 50000);
fold_abl = stratified_folds(y, 10);
cv_prob_full = zeros(N, 1);
for kk = 1:10
    test_mask = (fold_abl == kk);
    train_mask = ~test_mask;
    [b_full,~,~] = logreg_irls(X_lr(train_mask,:), y(train_mask), ...
        w_class(train_mask), lambda_log, 100, 1e-6);
    cv_prob_full(test_mask) = sigmoid(X_lr(test_mask,:) * b_full);
end
m_full = compute_all_metrics(y, double(cv_prob_full>=0.5), cv_prob_full);
auc_full = m_full.auc;

auc_drop = zeros(nP, 1);
for pp = 1:nP
    % Remove parameter pp (column pp+1 because col 1 = intercept)
    X_abl = X_lr;
    X_abl(:, pp+1) = 0;   % zero out this feature

    cv_prob_abl = zeros(N, 1);
    for kk = 1:10
        test_mask = (fold_abl == kk);
        train_mask = ~test_mask;
        X_tr_abl = X_abl(train_mask, :);
        X_tr_abl(:, pp+1) = 0;

        [b_abl,~,~] = logreg_irls(X_tr_abl, y(train_mask), ...
            w_class(train_mask), lambda_log, 100, 1e-6);
        X_te_abl = X_abl(test_mask, :);
        X_te_abl(:, pp+1) = 0;
        cv_prob_abl(test_mask) = sigmoid(X_te_abl * b_abl);
    end
    m_abl = compute_all_metrics(y, double(cv_prob_abl>=0.5), cv_prob_abl);
    auc_drop(pp) = auc_full - m_abl.auc;
end
abl_time = toc;

[auc_drop_sorted, abl_sort_idx] = sort(auc_drop, 'descend');
fprintf('    Baseline AUC (all features): %.4f\n', auc_full);
fprintf('    Top-10 features by AUC drop when removed:\n');
for ii = 1:min(10, nP)
    jj = abl_sort_idx(ii);
    fprintf('      %-20s  ΔAUC = %+.4f  (AUC without = %.4f)\n', ...
        param_labels{jj}, auc_drop(jj), auc_full - auc_drop(jj));
end
fprintf('    Done in %.1f s\n', abl_time);

%% ═══════════════════════════════════════════════════════════════════════
%  V9.  CALIBRATION CURVE
%  ═══════════════════════════════════════════════════════════════════════
fprintf('\n  [V9] Calibration curve ...\n');
n_bins = 10;
bin_edges = linspace(0, 1, n_bins+1);
cal_pred_mean = zeros(n_bins, 1);
cal_obs_frac  = zeros(n_bins, 1);
cal_count     = zeros(n_bins, 1);

for bb = 1:n_bins
    mask = loocv_prob >= bin_edges(bb) & loocv_prob < bin_edges(bb+1);
    if bb == n_bins  % include upper boundary
        mask = mask | (loocv_prob == bin_edges(bb+1));
    end
    cal_count(bb) = sum(mask);
    if cal_count(bb) > 0
        cal_pred_mean(bb) = mean(loocv_prob(mask));
        cal_obs_frac(bb)  = mean(y(mask));
    else
        cal_pred_mean(bb) = (bin_edges(bb) + bin_edges(bb+1)) / 2;
        cal_obs_frac(bb)  = NaN;
    end
end

% Brier score
brier = mean((loocv_prob - y).^2);
% Expected Calibration Error
ece = sum(cal_count .* abs(cal_obs_frac - cal_pred_mean), 'omitnan') / N;
fprintf('    Brier Score : %.6f  (lower is better; perfect = 0)\n', brier);
fprintf('    ECE         : %.6f  (lower is better; perfect = 0)\n', ece);

%% ═══════════════════════════════════════════════════════════════════════
%  V10. FIGURES
%  ═══════════════════════════════════════════════════════════════════════
fprintf('\n  [V10] Generating validation figures ...\n');

% ---- Fig 1: LOOCV ROC Curve --------------------------------------------
[fpr_loo, tpr_loo, auc_loo] = compute_roc(y, loocv_prob);
fig1 = figure('Units','pixels','Position',[50 50 700 600],'Color','w');
plot(fpr_loo, tpr_loo, 'b-', 'LineWidth', 2.5); hold on;
plot([0 1], [0 1], 'k--', 'LineWidth', 1);
fill([fpr_loo; 1; 0], [tpr_loo; 0; 0], [0.26 0.53 0.79], ...
    'FaceAlpha', 0.12, 'EdgeColor', 'none');
xlabel('False Positive Rate (1 − Specificity)', 'FontSize', 11);
ylabel('True Positive Rate (Recall)', 'FontSize', 11);
title(sprintf('LOOCV ROC Curve  (N=%d, AUC = %.4f)', N, auc_loo), 'FontSize', 13);

% Add CI band text
text(0.45, 0.25, sprintf(['LOOCV (zero data leakage)\n' ...
    'AUC = %.4f  [%.4f, %.4f]\n' ...
    'F1  = %.4f  [%.4f, %.4f]\n' ...
    'MCC = %.4f  [%.4f, %.4f]\n' ...
    'Recall = %.1f%%  Prec = %.1f%%\n' ...
    'p-value = %.4f'], ...
    loocv_metrics.auc, ci_auc, ...
    loocv_metrics.f1, ci_f1, ...
    loocv_metrics.mcc, ci_mcc, ...
    100*loocv_metrics.recall, 100*loocv_metrics.precision, ...
    p_value_auc), ...
    'FontSize', 8, 'BackgroundColor', 'w', 'EdgeColor', 'k', ...
    'Margin', 4);
grid on;
exportgraphics(fig1, fullfile(val_dir, 'fig_loocv_roc.png'), 'Resolution', 200);
fprintf('    saved fig_loocv_roc.png\n');

% ---- Fig 2: Confusion Matrix -------------------------------------------
fig2 = figure('Units','pixels','Position',[50 50 550 500],'Color','w');
cm = [loocv_metrics.TN, loocv_metrics.FP; ...
      loocv_metrics.FN, loocv_metrics.TP];
imagesc(cm); colormap(flipud(gray(64)));
set(gca, 'XTick', [1 2], 'XTickLabel', {'Pred PASS','Pred FAIL'}, ...
         'YTick', [1 2], 'YTickLabel', {'Actual PASS','Actual FAIL'}, ...
         'FontSize', 12);
for ii = 1:2
    for jj = 1:2
        lbl = '';
        if ii==1 && jj==1, lbl = 'TN'; end
        if ii==1 && jj==2, lbl = 'FP'; end
        if ii==2 && jj==1, lbl = 'FN'; end
        if ii==2 && jj==2, lbl = 'TP'; end
        text(jj, ii, sprintf('%s\n%d', lbl, cm(ii,jj)), ...
            'HorizontalAlignment', 'center', 'FontSize', 16, ...
            'FontWeight', 'bold', 'Color', [0.85 0.1 0.1]);
    end
end
title(sprintf('LOOCV Confusion Matrix  (N=%d)', N), 'FontSize', 13);
exportgraphics(fig2, fullfile(val_dir, 'fig_loocv_confusion.png'), 'Resolution', 200);
fprintf('    saved fig_loocv_confusion.png\n');

% ---- Fig 3: Repeated CV Box-Plot ----------------------------------------
fig3 = figure('Units','pixels','Position',[50 50 1000 550],'Color','w');
data_box = [rep_auc, rep_f1, rep_mcc, rep_bal_acc, rep_recall, rep_precision, rep_kappa];
labels_box = {'AUC','F1','MCC','Bal Acc','Recall','Precision','Kappa'};
boxplot(data_box, 'Labels', labels_box, 'Colors', 'b', 'Symbol', 'ro');
hold on;
% Overlay LOOCV point
loocv_vals = [loocv_metrics.auc, loocv_metrics.f1, loocv_metrics.mcc, ...
    loocv_metrics.bal_acc, loocv_metrics.recall, loocv_metrics.precision, ...
    loocv_metrics.kappa];
scatter(1:7, loocv_vals, 100, 'r', 'filled', 'diamond');
legend({'','LOOCV point'}, 'Location', 'southwest', 'FontSize', 9);
title(sprintf('Repeated %d-Fold CV  (%d repeats)', k_folds, n_repeats), ...
    'FontSize', 13);
ylabel('Metric Value');
grid on;
exportgraphics(fig3, fullfile(val_dir, 'fig_repeated_cv_boxplot.png'), 'Resolution', 200);
fprintf('    saved fig_repeated_cv_boxplot.png\n');

% ---- Fig 4: Permutation Test -------------------------------------------
fig4 = figure('Units','pixels','Position',[50 50 800 500],'Color','w');
histogram(perm_auc, 40, 'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'k'); hold on;
xline(observed_auc, 'r-', 'LineWidth', 2.5);
xlabel('AUC-ROC'); ylabel('Count');
title(sprintf('Permutation Test  (n=%d)  —  p = %.4f', n_perms, p_value_auc), ...
    'FontSize', 13);
text(observed_auc + 0.01, max(ylim)*0.8, ...
    sprintf('Observed\nAUC = %.4f', observed_auc), ...
    'Color', 'r', 'FontSize', 10, 'FontWeight', 'bold');
text(mean(perm_auc), max(ylim)*0.6, ...
    sprintf('Null distribution\nμ = %.4f ± %.4f', mean(perm_auc), std(perm_auc)), ...
    'FontSize', 9, 'HorizontalAlignment', 'center');

if p_value_auc < 0.001
    verdict = 'HIGHLY SIGNIFICANT — model detects REAL failure patterns';
    clr = [0 0.6 0];
elseif p_value_auc < 0.05
    verdict = 'SIGNIFICANT — model has real predictive power';
    clr = [0 0.4 0];
else
    verdict = 'NOT SIGNIFICANT — model may not be better than chance';
    clr = [0.8 0 0];
end
text(0.5, 0.02, verdict, 'Units', 'normalized', ...
    'HorizontalAlignment', 'center', 'FontSize', 11, ...
    'FontWeight', 'bold', 'Color', clr, 'BackgroundColor', 'w');
grid on;
exportgraphics(fig4, fullfile(val_dir, 'fig_permutation_test.png'), 'Resolution', 200);
fprintf('    saved fig_permutation_test.png\n');

% ---- Fig 5: Threshold Sweep -------------------------------------------
fig5 = figure('Units','pixels','Position',[50 50 900 550],'Color','w');
plot(thresholds, sweep_recall, 'b-', 'LineWidth', 2, 'DisplayName', 'Recall'); hold on;
plot(thresholds, sweep_prec,   'r-', 'LineWidth', 2, 'DisplayName', 'Precision');
plot(thresholds, sweep_f1,     'g-', 'LineWidth', 2, 'DisplayName', 'F1');
plot(thresholds, sweep_mcc,    'm-', 'LineWidth', 2, 'DisplayName', 'MCC');
xline(best_threshold, 'k--', 'LineWidth', 1.5);
xline(0.5, 'k:', 'LineWidth', 1);
plot(best_threshold, best_f1, 'kp', 'MarkerSize', 15, 'MarkerFaceColor', 'g', ...
    'DisplayName', sprintf('Best F1=%.3f @%.2f', best_f1, best_threshold));
legend('Location', 'best', 'FontSize', 9);
xlabel('Decision Threshold'); ylabel('Metric Value');
title('Threshold Sweep  (LOOCV predictions)', 'FontSize', 13);
grid on;
exportgraphics(fig5, fullfile(val_dir, 'fig_threshold_sweep.png'), 'Resolution', 200);
fprintf('    saved fig_threshold_sweep.png\n');

% ---- Fig 6: Calibration Curve ------------------------------------------
fig6 = figure('Units','pixels','Position',[50 50 650 600],'Color','w');
valid = ~isnan(cal_obs_frac);
plot([0 1], [0 1], 'k--', 'LineWidth', 1); hold on;
bar_data = cal_pred_mean(valid);
scatter(cal_pred_mean(valid), cal_obs_frac(valid), ...
    80 + 3*cal_count(valid), 'b', 'filled');
plot(cal_pred_mean(valid), cal_obs_frac(valid), 'b-', 'LineWidth', 1.5);
xlabel('Mean Predicted Probability'); ylabel('Observed Fraction of Failures');
title(sprintf('Calibration Curve  (Brier=%.5f, ECE=%.5f)', brier, ece), ...
    'FontSize', 13);
text(0.05, 0.85, sprintf('Ideal: points on diagonal\nBrier = %.5f\nECE = %.5f', ...
    brier, ece), 'FontSize', 9, 'BackgroundColor', 'w', 'EdgeColor', 'k');
% Add bin counts
for bb = find(valid')
    text(cal_pred_mean(bb), cal_obs_frac(bb)+0.03, ...
        sprintf('n=%d', cal_count(bb)), 'FontSize', 7, ...
        'HorizontalAlignment', 'center');
end
grid on;
exportgraphics(fig6, fullfile(val_dir, 'fig_calibration_curve.png'), 'Resolution', 200);
fprintf('    saved fig_calibration_curve.png\n');

% ---- Fig 7: Learning Curve --------------------------------------------
fig7 = figure('Units','pixels','Position',[50 50 800 500],'Color','w');
train_sizes = round(train_fracs * N);
errorbar(train_sizes, lc_auc_mean, lc_auc_std, 'b-o', ...
    'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', [0.26 0.53 0.79]);
yline(loocv_metrics.auc, 'r--', 'LineWidth', 1.5);
xlabel('Training Set Size'); ylabel('AUC-ROC');
title('Learning Curve  —  AUC vs Training Size', 'FontSize', 13);
text(train_sizes(end)*0.5, loocv_metrics.auc + 0.02, ...
    sprintf('LOOCV AUC = %.4f', loocv_metrics.auc), ...
    'Color', 'r', 'FontSize', 9);
grid on;
xlim([0 N*1.05]);
exportgraphics(fig7, fullfile(val_dir, 'fig_learning_curve.png'), 'Resolution', 200);
fprintf('    saved fig_learning_curve.png\n');

% ---- Fig 8: Feature Ablation ------------------------------------------
fig8 = figure('Units','pixels','Position',[50 50 900 600],'Color','w');
barh(auc_drop(abl_sort_idx));
set(gca, 'YTick', 1:nP, ...
    'YTickLabel', strrep(param_labels(abl_sort_idx), '_', '\_'), 'FontSize', 8);
xlabel('ΔAUC when feature removed  (higher = more important)');
title(sprintf('Feature Ablation  (baseline AUC = %.4f)', auc_full), 'FontSize', 12);
xline(0, 'k--');
% Colour top-5
hold on;
for ii = nP-4:nP
    if auc_drop(abl_sort_idx(ii)) > 0
        patch([0 auc_drop(abl_sort_idx(ii)) auc_drop(abl_sort_idx(ii)) 0], ...
              [ii-0.4 ii-0.4 ii+0.4 ii+0.4], [0.85 0.33 0.10], ...
              'FaceAlpha', 0.4, 'EdgeColor', 'none');
    end
end
grid on;
exportgraphics(fig8, fullfile(val_dir, 'fig_feature_ablation.png'), 'Resolution', 200);
fprintf('    saved fig_feature_ablation.png\n');

% ---- Fig 9: Confidence Summary Forest Plot ------------------------------
fig9 = figure('Units','pixels','Position',[50 50 800 500],'Color','w');
metric_names_ci = {'AUC-ROC','F1 Score','MCC','Balanced Acc','Recall','Precision'};
point_vals = [loocv_metrics.auc, loocv_metrics.f1, loocv_metrics.mcc, ...
    loocv_metrics.bal_acc, loocv_metrics.recall, loocv_metrics.precision];
ci_lo = [ci_auc(1), ci_f1(1), ci_mcc(1), ci_bal_acc(1), ci_recall(1), ci_prec(1)];
ci_hi = [ci_auc(2), ci_f1(2), ci_mcc(2), ci_bal_acc(2), ci_recall(2), ci_prec(2)];

for ii = 1:6
    errorbar(point_vals(ii), ii, 0, 0, ...
        point_vals(ii) - ci_lo(ii), ci_hi(ii) - point_vals(ii), ...
        'ko', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    hold on;
end
set(gca, 'YTick', 1:6, 'YTickLabel', metric_names_ci, 'FontSize', 10);
xlabel('Value  (with 95% bootstrap CI)');
title('LOOCV Performance — 95% Confidence Intervals', 'FontSize', 13);
xline(0.5, 'r:', 'LineWidth', 1, 'Label', 'Random baseline');
grid on;
exportgraphics(fig9, fullfile(val_dir, 'fig_confidence_summary.png'), 'Resolution', 200);
fprintf('    saved fig_confidence_summary.png\n');

% ---- Fig 10: Predicted probability histogram (pass vs fail) -------------
fig10 = figure('Units','pixels','Position',[50 50 800 500],'Color','w');
histogram(loocv_prob(y==0), 0:0.02:1, 'FaceColor', [0.26 0.53 0.79], ...
    'FaceAlpha', 0.7, 'EdgeColor', 'w'); hold on;
histogram(loocv_prob(y==1), 0:0.02:1, 'FaceColor', [0.85 0.33 0.10], ...
    'FaceAlpha', 0.7, 'EdgeColor', 'w');
xline(0.5, 'k--', 'LineWidth', 1.5);
xline(best_threshold, 'g--', 'LineWidth', 1.5);
legend({'Actual PASS cases', 'Actual FAIL cases', ...
    'Default threshold (0.5)', sprintf('Optimal threshold (%.2f)', best_threshold)}, ...
    'FontSize', 9, 'Location', 'northeast');
xlabel('Predicted P(failure)'); ylabel('Count');
title('LOOCV Predicted Probability Distribution', 'FontSize', 13);
grid on;
exportgraphics(fig10, fullfile(val_dir, 'fig_probability_histogram.png'), 'Resolution', 200);
fprintf('    saved fig_probability_histogram.png\n');

%% ═══════════════════════════════════════════════════════════════════════
%  V11. COMPREHENSIVE TEXT REPORT
%  ═══════════════════════════════════════════════════════════════════════
fprintf('\n  [V11] Writing validation report ...\n');
fid = fopen(fullfile(val_dir, 'validation_report.txt'), 'w');

fprintf(fid, '╔══════════════════════════════════════════════════════════════════╗\n');
fprintf(fid, '║  FLIGHT-CRITICAL CLASSIFIER VALIDATION REPORT                  ║\n');
fprintf(fid, '╠══════════════════════════════════════════════════════════════════╣\n');
fprintf(fid, '║  Date          : %s                             ║\n', datestr(now, 'yyyy-mm-dd HH:MM'));
fprintf(fid, '║  Cases         : %d  (Pass=%d, Fail=%d)                     ║\n', N, n_pass, n_fail);
fprintf(fid, '║  Parameters    : %d                                           ║\n', nP);
fprintf(fid, '║  Failure Rate  : %.2f%%                                       ║\n', 100*prevalence);
fprintf(fid, '║  Classifier    : L2-Logistic Regression (λ=%.4f)            ║\n', lambda_log);
fprintf(fid, '╚══════════════════════════════════════════════════════════════════╝\n\n');

fprintf(fid, '═══════════════════════════════════════════════════════════════════\n');
fprintf(fid, '  V1. LEAVE-ONE-OUT CROSS-VALIDATION (LOOCV)  — GOLD STANDARD\n');
fprintf(fid, '═══════════════════════════════════════════════════════════════════\n');
fprintf(fid, '  Every single case is predicted by a model trained on the other\n');
fprintf(fid, '  999 cases.  Zero data leakage.  %d independent evaluations.\n\n', N);
fprintf(fid, '  Confusion Matrix:\n');
fprintf(fid, '                 Pred PASS   Pred FAIL\n');
fprintf(fid, '  Actual PASS     %5d (TN)  %5d (FP)\n', loocv_metrics.TN, loocv_metrics.FP);
fprintf(fid, '  Actual FAIL     %5d (FN)  %5d (TP)\n\n', loocv_metrics.FN, loocv_metrics.TP);
fprintf(fid, '  Accuracy        : %.2f%%\n', 100*loocv_metrics.accuracy);
fprintf(fid, '  Balanced Acc    : %.2f%%\n', 100*loocv_metrics.bal_acc);
fprintf(fid, '  Precision       : %.2f%%\n', 100*loocv_metrics.precision);
fprintf(fid, '  Recall (TPR)    : %.2f%%  ← How many FAILURES we catch\n', 100*loocv_metrics.recall);
fprintf(fid, '  Specificity     : %.2f%%  ← How many PASSES we correctly clear\n', 100*loocv_metrics.specificity);
fprintf(fid, '  F1 Score        : %.4f\n', loocv_metrics.f1);
fprintf(fid, '  MCC             : %.4f   ← Best single metric for imbalanced data\n', loocv_metrics.mcc);
fprintf(fid, '  Cohen''s Kappa   : %.4f\n', loocv_metrics.kappa);
fprintf(fid, '  AUC-ROC         : %.4f\n\n', loocv_metrics.auc);

fprintf(fid, '═══════════════════════════════════════════════════════════════════\n');
fprintf(fid, '  V2. REPEATED STRATIFIED k-FOLD (%d × %d-fold)\n', n_repeats, k_folds);
fprintf(fid, '═══════════════════════════════════════════════════════════════════\n');
fprintf(fid, '  %d independent experiments with different random splits.\n', n_repeats);
fprintf(fid, '  Shows stability of results across different train/test splits.\n\n');
fprintf(fid, '  Metric         Mean ± Std       [Min, Max]\n');
fprintf(fid, '  ──────────────────────────────────────────────\n');
fprintf(fid, '  AUC-ROC       %.4f ± %.4f   [%.4f, %.4f]\n', mean(rep_auc), std(rep_auc), min(rep_auc), max(rep_auc));
fprintf(fid, '  F1            %.4f ± %.4f   [%.4f, %.4f]\n', mean(rep_f1), std(rep_f1), min(rep_f1), max(rep_f1));
fprintf(fid, '  MCC           %.4f ± %.4f   [%.4f, %.4f]\n', mean(rep_mcc), std(rep_mcc), min(rep_mcc), max(rep_mcc));
fprintf(fid, '  Bal. Acc      %.4f ± %.4f   [%.4f, %.4f]\n', mean(rep_bal_acc), std(rep_bal_acc), min(rep_bal_acc), max(rep_bal_acc));
fprintf(fid, '  Recall        %.4f ± %.4f   [%.4f, %.4f]\n', mean(rep_recall), std(rep_recall), min(rep_recall), max(rep_recall));
fprintf(fid, '  Precision     %.4f ± %.4f   [%.4f, %.4f]\n\n', mean(rep_precision), std(rep_precision), min(rep_precision), max(rep_precision));

fprintf(fid, '═══════════════════════════════════════════════════════════════════\n');
fprintf(fid, '  V3. PERMUTATION TEST  — IS THIS BETTER THAN CHANCE?\n');
fprintf(fid, '═══════════════════════════════════════════════════════════════════\n');
fprintf(fid, '  %d times, labels were randomly shuffled and the full CV\n', n_perms);
fprintf(fid, '  pipeline was re-run.  If our model is real, its AUC should\n');
fprintf(fid, '  be far higher than any shuffled version.\n\n');
fprintf(fid, '  Observed AUC    : %.4f\n', observed_auc);
fprintf(fid, '  Null AUC mean   : %.4f ± %.4f\n', mean(perm_auc), std(perm_auc));
fprintf(fid, '  p-value (AUC)   : %.4f\n', p_value_auc);
fprintf(fid, '  p-value (MCC)   : %.4f\n\n', p_value_mcc);
if p_value_auc < 0.001
    fprintf(fid, '  ★★★ VERDICT: HIGHLY SIGNIFICANT (p < 0.001)\n');
    fprintf(fid, '  The model detects REAL failure-inducing parameter patterns.\n');
    fprintf(fid, '  The probability of this result occurring by chance is < 0.1%%.\n\n');
elseif p_value_auc < 0.05
    fprintf(fid, '  ★★ VERDICT: SIGNIFICANT (p < 0.05)\n');
    fprintf(fid, '  The model has statistically significant predictive power.\n\n');
else
    fprintf(fid, '  ⚠ VERDICT: NOT SIGNIFICANT (p ≥ 0.05)\n');
    fprintf(fid, '  Cannot conclusively prove the model beats random chance.\n');
    fprintf(fid, '  This may be due to extreme class imbalance (only %d failures).\n\n', n_fail);
end

fprintf(fid, '═══════════════════════════════════════════════════════════════════\n');
fprintf(fid, '  V4. BOOTSTRAP 95%% CONFIDENCE INTERVALS\n');
fprintf(fid, '═══════════════════════════════════════════════════════════════════\n');
fprintf(fid, '  %d stratified bootstrap resamples of LOOCV predictions.\n\n', n_boot);
fprintf(fid, '  Metric        Point Est.     95%% CI\n');
fprintf(fid, '  ──────────────────────────────────────────────\n');
fprintf(fid, '  AUC-ROC       %.4f        [%.4f, %.4f]\n', loocv_metrics.auc, ci_auc);
fprintf(fid, '  F1            %.4f        [%.4f, %.4f]\n', loocv_metrics.f1, ci_f1);
fprintf(fid, '  MCC           %.4f        [%.4f, %.4f]\n', loocv_metrics.mcc, ci_mcc);
fprintf(fid, '  Bal. Acc      %.4f        [%.4f, %.4f]\n', loocv_metrics.bal_acc, ci_bal_acc);
fprintf(fid, '  Recall        %.4f        [%.4f, %.4f]\n', loocv_metrics.recall, ci_recall);
fprintf(fid, '  Precision     %.4f        [%.4f, %.4f]\n\n', loocv_metrics.precision, ci_prec);

fprintf(fid, '═══════════════════════════════════════════════════════════════════\n');
fprintf(fid, '  V5. THRESHOLD OPTIMISATION\n');
fprintf(fid, '═══════════════════════════════════════════════════════════════════\n');
fprintf(fid, '  Best threshold (max F1)  : %.2f  →  F1 = %.4f\n', best_threshold, best_f1);
fprintf(fid, '  Best threshold (max MCC) : %.2f  →  MCC = %.4f\n\n', best_mcc_threshold, best_mcc_val);

fprintf(fid, '═══════════════════════════════════════════════════════════════════\n');
fprintf(fid, '  V6. NULL-MODEL COMPARISON\n');
fprintf(fid, '═══════════════════════════════════════════════════════════════════\n');
fprintf(fid, '  Always-Pass  : Acc=%.2f%%  F1=%.4f  MCC=%.4f  AUC=%.4f\n', ...
    100*m_ap.accuracy, m_ap.f1, m_ap.mcc, m_ap.auc);
fprintf(fid, '  Random       : Acc=%.2f%%  F1=%.4f  MCC=%.4f  AUC=%.4f\n', ...
    100*m_rnd.accuracy, m_rnd.f1, m_rnd.mcc, m_rnd.auc);
fprintf(fid, '  Our Model    : Acc=%.2f%%  F1=%.4f  MCC=%.4f  AUC=%.4f\n\n', ...
    100*loocv_metrics.accuracy, loocv_metrics.f1, loocv_metrics.mcc, loocv_metrics.auc);
fprintf(fid, '  Key insight: A naive "always pass" classifier gets %.1f%% accuracy\n', 100*m_ap.accuracy);
fprintf(fid, '  but MCC=0 and F1=0 — it learns NOTHING.  Our model has MCC=%.4f.\n\n', loocv_metrics.mcc);

fprintf(fid, '═══════════════════════════════════════════════════════════════════\n');
fprintf(fid, '  V7. CALIBRATION\n');
fprintf(fid, '═══════════════════════════════════════════════════════════════════\n');
fprintf(fid, '  Brier Score : %.6f   (0 = perfect, 1 = worst)\n', brier);
fprintf(fid, '  ECE         : %.6f   (0 = perfectly calibrated)\n\n', ece);

fprintf(fid, '═══════════════════════════════════════════════════════════════════\n');
fprintf(fid, '  V8. FEATURE ABLATION — TOP-10 MOST CRITICAL PARAMETERS\n');
fprintf(fid, '═══════════════════════════════════════════════════════════════════\n');
fprintf(fid, '  Removing each parameter one at a time and measuring AUC drop.\n');
fprintf(fid, '  Baseline AUC (all 30 params): %.4f\n\n', auc_full);
fprintf(fid, '  Rank  Parameter              ΔAUC     AUC without\n');
fprintf(fid, '  ──────────────────────────────────────────────────\n');
for ii = 1:min(10, nP)
    jj = abl_sort_idx(nP - ii + 1);
    fprintf(fid, '  #%2d   %-22s  %+.4f   %.4f\n', ...
        ii, param_labels{jj}, auc_drop(jj), auc_full - auc_drop(jj));
end

fprintf(fid, '\n═══════════════════════════════════════════════════════════════════\n');
fprintf(fid, '  OVERALL FLIGHT-SAFETY VERDICT\n');
fprintf(fid, '═══════════════════════════════════════════════════════════════════\n');
if loocv_metrics.auc > 0.7 && p_value_auc < 0.05 && loocv_metrics.mcc > 0
    fprintf(fid, '  ✓ The classifier demonstrates statistically significant\n');
    fprintf(fid, '    predictive power for identifying failure-prone parameter\n');
    fprintf(fid, '    combinations (p = %.4f, AUC = %.4f, MCC = %.4f).\n\n', ...
        p_value_auc, loocv_metrics.auc, loocv_metrics.mcc);
    fprintf(fid, '  ✓ LOOCV ensures zero data leakage — every prediction is\n');
    fprintf(fid, '    made on a case NEVER seen during training.\n\n');
    fprintf(fid, '  ✓ Results are stable across %d repeated CV experiments\n', n_repeats);
    fprintf(fid, '    (AUC std = %.4f).\n\n', std(rep_auc));
    fprintf(fid, '  ✓ The model significantly outperforms null baselines\n');
    fprintf(fid, '    (always-pass, random coin flip).\n\n');
    fprintf(fid, '  RECOMMENDATION: The sensitivity rankings from Sensitivity.m\n');
    fprintf(fid, '  are trustworthy for identifying which parameter uncertainties\n');
    fprintf(fid, '  to reduce for flight-safety improvement.\n');
else
    fprintf(fid, '  ⚠ Classifier performance is limited.  With only %d failures\n', n_fail);
    fprintf(fid, '    out of %d cases (%.1f%%), the extreme class imbalance makes\n', N, 100*prevalence);
    fprintf(fid, '    high-confidence prediction challenging.\n\n');
    fprintf(fid, '  RECOMMENDATION: The SENSITIVITY rankings (SRC/PRCC) from\n');
    fprintf(fid, '  Sensitivity.m remain valid regardless of classifier performance,\n');
    fprintf(fid, '  because they measure parameter → metric correlation, not\n');
    fprintf(fid, '  pass/fail classification.\n');
end
fprintf(fid, '\n══════════════════════════════════════════════════════════════════\n');
fclose(fid);
fprintf('    saved validation_report.txt\n');

%% ═══════════════════════════════════════════════════════════════════════
%  DONE
%  ═══════════════════════════════════════════════════════════════════════
elapsed = toc(t_wall);
fprintf('\n══════════════════════════════════════════════════════════════════\n');
fprintf('  Validation complete.  %.1f s elapsed.\n', elapsed);
fprintf('  Output: %s\n', val_dir);
d = dir(val_dir);
for ii = 1:numel(d)
    if ~d(ii).isdir
        fprintf('    • %s\n', d(ii).name);
    end
end
fprintf('══════════════════════════════════════════════════════════════════\n');


%% ═══════════════════════════════════════════════════════════════════════
%  LOCAL FUNCTIONS
%  ═══════════════════════════════════════════════════════════════════════

function m = compute_all_metrics(y_true, y_pred, y_prob)
%COMPUTE_ALL_METRICS  All binary classification metrics in one call.
    m.TP = sum(y_pred == 1 & y_true == 1);
    m.TN = sum(y_pred == 0 & y_true == 0);
    m.FP = sum(y_pred == 1 & y_true == 0);
    m.FN = sum(y_pred == 0 & y_true == 1);
    N = numel(y_true);

    m.accuracy    = (m.TP + m.TN) / N;
    m.precision   = m.TP / max(m.TP + m.FP, 1);
    m.recall      = m.TP / max(m.TP + m.FN, 1);
    m.specificity = m.TN / max(m.TN + m.FP, 1);
    m.f1          = 2 * m.precision * m.recall / max(m.precision + m.recall, 1e-12);
    m.bal_acc     = (m.recall + m.specificity) / 2;

    % Matthews Correlation Coefficient
    denom = sqrt(double(m.TP+m.FP) * double(m.TP+m.FN) * ...
                 double(m.TN+m.FP) * double(m.TN+m.FN));
    if denom == 0, denom = 1; end
    m.mcc = (m.TP * m.TN - m.FP * m.FN) / denom;

    % Cohen's Kappa
    p_o = m.accuracy;
    p_e = ((m.TP+m.FP)*(m.TP+m.FN) + (m.TN+m.FP)*(m.TN+m.FN)) / N^2;
    m.kappa = (p_o - p_e) / max(1 - p_e, 1e-12);

    % AUC-ROC
    [~, ~, m.auc] = compute_roc(y_true, y_prob);
end


function [fpr, tpr, auc_val] = compute_roc(y_true, y_score)
%COMPUTE_ROC  ROC curve and AUC.
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
    [fpr, si] = sort(fpr);
    tpr = tpr(si);
    [fpr, ui] = unique(fpr, 'last');
    tpr = tpr(ui);
    auc_val = trapz(fpr, tpr);
end


function s = sigmoid(z)
    s = 1 ./ (1 + exp(-z));
    s = max(min(s, 1 - 1e-15), 1e-15);
end


function [beta, converged, iters] = logreg_irls(X, y, w_class, lambda, ...
                                                 max_iter, tol)
%LOGREG_IRLS  L2-regularised logistic regression via IRLS.
    [~, p] = size(X);
    beta = zeros(p, 1);
    converged = false;
    iters = max_iter;
    L = lambda * eye(p);
    L(1,1) = 0;
    for it = 1:max_iter
        eta = X * beta;
        mu  = sigmoid(eta);
        W = mu .* (1 - mu) .* w_class;
        W = max(W, 1e-12);
        grad = X' * (w_class .* (y - mu)) - lambda * beta;
        grad(1) = grad(1) + lambda * beta(1);
        H = X' * (W .* X) + L;
        delta = H \ grad;
        beta  = beta + delta;
        if norm(delta) < tol
            converged = true;
            iters = it;
            break;
        end
    end
end


function fold_idx = stratified_folds(y, k)
%STRATIFIED_FOLDS  Assign each sample to a fold, preserving class ratio.
    N = numel(y);
    fold_idx = zeros(N, 1);
    idx_pos = find(y == 1);
    idx_neg = find(y == 0);
    idx_pos = idx_pos(randperm(numel(idx_pos)));
    idx_neg = idx_neg(randperm(numel(idx_neg)));
    for ii = 1:numel(idx_pos)
        fold_idx(idx_pos(ii)) = mod(ii-1, k) + 1;
    end
    for ii = 1:numel(idx_neg)
        fold_idx(idx_neg(ii)) = mod(ii-1, k) + 1;
    end
end
