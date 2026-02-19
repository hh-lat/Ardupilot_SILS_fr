%% ═══════════════════════════════════════════════════════════════════════
%  PLOT_PARAMETER_DISTRIBUTIONS
%  ─────────────────────────────────────────────────────────────────────
%  Publication-quality distribution plots for all 30 perturbed Monte
%  Carlo parameters.  Reads only the results folder (summary.csv) and
%  the JSON config that contains nominal values & 3-sigma bounds.
%
%  Usage:
%    >> plot_distributions
%
%  Outputs (saved in results_dir/figures/):
%    fig_dist_mass_inertia.png
%    fig_dist_cg_env_thrust.png
%    fig_dist_CL.png
%    fig_dist_CD.png
%    fig_dist_Cl_roll.png
%    fig_dist_Cm_pitch.png
%
%  Author : LAT Avionics — MC Analysis Pipeline
%  Date   : 2026-02-19
%% ═══════════════════════════════════════════════════════════════════════
clear; clc; close all;
fprintf('══════════════════════════════════════════════════════════════════\n');
fprintf('  Monte Carlo — Perturbed Parameter Distribution Analysis\n');
fprintf('══════════════════════════════════════════════════════════════════\n\n');

%% ─── 1. PATHS ──────────────────────────────────────────────────────────
results_dir = '\\wsl.localhost\Ubuntu\home\lat_avionics\Ardupilot_SITL_LATEST\Ardupilot_SILS\Tools\autotest\monte_carlo\mc_results_20260217_074641';

json_path   = fullfile(results_dir, '..', 'monte_carlo_config.json');
csv_path    = fullfile(results_dir, 'summary.csv');

% Output folder
fig_dir = fullfile(results_dir, 'figures');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

%% ─── 2. LOAD JSON CONFIG (nominal values & 3-sigma bounds) ─────────────
fprintf('  [1] Loading JSON config ...\n');
json_txt = fileread(json_path);
cfg      = jsondecode(json_txt);
params   = cfg.params;

% Build lookup: param_name → {nominal, sigma_3, sigma_1}
pnames = fieldnames(params);
nP_json = numel(pnames);   % should be 30
nominal  = zeros(nP_json, 1);
sigma3   = zeros(nP_json, 1);
sigma1   = zeros(nP_json, 1);
for ii = 1:nP_json
    nominal(ii) = params.(pnames{ii}).nominal;
    sigma3(ii)  = params.(pnames{ii}).sigma_3;
    sigma1(ii)  = sigma3(ii) / 3;
end
fprintf('    %d parameters loaded from JSON\n', nP_json);

%% ─── 3. LOAD SUMMARY CSV (perturbed values per case) ───────────────────
fprintf('  [2] Loading summary.csv ...\n');
T = readtable(csv_path);
N = height(T);
fprintf('    %d cases loaded\n', N);

% Extract parameter columns (all start with 'p_')
param_cols = T.Properties.VariableNames(startsWith(T.Properties.VariableNames, 'p_'));
nP = numel(param_cols);
fprintf('    %d parameter columns found\n', nP);

% Build data matrix  [N × nP] and map column names to JSON names
data    = zeros(N, nP);
p_label = cell(nP, 1);     % stripped names (without 'p_' prefix)
p_nom   = zeros(nP, 1);
p_s3    = zeros(nP, 1);
p_s1    = zeros(nP, 1);

for ii = 1:nP
    col_name   = param_cols{ii};
    json_name  = col_name(3:end);        % strip 'p_'
    p_label{ii} = json_name;
    data(:, ii) = T.(col_name);

    jdx = find(strcmp(pnames, json_name), 1);
    if ~isempty(jdx)
        p_nom(ii) = nominal(jdx);
        p_s3(ii)  = sigma3(jdx);
        p_s1(ii)  = sigma1(jdx);
    else
        warning('Parameter "%s" not found in JSON config.', json_name);
    end
end

%% ─── 4. DEFINE PARAMETER GROUPS ────────────────────────────────────────
fprintf('  [3] Organising parameter groups ...\n');

groups = struct();

groups(1).name     = 'Mass & Inertia';
groups(1).params   = {'mass', 'Ixx', 'Iyy', 'Izz'};
groups(1).layout   = [2, 2];
groups(1).filename = 'fig_dist_mass_inertia.png';

groups(2).name     = 'CG Location, Environment & Propulsion';
groups(2).params   = {'cg_x', 'cg_z', 'rho', 'max_thrust'};
groups(2).layout   = [2, 2];
groups(2).filename = 'fig_dist_cg_env_thrust.png';

groups(3).name     = 'Lift Coefficients  (C_L)';
groups(3).params   = {'CL_0', 'CL_alpha', 'CL_delta_e', 'CL_q'};
groups(3).layout   = [2, 2];
groups(3).filename = 'fig_dist_CL.png';

groups(4).name     = 'Drag Coefficients  (C_D)';
groups(4).params   = {'CD_0', 'CD_alpha', 'CD_delta_e', 'CD_delta_f', 'CD_delta_e2'};
groups(4).layout   = [3, 2];
groups(4).filename = 'fig_dist_CD.png';

groups(5).name     = 'Roll Moment Coefficients  (C_l)';
groups(5).params   = {'Cl_0', 'Cl_beta', 'Cl_delta_r', 'Cl_delta_aL', 'Cl_delta_aR', 'Cl_r'};
groups(5).layout   = [3, 2];
groups(5).filename = 'fig_dist_Cl_roll.png';

groups(6).name     = 'Pitch Moment Coefficients  (C_m)';
groups(6).params   = {'Cm_0', 'Cm_alpha', 'Cm_delta_e', 'Cm_q', 'Cm_delta_f', 'Cm_Cmu', 'Cm_beta2'};
groups(6).layout   = [4, 2];
groups(6).filename = 'fig_dist_Cm_pitch.png';

%% ─── 5. UNITS MAP ─────────────────────────────────────────────────────
%  Map every parameter to its physical unit string.
units_map = containers.Map();
units_map('mass')        = 'kg';
units_map('Ixx')         = 'kg m^2';
units_map('Iyy')         = 'kg m^2';
units_map('Izz')         = 'kg m^2';
units_map('cg_x')        = 'm';
units_map('cg_z')        = 'm';
units_map('rho')         = 'kg/m^3';
units_map('max_thrust')  = 'N';
units_map('CL_0')        = '–';
units_map('CL_alpha')    = '1/rad';
units_map('CL_delta_e')  = '1/rad';
units_map('CL_q')        = '–';
units_map('CD_0')        = '–';
units_map('CD_alpha')    = '1/rad';
units_map('CD_delta_e')  = '1/rad';
units_map('CD_delta_f')  = '1/rad';
units_map('CD_delta_e2') = '1/rad^2';
units_map('Cl_0')        = '–';
units_map('Cl_beta')     = '1/rad';
units_map('Cl_delta_r')  = '1/rad';
units_map('Cl_delta_aL') = '1/rad';
units_map('Cl_delta_aR') = '1/rad';
units_map('Cl_r')        = '–';
units_map('Cm_0')        = '–';
units_map('Cm_alpha')    = '1/rad';
units_map('Cm_delta_e')  = '1/rad';
units_map('Cm_q')        = '–';
units_map('Cm_delta_f')  = '1/rad';
units_map('Cm_Cmu')      = '–';
units_map('Cm_beta2')    = '1/rad^2';

%% ─── 6. PRETTY-PRINT NAME MAP (TeX) ───────────────────────────────────
pretty_map = containers.Map();
pretty_map('mass')        = 'mass';
pretty_map('Ixx')         = 'I_{xx}';
pretty_map('Iyy')         = 'I_{yy}';
pretty_map('Izz')         = 'I_{zz}';
pretty_map('cg_x')        = 'x_{cg}';
pretty_map('cg_z')        = 'z_{cg}';
pretty_map('rho')         = '\rho';
pretty_map('max_thrust')  = 'T_{max}';
pretty_map('CL_0')        = 'C_{L0}';
pretty_map('CL_alpha')    = 'C_{L\alpha}';
pretty_map('CL_delta_e')  = 'C_{L\delta_e}';
pretty_map('CL_q')        = 'C_{Lq}';
pretty_map('CD_0')        = 'C_{D0}';
pretty_map('CD_alpha')    = 'C_{D\alpha}';
pretty_map('CD_delta_e')  = 'C_{D\delta_e}';
pretty_map('CD_delta_f')  = 'C_{D\delta_f}';
pretty_map('CD_delta_e2') = 'C_{D\delta_e^2}';
pretty_map('Cl_0')        = 'C_{l0}';
pretty_map('Cl_beta')     = 'C_{l\beta}';
pretty_map('Cl_delta_r')  = 'C_{l\delta_r}';
pretty_map('Cl_delta_aL') = 'C_{l\delta_{aL}}';
pretty_map('Cl_delta_aR') = 'C_{l\delta_{aR}}';
pretty_map('Cl_r')        = 'C_{lr}';
pretty_map('Cm_0')        = 'C_{m0}';
pretty_map('Cm_alpha')    = 'C_{m\alpha}';
pretty_map('Cm_delta_e')  = 'C_{m\delta_e}';
pretty_map('Cm_q')        = 'C_{mq}';
pretty_map('Cm_delta_f')  = 'C_{m\delta_f}';
pretty_map('Cm_Cmu')      = 'C_{m_{C_\mu}}';
pretty_map('Cm_beta2')    = 'C_{m\beta^2}';

%% ─── 7. COLOUR PALETTE & STYLE CONSTANTS ──────────────────────────────
% Background
clr_fig_bg  = [0.94  0.94  0.96];   % cool light grey
clr_ax_bg   = [0.98  0.98  1.00];   % very faint blue-grey axes

% Histogram — gradient fill (dark teal body, lighter edge)
clr_hist_face = [0.18  0.52  0.72];   % teal blue
clr_hist_edge = [0.10  0.38  0.56];   % darker edge

% KDE fills
clr_kde_fill  = [0.18  0.52  0.72  0.18];   % very light fill under KDE
clr_kde_line  = [0.05  0.05  0.15];          % near-black KDE line

% Reference lines
clr_nominal   = [0.84  0.12  0.12];   % red — nominal
clr_mean      = [0.08  0.62  0.22];   % green — sample mean
clr_sigma     = [0.55  0.10  0.68];   % purple — ±1σ
clr_sigma_fill= [0.72  0.45  0.85  0.10];  % light purple fill for ±1σ band

% Reference Gaussian
clr_ref_gauss = [0.50  0.50  0.50];   % grey

% Stats box
clr_box_bg    = [0.12  0.14  0.18];   % dark charcoal
clr_box_fg    = [1.00  1.00  1.00];   % white text
clr_box_edge  = [0.35  0.40  0.50];   % subtle border

hist_alpha = 0.75;
n_bins     = 45;

%% ─── 8. GENERATE FIGURES ──────────────────────────────────────────────
fprintf('  [4] Generating distribution figures ...\n\n');

for gg = 1:numel(groups)
    grp      = groups(gg);
    nSub     = numel(grp.params);
    nR       = grp.layout(1);
    nC       = grp.layout(2);
    fig_h    = 380 * nR + 160;
    fig_w    = 760 * nC + 100;

    fig = figure('Units', 'pixels', 'Position', [30 30 fig_w fig_h], ...
                 'Color', clr_fig_bg, 'Name', grp.name);

    for ss = 1:nSub
        pname = grp.params{ss};
        pidx  = find(strcmp(p_label, pname), 1);
        if isempty(pidx)
            warning('  Parameter "%s" not found in data — skipped.', pname);
            continue;
        end

        ax = subplot(nR, nC, ss);
        hold(ax, 'on');
        set(ax, 'Color', clr_ax_bg);

        x  = data(:, pidx);
        mu_data = mean(x);
        sd_data = std(x);
        mu_json = p_nom(pidx);
        sd_json = p_s1(pidx);

        % Get unit string
        if units_map.isKey(pname)
            unit_str = units_map(pname);
        else
            unit_str = '–';
        end

        % Get pretty name
        if pretty_map.isKey(pname)
            pretty_name = pretty_map(pname);
        else
            pretty_name = strrep(pname, '_', ' ');
        end

        % ── Histogram ──
        [counts, edges] = histcounts(x, n_bins);
        centres = (edges(1:end-1) + edges(2:end)) / 2;
        bin_w   = edges(2) - edges(1);
        density = counts / (N * bin_w);
        bar(ax, centres, density, 1, ...
            'FaceColor', clr_hist_face, 'EdgeColor', clr_hist_edge, ...
            'FaceAlpha', hist_alpha, 'LineWidth', 0.5);

        % ── KDE (kernel density estimate) ──
        x_kde = linspace(min(x) - 3*sd_data, max(x) + 3*sd_data, 600);
        bw = 1.06 * sd_data * N^(-1/5);
        kde_y = zeros(size(x_kde));
        for kk = 1:N
            kde_y = kde_y + exp(-0.5 * ((x_kde - x(kk)) / bw).^2);
        end
        kde_y = kde_y / (N * bw * sqrt(2*pi));

        % Fill under KDE
        fill(ax, [x_kde, fliplr(x_kde)], [kde_y, zeros(size(kde_y))], ...
            clr_hist_face, 'FaceAlpha', 0.12, 'EdgeColor', 'none');
        plot(ax, x_kde, kde_y, '-', 'Color', clr_kde_line, 'LineWidth', 2.2);

        y_max = max(max(density), max(kde_y)) * 1.45;

        % ── Reference Gaussian (from JSON nominal & sigma) ──
        ref_y = (1/(sd_json * sqrt(2*pi))) * exp(-0.5 * ((x_kde - mu_json)/sd_json).^2);
        plot(ax, x_kde, ref_y, '--', 'Color', clr_ref_gauss, 'LineWidth', 1.8);
        y_max = max(y_max, max(ref_y) * 1.45);

        % ── ±1σ shaded band (purple fill) ──
        x_band_lo = mu_data - sd_data;
        x_band_hi = mu_data + sd_data;
        patch(ax, [x_band_lo x_band_hi x_band_hi x_band_lo], ...
            [0 0 y_max y_max], ...
            [0.72 0.45 0.85], 'FaceAlpha', 0.08, 'EdgeColor', 'none');

        % ── ±1σ lines (purple, dotted) ──
        plot(ax, [x_band_lo x_band_lo], [0 y_max], ':', ...
            'Color', clr_sigma, 'LineWidth', 1.8);
        plot(ax, [x_band_hi x_band_hi], [0 y_max], ':', ...
            'Color', clr_sigma, 'LineWidth', 1.8);

        % ── Nominal line (red, dashed) ──
        plot(ax, [mu_json mu_json], [0 y_max], '--', ...
            'Color', clr_nominal, 'LineWidth', 2.2);

        % ── Sample mean line (green, solid) ──
        plot(ax, [mu_data mu_data], [0 y_max], '-', ...
            'Color', clr_mean, 'LineWidth', 2.2);

        % ── Axes formatting ──
        xlim(ax, [mu_json - 4.5*sd_json, mu_json + 4.5*sd_json]);
        ylim(ax, [0 y_max]);
        set(ax, 'FontSize', 10, 'FontWeight', 'bold', 'FontName', 'Helvetica', ...
            'Box', 'on', 'LineWidth', 1.0, 'TickDir', 'out', ...
            'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], ...
            'GridAlpha', 0.15, 'GridColor', [0.3 0.3 0.3]);
        grid(ax, 'on');

        % X-label with unit
        if strcmp(unit_str, char(8211))   % en-dash means dimensionless
            xlabel(ax, sprintf('%s  [–]', pretty_name), ...
                'FontSize', 12, 'FontWeight', 'bold', 'Interpreter', 'tex');
        else
            xlabel(ax, sprintf('%s  [%s]', pretty_name, unit_str), ...
                'FontSize', 12, 'FontWeight', 'bold', 'Interpreter', 'tex');
        end
        ylabel(ax, 'Probability Density', 'FontSize', 10, 'FontWeight', 'bold');

        % ── Title ──
        title(ax, pretty_name, ...
            'FontSize', 14, 'FontWeight', 'bold', 'Interpreter', 'tex', ...
            'Color', [0.10 0.10 0.15]);

        % ── Stats annotation box (dark background, white text) ──────────
        mu_err_pct = abs(mu_data - mu_json) / (abs(mu_json) + 1e-30) * 100;
        sd_err_pct = abs(sd_data - sd_json) / (abs(sd_json) + 1e-30) * 100;

        % Build a single string with newlines (no cell array issues)
        stats_str = sprintf([ ...
            'N = %d\n' ...
            '---  Sample  ---\n' ...
            '\\mu  = %s  %s\n' ...
            '\\sigma = %s  %s\n' ...
            '---  Config  ---\n' ...
            '\\mu_0  = %s  %s\n' ...
            '\\sigma_0 = %s  %s\n' ...
            '--- Accuracy ---\n' ...
            '\\Delta\\mu   = %.2f%%\n' ...
            '\\Delta\\sigma = %.2f%%'], ...
            N, ...
            smart_fmt(mu_data), unit_str, ...
            smart_fmt(sd_data), unit_str, ...
            smart_fmt(mu_json), unit_str, ...
            smart_fmt(sd_json), unit_str, ...
            mu_err_pct, sd_err_pct);

        text(ax, 0.97, 0.96, stats_str, ...
            'Units', 'normalized', ...
            'HorizontalAlignment', 'right', ...
            'VerticalAlignment', 'top', ...
            'FontSize', 8.5, 'FontName', 'Consolas', ...
            'FontWeight', 'bold', ...
            'Interpreter', 'tex', ...
            'BackgroundColor', clr_box_bg, ...
            'Color', clr_box_fg, ...
            'EdgeColor', clr_box_edge, ...
            'LineWidth', 1.2, ...
            'Margin', 6);

        hold(ax, 'off');
    end

    % ── Super-title ──
    sgtitle(fig, sprintf('Parameter Distributions  —  %s    (N = %d MC cases)', ...
        grp.name, N), ...
        'FontSize', 18, 'FontWeight', 'bold', 'Color', [0.08 0.08 0.12], ...
        'FontName', 'Helvetica');

    % ── Shared legend at bottom ──
    ax_lgd = axes(fig, 'Position', [0.08 0.003 0.84 0.045], 'Visible', 'off');
    hold(ax_lgd, 'on');
    h1 = bar(ax_lgd, NaN, NaN, 'FaceColor', clr_hist_face, ...
        'EdgeColor', clr_hist_edge, 'FaceAlpha', hist_alpha);
    h2 = plot(ax_lgd, NaN, NaN, '-',  'Color', clr_kde_line,  'LineWidth', 2.2);
    h3 = plot(ax_lgd, NaN, NaN, '--', 'Color', clr_ref_gauss, 'LineWidth', 1.8);
    h4 = plot(ax_lgd, NaN, NaN, '--', 'Color', clr_nominal,   'LineWidth', 2.2);
    h5 = plot(ax_lgd, NaN, NaN, '-',  'Color', clr_mean,      'LineWidth', 2.2);
    h6 = plot(ax_lgd, NaN, NaN, ':',  'Color', clr_sigma,     'LineWidth', 1.8);
    lgd = legend(ax_lgd, [h1 h2 h3 h4 h5 h6], ...
        {'Histogram', 'KDE (Silverman)', 'Ref. Gaussian (Config)', ...
         'Nominal \mu_0', 'Sample Mean \mu', 'Sample \pm1\sigma'}, ...
        'Orientation', 'horizontal', 'FontSize', 10, 'FontWeight', 'bold', ...
        'Location', 'south', 'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 0.8, ...
        'NumColumns', 6, 'Color', clr_fig_bg, 'TextColor', [0.1 0.1 0.1]);
    hold(ax_lgd, 'off');

    % ── Export ──
    exportgraphics(fig, fullfile(fig_dir, grp.filename), ...
        'Resolution', 300, 'BackgroundColor', clr_fig_bg);
    fprintf('    saved %s\n', grp.filename);
end

%% ─── 9. SUMMARY TABLE ─────────────────────────────────────────────────
fprintf('\n  [5] Parameter Distribution Summary\n');
fprintf('  %-22s  %8s  %12s  %12s  %12s  %12s  %8s  %8s\n', ...
    'Parameter', 'Unit', 'Nominal', 'Sample Mean', 'JSON sigma', 'Sample sigma', ...
    'dMu(%)', 'dSig(%)');
fprintf('  %s\n', repmat('-', 1, 110));
for ii = 1:nP
    mu_d  = mean(data(:,ii));
    sd_d  = std(data(:,ii));
    mu_j  = p_nom(ii);
    sd_j  = p_s1(ii);
    dmu   = abs(mu_d - mu_j) / (abs(mu_j) + 1e-30) * 100;
    dsd   = abs(sd_d - sd_j) / (abs(sd_j) + 1e-30) * 100;
    if units_map.isKey(p_label{ii})
        u = units_map(p_label{ii});
    else
        u = '-';
    end
    fprintf('  %-22s  %8s  %12.6f  %12.6f  %12.6f  %12.6f  %7.2f%%  %7.2f%%\n', ...
        p_label{ii}, u, mu_j, mu_d, sd_j, sd_d, dmu, dsd);
end
fprintf('  %s\n', repmat('-', 1, 110));

%% ─── DONE ──────────────────────────────────────────────────────────────
fprintf('\n==================================================================\n');
fprintf('  Distribution analysis complete.\n');
fprintf('  Figures saved to: %s\n', fig_dir);
fprintf('==================================================================\n');


%% ═══════════════════════════════════════════════════════════════════════
%  LOCAL FUNCTIONS
%  ═══════════════════════════════════════════════════════════════════════

function s = smart_fmt(v)
%SMART_FMT  Format a number nicely: scientific for tiny values, fixed otherwise.
    av = abs(v);
    if av == 0
        s = '0';
    elseif av < 0.0001
        s = sprintf('%.3e', v);
    elseif av < 0.01
        s = sprintf('%.5f', v);
    elseif av < 1
        s = sprintf('%.4f', v);
    elseif av < 100
        s = sprintf('%.4f', v);
    else
        s = sprintf('%.2f', v);
    end
end
