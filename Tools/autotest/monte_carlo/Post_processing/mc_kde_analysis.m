%% MC_KDE_ANALYSIS  Success-vs-Failure parameter distribution analysis (KDE)
% =========================================================================
%  Splits the Monte Carlo cases into success / failure groups, then uses
%  kde_dist() to KDE each perturbed parameter per group and rank them by the
%  total-variation (TV) distance between the two PDFs. Large TV => that
%  parameter's distribution differs most between success and failure, i.e. it
%  drives the outcome. Plots the top-N most-discriminating parameters.
%
%  Inputs: <results_path>/summary.csv  (1000 cases x 112 p_* param columns)
%  Outputs: kde_tv_ranking_matlab.csv + a tiled PDF figure.
%
%  Requires kde_dist.m (same folder) and base MATLAB kde() (R2024a+).
% =========================================================================
clc; clear; close all;

%% ---- CONFIG ----
results_path = '\\wsl.localhost\Ubuntu\home\sushanthvenkata\Projects\Ardupilot_SILS\Tools\autotest\monte_carlo\mc_results_20260611_173053';
TOPN = 9;                 % number of top parameters to plot

T = readtable(fullfile(results_path,'summary.csv'));
pcols = T.Properties.VariableNames(startsWith(T.Properties.VariableNames,'p_'));
paramNames = string(erase(pcols,'p_'));
X = T{:, pcols};

%% ---- SUCCESS / FAILURE SPLIT (edit the criterion here) ----
% Default: takeoff_success flag (reached >=15 m AGL during the takeoff phase).
% This is the reliable "did it fly" label -- land_result mislabels never-flew
% cases as "landed". Alternatives, e.g.:
%   ok = T.max_alt_m >= 100;                 % reached near cruise altitude
%   ok = strcmpi(string(T.land_result),"landed") & T.max_alt_m >= 15;
ok = strcmpi(string(T.takeoff_success),"true");

data  = { X(ok,:), X(~ok,:) };
names = ["Success (" + sum(ok) + ")", "Failure (" + sum(~ok) + ")"];
fprintf('success=%d  failure=%d  params=%d\n', sum(ok), sum(~ok), numel(pcols));
if sum(~ok) < 2, error('Need >=2 failure cases; loosen the failure criterion.'); end

%% ---- TV distances (no per-param figures) ----
[D, info] = kde_dist(data, names, paramNames, 'Plot', false);

%% ---- rank + save ----
[Dsort, idx] = sort(D(:,1), 'descend');
Rank = table(paramNames(idx)', Dsort, 'VariableNames', {'param','TV_distance'});
writetable(Rank, fullfile(results_path,'kde_tv_ranking_matlab.csv'));
fprintf('\nTop parameters by TV distance (success vs failure):\n');
disp(Rank(1:min(15,height(Rank)),:));

%% ---- plot top-N PDFs in one tiled layout ----
figure('Name','Top discriminating params (KDE)','Position',[80 80 1300 850]);
tl = tiledlayout('flow','TileSpacing','compact','Padding','compact');
for i = 1:min(TOPN, numel(idx))
    pp = idx(i);
    nexttile; hold on; box on;
    plot(info.grids{pp}, info.fS{pp},    'LineWidth',1.6);
    plot(info.grids{pp}, info.fF{pp,1},  'LineWidth',1.6);
    title(sprintf('%s  (TV=%.2f)', paramNames(pp), D(pp,1)), 'Interpreter','none');
    xlabel(paramNames(pp),'Interpreter','none'); ylabel('PDF');
    if i==1, legend(names,'Location','best'); end
end
title(tl, 'Success vs Failure: parameter PDFs (ranked by total-variation distance)');
