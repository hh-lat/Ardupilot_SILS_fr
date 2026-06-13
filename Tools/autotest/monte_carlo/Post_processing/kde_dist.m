function [D, info] = kde_dist(data, names, paramNames, varargin)
    nG = numel(data);
    if nG < 2, error('kde_dist:groups','Need success + >=1 failure group.'); end
    P  = size(data{1}, 2);
    if any(cellfun(@(g) size(g,2), data) ~= P)
        error('kde_dist:cols','All groups must have the same number of columns (parameters).');
    end

    if nargin < 2 || isempty(names)
        names = ["Success", "Failure " + string(1:nG-1)];
    end
    if nargin < 3 || isempty(paramNames)
        paramNames = "Param " + string(1:P);
    end

    p = inputParser;
    addParameter(p,'NumPoints',1000,@(v)isscalar(v)&&v>1);
    addParameter(p,'Bandwidth',"plug-in");
    addParameter(p,'Plot',false,@(v)islogical(v)&&isscalar(v));
    parse(p,varargin{:});
    opt = p.Results;

    D = zeros(P, nG-1);
    grids = cell(P,1); fSall = cell(P,1); fFall = cell(P,nG-1); bw = zeros(P,1);
    colors = lines(nG);

    for pp = 1:P
        % values of parameter pp in each group
        v = cellfun(@(g) g(:,pp), data, 'UniformOutput', false);
        pooled = vertcat(v{:});

        lo = min(pooled); hi = max(pooled);
        pad = 0.05*(hi-lo) + eps;
        grid = linspace(lo-pad, hi+pad, opt.NumPoints)';
        dx = grid(2)-grid(1);
        grids{pp} = grid;

        % one common bandwidth per parameter (from pooled data)
        if isnumeric(opt.Bandwidth)
            bw(pp) = opt.Bandwidth;
        else
            [~,~,bw(pp)] = kde(pooled, Bandwidth=opt.Bandwidth);
        end

        fS = kde(v{1}, EvaluationPoints=grid, Bandwidth=bw(pp));
        fSall{pp} = fS;

        if opt.Plot
            figure; hold on; box on;
            h = gobjects(nG,1);
            h(1) = plot(grid, fS, 'Color', colors(1,:), 'LineWidth',1.5);
        end

        for m = 2:nG
            fF = kde(v{m}, EvaluationPoints=grid, Bandwidth=bw(pp));
            fFall{pp,m-1} = fF;
            D(pp, m-1) = 0.5 * sum(abs(fF - fS)) * dx;   % total variation in [0,1]
            if opt.Plot
                h(m) = plot(grid, fF, 'Color', colors(m,:), 'LineWidth',1.5);
            end
        end

        if opt.Plot
            xlabel(paramNames(pp)); ylabel('P(X)');
            title("PDFs - " + paramNames(pp));
            legend(h, names, 'Location','best'); hold off;
        end
    end

    % optional "contribution" view: each failure column scaled to sum to 1
    colsum = sum(D,1); colsum(colsum==0) = 1;
    Dnorm = D ./ colsum;

    info = struct('grids',{grids},'fS',{fSall},'fF',{fFall},'bw',bw, ...
                  'names',names,'paramNames',paramNames,'Dnorm',Dnorm);
end
