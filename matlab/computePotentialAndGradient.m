function [Phi, gradPhi] = computePotentialAndGradient(robots, world, cfg, ...
                                                      exploreField, stayField)
% COMPUTEPOTENTIALANDGRADIENT  Total potential Phi and its gradient wrt each robot.
%
% Phi_total = Phi_pair + Phi_wall + Phi_obs + Phi_chain + Phi_conn + Phi_disp
%             + Phi_stay + Phi_entropy
%
% gradPhi(i,:) = ∇_{x_i} Phi_total.

dist = computeDistances(robots, world, cfg);
pos  = dist.pos;
D    = dist.D;
db   = dist.db;
nb   = dist.nb;
do   = dist.do;
no   = dist.no;

N       = size(pos,1);
gradPhi = zeros(N,2);
Phi     = 0.0;

d_min  = cfg.d_min;
d_far  = cfg.d_far;
k_pair = cfg.k_pair;

d_wall = cfg.d_wall;
k_wall = cfg.k_wall;

d_obs  = cfg.d_obs;
k_obs  = cfg.k_obs;

R_conn = cfg.R_conn;
k_conn = cfg.k_conn;

r0     = cfg.r0;
k_disp = cfg.k_disp;

% chain-spacing parameters (new)
k_chain   = cfg.k_chain;
d_chain   = cfg.d_chain;

eps_d  = 1e-3;

%% 0) Connectivity adjacency: minimum spanning tree (MST)
% Build a tree over all robots using Prim's algorithm on Euclidean distances.
% We then put chain + connectivity terms only on these tree edges.
A = zeros(N);              % upper-triangular adjacency matrix
inTree = false(1,N);
inTree(1) = true;          % start tree from robot 1

for k = 1:(N-1)
    best_d = inf;
    ia = 0; ja = 0;
    for i = 1:N
        if ~inTree(i), continue; end
        for j = 1:N
            if inTree(j), continue; end
            d = D(i,j);
            if d < best_d
                best_d = d;
                ia = i;
                ja = j;
            end
        end
    end
    if ia == 0
        % graph already disconnected by distance; nothing more to add
        break;
    end
    a = min(ia, ja);
    b = max(ia, ja);
    A(a,b) = 1;
    inTree(ja) = true;
end

%% 1) Pairwise potential (collision + short-range dispersion)
for i = 1:N
    for j = i+1:N
        d = D(i,j);
        if d < eps_d
            continue;
        end
        if d <= d_min + eps_d
            d_eff = d_min + eps_d;
        elseif d >= d_far
            continue;
        else
            d_eff = d;
        end
        s      = d_eff^2 - d_min^2;
        phi_ij = k_pair / s;
        Phi    = Phi + phi_ij;

        % gradient wrt xi and xj
        dphi_dd = k_pair * (-2*d_eff) / (s^2);
        e_ij    = (pos(i,:) - pos(j,:)) / d_eff;
        g_i     = dphi_dd * e_ij;
        g_j     = -g_i;
        gradPhi(i,:) = gradPhi(i,:) + g_i;
        gradPhi(j,:) = gradPhi(j,:) + g_j;
    end
end

%% 2) Boundary potential
for i = 1:N
    d = db(i);
    if d <= d_wall + eps_d
        d_eff = d_wall + eps_d;
    else
        d_eff = d;
    end
    s     = d_eff^2 - d_wall^2;
    phi_i = k_wall / s;
    Phi   = Phi + phi_i;

    dphi_dd = k_wall * (-2*d_eff) / (s^2);
    g_i     = dphi_dd * nb(i,:);
    gradPhi(i,:) = gradPhi(i,:) + g_i;
end

%% 3) Obstacle potential
nObs = size(do,2);
for i = 1:N
    for k = 1:nObs
        d = do(i,k);
        if d <= d_obs + eps_d
            d_eff = d_obs + eps_d;
        else
            d_eff = d;
        end
        s     = d_eff^2 - d_obs^2;
        phi_i = k_obs / s;
        Phi   = Phi + phi_i;

        dphi_dd = k_obs * (-2*d_eff) / (s^2);
        n_hat   = no{i,k};
        g_i     = dphi_dd * n_hat;
        gradPhi(i,:) = gradPhi(i,:) + g_i;
    end
end

%% 4) Chain-spacing potential on MST edges (Φ_chain)
% Encourages each tree edge length to be ~ d_chain (spread along backbone).
for i = 1:N
    for j = i+1:N
        if A(i,j) == 0
            continue;
        end
        d = D(i,j);
        if d < eps_d
            continue;
        end
        d_eff = d;

        phi_ij = 0.5 * k_chain * (d_eff - d_chain)^2;
        Phi    = Phi + phi_ij;

        dphi_dd = k_chain * (d_eff - d_chain);
        e_ij    = (pos(i,:) - pos(j,:)) / d_eff;

        g_i = dphi_dd * e_ij;
        g_j = -g_i;

        gradPhi(i,:) = gradPhi(i,:) + g_i;
        gradPhi(j,:) = gradPhi(j,:) + g_j;
    end
end

%% 5) Connectivity barrier on MST edges (near R_conn only)
R_safe = 0.9 * R_conn;    % below this, no connectivity barrier

for i = 1:N
    for j = i+1:N
        if A(i,j) == 0
            continue;
        end

        d = D(i,j);

        % If safely inside comm range, no connectivity force
        if d <= R_safe
            continue;
        end

        % Guard against degeneracy
        if d <= eps_d
            d_eff = eps_d;
        else
            d_eff = min(d, R_conn - 1e-3);   % avoid singularity at R_conn
        end

        % Barrier as d -> R_conn^- (prevents losing link)
        s      = R_conn^2 - d_eff^2;
        phi_ij = k_conn / s;
        Phi    = Phi + phi_ij;

        % Gradient wrt xi, xj
        dphi_dd = k_conn * (2*d_eff) / (s^2);   % > 0
        e_ij    = (pos(i,:) - pos(j,:)) / d_eff;

        g_i = dphi_dd * e_ij;
        g_j = -g_i;

        gradPhi(i,:) = gradPhi(i,:) + g_i;
        gradPhi(j,:) = gradPhi(j,:) + g_j;
    end
end

%% 6) Dispersion / ring potential (radial from origin)
for i = 1:N
    xi = pos(i,:);
    r  = norm(xi);
    if r < eps_d
        continue;
    end
    phi_i = 0.5 * k_disp * (r - r0)^2;
    Phi   = Phi + phi_i;

    dphi_dr = k_disp * (r - r0);
    g_i     = dphi_dr * (xi / r);
    gradPhi(i,:) = gradPhi(i,:) + g_i;
end

%% 7) Stay potential Φ_stay = k_stayField * S(x)
if cfg.useStayField && ~isempty(stayField)
    kS = cfg.k_stayField;
    for i = 1:N
        pose = robots(i).pose;
        % gradient of stayField S(x)
        gS = gradientOfFieldAtPose(stayField, pose, cfg, world);  % 1x2

        % Φ_stay = kS * S(x)  =>  ∇Φ_stay = kS * ∇S
        gradPhi(i,:) = gradPhi(i,:) + kS * gS;

        % accumulate Φ using local cell value
        [gy, gx] = worldToMapIndices(pose(1), pose(2), cfg, world);
        Phi = Phi + kS * stayField(gy, gx);
    end
end

%% 8) Exploration / entropy potential Φ_entropy = -kE * E(x)
%% 8) Exploration / entropy potential Φ_entropy = -kE_i * E(x_i)
if cfg.useExplorationField && ~isempty(exploreField)
    for i = 1:N
        pose = robots(i).pose;

        % Leader has stronger exploration gain
        if isfield(cfg, 'leaderID') && i == cfg.leaderID
            if isfield(cfg, 'leaderExploreFactor')
                kE_i = cfg.k_exploreField * cfg.leaderExploreFactor;
            else
                kE_i = cfg.k_exploreField;
            end
        else
            kE_i = cfg.k_exploreField;
        end

        % gradient of exploration field E(x)
        gE = gradientOfFieldAtPose(exploreField, pose, cfg, world);  % 1x2

        % Φ_entropy = -sum_i kE_i * E(x_i)
        % => ∇_{x_i} Φ_entropy = -kE_i * ∇E(x_i)
        gradPhi(i,:) = gradPhi(i,:) - kE_i * gE;

        [gy, gx] = worldToMapIndices(pose(1), pose(2), cfg, world);
        Phi = Phi - kE_i * exploreField(gy, gx);
    end
end


end
