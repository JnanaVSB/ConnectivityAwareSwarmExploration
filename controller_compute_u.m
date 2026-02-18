function u = controller_compute_u(robots, world, cfg, varargin)
% CONTROLLER_COMPUTE_U  Single-integrator control:
%   u = -k_grad * ∇Phi_total  -  k_rot * J ∇Phi_total
%
% Phi_total is assembled inside computePotentialAndGradient, and may include:
%   - Safety / collision avoidance
%   - Connectivity
%   - Dispersion / ring term
%   - Obstacle & boundary barriers
%   - Stay potential (time-in-region)
%   - Exploration / entropy potential from occupancy
%
% Signature variants supported:
%   u = controller_compute_u(robots, world, cfg)
%   u = controller_compute_u(robots, world, cfg, exploreField)
%   u = controller_compute_u(robots, world, cfg, exploreField, stayField)

% ---- Parse optional arguments ----
exploreField = [];
stayField    = [];

if ~isempty(varargin)
    exploreField = varargin{1};
    if numel(varargin) >= 2
        stayField = varargin{2};
    end
end

% ---- Potential and gradient ----
% Prefer the new extended signature; fall back gracefully if your
% computePotentialAndGradient is still on the old 3-argument version.
try
    [Phi, gradPhi] = computePotentialAndGradient(robots, world, cfg, ...
                                                 exploreField, stayField); %#ok<ASGLU>
catch
    % Old version: no exploration / stay contribution in Phi
    [Phi, gradPhi] = computePotentialAndGradient(robots, world, cfg); %#ok<ASGLU>
end

N = numel(robots);
u = zeros(N,2);

J = [0 -1; 1 0];   % 90° rotation matrix

for i = 1:N
    g = gradPhi(i,:).';      % 2x1 gradient wrt x_i

    % 1) Pure gradient descent term
    u_grad = -cfg.k_grad * g;

    % 2) Rotational term, orthogonal to ∇Phi
    u_rot  = -cfg.k_rot  * (J * g);

    ui = u_grad + u_rot;

    % Speed saturation
    v = norm(ui);
    if v > cfg.maxSpeed
        ui = (cfg.maxSpeed / v) * ui;
    end

    u(i,:) = ui.';
end

end
