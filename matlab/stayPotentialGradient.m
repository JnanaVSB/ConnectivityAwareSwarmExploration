function g_stay = stayPotentialGradient(robots, cfg)
% STAYPOTENTIALGRADIENT  ∇Φ_stay for each robot (N x 2).
%
% Φ_stay,i ~ h(τ_i) * (x_i - c_i), where:
%   τ_i: time spent near stayCenter
%   c_i: stayCenter
%   h(τ_i): increasing, saturating function in [0,1]

N = numel(robots);
g_stay = zeros(N, 2);

% If stay potential is disabled, return zeros
if ~isfield(cfg, 'useStayPotential') || ~cfg.useStayPotential
    return;
end

R    = cfg.stayRadius;
Tsat = cfg.stayTimeHorizon;
k_s  = cfg.k_stay;

for i = 1:N
    % Defensive: missing fields -> skip
    if ~isfield(robots(i), 'stayTime') || ~isfield(robots(i), 'stayCenter')
        continue;
    end

    tau = robots(i).stayTime;
    if tau <= 0
        % No dwell time yet -> no stay force
        continue;
    end

    % Positions
    x = robots(i).pose(1:2);        % [x; y] or [x y], treat as 2x1
    x = x(:);                       % force column 2x1
    c = robots(i).stayCenter(:);    % force column 2x1

    % Direction from stay center
    dx = x - c;
    d  = norm(dx);

    if d < 1e-6
        dir = [0; 0];               % no direction if exactly at center
    else
        dir = dx / d;               % unit vector
    end

    % h(τ) in [0,1]
    h = min(max(tau / Tsat, 0.0), 1.0);

    % Gradient of Φ_stay,i:
    %   ∇Φ_stay,i = k_s * h(τ_i) * dir
    g_vec = k_s * h * dir;          % 2x1

    % Assign row i (1x2)
    g_stay(i, :) = g_vec.';         % transpose to 1x2
end

end
