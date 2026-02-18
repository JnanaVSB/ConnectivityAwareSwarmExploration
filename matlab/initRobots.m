function robots = initRobots(cfg, world)
% INITROBOTS  Initialize robots near the center with empty maps.
% No inpolygon, no rejection sampling – fast and deterministic.

N = cfg.numRobots;
robots = struct([]);

M = cfg.mapSize;
map0 = cfg.unknown * ones(M, M);

for i = 1:N
    % Small random perturbation around origin (0,0)
    x = 0.2 * randn();
    y = 0.2 * randn();
    theta = rand()*2*pi - pi;

    robots(i).id    = i;
    robots(i).pose  = [x; y; theta];
    robots(i).map   = map0;
    robots(i).scan  = [];
    robots(i).trajX = [];
    robots(i).trajY = [];

    % For Φ_stay (time-in-region potential)
    robots(i).stayCenter = [x; y];   % 2x1
    robots(i).stayTime   = 0.0;      % [s]
end

end
