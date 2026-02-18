function main()
% MAIN  Multi-robot potential-field simulation with exploration & stay fields.

cfg    = config();
world  = loadWorld(cfg);
robots = initRobots(cfg, world);

dt       = cfg.dt;
T        = cfg.simTime;
numSteps = round(T/dt);

% Global fused map
globalMap = cfg.unknown * ones(cfg.mapSize, cfg.mapSize);

% Stay field: accumulates time robots spend in each region
stayField = zeros(cfg.mapSize, cfg.mapSize);

% Logs
covLog     = zeros(numSteps,1);
lambda2Log = zeros(numSteps,1);
minDegLog  = zeros(numSteps,1);
dminLog    = zeros(numSteps,1);
timeLog    = (0:numSteps-1)' * dt;

% Initial exploration field from empty global map
exploreField = computeExplorationField(globalMap, cfg);

for k = 1:numSteps
    % --- 1) Compute controls using current potential (incl. exploration & stay) ---
    u = controller_compute_u(robots, world, cfg, exploreField, stayField);

    % --- 2) Update robot states (single-integrator) ---
    for i = 1:cfg.numRobots
        pose = robots(i).pose;    % [x; y; theta]
        v    = u(i,:).';          % 2x1

        newPos = pose(1:2) + dt * v;

        if norm(v) > 1e-6
            theta = atan2(v(2), v(1)); % heading along velocity
        else
            theta = pose(3);           % keep previous heading if almost stopped
        end

        robots(i).pose = [newPos; theta];
    end

    % --- 3) Lidar + local map updates + global map fusion ---
    for i = 1:cfg.numRobots
        scan = simulateLidar(robots(i).pose, world, cfg);
        robots(i).map = updateMap(robots(i).map, robots(i).pose, scan, cfg);
        globalMap     = fuseMaps(globalMap, robots(i).map, cfg);
    end

    % --- 4) Update stayField: accumulate time spent per cell ---
    if cfg.useStayField
        for i = 1:cfg.numRobots
            x = robots(i).pose(1);
            y = robots(i).pose(2);
            [gy, gx] = worldToMapIndices(x, y, cfg, world);  % row, col

            if gy >= 1 && gy <= cfg.mapSize && gx >= 1 && gx <= cfg.mapSize
                stayField(gy, gx) = stayField(gy, gx) + dt;
            end
        end

        % smooth stayField so it becomes a broad "hill" instead of a spike
        R_s   = 1;
        sigma = 0.8;
        [Xs, Ys] = meshgrid(-R_s:R_s, -R_s:R_s);
        K = exp(-(Xs.^2 + Ys.^2)/(2*sigma^2));
        K = K / sum(K(:));

        stayField = conv2(stayField, K, 'same');
    end

    % --- 5) Recompute exploration field from updated global map ---
    exploreField = computeExplorationField(globalMap, cfg);

    % --- 6) Diagnostics ---
    [lambda2, minDeg] = computeConnectivity(robots, cfg);
    lambda2Log(k) = lambda2;
    minDegLog(k)  = minDeg;

    % Minimum inter-robot distance
    dist = computeDistances(robots, world, cfg);
    D    = dist.D;
    D(D == 0) = inf;
    dminLog(k) = min(D(:));

    % Coverage
    covLog(k) = computeCoverage(globalMap, world, cfg);

    % --- 7) Visualization & console log ---
    if mod(k, cfg.livePlotEvery) == 0 || k == 1 || k == numSteps
        t = (k-1)*dt;
        fprintf('t=%.1f s | cov=%.2f%% | lambda2=%.3f | d_min=%.3f m\n', ...
            t, covLog(k), lambda2Log(k), dminLog(k));

        plotState(world, robots, globalMap, cfg, ...
                  t, covLog(1:k), lambda2Log(1:k), minDegLog(1:k), dminLog(1:k));
        drawnow;
    end
end

% --- Final summary plots (λ2 and minDeg separated) ---
figure('Name', 'Coverage / Connectivity Summary', 'NumberTitle','off'); clf;
t = timeLog;

tiledlayout(2,2);

% 1) Coverage over time
nexttile;
plot(t, covLog, 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Coverage [%]');
title('Coverage vs Time');
grid on;

% 2) Min distance over time
nexttile;
plot(t, dminLog, 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('d_{min} [m]');
title('Minimum inter-robot distance');
grid on;

% 3) λ2 over time (own axis)
nexttile;
plot(t, lambda2Log, 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('\lambda_2');
title('Algebraic connectivity \lambda_2');
grid on;

% 4) min degree over time (own axis)
nexttile;
plot(t, minDegLog, 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('min degree');
title('Minimum degree of communication graph');
grid on;

end


function plotState(world, robots, globalMap, cfg, t, covLog, lambda2Log, minDegLog, dminLog)

persistent fh;

if isempty(fh) || ~isvalid(fh)
    fh = figure('Name', 'Multi-Robot Potential Field', 'NumberTitle','off');
end
figure(fh); clf;

tiledlayout(2,3);

% --- (1,1): World + robots ---
nexttile;
hold on; axis equal;

% Closed boundary polygon
B  = world.boundary;
Bc = [B; B(1,:)];   % close loop
plot(Bc(:,1), Bc(:,2), 'k-', 'LineWidth',1.5);

% Obstacles
for k = 1:numel(world.obstacles)
    poly = world.obstacles{k};
    patch(poly(:,1), poly(:,2), [0.7 0.7 0.7], 'EdgeColor','k');
end

% Robots
for i = 1:numel(robots)
    p = robots(i).pose;
    plot(p(1), p(2), 'bo', 'MarkerSize',6, 'MarkerFaceColor','b');
    text(p(1)+0.1, p(2)+0.1, sprintf('%d', robots(i).id), 'Color','k');
end
title(sprintf('World & robots (t = %.1f s)', t));
xlabel('x [m]'); ylabel('y [m]');
grid on;

% --- (1,2): Global occupancy map ---
nexttile;
imagesc(globalMap);
axis image off;
colormap(gray);
title('Global occupancy map');

% --- (1,3): Coverage ---
nexttile;
tt = (0:numel(covLog)-1) * cfg.dt;
plot(tt, covLog, 'LineWidth',1.5);
xlabel('Time [s]');
ylabel('Coverage [%]');
title('Coverage vs Time');
grid on;

% --- (2,1): Minimum inter-robot distance ---
nexttile;
tt_d = (0:numel(dminLog)-1) * cfg.dt;
plot(tt_d, dminLog, 'LineWidth',1.5);
xlabel('Time [s]');
ylabel('d_{min} [m]');
title('Minimum inter-robot distance');
grid on;

% --- (2,2): λ2 over time ---
nexttile;
tt_l = (0:numel(lambda2Log)-1) * cfg.dt;
plot(tt_l, lambda2Log, 'LineWidth',1.5);
xlabel('Time [s]');
ylabel('\lambda_2');
title('Algebraic connectivity \lambda_2');
grid on;

% --- (2,3): min degree over time ---
nexttile;
tt_deg = (0:numel(minDegLog)-1) * cfg.dt;
plot(tt_deg, minDegLog, 'LineWidth',1.5);
xlabel('Time [s]');
ylabel('min degree');
title(sprintf('Min degree (d_{min} = %.2f m)', dminLog(end)));
grid on;

end
