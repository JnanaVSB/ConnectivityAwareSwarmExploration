function ranges = simulateLidar(robotPose, world, cfg)
% SIMULATELIDAR  Simple ray-casting lidar in 2D polygonal world.
%
%   robotPose = [x; y; theta]
%   ranges    = 1 x lidarNumBeams array of ranges [0, lidarRange]

x = robotPose(1);
y = robotPose(2);
theta = robotPose(3);

numBeams = cfg.lidarNumBeams;
maxR     = cfg.lidarMaxRange;
step     = cfg.mapResolution;

ranges = maxR * ones(1, numBeams);

angles = linspace(-pi, pi, numBeams);  % relative to robot heading

% Pre-build boundary and obstacles polygon sets for inpolygon
bndX = world.boundary(:,1);
bndY = world.boundary(:,2);

obs = world.obstacles;

for k = 1:numBeams
    ang = theta + angles(k);
    dir = [cos(ang); sin(ang)];

    r = 0;
    while r < maxR
        r = r + step;
        px = x + r * dir(1);
        py = y + r * dir(2);

        % Check outside workspace: if point not inside boundary polygon
        inside = inpolygon(px, py, bndX, bndY);
        if ~inside
            ranges(k) = r;
            break;
        end

        % Check obstacle hit
        hitObs = false;
        for j = 1:numel(obs)
            poly = obs{j};
            if inpolygon(px, py, poly(:,1), poly(:,2))
                ranges(k) = r;
                hitObs = true;
                break;
            end
        end
        if hitObs
            break;
        end
    end
end

end
