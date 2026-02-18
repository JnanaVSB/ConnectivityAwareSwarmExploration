function map = updateMap(map, robotPose, ranges, cfg)
% UPDATEMAP  Integrate lidar ranges into robot's occupancy grid.
%
% Very simple: cast rays in grid, mark free cells up to hit, then occupied.

res = cfg.mapResolution;
N   = cfg.mapSize;
L   = res * N;
xMin = -L/2;
yMin = -L/2;

x = robotPose(1);
y = robotPose(2);
theta = robotPose(3);

numBeams = cfg.lidarNumBeams;
maxR     = cfg.lidarMaxRange;
step     = cfg.lidarStep;

angles = linspace(-pi, pi, numBeams);

for k = 1:numBeams
    rHit = ranges(k);
    ang  = theta + angles(k);
    dir  = [cos(ang); sin(ang)];

    r = 0;
    while r < rHit && r < maxR
        r = r + step;
        px = x + r * dir(1);
        py = y + r * dir(2);

        [row, col] = worldToMap(px, py, cfg);
        if row < 1 || row > N || col < 1 || col > N
            break;
        end
        if map(row,col) == cfg.unknown
            map(row,col) = cfg.freeVal;
        end
    end

    % Mark hit cell as occupied (if within map and not max range)
    if rHit < maxR
        px = x + rHit * dir(1);
        py = y + rHit * dir(2);
        [row, col] = worldToMap(px, py, cfg);
        if row >= 1 && row <= N && col >= 1 && col <= N
            map(row,col) = cfg.occVal;
        end
    end
end

end

function [row, col] = worldToMap(px, py, cfg)
res = cfg.mapResolution;
N   = cfg.mapSize;
L   = res * N;
xMin = -L/2;
yMin = -L/2;

col = floor((px - xMin)/res) + 1;
row = floor((py - yMin)/res) + 1;
end
