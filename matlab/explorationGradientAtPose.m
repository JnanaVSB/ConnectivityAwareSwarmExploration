function gE = explorationGradientAtPose(exploreField, pose, cfg, world)
% EXPLORATIONGRADIENTATPOSE  Approx gradient of exploration field at robot.
%
%   gE is a 1x2 vector (∂E/∂x, ∂E/∂y).
%   We will use u_explore = cfg.k_exploreField * gE.

gE = [0, 0];

if isempty(exploreField) || ~cfg.useExplorationField
    return;
end

% Pose -> grid indices (row, col)
x = pose(1); 
y = pose(2);
[gy, gx] = worldToMapIndices(x, y, cfg);

N = cfg.mapSize;

% Need interior cell for central differences
if gx <= 1 || gx >= N || gy <= 1 || gy >= N
    return;
end

res = cfg.mapResolution;

% Central finite-difference gradient in world coordinates
dEx = (exploreField(gy, gx+1) - exploreField(gy, gx-1)) / (2*res);
% Remember: row index increases downward, so y-world increases upward -> sign flip
dEy = (exploreField(gy-1, gx) - exploreField(gy+1, gx)) / (2*res);

gE = [dEx, dEy];

end

function [gy, gx] = worldToMapIndices(x, y, cfg)
% WORLD2MAPINDICES  Map world coords (x,y) to matrix indices (row=gy, col=gx).
res = cfg.mapResolution;
N   = cfg.mapSize;
L   = N * res;
xMin = -L/2;
yMin = -L/2;

gx = round( (x - xMin) / res );   % 1..N
gy = round( (y - yMin) / res );   % 1..N

gx = max(1, min(N, gx));
gy = max(1, min(N, gy));
end
