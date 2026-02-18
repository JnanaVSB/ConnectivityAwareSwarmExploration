function g = gradientOfFieldAtPose(field, pose, cfg, world)
% GRADIENTOFFIELDATPOSE  Approx gradient of a scalar field at robot pose.
%
%   field : HxW scalar field over the map (e.g., stayField or exploreField)
%   pose  : [x; y; theta]
%   g     : 1x2 gradient [dF/dx, dF/dy] in world coordinates.

g = [0, 0];

if isempty(field)
    return;
end

x = pose(1);
y = pose(2);

[gy, gx] = worldToMapIndices(x, y, cfg, world);  % row=gy, col=gx
[H, W] = size(field);

% Too close to field boundary => no gradient
if gx <= 1 || gx >= W || gy <= 1 || gy >= H
    return;
end

% Central differences in grid coordinates
dFdx_grid = 0.5 * (field(gy, gx+1) - field(gy, gx-1));
dFdy_grid = 0.5 * (field(gy+1, gx) - field(gy-1, gx));

% Convert to metric gradient
res = cfg.mapResolution;
dFdx = dFdx_grid / res;
dFdy = dFdy_grid / res;

g = [dFdx, dFdy];

end
