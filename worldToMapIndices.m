function [gy, gx] = worldToMapIndices(x, y, cfg, ~)
% WORLD2MAPINDICES  Map world coords (x,y) to matrix indices (row=gy, col=gx).
%
% Assumes world roughly centered at (0,0) and map covers [-R,R]^2.

R       = cfg.worldRadius;
res     = cfg.mapResolution;   %#ok<NASGU>  % R used for interpretation only
halfSize = cfg.mapSize / 2;

gx = round(halfSize + x / cfg.mapResolution);
gy = round(halfSize - y / cfg.mapResolution); % y ↑ -> row index decreases

% Clamp
gx = max(1, min(cfg.mapSize, gx));
gy = max(1, min(cfg.mapSize, gy));

end
