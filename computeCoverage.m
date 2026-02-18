function covPercent = computeCoverage(globalMap, world, cfg)
% COMPUTECOVERAGE  Percentage of known cells inside workspace polygon.

N   = cfg.mapSize;
res = cfg.mapResolution;
L   = res * N;
xMin = -L/2;
yMin = -L/2;

% Precompute workspace mask
persistent mask;
if isempty(mask)
    mask = false(N,N);
    [Xc, Yc] = meshgrid(0:N-1, 0:N-1);
    Xw = xMin + (Xc + 0.5)*res;
    Yw = yMin + (Yc + 0.5)*res;
    in = inpolygon(Xw, Yw, world.boundary(:,1), world.boundary(:,2));
    mask = in;
end

known = globalMap ~= cfg.unknown;
knownIn = known & mask;

numKnownIn = nnz(knownIn);
numTotalIn = nnz(mask);

if numTotalIn == 0
    covPercent = 0;
else
    covPercent = 100 * (numKnownIn / numTotalIn);
end

end
