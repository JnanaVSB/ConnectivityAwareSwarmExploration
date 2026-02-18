function exploreField = computeExplorationField(globalMap, cfg, varargin)
% COMPUTEEXPLORATIONFIELD  Build smooth exploration field from occupancy.
%
%   exploreField = computeExplorationField(globalMap, cfg)
%   exploreField = computeExplorationField(globalMap, cfg, world)
%
%   globalMap: cfg.unknown / cfg.freeVal / cfg.occVal
%   exploreField: same size, higher = more attractive to explore
%
% This implements a smoothed entropy-like field:
%   - unknown cells: high weight
%   - free-known cells: smaller weight
%   - occupied: zero (not attractive)
%
% If the config is missing fields, reasonable defaults are used.

% Optional world (for boundary masking, if desired)
world = [];
if ~isempty(varargin)
    world = varargin{1};
end

% --- Safe defaults for config fields, in case config.m is older ---
if ~isfield(cfg, 'useExplorationField');  cfg.useExplorationField = true;     end
if ~isfield(cfg, 'weightUnknown');        cfg.weightUnknown       = 1.0;      end
if ~isfield(cfg, 'weightFree');           cfg.weightFree          = 0.2;      end
if ~isfield(cfg, 'weightOcc');            cfg.weightOcc           = 0.0;      end
if ~isfield(cfg, 'exploreKernelRadius');  cfg.exploreKernelRadius = 3;        end
if ~isfield(cfg, 'exploreKernelSigma');   cfg.exploreKernelSigma  = 1.5;      end

if ~cfg.useExplorationField
    exploreField = [];
    return;
end

% --- 1) Base field from occupancy labels ---
field = zeros(size(globalMap));

field(globalMap == cfg.unknown) = cfg.weightUnknown;
field(globalMap == cfg.freeVal) = cfg.weightFree;
field(globalMap == cfg.occVal)  = cfg.weightOcc;

% --- 2) Optional boundary mask (if world.boundary exists) ---
if ~isempty(world) && isfield(world, 'boundary') && ~isempty(world.boundary)
    N   = cfg.mapSize;
    res = cfg.mapResolution;
    L   = N * res;
    xMin = -L/2;
    yMin = -L/2;

    [Xc, Yc] = meshgrid(0:N-1, 0:N-1);
    Xw = xMin + (Xc + 0.5)*res;
    Yw = yMin + (Yc + 0.5)*res;

    in = inpolygon(Xw, Yw, world.boundary(:,1), world.boundary(:,2));
    field(~in) = 0;
end

% --- 3) Gaussian smoothing in grid space (kernel radius + sigma in cells) ---
R       = cfg.exploreKernelRadius;
sigma_c = cfg.exploreKernelSigma;

if R > 0 && sigma_c > 0
    sz = 2*R + 1;
    [Xg, Yg] = meshgrid(-R:R, -R:R);
    G = exp(-(Xg.^2 + Yg.^2)/(2*sigma_c^2));
    G = G / sum(G(:));
    exploreField = conv2(field, G, 'same');
else
    exploreField = field;
end

end
