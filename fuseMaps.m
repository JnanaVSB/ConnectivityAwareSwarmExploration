function globalMap = fuseMaps(globalMap, localMap, cfg)
% FUSEMAPS  Fuse a robot's local occupancy map into the global map.
%
% Priority:
%   1) Any occupied cell in localMap -> occupied in globalMap
%   2) Free cells in localMap overwrite unknown in globalMap
%   3) Existing non-unknown global cells are preserved unless set to occ.

occVal   = cfg.occVal;
freeVal  = cfg.freeVal;
unknownV = cfg.unknown;

% 1) Occupied dominates
occMask = (localMap == occVal);
globalMap(occMask) = occVal;

% 2) Free overwrites unknown only
freeMask = (localMap == freeVal) & (globalMap == unknownV);
globalMap(freeMask) = freeVal;

% Unknown data does nothing

end
