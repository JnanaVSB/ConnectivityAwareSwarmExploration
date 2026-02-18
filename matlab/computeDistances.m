function dist = computeDistances(robots, world, cfg)
% COMPUTEDISTANCES  Compute geometric distances for potential fields.
%
% Returns struct with:
%   dist.pos    : N x 2 positions
%   dist.D      : N x N pairwise distances
%   dist.db     : N x 1 distance to boundary
%   dist.nb     : N x 2 boundary normals
%   dist.do     : N x nObs distances to obstacles
%   dist.no     : cell{N,nObs} normals to obstacles

N = numel(robots);
pos = zeros(N,2);
for i = 1:N
    pos(i,:) = robots(i).pose(1:2).';
end
dist.pos = pos;

% Pairwise distances
D = zeros(N);
for i = 1:N
    for j = i+1:N
        d = norm(pos(i,:) - pos(j,:));
        D(i,j) = d;
        D(j,i) = d;
    end
end
dist.D = D;

% Boundary distances and normals
db = zeros(N,1);
nb = zeros(N,2);
for i = 1:N
    [db(i), nb(i,:)] = distanceToPolygon(pos(i,:), world.boundary);
end
dist.db = db;
dist.nb = nb;

% Obstacles
nObs = numel(world.obstacles);
dist.do = zeros(N, nObs);
dist.no = cell(N, nObs);

for i = 1:N
    for k = 1:nObs
        poly = world.obstacles{k};
        [d, n] = distanceToPolygon(pos(i,:), poly);
        dist.do(i,k) = d;
        dist.no{i,k} = n;
    end
end

end

% --- helper: distance from point to polygon edges ---
function [d_min, n_hat] = distanceToPolygon(p, poly)
M = size(poly,1);
d_min = inf;
n_hat = [0,0];
for k = 1:M
    a = poly(k,:);
    b = poly(mod(k,M)+1,:);
    [d_seg, proj] = pointSegmentDistance(p, a, b);
    if d_seg < d_min
        d_min = d_seg;
        v = p - proj;
        if d_seg > 1e-12
            n_hat = v / d_seg;
        else
            t = b - a;
            t = t / max(norm(t),1e-12);
            n_hat = [-t(2), t(1)];
        end
    end
end
end

function [d, proj] = pointSegmentDistance(p, a, b)
ab = b - a;
t = dot(p - a, ab) / max(dot(ab,ab), 1e-12);
t = max(0, min(1, t));
proj = a + t * ab;
d = norm(p - proj);
end
