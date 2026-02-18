function [lambda2, minDeg] = computeConnectivity(robots, cfg)
% COMPUTECONNECTIVITY  Algebraic connectivity and min degree.

N = numel(robots);
pos = zeros(N,2);
for i = 1:N
    pos(i,:) = robots(i).pose(1:2).';
end

R = cfg.commRange;

A = zeros(N);
for i = 1:N
    for j = i+1:N
        d = norm(pos(i,:) - pos(j,:));
        if d <= R
            A(i,j) = 1;
            A(j,i) = 1;
        end
    end
end

deg = sum(A,2);
minDeg = min(deg);

L = diag(deg) - A;
eigVals = sort(eig(L));
if numel(eigVals) >= 2
    lambda2 = eigVals(2);
else
    lambda2 = 0;
end

end
