function plotLocalMaps(robots, cfg)
% PLOTLOCALMAPS  Show each robot's local occupancy grid.

N = numel(robots);
nCols = min(N, 3);
nRows = ceil(N / nCols);

figure('Name','Local occupancy maps','NumberTitle','off'); clf;
tiledlayout(nRows, nCols, 'Padding','compact','TileSpacing','compact');

for i = 1:N
    nexttile;
    imagesc(robots(i).map);
    axis image off;
    colormap(gray);
    title(sprintf('Robot %d local map', robots(i).id), 'FontSize',10);
end

end
