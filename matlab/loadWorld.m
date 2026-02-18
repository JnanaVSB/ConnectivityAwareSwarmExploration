function world = loadWorld(cfg)
% LOADWORLD  Create an octagonal workspace and polygonal obstacles.
%
%   world.boundary : [Nb x 2] polygon (workspace boundary)
%   world.obstacles: cell array, each cell = [No x 2] polygon

R = cfg.worldRadius;

% Regular octagon, centered at origin, vertices radius R
theta = (0:7)' * (2*pi/8) + pi/8;  % rotated so flat side on top
bx = R * cos(theta);
by = R * sin(theta);
world.boundary = [bx, by];

% Obstacles: depends on cfg.mapID
obs = {};

switch cfg.mapID
    case 1  % sparse (original)
        obs{end+1} = [-3.0  0.5;  -2.0  0.5;  -2.0 -0.5; -3.0 -0.5];
        obs{end+1} = [ 1.0  1.0;  2.0  1.0;  2.0  0.0;  1.0  0.0];

    case 2  % medium (original)
        obs{end+1} = [-1.5  0.5; -0.5  0.5; -0.5 -0.5; -1.5 -0.5];
        obs{end+1} = [ 0.8  1.8;  1.8  1.8;  1.8  0.8;  0.8  0.8];
        obs{end+1} = [-0.3 -1.0;  0.7 -1.0;  0.7 -2.0; -0.3 -2.0];

    case 3  % dense (original)
        obs{end+1} = [-1.5  0.5; -0.5  0.5; -0.5 -0.5; -1.5 -0.5];
        % obs{end+1} = [-3.5  0.5; -2.5  0.5; -2.5 -0.5; -3.5 -0.5];
        obs{end+1} = [ 0.8  1.8;  1.8  1.8;  1.8  0.8;  0.8  0.8];
        obs{end+1} = [-0.3 -1.0;  0.7 -1.0;  0.7 -2.0; -0.3 -2.0];
        obs{end+1} = [-2.0  2.0; -1.0  2.0; -1.0  1.0; -2.0  1.0];
        obs{end+1} = [ 1.5 -1.5;  2.5 -1.5;  2.5 -2.5;  1.5 -2.5];

    case 4  % corridor-style world
        % horizontal corridor
        obs{end+1} = [-4.0  1.0;  4.0  1.0;  4.0  1.5; -4.0  1.5];
        obs{end+1} = [-4.0 -1.0;  4.0 -1.0;  4.0 -1.5; -4.0 -1.5];
        % block near the right to create a trap
        obs{end+1} = [ 2.0 -0.5;  3.0 -0.5;  3.0  0.5;  2.0  0.5];

    case 5  % cluttered / maze-like, but clear space near origin
        % Leave a free zone roughly in [-1,1] x [-1,1]

        % Left vertical wall (x ~ -2.5)
        obs{end+1} = [-3.0  1.5; -2.0  1.5; -2.0 -1.5; -3.0 -1.5];

        % Right vertical wall (x ~ +2.5)
        obs{end+1} = [ 2.0  1.5;  3.0  1.5;  3.0 -1.5;  2.0 -1.5];

        % Top horizontal wall (y ~ +2.5), with a central gap
        obs{end+1} = [-3.0  2.5; -0.7  2.5; -0.7  2.0; -3.0  2.0];
        obs{end+1} = [ 1.7  2.5;  3.0  2.5;  3.0  2.0;  1.7  2.0];

        % Bottom horizontal wall (y ~ -2.5), with a central gap
        % obs{end+1} = [-3.0 -2.0; -0.7 -2.0; -0.7 -2.5; -3.0 -2.5];
        obs{end+1} = [ 0.7 -2.0;  3.0 -2.0;  3.0 -2.5;  0.7 -2.5];

        % A couple of inner blocks, still away from origin
        obs{end+1} = [-1.8  0.3; -1.2  0.3; -1.2 -0.3; -1.8 -0.3];
        obs{end+1} = [ 1.2  0.8;  1.8  0.8;  1.8  0.2;  1.2  0.2];

    otherwise
        warning('Unknown mapID=%d, using sparse obstacles.', cfg.mapID);
        obs{end+1} = [-1.0  0.5;  0.0  0.5;  0.0 -0.5; -1.0 -0.5];
end

world.obstacles = obs;

end
