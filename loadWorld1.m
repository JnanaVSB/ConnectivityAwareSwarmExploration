    function world = loadWorld(cfg)
    % LOADWORLD  Create an octagonal workspace and some polygonal obstacles.
    
    R = cfg.worldRadius;
    
    % Regular octagon, centered at origin, vertices radius R
    theta = (0:7)' * (2*pi/8) + pi/8;  % rotated so flat side on top
    bx = R * cos(theta);
    by = R * sin(theta);
    world.boundary = [bx, by];
    
    % Obstacles: depends on cfg.mapID
    obs = {};
    
    switch cfg.mapID
        case 1  % sparse
            obs{end+1} = [-1.0  0.5;  0.0  0.5;  0.0 -0.5; -1.0 -0.5];
            obs{end+1} = [ 1.0  1.0;  2.0  1.0;  2.0  0.0;  1.0  0.0];
    
        case 2  % medium
            obs{end+1} = [-1.5  0.5; -0.5  0.5; -0.5 -0.5; -1.5 -0.5];
            obs{end+1} = [ 0.8  1.8;  1.8  1.8;  1.8  0.8;  0.8  0.8];
            obs{end+1} = [-0.3 -1.0;  0.7 -1.0;  0.7 -2.0; -0.3 -2.0];
    
        case 3  % dense
            obs{end+1} = [-1.5  0.5; -0.5  0.5; -0.5 -0.5; -1.5 -0.5];
            % obs{end+1} = [-3.5  0.5; -2.5  0.5; -2.5 -0.5; -3.5 -0.5];
            obs{end+1} = [ 0.8  1.8;  1.8  1.8;  1.8  0.8;  0.8  0.8];
            obs{end+1} = [-0.3 -1.0;  0.7 -1.0;  0.7 -2.0; -0.3 -2.0];
            obs{end+1} = [-2.0  2.0; -1.0  2.0; -1.0  1.0; -2.0  1.0];
            obs{end+1} = [ 1.5 -1.5;  2.5 -1.5;  2.5 -2.5;  1.5 -2.5];
    
        otherwise
            warning('Unknown mapID, using sparse obstacles.');
            obs{end+1} = [-1.0  0.5;  0.0  0.5;  0.0 -0.5; -1.0 -0.5];
    end
    
    world.obstacles = obs;
    
    end
