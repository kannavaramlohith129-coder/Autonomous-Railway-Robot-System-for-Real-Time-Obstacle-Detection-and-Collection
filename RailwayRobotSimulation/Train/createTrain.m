function train = createTrain(ax, config)
% CREATETRAIN - Builds a simple train (engine + 2 coaches) that will
% travel along the same track as the robot, approaching from behind.

    train.mainTransform = hgtransform('Parent', ax);

    train.engine = buildCar(train.mainTransform, 0,    [0.7 0.1 0.1], 0.55);
    train.coach1 = buildCar(train.mainTransform, -2.2, [0.2 0.3 0.6], 0.45);
    train.coach2 = buildCar(train.mainTransform, -4.2, [0.2 0.3 0.6], 0.45);

    train.x = config.robotStartX - config.trainStartDistance;
    train.y = 0;
    train.z = 0;
    train.speed = config.trainSpeed;    % nominal top speed
    train.actualSpeed = 0;              % actually-achieved speed (0 while parked)
    train.moving = false;               % main.m decides when/if this train runs

    updateTrainTransform(train);
end

function h = buildCar(parentT, xOffset, color, height)
    L = 1.8; W = 0.9;
    x = [-L/2 L/2]; y = [-W/2 W/2]; z = [0 height];
    [X,Y,Z] = ndgrid(x,y,z);
    verts = [X(:)+xOffset, Y(:), Z(:)];
    faces = [1 2 4 3; 5 6 8 7; 1 2 6 5; 3 4 8 7; 1 3 7 5; 2 4 8 6];
    h = patch('Parent', parentT, 'Vertices', verts, 'Faces', faces, ...
        'FaceColor', color, 'EdgeColor', [0.1 0.1 0.1]);
end
