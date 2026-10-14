function train = moveTrain(train, robot, config, dt)
% MOVETRAIN - Moves the train forward along +X at its current speed,
% unless train.moving has been set false (e.g. it has been signalled to
% halt for safety).
%
% The train can never occupy the same stretch of track as the robot: its
% position is clamped so it always stays at least config.trainMinGap
% behind the robot. Without this, a fast train started far behind a slow
% robot would silently overtake the robot's X position in this simplified
% 1D model, which previously caused a permanent false "0 seconds to
% impact" reading once the robot later stopped for an obstacle.
%
% train.actualSpeed records what speed was actually achieved THIS frame
% (0 while parked, less than train.speed while pinned against the clamp,
% train.speed while freely closing). trainSafetyManager.m uses this
% actual speed - not the nominal train.speed - so a train that has
% caught up and is simply riding along behind the robot correctly reads
% as "not currently closing" instead of a permanent false alarm.

    prevX = train.x;

    if train.moving
        train.x = train.x + train.speed * dt;
    end

    maxX = robot.x - config.trainMinGap;
    if train.x > maxX
        train.x = maxX;
    end

    train.actualSpeed = (train.x - prevX) / dt;

    updateTrainTransform(train);
end
