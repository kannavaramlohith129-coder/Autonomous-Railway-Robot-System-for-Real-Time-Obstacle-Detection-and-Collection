function isClear = trackClearanceCheck(robot, obstacles, config)
% TRACKCLEARANCECHECK - True if no unremoved obstacle is sitting in the
% immediate robot safety clearance zone (within 3.5m ahead/behind).
% Used after an obstacle removal/push before the robot is allowed to resume.

    isClear = true;
    clearanceZone = 3.5; % meters ahead/behind robot
    for i = 1:numel(obstacles)
        obs = obstacles(i);
        if ~obs.removed
            d = calculateObstacleDistance(robot, obs);
            inCorridor = abs(obs.position(2) - robot.y) <= config.corridorWidth;
            if d >= -1.5 && d <= clearanceZone && inCorridor
                isClear = false;
                return;
            end
        end
    end
end
