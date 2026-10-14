function detected = detectObstacle(robot, obstacle, config)
% DETECTOBSTACLE - True if this obstacle is inside the scanner's range,
% field of view, and the railway safety corridor (i.e. actually on the
% track ahead, not just scenery beside it).

    if obstacle.removed
        detected = false;
        return;
    end

    d = calculateObstacleDistance(robot, obstacle);

    inRange   = (d > 0) && (d <= config.scannerRange);
    inCorridor = abs(obstacle.position(2) - robot.y) <= config.corridorWidth;

    % Angle of the obstacle relative to the robot's forward heading
    angleToObstacle = atan2d(obstacle.position(2) - robot.y, d);
    inFOV = abs(angleToObstacle) <= config.scannerFOV;

    detected = inRange && inCorridor && inFOV;
end
