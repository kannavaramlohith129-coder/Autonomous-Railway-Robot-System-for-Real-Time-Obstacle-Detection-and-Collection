function d = calculateObstacleDistance(robot, obstacle)
% CALCULATEOBSTACLEDISTANCE - Straight-line forward distance (along the
% track) from the robot's current position to an obstacle.
    d = obstacle.position(1) - robot.x;
end
