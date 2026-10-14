function robot = moveRobot(robot, config, dt)
% MOVEROBOT - Smooth speed ramp + position update.
% Note: robot.y lateral movement is handled directly in RobotController
% for RETURNING_HOME state; updateRobotTransform is called there too.

    if robot.speed < robot.targetSpeed
        robot.speed = min(robot.targetSpeed, robot.speed + config.robotAcceleration*dt);
    elseif robot.speed > robot.targetSpeed
        robot.speed = max(robot.targetSpeed, robot.speed - config.robotAcceleration*dt);
    end

    % Only advance X if not in lateral return (controller handles that)
    if robot.state ~= "RETURNING_HOME"
        robot.x = robot.x + robot.speed * dt;
    end

    updateRobotTransform(robot);
end
