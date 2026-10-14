function status = trainSafetyManager(train, robot, config)
% TRAINSAFETYMANAGER - Monitors the train relative to the robot and
% returns a status struct describing the current safety situation.
%
% status fields:
%   distance        metres from train to robot (never negative - the
%                   train physically cannot pass through the robot;
%                   see moveTrain.m, which enforces this)
%   closingSpeed    train speed relative to the robot (m/s); <=0 means
%                   the train isn't currently gaining on the robot
%   timeToRobot     seconds until the train reaches the robot AT THE
%                   CURRENT closing rate - Inf if it isn't closing
%   level           "SAFE" | "WARNING" | "EMERGENCY"
%   message         human-readable status string for the dashboard/log
%
% Using the RELATIVE (closing) speed rather than the train's raw speed
% is the key fix that avoids a false "0 seconds to impact" once a fast
% train has caught up and is simply riding along behind a moving robot:
% at that point its speed relative to the robot is ~0, so it correctly
% reads as SAFE right up until the robot actually slows or stops, at
% which point the closing speed - and the real danger - reappears.

    status.distance = max(robot.x - train.x, 0);
    status.speed = train.actualSpeed;

    status.closingSpeed = train.actualSpeed - robot.speed;
    if status.closingSpeed > 0
        status.timeToRobot = status.distance / status.closingSpeed;
    else
        status.timeToRobot = inf;
    end

    if status.timeToRobot <= config.trainDangerTime
        status.level = "EMERGENCY";
        status.message = "TRAIN EMERGENCY: robot must clear the track immediately";
    elseif status.distance <= config.trainWarnDistance
        status.level = "WARNING";
        status.message = "Train approaching - robot should hurry / prepare to stop";
    else
        status.level = "SAFE";
        status.message = "Train at safe distance";
    end
end
