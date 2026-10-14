function log = eventLogger(log, dash, t, robot, obstacleInfo, trainStatus, action)
% EVENTLOGGER - Appends one row to the event log (a struct array) and,
% if a dashboard is provided, prints a short line into its event feed.
%
% log is a struct array with fields:
%   Timestamp, RobotPosition, RobotState, ObstacleID, ObstacleType,
%   ObstacleDistance, TrainDistance, Action, SafetyStatus

    entry.Timestamp = t;
    entry.RobotPosition = robot.x;
    entry.RobotState = char(robot.state);
    if obstacleInfo.found
        entry.ObstacleID = obstacleInfo.obstacle.id;
        entry.ObstacleType = char(obstacleInfo.obstacle.category);
        entry.ObstacleDistance = obstacleInfo.distance;
    else
        entry.ObstacleID = 0;
        entry.ObstacleType = 'NONE';
        entry.ObstacleDistance = NaN;
    end
    entry.TrainDistance = trainStatus.distance;
    entry.Action = char(action);
    entry.SafetyStatus = char(trainStatus.level);

    if isempty(log)
        log = entry;
    else
        log(end+1) = entry;
    end

    if nargin >= 2 && ~isempty(dash) && isgraphics(dash.fig)
        line = sprintf('[%6.2fs] %s | %s', t, entry.RobotState, entry.Action);
        dash.eventLog.Value = [dash.eventLog.Value; {line}];
        scroll(dash.eventLog, 'bottom');
    end
end
