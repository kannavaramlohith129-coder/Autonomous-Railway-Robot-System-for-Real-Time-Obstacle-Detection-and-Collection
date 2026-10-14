function updateDashboard(dash, robot, obstacleInfo, trainStatus, decision, trackClear, dayNightMode)
% UPDATEDASHBOARD - Pushes current simulation values into the dashboard labels.

    if ~isgraphics(dash.fig)
        return; % dashboard window was closed by the user
    end

    dash.fields.robotStatus.Text   = char(robot.state);

    % Robot Location string (X distance along track, Y track centerline)
    if isfield(robot,'y') && abs(robot.y - 6.0) < 0.5
        trackName = 'Track 2 (Home)';
    else
        trackName = 'Track 1 (Patrol)';
    end
    dash.fields.robotLocation.Text = sprintf('X=%.1f m (%s)', robot.x, trackName);

    % Pouch capacity string
    if isfield(robot,'pouchCount') && isfield(robot,'pouchMaxItems')
        if isfield(robot,'pouchFull') && robot.pouchFull
            dash.fields.pouchCapacity.Text = sprintf('%d/%d FULL (Plow Fallback)', robot.pouchCount, robot.pouchMaxItems);
            dash.fields.pouchCapacity.FontColor = [0.9 0.1 0.1];
        else
            dash.fields.pouchCapacity.Text = sprintf('%d/%d items (535.5 L)', robot.pouchCount, robot.pouchMaxItems);
            dash.fields.pouchCapacity.FontColor = [0 0.5 0];
        end
    else
        dash.fields.pouchCapacity.Text = '0/4 items';
    end

    dash.fields.currentState.Text  = char(robot.state);
    dash.fields.robotSpeed.Text    = sprintf('%.1f m/s', robot.speed);

    if obstacleInfo.found
        dash.fields.obstacle.Text         = sprintf('DETECTED (#%d)', obstacleInfo.obstacle.id);
        dash.fields.obstacleDistance.Text = sprintf('%.1f m', obstacleInfo.distance);
        
        % Obstacle Type/Category
        obsTypeStr = '';
        if isfield(obstacleInfo.obstacle, 'obstacleType')
            obsTypeStr = upper(strrep(obstacleInfo.obstacle.obstacleType, '_', ' '));
        end
        if ~isempty(obsTypeStr)
            dash.fields.obstacleType.Text = sprintf('%s (%s)', char(obstacleInfo.obstacle.category), obsTypeStr);
        else
            dash.fields.obstacleType.Text = char(obstacleInfo.obstacle.category);
        end

        % Obstacle Risk (displayed ONLY when obstacle is scanned by the sensor)
        if isfield(obstacleInfo.obstacle, 'riskLevel') && isfield(obstacleInfo.obstacle, 'riskScore')
            rLvl = char(obstacleInfo.obstacle.riskLevel);
            rSc  = obstacleInfo.obstacle.riskScore;
            dash.fields.obstacleRisk.Text = sprintf('%s (%.0f/100)', rLvl, rSc);
            switch rLvl
                case 'LOW'
                    dash.fields.obstacleRisk.FontColor = [0 0.65 0];      % Green - Safe/Passable
                case 'MEDIUM'
                    dash.fields.obstacleRisk.FontColor = [0.85 0.50 0];   % Orange - Moderate Risk
                case 'HIGH'
                    dash.fields.obstacleRisk.FontColor = [0.85 0 0];      % Red - Severe Hazard
                otherwise
                    dash.fields.obstacleRisk.FontColor = [0 0 0];
            end
        else
            dash.fields.obstacleRisk.Text = 'SCANNING...';
            dash.fields.obstacleRisk.FontColor = [0.4 0.4 0.4];
        end
    else
        dash.fields.obstacle.Text         = 'NONE';
        dash.fields.obstacleDistance.Text = '-';
        dash.fields.obstacleType.Text     = '-';
        dash.fields.obstacleRisk.Text     = '-';
        dash.fields.obstacleRisk.FontColor = [0.4 0.4 0.4];
    end

    dash.fields.decision.Text = strrep(char(decision), '_', ' ');

    dash.fields.trainDistance.Text = sprintf('%.1f m', trainStatus.distance);
    dash.fields.trainSpeed.Text    = sprintf('%.1f m/s', trainStatus.speed);
    dash.fields.timeToTrain.Text   = sprintf('%.1f s', trainStatus.timeToRobot);

    if trackClear
        dash.fields.trackStatus.Text = 'CLEAR';
    else
        dash.fields.trackStatus.Text = 'BLOCKED';
    end

    dash.fields.safetyStatus.Text = char(trainStatus.level);
    if trainStatus.level == "EMERGENCY"
        dash.fields.safetyStatus.FontColor = [0.8 0 0];
    elseif trainStatus.level == "WARNING"
        dash.fields.safetyStatus.FontColor = [0.85 0.55 0];
    else
        dash.fields.safetyStatus.FontColor = [0 0.6 0];
    end

    dash.fields.environment.Text = char(dayNightMode);
end
