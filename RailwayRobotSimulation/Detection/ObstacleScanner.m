function [scanVis, result] = ObstacleScanner(ax, scanVis, robot, obstacles, config)
% OBSTACLESCANNER - Draws a fan of forward-facing sensor rays (like a
% simple LIDAR sweep) plus the scan-cone outline, and checks every
% obstacle with detectObstacle(). Returns the nearest currently-detected
% obstacle (if any) as `result`.
%
% scanVis is created on the first call (pass [] the first time) and then
% reused/updated on every subsequent call so we don't create new graphics
% objects every frame.
%
% result fields: found (logical), index, distance, obstacle (struct copy)

    numRays = 9;

    if isempty(scanVis) || ~isfield(scanVis,'rays') || any(~isgraphics(scanVis.rays))
        scanVis.cone = patch(ax, nan(1,4), nan(1,4), nan(1,4), [0.2 0.7 1], ...
            'FaceAlpha', 0.06, 'EdgeColor', [0.2 0.7 1], 'EdgeAlpha', 0.3);
        scanVis.rays = gobjects(numRays,1);
        for k = 1:numRays
            scanVis.rays(k) = line(ax, [0 0], [0 0], [0 0], ...
                'Color', [0.35 0.8 0.95], 'LineWidth', 0.6);
        end
        scanVis.hitRay = line(ax, [0 0], [0 0], [0 0], ...
            'Color', [1 0.15 0.1], 'LineWidth', 2.2, 'Visible', 'off');
        scanVis.hitMarker = plot3(ax, nan, nan, nan, 'o', ...
            'MarkerSize', 7, 'MarkerFaceColor', [1 0.2 0.1], 'MarkerEdgeColor', 'none');
        scanVis.distText = text(ax, 0,0,0, '', 'Color', [1 1 0], 'FontWeight','bold', 'Interpreter','none');
    end

    % ---- Scanner sits at the front of the robot body, at sensor height ----
    range = config.scannerRange;
    fov = config.scannerFOV;
    originX = robot.x + 0.75;
    originY = robot.y;
    originZ = 0.55;

    % ---- Scan-cone outline (context/visual only) ----
    leftY  = originY + range*tand(fov);
    rightY = originY - range*tand(fov);
    coneX = [originX, originX+range, originX+range, originX];
    coneY = [originY,  leftY,        rightY,        originY];
    coneZ = [0.02 0.02 0.02 0.02];
    set(scanVis.cone, 'XData', coneX, 'YData', coneY, 'ZData', coneZ);

    % ---- Fan of individual rays across the field of view ----
    rayAngles = linspace(-fov, fov, numRays);
    for k = 1:numRays
        endX = originX + range*cosd(rayAngles(k));
        endY = originY + range*sind(rayAngles(k));
        set(scanVis.rays(k), 'XData', [originX endX], 'YData', [originY endY], ...
            'ZData', [originZ originZ]);
    end

    % ---- Check every obstacle for the nearest actual detection ----
    result.found = false;
    result.index = 0;
    result.distance = inf;
    result.obstacle = [];

    for i = 1:numel(obstacles)
        obs = obstacles(i);
        if detectObstacle(robot, obs, config)
            d = calculateObstacleDistance(robot, obs);
            if d < result.distance
                result.found = true;
                result.index = i;
                result.distance = d;
                result.obstacle = obs;
            end
        end
    end

    % ---- Highlight the ray that actually found something ----
    if result.found
        % ---- Sensor Perception: Evaluate obstacle risk & category on the fly ----
        [riskLvl, riskSc, riskRsn] = assessObstacleRisk(result.obstacle, config);
        cat = classifyObstacle(result.obstacle, config);
        result.obstacle.riskLevel  = riskLvl;
        result.obstacle.riskScore  = riskSc;
        result.obstacle.riskReason = riskRsn;
        result.obstacle.category   = cat;
        result.riskLevel  = riskLvl;
        result.riskScore  = riskSc;
        result.riskReason = riskRsn;
        result.category   = cat;

        hitX = result.obstacle.position(1);
        hitY = result.obstacle.position(2);
        hitZ = originZ;

        switch riskLvl
            case "LOW";    rayClr = [0.10 0.90 0.20];   % bright green  - safe / low risk
            case "MEDIUM"; rayClr = [1.00 0.60 0.00];   % orange        - moderate risk
            case "HIGH";   rayClr = [0.95 0.08 0.08];   % red           - high risk danger
            otherwise;     rayClr = [1.00 0.15 0.10];
        end

        set(scanVis.hitRay, 'XData', [originX hitX], 'YData', [originY hitY], ...
            'ZData', [originZ hitZ], 'Color', rayClr, 'Visible', 'on');
        set(scanVis.hitMarker, 'XData', hitX, 'YData', hitY, 'ZData', hitZ, ...
            'MarkerFaceColor', rayClr, 'Visible', 'on');
        set(scanVis.distText, 'Position', [hitX, hitY+0.5, 1.0], ...
            'String', sprintf('%.1fm [%s %.0f/100]', result.distance, riskLvl, riskSc), ...
            'Color', rayClr, 'Visible', 'on');

        % Reveal clean obstacle name on 3D floating label without weight/mass
        if result.index >= 1 && result.index <= numel(obstacles) && ...
           isfield(obstacles(result.index), 'label') && isgraphics(obstacles(result.index).label)
            switch lower(char(result.obstacle.obstacleType))
                case 'mud'; cleanName = 'Mud';
                case {'small_rock', 'rock', 'big_rock'}; cleanName = 'Rock';
                case {'small_log', 'log'}; cleanName = 'Log';
                case 'stone'; cleanName = 'Stone';
                otherwise; cleanName = char(result.obstacle.obstacleType);
            end
            set(obstacles(result.index).label, ...
                'String', cleanName, ...
                'FontWeight', 'normal', 'FontSize', 7.5, ...
                'BackgroundColor', rayClr * 0.35);
        end
    else
        set(scanVis.hitRay, 'Visible', 'off');
        set(scanVis.hitMarker, 'Visible', 'off');
        set(scanVis.distText, 'Visible', 'off');
    end
end
