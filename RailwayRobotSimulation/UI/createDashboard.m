function dash = createDashboard()
% CREATEDASHBOARD - Builds a small always-on-top uifigure showing live
% robot/obstacle/train/safety status, separate from the 3D scene window.

    % Position dashboard on right side of screen (full height, no arm graph)
    screenSz = get(0, 'ScreenSize');
    scrW = screenSz(3); scrH = screenSz(4);
    rightX = round(scrW * 0.63) + 10;   % just right of simulation figure
    dashW  = scrW - rightX - 10;
    dashH  = round(scrH * 0.88);        % full right column height
    dashY  = scrH - dashH - 60;         % positioned at top-right
    dash.fig = uifigure('Name', 'Autonomous Railway Safety Robot - Dashboard', ...
        'Position', [rightX dashY dashW dashH]);

    title = uilabel(dash.fig, 'Text', 'AUTONOMOUS RAILWAY SAFETY ROBOT', ...
        'FontWeight','bold', 'FontSize', 14, 'Position', [10 dashH-35 dashW-20 25], ...
        'HorizontalAlignment','center');
    dash.title = title;

    labels = {'Robot Status', 'Robot Location', 'Pouch Capacity', 'Current State', 'Robot Speed', ...
              'Obstacle', 'Obstacle Distance', 'Obstacle Type', 'Obstacle Risk', 'Decision', ...
              'Train Distance', 'Train Speed', 'Time to Train', ...
              'Track Status', 'Safety Status', 'Environment'};

    dash.fields = struct();
    y = dashH - 65;   % start below title, relative to dashboard height
    valW = max(dashW - 170, 180);   % value label width adapts to dashboard width
    fieldNames = {'robotStatus', 'robotLocation', 'pouchCapacity', 'currentState', 'robotSpeed', ...
                  'obstacle', 'obstacleDistance', 'obstacleType', 'obstacleRisk', 'decision', ...
                  'trainDistance', 'trainSpeed', 'timeToTrain', ...
                  'trackStatus', 'safetyStatus', 'environment'};

    for i = 1:numel(labels)
        uilabel(dash.fig, 'Text', [labels{i} ':'], 'Position', [10 y 140 22]);
        dash.fields.(fieldNames{i}) = uilabel(dash.fig, 'Text', '-', ...
            'Position', [155 y valW 22], 'FontWeight','bold');
        y = y - 25;
    end

    uilabel(dash.fig, 'Text', 'Recent Events:', 'FontWeight','bold', ...
        'Position', [10 y-2 200 22]);
    y = y - 24;
    taH = max(y - 10, 30);   % ensure height is always positive
    dash.eventLog = uitextarea(dash.fig, 'Position', [10 10 dashW-20 taH], ...
        'Editable', 'off', 'Value', {'Simulation ready.'});
end
