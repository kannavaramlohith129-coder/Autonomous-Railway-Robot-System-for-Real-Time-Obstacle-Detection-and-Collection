%% AUTONOMOUS RAILWAY SAFETY ROBOT - DUAL TRACK SIMULATION
function main(selectedScenario)
    clc; close all force;
    addpath(genpath(fileparts(mfilename('fullpath'))));
    config = createConfig();

    scenarioNames = {
        'No obstacles (patrol only)', ...
        'Single removable obstacle', ...
        'Single heavy obstacle (push)', ...
        'Multiple obstacles (mixed)', ...
        'FULL MISSION: Heavy + Train + Return Home', ...
        'Night patrol'};
    scenarioIds = ["none","single_removable","single_heavy","multiple","heavy_plus_train","night"];

    if nargin < 1 || isempty(selectedScenario)
        choice = menu('SELECT SIMULATION SCENARIO', scenarioNames{:});
        if choice == 0; choice = 5; end
        scenario = scenarioIds(choice);
    else
        scenario = string(selectedScenario);
    end
    config.scenario          = scenario;
    config.enableTrackSwitch = true;   % ALL operations return home to Track 2!
    if scenario == "night";            config.dayNightMode = "NIGHT"; end
    if scenario == "heavy_plus_train"; config.trainStartDistance = 110; end

    %% Figure
    % --- Get screen size for adaptive single-screen layout ---
    screenSz = get(0, 'ScreenSize');
    scrW = screenSz(3); scrH = screenSz(4);
    simW = round(scrW * 0.62);  % simulation takes ~62% width
    simH = round(scrH * 0.92);
    fig = figure('Name','Railway Safety Robot - Dual Track Simulation', ...
        'Color',[0.06 0.08 0.12], 'Position',[5, scrH-simH-60, simW, simH], ...
        'Renderer','opengl', 'DoubleBuffer','on');

    % Axes - NO blue sky-color background; use dark grey so any gaps are invisible
    ax = axes('Parent',fig,'Color',[0.32 0.50 0.24]);  % matches grass color
    hold(ax,'on'); axis(ax,'equal'); grid(ax,'off');
    xlim(ax,[-15, config.trackLength+12]);
    ylim(ax,[-7,  config.track2Y+9]);
    zlim(ax,[-0.2, 14]);
    xlabel(ax,'Track Distance (m)','Color',[0.9 0.9 0.9],'FontWeight','bold');
    ylabel(ax,'Lateral (m)',       'Color',[0.9 0.9 0.9],'FontWeight','bold');
    zlabel(ax,'Height (m)',        'Color',[0.9 0.9 0.9],'FontWeight','bold');
    view(ax,-28,20); camva(ax,36);
    set(ax,'Projection','perspective');

    % Studio lighting
    sceneLight = light('Parent',ax,'Style','infinite','Position',[-8,-14,22],'Color',[1.00 0.97 0.88]);
    light('Parent',ax,'Style','infinite','Position',[18, 10, 9],'Color',[0.40 0.48 0.65]);
    light('Parent',ax,'Style','infinite','Position',[0,-18, 5],'Color',[0.70 0.82 1.00]);
    lighting(ax,'gouraud');

    %% Scene
    createSky(ax,config);
    createGround(ax,config);
    createRailway(ax,config);
    createRailwayObjects(ax,config);

    dayNight.mode        = config.dayNightMode;
    dayNight.lightHandle = sceneLight;
    dayNight.autoCycle   = config.autoCycle;
    dayNight.cyclePeriod = config.cyclePeriod;
    dayNight.timer       = 0;
    dayNight = DayNightController(ax, dayNight, dayNight.mode);

    robot     = createRobot(ax, config);
    robot.switchState = [];   % Track-switch state (initialized on first use)
    robot.ax = ax;            % Store axes handle for track-switch animation
    arm       = createManipulator(robot);
    train     = createTrain(ax, config);
    obstacles = spawnObstacles(ax, config, scenario);
    scanVis   = [];

    % Return-home arrow indicator (hidden until needed)
    homeArrow = annotation(fig,'arrow',[0.5 0.55],[0.15 0.20],...
        'Color',[0.1 0.95 0.35],'LineWidth',3,'HeadStyle','vback2',...
        'HeadWidth',14,'HeadLength',12,'Visible','off');

    dash = createDashboard();
    log  = [];
    trainStatus = struct('distance',inf,'timeToRobot',inf,'level',"SAFE",'speed',train.speed);
    log = eventLogger(log,dash,0,robot,struct('found',false),trainStatus,"SIMULATION_STARTED");

    % HUD
    armHUD  = buildArmHUD(ax);
    mainHUD = buildMainHUD(fig);
    mlHUD   = buildMLHUD(fig);        % ML Analysis & optimizer panel

    % ARM ANGLE GRAPH FIGURE removed for performance

    % Train scripting
    trainActive  = (scenario == "heavy_plus_train");
    train.moving = false;
    trainStarted = false;
    trainHalted  = false;

    t         = 0;
    dt        = config.dt;
    lastDashT = -1;
    lastMLT   = -1;    % ML analyzer throttle timer
    mlResult  = [];    % most recent ML analysis result

    while isgraphics(fig) && t < config.maxSimTime && robot.state ~= "COMPLETED"

        dayNight = DayNightController(ax, dayNight, [], dt);

        [scanVis, obstacleInfo] = ObstacleScanner(ax, scanVis, robot, obstacles, config);
        trainStatus = trainSafetyManager(train, robot, config);

        if trainActive
            if ~trainStarted && any(robot.state == ["PUSHING","WARNING","WAITING","REMOVING"])
                trainStarted = true;
            end
            if trainStarted && ~trainHalted && trainStatus.level == "EMERGENCY"
                trainHalted = true;
            end
            train.moving = trainStarted && ~trainHalted;
        end

        % Train brake
        if trainStatus.level == "EMERGENCY" || trainHalted
            train.targetSpeed = 0;
        else
            train.targetSpeed = train.speed;
        end

        [robot,arm,obstacles,log,action] = RobotController(robot,arm,obstacles,...
            obstacleInfo,trainStatus,config,dt,log,dash,t); %#ok<ASGLU>

        robot = moveRobot(robot, config, dt);
        train = moveTrain(train, robot, config, dt);

        % Headlights
        nightMode = (dayNight.mode == "NIGHT");
        if isfield(robot,'headlightL') && isgraphics(robot.headlightL)
            set(robot.headlightL,'Visible',vis(nightMode));
            set(robot.headlightR,'Visible',vis(nightMode));
        end

        % Home arrow + track switch announcement
        if robot.state == "RETURNING_HOME" && isgraphics(homeArrow)
            set(homeArrow,'Visible','on');
            % Show one-time "TRACK SWITCH" banner
            if ~isfield(robot,'switchAnnounced') || ~robot.switchAnnounced
                robot.switchAnnounced = true;
                text(ax, robot.x+2, (config.track1Y+config.track2Y)/2, 3.8, ...
                    '  ► TRACK SWITCH: Returning to Home Track  ◄  ', ...
                    'Color',[0.1 1.0 0.5], 'FontSize',11, 'FontWeight','bold', ...
                    'BackgroundColor',[0.02 0.12 0.05], 'Margin',7, ...
                    'HorizontalAlignment','center', 'EdgeColor',[0.1 0.85 0.3], ...
                    'Tag','SwitchBanner');
            end
        elseif isgraphics(homeArrow) && robot.state ~= "RETURNING_HOME"
            set(homeArrow,'Visible','off');
        end

        % ---- Camera Tracking with Exponential Moving Average Filter ----
        if robot.state == "PUSHING"
            camTargetPos = [robot.x + 4.5,  robot.y - 4.0,  2.8];
            camTargetTgt = [robot.x + 0.5,  robot.y + 0.5,  0.2];
        elseif any(robot.state == ["REMOVING","STOPPED"])
            camTargetPos = [robot.x - 4.5, robot.y - 5.5, 5.0];
            camTargetTgt = [robot.x + 1.5, robot.y,        0.5];
        elseif robot.state == "RETURNING_HOME"
            camTargetPos = [robot.x - 7, (robot.y+config.track2Y)/2 - 5, 6.5];
            camTargetTgt = [robot.x + 2, config.track2Y, 0.5];
        else
            camTargetPos = [robot.x - 10, robot.y - 8, 6.5];
            camTargetTgt = [robot.x +  5, robot.y,     0.8];
        end

        % Exponential low-pass filter eliminates camera stutter & jitter
        if ~exist('currCamPos','var') || isempty(currCamPos)
            currCamPos = camTargetPos;
            currCamTgt = camTargetTgt;
        else
            currCamPos = currCamPos * 0.84 + camTargetPos * 0.16;
            currCamTgt = currCamTgt * 0.84 + camTargetTgt * 0.16;
        end
        campos(ax, currCamPos); camtarget(ax, currCamTgt);
        if isgraphics(sceneLight)
            set(sceneLight,'Position',[robot.x-8, robot.y-12, 22]);
        end

        % Throttled UI overlays to prevent UI thread lock
        if ~exist('lastHudT','var'); lastHudT = -1; lastGraphT = -1; end
        if (t - lastHudT) >= 0.15
            updateArmHUD(armHUD, robot);
            updateMainHUD(mainHUD, robot, obstacles, obstacleInfo, config);
            lastHudT = t;
        end

        % Arm graph update removed (real-time monitor disabled)

        % ---- ML Analysis: runs at ~0.5 Hz (every 2.0 sim-second) to reduce load ----
        robot.lastTrainDist = trainStatus.distance;
        if (t - lastMLT) >= 2.0
            lastMLT  = t;
            mlResult = ObstacleRemovalMLAnalyzer(robot, arm, obstacles, config, t);
            updateMLHUD(mlHUD, mlResult, t);
        end

        % Throttle App Designer dashboard refresh to ~3 Hz to eliminate UI buffering stalls
        if (t - lastDashT) >= 0.33
            trackClear = trackClearanceCheck(robot, obstacles, config);
            decision   = safetyDecision(obstacleInfo, trainStatus, robot.state);
            updateDashboard(dash, robot, obstacleInfo, trainStatus, decision, trackClear, dayNight.mode);
            lastDashT = t;
        end

        % Hardware-accelerated smooth rendering (no pause to maximize speed)
        drawnow limitrate;
        t = t + dt * config.simulationSpeed;

        % Completed state handling: show banner on scene and cleanly finish after dwell
        if robot.state == "COMPLETED"
            if ~isfield(robot, 'completedBannerShown') || ~robot.completedBannerShown
                robot.completedBannerShown = true;
                robot.completedDwellTimer  = 0;
                showMissionBanner(ax, robot, config);
            else
                robot.completedDwellTimer = robot.completedDwellTimer + dt;
                if robot.completedDwellTimer > 4.5
                    break;
                end
            end
        end

        % End-of-track check: only enter COMPLETED state if not already switching tracks
        % (RETURNING_HOME is handled by state machine when allClear, not here)
        if robot.x > config.trackLength - 5 && robot.state == "MOVING"
            numLeft = 0;
            for k = 1:numel(obstacles)
                if ~isfield(obstacles(k),'removed') || ~isequal(obstacles(k).removed, true)
                    numLeft = numLeft + 1;
                end
            end
            if numLeft > 0 || numel(obstacles) == 0 || ~config.enableTrackSwitch
                robot.state = "COMPLETED";
            end
        end
    end

    if isgraphics(fig)
        log = eventLogger(log,dash,t,robot,struct('found',false),trainStatus,"SIMULATION_ENDED");
        if any(robot.state == ["COMPLETED","RETURNING_HOME"])
            showMissionBanner(ax, robot, config);
        end
    end

    try
        if ~isempty(log)
            dataDir = fullfile(fileparts(mfilename('fullpath')),'Data');
            if ~exist(dataDir,'dir'); mkdir(dataDir); end
            writetable(struct2table(log), config.logFile);
        end
    catch; end

    % Launch Two-Link Manipulator Analysis if an obstacle was manipulated
    if isfield(robot, 'pickEvent') && isfield(robot.pickEvent, 'hasPicked') && robot.pickEvent.hasPicked
        try
            fprintf('\n>>> Displaying Two-Link Manipulator Analysis Figures (Figures 2 to 7)...\n');
            plotTwoLinkManipulatorMovement(robot, 'Dock', true, 'Animate', false);
        catch ME
            fprintf('Notice: Could not launch arm analysis: %s\n', ME.message);
        end
    end
end

function hud = buildArmHUD(ax)
    hud.phaseBox = text(ax,0,0,0,'',...
        'Color',[1 0.92 0.1],'FontSize',12,'FontWeight','bold',...
        'BackgroundColor',[0.04 0.06 0.10],'Margin',7,...
        'HorizontalAlignment','center','VerticalAlignment','top',...
        'Visible','off','EdgeColor',[0.45 0.45 0.55]);
    hud.stateBox = text(ax,0,0,0,'',...
        'Color',[0.3 0.9 1.0],'FontSize',10,'FontWeight','bold',...
        'BackgroundColor',[0.04 0.06 0.10],'Margin',5,...
        'HorizontalAlignment','center','VerticalAlignment','top',...
        'Visible','off','EdgeColor',[0.3 0.6 0.8]);
end

function updateArmHUD(hud, robot)
    inOp = any(robot.state == ["REMOVING","PUSHING","STOPPED"]);
    if inOp
        sub = robot.armSubState;
        if sub == ""; sub = "READY"; end
        if robot.state == "PUSHING" || startsWith(string(sub), "PLOW")
            hudStr = sprintf(' PLOW: %s ', sub);
            boxCol = [1.0 0.65 0.1];
        else
            hudStr = sprintf(' ARM: %s ', sub);
            boxCol = [1.0 0.92 0.1];
        end
        if isgraphics(hud.phaseBox)
            set(hud.phaseBox,'Position',[robot.x+1.8, robot.y, 5.5],...
                'String',hudStr,'Color',boxCol,'Visible','on');
        end
        if isgraphics(hud.stateBox)
            set(hud.stateBox,'Position',[robot.x+1.8, robot.y, 4.6],...
                'String',sprintf(' STATE: %s ',robot.state),'Visible','on');
        end
    else
        if isgraphics(hud.phaseBox); set(hud.phaseBox,'Visible','off'); end
        if isgraphics(hud.stateBox); set(hud.stateBox,'Visible','off'); end
    end
end

function showMissionBanner(ax, robot, config)
    text(ax, robot.x, config.track2Y, 4.0, ...
        '  MISSION COMPLETE!  All obstacles cleared. Robot at home track.  ',...
        'Color',[0.05 0.98 0.35],'FontSize',15,'FontWeight','bold',...
        'BackgroundColor',[0.02 0.10 0.04],'Margin',10,...
        'HorizontalAlignment','center','EdgeColor',[0.1 0.9 0.2]);
    drawnow;
end

function v = vis(tf)
    if tf; v='on'; else; v='off'; end
end

% (Arm graph figure and functions removed for cleaner single-screen layout)

% =========================================================================
function hud = buildMainHUD(fig)
% BUILDMAINHUD - Builds overlay box on 3D scene displaying live robot position,
% speed, pouch fill, target obstacle with RISK LEVEL, and progress.
% Positioned as a narrow TOP STRIP so it never covers the robot.
    hud.panel = uicontrol(fig, 'Style','text', ...
        'Units','normalized', 'Position',[0.005, 0.87, 0.55, 0.125], ...
        'BackgroundColor',[0.03 0.05 0.10 ], 'ForegroundColor',[0.90 0.95 1.00], ...
        'FontSize', 8.5, 'FontName','Consolas', 'FontWeight','bold', ...
        'HorizontalAlignment','left');
end

function updateMainHUD(hud, robot, obstacles, obstacleInfo, config)
% UPDATEMAINHUD - Refreshes location and status overlay on 3D figure frame.
    if ~isgraphics(hud.panel); return; end

    if isfield(robot,'y') && abs(robot.y - config.track2Y) < 0.5
        trName = 'Track 2 (Home)';
    else
        trName = 'Track 1 (Patrol)';
    end

    pCount = 0; pMax = 4; pFull = false;
    if isfield(robot,'pouchCount'); pCount = robot.pouchCount; end
    if isfield(robot,'pouchMaxItems'); pMax = robot.pouchMaxItems; end
    if isfield(robot,'pouchFull'); pFull = robot.pouchFull; end

    if pFull
        pouchStr = sprintf('%d/%d FULL (Front-Leveler Push Active)', pCount, pMax);
    else
        pouchStr = sprintf('%d/%d Items (535.5 L Volume)', pCount, pMax);
    end

    if obstacleInfo.found
        obs = obstacleInfo.obstacle;
        riskLvl = 'MEDIUM';
        riskSc  = 50;
        if isfield(obs,'riskLevel'); riskLvl = char(obs.riskLevel); end
        if isfield(obs,'riskScore'); riskSc  = obs.riskScore; end

        switch lower(char(obs.obstacleType))
            case 'mud'; obsTypeStr = 'Mud';
            case {'small_rock', 'rock', 'big_rock'}; obsTypeStr = 'Rock';
            case {'small_log', 'log'}; obsTypeStr = 'Log';
            case 'stone'; obsTypeStr = 'Stone';
            otherwise; obsTypeStr = char(obs.obstacleType);
        end

        if strcmp(riskLvl, 'LOW') && ~strcmpi(obsTypeStr, 'Mud')
            actionDesc = 'SAFE -> PATROL SKIP';
        elseif strcmpi(obsTypeStr, 'Mud')
            actionDesc = 'FRONT LEVELER SCRAPE';
        elseif strcmp(obs.category, 'HEAVY')
            actionDesc = 'FRONT PLOW PUSH';
        else
            actionDesc = 'ARM PICK INTO POUCH';
        end

        obsStr = sprintf('%s | RISK: %s (%.0f/100)\n    -> ACTION: %s | Dist: %.1fm', ...
            obsTypeStr, riskLvl, riskSc, actionDesc, obstacleInfo.distance);
    else
        obsStr = 'Track Clear - No Obstacle Ahead';
    end

    clearedCount = 0;
    skippedCount = 0;
    for k = 1:numel(obstacles)
        if isfield(obstacles(k),'removed') && isequal(obstacles(k).removed, true)
            clearedCount = clearedCount + 1;
        end
        if isfield(obstacles(k),'skipped') && isequal(obstacles(k).skipped, true)
            skippedCount = skippedCount + 1;
        end
    end
    totalObs = numel(obstacles);

    line0 = '╔══════════ ROBOT MISSION TELEMETRY ══════════╗';
    line1 = sprintf('📍 LOCATION:  X=%5.1fm | Y=%3.1fm (%s)', robot.x, robot.y, trName);
    line2 = sprintf('⚡ STATE:     %-15s | Speed: %3.1f m/s', robot.state, robot.speed);
    line3 = sprintf('📦 POUCH:     %s', pouchStr);
    line4 = sprintf('🎯 TARGET:    %s', obsStr);
    line5 = sprintf('📊 PROGRESS:  %d Removed | %d Low-Risk Skipped | %d Total', ...
        clearedCount-skippedCount, skippedCount, totalObs);

    set(hud.panel, 'String', sprintf('%s\n%s\n%s\n%s\n%s\n%s', line0, line1, line2, line3, line4, line5));
end

% =========================================================================
function hud = buildMLHUD(fig)
% BUILDMLHUD - ML Analysis and Optimizer status panel.
% Positioned at BOTTOM-LEFT, raised above axis labels so all lines are visible.
    hud.panel = uicontrol(fig, 'Style','text', ...
        'Units','normalized', 'Position',[0.005, 0.05, 0.55, 0.17], ...
        'BackgroundColor',[0.04 0.02 0.10], 'ForegroundColor',[0.70 0.95 0.70], ...
        'FontSize', 8.5, 'FontName','Consolas', 'FontWeight','bold', ...
        'HorizontalAlignment','left');
end

% =========================================================================
function updateMLHUD(hud, mlResult, t)
% UPDATEMLHUD - Refreshes the ML analysis + gradient-descent optimizer display.
    if ~isgraphics(hud.panel) || isempty(mlResult); return; end

    % Color based on action agreement
    if mlResult.actionMatch
        matchStr = 'AGREE [100%]';
        clr = [0.55 1.00 0.55];
    else
        matchStr = sprintf('DIFF -> %s', mlResult.predictedAction);
        clr = [1.00 0.70 0.20];
    end
    set(hud.panel, 'ForegroundColor', clr);

    Kp = mlResult.optimizedKp;
    Kd = mlResult.optimizedKd;

    lineH = sprintf('┌── ML INTELLIGENCE & GD OPTIMIZER (t=%.1fs) ──┐', t);
    line1 = sprintf('│ ML Model  : k-NN Classifier (k=5, 10-D State) │');
    line2 = sprintf('│ Pred Act  : %-16s [Conf: %3.0f%%]   │', mlResult.predictedAction, mlResult.confidence*100);
    line3 = sprintf('│ FSM Act   : %-16s [%-13s] │', mlResult.actualAction, matchStr);
    line4 = sprintf('│ Arm Error : RMS %.4f deg | Loss: %.5f     │', mlResult.trackingError, mlResult.optimizerLoss);
    line5 = sprintf('│ Opt Kp    : [%.1f, %.1f, %.1f, %.1f] Nm/rad        │', Kp(1),Kp(2),Kp(3),Kp(4));
    line6 = sprintf('│ Opt Kd    : [%.1f, %.1f, %.1f, %.1f] Nms/rad       │', Kd(1),Kd(2),Kd(3),Kd(4));
    line7 = sprintf('└───────────────────────────────────────────────┘');

    set(hud.panel,'String', ...
        sprintf('%s\n%s\n%s\n%s\n%s\n%s\n%s', lineH,line1,line2,line3,line4,line5,line7));
end


