function [robot, arm, obstacles, log, action] = RobotController(robot, arm, obstacles, obstacleInfo, trainStatus, config, dt, log, dash, t)
% ROBOTCONTROLLER - One frame of sense->decide->act.

    ctx = struct();
    ctx.trainEmergency   = (trainStatus.level == "EMERGENCY");
    ctx.trainClearNow    = (trainStatus.level ~= "EMERGENCY");
    ctx.obstacleDetected = obstacleInfo.found;
    ctx.obstacleClass    = "";
    ctx.closeEnoughToAct = false;
    ctx.armSequenceDone  = false;
    ctx.trackClear       = false;
    ctx.allClear         = false;
    ctx.atHomeTrack      = false;
    ctx.lowRiskSkip      = false;  % true when detected obstacle is LOW risk -> robot ignores it
    action = "CONTINUE";

    if ~isfield(robot,'pouchOffset');    robot.pouchOffset    = [-1.20, 0, 0.475]; end
    if ~isfield(robot,'holdingObstacle');robot.holdingObstacle = false; end
    if ~isfield(robot,'armDwellTimer'); robot.armDwellTimer   = 0; end
    if ~isfield(robot,'armSubState');   robot.armSubState     = ""; end
    if ~isfield(robot,'track1Y');       robot.track1Y         = config.track1Y; end
    if ~isfield(robot,'angleHistory');  robot.angleHistory    = zeros(0,4); end
    if ~isfield(robot,'angleHistoryT'); robot.angleHistoryT   = zeros(0,1); end
    if ~isfield(robot,'pouchCount');    robot.pouchCount      = 0; end   % # items in pouch
    if ~isfield(robot,'pouchMaxItems'); robot.pouchMaxItems   = 4; end   % 535L / ~130L per rock
    if ~isfield(robot,'pouchFull');     robot.pouchFull       = false; end

    switch robot.state

        case "MOVING"
            robot.targetSpeed = config.robotSpeed;
            % Count non-removed, non-skipped obstacles on patrol track
            numRemaining = 0;
            for k = 1:numel(obstacles)
                if ~obstacles(k).removed; numRemaining = numRemaining + 1; end
            end
            ctx.allClear = (numRemaining == 0) && (numel(obstacles) > 0);

            if obstacleInfo.found && ~ctx.allClear
                obs_ahead = obstacleInfo.obstacle;
                % ---- RISK CHECK: evaluate how dangerous this obstacle is ----
                obsRisk = "MEDIUM";  % default
                if isfield(obs_ahead, 'riskLevel')
                    obsRisk = obs_ahead.riskLevel;
                end
                isMud = (isfield(obs_ahead, 'obstacleType') && strcmp(obs_ahead.obstacleType, 'mud'));

                if obsRisk == "LOW" && ~isMud
                    % LOW risk pebble/small rock: robot detects, logs, but keeps patrolling
                    % Mark as skipped so it doesn't count as remaining work
                    idx = obstacleInfo.index;
                    if isfield(obstacles(idx),'skipped') && isequal(obstacles(idx).skipped, true)
                        % Already logged this skip - just continue moving
                    else
                        obstacles(idx).skipped = true;
                        obstacles(idx).removed  = true;   % treat as done (no action)
                        log = eventLogger(log,dash,t,robot,obstacleInfo,trainStatus, ...
                            sprintf('LOW_RISK_SKIPPED: %s score=%.0f - train can pass safely', ...
                                obs_ahead.obstacleType, obs_ahead.riskScore));
                    end
                    ctx.lowRiskSkip = true;
                    % Robot keeps full speed - no slowdown
                else
                    % Mud (scraped via front leveler) or MEDIUM/HIGH risk: engage removal
                    robot.activeObstacleIdx = obstacleInfo.index;
                    action = "SLOW_DOWN";
                end
            end
            updatePlowTransform(robot, 0, 0, 0);

        case "OBSTACLE_DETECTED"
            log = eventLogger(log,dash,t,robot,obstacleInfo,trainStatus,"OBSTACLE_DETECTED");
            if robot.activeObstacleIdx >= 1 && robot.activeObstacleIdx <= numel(obstacles)
                ctx.obstacleClass = getObsCategory(obstacles(robot.activeObstacleIdx), config);
            end

        case "CLASSIFYING"
            if robot.activeObstacleIdx >= 1 && robot.activeObstacleIdx <= numel(obstacles)
                obs = obstacles(robot.activeObstacleIdx);
                ctx.obstacleClass = getObsCategory(obs, config);
                robot.targetSpeed = config.robotSpeed * 0.4;
                % Log risk level alongside category
                riskStr = "UNKNOWN";
                if isfield(obs,'riskLevel'); riskStr = obs.riskLevel; end
                riskSc  = 0;
                if isfield(obs,'riskScore');  riskSc  = obs.riskScore; end
                isMud = (isfield(obs, 'obstacleType') && strcmp(obs.obstacleType, 'mud'));
                if isMud
                    log = eventLogger(log,dash,t,robot,obstacleInfo,trainStatus, ...
                        sprintf('MUD_DEPOSIT_WILL_SCRAPE [Front Leveler Blade | Risk:%s score=%.0f]', riskStr, riskSc));
                elseif ctx.obstacleClass == "HEAVY"
                    log = eventLogger(log,dash,t,robot,obstacleInfo,trainStatus, ...
                        sprintf('HEAVY_OBS_WILL_PUSH [Front Leveler Plow | Risk:%s score=%.0f]', riskStr, riskSc));
                else
                    log = eventLogger(log,dash,t,robot,obstacleInfo,trainStatus, ...
                        sprintf('REMOVABLE_OBS_WILL_PICK [Manipulator Arm + PD | Risk:%s score=%.0f]', riskStr, riskSc));
                end
            end

        case "APPROACHING"
            if robot.activeObstacleIdx >= 1 && robot.activeObstacleIdx <= numel(obstacles)
                obs_app = obstacles(robot.activeObstacleIdx);
                d = calculateObstacleDistance(robot, obs_app);
                obsCat = getObsCategory(obs_app, config);
                isMud = (isfield(obs_app, 'obstacleType') && strcmp(obs_app.obstacleType, 'mud'));
                if (robot.pouchFull || isMud) && obsCat == "REMOVABLE"
                    obsCat = "HEAVY"; % mud or pouch full routes to front leveler/plow
                end
            else
                d = 0;
                obsCat = "";
            end
            % HEAVY / MUD (or POUCH FULL): stop at 2.5m for leveler/plow engagement
            % REMOVABLE: stop at 1.25m for arm reach
            if obsCat == "HEAVY"
                ctx.closeEnoughToAct = (d <= 2.5);
            else
                ctx.closeEnoughToAct = (d <= 1.25);
            end
            if d > 12;      robot.targetSpeed = config.robotSpeed * 1.00;
            elseif d > 6;   robot.targetSpeed = config.robotSpeed * 0.50;
            elseif d > 3;   robot.targetSpeed = config.robotSpeed * 0.25;
            elseif d > 1.8; robot.targetSpeed = config.robotSpeed * 0.12;
            else;           robot.targetSpeed = config.robotSpeed * 0.06; end
            action = "SLOW_DOWN";

        case "STOPPED"
            robot.speed = 0; robot.targetSpeed = 0;
            if robot.armSubState == "" || robot.armSubState == "IDLE"
                if robot.activeObstacleIdx >= 1 && robot.activeObstacleIdx <= numel(obstacles)
                    obs_stop = obstacles(robot.activeObstacleIdx);
                    obsCat = getObsCategory(obs_stop, config);
                    isCatHeavy = (obsCat == "HEAVY");
                    isMud = (isfield(obs_stop, 'obstacleType') && strcmp(obs_stop.obstacleType, 'mud'));
                    isPouchFull = robot.pouchFull;
                    if isCatHeavy || isMud || isPouchFull
                        % HEAVY obstacle, MUD deposit, OR pouch is full -> use front leveler/plow
                        robot.armSubState = "PLOW_ENGAGE";
                        if isMud
                            log = eventLogger(log,dash,t,robot,obstacleInfo,trainStatus,"FRONT_LEVELER_SCRAPE_MUD");
                        elseif isPouchFull && ~isCatHeavy
                            log = eventLogger(log,dash,t,robot,obstacleInfo,trainStatus,"POUCH_FULL_PUSH_FALLBACK");
                        end
                    else
                        robot.armSubState = "REACH";  % normal arm pick
                    end
                end
                robot.armDwellTimer = 0;
            end
            if robot.activeObstacleIdx >= 1 && robot.activeObstacleIdx <= numel(obstacles)
                obs_stop = obstacles(robot.activeObstacleIdx);
                ctx.obstacleClass = getObsCategory(obs_stop, config);
                isMud = (isfield(obs_stop, 'obstacleType') && strcmp(obs_stop.obstacleType, 'mud'));
                % If mud or pouch full, reclassify as HEAVY so state machine routes to PUSHING (leveler/plow)
                if (isMud || robot.pouchFull) && ctx.obstacleClass == "REMOVABLE"
                    ctx.obstacleClass = "HEAVY";
                end
            end

        case "REMOVING"
            robot.speed = 0; robot.targetSpeed = 0;
            if ctx.trainEmergency && ~any(robot.armSubState == ["RETURN","IDLE",""])
                robot.armSubState = "RETURN";
            end
            [robot,arm,obstacles,log,seqDone] = runPickSequence(robot,arm,obstacles,config,dt,log,dash,t,obstacleInfo,trainStatus);
            ctx.armSequenceDone = seqDone;

        case "PUSHING"
            if ctx.trainEmergency && ~any(robot.armSubState == ["PLOW_REVERSE","PLOW_DONE","IDLE",""])
                robot.armSubState = "PLOW_REVERSE";   % back off from boulder
                robot.targetSpeed = 0;
                robot.speed = 0;
            end
            [robot,arm,obstacles,log,seqDone] = runPushSequence(robot,arm,obstacles,config,dt,log,dash,t,obstacleInfo,trainStatus);
            ctx.armSequenceDone = seqDone;

        case "CLEARANCE_CHECK"
            robot.speed = 0; robot.targetSpeed = 0;
            % IMPORTANT: check clearance against PATROL TRACK centre (track1Y),
            % NOT robot.y (which may differ during returns). This prevents
            % false "not clear" readings after the robot moves laterally.
            robotProxy = robot;
            robotProxy.y = config.track1Y;
            ctx.trackClear = trackClearanceCheck(robotProxy, obstacles, config);
            if ctx.trackClear
                robot.armSubState = "";
                robot.activeObstacleIdx = 0;
                robot.armDwellTimer = 0;
            end

        case "RESUMING"
            robot.targetSpeed = config.robotSpeed;
            robot.armSubState = "";
            robot.activeObstacleIdx = 0;
            robot.armDwellTimer = 0;
            log = eventLogger(log,dash,t,robot,obstacleInfo,trainStatus,"ROBOT_RESUMED");

        case "RETURNING_HOME"
            % Track switch: Bezier-curved realistic track change animation
            % Uses trackSwitchController for physically realistic turnout motion
            if ~isfield(robot, 'switchState') || isempty(robot.switchState)
                robot.switchState = struct('phase','INIT','t_param',0,'done',false, ...
                    'switchX',robot.x+3.0,'switchLen',12.0, ...
                    'bladeHandles',[],'curveHandle',[]);
            end
            [robot, robot.switchState] = trackSwitchController(robot, robot.switchState, config, dt, []);
            if robot.switchState.done
                ctx.atHomeTrack = true;
                log = eventLogger(log,dash,t,robot,obstacleInfo,trainStatus,"ROBOT_AT_HOME_TRACK");
            end
            updateRobotTransform(robot);

        case "WARNING"
            robot.targetSpeed = 0;
        case "WAITING"
            robot.targetSpeed = 0;
            ctx.trackClear = trackClearanceCheck(robot, obstacles, config);
        case "EMERGENCY_STOP"
            robot.speed = 0; robot.targetSpeed = 0;
        case "COMPLETED"
            robot.targetSpeed = 0;
    end

    % State transition
    nextState = RobotStateMachine(robot.state, ctx);
    if nextState ~= robot.state
        log = eventLogger(log,dash,t,robot,obstacleInfo,trainStatus,...
            sprintf('%s -> %s', robot.state, nextState));
        robot.state = nextState;
        robot.armDwellTimer = 0;
    end

    % Record arm angles for graph (every frame)
    robot.angleHistory  = [robot.angleHistory;  arm.angles];
    robot.angleHistoryT = [robot.angleHistoryT; t];
    maxSamples = 2000;
    if size(robot.angleHistory,1) > maxSamples
        robot.angleHistory  = robot.angleHistory(end-maxSamples+1:end,:);
        robot.angleHistoryT = robot.angleHistoryT(end-maxSamples+1:end);
    end

    % Record detailed arm telemetry for 2-Link Manipulator Analysis
    if ~isfield(robot, 'armLog') || isempty(robot.armLog)
        robot.armLog = struct('t', zeros(0,1), 'q', zeros(0,4), 'qd', zeros(0,4), ...
                              'qdot', zeros(0,4), 'tau', zeros(0,4), 'holding', false(0,1));
    end
    qd_target = arm.homeAngles;
    if isfield(robot, 'lastArmTarget') && ~isempty(robot.lastArmTarget)
        qd_target = robot.lastArmTarget;
    end
    try
        [tau_now, ~] = DynamicRobotController(arm, qd_target, zeros(1,4), config, dt);
    catch
        tau_now = zeros(4,1);
    end

    robot.armLog.t(end+1, 1)       = t;
    robot.armLog.q(end+1, :)       = arm.angles;
    robot.armLog.qd(end+1, :)      = qd_target;
    robot.armLog.qdot(end+1, :)    = arm.angularVelocity;
    robot.armLog.tau(end+1, :)     = tau_now(:)';
    robot.armLog.holding(end+1, 1) = robot.holdingObstacle;

    % Keep IN_POUCH obstacles locked to robot rear pouch
    for k = 1:numel(obstacles)
        if isfield(obstacles(k),'state') && obstacles(k).state == "IN_POUCH"
            ppos = [robot.x + robot.pouchOffset(1), ...
                    robot.y + robot.pouchOffset(2), ...
                    robot.z + robot.pouchOffset(3)];
            obstacles(k) = setObstaclePosition(obstacles(k), ppos);
        end
    end

    updateWarningLight(robot);
end

% =========================================================================
function [robot,arm,obstacles,log,seqDone] = runPickSequence(robot,arm,obstacles,config,dt,log,dash,t,obstacleInfo,trainStatus)
% RUNPICKSEQUENCE  -  REMOVABLE obstacle removal via 7-phase arm sequence.
%
% Phase sequence:
%   REACH       -> Hover arm ABOVE obstacle (IK: 'above' phase, tip ~0.4m above obs top)
%   DESCEND     -> Lower arm to grasp height (IK: 'grasp' phase, gripper open)
%   GRASP       -> Close gripper. Obstacle locks to tip once tip confirmed near obs.
%   LIFT        -> Raise obstacle off track into high safe transport height
%   SWING       -> Swing arm 180deg to rear in high fold configuration (clears all chassis/walls)
%   POUCH_HOVER -> Lower arm vertically above center of pouch opening (19cm clearance over rim)
%   DROP        -> Open gripper, obstacle drops cleanly into pouch container
%   POUCH_RAISE -> Lift arm vertically back to high fold configuration
%   RETURN      -> Arm returns to front home fold position

    seqDone = false;
    idx = robot.activeObstacleIdx;
    if idx < 1 || idx > numel(obstacles)
        robot.armSubState = ""; seqDone = true; return;
    end
    obs = obstacles(idx);
    holdColor = [1.0, 0.55, 0.05];

    switch robot.armSubState

        case "REACH"
            % ArmDirectionController: decides approach direction/phase based on
            % obstacle TYPE, then applies PD correction for residual tip error.
            % For 'big_rock' it returns 'above' phase angles (high hover);
            % all other removable types return type-aware 'grasp' or 'push' angles.
            [target, armCmd] = ArmDirectionController(arm, robot, obs, config, dt);
            robot.lastTipErrorNorm = armCmd.tipErrorNorm;
            % Override to 'above' hover for approach (controller may give grasp;
            % we still want hover first, then descend)
            hoverTarget = moveArmToObstacle(arm, robot, obs, 'above');
            % Blend: use controller angles when type is big_rock (needs more clearance),
            % otherwise use standard hover to avoid clipping
            if strcmp(armCmd.obstacleType,'big_rock')
                target = target;   % ArmDirectionController already gave above
            else
                target = hoverTarget;
            end
            [arm,reached] = controlArm(arm, target, 1, config, dt);
            robot.lastArmTarget = target;
            if advance(reached)
                robot.armSubState = "DESCEND";
                log = eventLogger(log,dash,t,robot,obstacleInfo,trainStatus, ...
                    sprintf('ARM_HOVERING_ABOVE [%s, err=%.3fm]', armCmd.obstacleType, armCmd.tipErrorNorm));
            end

        case "DESCEND"
            % ArmDirectionController: computes type-aware descend direction + PD correction.
            % For logs/mud: uses 'push' phase (lateral approach).
            % For rocks/stones: uses 'grasp' phase (top-down descent).
            [target, armCmd] = ArmDirectionController(arm, robot, obs, config, dt);
            robot.lastTipErrorNorm = armCmd.tipErrorNorm;
            robot.lastArmTarget = target;
            [arm,reached] = controlArm(arm, target, 1, config, dt);
            if advance(reached)
                robot.armSubState = "GRASP";
                log = eventLogger(log,dash,t,robot,obstacleInfo,trainStatus, ...
                    sprintf('ARM_AT_SURFACE [%s, tipErr=%.3fm, pdCorr=[%.1f,%.1f,%.1f,%.1f]deg]', ...
                        armCmd.obstacleType, armCmd.tipErrorNorm, armCmd.pdCorrection));
            end

        case "GRASP"
            % Close gripper. With the corrected FK (getGripperWorldPosition),
            % tip IS physically at obstacle when DESCEND completes.
            % tipToObs check guarantees no teleport before locking.
            target = moveArmToObstacle(arm, robot, obs, 'grasp');
            [arm,reached] = controlArm(arm, target, 0, config, dt);
            tip = getGripperWorldPosition(robot, arm);

            if robot.holdingObstacle
                obstacles(idx) = setObstaclePosition(obstacles(idx), tip);
            end
            if reached && ~robot.holdingObstacle
                % Verify tip is physically near obstacle before locking (no teleport)
                tipToObs = norm(tip - obs.position);
                if tipToObs < 0.50   % 50cm tolerance (IK has ~3cm residual error)
                    robot.holdingObstacle = true;
                    obstacles(idx).state  = "HELD";
                    obstacles(idx) = setObstaclePosition(obstacles(idx), tip);
                    if ~isfield(robot, 'pickEvent') || isempty(robot.pickEvent)
                        robot.pickEvent = struct();
                    end
                    robot.pickEvent.hasPicked     = true;
                    robot.pickEvent.t_pick        = t;
                    robot.pickEvent.obstacleMass  = obs.mass;
                    robot.pickEvent.pickPos       = [obs.position(1) - robot.x, obs.position(3) - (robot.z + 0.95)];
                    robot.pickEvent.dropPos       = [-0.35, 0.40];
                    if isgraphics(obstacles(idx).handle)
                        set(obstacles(idx).handle,'FaceColor',holdColor);
                    end
                    log = eventLogger(log,dash,t,robot,obstacleInfo,trainStatus,"GRIPPER_CLOSED_HOLDING");
                end
            end
            if advance(reached); robot.armSubState = "LIFT"; end

        case "LIFT"
            % Lift obstacle UP off track into compact safe transport posture
            target = [0, 50, 40, -80];
            [arm,reached] = controlArm(arm, target, 0, config, dt);
            tip = getGripperWorldPosition(robot, arm);
            obstacles(idx) = setObstaclePosition(obstacles(idx), tip);
            if advance(reached); robot.armSubState = "SWING"; end

        case "SWING"
            % High-altitude 180deg yaw swing to rear in compact carry fold:
            % Arm is folded high (z > 1.45m), safely clearing chassis, warning beacon,
            % antenna, and rear pouch side walls during the entire 180deg rotation.
            target = [180, 60, 45, -80];
            [arm,reached] = controlArm(arm, target, 0, config, dt);
            tip = getGripperWorldPosition(robot, arm);
            obstacles(idx) = setObstaclePosition(obstacles(idx), tip);
            if advance(reached); robot.armSubState = "POUCH_HOVER"; end

        case "POUCH_HOVER"
            % Lower arm smoothly straight down over the center of the pouch opening:
            % Tip: x=-1.00m (exact center of pouch cavity), z=1.12m (20cm clearance above rim at 0.92m).
            % Arm and gripper do not touch rim or walls.
            target = [180, 33, -70, 37];
            [arm,reached] = controlArm(arm, target, 0, config, dt);
            tip = getGripperWorldPosition(robot, arm);
            obstacles(idx) = setObstaclePosition(obstacles(idx), tip);
            if advance(reached); robot.armSubState = "DROP"; end

        case "DROP"
            % Open gripper: obstacle drops cleanly into hollow pouch container below.
            target = [180, 33, -70, 37];
            [arm,reached] = controlArm(arm, target, 1, config, dt);
            tip = getGripperWorldPosition(robot, arm);
            if robot.holdingObstacle
                obstacles(idx) = setObstaclePosition(obstacles(idx), tip);
            end
            if reached && robot.holdingObstacle
                robot.holdingObstacle  = false;
                obstacles(idx).state   = "IN_POUCH";
                obstacles(idx).removed = true;
                if isfield(robot, 'pickEvent')
                    robot.pickEvent.t_drop = t;
                end
                ppos = [robot.x + robot.pouchOffset(1), ...
                        robot.y + robot.pouchOffset(2), ...
                        robot.z + robot.pouchOffset(3)];
                obstacles(idx) = setObstaclePosition(obstacles(idx), ppos);
                if isgraphics(obstacles(idx).handle)
                    set(obstacles(idx).handle,'FaceColor',[0.32 0.34 0.38]);
                end
                % Track pouch fill level
                robot.pouchCount = robot.pouchCount + 1;
                robot.pouchFull  = (robot.pouchCount >= robot.pouchMaxItems);
                if robot.pouchFull
                    log = eventLogger(log,dash,t,robot,obstacleInfo,trainStatus,"POUCH_FULL");
                end
                log = eventLogger(log,dash,t,robot,obstacleInfo,trainStatus,"OBSTACLE_IN_POUCH");
                log = eventLogger(log,dash,t,robot,obstacleInfo,trainStatus,"OBSTACLE_IN_POUCH");
            end
            if advance(reached); robot.armSubState = "POUCH_RAISE"; end
        case "POUCH_RAISE"
            % Lift arm vertically back up to high fold posture before yaw rotation:
            % Clears the pouch rim completely so no wall clipping occurs.
            target = [180, 60, 45, -80];
            [arm,reached] = controlArm(arm, target, 1, config, dt);
            if advance(reached); robot.armSubState = "RETURN"; end

        case "RETURN"
            % Return arm from high fold pose back to front home fold position
            [arm,reached] = controlArm(arm, arm.homeAngles, 1, config, dt);
            if advance(reached)
                robot.armSubState = ""; seqDone = true;
                log = eventLogger(log,dash,t,robot,obstacleInfo,trainStatus,"ARM_HOME");
            end

        otherwise
            robot.armSubState = "REACH";
    end

    function ok = advance(isReached)
        if isReached; robot.armDwellTimer = robot.armDwellTimer + dt;
        else; robot.armDwellTimer = 0; end
        ok = isReached && robot.armDwellTimer >= config.armHoldTime;
        if ok; robot.armDwellTimer = 0; end
    end
end

% =========================================================================
function [robot,arm,obstacles,log,seqDone] = runPushSequence(robot,arm,obstacles,config,dt,log,dash,t,obstacleInfo,trainStatus)
% RUNPUSHSEQUENCE - HEAVY obstacle lateral push using FRONT ROAD LEVELER / PLOW BLADE.
%
% Sub-state sequence:
%  PLOW_ENGAGE  -> Robot drives slowly forward until the front plow blade
%                  physically contacts the front face of the heavy boulder.
%                  Robot speed: 0.22 m/s creep approach.
%  PLOW_PUSH    -> Robot CONTINUES slowly forward (0.28 m/s low-gear).
%                  The V-angled blade geometry deflects the boulder SIDEWAYS
%                  off the track. Boulder visibly slides outward each frame.
%                  Phase ends when boulder lateral offset > corridor+margin.
%  PLOW_REVERSE -> Robot backs up slightly (0.20 m/s) to disengage blade
%                  from cleared boulder position for a clear visual separation.
%  PLOW_DONE    -> Robot stops. Holds 1.5 s dwell so user can clearly see
%                  the cleared track before resuming.

    seqDone = false;
    idx = robot.activeObstacleIdx;
    if idx < 1 || idx > numel(obstacles)
        robot.armSubState = ""; seqDone = true; return;
    end
    obs = obstacles(idx);

    % Articulated arm stays parked at home - no actuator stress
    [arm,~] = controlArm(arm, arm.homeAngles, 1, config, dt);

    switch robot.armSubState

        case "PLOW_ENGAGE"
            % ---- Phase 1: Leveler blade lowers and extends to engage obstacle ----
            isMudObs = (isfield(obs, 'obstacleType') && strcmp(obs.obstacleType, 'mud'));
            plowTip   = robot.x + 1.05;
            obsFront  = obs.position(1) - obs.size * 0.55;
            gap       = obsFront - plowTip;

            if isMudObs
                % Front leveler scraper blade lowers smoothly down to rail level
                engageFrac = min(1.0, max(0.2, robot.armDwellTimer / 0.45));
                updatePlowTransform(robot, 0.16 * engageFrac, 0, -0.045 * engageFrac, 0);
            else
                % Boulder plow extends and angles
                updatePlowTransform(robot, 0.22, -0.08, 0, -6);
            end

            if gap > 0.03
                robot.targetSpeed = max(0.18, min(0.28, gap * 1.2));
                robot.armDwellTimer = 0;
            else
                % CONTACT - leveler blade is touching boulder/mud
                robot.targetSpeed = 0;
                robot.speed = 0;
                robot.armDwellTimer = robot.armDwellTimer + dt;
                if robot.armDwellTimer >= 0.45
                    robot.armDwellTimer = 0;
                    robot.armSubState = "PLOW_PUSH";
                    robot.mudScrapeStartX = [];   % reset so PLOW_PUSH measures from contact point
                    if isMudObs
                        log = eventLogger(log,dash,t,robot,obstacleInfo,trainStatus,"LEVELER_MUD_SCRAPE_ENGAGED");
                    else
                        log = eventLogger(log,dash,t,robot,obstacleInfo,trainStatus,"PLOW_BLADE_CONTACT");
                    end
                end
            end

        case "PLOW_PUSH"
            % ---- Phase 2: Leveler blade drives FORWARD over obstacle and clears it ----
            isMudObs = (isfield(obs, 'obstacleType') && strcmp(obs.obstacleType, 'mud'));

            if isMudObs
                % ====================================================================
                % REALISTIC RAILWAY TRACK LEVELER / BALLAST SCRAPER OPERATION:
                %   1. Scraper blade is lowered to rail head, sweeping along track.
                %   2. The V-wedge geometry deflects mud OUTWARD to both ballast shoulders
                %      (left and right of rails) rather than sliding as a solid block.
                %   3. Mud height compresses/flattens smoothly down to track level.
                %   4. Soil particles spray along the V-wings to the track shoulders.
                %   5. Once scraped past the obstacle, track corridor is completely clear.
                % ====================================================================
                scrapeSpeed = 0.22;   % controlled scraping speed
                robot.targetSpeed = scrapeSpeed;
                robot.speed       = scrapeSpeed;

                % Record robot position at start of scraping
                if ~isfield(robot, 'mudScrapeStartX') || isempty(robot.mudScrapeStartX)
                    robot.mudScrapeStartX = robot.x;
                    if isfield(obs, 'label') && ~isempty(obs.label) && all(isgraphics(obs.label))
                        set(obstacles(idx).label, 'Visible', 'off');
                    end
                end

                mudOrigWidth = 0.70;
                if isfield(obs, 'origWidth'); mudOrigWidth = obs.origWidth; end
                scrapeTotalDist = mudOrigWidth * 1.8 + 0.40;
                scrapeDist = robot.x - robot.mudScrapeStartX;
                pushedFrac = min(1.0, max(0, scrapeDist / scrapeTotalDist));

                % --- Leveler blade held low to rail head with realistic cutting vibration ---
                bladeVibe = 0.002 * sin(robot.x * 35);
                updatePlowTransform(robot, 0.18, 0, -0.045 + bladeVibe, 0);

                % --- Symmetrical V-wedge mud deformation: divides outward to shoulders ---
                heightScale = max(0.015, (1.0 - pushedFrac)^1.8);  % non-linear flattening
                spreadYScale = 1.0 + 1.8 * pushedFrac;

                if isfield(obs, 'origShapeX')
                    % Points deflect laterally based on which side of the V-wedge they are on
                    dispY = sign(obs.origShapeY + 1e-4) .* (0.42 * (pushedFrac^1.2));
                    obstacles(idx).shapeX = obs.origShapeX * (1.0 + 0.6 * pushedFrac);
                    obstacles(idx).shapeY = obs.origShapeY * spreadYScale + dispY;
                    obstacles(idx).shapeZ = obs.origShapeZ * heightScale;
                end

                baseH = 0.08;
                if isfield(obs, 'origHeight'); baseH = obs.origHeight; end
                obstacles(idx).restHeight = max(0.004, baseH * heightScale);

                % --- Center position based on fixed initial position (no compounding!) ---
                initX = obs.position(1);
                if isfield(obs, 'initPosition'); initX = obs.initPosition(1); end
                newX = initX + (scrapeTotalDist * 0.40) * pushedFrac;
                newY = config.track1Y;

                % --- Granular soil particles deflect along V-wings to ballast shoulders ---
                if isfield(obs, 'sandHandle') && ~isempty(obs.sandHandle) && ...
                        all(isgraphics(obs.sandHandle)) && isfield(obs, 'origSandRelX') && ~isempty(obs.origSandRelX)
                    sideSign = sign(obs.origSandRelY + 1e-4);
                    pX = obs.origSandRelX + 0.28 * pushedFrac;
                    pY = obs.origSandRelY * (1.0 + 1.5 * pushedFrac) + sideSign .* (0.45 * (pushedFrac^1.2));
                    pZ = max(0.005, obs.origSandRelZ * (1.0 - 0.85 * pushedFrac));
                    obstacles(idx).sandRelX = pX;
                    obstacles(idx).sandRelY = pY;
                    obstacles(idx).sandRelZ = pZ;
                    if isfield(obs, 'origSandSizes') && ~isempty(obs.origSandSizes)
                        curSizes = max(4, obs.origSandSizes .* (1.0 - 0.60 * pushedFrac));
                        set(obstacles(idx).sandHandle, 'SizeData', curSizes);
                    end
                end

                % --- Move graphics to new position and deformed shapes ---
                obstacles(idx) = setObstaclePosition(obstacles(idx), [newX, newY, obstacles(idx).restHeight]);

                % --- Color & transparency: moist soil smeared thin then blending into ballast ---
                if isgraphics(obstacles(idx).handle)
                    if pushedFrac < 0.4
                        % Wet compressed soil
                        mudC = [0.24, 0.16, 0.08];
                        mudA = 1.0;
                    else
                        % Thinning deposit blending into ballast
                        ff = (pushedFrac - 0.4) / 0.6;
                        mudC = [0.24 + 0.18*ff, 0.16 + 0.18*ff, 0.08 + 0.18*ff];
                        mudA = max(0.05, 1.0 - 0.92*ff);
                    end
                    set(obstacles(idx).handle, 'FaceColor', mudC, 'FaceAlpha', mudA);
                end

                % Shadow fades out as mass is cleared
                if isfield(obs, 'shadow') && ~isempty(obs.shadow) && all(isgraphics(obs.shadow))
                    set(obstacles(idx).shadow, 'FaceAlpha', max(0.01, 0.18 * (1.0 - pushedFrac)));
                end

                % --- Mud cleared when robot has fully passed over it ---
                if pushedFrac >= 0.995
                    % Hide mud visuals smoothly (fully scraped flat)
                    if isgraphics(obstacles(idx).handle)
                        set(obstacles(idx).handle, 'Visible', 'off');
                    end
                    if isfield(obs, 'sandHandle') && ~isempty(obs.sandHandle) && all(isgraphics(obs.sandHandle))
                        set(obstacles(idx).sandHandle, 'Visible', 'off');
                    end
                    if isfield(obs, 'shadow') && ~isempty(obs.shadow) && all(isgraphics(obs.shadow))
                        set(obstacles(idx).shadow, 'Visible', 'off');
                    end
                    if isfield(obs, 'label') && ~isempty(obs.label) && all(isgraphics(obs.label))
                        set(obstacles(idx).label, 'Visible', 'off');
                    end
                    obstacles(idx).state   = "REMOVED";
                    obstacles(idx).removed = true;
                    robot.mudScrapeStartX  = [];
                    robot.targetSpeed = 0;
                    robot.speed       = 0;
                    robot.armDwellTimer = 0;
                    robot.armSubState = "PLOW_REVERSE";
                    log = eventLogger(log,dash,t,robot,obstacleInfo,trainStatus,"MUD_SCRAPED_FLAT_CLEAR");
                end

            else
                % ====================================================================
                % SOLID ROCK / BOULDER LATERAL PUSH (rigid-body physics)
                % ====================================================================
                pushSpeed = 0.38;
                robot.targetSpeed = pushSpeed;
                robot.speed       = pushSpeed;

                obsPos  = obstacles(idx).position;
                latDist = abs(obsPos(2) - config.track1Y);

                extY   = min(0.55, latDist * 0.35);
                extRot = min(16, latDist * 10);
                updatePlowTransform(robot, 0.38, -extY, 0, -extRot);

                obsRadius  = obs.size * 0.50;
                bladeApex  = robot.x + 1.02 + 0.38;
                bladeSlope = 0.28;
                pushedFrac = min(1.0, latDist / (config.corridorWidth + 0.65));

                newX = bladeApex + obsRadius - bladeSlope * latDist;
                newY = obsPos(2) - (pushSpeed * 0.70) * dt;
                obstacles(idx) = setObstaclePosition(obstacles(idx), [newX, newY, obstacles(idx).restHeight]);

                if isgraphics(obstacles(idx).handle)
                    rc = [0.90 - 0.35*pushedFrac, 0.25 + 0.15*pushedFrac, 0.05];
                    set(obstacles(idx).handle, 'FaceColor', rc);
                end

                lateralOff = abs(obstacles(idx).position(2) - config.track1Y);
                if lateralOff >= config.corridorWidth + 0.65
                    obstacles(idx) = removeObstacle(obstacles(idx), config);
                    if isgraphics(obstacles(idx).handle)
                        set(obstacles(idx).handle, 'FaceColor', [0.30 0.30 0.30]);
                    end
                    robot.targetSpeed = 0;
                    robot.speed = 0;
                    robot.armDwellTimer = 0;
                    robot.armSubState = "PLOW_REVERSE";
                    log = eventLogger(log,dash,t,robot,obstacleInfo,trainStatus,"HEAVY_OBS_PLOWED_CLEAR");
                end
            end

        case "PLOW_REVERSE"
            % ---- Phase 3: Reverse slightly while leveler blade smoothly RETRACTS back to origin ----
            robot.targetSpeed = -0.20;
            robot.armDwellTimer = robot.armDwellTimer + dt;

            % Smooth retraction of front leveler blade back to home position [0,0,0]
            frac = max(0, 1 - robot.armDwellTimer / 1.2);
            isMudObs = (isfield(obs, 'obstacleType') && strcmp(obs.obstacleType, 'mud'));
            if isMudObs
                updatePlowTransform(robot, 0.18*frac, 0, -0.045*frac, 0);
            else
                updatePlowTransform(robot, 0.38*frac, -0.55*frac, 0, -16*frac);
            end

            if robot.armDwellTimer >= 1.2
                robot.targetSpeed = 0;
                robot.speed = 0;
                robot.armDwellTimer = 0;
                robot.armSubState = "PLOW_DONE";
            end

        case "PLOW_DONE"
            % ---- Phase 4: Leveler blade fully retracted at origin [0,0,0] ----
            robot.targetSpeed = 0;
            robot.speed = 0;
            updatePlowTransform(robot, 0, 0, 0, 0);

            robot.armDwellTimer = robot.armDwellTimer + dt;
            if robot.armDwellTimer >= 1.80
                robot.armDwellTimer = 0;
                robot.armSubState = ""; seqDone = true;
                log = eventLogger(log,dash,t,robot,obstacleInfo,trainStatus,"PLOW_OPERATION_COMPLETE");
            end

        otherwise
            robot.armSubState = "PLOW_ENGAGE";
            robot.armDwellTimer = 0;
            updatePlowTransform(robot, 0, 0, 0, 0);
    end
end

% =========================================================================
function updatePlowTransform(robot, px, py, varargin)
% UPDATEPLOWTRANSFORM - Dynamic articulation of front road leveler blade.
% Translates and rotates the leveler blade relative to robot chassis.
% Supports:
%   updatePlowTransform(robot, px, py, pRot)
%   updatePlowTransform(robot, px, py, pz, pRot)
    if nargin == 4
        pz = 0;
        pRot = varargin{1};
    elseif nargin >= 5
        pz = varargin{1};
        pRot = varargin{2};
    else
        pz = 0;
        pRot = 0;
    end
    if isfield(robot,'plowTransform') && isgraphics(robot.plowTransform)
        M = makehgtform('translate', [px, py, pz], 'zrotate', deg2rad(pRot));
        set(robot.plowTransform, 'Matrix', M);
    end
end

% =========================================================================
function updateWarningLight(robot)
    if ~isfield(robot,'warningLight') || ~isgraphics(robot.warningLight); return; end
    switch robot.state
        case "MOVING";                                           c=[0.10 0.95 0.10];
        case {"OBSTACLE_DETECTED","CLASSIFYING","APPROACHING"};  c=[0.95 0.88 0.10];
        case {"WARNING","WAITING","EMERGENCY_STOP"};             c=[0.92 0.08 0.08];
        case {"STOPPED","REMOVING","PUSHING","CLEARANCE_CHECK"}; c=[0.95 0.52 0.08];
        case {"RESUMING","RETURNING_HOME"};                      c=[0.10 0.80 0.95];
        otherwise;                                               c=[0.55 0.55 0.55];
    end
    set(robot.warningLight,'FaceColor',c);
end

% =========================================================================
function cat = getObsCategory(obs, config)
% GETOBSCATEGORY - Safely gets category string, computing from mass if missing.
    if isfield(obs, 'category') && ~isempty(obs.category)
        cat = obs.category;
    else
        cat = classifyObstacle(obs, config);
    end
end

