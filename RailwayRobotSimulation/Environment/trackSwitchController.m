function [robot, switchState] = trackSwitchController(robot, switchState, config, dt, ax)
% TRACKSWITCHCONTROLLER - Physically realistic railway turnout & track switch.
%
% Generates a full 3D railway turnout linking Track 1 (patrol) and Track 2 (home):
%   - Ballast gravel bed under the crossover section
%   - Angled wooden sleepers (ties) aligned perpendicular to the turnout curve
%   - Dual curved steel rails (standard gauge = 1.435m) at rail height z = 0.13m
%   - Turnout frog point and guard rails
%   - Animated movable switch point blades with motor housing and indicator lantern
%   - Smooth kinematic tracking along the Bezier curve (x, y, z=0.13, yaw)
%
% Robot wheels ride directly ON the switch rails at all times (not on bare ground).

    if isempty(ax) && isfield(robot, 'ax') && isgraphics(robot.ax)
        ax = robot.ax;
    end

    if ~isfield(switchState, 'phase') || isempty(switchState.phase)
        switchState.phase        = 'INIT';
        switchState.t_param      = 0;
        switchState.done         = false;
        switchState.switchX      = robot.x + 0.8;
        switchState.switchLen    = 14.0;
        switchState.bladeHandles = [];
        switchState.trackHandles = [];
    end

    if switchState.done; return; end

    switchX     = switchState.switchX;
    switchLen   = switchState.switchLen;
    switchSpeed = 1.0;   % safe transition speed through turnout [m/s]

    y1 = config.track1Y;
    y2 = config.track2Y;

    % Cubic Bezier control points for smooth tangent continuity at both ends
    P0 = [switchX,                  y1];
    P1 = [switchX + switchLen*0.35, y1];  % tangent parallel to Track 1
    P2 = [switchX + switchLen*0.65, y2];  % tangent parallel to Track 2
    P3 = [switchX + switchLen,      y2];

    switch switchState.phase

        case 'INIT'
            % Build the entire physical turnout track: ballast, sleepers, steel rails, frog
            if ~isempty(ax) && isgraphics(ax)
                switchState.trackHandles = drawPhysicalTurnoutTrack(ax, P0, P1, P2, P3, config);
                switchState.bladeHandles = drawSwitchBlades(ax, switchX, y1, y2, config);
            end
            switchState.phase = 'APPROACH';
            robot.targetSpeed = switchSpeed;

        case 'APPROACH'
            % Advance robot along Track 1 until reaching turnout start point P0
            if robot.speed > switchSpeed
                robot.speed = max(switchSpeed, robot.speed - config.robotAcceleration * dt);
            else
                robot.speed = min(switchSpeed, robot.speed + config.robotAcceleration * dt);
            end
            robot.x = robot.x + robot.speed * dt;
            robot.y = y1;
            robot.z = 0.13;
            robot.yaw = 0;
            updateRobotTransform(robot);

            if robot.x >= switchX
                robot.x = switchX;
                switchState.phase = 'TRAVERSE';
                switchState.t_param = 0;
            end

        case 'TRAVERSE'
            % Kinematic trajectory along cubic Bezier curve
            t = switchState.t_param;

            % Cubic Bezier derivative (tangent) for heading yaw and arc length speed
            dBdt = 3*(1-t)^2*(P1-P0) + 6*(1-t)*t*(P2-P1) + 3*t^2*(P3-P2);
            tangentLen = norm(dBdt);
            if tangentLen < 1e-4; tangentLen = switchLen; end

            % Advance parameter t based on physical speed
            dt_bezier = (robot.speed / tangentLen) * dt;
            switchState.t_param = min(switchState.t_param + dt_bezier, 1.0);
            t = switchState.t_param;

            % Re-evaluate at updated t
            Bt = (1-t)^3*P0 + 3*(1-t)^2*t*P1 + 3*(1-t)*t^2*P2 + t^3*P3;
            dBdt = 3*(1-t)^2*(P1-P0) + 6*(1-t)*t*(P2-P1) + 3*t^2*(P3-P2);
            yawAngle = atan2(dBdt(2), dBdt(1));

            % Update robot state: sits ON RAILS (z = 0.13m)
            robot.x   = Bt(1);
            robot.y   = Bt(2);
            robot.z   = 0.13;   % rail head height - wheels remain on steel rails
            robot.yaw = yawAngle;

            % Animate switch blades and lantern
            bladeProgress = min(t * 3.0, 1.0);
            animateSwitchBlades(switchState.bladeHandles, bladeProgress, y1, y2, config);

            updateRobotTransform(robot);

            if t >= 1.0
                switchState.phase = 'STRAIGHTEN';
            end

        case 'STRAIGHTEN'
            % Smoothly realign yaw to 0 on Track 2 (home track)
            robot.yaw = robot.yaw * 0.80;
            robot.z   = 0.13;   % maintain rail surface height
            if abs(robot.yaw) < 0.008
                robot.yaw = 0;
                robot.y   = y2;
                robot.z   = 0.13;
                robot.targetSpeed = config.robotSpeed;
                switchState.phase = 'COMPLETE';
                switchState.done  = true;

                % Ensure switch blades and lantern show locked position
                animateSwitchBlades(switchState.bladeHandles, 1.0, y1, y2, config);

                if ~isempty(ax) && isgraphics(ax)
                    text(ax, robot.x + 1.0, y2, 3.2, '  ► ARRIVED ON HOME TRACK ◄  ', ...
                        'Color',[0.05 1.0 0.45], 'FontSize',11, 'FontWeight','bold', ...
                        'BackgroundColor',[0.02 0.10 0.04], 'Margin',6, ...
                        'HorizontalAlignment','center', 'EdgeColor',[0.1 0.85 0.3], ...
                        'Tag','HomeTrackArrival');
                end
            end
            updateRobotTransform(robot);
    end
end

% =========================================================================
function h = drawPhysicalTurnoutTrack(ax, P0, P1, P2, P3, config)
% Renders the full 3D railway turnout: gravel ballast, wooden sleepers,
% dual curved steel rails, guard rails, and frog point.

    h = struct();
    halfGauge = config.railGauge / 2;
    railH     = 0.13;   % steel rail head height [m]
    sleeperH  = 0.06;   % sleeper top height [m]
    railW     = 0.055;  % rail head width [m]

    % 1. ---- BALLAST GRAVEL BED UNDER TURNOUT ----
    nPtsB = 25;
    tB = linspace(0, 1, nPtsB);
    cLeft  = zeros(nPtsB, 2);
    cRight = zeros(nPtsB, 2);
    for i = 1:nPtsB
        t = tB(i);
        B = (1-t)^3*P0 + 3*(1-t)^2*t*P1 + 3*(1-t)*t^2*P2 + t^3*P3;
        dB = 3*(1-t)^2*(P1-P0) + 6*(1-t)*t*(P2-P1) + 3*t^2*(P3-P2);
        T_u = dB / norm(dB);
        N_u = [-T_u(2), T_u(1)];
        cLeft(i,:)  = B + (halfGauge + 0.85) * N_u;
        cRight(i,:) = B - (halfGauge + 0.85) * N_u;
    end
    bX = [cLeft(:,1); flipud(cRight(:,1))];
    bY = [cLeft(:,2); flipud(cRight(:,2))];
    bZ = ones(size(bX)) * 0.035;
    h.ballast = patch(ax, bX, bY, bZ, [0.50 0.50 0.52], 'EdgeColor','none', 'FaceAlpha', 0.85);

    % 2. ---- TURNOUT SLEEPERS (WOODEN TIES) ----
    nSleepers = 14;
    tSleepers = linspace(0.04, 0.96, nSleepers);
    h.sleepers = gobjects(nSleepers, 1);
    sw = 0.22;   % sleeper width along track
    sLen = config.railGauge + 0.90;  % sleeper total length

    for k = 1:nSleepers
        t = tSleepers(k);
        B = (1-t)^3*P0 + 3*(1-t)^2*t*P1 + 3*(1-t)*t^2*P2 + t^3*P3;
        dB = 3*(1-t)^2*(P1-P0) + 6*(1-t)*t*(P2-P1) + 3*t^2*(P3-P2);
        T_u = dB / norm(dB);
        N_u = [-T_u(2), T_u(1)];

        % Sleeper 4 corners in horizontal plane
        p1 = B - (sw/2)*T_u - (sLen/2)*N_u;
        p2 = B - (sw/2)*T_u + (sLen/2)*N_u;
        p3 = B + (sw/2)*T_u + (sLen/2)*N_u;
        p4 = B + (sw/2)*T_u - (sLen/2)*N_u;

        vx = [p1(1) p2(1) p3(1) p4(1)];
        vy = [p1(2) p2(2) p3(2) p4(2)];
        vz = [sleeperH sleeperH sleeperH sleeperH];
        h.sleepers(k) = patch(ax, vx, vy, vz, [0.38 0.25 0.14], 'EdgeColor','none');
    end

    % 3. ---- DUAL CURVED STEEL RAILS (LEFT & RIGHT) ----
    nPtsR = 50;
    tR = linspace(0, 1, nPtsR);
    leftRailInner  = zeros(nPtsR, 2);
    leftRailOuter  = zeros(nPtsR, 2);
    rightRailInner = zeros(nPtsR, 2);
    rightRailOuter = zeros(nPtsR, 2);

    for i = 1:nPtsR
        t = tR(i);
        B = (1-t)^3*P0 + 3*(1-t)^2*t*P1 + 3*(1-t)*t^2*P2 + t^3*P3;
        dB = 3*(1-t)^2*(P1-P0) + 6*(1-t)*t*(P2-P1) + 3*t^2*(P3-P2);
        T_u = dB / norm(dB);
        N_u = [-T_u(2), T_u(1)];

        % Left rail centreline: B + halfGauge*N
        rLeftCtr  = B + halfGauge * N_u;
        leftRailInner(i,:) = rLeftCtr - (railW/2) * N_u;
        leftRailOuter(i,:) = rLeftCtr + (railW/2) * N_u;

        % Right rail centreline: B - halfGauge*N
        rRightCtr = B - halfGauge * N_u;
        rightRailInner(i,:) = rRightCtr - (railW/2) * N_u;
        rightRailOuter(i,:) = rRightCtr + (railW/2) * N_u;
    end

    % Draw Left Rail Top surface
    rxL = [leftRailInner(:,1); flipud(leftRailOuter(:,1))];
    ryL = [leftRailInner(:,2); flipud(leftRailOuter(:,2))];
    rzL = ones(size(rxL)) * railH;
    h.railL = patch(ax, rxL, ryL, rzL, [0.62 0.62 0.68], ...
        'EdgeColor',[0.20 0.20 0.22], 'LineWidth', 1.2);

    % Draw Right Rail Top surface
    rxR = [rightRailInner(:,1); flipud(rightRailOuter(:,1))];
    ryR = [rightRailInner(:,2); flipud(rightRailOuter(:,2))];
    rzR = ones(size(rxR)) * railH;
    h.railR = patch(ax, rxR, ryR, rzR, [0.62 0.62 0.68], ...
        'EdgeColor',[0.20 0.20 0.22], 'LineWidth', 1.2);

    % 4. ---- FROG & GUARD RAILS ----
    % Guard rails at divergence and convergence points
    gX1 = [P0(1)+1.2, P0(1)+3.5];
    gY1 = [P0(2)-halfGauge-0.08, P0(2)-halfGauge-0.08];
    h.guard1 = plot3(ax, gX1, gY1, [railH railH], 'Color', [0.85 0.70 0.05], 'LineWidth', 3.0);

    gX2 = [P3(1)-3.5, P3(1)-1.2];
    gY2 = [P3(2)+halfGauge+0.08, P3(2)+halfGauge+0.08];
    h.guard2 = plot3(ax, gX2, gY2, [railH railH], 'Color', [0.85 0.70 0.05], 'LineWidth', 3.0);

    % Label for the crossover turnout
    h.label = text(ax, (P0(1)+P3(1))/2, (P0(2)+P3(2))/2, 0.45, '◄ TURNOUT TRACK ►', ...
        'Color',[0.90 0.85 0.20], 'FontSize',8, 'FontWeight','bold', ...
        'BackgroundColor',[0.08 0.08 0.12], 'HorizontalAlignment','center');
end

% =========================================================================
function h = drawSwitchBlades(ax, switchX, y1, y2, config)
% Movable switch point blades and electric turnout motor Stand

    h = gobjects(5, 1);
    hg = config.railGauge / 2;
    bladeLen = 4.2;

    % Left rail switch point blade
    bx = [switchX switchX+bladeLen];
    by = [y1-hg   y1-hg];
    bz = [0.14    0.14];
    h(1) = patch(ax, bx, by, bz, [1.0 0.78 0.02], 'EdgeColor',[0.75 0.55 0], 'LineWidth', 2.2);

    % Right rail switch point blade
    h(2) = patch(ax, bx, [y1+hg y1+hg], bz, [1.0 0.78 0.02], 'EdgeColor',[0.75 0.55 0], 'LineWidth', 2.2);

    % Turnout motor housing box
    motorX = switchX + 0.8;
    motorY = (y1 + y2) / 2;
    verts = [motorX-0.18 motorY-0.14 0.08;
             motorX+0.18 motorY-0.14 0.08;
             motorX+0.18 motorY+0.14 0.08;
             motorX-0.18 motorY+0.14 0.08;
             motorX-0.18 motorY-0.14 0.42;
             motorX+0.18 motorY-0.14 0.42;
             motorX+0.18 motorY+0.14 0.42;
             motorX-0.18 motorY+0.14 0.42];
    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 3 4 8 7; 1 4 8 5; 2 3 7 6];
    h(3) = patch(ax, 'Vertices', verts, 'Faces', faces, ...
        'FaceColor', [0.22 0.24 0.28], 'EdgeColor', [0.38 0.40 0.45]);

    % Operating throw rod connecting motor to switch blades
    h(4) = plot3(ax, [motorX switchX+0.5], [motorY y1-hg], [0.10 0.10], ...
        'Color', [0.70 0.72 0.76], 'LineWidth', 2.0);

    % Illuminated railway signal lantern on motor housing
    [lx,ly,lz] = sphere(8);
    h(5) = surface(ax, lx*0.05+motorX, ly*0.05+motorY, lz*0.05+0.45, ...
        'FaceColor',[1.0 0.80 0.0], 'EdgeColor','none', 'FaceLighting','gouraud');
end

% =========================================================================
function animateSwitchBlades(h, progress, y1, y2, config)
% Animates the point blades pivoting to guide robot onto the turnout

    if isempty(h) || ~isgraphics(h(1)); return; end
    hg = config.railGauge / 2;

    % Blade tips deflect laterally toward Track 2 alignment
    tipY_L = y1 - hg + (y2 - y1 - hg) * progress * 0.45;
    tipY_R = y1 + hg + (y2 - y1 - hg) * progress * 0.45;

    if isgraphics(h(1)); set(h(1), 'YData', [y1-hg, tipY_L]); end
    if isgraphics(h(2)); set(h(2), 'YData', [y1+hg, tipY_R]); end

    % Switch lantern changes from amber -> bright green when switch is aligned
    if numel(h) >= 5 && isgraphics(h(5))
        c = [1.0 - progress*0.9, 0.45 + progress*0.55, 0.1];
        set(h(5), 'FaceColor', c);
    end
end
