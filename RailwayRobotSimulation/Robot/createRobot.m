function robot = createRobot(ax, config)
% CREATEROBOT - Premium inspection robot with visible rear storage pouch.
% Everything parented to robot.mainTransform so one matrix move = whole robot.

    hold(ax,'on');
    robot.mainTransform = hgtransform('Parent', ax);
    T = robot.mainTransform;

    % ======= BODY PANELS =======
    % Main chassis - dark gunmetal
    drawBox(T, 1.50, 0.85, 0.52, [0 0 0.30], [0.18 0.20 0.24]);
    % Orange accent top stripe
    drawBox(T, 1.50, 0.85, 0.06, [0 0 0.58], [0.95 0.40 0.05]);
    % Front bumper (bright yellow)
    drawBox(T, 0.10, 0.90, 0.40, [0.76 0 0.25], [0.98 0.85 0.02]);
    
    % ======= HEAVY ARTICULATED FRONT TRACK LEVELER / COW-CATCHER PLOW =======
    % Heavy-duty angled scraper blade for pushing heavy boulders/debris off rails.
    % Articulated via robot.plowTransform for dynamic extension and retraction.
    robot.plowTransform = hgtransform('Parent', T);
    P = robot.plowTransform;
    robot.plowOffset    = [0, 0, 0];

    % Dual hydraulic mounting struts connecting front bumper to plow blade
    drawBox(P, 0.20, 0.08, 0.08, [0.84,  0.28, 0.22], [0.30 0.33 0.38]);
    drawBox(P, 0.20, 0.08, 0.08, [0.84, -0.28, 0.22], [0.30 0.33 0.38]);
    % Heavy steel transverse push beam
    drawBox(P, 0.08, 0.88, 0.10, [0.88, 0, 0.22], [0.18 0.20 0.24]);
    
    % Angled V-wedge plow deflector blades (directs mass sideways off rails)
    % Center apex at x=1.02, wings angle back to x=0.86 at y=±0.54
    vLeftX = [1.02 0.86 0.86 1.02];
    vLeftY = [0.00 0.54 0.54 0.00];
    vLeftZ = [0.08 0.08 0.38 0.38];
    patch('Parent',P,'XData',vLeftX,'YData',vLeftY,'ZData',vLeftZ,...
        'FaceColor',[0.96 0.82 0.05],'EdgeColor',[0.1 0.1 0.12],...  % bright industrial yellow
        'FaceLighting','gouraud','AmbientStrength',0.4,'DiffuseStrength',0.7);

    vRightX = [1.02 0.86 0.86 1.02];
    vRightY = [0.00 -0.54 -0.54 0.00];
    vRightZ = [0.08 0.08 0.38 0.38];
    patch('Parent',P,'XData',vRightX,'YData',vRightY,'ZData',vRightZ,...
        'FaceColor',[0.96 0.82 0.05],'EdgeColor',[0.1 0.1 0.12],...  % bright industrial yellow
        'FaceLighting','gouraud','AmbientStrength',0.4,'DiffuseStrength',0.7);

    % Hardened scraper wear-edge strip (dark steel blade bottom)
    eLeftX = [1.025 0.865 0.865 1.025];
    eLeftY = [0.00  0.54  0.54  0.00];
    eLeftZ = [0.08  0.08  0.15  0.15];
    patch('Parent',P,'XData',eLeftX,'YData',eLeftY,'ZData',eLeftZ,...
        'FaceColor',[0.15 0.16 0.18],'EdgeColor','none');

    eRightX = [1.025 0.865 0.865 1.025];
    eRightY = [0.00 -0.54 -0.54  0.00];
    eRightZ = [0.08  0.08  0.15  0.15];
    patch('Parent',P,'XData',eRightX,'YData',eRightY,'ZData',eRightZ,...
        'FaceColor',[0.15 0.16 0.18],'EdgeColor','none');

    % Hazard chevron stripes on deflector blade
    for s = [0.14, 0.28, 0.42]
        patch('Parent',P,...
            'XData',[1.02-s*0.28, 1.02-(s+0.05)*0.28, 1.02-(s+0.05)*0.28, 1.02-s*0.28],...
            'YData',[s, s+0.05, s+0.05, s],...
            'ZData',[0.16, 0.16, 0.36, 0.36],...
            'FaceColor',[0.10 0.10 0.12],'EdgeColor','none');
        patch('Parent',P,...
            'XData',[1.02-s*0.28, 1.02-(s+0.05)*0.28, 1.02-(s+0.05)*0.28, 1.02-s*0.28],...
            'YData',[-s, -(s+0.05), -(s+0.05), -s],...
            'ZData',[0.16, 0.16, 0.36, 0.36],...
            'FaceColor',[0.10 0.10 0.12],'EdgeColor','none');
    end

    % Top deflector lip (prevents rocks spilling over onto robot hood)
    tDefX = [1.02 0.86 0.83 0.99];
    tDefY = [0.00 0.54 0.50 0.00];
    tDefZ = [0.38 0.38 0.43 0.43];
    patch('Parent',P,'XData',tDefX,'YData',tDefY,'ZData',tDefZ,...
        'FaceColor',[0.95 0.40 0.05],'EdgeColor','none');

    tDefXR = [1.02 0.86 0.83 0.99];
    tDefYR = [0.00 -0.54 -0.50 0.00];
    tDefZR = [0.38 0.38 0.43 0.43];
    patch('Parent',P,'XData',tDefXR,'YData',tDefYR,'ZData',tDefZR,...
        'FaceColor',[0.95 0.40 0.05],'EdgeColor','none');

    % Center reinforcement spine & leveler blade badge
    drawBox(P, 0.06, 0.06, 0.32, [1.00, 0, 0.23], [0.15 0.16 0.18]);

    % Rear plate (dark)
    drawBox(T, 0.08, 0.90, 0.40, [-0.77 0 0.25], [0.12 0.14 0.18]);
    % Side panels (lighter grey)
    drawBox(T, 1.30, 0.06, 0.44, [0 0.46 0.28], [0.28 0.30 0.35]);
    drawBox(T, 1.30, 0.06, 0.44, [0 -0.46 0.28], [0.28 0.30 0.35]);

    % ======= WHEELS (6 wheels - 3 per side, rubber look) =======
    wheelR = 0.20;
    wx = [0.55 0.0 -0.55  0.55 0.0 -0.55];
    wy = [0.47 0.47 0.47 -0.47 -0.47 -0.47];
    wz = [0.20 0.20 0.20  0.20  0.20  0.20];
    robot.wheels = gobjects(6,1);
    for i = 1:6
        [cx,cy,cz] = cylinder(wheelR, 16);
        thick = 0.20;
        wX = cx + wx(i);
        wY = (cz-0.5)*thick + wy(i);
        wZ = cy + wz(i);
        robot.wheels(i) = surface('Parent',T,'XData',wX,'YData',wY,'ZData',wZ,...
            'FaceColor',[0.08 0.08 0.10],'EdgeColor','none',...
            'FaceLighting','gouraud','SpecularStrength',0.3);
        % Wheel rim (silver ring)
        [cx2,cy2,cz2] = cylinder(wheelR*0.65,16);
        thick2 = 0.22;
        surface('Parent',T,'XData',cx2+wx(i),'YData',(cz2-0.5)*thick2+wy(i),'ZData',cy2+wz(i),...
            'FaceColor',[0.62 0.64 0.68],'EdgeColor','none','FaceLighting','gouraud');
    end

    % Track belt between wheels (side view rubber belt)
    for side = [1 -1]
        bx = [-0.68 0.68 0.68 -0.68];
        by = [side*0.47 side*0.47 side*0.47 side*0.47];
        bz = [0.20 0.20 0.40 0.40];
        patch('Parent',T,'XData',bx,'YData',by,'ZData',bz,...
            'FaceColor',[0.08 0.08 0.10],'EdgeColor','none');
    end

    % ======= SENSOR HEAD (front-mounted LIDAR dome) =======
    [lx,ly,lz] = sphere(14);
    surface('Parent',T,'XData',lx*0.14+0.80,'YData',ly*0.14,'ZData',lz*0.08+0.65,...
        'FaceColor',[0.15 0.45 0.90],'EdgeColor','none',...
        'FaceLighting','gouraud','SpecularStrength',0.9,'SpecularExponent',25);
    % Sensor ring
    [cx,cy,cz] = cylinder(0.16,20);
    surface('Parent',T,'XData',cz*0.04+0.80,'YData',cx,'ZData',cy*0.5+0.65,...
        'FaceColor',[0.88 0.88 0.92],'EdgeColor','none','FaceLighting','gouraud');

    % ======= WARNING LIGHT (rotating beacon on top) =======
    [lx,ly,lz] = sphere(12);
    r = 0.10;
    robot.warningLight = surface('Parent',T,...
        'XData',lx*r+0.1,'YData',ly*r,'ZData',lz*r*0.7+0.70,...
        'FaceColor',[0.1 0.9 0.1],'EdgeColor','none',...
        'FaceLighting','gouraud','SpecularStrength',0.95,'SpecularExponent',30);
    % Beacon housing
    drawBox(T,0.12,0.12,0.08,[0.1 0 0.64],[0.20 0.20 0.24]);

    % ======= HEADLIGHTS =======
    [lx,ly,lz] = sphere(8);
    robot.headlightL = surface('Parent',T,...
        'XData',lx*0.055+0.76,'YData',ly*0.055+0.30,'ZData',lz*0.055+0.45,...
        'FaceColor',[1 1 0.75],'EdgeColor','none','Visible','off',...
        'FaceLighting','gouraud','SpecularStrength',1.0);
    robot.headlightR = surface('Parent',T,...
        'XData',lx*0.055+0.76,'YData',ly*0.055-0.30,'ZData',lz*0.055+0.45,...
        'FaceColor',[1 1 0.75],'EdgeColor','none','Visible','off',...
        'FaceLighting','gouraud','SpecularStrength',1.0);

    % ======= REAR STORAGE POUCH (LARGE CAPACITY - open top cargo hopper) =======
    % Enlarged rugged cargo hopper to hold 4+ removable obstacles.
    % Internal volume: 0.90 x 0.85 x 0.70 m = 0.5355 m^3 = 535.5 Litres
    pL = 0.90;  % length along X axis (robot forward direction)
    pW = 0.85;  % width  along Y axis (track-cross direction)
    pH = 0.70;  % depth  along Z axis (height of walls)
    pouchX = -1.20;  % centre of pouch X (behind rear plate at -0.77)
    pouchY = 0;      % centred on track
    pouchZ = 0.30;   % bottom of walls (sits lower for stable low CG)

    % Heavy mounting brackets (wider pouch needs stronger brackets)
    drawBox(T, 0.10, 0.10, 0.16, [-0.77,  0.30, pouchZ+0.08], [0.22 0.24 0.28]);
    drawBox(T, 0.10, 0.10, 0.16, [-0.77, -0.30, pouchZ+0.08], [0.22 0.24 0.28]);
    drawBox(T, 0.10, 0.10, 0.16, [-0.77,  0.00, pouchZ+0.08], [0.22 0.24 0.28]); % centre bracket

    % Pouch front wall (flush against chassis rear plate)
    drawBox(T, 0.04, pW, pH, [pouchX+pL/2, pouchY, pouchZ+pH/2], [0.12 0.14 0.18]);
    % Pouch back wall (rear exterior)
    drawBox(T, 0.04, pW, pH, [pouchX-pL/2, pouchY, pouchZ+pH/2], [0.10 0.12 0.15]);
    % Pouch left side wall
    drawBox(T, pL,   0.04, pH, [pouchX, pouchY+pW/2, pouchZ+pH/2], [0.10 0.12 0.15]);
    % Pouch right side wall
    drawBox(T, pL,   0.04, pH, [pouchX, pouchY-pW/2, pouchZ+pH/2], [0.10 0.12 0.15]);
    % Pouch bottom floor (thick steel base plate)
    drawBox(T, pL+0.06, pW+0.06, 0.05, [pouchX, pouchY, pouchZ], [0.14 0.16 0.20]);
    % Bright orange open-top rim highlight
    drawBox(T, pL+0.08, pW+0.08, 0.05, [pouchX, pouchY, pouchZ+pH+0.01], [0.95 0.42 0.02]);
    % Diagonal reinforcement ribs on rear exterior wall
    for yi = [-0.30 -0.10  0.10  0.30]
        drawBox(T, 0.03, 0.03, pH, [pouchX-pL/2-0.01, pouchY+yi, pouchZ+pH/2], [0.22 0.25 0.30]);
    end
    % Volume label on rear exterior
    text('Parent',T,'Position',[pouchX-pL/2-0.04, pouchY, pouchZ+pH*0.6],...
        'String',sprintf('POUCH\n535L'),'Color',[1 0.9 0.1],'FontSize',7,'FontWeight','bold',...
        'HorizontalAlignment','center');

    % Store pouch graphics handle for obstacle display
    robot.pouchMarker = [];

    % ======= ANTENNA =======
    drawBox(T, 0.025, 0.025, 0.28, [0.20 0.20 0.65], [0.55 0.58 0.65]);
    [lx,ly,lz] = sphere(6);
    surface('Parent',T,'XData',lx*0.035+0.20,'YData',ly*0.035+0.20,'ZData',lz*0.035+0.95,...
        'FaceColor',[0.88 0.08 0.08],'EdgeColor','none','FaceLighting','gouraud');

    % ======= SAFETY STRIPES (diagonal yellow/black on sides) =======
    for xi = [-0.5 0 0.5]
        patch('Parent',T,...
            'XData',[xi xi+0.15 xi+0.15 xi],...
            'YData',[0.435 0.435 0.435 0.435],...
            'ZData',[0.28 0.28 0.52 0.52],...
            'FaceColor',[0.98 0.85 0.02],'EdgeColor','none');
    end

    robot.stateLabel = gobjects(1);

    % ======= ROBOT STATE FIELDS =======
    robot.x = config.robotStartX;
    robot.y = config.track1Y;
    robot.z = 0.13;   % rail head height
    robot.yaw = 0;
    robot.speed = config.robotSpeed;
    robot.targetSpeed = config.robotSpeed;
    robot.state = "MOVING";
    robot.scanTimer = 0;
    robot.activeObstacleIdx = 0;
    robot.armSubState = "";
    robot.armDwellTimer = 0;
    robot.waitingTimer = 0;
    robot.preEmergencyState = "";
    robot.holdingObstacle = false;
    % Obstacle settles at pouch floor centre: X at pouch centre, Y=0, Z just above floor
    robot.pouchOffset = [pouchX, pouchY, pouchZ + pH*0.25];  % stored inside pouch lower third

    updateRobotTransform(robot);
end

% ---- Helper: draw a box patch ----
function drawBox(parentT, L, W, H, center, color)
    x = [-L/2 L/2]; y = [-W/2 W/2]; z = [0 H];
    [X,Y,Z] = ndgrid(x,y,z);
    vx = X(:)+center(1); vy = Y(:)+center(2); vz = Z(:)+center(3);
    faces = [1 2 4 3; 5 6 8 7; 1 2 6 5; 3 4 8 7; 1 3 7 5; 2 4 8 6];
    patch('Parent',parentT,'Vertices',[vx vy vz],'Faces',faces,...
        'FaceColor',color,'EdgeColor',[0.05 0.05 0.06],...
        'FaceLighting','gouraud','AmbientStrength',0.45,...
        'DiffuseStrength',0.70,'SpecularStrength',0.35);
end

