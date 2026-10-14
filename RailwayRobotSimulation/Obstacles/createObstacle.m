function obstacle = createObstacle(ax, id, position, mass, config, obstacleType)
% CREATEOBSTACLE - Builds one obstacle on the track with realistic geometry
%   based on its physical type. Supported obstacle types encountered on real
%   railway tracks:
%
%   'mud'       - Mud / wet soil deposits (low mass, wide flat blob)
%   'small_rock'- Small rocks/stones (<15 kg, compact sphere)
%   'rock'      - Medium rocks (15-60 kg, flattened sphere)
%   'big_rock'  - Large boulders (>60 kg, lumpy large sphere)
%   'small_log' - Small tree logs/branches (<30 kg, cylindrical)
%   'log'       - Full tree logs (>30 kg, long cylinder)
%   'stone'     - Flat paving-style stone slabs (broad flat disc)
%
% If obstacleType is not provided, it is AUTO-INFERRED from mass.

    % ---- Auto-infer type from mass when not explicitly given ----
    if nargin < 6 || isempty(obstacleType)
        if mass < 5
            obstacleType = 'mud';
        elseif mass < 15
            obstacleType = 'small_rock';
        elseif mass < 40
            obstacleType = 'rock';
        elseif mass < 80
            obstacleType = 'big_rock';
        elseif mass < 100
            obstacleType = 'small_log';
        elseif mass < 150
            obstacleType = 'log';
        else
            obstacleType = 'stone';
        end
    end

    obstacle.obstacleType = obstacleType;

    % =========================================================================
    % BUILD GEOMETRY AND COLOR BASED ON TYPE
    % =========================================================================
    switch obstacleType

        case 'mud'
            % Wide, organic granular mud / wet soil mound deposit across the track
            % Realistic railway track mud deposit resting flat over rails and ballast
            width  = 0.72 + 0.06*rand;   % spans across rail heads
            height = 0.09 + 0.02*rand;   % mound height above base
            % Smooth radial mound grid resting flat (z >= 0)
            nRadial = 14; nTheta = 24;
            [rNorm, thGrid] = meshgrid(linspace(0, 1, nRadial), linspace(0, 2*pi, nTheta));
            % Organic uneven boundary: wider along rails and ballast
            rBoundary = width * (0.92 + 0.08*cos(3*thGrid) + 0.04*sin(5*thGrid));
            ux = rNorm .* (rBoundary .* cos(thGrid));
            uy = rNorm .* (rBoundary * 0.88 .* sin(thGrid));
            % Smooth domed mound: peaks in center, tapers flat to ground at edge
            uz = height * (1 - rNorm.^1.8);
            % Natural soil texture variation
            clods = 0.012 * cos(3*ux) .* sin(3*uy) .* (1 - rNorm);
            uz = max(0.002, uz + clods);
            
            obstacle.shapeX = ux;
            obstacle.shapeY = uy;
            obstacle.shapeZ = uz;
            obstacle.restHeight = 0.08;   % base sits on ballast & sleepers around rails
            mudBrown = [0.28 + 0.03*rand, 0.20 + 0.02*rand, 0.11 + 0.02*rand];
            rockColor = mudBrown;
            obstacle.size = width;

        case 'small_rock'
            % Small compact rock / stone - roughly spherical
            sz = 0.14 + 0.008*mass;
            sz = min(sz, 0.28);
            [ux, uy, uz] = sphere(10);
            obstacle.shapeX = ux * sz * (0.90 + 0.1*rand);
            obstacle.shapeY = uy * sz * (0.90 + 0.1*rand);
            obstacle.shapeZ = uz * sz * 0.80;
            obstacle.restHeight = sz * 0.80;
            grayStone = [0.52 + 0.08*rand, 0.50 + 0.06*rand, 0.48 + 0.06*rand];
            rockColor = grayStone;
            obstacle.size = sz;

        case 'rock'
            % Medium rock - lumpy, flattened sphere
            sz = 0.18 + 0.006*mass;
            sz = min(sz, 0.45);
            [ux, uy, uz] = sphere(12);
            obstacle.shapeX = ux * sz;
            obstacle.shapeY = uy * sz * (0.85 + 0.1*rand);
            obstacle.shapeZ = uz * sz * 0.72;
            obstacle.restHeight = sz * 0.72;
            rockGray = [0.42 + 0.06*rand, 0.38 + 0.06*rand, 0.32 + 0.06*rand];
            rockColor = rockGray;
            obstacle.size = sz;

        case 'big_rock'
            % Large boulder - very prominent, chunky
            sz = 0.22 + 0.004*mass;
            sz = min(sz, 0.65);
            [ux, uy, uz] = sphere(14);
            obstacle.shapeX = ux * sz;
            obstacle.shapeY = uy * sz * (0.90 + 0.08*rand);
            obstacle.shapeZ = uz * sz * 0.78;
            obstacle.restHeight = sz * 0.78;
            darkRock = [0.38 + 0.04*rand, 0.32 + 0.04*rand, 0.26 + 0.04*rand];
            rockColor = darkRock;
            obstacle.size = sz;

        case 'small_log'
            % Small tree log / branch - horizontal cylinder lying across track
            logR  = 0.08 + 0.001*mass;
            logL  = 0.55 + 0.006*mass;
            logL  = min(logL, 0.95);
            [cx, cz, cy] = cylinder(logR, 16);
            cy = (cy - 0.5) * logL;
            % Natural 12-degree angle across rails
            ang = deg2rad(12);
            obstacle.shapeX = cx * cos(ang) - cy * sin(ang);
            obstacle.shapeY = cx * sin(ang) + cy * cos(ang);
            obstacle.shapeZ = cz;
            obstacle.restHeight = logR;
            logBrown = [0.42 + 0.04*rand, 0.28 + 0.03*rand, 0.12 + 0.02*rand];
            rockColor = logBrown;
            obstacle.size = max(logR*2, logL/2);

        case 'log'
            % Full heavy tree trunk - horizontal cylinder lying across both rails
            logR  = 0.14 + 0.001*mass;
            logL  = 1.25 + 0.005*mass;
            logL  = min(logL, 2.00);
            [cx, cz, cy] = cylinder(logR, 18);
            cy = (cy - 0.5) * logL;
            % Natural 8-degree angle across rails
            ang = deg2rad(8);
            obstacle.shapeX = cx * cos(ang) - cy * sin(ang);
            obstacle.shapeY = cx * sin(ang) + cy * cos(ang);
            obstacle.shapeZ = cz;
            obstacle.restHeight = logR;
            darkBrown = [0.32 + 0.03*rand, 0.20 + 0.03*rand, 0.08 + 0.02*rand];
            rockColor = darkBrown;
            obstacle.size = max(logR*2, logL/2);

        case 'stone'
            % Flat paving stone / concrete slab - wide, thin disc
            stoneW = 0.38 + 0.004*mass;
            stoneW = min(stoneW, 0.80);
            stoneH = 0.08 + 0.001*mass;
            [ux, uy, uz] = ellipsoid(0, 0, 0, stoneW, stoneW*0.75, stoneH, 10);
            obstacle.shapeX = ux;
            obstacle.shapeY = uy;
            obstacle.shapeZ = uz;
            obstacle.restHeight = stoneH;
            stoneGray = [0.58 + 0.06*rand, 0.56 + 0.06*rand, 0.52 + 0.06*rand];
            rockColor = stoneGray;
            obstacle.size = stoneW;

        otherwise
            % Fallback: generic rock
            sz = 0.15 + 0.01*mass;
            sz = min(sz, 0.6);
            [ux, uy, uz] = sphere(12);
            obstacle.shapeX = ux * sz;
            obstacle.shapeY = uy * sz;
            obstacle.shapeZ = uz * sz * 0.75;
            obstacle.restHeight = sz * 0.75;
            rockColor = [0.42 0.30 0.20] + 0.06*(rand(1,3)-0.5);
            obstacle.size = sz;
    end

    % ---- Store original undeformed dimensions across ALL types ----
    obstacle.origShapeX = obstacle.shapeX;
    obstacle.origShapeY = obstacle.shapeY;
    obstacle.origShapeZ = obstacle.shapeZ;
    obstacle.origHeight = obstacle.restHeight;
    obstacle.origWidth  = obstacle.size;
    obstacle.sandRelX   = [];
    obstacle.sandRelY   = [];
    obstacle.sandRelZ   = [];
    obstacle.sandColors = [];
    obstacle.sandSizes  = [];
    obstacle.sandHandle = [];

    % ---- Place at rest position ----
    restPos = [position(1), position(2), obstacle.restHeight];

    % ---- Render surface ----
    obstacle.baseColor = rockColor;
    obstacle.handle = surf(ax, ...
        obstacle.shapeX + restPos(1), ...
        obstacle.shapeY + restPos(2), ...
        obstacle.shapeZ + restPos(3), ...
        'FaceColor', rockColor, 'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', 'SpecularStrength', 0.15, 'AmbientStrength', 0.5);

    % ---- Granular Sand/Mud Particles for Loose Soil Deposits ----
    if strcmp(obstacleType, 'mud')
        nP = 48;
        pTheta = rand(1, nP) * 2 * pi;
        pR     = sqrt(rand(1, nP)) * (obstacle.size * 0.90);
        pX     = pR .* cos(pTheta);
        pY     = pR .* sin(pTheta) * 0.85;
        pZ     = max(0.006, height * (1 - (pR/obstacle.size).^1.8) + 0.005 * randn(1, nP));
        obstacle.sandRelX = pX;
        obstacle.sandRelY = pY;
        obstacle.sandRelZ = pZ;
        obstacle.origSandRelX = pX;
        obstacle.origSandRelY = pY;
        obstacle.origSandRelZ = pZ;
        sandPal = [0.38 0.28 0.16; 0.48 0.36 0.22; 0.28 0.20 0.10; 0.52 0.42 0.26; 0.22 0.16 0.08];
        cIdx    = randi(size(sandPal,1), nP, 1);
        obstacle.sandColors = sandPal(cIdx, :);
        obstacle.sandSizes  = 16 + 18*rand(1, nP);
        obstacle.origSandSizes = obstacle.sandSizes;
        obstacle.sandHandle = scatter3(ax, pX + restPos(1), pY + restPos(2), pZ + restPos(3), ...
            obstacle.sandSizes, obstacle.sandColors, 'filled', ...
            'MarkerEdgeColor', [0.18 0.12 0.06], 'LineWidth', 0.5);
    else
        obstacle.origSandRelX = [];
        obstacle.origSandRelY = [];
        obstacle.origSandRelZ = [];
        obstacle.origSandSizes = [];
    end

    % ---- Contact shadow ----
    shadowR = obstacle.size * 1.15;
    [sx, sy] = pol2cart(linspace(0, 2*pi, 16), shadowR);
    obstacle.shadow = patch(ax, sx+restPos(1), sy+restPos(2), zeros(1,16)+0.015, ...
        [0 0 0], 'FaceAlpha', 0.18, 'EdgeColor', 'none');

    % ---- 3D Floating Label (clean, small, normal text without mass) ----
    labelZ = obstacle.restHeight * 2 + 0.38;
    displayName = cleanObstacleTitle(obstacleType);
    obstacle.label = text(ax, restPos(1), restPos(2), labelZ, ...
        displayName, ...
        'Color', [0.92 0.94 0.98], 'FontWeight', 'normal', 'HorizontalAlignment', 'center', ...
        'BackgroundColor', [0.10 0.12 0.16], 'Margin', 2.5, 'FontSize', 7.5, 'Interpreter', 'none');

    % ---- Pre-calculate Risk values for struct ----
    tempObs.obstacleType = obstacleType;
    tempObs.mass         = mass;
    tempObs.size         = obstacle.size;
    tempObs.position     = restPos;
    [riskLvl, riskSc, riskRsn] = assessObstacleRisk(tempObs, config);

    % ---- Data struct ----
    obstacle.id         = id;
    obstacle.position   = restPos;
    obstacle.initPosition = restPos;
    obstacle.mass       = mass;
    obstacle.obstacleType = obstacleType;
    obstacle.category   = classifyObstacle(struct('mass', mass), config);
    obstacle.removable  = obstacle.category == "REMOVABLE";
    obstacle.riskLevel  = riskLvl;
    obstacle.riskScore  = riskSc;
    obstacle.riskReason = riskRsn;
    obstacle.detectionDistance = config.scannerRange;
    obstacle.state      = "ON_TRACK";   % ON_TRACK | HELD | IN_POUCH | REMOVED
    obstacle.removed    = false;
    obstacle.skipped    = false;
end

function s = cleanObstacleTitle(t)
    switch lower(char(t))
        case 'mud'; s = 'Mud';
        case 'small_rock'; s = 'Rock';
        case 'rock'; s = 'Rock';
        case 'big_rock'; s = 'Rock';
        case 'small_log'; s = 'Log';
        case 'log'; s = 'Log';
        case 'stone'; s = 'Stone';
        otherwise
            s = strrep(char(t), '_', ' ');
            if ~isempty(s)
                s = [upper(s(1)), lower(s(2:end))];
            end
    end
end
