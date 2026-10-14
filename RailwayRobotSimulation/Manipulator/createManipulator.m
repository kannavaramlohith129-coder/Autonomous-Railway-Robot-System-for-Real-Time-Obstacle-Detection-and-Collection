function arm = createManipulator(robot)
% CREATEMANIPULATOR  -  4-DOF Serial Arm with mathematically correct kinematics.
%
% Denavit-Hartenberg Parameters (MATLAB yrotate(-q) corrected convention):
%   Joint | Type  | a(m)  | alpha | d(m) | theta
%   ------+-------+-------+-------+------+------
%     1   | Revolve| 0.00 | 90deg | 0.95 | q1 (base yaw, zrotate)
%     2   | Revolve| 0.60 |  0deg | 0.15 | q2 (shoulder pitch, +q=UP)
%     3   | Revolve| 0.48 |  0deg | 0.00 | q3 (elbow pitch, +q=elbow-UP)
%     4   | Revolve| 0.36 |  0deg | 0.00 | q4 (wrist pitch, -(q2+q3)=horiz)
%
% Joint ranges: q2 in [-85,+85]deg, q3 in [-125,+125]deg, q4 in [-85,+85]deg

    parent     = robot.mainTransform;
    mountPoint = [0.25 0 0.95];

    upperLen  = 0.60;
    foreLen   = 0.48;
    wristLen  = 0.18;
    fingerLen = 0.18;

    % ---- Pedestal ----
    [px, py, pz] = cylinder(0.09, 14);
    pz = pz * 0.12;
    surface('Parent', parent, 'XData', px+mountPoint(1), 'YData', py, ...
        'ZData', pz+0.85, 'FaceColor',[0.18 0.18 0.20], 'EdgeColor','none', ...
        'FaceLighting','gouraud', 'SpecularStrength',0.6);

    % ---- Transform chain ----
    arm.baseT     = hgtransform('Parent', parent);
    arm.shoulderT = hgtransform('Parent', arm.baseT);
    arm.elbowT    = hgtransform('Parent', arm.shoulderT);
    arm.wristT    = hgtransform('Parent', arm.elbowT);

    % ---- Joint spheres (large, chrome-look) ----
    arm.jBase     = makeJoint(arm.baseT,     0.095, [0.25 0.30 0.90]);  % blue  = base
    arm.jShoulder = makeJoint(arm.shoulderT, 0.082, [0.15 0.78 0.28]);  % green = shoulder
    arm.jElbow    = makeJoint(arm.elbowT,    0.070, [0.90 0.72 0.08]);  % gold  = elbow
    arm.jWrist    = makeJoint(arm.wristT,    0.058, [0.88 0.22 0.18]);  % red   = wrist

    % ---- Links (fat safety-yellow cylinders) ----
    arm.upperArm = makeLink(arm.shoulderT, upperLen, 0.095, [0.97 0.80 0.04]);
    arm.foreArm  = makeLink(arm.elbowT,   foreLen,  0.075, [0.97 0.80 0.04]);
    arm.wristArm = makeLink(arm.wristT,   wristLen, 0.055, [0.72 0.75 0.80]);

    % Hydraulic accent rings on upper arm
    makeRing(arm.shoulderT, upperLen*0.35, 0.055);
    makeRing(arm.shoulderT, upperLen*0.68, 0.055);

    % ---- Gripper fingers (dark, textured) ----
    arm.fingerL = makeFingerPatch(arm.wristT,  0.065, fingerLen, 0.032);
    arm.fingerR = makeFingerPatch(arm.wristT, -0.065, fingerLen, 0.032);

    % ---- Gripper "claw tip" indicator spheres (glow effect) ----
    arm.tipL = makeJoint(arm.wristT, 0.028, [0.05 0.95 0.30]);
    arm.tipR = makeJoint(arm.wristT, 0.028, [0.05 0.95 0.30]);

    % Store dims
    arm.mountPoint = mountPoint;
    arm.upperLen   = upperLen;
    arm.foreLen    = foreLen;
    arm.wristLen   = wristLen;
    arm.fingerLen  = fingerLen;

    % Home pose angles (FIXED sign convention: +q2 = arm UP, +q3 = elbow-UP)
    % q2=+65deg: shoulder raised 65deg — elbow at z=1.49m >> robot body top (0.65m) ✓
    % q3=+45deg: forearm direction at q2+q3=110deg (up+slightly back) — compact fold
    % q4=-80deg: wrist at joint limit for tightest fold
    % Result: arm folds HIGH above robot body, no intersection possible.
    arm.homeAngles      = [0, 65, 45, -80];
    arm.angles          = arm.homeAngles;
    arm.gripperOpen     = 1;   % 1=fully open, 0=fully closed
    arm.angularVelocity = [0, 0, 0, 0];  % [deg/s] for each joint

    arm = applyArmTransforms(arm);
end

% ---- Helpers ----
function h = makeJoint(parentT, r, color)
    [sx,sy,sz] = sphere(14);
    h = surface('Parent',parentT,'XData',sx*r,'YData',sy*r,'ZData',sz*r, ...
        'FaceColor',color,'EdgeColor','none', ...
        'FaceLighting','gouraud','SpecularStrength',0.85,'SpecularExponent',18);
end

function h = makeLink(parentT, len, radius, color)
    [cx,cy,cz] = cylinder(radius, 16);
    cz = cz*len;
    h = surface('Parent',parentT,'XData',cz,'YData',cx,'ZData',cy, ...
        'FaceColor',color,'EdgeColor','none', ...
        'FaceLighting','gouraud','AmbientStrength',0.40, ...
        'DiffuseStrength',0.75,'SpecularStrength',0.55,'SpecularExponent',14);
end

function makeRing(parentT, xPos, r)
    [cx,cy,cz] = cylinder(r*1.25, 12);
    cz = cz*0.055 + xPos;
    surface('Parent',parentT,'XData',cz,'YData',cx,'ZData',cy, ...
        'FaceColor',[0.82 0.85 0.90],'EdgeColor','none', ...
        'FaceLighting','gouraud','SpecularStrength',0.9,'SpecularExponent',28);
end

function h = makeFingerPatch(parentT, yOffset, len, w)
    tx = 0.18;
    v = [tx       yOffset-w  -w;
         tx+len   yOffset-w  -w;
         tx       yOffset+w  -w;
         tx+len   yOffset+w  -w;
         tx       yOffset-w   w;
         tx+len   yOffset-w   w;
         tx       yOffset+w   w;
         tx+len   yOffset+w   w];
    f = [1 2 4 3; 5 6 8 7; 1 2 6 5; 3 4 8 7; 1 3 7 5; 2 4 8 6];
    h = patch('Parent',parentT,'Vertices',v,'Faces',f, ...
        'FaceColor',[0.10 0.10 0.12],'EdgeColor',[0.04 0.04 0.05], ...
        'FaceLighting','gouraud','AmbientStrength',0.35,'SpecularStrength',0.45);
end


