function targetAngles = moveArmToObstacle(arm, robot, obstacle, phase)
% MOVEARMTOOBSTACLE - Analytic Planar Inverse Kinematics (IK)
%
% Calculates joint angles [q1, q2, q3, q4] so that the gripper tip reaches
% the obstacle position in 3D world space.
%
% Joint rotation convention in applyArmTransforms.m:
%   Joint 1 (base yaw) : zrotate(q1)
%   Joint 2 (shoulder) : yrotate(-q2)  [+q2 raises arm in +Z, -q2 lowers in -Z]
%   Joint 3 (elbow)    : yrotate(-q3)  [+q3 bends up in +Z, -q3 bends down in -Z]
%   Joint 4 (wrist)    : yrotate(-q4)  [tool orientation]

    if nargin < 4; phase = 'grasp'; end
    if nargin < 3 || isempty(robot) || isempty(obstacle)
        targetAngles = arm.homeAngles;
        return;
    end

    L1  = arm.upperLen;                             % 0.60 m
    L2  = arm.foreLen;                              % 0.48 m
    L3  = arm.wristLen + arm.fingerLen;             % 0.36 m
    mnt = arm.mountPoint;   % mount offset from robot origin

    % ---- Forward and lateral distance to obstacle in robot-local frame ----
    if isfield(robot, 'yaw')
        cy = cos(robot.yaw); sy = sin(robot.yaw);
        dx_rel = obstacle.position(1) - robot.x;
        dy_rel = obstacle.position(2) - robot.y;
        dx_local = cy * dx_rel + sy * dy_rel;
        dy_local = -sy * dx_rel + cy * dy_rel;
    else
        dx_local = obstacle.position(1) - robot.x;
        dy_local = obstacle.position(2) - robot.y;
    end

    % Base yaw to face obstacle
    dx_from_mount = dx_local - mnt(1);
    dy_from_mount = dy_local - mnt(2);
    q1 = atan2d(dy_from_mount, max(dx_from_mount, 0.05));

    % Planar reach along arm heading
    d_planar = sqrt(dx_from_mount^2 + dy_from_mount^2);

    % ---- Obstacle geometry in world Z ----
    mount_z = robot.z + mnt(3);                    % shoulder mount Z [world]
    obs_cz  = obstacle.position(3);                % obstacle center Z
    obs_top = obs_cz + obstacle.size * 0.75;       % top of flattened sphere
    obs_mid = obs_cz + obstacle.size * 0.50;       % mid height for push

    % ---- Target tip world Z and approach tool pitch per phase ----
    switch phase
        case 'above'
            % Hover 0.28m above obstacle top - safe approach clearance
            tool_pitch = deg2rad(-15);
            tip_z = obs_top + 0.28;

        case 'grasp'
            % Gripper contacts obstacle top with natural downward grasp angle (-22deg)
            % This keeps upper arm elevated over hood while forearm descends to rock
            tool_pitch = deg2rad(-22);
            tip_z = obs_top + 0.02;

        case 'push'
            % Horizontal contact at obstacle mid-height for lateral push
            tool_pitch = deg2rad(0);
            tip_z = obs_mid + 0.04;

        otherwise
            targetAngles = arm.homeAngles;
            return;
    end

    % ---- 2-DOF Planar IK for Wrist Joint ----
    % tip = wrist + [L3*cos(tool_pitch), 0, L3*sin(tool_pitch)]
    xw = max(d_planar - L3 * cos(tool_pitch), 0.10);
    zw = (tip_z - mount_z) - L3 * sin(tool_pitch);

    r = sqrt(xw^2 + zw^2);
    r = min(r, L1 + L2 - 0.02);           % clamp: max reach with 2cm margin
    r = max(r, abs(L1 - L2) + 0.02);      % clamp: min reach

    % Law of cosines
    cosAlpha = (r^2 - L1^2 - L2^2) / (2 * L1 * L2);
    cosAlpha = max(min(cosAlpha, 1.0), -1.0);

    alpha = acos(cosAlpha);               % elbow deflection magnitude [rad]
    phi   = atan2(zw, xw);                % angle from shoulder to wrist (negative when down)
    psi   = atan2(L2 * sin(alpha), L1 + L2 * cos(alpha)); % elbow triangle interior angle

    % Elbow-up configuration:
    % Shoulder aims higher (phi + psi), keeping upper arm above vehicle hood
    % Forearm bends down (-alpha) reaching over front bumper to track
    q2_rad = phi + psi;
    q3_rad = -alpha;
    q4_rad = tool_pitch - (q2_rad + q3_rad);

    % Convert to degrees and apply joint limits
    q2 = max(min(rad2deg(q2_rad),  85), -85);
    q3 = max(min(rad2deg(q3_rad), 125), -125);
    q4 = max(min(rad2deg(q4_rad),  85), -85);

    targetAngles = [q1, q2, q3, q4];
end

% =========================================================================
function tipWorld = getGripperWorldPosition(robot, arm)
% GETGRIPPERWORLDPOSITION  -  World-space [x y z] of gripper tip.
%
% Forward kinematics chain (matching applyArmTransforms.m convention):
%   p = Trobot * Tbase * Tshoulder * Telbow * Twrist * Ttip * [0;0;0;1]
%
% KEY: shoulder/elbow/wrist use yrotate(-q) to match the corrected sign
% convention (positive q = arm raises in +Z).

    q1 = arm.angles(1); q2 = arm.angles(2);
    q3 = arm.angles(3); q4 = arm.angles(4);

    % Homogeneous transform chain - NEGATED rotations for q2/q3/q4
    % This MUST match applyArmTransforms.m exactly
    Trobot    = makehgtform('translate',[robot.x robot.y robot.z], 'zrotate', robot.yaw);
    Tbase     = makehgtform('translate', arm.mountPoint, 'zrotate', deg2rad(q1));
    Tshoulder = makehgtform('yrotate', deg2rad(-q2));   % negated: +q2 => arm UP
    Telbow    = makehgtform('translate',[arm.upperLen 0 0], 'yrotate', deg2rad(-q3));
    Twrist    = makehgtform('translate',[arm.foreLen  0 0], 'yrotate', deg2rad(-q4));
    Ttip      = makehgtform('translate',[arm.wristLen + arm.fingerLen, 0, 0]);

    p = Trobot * Tbase * Tshoulder * Telbow * Twrist * Ttip * [0;0;0;1];
    tipWorld = p(1:3)';
end
