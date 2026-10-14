function [targetAngles, armCmd] = ArmDirectionController(arm, robot, obstacle, config, dt)
% ARMDIRECTIONCONTROLLER - Decides WHAT direction and position the robot arm
%   should move to in order to effectively remove an obstacle, then computes
%   PD-corrected joint commands to reach that position.
%
% =========================================================================
% PURPOSE
% =========================================================================
% This controller answers the question:
%   "Given the obstacle type, size, and position -- WHERE should the arm go
%    and HOW should it correct errors to get there reliably?"
%
% It has two responsibilities:
%   1. DECISION LAYER (position/direction planner):
%      Chooses a grasp phase and orientation strategy based on obstacle type.
%      Different obstacles need different arm approaches:
%        mud        -> flat horizontal sweeping approach (low angle)
%        small_rock -> direct top-down grasp (small reach)
%        rock       -> angled descend grasp
%        big_rock   -> wide angled grasp + extra clearance hover
%        small_log  -> perpendicular side approach (log axis perpendicular)
%        log        -> side approach, IK uses 'push' phase for lateral contact
%        stone      -> flat low angle gripper
%
%   2. ERROR CORRECTION LAYER (PD controller):
%      Computes the position error (desired vs current gripper) and applies
%      a PD correction to the raw IK joint angles.
%      Error signal:  e   = targetTip - currentTip  (world-space, 3D)
%      PD correction: du  = Kp_tip*e + Kd_tip*de_dt
%      This correction is projected back through the Jacobian transpose:
%        dq = J^T * du   (task-space PD -> joint-space correction)
%
% =========================================================================
% MATHEMATICAL FRAMEWORK
% =========================================================================
%
%  Step 1 - IK target from obstacle type planner:
%    q_IK = moveArmToObstacle(arm, robot, obstacle, phase)
%
%  Step 2 - Desired gripper position from FK:
%    p_desired = FK(q_IK)   [world x,y,z]
%    p_current = FK(q)      [world x,y,z from arm.angles]
%
%  Step 3 - Task-space error:
%    e(t)  = p_desired - p_current                     [m]
%    de(t) = (e(t) - e(t-1)) / dt                     [m/s]
%
%  Step 4 - Task-space PD control:
%    F_pd = Kp_tip * e + Kd_tip * de                  [virtual force, N]
%
%  Step 5 - Project to joint space via Jacobian transpose:
%    J     = computeJacobian(arm, robot)              [3x4]
%    dq_pd = J' * F_pd                                [deg correction, 4x1]
%
%  Step 6 - Final corrected target:
%    targetAngles = q_IK + alpha * dq_pd
%    (alpha = 0.08: small blending factor so IK dominates, PD smooths)
%
% =========================================================================
% OUTPUTS
% =========================================================================
%   targetAngles - [q1, q2, q3, q4] joint angles in degrees (PD-corrected)
%   armCmd       - struct with diagnostics:
%     .phase         - grasp phase chosen for this obstacle type
%     .tipError_m    - 3D gripper position error [m]
%     .tipErrorNorm  - scalar magnitude of position error [m]
%     .pdCorrection  - dq_pd correction vector [deg]
%     .obstacleType  - type string of the obstacle

    % ---- Persistent error memory for derivative term ----
    persistent prevError;
    if isempty(prevError); prevError = zeros(3,1); end

    % ---- PD gains for task-space gripper position control ----
    %   Kp: proportional - stiffness of tip position correction
    %   Kd: derivative   - damping (suppresses oscillation during approach)
    Kp_tip = 8.0;   % [Nm/m] virtual stiffness
    Kd_tip = 2.5;   % [Nm.s/m] virtual damping
    alpha  = 0.08;  % blending weight: IK + small PD perturbation

    % ---- Step 1: Choose approach phase based on obstacle TYPE ----
    obsType = '';
    if isfield(obstacle, 'obstacleType')
        obsType = obstacle.obstacleType;
    end

    switch obsType
        case 'mud'
            % Mud: low flat approach, gripper sweeps horizontally
            phase = 'push';      % horizontal contact at mid-height

        case 'small_rock'
            % Small rock: compact - direct top-down grasp
            phase = 'grasp';

        case 'rock'
            % Medium rock: normal angled descend grasp
            phase = 'grasp';

        case 'big_rock'
            % Large boulder: hover higher first for clearance, then grasp
            phase = 'above';     % returns above-hover angles; caller drives DESCEND

        case 'small_log'
            % Small log: perpendicular side approach is better
            % (log axis is along Y, gripper approaches from above at angle)
            phase = 'grasp';

        case 'log'
            % Full log: too heavy to pick; use lateral push approach
            phase = 'push';

        case 'stone'
            % Flat stone slab: low shallow angle descent
            phase = 'grasp';

        otherwise
            phase = 'grasp';
    end

    % ---- Step 2: Compute IK target angles for the chosen phase ----
    q_IK = moveArmToObstacle(arm, robot, obstacle, phase);

    % ---- Step 3: Forward Kinematics - desired vs current tip position ----
    % Build a temporary arm struct at the IK angles to get desired tip
    armDesired        = arm;
    armDesired.angles = q_IK;
    p_desired = getGripperWorldPositionLocal(robot, armDesired);
    p_current = getGripperWorldPositionLocal(robot, arm);

    % ---- Step 4: Task-space position error ----
    e    = (p_desired - p_current)';   % column 3x1 [m]
    de   = (e - prevError) / max(dt, 1e-4);
    prevError = e;

    tipErrorNorm = norm(e);

    % ---- Step 5: PD virtual force ----
    F_pd = Kp_tip * e + Kd_tip * de;   % 3x1 virtual force

    % ---- Step 6: Jacobian (analytic, planar approximation) ----
    %   J [3x4]: columns are partial derivatives of tip w.r.t. each joint
    J = computeJacobianLocal(arm, robot);

    % ---- Step 7: Jacobian-transpose projection to joint space ----
    dq_rad = J' * F_pd;                 % 4x1, radians correction
    dq_deg = rad2deg(dq_rad);           % convert to degrees

    % Clamp large corrections (safety: max 8 deg correction per frame)
    dq_deg = max(min(dq_deg, 8), -8);

    % ---- Step 8: Final PD-blended target ----
    % IK gives the right posture; PD smoothly corrects for residual tip error
    targetAngles = q_IK + alpha * dq_deg(:)';

    % ---- Apply joint limits ----
    targetAngles(1) = max(min(targetAngles(1),  180), -180);
    targetAngles(2) = max(min(targetAngles(2),   85),  -85);
    targetAngles(3) = max(min(targetAngles(3),  125), -125);
    targetAngles(4) = max(min(targetAngles(4),   85),  -85);

    % ---- Diagnostic output struct ----
    armCmd.phase        = phase;
    armCmd.tipError_m   = e';
    armCmd.tipErrorNorm = tipErrorNorm;
    armCmd.pdCorrection = dq_deg';
    armCmd.obstacleType = obsType;
    armCmd.p_desired    = p_desired;
    armCmd.p_current    = p_current;
end

% =========================================================================
% LOCAL HELPER: Forward kinematics - gripper tip world position
% Mirrors getGripperWorldPosition.m exactly (same sign convention)
% =========================================================================
function tipWorld = getGripperWorldPositionLocal(robot, arm)
    q1 = arm.angles(1); q2 = arm.angles(2);
    q3 = arm.angles(3); q4 = arm.angles(4);

    Trobot    = makehgtform('translate',[robot.x robot.y robot.z], 'zrotate', robot.yaw);
    Tbase     = makehgtform('translate', arm.mountPoint, 'zrotate', deg2rad(q1));
    Tshoulder = makehgtform('yrotate', deg2rad(-q2));
    Telbow    = makehgtform('translate',[arm.upperLen 0 0], 'yrotate', deg2rad(-q3));
    Twrist    = makehgtform('translate',[arm.foreLen  0 0], 'yrotate', deg2rad(-q4));
    Ttip      = makehgtform('translate',[arm.wristLen + arm.fingerLen, 0, 0]);

    p = Trobot * Tbase * Tshoulder * Telbow * Twrist * Ttip * [0;0;0;1];
    tipWorld = p(1:3)';
end

% =========================================================================
% LOCAL HELPER: Analytic Jacobian (3 x 4)
%   Computes partial derivative of end-effector world-position w.r.t. each
%   joint angle using a numerical central-difference approach.
%   Exact analytic Jacobian derivation would require full chain-rule
%   differentiation; central differences give equivalent accuracy at dt=0.05s.
%   Delta used: 0.5 deg = 8.7e-3 rad (safe for linearization).
% =========================================================================
function J = computeJacobianLocal(arm, robot)
    delta = 0.5;   % degrees perturbation per joint
    J = zeros(3, 4);
    p0 = getGripperWorldPositionLocal(robot, arm);
    for k = 1:4
        armPlus = arm;
        armPlus.angles(k) = arm.angles(k) + delta;
        pPlus = getGripperWorldPositionLocal(robot, armPlus);
        % Forward difference column (convert delta to radians for J units)
        J(:, k) = (pPlus - p0)' / deg2rad(delta);
    end
end
