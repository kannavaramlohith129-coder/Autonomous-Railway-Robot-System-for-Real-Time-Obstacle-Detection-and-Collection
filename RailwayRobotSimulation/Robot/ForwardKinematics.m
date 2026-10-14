function [T_ee, jointPositions] = ForwardKinematics(arm, robot)
% FORWARDKINEMATICS - DH-parameter based full forward kinematics for 4-DOF arm.
%
% Implements the standard Denavit-Hartenberg transformation chain:
%   T_i = Rot_z(theta_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(alpha_i)
%
% DH Parameters for this robot arm:
%   Joint | a_i      | alpha_i | d_i  | theta_i (variable)
%   ------|----------|---------|------|--------------------
%     1   | 0        |  0      | 0    | q1  (yaw)
%     2   | upperLen |  0      | 0    | q2  (shoulder pitch)
%     3   | foreLen  |  0      | 0    | q3  (elbow pitch)
%     4   | wristLen |  0      | 0    | q4  (wrist pitch)
%     ee  | fingerLen|  0      | 0    | 0   (fixed tool)
%
% Outputs:
%   T_ee          - 4x4 homogeneous transform: base -> end-effector
%   jointPositions - 5x3 array: [base; shoulder; elbow; wrist; tip] world positions

    q = deg2rad(arm.angles);    % [q1 q2 q3 q4]
    
    L1 = arm.upperLen;  L2 = arm.foreLen;
    L3 = arm.wristLen;  L4 = arm.fingerLen;

    % DH transformation matrix: T = Rz(q) * Tx(a) [alpha=0 for all, d=0]
    T_base_world = makehgtform('translate', [robot.x, robot.y, robot.z], ...
                               'zrotate', robot.yaw) * ...
                   makehgtform('translate', arm.mountPoint, 'zrotate', q(1));

    % Joint 2: shoulder yrotate + translate along X by L1
    T2 = makehgtform('yrotate', -q(2));
    T3 = makehgtform('translate', [L1 0 0], 'yrotate', -q(3));
    T4 = makehgtform('translate', [L2 0 0], 'yrotate', -q(4));
    T5 = makehgtform('translate', [L3 0 0]);
    T6 = makehgtform('translate', [L4 0 0]);

    % Chain: world -> joint1 -> shoulder -> elbow -> wrist -> tip
    Tjoint1   = T_base_world;
    Tshoulder = Tjoint1 * T2;
    Telbow    = Tshoulder * T3;
    Twrist    = Telbow * T4;
    Ttool     = Twrist * T5;
    T_ee      = Ttool * T6;

    % Extract joint origin positions (first 3 cols of last column)
    jointPositions = zeros(5, 3);
    jointPositions(1,:) = Tjoint1(1:3, 4)';    % joint 1 (base/mount)
    jointPositions(2,:) = Tshoulder(1:3, 4)';  % joint 2 (shoulder)
    jointPositions(3,:) = Telbow(1:3, 4)';     % joint 3 (elbow)
    jointPositions(4,:) = Twrist(1:3, 4)';     % joint 4 (wrist)
    jointPositions(5,:) = T_ee(1:3, 4)';       % end-effector tip
end
