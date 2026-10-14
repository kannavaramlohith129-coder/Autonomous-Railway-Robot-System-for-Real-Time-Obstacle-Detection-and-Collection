function arm = applyArmTransforms(arm)
% APPLYARMTRANSFORMS - Push arm.angles / gripperOpen into the hgtransform chain.
%
% =========================================================================
% MATHEMATICAL FOUNDATION  -  4-DOF Serial Manipulator
% =========================================================================
% DH-style kinematic chain (MATLAB convention, yrotate sign corrected):
%
%   Joint 1 (base yaw)     : zrotate(q1)   - rotation about vertical Z axis
%   Joint 2 (shoulder)     : yrotate(-q2)  - pitch in robot XZ plane
%   Joint 3 (elbow)        : yrotate(-q3)  - pitch relative to upper arm
%   Joint 4 (wrist)        : yrotate(-q4)  - pitch for gripper orientation
%
% Why the NEGATION of q2/q3/q4?
%   MATLAB's makehgtform('yrotate', theta) uses the rotation matrix:
%       Ry(theta) = [cos(t) 0  sin(t)]
%                  [0       1  0     ]
%                  [-sin(t) 0  cos(t)]
%   Applied to [L, 0, 0]:  x' = L*cos(t),  z' = -L*sin(t)
%   => positive theta LOWERS the arm (-Z direction).
%
%   The IK in moveArmToObstacle uses STANDARD robotics convention:
%   Applied to [L, 0, 0]:  z = L*sin(q2)
%   => positive q2 RAISES the arm (+Z direction).
%
%   Fix: apply yrotate(-q) so that MATLAB's rendering matches IK convention:
%   yrotate(-q) on [L,0,0]: z' = -L*sin(-q) = L*sin(q)  ✓  (+Z for +q)
%
% Forward Kinematics (world-space tip position):
%   p_tip = T_robot * T_base * T_shoulder * T_elbow * T_wrist * T_tip * [0;0;0;1]
%   where each T_i = translate(a_i) * yrotate(-q_i)
%
% Lagrangian Dynamics (displayed in DynamicRobotController.m):
%   M(q)*q_ddot + C(q,q_dot)*q_dot + G(q) = tau
%   PD + Gravity compensation: tau = Kp*(qd-q) + Kd*(dqd-dq) + G(q)
% =========================================================================

    q1 = arm.angles(1);
    q2 = arm.angles(2);
    q3 = arm.angles(3);
    q4 = arm.angles(4);

    % Base: mount translation + base yaw (q1 about Z - no sign flip needed for zrotate)
    if isfield(arm,'baseT') && isgraphics(arm.baseT)
        set(arm.baseT,'Matrix', makehgtform('translate',arm.mountPoint,'zrotate',deg2rad(q1)));
    end

    % Shoulder: pitch - NEGATED for MATLAB yrotate convention
    % yrotate(-q2): z' = L1*sin(q2)  =>  +q2 raises arm  ✓
    if isfield(arm,'shoulderT') && isgraphics(arm.shoulderT)
        set(arm.shoulderT,'Matrix', makehgtform('yrotate', deg2rad(-q2)));
    end

    % Elbow: step along upper arm + pitch (NEGATED)
    % yrotate(-q3): forearm bends correctly relative to upper arm
    if isfield(arm,'elbowT') && isgraphics(arm.elbowT)
        set(arm.elbowT,'Matrix', makehgtform('translate',[arm.upperLen 0 0],'yrotate',deg2rad(-q3)));
    end

    % Wrist: step along forearm + pitch (NEGATED)
    % q4 = -(q2+q3) keeps gripper tip parallel to ground (horizontal)
    if isfield(arm,'wristT') && isgraphics(arm.wristT)
        set(arm.wristT,'Matrix', makehgtform('translate',[arm.foreLen 0 0],'yrotate',deg2rad(-q4)));
    end

    % ---- Gripper fingers open/close (slide along Y) ----
    gap = 0.022 + 0.090 * arm.gripperOpen;  % 0.022=closed gap, 0.112=fully open gap
    wl = arm.wristLen;
    fl = arm.fingerLen;
    fw = 0.032;

    vL = fVerts(wl,  gap, fl, fw);
    vR = fVerts(wl, -gap, fl, fw);
    if isfield(arm,'fingerL') && isgraphics(arm.fingerL); set(arm.fingerL,'Vertices',vL); end
    if isfield(arm,'fingerR') && isgraphics(arm.fingerR); set(arm.fingerR,'Vertices',vR); end

    % ---- Claw tip indicators follow wrist + finger offset ----
    tipOff = wl + fl + 0.01;
    if isfield(arm,'tipL') && isgraphics(arm.tipL)
        [sx,sy,sz] = sphere(6); r=0.026;
        set(arm.tipL,'XData',sx*r+tipOff,'YData',sy*r+gap,'ZData',sz*r);
    end
    if isfield(arm,'tipR') && isgraphics(arm.tipR)
        [sx,sy,sz] = sphere(6); r=0.026;
        set(arm.tipR,'XData',sx*r+tipOff,'YData',sy*r-gap,'ZData',sz*r);
    end
end

function v = fVerts(xStart, yOff, len, w)
    v = [xStart      yOff-w  -w;
         xStart+len  yOff-w  -w;
         xStart      yOff+w  -w;
         xStart+len  yOff+w  -w;
         xStart      yOff-w   w;
         xStart+len  yOff-w   w;
         xStart      yOff+w   w;
         xStart+len  yOff+w   w];
end
