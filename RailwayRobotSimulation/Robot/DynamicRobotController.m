function [tau, armOut] = DynamicRobotController(arm, qd_deg, qd_dot_deg, config, dt)
% DYNAMICROBOTCONTROLLER  Full rigid-body dynamics + PD+G controller.
%
% Robot dynamics equation (Lagrangian formulation):
%   M(q)*q_ddot + C(q,q_dot)*q_dot + G(q) = tau
%
% Control law (PD with gravity compensation):
%   tau = Kp*(qd - q) + Kd*(qd_dot - q_dot) + G(q)
%
% NOTE: This function computes torques for analysis/logging.
%       controlArm.m uses stable angle-slewing (dt=0.05s is too large
%       for direct torque integration - requires dt < 5ms for stability).
%
% Inputs:
%   arm        - arm struct with .angles [deg], .angularVelocity [deg/s]
%   qd_deg     - desired joint angles [degrees]
%   qd_dot_deg - desired joint velocities [deg/s]
%   config     - simulation config struct
%   dt         - timestep [s] (used for arm velocity estimate only)
%
% Outputs:
%   tau    - joint torques [Nm] (4x1): [tau_yaw; tau_shoulder; tau_elbow; tau_wrist]
%   armOut - arm struct (unchanged angles/velocity - kinematic controller handles motion)
%
% Reference: Spong, Hutchinson & Vidyasagar "Robot Modeling and Control" Ch.6-9

    armOut = arm;

    if nargin < 3 || isempty(qd_dot_deg)
        qd_dot_deg = zeros(1, 4);
    end

    % ---- Link Parameters (from createManipulator.m) ----
    L1 = arm.upperLen;  L2 = arm.foreLen;  L3 = arm.wristLen;  L4 = arm.fingerLen;
    m1=1.80; m2=1.20; m3=0.50; m4=0.30;
    lc1=L1/2; lc2=L2/2; lc3=L3/2; lc4=L4/2;
    Ic1=m1*L1^2/12; Ic2=m2*L2^2/12; Ic3=m3*L3^2/12; Ic4=m4*L4^2/12;
    g_acc = 9.81;

    % ---- Current state ----
    q   = deg2rad(arm.angles(:));
    dq  = deg2rad(arm.angularVelocity(:));
    qd  = deg2rad(qd_deg(:));
    dqd = deg2rad(qd_dot_deg(:));

    c2=cos(q(2)); c3=cos(q(3)); c4=cos(q(4));
    s3=sin(q(3)); s4=sin(q(4));
    c23=cos(q(2)+q(3));
    c234=cos(q(2)+q(3)+q(4));

    % ---- Mass/Inertia Matrix M(q) [4x4] ----
    M11 = Ic1+m1*lc1^2 + Ic2+m2*(L1^2+lc2^2+2*L1*lc2*c3) ...
        + Ic3+m3*(L1^2+L2^2+lc3^2+2*L1*L2*c3+2*L1*lc3*c23+2*L2*lc3*c4) ...
        + Ic4+m4*(L1^2+L2^2+L3^2+lc4^2+2*L1*L2*c3+2*L1*lc4*c234+2*L2*L3*c4);
    M22 = Ic2+m2*lc2^2 + Ic3+m3*(L2^2+lc3^2+2*L2*lc3*c4) + Ic4+m4*(L2^2+L3^2+lc4^2+2*L2*L3*c4);
    M33 = Ic3+m3*lc3^2 + Ic4+m4*(L3^2+lc4^2+2*L3*lc4);
    M44 = Ic4+m4*lc4^2;
    M12 = Ic2+m2*(lc2^2+L1*lc2*c3) + Ic3+m3*(L2^2+lc3^2+L1*L2*c3+L1*lc3*c23+2*L2*lc3*c4) ...
        + Ic4+m4*(L2^2+L3^2+lc4^2+L1*L2*c3+L1*lc4*c234+2*L2*L3*c4);
    M13 = Ic3+m3*(lc3^2+L2*lc3*c4) + Ic4+m4*(L3^2+lc4^2+L2*L3*c4);
    M14 = Ic4+m4*lc4^2;
    M23 = M13; M24 = M14; M34 = M14;
    M = [M11 M12 M13 M14; M12 M22 M23 M24; M13 M23 M33 M34; M14 M24 M34 M44];
    M = (M+M')/2 + 0.005*eye(4);

    % ---- Gravity Vector G(q) [4x1] ----
    G = zeros(4,1);
    G(1) = 0;
    G(2) = g_acc*(m1*lc1*c2 + m2*(L1*c2+lc2*c23) + m3*(L1*c2+L2*c23+lc3*c234) + m4*(L1*c2+L2*c23+L3*c234+lc4*c234));
    G(3) = g_acc*(m2*lc2*c23 + m3*(L2*c23+lc3*c234) + m4*(L2*c23+L3*c234+lc4*c234));
    G(4) = g_acc*(m3*lc3*c234 + m4*(L3*c234+lc4*c234));

    % ---- Coriolis/Centripetal Vector C(q,dq)*dq [4x1] ----
    h  = -(m2*L1*lc2*s3 + m3*(L1*L2*s3+L1*lc3*sin(q(2)+q(3))) + m4*(L1*L2*s3+L1*lc4*sin(q(2)+q(3)+q(4))));
    h2 = -(m3*L2*lc3*s4 + m4*L2*L3*s4);
    Cv = zeros(4,1);
    Cv(1) = h*(2*dq(1)*dq(2)+2*dq(1)*dq(3)+2*dq(1)*dq(4)) + h2*(2*dq(1)*dq(3)+2*dq(1)*dq(4));
    Cv(2) = -h*dq(1)^2 + h2*(2*dq(2)*dq(3)+2*dq(2)*dq(4)+dq(3)^2+dq(4)^2);
    Cv(3) = h*dq(1)^2 - h2*(dq(2)^2 - dq(3)*dq(4));
    Cv(4) = (h+h2)*dq(1)^2;

    % ---- PD + Gravity Compensation Controller ----
    %   tau = Kp*(qd-q) + Kd*(dqd-dq) + G(q)
    %   (G cancels gravity; Kp/Kd provide restoring + damping force)
    Kp = diag([10.0, 55.0, 40.0, 25.0]);
    Kd = diag([ 2.5, 14.0, 10.0,  6.0]);
    e   = qd  - q;
    e_d = dqd - dq;

    % CORRECT: tau includes G for gravity compensation
    % q_ddot = M^-1*(tau - Cv - G) = M^-1*(Kp*e + Kd*e_d - Cv)
    tau = Kp*e + Kd*e_d + G;   % <-- G here, NOT subtracting Cv from tau
    tau_max = [20; 90; 65; 35];
    tau = max(min(tau, tau_max), -tau_max);

    % ---- Forward dynamics (for reference only - NOT used to drive joints) ----
    % q_ddot = M \ (tau - Cv - G)
    % This equals M \ (Kp*e + Kd*e_d - Cv) which is correct
    % armOut.angles stay unchanged - controlArm.m handles motion via slewing
end
