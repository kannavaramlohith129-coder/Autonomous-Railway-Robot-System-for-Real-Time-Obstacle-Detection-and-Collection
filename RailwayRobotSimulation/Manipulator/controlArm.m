function [arm, reached] = controlArm(arm, targetAngles, targetGripper, config, dt)
% CONTROLARM - Quintic Polynomial (Minimum-Jerk) Trajectory Generator & Joint Controller
%
% =========================================================================
% MATHEMATICAL FOUNDATION: MINIMUM-JERK TRAJECTORY GENERATION
% =========================================================================
% Standard industrial robotics uses 5th-order (Quintic) polynomial interpolation
% to eliminate robotic stiffness, abrupt velocity steps, and actuator jerk:
%
%   Boundary conditions on normalized time tau = t / T in [0, 1]:
%     s(0)  = 0,     s(1)  = 1       (exact start and target angles)
%     s'(0) = 0,     s'(1) = 0       (zero initial & terminal velocities)
%     s"(0) = 0,     s"(1) = 0       (zero initial & terminal accelerations)
%
%   Quintic Blend Function:
%     s(tau)  = 10*tau^3 - 15*tau^4 + 6*tau^5
%     s'(tau) = (30*tau^2 - 60*tau^3 + 30*tau^4) / T     [rad/s or deg/s]
%     s"(tau) = (60*tau - 180*tau^2 + 120*tau^3) / T^2   [rad/s^2 or deg/s^2]
%
%   Joint State Trajectories:
%     q(t)    = q_start + (q_target - q_start) * s(tau)
%     q_dot   = (q_target - q_start) * s'(tau)
%     q_ddot  = (q_target - q_start) * s"(tau)
%
%   Lagrangian Actuator Torques:
%     tau_act = M(q)*q_ddot + C(q,q_dot)*q_dot + G(q)
%     tau_ctl = Kp*(q_des - q) + Kd*(q_dot_des - q_dot) + G(q)
% =========================================================================

    % Ensure trajectory state struct exists in arm
    if ~isfield(arm, 'traj') || isempty(arm.traj)
        arm.traj = struct();
        arm.traj.q_start    = arm.angles;
        arm.traj.q_target   = targetAngles;
        arm.traj.g_start    = arm.gripperOpen;
        arm.traj.g_target   = targetGripper;
        arm.traj.time       = 0;
        arm.traj.duration   = 0.5;
        arm.traj.g_time     = 0;
        arm.traj.g_duration = 0.4;
    end

    % Detect change in target joint angles (new waypoint triggered)
    targetChanged = any(abs(targetAngles - arm.traj.q_target) > 0.05);
    if targetChanged
        arm.traj.q_start  = arm.angles;
        arm.traj.q_target = targetAngles;
        arm.traj.time     = 0;
        % Calculate duration based on maximum joint displacement and configured speed
        maxDisplacement   = max(abs(targetAngles - arm.angles));
        armSpeed          = max(10, config.armSpeed);
        arm.traj.duration = max(0.45, maxDisplacement / armSpeed);
    end

    % Detect change in target gripper state
    gripperChanged = abs(targetGripper - arm.traj.g_target) > 0.02;
    if gripperChanged
        arm.traj.g_start    = arm.gripperOpen;
        arm.traj.g_target   = targetGripper;
        arm.traj.g_time     = 0;
        arm.traj.g_duration = max(0.30, abs(targetGripper - arm.gripperOpen) / 1.5);
    end

    % ---- 1. Joint Trajectory Evaluation (Quintic Minimum-Jerk) ----
    arm.traj.time = arm.traj.time + dt;
    T = max(1e-4, arm.traj.duration);
    tau = min(1.0, max(0.0, arm.traj.time / T));

    % Quintic polynomial basis and derivatives
    s      = 10*tau^3 - 15*tau^4 + 6*tau^5;
    s_dot  = (30*tau^2 - 60*tau^3 + 30*tau^4) / T;
    s_ddot = (60*tau - 180*tau^2 + 120*tau^3) / (T^2);

    deltaQ = arm.traj.q_target - arm.traj.q_start;
    arm.angles              = arm.traj.q_start + deltaQ * s;
    arm.angularVelocity     = deltaQ * s_dot;
    arm.angularAcceleration = deltaQ * s_ddot;

    % ---- 2. Gripper Trajectory Evaluation (Cubic Smooth Blend) ----
    arm.traj.g_time = arm.traj.g_time + dt;
    Tg = max(1e-4, arm.traj.g_duration);
    tau_g = min(1.0, max(0.0, arm.traj.g_time / Tg));

    sg = 3*tau_g^2 - 2*tau_g^3;   % smooth cubic blend
    arm.gripperOpen = arm.traj.g_start + (arm.traj.g_target - arm.traj.g_start) * sg;

    % ---- 3. Apply Forward Kinematic Transforms to Scene Graph ----
    arm = applyArmTransforms(arm);

    % ---- 4. Check Convergence ----
    % Reached when trajectory time >= duration AND angles within 0.8 deg
    jointsReached  = (tau >= 1.0) || all(abs(targetAngles - arm.angles) < 0.8);
    gripperReached = (tau_g >= 1.0) || (abs(targetGripper - arm.gripperOpen) < 0.02);
    reached = jointsReached && gripperReached;
end
