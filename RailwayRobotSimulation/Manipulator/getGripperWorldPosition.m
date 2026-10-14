function tipWorld = getGripperWorldPosition(robot, arm)
% GETGRIPPERWORLDPOSITION - World-space [x y z] of gripper tip.
% Composes the full kinematic chain: robot -> base -> shoulder -> elbow -> wrist -> tip

    q1 = arm.angles(1); q2 = arm.angles(2);
    q3 = arm.angles(3); q4 = arm.angles(4);

    Trobot    = makehgtform('translate',[robot.x robot.y robot.z],'zrotate',robot.yaw);
    Tbase     = makehgtform('translate',arm.mountPoint,'zrotate',deg2rad(q1));
    Tshoulder = makehgtform('yrotate',deg2rad(-q2));
    Telbow    = makehgtform('translate',[arm.upperLen 0 0],'yrotate',deg2rad(-q3));
    Twrist    = makehgtform('translate',[arm.foreLen  0 0],'yrotate',deg2rad(-q4));
    Ttip      = makehgtform('translate',[arm.wristLen + arm.fingerLen 0 0]);

    p = Trobot * Tbase * Tshoulder * Telbow * Twrist * Ttip * [0;0;0;1];
    tipWorld = p(1:3)';
end
