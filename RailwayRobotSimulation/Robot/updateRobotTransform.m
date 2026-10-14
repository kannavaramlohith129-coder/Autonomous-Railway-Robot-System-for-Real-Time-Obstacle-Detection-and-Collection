function updateRobotTransform(robot)
% UPDATEROBOTTRANSFORM - Applies robot.x/y/z/yaw to the hgtransform.
% Called by moveRobot every frame AND by RobotController during
% RETURNING_HOME lateral movement so the robot visually slides sideways.
    T = makehgtform('translate',[robot.x robot.y robot.z],'zrotate',robot.yaw);
    set(robot.mainTransform,'Matrix',T);
end
