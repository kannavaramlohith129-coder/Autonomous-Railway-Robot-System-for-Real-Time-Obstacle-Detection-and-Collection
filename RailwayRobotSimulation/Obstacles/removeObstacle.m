function obstacle = removeObstacle(obstacle, config)
% REMOVEOBSTACLE - Called once the gripper has carried the obstacle
% sideways and opened. Settles the obstacle onto the ground exactly where
% the gripper released it, with a small guaranteed push clear of the
% safety corridor so trackClearanceCheck.m reliably sees the track as
% clear afterward (the arm's fixed-waypoint sideways swing gets it most
% of the way there; this guarantees it).

    dropX = obstacle.position(1);
    dropY = obstacle.position(2);

    minClearY = config.corridorWidth + 0.5;
    if abs(dropY) < minClearY
        if dropY == 0
            dropY = minClearY; % default to the +Y side if it released dead-center
        else
            dropY = sign(dropY) * minClearY;
        end
    end

    finalPos = [dropX, dropY, obstacle.restHeight];
    obstacle = setObstaclePosition(obstacle, finalPos);

    obstacle.state = "REMOVED";
    obstacle.removed = true;
end
