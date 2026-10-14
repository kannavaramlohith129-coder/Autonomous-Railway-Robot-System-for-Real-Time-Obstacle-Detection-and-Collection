function decision = safetyDecision(obstacleInfo, trainStatus, robotState)
% SAFETYDECISION - Combines obstacle detection results, train safety status,
%   and RISK LEVEL into one of the documented high-level decisions:
%
%   "CONTINUE"        - No obstacle, or LOW-risk obstacle (train passes safely)
%   "SLOW_DOWN"       - MEDIUM/HIGH risk obstacle detected, closing in
%   "STOP"            - Stopping to evaluate
%   "REMOVE_OBSTACLE" - MEDIUM/HIGH risk: robot removes obstacle
%   "SKIP_LOW_RISK"   - LOW risk obstacle detected but NOT acted upon
%   "ISSUE_WARNING"   - Heavy obstacle, pushing aside
%   "EMERGENCY_STOP"  - Train emergency override
%   "RESUME"          - Track clear, resuming patrol

    if trainStatus.level == "EMERGENCY"
        decision = "EMERGENCY_STOP";
        return;
    end

    if ~obstacleInfo.found
        if robotState == "RESUMING"
            decision = "RESUME";
        else
            decision = "CONTINUE";
        end
        return;
    end

    % ---- Check risk level of detected obstacle ----
    obsRisk = "MEDIUM";   % default when riskLevel field not present
    if isfield(obstacleInfo.obstacle, 'riskLevel')
        obsRisk = obstacleInfo.obstacle.riskLevel;
    end

    % LOW risk: robot does NOT act - train can safely pass
    if obsRisk == "LOW"
        decision = "SKIP_LOW_RISK";
        return;
    end

    % MEDIUM or HIGH risk: act based on category and distance
    switch obstacleInfo.obstacle.category
        case "REMOVABLE"
            if obsRisk == "HIGH"
                decision = "REMOVE_OBSTACLE";  % HIGH urgency - act immediately
            elseif obstacleInfo.distance > 6
                decision = "SLOW_DOWN";
            else
                decision = "REMOVE_OBSTACLE";
            end
        case "HEAVY"
            decision = "ISSUE_WARNING";
        otherwise
            decision = "STOP";
    end
end
