function nextState = RobotStateMachine(currentState, ctx)
% ROBOTSTATEMACHINE - Pure state transition logic. No graphics here.
%
% States:
%   MOVING          - patrolling along Track 1
%   OBSTACLE_DETECTED / CLASSIFYING / APPROACHING
%   STOPPED         - reached obstacle, starting arm sequence
%   REMOVING        - arm picking up REMOVABLE obstacle into pouch
%   PUSHING         - arm pushing HEAVY obstacle off track sideways
%   CLEARANCE_CHECK - verify track is clear after action
%   RESUMING        - resuming patrol speed
%   RETURNING_HOME  - all obstacles done, lateral move to Track 2 (home)
%   WARNING / WAITING / EMERGENCY_STOP
%   COMPLETED

    % Train emergency always overrides
    if isfield(ctx,'trainEmergency') && ctx.trainEmergency && currentState ~= "EMERGENCY_STOP"
        nextState = "EMERGENCY_STOP";
        return;
    end

    switch currentState
        case "MOVING"
            if isfield(ctx,'allClear') && ctx.allClear
                nextState = "RETURNING_HOME";
            elseif isfield(ctx,'obstacleDetected') && ctx.obstacleDetected
                nextState = "OBSTACLE_DETECTED";
            else
                nextState = "MOVING";
            end

        case "OBSTACLE_DETECTED"
            nextState = "CLASSIFYING";

        case "CLASSIFYING"
            nextState = "APPROACHING";

        case "APPROACHING"
            if isfield(ctx,'closeEnoughToAct') && ctx.closeEnoughToAct
                nextState = "STOPPED";
            else
                nextState = "APPROACHING";
            end

        case "STOPPED"
            if isfield(ctx,'obstacleClass') && ctx.obstacleClass == "HEAVY"
                nextState = "PUSHING";
            else
                nextState = "REMOVING";
            end

        case "REMOVING"
            if isfield(ctx,'armSequenceDone') && ctx.armSequenceDone
                nextState = "CLEARANCE_CHECK";
            else
                nextState = "REMOVING";
            end

        case "PUSHING"
            if isfield(ctx,'armSequenceDone') && ctx.armSequenceDone
                nextState = "CLEARANCE_CHECK";
            else
                nextState = "PUSHING";
            end

        case "CLEARANCE_CHECK"
            if isfield(ctx,'trackClear') && ctx.trackClear
                nextState = "RESUMING";
            else
                nextState = "CLEARANCE_CHECK";
            end

        case "RESUMING"
            nextState = "MOVING";

        case "RETURNING_HOME"
            if isfield(ctx,'atHomeTrack') && ctx.atHomeTrack
                nextState = "COMPLETED";
            else
                nextState = "RETURNING_HOME";
            end

        case "WARNING"
            nextState = "WAITING";

        case "WAITING"
            if isfield(ctx,'trainClearNow') && ctx.trainClearNow
                nextState = "RESUMING";
            else
                nextState = "WAITING";
            end

        case "EMERGENCY_STOP"
            if isfield(ctx,'trainClearNow') && ctx.trainClearNow && ...
               ~(isfield(ctx,'trainEmergency') && ctx.trainEmergency)
                nextState = "RESUMING";
            else
                nextState = "EMERGENCY_STOP";
            end

        case "COMPLETED"
            nextState = "COMPLETED";

        otherwise
            nextState = "MOVING";
    end
end
