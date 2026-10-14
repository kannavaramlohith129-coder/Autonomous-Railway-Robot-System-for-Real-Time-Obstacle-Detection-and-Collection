function mlResult = ObstacleRemovalMLAnalyzer(robot, arm, obstacles, config, t)
% OBSTACLEREMOVALMLANALYZER - Machine Learning analysis module for the
%   Railway Robot Simulation.
%
% =========================================================================
% PURPOSE
% =========================================================================
% This module analyses HOW CORRECTLY actions and situations are working by:
%   1. Extracting feature vectors from the current simulation state
%   2. Running a trained ML model (k-NN classifier + Gradient Descent optimizer)
%      to predict the correct action for the current situation
%   3. Comparing the robot's actual action with the model's recommended action
%   4. Optimizing PD controller gains using gradient descent to minimize the
%      error between actual and desired arm trajectory
%
% =========================================================================
% ML MODEL: k-Nearest Neighbors (k-NN) Classifier
% =========================================================================
% Why k-NN?
%   - Interpretable and fast for small feature spaces
%   - No offline training needed: uses a labeled reference dataset built
%     from known good trajectories
%   - Works well for classifying robot action states based on sensor readings
%
% Feature vector (10 features):
%   [obsDistance, obsMass, obsType_enc, armQ1, armQ2, armQ3, armQ4,
%    tipErrorNorm, robotSpeed, trainDist]
%
% Labels (action classes):
%   0 = CONTINUE_MOVING   (no obstacle, robot patrols)
%   1 = APPROACH_SLOW     (obstacle detected, closing in)
%   2 = ARM_PICK          (removable obstacle, arm picking sequence)
%   3 = PLOW_PUSH         (heavy obstacle, front plow push)
%   4 = EMERGENCY_STOP    (train approaching, halt immediately)
%   5 = RETURN_HOME       (all clear, return to home track)
%
% =========================================================================
% OPTIMIZER: Gradient Descent for PD Gain Tuning
% =========================================================================
% Objective: Minimize tracking error of the arm over the current time window
%   L(Kp,Kd) = (1/N) * sum_i || q_desired_i - q_actual_i ||^2
%            + lambda * (||Kp||^2 + ||Kd||^2)   (L2 regularization)
%
% Update rule (gradient descent):
%   Kp_new = Kp - lr * dL/dKp
%   Kd_new = Kd - lr * dL/dKd
%
% Gradient:
%   dL/dKp_j = -(2/N) * sum_i e_j(i)   (e = q_desired - q_actual)
%   dL/dKd_j = -(2/N) * sum_i de_j(i)  (de = velocity error)
%
% The optimizer runs once per analysis call and suggests updated gains.
%
% =========================================================================
% OUTPUTS: mlResult struct with fields:
%   .predictedAction  - action class string the ML model recommends
%   .confidence       - confidence score [0-1] based on k-NN distance
%   .actualAction     - current robot state mapped to action class
%   .actionMatch      - true if model agrees with robot's actual action
%   .featureVector    - 10-feature input to the model
%   .optimizedKp      - gradient-descent optimized Kp gains (4x1)
%   .optimizedKd      - gradient-descent optimized Kd gains (4x1)
%   .trackingError    - mean joint tracking error over log window [deg]
%   .optimizerLoss    - final loss value after optimizer step
%   .analysisTime     - simulation time of this analysis [s]
% =========================================================================

    % ---- Persistent state for optimizer and k-NN reference dataset ----
    persistent refDataset;
    persistent optimKp optimKd;
    persistent prevAnalysisT;

    if isempty(prevAnalysisT); prevAnalysisT = -1; end

    % Initialize PD gains from config if first call
    if isempty(optimKp)
        optimKp = config.armKp(:);   % 4x1 column
        optimKd = config.armKd(:);
    end

    % Build reference dataset once (labeled training samples)
    if isempty(refDataset)
        refDataset = buildReferenceDataset();
    end

    % ---- Extract feature vector from current state ----
    feat = extractFeatures(robot, arm, obstacles, config, t);

    % ---- k-NN Classification ----
    [predictedLabel, confidence] = knnClassify(feat, refDataset, 5);

    % ---- Map predicted label to action string ----
    actionLabels = {'CONTINUE_MOVING','APPROACH_SLOW','ARM_PICK',...
                    'PLOW_PUSH','EMERGENCY_STOP','RETURN_HOME'};
    if predictedLabel >= 0 && predictedLabel <= 5
        predictedAction = actionLabels{predictedLabel + 1};
    else
        predictedAction = 'UNKNOWN';
    end

    % ---- Map robot's actual state to action label ----
    actualAction = robotStateToAction(robot.state);

    % ---- Check agreement ----
    actionMatch = strcmp(predictedAction, actualAction);

    % ---- Gradient Descent Optimizer for PD gains ----
    [optimKp, optimKd, trackingError, optimLoss] = ...
        gradientDescentOptimizer(robot, optimKp, optimKd, config, t);

    % ---- Package results ----
    mlResult.predictedAction  = predictedAction;
    mlResult.confidence       = confidence;
    mlResult.actualAction     = actualAction;
    mlResult.actionMatch      = actionMatch;
    mlResult.featureVector    = feat;
    mlResult.optimizedKp      = optimKp;
    mlResult.optimizedKd      = optimKd;
    mlResult.trackingError    = trackingError;
    mlResult.optimizerLoss    = optimLoss;
    mlResult.analysisTime     = t;

    prevAnalysisT = t;
end

% =========================================================================
% FEATURE EXTRACTION
% =========================================================================
function feat = extractFeatures(robot, arm, obstacles, config, t)
% Returns a 10-element normalized feature vector for the current state.

    % Obstacle distance and mass
    nearestDist = 999;
    nearestMass = 0;
    obsTypeEnc  = 0;   % encoded: mud=1, small_rock=2, rock=3, big_rock=4, small_log=5, log=6, stone=7
    for i = 1:numel(obstacles)
        if ~obstacles(i).removed
            d = norm([obstacles(i).position(1)-robot.x, ...
                      obstacles(i).position(2)-robot.y]);
            if d < nearestDist
                nearestDist = d;
                nearestMass = obstacles(i).mass;
                if isfield(obstacles(i),'obstacleType')
                    obsTypeEnc = obstacleTypeEncode(obstacles(i).obstacleType);
                end
            end
        end
    end

    % Arm joint angles (normalized to [-1,1] range)
    q1n = arm.angles(1) / 180;
    q2n = arm.angles(2) / 85;
    q3n = arm.angles(3) / 125;
    q4n = arm.angles(4) / 85;

    % Tip error norm (from last arm direction controller call, if available)
    tipErrNorm = 0;
    if isfield(robot,'lastTipErrorNorm')
        tipErrNorm = min(robot.lastTipErrorNorm / 2.0, 1.0);  % normalize to 2m max
    end

    % Robot speed (normalized)
    speedNorm = robot.speed / max(config.robotSpeed, 0.01);

    % Train distance (normalized, 0=close, 1=far/no train)
    trainDistNorm = 1.0;  % default: no train
    if isfield(robot,'lastTrainDist')
        trainDistNorm = min(robot.lastTrainDist / 100.0, 1.0);
    end

    % Normalize distance and mass
    distNorm = min(nearestDist / config.scannerRange, 1.0);
    massNorm = min(nearestMass / 200.0, 1.0);
    typeNorm = obsTypeEnc / 7.0;

    feat = [distNorm, massNorm, typeNorm, q1n, q2n, q3n, q4n, ...
            tipErrNorm, speedNorm, trainDistNorm];
end

% =========================================================================
% OBSTACLE TYPE ENCODER
% =========================================================================
function enc = obstacleTypeEncode(typeStr)
    switch typeStr
        case 'mud';        enc = 1;
        case 'small_rock'; enc = 2;
        case 'rock';       enc = 3;
        case 'big_rock';   enc = 4;
        case 'small_log';  enc = 5;
        case 'log';        enc = 6;
        case 'stone';      enc = 7;
        otherwise;         enc = 0;
    end
end

% =========================================================================
% ROBOT STATE -> ACTION LABEL MAPPING
% =========================================================================
function action = robotStateToAction(state)
    switch state
        case {'MOVING','RESUMING'};                    action = 'CONTINUE_MOVING';
        case {'OBSTACLE_DETECTED','CLASSIFYING','APPROACHING'}; action = 'APPROACH_SLOW';
        case {'STOPPED','REMOVING','CLEARANCE_CHECK'}; action = 'ARM_PICK';
        case 'PUSHING';                                action = 'PLOW_PUSH';
        case {'WARNING','WAITING','EMERGENCY_STOP'};   action = 'EMERGENCY_STOP';
        case {'RETURNING_HOME','COMPLETED'};           action = 'RETURN_HOME';
        otherwise;                                     action = 'UNKNOWN';
    end
end

% =========================================================================
% k-NEAREST NEIGHBORS CLASSIFIER
% =========================================================================
function [label, confidence] = knnClassify(feat, refDataset, k)
% Simple Euclidean k-NN over the reference dataset.
%   Returns the majority label and a confidence score [0,1].

    n = size(refDataset.X, 1);
    dists = zeros(n, 1);
    for i = 1:n
        dists(i) = norm(feat - refDataset.X(i,:));
    end
    [sortedDists, idx] = sort(dists);
    kIdx    = idx(1:k);
    kDists  = sortedDists(1:k);
    kLabels = refDataset.Y(kIdx);

    % Majority vote
    label = mode(kLabels);

    % Confidence: fraction of k neighbors with winning label,
    % weighted inversely by distance
    weights = 1 ./ max(kDists, 1e-6);
    winMask = (kLabels == label);
    confidence = sum(weights(winMask)) / sum(weights);
    confidence = min(max(confidence, 0), 1);
end

% =========================================================================
% REFERENCE DATASET (labeled training samples)
% =========================================================================
function ds = buildReferenceDataset()
% Builds a small hand-crafted reference dataset of 10-feature vectors
% covering the six action classes. In a production system this would be
% loaded from a CSV of recorded simulation runs.
%
% Feature order:
%   [distNorm, massNorm, typeNorm, q1n, q2n, q3n, q4n, tipErr, speed, trainDist]
%
% Label: 0=CONTINUE_MOVING, 1=APPROACH_SLOW, 2=ARM_PICK,
%        3=PLOW_PUSH, 4=EMERGENCY_STOP, 5=RETURN_HOME

    X = [
        % CONTINUE_MOVING (0): obstacle far/none, high speed, no train threat
        1.00, 0.00, 0.00,  0.00, 0.00, 0.00, 0.00,  0.00, 1.00, 1.00;
        0.95, 0.10, 0.20,  0.00, 0.00, 0.00, 0.00,  0.00, 0.95, 1.00;
        0.90, 0.00, 0.00,  0.00, 0.00, 0.00, 0.00,  0.00, 1.00, 0.90;

        % APPROACH_SLOW (1): obstacle in mid range, slowing down
        0.55, 0.15, 0.30,  0.00, 0.00, 0.00, 0.00,  0.05, 0.50, 1.00;
        0.40, 0.20, 0.43,  0.00, 0.00, 0.00, 0.00,  0.10, 0.30, 1.00;
        0.30, 0.12, 0.29,  0.00, 0.00, 0.00, 0.00,  0.08, 0.20, 0.90;

        % ARM_PICK (2): obstacle close, arm actively moving to it
        0.05, 0.15, 0.43,  0.10, 0.50, 0.30, 0.60,  0.30, 0.00, 1.00;
        0.04, 0.20, 0.57,  0.20, 0.55, 0.35, 0.65,  0.20, 0.00, 1.00;
        0.05, 0.10, 0.29,  0.05, 0.45, 0.28, 0.55,  0.15, 0.00, 0.85;

        % PLOW_PUSH (3): heavy obstacle (high mass), obstacle close, low speed
        0.05, 0.70, 0.57,  0.00, 0.00, 0.00, 0.00,  0.00, 0.15, 1.00;
        0.04, 0.85, 0.86,  0.00, 0.00, 0.00, 0.00,  0.00, 0.20, 1.00;
        0.05, 0.65, 1.00,  0.00, 0.00, 0.00, 0.00,  0.00, 0.10, 0.90;

        % EMERGENCY_STOP (4): train very close, robot stopped
        0.50, 0.30, 0.43,  0.00, 0.00, 0.00, 0.00,  0.00, 0.00, 0.05;
        0.60, 0.20, 0.29,  0.00, 0.00, 0.00, 0.00,  0.00, 0.00, 0.08;
        0.30, 0.40, 0.57,  0.00, 0.00, 0.00, 0.00,  0.00, 0.00, 0.03;

        % RETURN_HOME (5): all obstacles cleared, robot at high speed returning
        0.98, 0.00, 0.00,  0.00, 0.00, 0.00, 0.00,  0.00, 0.80, 1.00;
        1.00, 0.00, 0.00,  0.00, 0.00, 0.00, 0.00,  0.00, 0.90, 1.00;
        0.95, 0.00, 0.00,  0.00, 0.00, 0.00, 0.00,  0.00, 0.85, 0.95;
    ];

    Y = [0;0;0;  1;1;1;  2;2;2;  3;3;3;  4;4;4;  5;5;5];

    ds.X = X;
    ds.Y = Y;
end

% =========================================================================
% GRADIENT DESCENT OPTIMIZER FOR PD GAINS
% =========================================================================
function [Kp_new, Kd_new, trackErr, loss] = gradientDescentOptimizer(...
        robot, Kp, Kd, config, t)
% Performs one gradient descent step to minimize arm tracking error.
%
% Loss function:
%   L = (1/N)*sum||q_desired - q_actual||^2 + lambda*(||Kp||^2 + ||Kd||^2)
%
% Gradients (analytical):
%   dL/dKp_j = -2*(1/N)*sum(e_j) + 2*lambda*Kp_j
%   dL/dKd_j = -2*(1/N)*sum(de_j) + 2*lambda*Kd_j

    lr     = 0.005;   % learning rate (small: conservative step)
    lambda = 1e-4;    % L2 regularization coefficient

    % ---- Pull tracking data from armLog ----
    if ~isfield(robot,'armLog') || isempty(robot.armLog) || ...
       size(robot.armLog.q,1) < 3
        % Not enough history yet
        Kp_new   = Kp;
        Kd_new   = Kd;
        trackErr = 0;
        loss     = 0;
        return;
    end

    N  = size(robot.armLog.q, 1);
    % Use last 60 samples (3 seconds at dt=0.05)
    window = max(1, N-59):N;
    q_act  = robot.armLog.q(window, :);    % actual angles [deg]
    q_des  = robot.armLog.qd(window, :);   % desired angles [deg]
    dq_act = robot.armLog.qdot(window, :); % actual velocities [deg/s]

    e     = q_des - q_act;         % N_w x 4 error matrix
    de    = -dq_act;               % N_w x 4 velocity error (desired vel=0 assumption)

    Nw = size(e,1);
    mean_e  = mean(e, 1)';    % 4x1
    mean_de = mean(de, 1)';   % 4x1

    % Tracking error (root mean square across joints and time)
    trackErr = sqrt(mean(e(:).^2));

    % Loss
    loss = mean(e(:).^2) + lambda*(sum(Kp.^2) + sum(Kd.^2));

    % Gradients
    grad_Kp = -2*mean_e  + 2*lambda*Kp;
    grad_Kd = -2*mean_de + 2*lambda*Kd;

    % Gradient step
    Kp_new = Kp - lr * grad_Kp;
    Kd_new = Kd - lr * grad_Kd;

    % Clamp to physically meaningful PD gain ranges
    Kp_new = max(min(Kp_new, [30; 120; 90; 60]), [1; 10; 8; 5]);
    Kd_new = max(min(Kd_new, [ 8;  30; 22; 15]), [0.5; 2; 1.5; 1]);
end
