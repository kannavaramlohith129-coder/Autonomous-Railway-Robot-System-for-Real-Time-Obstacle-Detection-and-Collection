function ds = buildReferenceDataset(saveToFile)
% BUILDREFERENCEDATASET - Builds and exports labeled reference dataset for ML k-NN classifier
%
% Syntax:
%   ds = buildReferenceDataset()
%   ds = buildReferenceDataset(saveToFile)  % Pass true to save to CSV & MAT
%
% Feature columns (10 features):
%   1. distNorm      - Normalized distance to nearest obstacle [0 to 1]
%   2. massNorm      - Normalized obstacle mass [0 to 1]
%   3. typeNorm      - Normalized obstacle type code [0 to 1]
%   4. q1n           - Arm joint 1 angle (normalized by 180 deg)
%   5. q2n           - Arm joint 2 angle (normalized by 85 deg)
%   6. q3n           - Arm joint 3 angle (normalized by 125 deg)
%   7. q4n           - Arm joint 4 angle (normalized by 85 deg)
%   8. tipErrNorm    - Normalized end-effector tracking error [0 to 1]
%   9. speedNorm     - Normalized robot patrol speed [0 to 1]
%  10. trainDistNorm - Normalized distance to incoming train [0 to 1]
%
% Target classes (Y):
%   0 = CONTINUE_MOVING
%   1 = APPROACH_SLOW
%   2 = ARM_PICK
%   3 = PLOW_PUSH
%   4 = EMERGENCY_STOP
%   5 = RETURN_HOME

    if nargin < 1
        saveToFile = false;
    end

    % 10-feature training matrices
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

    actionNames = {
        'CONTINUE_MOVING';
        'CONTINUE_MOVING';
        'CONTINUE_MOVING';
        'APPROACH_SLOW';
        'APPROACH_SLOW';
        'APPROACH_SLOW';
        'ARM_PICK';
        'ARM_PICK';
        'ARM_PICK';
        'PLOW_PUSH';
        'PLOW_PUSH';
        'PLOW_PUSH';
        'EMERGENCY_STOP';
        'EMERGENCY_STOP';
        'EMERGENCY_STOP';
        'RETURN_HOME';
        'RETURN_HOME';
        'RETURN_HOME'
    };

    featureNames = {
        'distNorm', 'massNorm', 'typeNorm', ...
        'q1n', 'q2n', 'q3n', 'q4n', ...
        'tipErrNorm', 'speedNorm', 'trainDistNorm'
    };

    ds.X = X;
    ds.Y = Y;
    ds.featureNames = featureNames;
    ds.actionNames  = actionNames;
    ds.classes = {
        '0: CONTINUE_MOVING', ...
        '1: APPROACH_SLOW', ...
        '2: ARM_PICK', ...
        '3: PLOW_PUSH', ...
        '4: EMERGENCY_STOP', ...
        '5: RETURN_HOME'
    };

    fprintf('[buildReferenceDataset] Successfully built reference dataset: %d samples, %d features.\n', ...
        size(X, 1), size(X, 2));

    if saveToFile
        robotDir = fileparts(mfilename('fullpath'));
        dataDir  = fullfile(robotDir, '..', 'Data');
        if ~exist(dataDir, 'dir')
            mkdir(dataDir);
        end

        matPath = fullfile(dataDir, 'ml_reference_dataset.mat');
        save(matPath, 'ds');
        fprintf('[buildReferenceDataset] Saved MAT file: %s\n', matPath);

        csvPath = fullfile(dataDir, 'ml_reference_dataset.csv');
        headers = [featureNames, {'TargetClass', 'ActionName'}];
        T = cell2table([num2cell([X, Y]), actionNames], 'VariableNames', headers);
        writetable(T, csvPath);
        fprintf('[buildReferenceDataset] Saved CSV file: %s\n', csvPath);
    end
end
