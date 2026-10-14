function category = classifyObstacle(obstacle, config)
% CLASSIFYOBSTACLE - Rule-based classification (mass threshold).
%
% Kept deliberately simple and modular: this file can later be swapped
% for classifyObstacleML.m which would take the same kind of inputs
% (size, mass, shape, camera image, distance, position) and return the
% same REMOVABLE/HEAVY category string, without changing any caller.

    if obstacle.mass < config.removableMassThreshold
        category = "REMOVABLE";
    else
        category = "HEAVY";
    end
end
