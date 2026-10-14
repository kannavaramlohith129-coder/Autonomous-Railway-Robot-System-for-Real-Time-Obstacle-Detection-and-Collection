function [riskLevel, riskScore, riskReason] = assessObstacleRisk(obstacle, config)
% ASSESSOBSTACLERISK - Calculates how much risk an obstacle poses to a
%   passing train and classifies it as LOW / MEDIUM / HIGH risk.
%
% =========================================================================
% RISK MODEL
% =========================================================================
% Risk is scored on a 0-100 scale combining four factors:
%
%  Factor 1 - OBSTACLE TYPE (base risk from physical nature):
%    mud        ->  5   (soft, train tyres squash it harmlessly)
%    small_rock -> 15   (small stones get deflected by cowcatcher)
%    rock       -> 45   (medium rock can jam under wheels)
%    big_rock   -> 80   (large boulder - definite derailment risk)
%    small_log  -> 35   (branch can get under bogies)
%    log        -> 75   (full log - major derailment hazard)
%    stone      -> 60   (flat slab - wedges under bogie frame)
%
%  Factor 2 - MASS modifier (+0 to +15):
%    Heavy objects have more momentum -> harder to deflect
%    modifier = 15 * min(mass / 200, 1)
%
%  Factor 3 - TRACK CORRIDOR POSITION modifier (+0 to +10):
%    Dead-centre on rail is maximum risk; edge of corridor is lower
%    lateralOffset = |obs.y - track_centre| / corridorWidth
%    modifier = 10 * (1 - lateralOffset)
%
%  Factor 4 - SIZE modifier (+0 to +10):
%    Larger objects block more of the rail head profile
%    modifier = 10 * min(obstacle.size / 0.65, 1)
%
%  Total score = TypeBase + MassMod + PosMod + SizeMod   (capped at 100)
%
%  Risk classification:
%    LOW    : score  0–30  -> Robot skips, keeps moving
%    MEDIUM : score 31–60  -> Robot removes obstacle
%    HIGH   : score 61–100 -> Robot removes urgently (top priority)
%
% =========================================================================

    % ---- Factor 1: Type base score ----
    if isfield(obstacle, 'obstacleType')
        obsType = obstacle.obstacleType;
    else
        obsType = 'rock';   % default if type missing
    end

    switch obsType
        case 'mud';        typeBase = 38;   % MEDIUM risk: track adhesion loss & soil deposit requires leveler clearance
        case 'small_rock'; typeBase = 15;   % LOW risk: small stone deflected
        case 'rock';       typeBase = 45;   % MEDIUM risk: can jam under wheel
        case 'big_rock';   typeBase = 80;   % HIGH risk: severe derailment danger
        case 'small_log';  typeBase = 35;   % MEDIUM risk: branch under bogie
        case 'log';        typeBase = 75;   % HIGH risk: heavy tree trunk
        case 'stone';      typeBase = 60;   % MEDIUM/HIGH risk: slab under frame
        otherwise;         typeBase = 40;
    end

    % ---- Factor 2: Mass modifier ----
    massMod = 15 * min(obstacle.mass / 200.0, 1.0);

    % ---- Factor 3: Lateral position modifier ----
    % Centre of track = max risk, edge = min risk
    lateralOff = abs(obstacle.position(2) - config.track1Y) / max(config.corridorWidth, 0.1);
    lateralOff = min(lateralOff, 1.0);
    posMod = 10 * (1 - lateralOff);

    % ---- Factor 4: Size modifier ----
    sizeMod = 10 * min(obstacle.size / 0.65, 1.0);

    % ---- Total risk score ----
    riskScore = min(typeBase + massMod + posMod + sizeMod, 100);

    % ---- Classify risk level ----
    if riskScore <= 30
        riskLevel = "LOW";
    elseif riskScore <= 60
        riskLevel = "MEDIUM";
    else
        riskLevel = "HIGH";
    end

    % ---- Human-readable reason ----
    switch riskLevel
        case "LOW"
            riskReason = sprintf('%s (%.0f/100): Train can safely pass - no action needed', ...
                obsType, riskScore);
        case "MEDIUM"
            riskReason = sprintf('%s (%.0f/100): Moderate derailment risk - robot will remove', ...
                obsType, riskScore);
        case "HIGH"
            riskReason = sprintf('%s (%.0f/100): HIGH derailment hazard - urgent removal', ...
                obsType, riskScore);
    end
end
