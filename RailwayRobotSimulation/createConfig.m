function config = createConfig()
% CREATECONFIG - All simulation parameters in one place.

    % ---- Track / environment ----
    config.trackLength      = 220;
    config.railGauge        = 1.435;   % standard gauge (m)
    config.sleeperSpacing   = 2.0;
    config.numTrees         = 16;
    config.numPoles         = 12;

    % ---- TWO-TRACK layout ----
    % Track 1 (robot patrol track) centred at y = 0
    % Track 2 (home / safe return track) centred at y = +6  
    config.track1Y          = 0;       % patrol track Y centre
    config.track2Y          = 6.0;     % home track Y centre (robot returns here after mission)
    config.trackSeparation  = 6.0;     % distance between track centrelines

    % ---- Robot ----
    config.robotSpeed        = 2.5;
    config.robotAcceleration = 1.8;
    config.robotStartX       = 0;
    config.robotHomeTrackY   = 6.0;    % Y position of the home/safe track

    % ---- Scanner / detection ----
    config.scannerRange     = 32;
    config.scannerFOV       = 28;
    config.corridorWidth    = 1.4;

    % ---- Obstacles ----
    config.numberOfObstacles      = 7;   % pouch now fits removable obstacles + full suite of 7 types
    config.removableMassThreshold = 70;  % kg - below = REMOVABLE (arm pick), above = HEAVY (plow push)

    % ---- Manipulator arm ----
    config.armSpeed    = 45;   % deg/s - slow enough to watch clearly
    config.armHoldTime = 0.55; % seconds pause at each waypoint

    % ---- Train ----
    config.trainSpeed         = 15;
    config.trainStartDistance = 140;
    config.trainWarnDistance  = 65;
    config.trainDangerTime    = 8;
    config.trainMinGap        = 5;

    % ---- Day / Night ----
    config.dayNightMode = "DAY";
    config.autoCycle    = false;
    config.cyclePeriod  = 40;

    % ---- Simulation ----
    config.dt              = 0.05;
    config.simulationSpeed = 2.0;    % 2x speed for snappier playback
    config.maxSimTime      = 700;

    % ---- Track Switch ----
    config.switchSpeed       = 0.8;    % robot speed through track switch [m/s]
    config.switchCurveLength = 12.0;   % Bezier curve length for switch transition [m]

    % ---- Dynamic Arm Controller Gains ----
    % PD + Gravity Compensation: tau = Kp*e + Kd*de + G(q)
    config.armKp = [10.0, 55.0, 40.0, 25.0];  % position gains [Nm/rad]
    config.armKd = [ 2.5, 14.0, 10.0,  6.0];  % velocity gains [Nm.s/rad]

    % ---- Logging ----
    config.logFile = fullfile(fileparts(mfilename('fullpath')), 'Data', 'simulationLog.csv');
end

