function obstacles = spawnObstacles(ax, config, scenario)
% SPAWNOBSTACLES - Creates obstacle arrays for each scenario.
%
% RISK-BASED OBSTACLE DESIGN:
%   Each obstacle has a riskLevel (LOW / MEDIUM / HIGH) computed at spawn.
%   The robot evaluates risk before deciding to act:
%     LOW    -> Robot detects it, logs it, but SKIPS and keeps moving
%     MEDIUM -> Robot slows, removes the obstacle
%     HIGH   -> Robot treats as top priority, removes urgently
%
% Obstacle types and their typical risk:
%   mud        -> MEDIUM (track adhesion loss / slip - robot clears with leveler)
%   small_rock -> LOW    (small enough to deflect)
%   rock       -> MEDIUM (can jam under wheels)
%   big_rock   -> HIGH   (certain derailment)
%   small_log  -> MEDIUM (can get under bogies)
%   log        -> HIGH   (major derailment hazard)
%   stone      -> MEDIUM/HIGH (wedges under bogie frame)

    switch scenario

        case "none"
            % Clean track - one dummy obstacle off track, immediately removed
            obstacles = createObstacle(ax, 1, [999 config.track1Y 0], 1, config, 'mud');
            obstacles.removed = true;

        case "single_removable"
            % One medium rock (MEDIUM risk) - robot removes it
            obstacles = createObstacle(ax, 1, [42 config.track1Y 0], 20, config, 'rock');

        case "single_heavy"
            % One large boulder (HIGH risk) - robot plows it
            obstacles = createObstacle(ax, 1, [50 config.track1Y 0], 130, config, 'big_rock');

        case "multiple"
            % === MIXED RISK SCENARIO ===
            % Demonstrates risk levels and BOTH removal methods on the same track:
            %  (removableMassThreshold = 70 kg)
            %
            %  #1 mud        x=20  mass=3   -> MEDIUM risk -> robot CLEARS with Front Leveler
            %  #2 small_rock x=40  mass=8   -> LOW risk    -> robot SKIPS
            %  #3 rock       x=70  mass=22  -> MEDIUM      -> robot REMOVES (ARM pick into pouch)
            %  #4 big_rock   x=105 mass=90  -> HIGH        -> robot REMOVES (PLOW push)
            %  #5 small_log  x=140 mass=55  -> MEDIUM      -> robot REMOVES (ARM pick into pouch)
            %  #6 log        x=175 mass=120 -> HIGH        -> robot REMOVES (PLOW push)
            %  #7 stone      x=205 mass=65  -> MEDIUM      -> robot REMOVES (ARM pick into pouch)
            %
            positions = [20,   40,   70,   105,       140,         175,   205];
            masses    = [3,    8,    22,   90,         55,          120,   65];
            types     = {'mud','small_rock','rock','big_rock','small_log','log','stone'};
            n = min(config.numberOfObstacles, numel(positions));
            obstacles = createObstacle(ax, 1, [positions(1) config.track1Y 0], ...
                            masses(1), config, types{1});
            for i = 2:n
                obstacles = [obstacles, ...
                    createObstacle(ax, i, [positions(i) config.track1Y 0], ...
                        masses(i), config, types{i})]; %#ok<AGROW>
            end

        case "heavy_plus_train"
            % LOW risk small_rock at start (robot skips it)
            % Then HEAVY log (HIGH risk) that triggers train emergency
            % Then medium rock + another small_rock (LOW, skipped)
            o1 = createObstacle(ax, 1, [25  config.track1Y 0],   7, config, 'small_rock');  % LOW  -> skip
            o2 = createObstacle(ax, 2, [65  config.track1Y 0], 140, config, 'log');          % HIGH -> plow
            o3 = createObstacle(ax, 3, [95  config.track1Y 0],  28, config, 'rock');         % MED  -> pick
            o4 = createObstacle(ax, 4, [130 config.track1Y 0],   4, config, 'mud');          % LOW  -> skip
            obstacles = [o1, o2, o3, o4];

        case "night"
            % Night patrol: small mud patch on track (LOW risk - robot logs but skips)
            % followed by a rock further ahead (MEDIUM - robot removes)
            o1 = createObstacle(ax, 1, [30 config.track1Y 0],  3, config, 'mud');        % LOW
            o2 = createObstacle(ax, 2, [55 config.track1Y 0], 22, config, 'rock');       % MEDIUM
            obstacles = [o1, o2];

        otherwise
            obstacles = createObstacle(ax, 1, [42 config.track1Y 0], 20, config, 'rock');
    end
end
