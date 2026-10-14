function envObjects = createRailwayObjects(ax, config)
% CREATERAILWAYOBJECTS - Adds trees, signal poles and fences beside the
% track so the scene doesn't look empty. Purely decorative.

    hold(ax, 'on');
    envObjects.trees = gobjects(config.numTrees,1);
    envObjects.poles = gobjects(config.numPoles,1);

    rng(1); % fixed seed so the layout is reproducible between runs

    % ---- Trees (layered canopy + trunk + ground shadow), both sides ----
    for i = 1:config.numTrees
        side = (mod(i,2)*2 - 1); % alternate left/right
        x = rand()*config.trackLength;
        y = side*(3 + rand()*6);
        scale = 0.8 + rand()*0.5;
        envObjects.trees(i) = drawTree(ax, x, y, scale);
    end

    % ---- Signal / lamp poles, evenly spaced beside the track ----
    poleXs = linspace(5, config.trackLength-5, config.numPoles);
    for i = 1:config.numPoles
        side = (mod(i,2)*2 - 1);
        y = side*1.9;
        envObjects.poles(i) = drawPole(ax, poleXs(i), y);
    end

    % ---- Simple fence lines on both sides ----
    fenceY = 4.5;
    envObjects.fenceLeft  = line(ax, [0 config.trackLength], [fenceY fenceY], [0 0], ...
        'Color', [0.5 0.4 0.3], 'LineWidth', 2);
    envObjects.fenceRight = line(ax, [0 config.trackLength], [-fenceY -fenceY], [0 0], ...
        'Color', [0.5 0.4 0.3], 'LineWidth', 2);
end

function h = drawTree(ax, x, y, scale)
    h = hggroup(ax); % groups trunk+canopy+shadow into a single graphics handle
    trunkH = 1.1*scale;

    % ---- Ground contact shadow (helps the tree read as "planted") ----
    [shx, shy] = pol2cart(linspace(0,2*pi,14), 0.55*scale);
    patch('Parent', h, 'XData', shx+x, 'YData', shy+y, 'ZData', zeros(1,14)+0.01, ...
        'FaceColor', [0 0 0], 'FaceAlpha', 0.15, 'EdgeColor', 'none');

    % ---- Trunk ----
    [cx, cy, cz] = cylinder(0.08*scale, 8);
    cz = cz*trunkH;
    surface('Parent', h, 'XData', cx+x, 'YData', cy+y, 'ZData', cz, ...
        'FaceColor', [0.38 0.24 0.12], 'EdgeColor', 'none', 'FaceLighting', 'gouraud');

    % ---- Layered canopy: three overlapping cone tiers for fullness ----
    tiers = [1.05 0.55; 0.80 0.45; 0.55 0.4]; % [radius height] per tier, base->top
    greenShades = [0.12 0.42 0.16; 0.15 0.48 0.19; 0.18 0.53 0.20];
    zBase = trunkH * 0.55;
    for k = 1:size(tiers,1)
        r = tiers(k,1)*scale;
        ht = tiers(k,2)*scale;
        [conex, coney, conez] = cylinder([r 0], 10);
        conez = conez*ht + zBase;
        surface('Parent', h, 'XData', conex+x, 'YData', coney+y, 'ZData', conez, ...
            'FaceColor', greenShades(k,:), 'EdgeColor', 'none', 'FaceLighting', 'gouraud', ...
            'AmbientStrength', 0.55);
        zBase = zBase + ht*0.55;
    end
end

function h = drawPole(ax, x, y)
    poleH = 3.2;
    h = line(ax, [x x], [y y], [0 poleH], 'Color', [0.3 0.3 0.3], 'LineWidth', 3);
end
