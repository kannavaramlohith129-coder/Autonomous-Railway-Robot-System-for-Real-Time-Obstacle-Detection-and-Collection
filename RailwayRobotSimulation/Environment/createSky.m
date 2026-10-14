function sky = createSky(ax, config)
% CREATESKY - Adds distant rolling hills beyond the fence line and a few
% static clouds high overhead, purely for visual depth so the scene
% doesn't look empty at the horizon.

    hold(ax, 'on');
    rng(7);

    %% ---- Distant hills (flattened blobs along both horizons) ----
    hillXs = -20:25:(config.trackLength+20);
    sky.hillsFar  = gobjects(numel(hillXs),1);
    sky.hillsNear = gobjects(numel(hillXs),1);
    for i = 1:numel(hillXs)
        h1 = 3 + rand()*2.5;
        sky.hillsFar(i)  = drawHill(ax, hillXs(i)+rand()*8, 34+rand()*4, 10+rand()*4, h1, ...
            [0.30 0.42 0.26] + 0.03*(rand(1,3)-0.5));
        h2 = 2.5 + rand()*2;
        sky.hillsNear(i) = drawHill(ax, hillXs(i)+rand()*8, -(34+rand()*4), 10+rand()*4, h2, ...
            [0.33 0.45 0.28] + 0.03*(rand(1,3)-0.5));
    end

    %% ---- A few static clouds ----
    cloudXs = linspace(0, config.trackLength, 6) + (rand(1,6)-0.5)*15;
    sky.clouds = gobjects(numel(cloudXs),1);
    for i = 1:numel(cloudXs)
        sky.clouds(i) = drawCloud(ax, cloudXs(i), (rand()-0.5)*20, 9+rand()*2.5);
    end
end

function h = drawHill(ax, x, y, radius, height, color)
    [ux, uy, uz] = sphere(14);
    uz(uz<0) = 0; % keep only the upper half so it sits on the ground
    h = surf(ax, ux*radius+x, uy*radius+y, uz*height, ...
        'FaceColor', color, 'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', 'AmbientStrength', 0.6, 'SpecularStrength', 0);
end

function h = drawCloud(ax, x, y, z)
    h = hggroup(ax);
    puffs = [0 0 0; 0.9 0.1 0.05; -0.8 -0.05 0.05; 0.3 0.4 0.15; -0.4 0.35 0.1];
    for k = 1:size(puffs,1)
        p = puffs(k,:);
        [sx, sy, sz] = sphere(8);
        surface('Parent', h, 'XData', sx*0.8+x+p(1)*1.3, 'YData', sy*0.6+y+p(2)*1.3, ...
            'ZData', sz*0.5+z+p(3), 'FaceColor', [1 1 1], 'FaceAlpha', 0.85, 'EdgeColor', 'none');
    end
end
