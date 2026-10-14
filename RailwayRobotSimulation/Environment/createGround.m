function ground = createGround(ax, config)
% CREATEGROUND - Full-coverage terrain that eliminates the blue-gap artifact.
% Covers x: -30 to trackLength+20, y: -8 to track2Y+12 completely.

    hold(ax,'on');
    x0 = -30;  x1 = config.trackLength + 20;
    y0 = -8;   y1 = config.track2Y + 12;

    n = 50;
    [X,Y] = meshgrid(linspace(x0,x1,n), linspace(y0,y1,n));
    Z = 0.001*ones(size(X));

    % Grass-green with subtle noise
    baseR = 0.30; baseG = 0.50; baseB = 0.24;
    noise = 0.05*(rand(n,n)-0.5);
    C = cat(3, baseR+noise, baseG+noise*1.2, baseB+noise*0.5);
    C = min(max(C,0),1);

    ground.surf = surf(ax, X, Y, Z, C, ...
        'FaceColor','interp','EdgeColor','none', ...
        'FaceLighting','gouraud','AmbientStrength',0.60,'SpecularStrength',0);

    % Also patch the area between tracks to look like ballast/gravel
    bx = [x0 x0 x1 x1];
    by_inner = [-(config.railGauge/2+0.5), config.track2Y-(config.railGauge/2+0.5), ...
                config.track2Y-(config.railGauge/2+0.5), -(config.railGauge/2+0.5)];
    patch(ax, bx, by_inner, [0.025 0.025 0.025 0.025], ...
        [0.48 0.46 0.42],'EdgeColor','none','FaceAlpha',0.5);

    ground.x0 = x0; ground.x1 = x1;
end
