function railway = createRailway(ax, config)
% CREATERAILWAY - Draws TWO parallel tracks:
%   Track 1 (patrol track) at Y = config.track1Y  (robot works here)
%   Track 2 (home track)   at Y = config.track2Y  (robot returns here)
% Each track has sleepers + two steel rails.

    x0 = -10;
    x1 = config.trackLength + 10;
    halfGauge = config.railGauge / 2;

    % ---- Track 1 (patrol) - brown sleepers, grey rails ----
    railway.t1 = buildTrack(ax, x0, x1, config.track1Y, halfGauge, ...
        [0.35 0.22 0.12], [0.62 0.62 0.68], config.sleeperSpacing);

    % ---- Track 2 (home) - slightly lighter sleepers ----
    railway.t2 = buildTrack(ax, x0, x1, config.track2Y, halfGauge, ...
        [0.42 0.28 0.15], [0.58 0.58 0.64], config.sleeperSpacing);

    % ---- Ballast between and around both tracks ----
    % Wide grey gravel bed
    bx = [x0 x0 x1 x1];
    by = [-halfGauge-0.8  config.track2Y+halfGauge+0.8 ...
           config.track2Y+halfGauge+0.8  -halfGauge-0.8];
    bz = [0.03 0.03 0.03 0.03];
    patch(ax, bx, by, bz, [0.52 0.52 0.52], 'EdgeColor','none','FaceAlpha',0.7);

    % ---- Home-track side marker: yellow stripe on the platform edge ----
    stripY = config.track2Y + halfGauge + 0.55;
    patch(ax, [x0 x0 x1 x1], [stripY stripY+0.12 stripY+0.12 stripY], ...
        [0.06 0.06 0.06 0.06], [0.98 0.85 0.05], 'EdgeColor','none');

    % "HOME TRACK" text at start
    text(ax, 5, config.track2Y + halfGauge + 0.9, 0.25, 'HOME TRACK', ...
        'Color',[1 0.9 0.1], 'FontSize',9, 'FontWeight','bold', ...
        'BackgroundColor',[0.08 0.08 0.10]);
    text(ax, 5, config.track1Y - halfGauge - 0.9, 0.25, 'PATROL TRACK', ...
        'Color',[0.3 0.9 1.0], 'FontSize',9, 'FontWeight','bold', ...
        'BackgroundColor',[0.08 0.08 0.10]);

    railway.trackLength = config.trackLength;
    railway.x0 = x0;
end

function t = buildTrack(ax, x0, x1, centreY, halfGauge, sleeperColor, railColor, spacing)
    % Sleepers
    sleeperXs = x0:spacing:x1;
    t.sleepers = gobjects(numel(sleeperXs),1);
    for i = 1:numel(sleeperXs)
        sx = sleeperXs(i);
        sw = 0.22;
        sy = centreY;
        vx = [sx-sw sx-sw sx+sw sx+sw];
        vy = [sy-halfGauge-0.45  sy+halfGauge+0.45 ...
              sy+halfGauge+0.45  sy-halfGauge-0.45];
        vz = [0.06 0.06 0.06 0.06];
        t.sleepers(i) = patch(ax, vx, vy, vz, sleeperColor, 'EdgeColor','none');
    end
    % Rails
    railH = 0.13;
    t.railL = drawRail(ax, x0, x1, centreY - halfGauge, railH, railColor);
    t.railR = drawRail(ax, x0, x1, centreY + halfGauge, railH, railColor);
end

function h = drawRail(ax, x0, x1, yPos, railH, color)
    w = 0.055;
    vx = [x0 x0 x1 x1];
    vy = [yPos-w yPos+w yPos+w yPos-w];
    vz = [railH railH railH railH];
    h = patch(ax, vx, vy, vz, color, 'EdgeColor',[0.18 0.18 0.18]);
end
