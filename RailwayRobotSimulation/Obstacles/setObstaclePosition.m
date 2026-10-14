function obstacle = setObstaclePosition(obstacle, newPos)
% SETOBSTACLEPOSITION - Moves the obstacle's graphics to newPos = [x y z]
% by re-adding its stored centered shape (shapeX/Y/Z) at the new center.
% Completely scalar-safe for all graphics handles.

    obstacle.position = newPos;

    if isfield(obstacle,'handle') && ~isempty(obstacle.handle) && all(isgraphics(obstacle.handle))
        set(obstacle.handle, ...
            'XData', obstacle.shapeX + newPos(1), ...
            'YData', obstacle.shapeY + newPos(2), ...
            'ZData', obstacle.shapeZ + newPos(3));
    end

    if isfield(obstacle,'shadow') && ~isempty(obstacle.shadow) && all(isgraphics(obstacle.shadow))
        % Shadow always stays pinned to the ground beneath the obstacle
        sx = get(obstacle.shadow, 'XData') - mean(get(obstacle.shadow,'XData')) + newPos(1);
        sy = get(obstacle.shadow, 'YData') - mean(get(obstacle.shadow,'YData')) + newPos(2);
        set(obstacle.shadow, 'XData', sx, 'YData', sy);
    end

    if isfield(obstacle,'sandHandle') && ~isempty(obstacle.sandHandle) && all(isgraphics(obstacle.sandHandle)) && isfield(obstacle,'sandRelX') && ~isempty(obstacle.sandRelX)
        set(obstacle.sandHandle, ...
            'XData', obstacle.sandRelX + newPos(1), ...
            'YData', obstacle.sandRelY + newPos(2), ...
            'ZData', obstacle.sandRelZ + newPos(3));
    end

    if isfield(obstacle,'label') && ~isempty(obstacle.label) && all(isgraphics(obstacle.label))
        set(obstacle.label, 'Position', [newPos(1), newPos(2), newPos(3)+obstacle.size*0.75+0.3]);
    end
end
