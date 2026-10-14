function ttc = calculateTimeToCollision(distance, speed)
% CALCULATETIMETOCOLLISION - Simple distance/speed time estimate.
% Returns Inf if speed is zero or negative (nothing closing in).
    if speed <= 0
        ttc = inf;
    else
        ttc = distance / speed;
    end
end
