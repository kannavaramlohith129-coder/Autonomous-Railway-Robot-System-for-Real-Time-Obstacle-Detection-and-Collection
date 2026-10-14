function dayNight = DayNightController(ax, dayNight, mode, dt)
% DAYNIGHTCONTROLLER - Switches / auto-cycles between DAY and NIGHT.
% dayNight is a struct that persists across calls:
%   dayNight.mode        "DAY" or "NIGHT"
%   dayNight.lightHandle handle to the scene light
%   dayNight.autoCycle   true/false
%   dayNight.cyclePeriod seconds per half cycle
%   dayNight.timer       internal accumulator (only used if autoCycle)
%
% Call with mode = [] to just tick the auto-cycle timer, or pass a mode
% string ("DAY"/"NIGHT") to force a switch.

    if nargin >= 3 && ~isempty(mode)
        dayNight.mode = mode;
        dayNight.timer = 0;
    elseif isfield(dayNight,'autoCycle') && dayNight.autoCycle
        dayNight.timer = dayNight.timer + dt;
        if dayNight.timer >= dayNight.cyclePeriod
            dayNight.timer = 0;
            if dayNight.mode == "DAY"
                dayNight.mode = "NIGHT";
            else
                dayNight.mode = "DAY";
            end
        end
    end

    if dayNight.mode == "DAY"
        set(ax, 'Color', [0.65 0.82 0.95]); % sky-blue background
        if isgraphics(dayNight.lightHandle)
            set(dayNight.lightHandle, 'Color', [1 1 0.95]);
        end
        ax.Parent.Color = [0.65 0.82 0.95];
    else
        set(ax, 'Color', [0.03 0.03 0.08]); % night sky
        if isgraphics(dayNight.lightHandle)
            set(dayNight.lightHandle, 'Color', [0.55 0.6 0.8]); % dim, cool moonlight
        end
        ax.Parent.Color = [0.03 0.03 0.08];
    end
end
