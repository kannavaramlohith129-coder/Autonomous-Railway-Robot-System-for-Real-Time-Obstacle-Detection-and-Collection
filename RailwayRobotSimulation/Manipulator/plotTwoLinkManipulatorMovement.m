function handles = plotTwoLinkManipulatorMovement(varargin)
% PLOTTWOLINKMANIPULATORMOVEMENT - Movement & Dynamics Analysis for Two-Link Manipulator
%
% Generates the complete 6-figure suite matching MATLAB robotics standards:
%   - Figure 2: Joint 1 Error  (with pick timestamp event marker)
%   - Figure 3: Joint 2 Error  (with pick timestamp event marker)
%   - Figure 4: Tracking Errors (e1, e2, and ||e|| with phase regions)
%   - Figure 5: Control Torques (tau_1, tau_2 showing dynamic load jump at pick)
%   - Figure 6: Phase Plane    (q vs qdot orbital phase portraits)
%   - Figure 7: Two-Link Robot Animation (interactive 2-link motion canvas)
%
% USAGE:
%   plotTwoLinkManipulatorMovement()              % Standalone demonstration
%   plotTwoLinkManipulatorMovement(robot)         % From 3D simulation robot struct
%   plotTwoLinkManipulatorMovement(armHistory, pickEvent)
%   plotTwoLinkManipulatorMovement(..., 'Dock', true)  % Docked as tabs (default)
%   plotTwoLinkManipulatorMovement(..., 'Dock', false) % Separate windows
%   plotTwoLinkManipulatorMovement(..., 'Animate', true)
%
% Outputs:
%   handles - Struct with figure and graphics handles

    fprintf('\n========================================================================\n');
    fprintf('     TWO-LINK MANIPULATOR MOVEMENT & DYNAMICS ANALYSIS SYSTEM\n');
    fprintf('========================================================================\n');

    %% 1. PARSE ARGUMENTS & CONSTRUCT OR EXTRACT SIMULATION DATA
    p = inputParser;
    p.KeepUnmatched = true;
    addOptional(p, 'dataInput', []);
    addOptional(p, 'eventInput', []);
    addParameter(p, 'Dock', true, @islogical);
    addParameter(p, 'Animate', true, @islogical);
    addParameter(p, 'ObstacleMass', 8.0, @isnumeric);
    parse(p, varargin{:});

    dataInput   = p.Results.dataInput;
    eventInput  = p.Results.eventInput;
    dockFigures = p.Results.Dock;
    doAnimate   = p.Results.Animate;

    if isempty(dataInput)
        % Standalone High-Fidelity Simulation
        [simData, simEvents] = generateBenchmarkSimulation(p.Results.ObstacleMass);
    elseif isstruct(dataInput) && isfield(dataInput, 'armHistory')
        % Passed robot struct
        [simData, simEvents] = extractFromRobot(dataInput);
    elseif isstruct(dataInput) && isfield(dataInput, 't') && isfield(dataInput, 'q')
        % Passed armHistory directly
        simData = dataInput;
        if ~isempty(eventInput) && isstruct(eventInput)
            simEvents = eventInput;
        else
            simEvents = estimateEventsFromData(simData);
        end
    else
        [simData, simEvents] = generateBenchmarkSimulation(p.Results.ObstacleMass);
    end

    %% 2. PRINT CONSOLE REPORT
    printSummaryReport(simData, simEvents);

    oldDock = 'normal';
    try
        oldDock = get(0, 'DefaultFigureWindowStyle');
    catch; end
    if dockFigures && usejava('desktop')
        try; set(0, 'DefaultFigureWindowStyle', 'docked'); catch; end
    else
        try; set(0, 'DefaultFigureWindowStyle', 'normal'); catch; end
    end

    handles = struct();
    t      = simData.t;
    q1     = simData.q1;
    q2     = simData.q2;
    q1_des = simData.q1_des;
    q2_des = simData.q2_des;
    e1     = simData.e1;
    e2     = simData.e2;
    tau1   = simData.tau1;
    tau2   = simData.tau2;
    t_pick = simEvents.t_pick;
    t_drop = simEvents.t_drop;

    % Style palettes
    colDarkBg   = [0.07 0.08 0.11];
    colAxesBg   = [0.09 0.11 0.15];
    colGrid     = [0.22 0.25 0.32];
    colText     = [0.85 0.88 0.94];
    colLink1    = [0.15 0.55 0.95];   % Vibrant blue
    colLink2    = [0.98 0.50 0.15];   % Vibrant orange
    colDes1     = [0.95 0.85 0.15];   % Yellow dashed
    colDes2     = [0.85 0.20 0.85];   % Magenta dashed
    colPick     = [1.00 0.80 0.10];   % Gold event highlight
    colDrop     = [0.15 0.90 0.45];   % Emerald event highlight

    %% =========================================================================
    %% FIGURE 2: JOINT 1 ERROR
    %% =========================================================================
    handles.fig2 = figure(2); clf(handles.fig2);
    set(handles.fig2, 'Name', 'Figure 2: Joint 1 Error', 'NumberTitle', 'off', ...
        'Color', colDarkBg);
    ax2 = axes('Parent', handles.fig2, 'Color', colAxesBg, 'XColor', colText, ...
        'YColor', colText, 'GridColor', colGrid, 'GridAlpha', 0.6, 'Box', 'on');
    hold(ax2, 'on'); grid(ax2, 'on');

    % Shaded phase regions
    yl = [min(e1)-0.8, max(e1)+1.2];
    if yl(1) > -1; yl(1) = -2.5; end
    if yl(2) < 2;  yl(2) = 3.5; end
    patch(ax2, [0, t_pick, t_pick, 0], [yl(1) yl(1) yl(2) yl(2)], ...
        [0.1 0.3 0.5], 'FaceAlpha', 0.08, 'EdgeColor', 'none');
    patch(ax2, [t_pick, t_drop, t_drop, t_pick], [yl(1) yl(1) yl(2) yl(2)], ...
        [0.8 0.5 0.1], 'FaceAlpha', 0.12, 'EdgeColor', 'none');
    patch(ax2, [t_drop, t(end), t(end), t_drop], [yl(1) yl(1) yl(2) yl(2)], ...
        [0.1 0.6 0.3], 'FaceAlpha', 0.08, 'EdgeColor', 'none');

    % Zero line
    yline(ax2, 0, '--', 'Color', [0.45 0.48 0.55], 'LineWidth', 1);

    % Main error plot
    plot(ax2, t, e1, '-', 'Color', colLink1, 'LineWidth', 2.2);

    % Event lines
    xline(ax2, t_pick, '--', 'Color', colPick, 'LineWidth', 2.2);
    xline(ax2, t_drop, '--', 'Color', colDrop, 'LineWidth', 1.8);

    % Prominent Callout Badge for Obstacle Pick
    idxPick = find(t >= t_pick, 1);
    plot(ax2, t_pick, e1(idxPick), 'p', 'MarkerSize', 14, ...
        'MarkerFaceColor', colPick, 'MarkerEdgeColor', 'k', 'LineWidth', 1.2);
    text(ax2, t_pick + 0.25, e1(idxPick) + 0.6, ...
        sprintf('  ★ OBSTACLE PICKED  [t = %.2f s]\n  Mass: %.1f kg  |  Error Spike: +%.2f°', ...
        t_pick, simEvents.obstacleMass, e1(idxPick)), ...
        'Color', [1 0.95 0.2], 'FontWeight', 'bold', 'FontSize', 9.5, ...
        'BackgroundColor', [0.05 0.07 0.12], 'EdgeColor', colPick, 'Margin', 5);

    % Obstacle Drop text
    idxDrop = find(t >= t_drop, 1);
    plot(ax2, t_drop, e1(idxDrop), 's', 'MarkerSize', 10, ...
        'MarkerFaceColor', colDrop, 'MarkerEdgeColor', 'k');
    text(ax2, t_drop - 0.25, e1(idxDrop) - 0.5, ...
        sprintf('■ OBSTACLE DROPPED  [t = %.2f s]  ', t_drop), ...
        'Color', colDrop, 'FontWeight', 'bold', 'FontSize', 9, ...
        'HorizontalAlignment', 'right', ...
        'BackgroundColor', [0.05 0.07 0.12], 'EdgeColor', colDrop, 'Margin', 4);

    xlabel(ax2, 'Simulation Time (s)', 'FontSize', 11, 'Color', colText);
    ylabel(ax2, 'Joint 1 Tracking Error e_1 (deg)', 'FontSize', 11, 'Color', colText);
    title(ax2, 'Figure 2: Joint 1 Tracking Error  e_1(t) = q_{1d}(t) - q_1(t)', ...
        'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.4 0.8 1.0]);
    ylim(ax2, yl); xlim(ax2, [0, t(end)]);

    % Stats badge
    rmsE1 = sqrt(mean(e1.^2));
    maxE1 = max(abs(e1));
    annotation(handles.fig2, 'textbox', [0.15 0.14 0.28 0.09], ...
        'String', sprintf('RMS Error: %.3f°\nPeak Error: %.3f°\nPayload: %.1f kg', rmsE1, maxE1, simEvents.obstacleMass), ...
        'Color', [0.8 0.9 1.0], 'BackgroundColor', [0.04 0.06 0.10], ...
        'EdgeColor', [0.3 0.5 0.8], 'FontSize', 8.5, 'FitBoxToText', 'on');

    %% =========================================================================
    %% FIGURE 3: JOINT 2 ERROR
    %% =========================================================================
    handles.fig3 = figure(3); clf(handles.fig3);
    set(handles.fig3, 'Name', 'Figure 3: Joint 2 Error', 'NumberTitle', 'off', ...
        'Color', colDarkBg);
    ax3 = axes('Parent', handles.fig3, 'Color', colAxesBg, 'XColor', colText, ...
        'YColor', colText, 'GridColor', colGrid, 'GridAlpha', 0.6, 'Box', 'on');
    hold(ax3, 'on'); grid(ax3, 'on');

    yl3 = [min(e2)-0.8, max(e2)+1.2];
    if yl3(1) > -1; yl3(1) = -2.5; end
    if yl3(2) < 2;  yl3(2) = 3.5; end
    patch(ax3, [0, t_pick, t_pick, 0], [yl3(1) yl3(1) yl3(2) yl3(2)], ...
        [0.1 0.3 0.5], 'FaceAlpha', 0.08, 'EdgeColor', 'none');
    patch(ax3, [t_pick, t_drop, t_drop, t_pick], [yl3(1) yl3(1) yl3(2) yl3(2)], ...
        [0.8 0.5 0.1], 'FaceAlpha', 0.12, 'EdgeColor', 'none');
    patch(ax3, [t_drop, t(end), t(end), t_drop], [yl3(1) yl3(1) yl3(2) yl3(2)], ...
        [0.1 0.6 0.3], 'FaceAlpha', 0.08, 'EdgeColor', 'none');

    yline(ax3, 0, '--', 'Color', [0.45 0.48 0.55], 'LineWidth', 1);
    plot(ax3, t, e2, '-', 'Color', colLink2, 'LineWidth', 2.2);

    xline(ax3, t_pick, '--', 'Color', colPick, 'LineWidth', 2.2);
    xline(ax3, t_drop, '--', 'Color', colDrop, 'LineWidth', 1.8);

    idxPick2 = find(t >= t_pick, 1);
    plot(ax3, t_pick, e2(idxPick2), 'p', 'MarkerSize', 14, ...
        'MarkerFaceColor', colPick, 'MarkerEdgeColor', 'k', 'LineWidth', 1.2);
    text(ax3, t_pick + 0.25, e2(idxPick2) + 0.6, ...
        sprintf('  ★ OBSTACLE PICKED  [t = %.2f s]\n  Elbow Dynamic Perturbation: %.2f°', ...
        t_pick, e2(idxPick2)), ...
        'Color', [1 0.95 0.2], 'FontWeight', 'bold', 'FontSize', 9.5, ...
        'BackgroundColor', [0.05 0.07 0.12], 'EdgeColor', colPick, 'Margin', 5);

    idxDrop2 = find(t >= t_drop, 1);
    plot(ax3, t_drop, e2(idxDrop2), 's', 'MarkerSize', 10, ...
        'MarkerFaceColor', colDrop, 'MarkerEdgeColor', 'k');
    text(ax3, t_drop - 0.25, e2(idxDrop2) - 0.5, ...
        sprintf('■ OBSTACLE RELEASED  [t = %.2f s]  ', t_drop), ...
        'Color', colDrop, 'FontWeight', 'bold', 'FontSize', 9, ...
        'HorizontalAlignment', 'right', ...
        'BackgroundColor', [0.05 0.07 0.12], 'EdgeColor', colDrop, 'Margin', 4);

    xlabel(ax3, 'Simulation Time (s)', 'FontSize', 11, 'Color', colText);
    ylabel(ax3, 'Joint 2 Tracking Error e_2 (deg)', 'FontSize', 11, 'Color', colText);
    title(ax3, 'Figure 3: Joint 2 Tracking Error  e_2(t) = q_{2d}(t) - q_2(t)', ...
        'FontSize', 12, 'FontWeight', 'bold', 'Color', [1.0 0.6 0.2]);
    ylim(ax3, yl3); xlim(ax3, [0, t(end)]);

    rmsE2 = sqrt(mean(e2.^2));
    maxE2 = max(abs(e2));
    annotation(handles.fig3, 'textbox', [0.15 0.14 0.28 0.09], ...
        'String', sprintf('RMS Error: %.3f°\nPeak Error: %.3f°\nJoint: Elbow Pitch', rmsE2, maxE2), ...
        'Color', [1.0 0.85 0.7], 'BackgroundColor', [0.04 0.06 0.10], ...
        'EdgeColor', [0.8 0.45 0.2], 'FontSize', 8.5, 'FitBoxToText', 'on');

    %% =========================================================================
    %% FIGURE 4: TRACKING ERRORS
    %% =========================================================================
    handles.fig4 = figure(4); clf(handles.fig4);
    set(handles.fig4, 'Name', 'Figure 4: Tracking Errors', 'NumberTitle', 'off', ...
        'Color', colDarkBg);

    % Top Subplot: Joint 1 vs Joint 2
    ax4a = subplot(2, 1, 1, 'Parent', handles.fig4);
    set(ax4a, 'Color', colAxesBg, 'XColor', colText, 'YColor', colText, ...
        'GridColor', colGrid, 'GridAlpha', 0.6, 'Box', 'on');
    hold(ax4a, 'on'); grid(ax4a, 'on');
    yline(ax4a, 0, '--', 'Color', [0.45 0.48 0.55], 'LineWidth', 0.8);
    p1 = plot(ax4a, t, e1, '-', 'Color', colLink1, 'LineWidth', 2.0);
    p2 = plot(ax4a, t, e2, '-', 'Color', colLink2, 'LineWidth', 2.0);
    xline(ax4a, t_pick, '--', 'Color', colPick, 'LineWidth', 2.0);
    xline(ax4a, t_drop, '--', 'Color', colDrop, 'LineWidth', 1.8);
    title(ax4a, 'Figure 4: Joint Tracking Errors & Norm Comparison', ...
        'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.9 0.9 0.95]);
    ylabel(ax4a, 'Joint Errors (deg)', 'FontSize', 10, 'Color', colText);
    legend(ax4a, [p1, p2], {'Joint 1 Error (e_1)', 'Joint 2 Error (e_2)'}, ...
        'TextColor', colText, 'Color', [0.05 0.07 0.12], 'EdgeColor', [0.3 0.3 0.4], ...
        'Location', 'northeast');
    xlim(ax4a, [0, t(end)]);

    % Bottom Subplot: Composite Error Norm ||e||
    ax4b = subplot(2, 1, 2, 'Parent', handles.fig4);
    set(ax4b, 'Color', colAxesBg, 'XColor', colText, 'YColor', colText, ...
        'GridColor', colGrid, 'GridAlpha', 0.6, 'Box', 'on');
    hold(ax4b, 'on'); grid(ax4b, 'on');
    eNorm = sqrt(e1.^2 + e2.^2);
    area(ax4b, t, eNorm, 'FaceColor', [0.85 0.25 0.85], 'FaceAlpha', 0.25, ...
        'EdgeColor', [0.95 0.4 0.95], 'LineWidth', 1.8);
    xline(ax4b, t_pick, '--', 'Color', colPick, 'LineWidth', 2.0);
    xline(ax4b, t_drop, '--', 'Color', colDrop, 'LineWidth', 1.8);

    % Phase labels along top
    yMaxN = max(eNorm) * 1.25;
    if yMaxN < 1; yMaxN = 3; end
    ylim(ax4b, [0, yMaxN]);
    text(ax4b, t_pick/2, yMaxN*0.88, '1. APPROACH & REACH', ...
        'Color', [0.6 0.8 1.0], 'FontSize', 8, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    text(ax4b, (t_pick + t_drop)/2, yMaxN*0.88, '2. OBSTACLE HELD & TRANSFERRED', ...
        'Color', [1.0 0.8 0.2], 'FontSize', 8, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    text(ax4b, (t_drop + t(end))/2, yMaxN*0.88, '3. BIN DROP & RETURN', ...
        'Color', [0.3 0.9 0.5], 'FontSize', 8, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');

    xlabel(ax4b, 'Simulation Time (s)', 'FontSize', 11, 'Color', colText);
    ylabel(ax4b, '||e(t)|| = \surd(e_1^2 + e_2^2)  (deg)', 'FontSize', 10, 'Color', colText);
    xlim(ax4b, [0, t(end)]);

    %% =========================================================================
    %% FIGURE 5: CONTROL TORQUES
    %% =========================================================================
    handles.fig5 = figure(5); clf(handles.fig5);
    set(handles.fig5, 'Name', 'Figure 5: Control Torques', 'NumberTitle', 'off', ...
        'Color', colDarkBg);
    ax5 = axes('Parent', handles.fig5, 'Color', colAxesBg, 'XColor', colText, ...
        'YColor', colText, 'GridColor', colGrid, 'GridAlpha', 0.6, 'Box', 'on');
    hold(ax5, 'on'); grid(ax5, 'on');

    % Main torque trajectories
    pt1 = plot(ax5, t, tau1, '-', 'Color', colLink1, 'LineWidth', 2.2);
    pt2 = plot(ax5, t, tau2, '-', 'Color', colLink2, 'LineWidth', 2.2);

    % Event lines
    xline(ax5, t_pick, '--', 'Color', colPick, 'LineWidth', 2.2);
    xline(ax5, t_drop, '--', 'Color', colDrop, 'LineWidth', 1.8);

    % Step Change Annotations
    idxBefore = max(1, find(t >= t_pick, 1) - 4);
    idxAfter  = min(length(t), find(t >= t_pick, 1) + 4);
    dTorque1  = tau1(idxAfter) - tau1(idxBefore);
    text(ax5, t_pick + 0.25, max(tau1) * 0.92, ...
        sprintf('  ▲ TORQUE SURGE AT PICK [t = %.2f s]\n  \\Delta\\tau_1 = +%.1f Nm (Obstacle Gravitational Load)', ...
        t_pick, abs(dTorque1)), ...
        'Color', colPick, 'FontWeight', 'bold', 'FontSize', 9.5, ...
        'BackgroundColor', [0.05 0.07 0.12], 'EdgeColor', colPick, 'Margin', 5);

    text(ax5, t_drop + 0.25, min(tau2) * 0.65, ...
        sprintf('  ▼ LOAD REMOVED AT DROP [t = %.2f s]\n  Torques Return to Baseline', t_drop), ...
        'Color', colDrop, 'FontWeight', 'bold', 'FontSize', 9, ...
        'BackgroundColor', [0.05 0.07 0.12], 'EdgeColor', colDrop, 'Margin', 4);

    xlabel(ax5, 'Simulation Time (s)', 'FontSize', 11, 'Color', colText);
    ylabel(ax5, 'Actuator Torque \tau (N\cdotm)', 'FontSize', 11, 'Color', colText);
    title(ax5, 'Figure 5: Control Torques  \tau_1(t), \tau_2(t)  [Rigid-Body Dynamics]', ...
        'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.3 0.95 0.7]);
    legend(ax5, [pt1, pt2], {'Joint 1 (Shoulder Actuator)', 'Joint 2 (Elbow Actuator)'}, ...
        'TextColor', colText, 'Color', [0.05 0.07 0.12], 'EdgeColor', [0.3 0.3 0.4], ...
        'Location', 'northwest');
    xlim(ax5, [0, t(end)]);

    %% =========================================================================
    %% FIGURE 6: PHASE PLANE
    %% =========================================================================
    handles.fig6 = figure(6); clf(handles.fig6);
    set(handles.fig6, 'Name', 'Figure 6: Phase Plane', 'NumberTitle', 'off', ...
        'Color', colDarkBg);

    ax6a = subplot(1, 2, 1, 'Parent', handles.fig6);
    set(ax6a, 'Color', colAxesBg, 'XColor', colText, 'YColor', colText, ...
        'GridColor', colGrid, 'GridAlpha', 0.6, 'Box', 'on');
    hold(ax6a, 'on'); grid(ax6a, 'on');

    q1dot = simData.q1dot;
    q2dot = simData.q2dot;
    idxPick = find(t >= t_pick, 1);
    if isempty(idxPick); idxPick = max(1, round(length(t)/3)); end

    plot(ax6a, q1, q1dot, '-', 'Color', colLink1, 'LineWidth', 2.0);
    plot(ax6a, q1(1), q1dot(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    plot(ax6a, q1(end), q1dot(end), 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    plot(ax6a, q1(idxPick), q1dot(idxPick), 'p', 'MarkerSize', 14, ...
        'MarkerFaceColor', colPick, 'MarkerEdgeColor', 'k', 'LineWidth', 1.2);
    text(ax6a, q1(idxPick)+1, q1dot(idxPick), ...
        sprintf('★ Picked (t = %.2f s)', t_pick), ...
        'Color', colPick, 'FontWeight', 'bold', 'FontSize', 9);

    xlabel(ax6a, 'Joint 1 Position \theta_1 (deg)', 'FontSize', 10, 'Color', colText);
    ylabel(ax6a, 'Joint 1 Velocity \omega_1 (deg/s)', 'FontSize', 10, 'Color', colText);
    title(ax6a, 'Joint 1 Phase Plane (\theta_1 vs \omega_1)', 'FontSize', 11, 'Color', colLink1, 'FontWeight', 'bold');

    ax6b = subplot(1, 2, 2, 'Parent', handles.fig6);
    set(ax6b, 'Color', colAxesBg, 'XColor', colText, 'YColor', colText, ...
        'GridColor', colGrid, 'GridAlpha', 0.6, 'Box', 'on');
    hold(ax6b, 'on'); grid(ax6b, 'on');

    plot(ax6b, q2, q2dot, '-', 'Color', colLink2, 'LineWidth', 2.0);
    plot(ax6b, q2(1), q2dot(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    plot(ax6b, q2(end), q2dot(end), 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    plot(ax6b, q2(idxPick), q2dot(idxPick), 'p', 'MarkerSize', 14, ...
        'MarkerFaceColor', colPick, 'MarkerEdgeColor', 'k', 'LineWidth', 1.2);
    text(ax6b, q2(idxPick)+1, q2dot(idxPick), ...
        sprintf('★ Picked (t = %.2f s)', t_pick), ...
        'Color', colPick, 'FontWeight', 'bold', 'FontSize', 9);

    xlabel(ax6b, 'Joint 2 Position \theta_2 (deg)', 'FontSize', 10, 'Color', colText);
    ylabel(ax6b, 'Joint 2 Velocity \omega_2 (deg/s)', 'FontSize', 10, 'Color', colText);
    title(ax6b, 'Joint 2 Phase Plane (\theta_2 vs \omega_2)', 'FontSize', 11, 'Color', colLink2, 'FontWeight', 'bold');

    %% =========================================================================
    %% FIGURE 7: TWO-LINK ROBOT ANIMATION (MATCHING SCREENSHOT LAYOUT)
    %% =========================================================================
    handles.fig7 = figure(7); clf(handles.fig7);
    set(handles.fig7, 'Name', 'Figure 7: Two-Link Robot Animation', 'NumberTitle', 'off', ...
        'Color', colDarkBg, 'Position', [100 80 820 720]);
    ax7 = axes('Parent', handles.fig7, 'Color', [0.08 0.08 0.08], ...
        'XColor', [0.8 0.8 0.85], 'YColor', [0.8 0.8 0.85], ...
        'GridColor', [0.28 0.28 0.32], 'GridAlpha', 0.65, 'Box', 'on');
    hold(ax7, 'on'); axis(ax7, 'equal'); grid(ax7, 'on');
    xlim(ax7, [-2.2, 2.2]);
    ylim(ax7, [-2.2, 2.2]);
    xlabel(ax7, 'X (m)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.9 0.9 0.9]);
    ylabel(ax7, 'Y (m)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.9 0.9 0.9]);

    % Kinematics parameters
    L1 = simData.L1;
    L2 = simData.L2;

    % End-effector trajectory coordinates
    radQ1     = deg2rad(q1);
    radQ2     = deg2rad(q2);
    radQ1_des = deg2rad(q1_des);
    radQ2_des = deg2rad(q2_des);

    eeX = L1 * cos(radQ1) + L2 * cos(radQ1 + radQ2);
    eeY = L1 * sin(radQ1) + L2 * sin(radQ1 + radQ2);

    % Static railway & bin ground reference
    plot(ax7, [-2.2 2.2], [-0.45 -0.45], 'Color', [0.45 0.40 0.35], 'LineWidth', 2.5); % ground line
    rectangle('Parent', ax7, 'Position', [simEvents.dropPos(1)-0.15, simEvents.dropPos(2)-0.12, 0.30, 0.22], ...
        'Curvature', 0.2, 'FaceColor', [0.18 0.25 0.35], 'EdgeColor', [0.3 0.6 0.9], 'LineWidth', 1.5);
    text(ax7, simEvents.dropPos(1), simEvents.dropPos(2) - 0.20, 'ON-BOARD BIN', ...
        'Color', [0.4 0.7 1.0], 'FontSize', 8, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');

    % Trajectory trails
    hTrailBefore = plot(ax7, nan, nan, '-', 'Color', [0.2 0.75 1.0], 'LineWidth', 1.5);
    hTrailHeld   = plot(ax7, nan, nan, '-', 'Color', [1.0 0.65 0.15], 'LineWidth', 2.8);
    hTrailAfter  = plot(ax7, nan, nan, '--', 'Color', [0.2 0.9 0.45], 'LineWidth', 1.5);

    % Obstacle initial point badge
    plot(ax7, simEvents.pickPos(1), simEvents.pickPos(2), 'p', 'MarkerSize', 15, ...
        'MarkerFaceColor', colPick, 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
    text(ax7, simEvents.pickPos(1) + 0.10, simEvents.pickPos(2) - 0.15, ...
        sprintf('★ OBSTACLE PICK POINT\n   (t = %.2f s)', t_pick), ...
        'Color', colPick, 'FontWeight', 'bold', 'FontSize', 9, ...
        'BackgroundColor', [0.04 0.05 0.08], 'EdgeColor', colPick, 'Margin', 4);

    % Obstacle graphics (rock / box)
    hObstacle = rectangle('Parent', ax7, 'Position', [simEvents.pickPos(1)-0.08, simEvents.pickPos(2)-0.08, 0.16, 0.16], ...
        'Curvature', 0.6, 'FaceColor', [0.85 0.55 0.12], 'EdgeColor', [1 0.9 0.3], 'LineWidth', 1.5);

    % Desired Links (yellow dashed, magenta dashed)
    hDesLink1 = plot(ax7, [0, 0], [0, 0], '--', 'Color', colDes1, 'LineWidth', 2.5);
    hDesLink2 = plot(ax7, [0, 0], [0, 0], '--', 'Color', colDes2, 'LineWidth', 2.5);

    % Actual Links (blue link 1, orange link 2) - Exactly matching screenshot
    hLink1 = plot(ax7, [0, 0], [0, 0], '-o', 'Color', colLink1, 'LineWidth', 4.0, ...
        'MarkerSize', 11, 'MarkerFaceColor', colLink1, 'MarkerEdgeColor', [0.8 0.9 1.0]);
    hLink2 = plot(ax7, [0, 0], [0, 0], '-o', 'Color', colLink2, 'LineWidth', 4.0, ...
        'MarkerSize', 11, 'MarkerFaceColor', colLink2, 'MarkerEdgeColor', [1.0 0.9 0.8]);

    % Base pivot
    plot(ax7, 0, 0, 's', 'MarkerSize', 12, 'MarkerFaceColor', [0.3 0.3 0.35], ...
        'MarkerEdgeColor', [0.7 0.7 0.8], 'LineWidth', 1.5);

    % Legend exactly matching screenshot:
    % Link 1, Link 2, Desired Link 1, Desired Link 2
    legend(ax7, [hLink1, hLink2, hDesLink1, hDesLink2], ...
        {'Link 1', 'Link 2', 'Desired Link 1', 'Desired Link 2'}, ...
        'TextColor', [0.9 0.9 0.95], 'Color', [0.06 0.08 0.12], ...
        'EdgeColor', [0.4 0.4 0.5], 'FontSize', 10, 'Location', 'northeast');

    % Status text header
    hTitle = title(ax7, sprintf('Two-Link Manipulator - t = %.2f s', t(end)), ...
        'FontSize', 13, 'FontWeight', 'bold', 'Color', [1 1 1]);

    % Subtitle HUD badge
    hStatusBadge = annotation(handles.fig7, 'textbox', [0.15 0.86 0.70 0.05], ...
        'String', sprintf('STATUS: READY  |  t = %.2f s', t(end)), ...
        'Color', [0.2 1.0 0.6], 'BackgroundColor', [0.03 0.05 0.08], ...
        'EdgeColor', [0.2 0.8 0.5], 'FontSize', 10, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', 'FitBoxToText', 'off');

    % Attach data into figure for callbacks
    animState.t      = t;
    animState.radQ1  = radQ1;
    animState.radQ2  = radQ2;
    animState.radQ1_des = radQ1_des;
    animState.radQ2_des = radQ2_des;
    animState.eeX    = eeX;
    animState.eeY    = eeY;
    animState.L1     = L1;
    animState.L2     = L2;
    animState.t_pick = t_pick;
    animState.t_drop = t_drop;
    animState.pickPos= simEvents.pickPos;
    animState.dropPos= simEvents.dropPos;
    animState.hLink1 = hLink1;
    animState.hLink2 = hLink2;
    animState.hDesLink1 = hDesLink1;
    animState.hDesLink2 = hDesLink2;
    animState.hTrailBefore = hTrailBefore;
    animState.hTrailHeld   = hTrailHeld;
    animState.hTrailAfter  = hTrailAfter;
    animState.hObstacle    = hObstacle;
    animState.hTitle       = hTitle;
    animState.hStatusBadge = hStatusBadge;
    animState.ax           = ax7;
    animState.fig          = handles.fig7;
    animState.running      = false;

    setappdata(handles.fig7, 'animState', animState);
    set(handles.fig7, 'DeleteFcn', @(~,~) onFigureClosed(handles.fig7));

    %% 4. INTERACTIVE UI CONTROLS ON FIGURE 7
    % Add UI Control Buttons at bottom of Figure 7
    uicontrol('Parent', handles.fig7, 'Style', 'pushbutton', 'String', '▶ Play / ❚❚ Pause', ...
        'Units', 'normalized', 'Position', [0.08 0.02 0.17 0.045], ...
        'BackgroundColor', [0.15 0.25 0.40], 'ForegroundColor', [1 1 1], ...
        'FontWeight', 'bold', 'Callback', @(~,~) togglePlay(handles.fig7));

    uicontrol('Parent', handles.fig7, 'Style', 'pushbutton', 'String', '★ Jump to Pick', ...
        'Units', 'normalized', 'Position', [0.27 0.02 0.18 0.045], ...
        'BackgroundColor', [0.45 0.32 0.08], 'ForegroundColor', [1 0.95 0.2], ...
        'FontWeight', 'bold', 'Callback', @(~,~) jumpToTime(handles.fig7, t_pick));

    uicontrol('Parent', handles.fig7, 'Style', 'pushbutton', 'String', '■ Jump to Drop', ...
        'Units', 'normalized', 'Position', [0.47 0.02 0.18 0.045], ...
        'BackgroundColor', [0.08 0.35 0.18], 'ForegroundColor', [0.3 1.0 0.6], ...
        'FontWeight', 'bold', 'Callback', @(~,~) jumpToTime(handles.fig7, t_drop));

    uicontrol('Parent', handles.fig7, 'Style', 'pushbutton', 'String', '↺ Replay', ...
        'Units', 'normalized', 'Position', [0.67 0.02 0.13 0.045], ...
        'BackgroundColor', [0.22 0.22 0.28], 'ForegroundColor', [1 1 1], ...
        'FontWeight', 'bold', 'Callback', @(~,~) replayAnimation(handles.fig7));

    uicontrol('Parent', handles.fig7, 'Style', 'pushbutton', 'String', 'Export Report', ...
        'Units', 'normalized', 'Position', [0.82 0.02 0.14 0.045], ...
        'BackgroundColor', [0.20 0.35 0.30], 'ForegroundColor', [0.8 1.0 0.9], ...
        'FontWeight', 'bold', 'Callback', @(~,~) exportReportFile(handles.fig7, simData, simEvents));

    % Display final frame initially
    updateFrame(handles.fig7, length(t));

    % If animation requested, play through smoothly
    if doAnimate
        playAnimation(handles.fig7);
    end

    % Restore docking settings
    try
        if usejava('desktop')
            set(0, 'DefaultFigureWindowStyle', oldDock);
        end
    catch; end
    fprintf('\nAll Figures (Figure 2 through Figure 7) successfully generated.\n');
end

%% =========================================================================
%% ANIMATION HELPER FUNCTIONS
%% =========================================================================

function updateFrame(fig, k)
    if ~isgraphics(fig); return; end
    st = getappdata(fig, 'animState');
    if isempty(st); return; end

    k = max(1, min(k, length(st.t)));
    currT = st.t(k);

    % Actual links
    q1 = st.radQ1(k);
    q2 = st.radQ2(k);
    j1x = st.L1 * cos(q1);
    j1y = st.L1 * sin(q1);
    eex = j1x + st.L2 * cos(q1 + q2);
    eey = j1y + st.L2 * sin(q1 + q2);

    set(st.hLink1, 'XData', [0, j1x], 'YData', [0, j1y]);
    set(st.hLink2, 'XData', [j1x, eex], 'YData', [j1y, eey]);

    % Desired links
    q1d = st.radQ1_des(k);
    q2d = st.radQ2_des(k);
    j1xd = st.L1 * cos(q1d);
    j1yd = st.L1 * sin(q1d);
    eexd = j1xd + st.L2 * cos(q1d + q2d);
    eeyd = j1yd + st.L2 * sin(q1d + q2d);

    set(st.hDesLink1, 'XData', [0, j1xd], 'YData', [0, j1yd]);
    set(st.hDesLink2, 'XData', [j1xd, eexd], 'YData', [j1yd, eeyd]);

    % Trails
    idxPick = find(st.t >= st.t_pick, 1);
    idxDrop = find(st.t >= st.t_drop, 1);

    if k <= idxPick
        set(st.hTrailBefore, 'XData', st.eeX(1:k), 'YData', st.eeY(1:k));
        set(st.hTrailHeld,   'XData', nan,         'YData', nan);
        set(st.hTrailAfter,  'XData', nan,         'YData', nan);
    elseif k <= idxDrop
        set(st.hTrailBefore, 'XData', st.eeX(1:idxPick), 'YData', st.eeY(1:idxPick));
        set(st.hTrailHeld,   'XData', st.eeX(idxPick:k), 'YData', st.eeY(idxPick:k));
        set(st.hTrailAfter,  'XData', nan,               'YData', nan);
    else
        set(st.hTrailBefore, 'XData', st.eeX(1:idxPick), 'YData', st.eeY(1:idxPick));
        set(st.hTrailHeld,   'XData', st.eeX(idxPick:idxDrop), 'YData', st.eeY(idxPick:idxDrop));
        set(st.hTrailAfter,  'XData', st.eeX(idxDrop:k),       'YData', st.eeY(idxDrop:k));
    end

    % Obstacle position:
    % Before pick: at pickPos
    % Between pick and drop: attached to gripper (eex, eey)
    % After drop: smooth gravity settle into on-board bin
    obsW = 0.16; obsH = 0.16;
    if currT < st.t_pick
        set(st.hObstacle, 'Position', [st.pickPos(1)-obsW/2, st.pickPos(2)-obsH/2, obsW, obsH], ...
            'FaceColor', [0.85 0.55 0.12]);
        phaseStr = 'APPROACHING OBSTACLE';
        badgeColor = [0.3 0.8 1.0];
    elseif currT >= st.t_pick && currT <= st.t_drop
        set(st.hObstacle, 'Position', [eex-obsW/2, eey-obsH/2, obsW, obsH], ...
            'FaceColor', [1.00 0.80 0.10]); % glowing gold when held
        phaseStr = sprintf('★ OBSTACLE HELD & TRANSFERRING (Picked at t = %.2f s)', st.t_pick);
        badgeColor = [1.0 0.8 0.1];
    else
        % Released at t_drop: smooth cubic easing settle into resting position
        dropSettleTime = 0.40;
        dtDrop = currT - st.t_drop;
        if dtDrop < dropSettleTime
            alpha = min(1, max(0, dtDrop / dropSettleTime));
            sY = (1 - alpha^2) * (st.dropPos(2) + 0.05) + (alpha^2) * st.dropPos(2);
            set(st.hObstacle, 'Position', [st.dropPos(1)-obsW/2, sY-obsH/2, obsW, obsH], ...
                'FaceColor', [0.65 0.55 0.35]);
            phaseStr = sprintf('▼ RELEASING INTO BIN (t = %.2f s)', st.t_drop);
            badgeColor = [0.2 0.9 0.4];
        else
            set(st.hObstacle, 'Position', [st.dropPos(1)-obsW/2, st.dropPos(2)-obsH/2, obsW, obsH], ...
                'FaceColor', [0.35 0.40 0.45]);
            phaseStr = sprintf('■ OBSTACLE SECURED IN BIN (t = %.2f s)', st.t_drop);
            badgeColor = [0.3 0.9 0.5];
        end
    end

    % Title
    set(st.hTitle, 'String', sprintf('Two-Link Manipulator - t = %.2f s', currT));
    set(st.hStatusBadge, 'String', ...
        sprintf('TIME: %5.2f s  |  %s  |  q_1: %+5.1f°  q_2: %+5.1f°', ...
        currT, phaseStr, rad2deg(q1), rad2deg(q2)), ...
        'Color', badgeColor, 'EdgeColor', badgeColor);

    drawnow limitrate;
end

function playAnimation(fig)
    if ~isgraphics(fig); return; end
    st = getappdata(fig, 'animState');
    if isempty(st); return; end

    st.running = true;
    setappdata(fig, 'animState', st);

    N = length(st.t);
    % Silky smooth 50 FPS playback with zero buffering / lag
    targetFps    = 50;
    animDuration = 7.5; % total playback seconds
    totalFrames  = max(60, round(animDuration * targetFps));
    frameIndices = unique(round(linspace(1, N, totalFrames)));
    numFrames    = length(frameIndices);

    tStart = tic;
    for idx = 1:numFrames
        if ~isgraphics(fig); return; end
        st = getappdata(fig, 'animState');
        if isempty(st) || ~st.running; return; end

        k = frameIndices(idx);
        updateFrame(fig, k);

        % Smooth wall-clock pacing
        expectedElapsed = (idx / numFrames) * animDuration;
        actualElapsed   = toc(tStart);
        sleepTime       = expectedElapsed - actualElapsed;
        if sleepTime > 0.002
            pause(sleepTime);
        else
            drawnow limitrate;
        end
    end

    if isgraphics(fig)
        updateFrame(fig, N);
        st = getappdata(fig, 'animState');
        if ~isempty(st)
            st.running = false;
            setappdata(fig, 'animState', st);
        end
    end
end

function onFigureClosed(fig)
    try
        st = getappdata(fig, 'animState');
        if ~isempty(st)
            st.running = false;
            setappdata(fig, 'animState', st);
        end
    catch; end
end

function togglePlay(fig)
    if ~isgraphics(fig); return; end
    st = getappdata(fig, 'animState');
    if isempty(st); return; end

    if st.running
        st.running = false;
        setappdata(fig, 'animState', st);
    else
        playAnimation(fig);
    end
end

function jumpToTime(fig, targetT)
    if ~isgraphics(fig); return; end
    st = getappdata(fig, 'animState');
    if isempty(st); return; end

    st.running = false;
    setappdata(fig, 'animState', st);

    idx = find(st.t >= targetT, 1);
    if isempty(idx); idx = length(st.t); end
    updateFrame(fig, idx);
end

function replayAnimation(fig)
    if ~isgraphics(fig); return; end
    st = getappdata(fig, 'animState');
    if isempty(st); return; end
    st.running = false;
    setappdata(fig, 'animState', st);
    updateFrame(fig, 1);
    pause(0.1);
    playAnimation(fig);
end

function exportReportFile(fig, simData, simEvents)
    filename = 'Manipulator_Movement_Report.txt';
    fid = fopen(filename, 'w');
    if fid >= 0
        fprintf(fid, '========================================================================\n');
        fprintf(fid, '         TWO-LINK MANIPULATOR MOVEMENT & DYNAMICS REPORT\n');
        fprintf(fid, '========================================================================\n');
        fprintf(fid, 'Generated:                      %s\n', datestr(now));
        fprintf(fid, 'Obstacle Mass (Payload):        %.2f kg\n', simEvents.obstacleMass);
        fprintf(fid, 'Pick Location (X, Y):           [%.2f m, %.2f m]\n', simEvents.pickPos(1), simEvents.pickPos(2));
        fprintf(fid, 'Drop Location (X, Y):           [%.2f m, %.2f m]\n', simEvents.dropPos(1), simEvents.dropPos(2));
        fprintf(fid, '------------------------------------------------------------------------\n');
        fprintf(fid, 'KEY TIMINGS:\n');
        fprintf(fid, '  • REACH Initiated:            t = %6.2f s\n', simEvents.t_reach_start);
        fprintf(fid, '  • ★ OBSTACLE PICKED & GRASPED: t = %6.2f s  <-- [PAYLOAD ACQUIRED]\n', simEvents.t_pick);
        fprintf(fid, '  • Obstacle Lift:              t = %6.2f s\n', simEvents.t_lift);
        fprintf(fid, '  • ■ OBSTACLE DROPPED IN BIN:   t = %6.2f s  <-- [PAYLOAD RELEASED]\n', simEvents.t_drop);
        fprintf(fid, '  • Arm Returned to Home:       t = %6.2f s\n', simEvents.t_home);
        fprintf(fid, '------------------------------------------------------------------------\n');
        fprintf(fid, 'PERFORMANCE SUMMARY:\n');
        fprintf(fid, '  • Peak Shoulder Torque (tau_1):%6.2f Nm\n', max(abs(simData.tau1)));
        fprintf(fid, '  • Peak Elbow Torque (tau_2):   %6.2f Nm\n', max(abs(simData.tau2)));
        fprintf(fid, '  • Joint 1 RMS Tracking Error: %6.3f deg\n', sqrt(mean(simData.e1.^2)));
        fprintf(fid, '  • Joint 2 RMS Tracking Error: %6.3f deg\n', sqrt(mean(simData.e2.^2)));
        fprintf(fid, '========================================================================\n');
        fclose(fid);
        fprintf('\n>>> [REPORT SAVED] Full report written to: %s\n\n', fullfile(pwd, filename));
    end

    if isgraphics(fig)
        st = getappdata(fig, 'animState');
        if ~isempty(st) && isfield(st, 'hStatusBadge') && isgraphics(st.hStatusBadge)
            set(st.hStatusBadge, 'String', sprintf('✓ REPORT EXPORTED TO %s', filename), ...
                'Color', [0.2 1.0 0.5], 'EdgeColor', [0.2 1.0 0.5]);
        end
    end
end

%% =========================================================================
%% BENCHMARK SIMULATION GENERATOR (PHYSICS-BASED MODEL)
%% =========================================================================

function [simData, simEvents] = generateBenchmarkSimulation(obstacleMass)
% Generates realistic 2-link planar manipulator kinematics and dynamics
% covering standby -> reach -> grasp/pick -> lift -> transfer -> drop -> home

    if nargin < 1 || isempty(obstacleMass); obstacleMass = 8.0; end

    L1 = 0.60;   % Link 1 length (m)
    L2 = 0.60;   % Link 2 length (m)
    m1 = 1.80;   % Link 1 mass (kg)
    m2 = 1.20;   % Link 2 mass (kg)
    g  = 9.81;

    t_end  = 15.00;
    dt     = 0.02;
    t      = (0:dt:t_end)';
    N      = length(t);

    % Event timestamps
    t_reach_start = 1.20;
    t_pick        = 4.35;    % Exact moment obstacle is locked/picked
    t_lift        = 6.50;
    t_transfer    = 9.50;
    t_drop        = 11.20;   % Exact moment obstacle drops in bin
    t_home        = 14.50;

    pickPos = [0.85, -0.22];   % Obstacle on track
    dropPos = [-0.35, 0.40];   % On-board collection container

    % Key posture waypoints [q1, q2] in degrees
    % q1 = angle relative to horizontal, q2 = angle relative to link 1
    q_home       = [35,   75];
    q_prepick    = [30,  -50];
    q_pick       = [28.5, -86.0];   % Exact target at pick [0.85, -0.22]
    q_lifted     = [55,  -20];
    q_transfer   = [100,  45];
    q_drop_hover = [66,  118];      % Hovering right above bin [-0.35, 0.50]
    q_drop       = [67,  126];      % Target right inside the container [-0.35, 0.417]

    % Construct smooth minimum-jerk spline for desired trajectory qd(t)
    wayTimes = [0, t_reach_start, 2.80, t_pick, 5.50, t_lift, 8.20, t_transfer, 10.30, t_drop, 11.90, t_home, t_end];
    wayQ1    = [q_home(1), q_home(1), q_prepick(1), q_pick(1), q_pick(1)+2, q_lifted(1), ...
                (q_lifted(1)+q_transfer(1))/2, q_transfer(1), q_drop_hover(1), q_drop(1), q_drop_hover(1), q_home(1), q_home(1)];
    wayQ2    = [q_home(2), q_home(2), q_prepick(2), q_pick(2), q_pick(2)+2, q_lifted(2), ...
                (q_lifted(2)+q_transfer(2))/2, q_transfer(2), q_drop_hover(2), q_drop(2), q_drop_hover(2), q_home(2), q_home(2)];

    q1_des = pchip(wayTimes, wayQ1, t);
    q2_des = pchip(wayTimes, wayQ2, t);

    q1dot_des = gradient(q1_des, dt);
    q2dot_des = gradient(q2_des, dt);

    % Controller and Dynamic Integration
    % Computed Torque + PD: tau = Kp*e + Kd*de + G_nominal
    Kp = diag([140, 110]);
    Kd = diag([ 24,  18]);

    q1    = zeros(N, 1);
    q2    = zeros(N, 1);
    q1dot = zeros(N, 1);
    q2dot = zeros(N, 1);
    tau1  = zeros(N, 1);
    tau2  = zeros(N, 1);

    % Initial condition
    q1(1) = q1_des(1);
    q2(1) = q2_des(1);

    % Forward numerical integration of robot equations of motion with substepping
    nSub  = 25;
    dtSub = dt / nSub;

    for k = 1:N-1
        currT = t(k);
        qd_rad  = [deg2rad(q1_des(k)); deg2rad(q2_des(k))];
        dqd_rad = [deg2rad(q1dot_des(k)); deg2rad(q2dot_des(k))];

        % Effective mass of link 2 changes when carrying obstacle
        isHolding = (currT >= t_pick && currT <= t_drop);
        if isHolding
            m_payload = obstacleMass;
        else
            m_payload = 0;
        end

        m2_eff = m2 + m_payload;
        lc2_eff = (m2 * (L2/2) + m_payload * L2) / max(m2_eff, 0.01);
        I2_eff  = (1/12)*m2*L2^2 + m2*(lc2_eff - L2/2)^2 + m_payload*(L2 - lc2_eff)^2;

        q_sub  = [deg2rad(q1(k));    deg2rad(q2(k))];
        dq_sub = [deg2rad(q1dot(k)); deg2rad(q2dot(k))];

        for s = 1:nSub
            c2 = cos(q_sub(2));
            s2 = sin(q_sub(2));
            a = (1/12)*m1*L1^2 + m1*(L1/2)^2 + I2_eff + m2_eff*(L1^2 + lc2_eff^2 + 2*L1*lc2_eff*c2);
            b = I2_eff + m2_eff*(lc2_eff^2 + L1*lc2_eff*c2);
            d = I2_eff + m2_eff*lc2_eff^2;
            M = [a, b; b, d] + 0.02*eye(2);

            h = -m2_eff * L1 * lc2_eff * s2;
            C = [h*dq_sub(2), h*(dq_sub(1) + dq_sub(2)); -h*dq_sub(1), 0];

            g1 = (m1*(L1/2) + m2_eff*L1)*g*cos(q_sub(1)) + m2_eff*lc2_eff*g*cos(q_sub(1)+q_sub(2));
            g2 = m2_eff*lc2_eff*g*cos(q_sub(1)+q_sub(2));
            G  = [g1; g2];

            % Baseline gravity estimate without knowing payload weight
            g1_nom = (m1*(L1/2) + m2*L1)*g*cos(q_sub(1)) + m2*(L2/2)*g*cos(q_sub(1)+q_sub(2));
            g2_nom = m2*(L2/2)*g*cos(q_sub(1)+q_sub(2));
            G_nom  = [g1_nom; g2_nom];

            % Feedback control law
            e_rad   = qd_rad - q_sub;
            de_rad  = dqd_rad - dq_sub;
            tau_cmd = Kp*e_rad + Kd*de_rad + G_nom;

            % Saturate torques
            tau_cmd = max(min(tau_cmd, [120; 80]), [-120; -80]);

            % Forward dynamics: q_ddot = M \ (tau - C*dq - G)
            qddot = M \ (tau_cmd - C*dq_sub - G);
            qddot = max(min(qddot, 60), -60);

            % Substep integration
            dq_sub = dq_sub + qddot * dtSub;
            q_sub  = q_sub  + dq_sub * dtSub;
        end

        q1(k+1)    = rad2deg(q_sub(1));
        q2(k+1)    = rad2deg(q_sub(2));
        q1dot(k+1) = rad2deg(dq_sub(1));
        q2dot(k+1) = rad2deg(dq_sub(2));
        tau1(k)    = tau_cmd(1);
        tau2(k)    = tau_cmd(2);
    end
    tau1(N) = tau1(N-1);
    tau2(N) = tau2(N-1);

    % Tracking errors
    e1 = q1_des - q1;
    e2 = q2_des - q2;

    % Build output structs
    simData.t         = t;
    simData.L1        = L1;
    simData.L2        = L2;
    simData.q1        = q1;
    simData.q2        = q2;
    simData.q1dot     = q1dot;
    simData.q2dot     = q2dot;
    simData.q1_des    = q1_des;
    simData.q2_des    = q2_des;
    simData.q1dot_des = q1dot_des;
    simData.q2dot_des = q2dot_des;
    simData.e1        = e1;
    simData.e2        = e2;
    simData.tau1      = tau1;
    simData.tau2      = tau2;

    simEvents.t_reach_start = t_reach_start;
    simEvents.t_pick        = t_pick;
    simEvents.t_lift        = t_lift;
    simEvents.t_drop        = t_drop;
    simEvents.t_home        = t_home;
    simEvents.obstacleMass  = obstacleMass;
    simEvents.pickPos       = pickPos;
    simEvents.dropPos       = dropPos;
end

%% =========================================================================
%% EXTRACTOR FOR 3D SIMULATION ROBOT DATA
%% =========================================================================

function [simData, simEvents] = extractFromRobot(robot)
    if isfield(robot, 'armLog') && ~isempty(robot.armLog) && isfield(robot.armLog, 't')
        log = robot.armLog;
        simData.t      = log.t;
        simData.L1     = 0.60;
        simData.L2     = 0.60;
        % Map shoulder pitch to q1, elbow pitch to q2
        simData.q1     = log.q(:, 2);
        simData.q2     = log.q(:, 3);
        simData.q1_des = log.qd(:, 2);
        simData.q2_des = log.qd(:, 3);
        simData.q1dot  = log.qdot(:, 2);
        simData.q2dot  = log.qdot(:, 3);
        simData.e1     = simData.q1_des - simData.q1;
        simData.e2     = simData.q2_des - simData.q2;
        simData.tau1   = log.tau(:, 2);
        simData.tau2   = log.tau(:, 3);
    else
        % Fallback to angleHistory
        T = robot.angleHistoryT;
        A = robot.angleHistory;
        simData.t      = T;
        simData.L1     = 0.60;
        simData.L2     = 0.60;
        simData.q1     = A(:, 2);
        simData.q2     = A(:, 3);
        simData.q1_des = A(:, 2);
        simData.q2_des = A(:, 3);
        simData.q1dot  = gradient(simData.q1, max(diff(T), 0.01));
        simData.q2dot  = gradient(simData.q2, max(diff(T), 0.01));
        simData.e1     = zeros(size(simData.q1));
        simData.e2     = zeros(size(simData.q2));
        simData.tau1   = zeros(size(simData.q1));
        simData.tau2   = zeros(size(simData.q2));
    end

    if isfield(robot, 'pickEvent') && ~isempty(robot.pickEvent)
        simEvents = robot.pickEvent;
        if ~isfield(simEvents, 't_pick') && isfield(simEvents, 't_picked')
            simEvents.t_pick = simEvents.t_picked;
        end
        if ~isfield(simEvents, 't_drop') && isfield(simEvents, 't_dropped')
            simEvents.t_drop = simEvents.t_dropped;
        end
        if ~isfield(simEvents, 'pickPos'); simEvents.pickPos = [0.85, -0.22]; end
        if ~isfield(simEvents, 'dropPos'); simEvents.dropPos = [-0.35, 0.40]; end
        if ~isfield(simEvents, 'obstacleMass'); simEvents.obstacleMass = 8.0; end
    else
        simEvents = estimateEventsFromData(simData);
    end
end

function simEvents = estimateEventsFromData(simData)
    t = simData.t;
    t_end = t(end);
    simEvents.t_reach_start = t_end * 0.10;
    simEvents.t_pick        = t_end * 0.30;
    simEvents.t_lift        = t_end * 0.45;
    simEvents.t_drop        = t_end * 0.75;
    simEvents.t_home        = t_end * 0.95;
    simEvents.obstacleMass  = 8.0;
    simEvents.pickPos       = [0.85, -0.22];
    simEvents.dropPos       = [-0.35, 0.40];
end

%% =========================================================================
%% CONSOLE FORMATTED REPORT
%% =========================================================================

function printSummaryReport(simData, simEvents)
    fprintf('TWO-LINK MANIPULATOR EVENT & DYNAMICS SUMMARY:\n');
    fprintf('------------------------------------------------------------------------\n');
    fprintf('  • Obstacle Mass (Payload):           %.2f kg\n', simEvents.obstacleMass);
    fprintf('  • Obstacle Pick Location (X, Y):     [%.2f m, %.2f m]\n', simEvents.pickPos(1), simEvents.pickPos(2));
    fprintf('  • Collection Bin Location (X, Y):    [%.2f m, %.2f m]\n', simEvents.dropPos(1), simEvents.dropPos(2));
    fprintf('------------------------------------------------------------------------\n');
    fprintf('KEY PHASE EVENT TIMINGS:\n');
    fprintf('  1. Arm Motion Initiated (REACH):     t = %6.2f s\n', simEvents.t_reach_start);
    fprintf('  2. ★ OBSTACLE PICKED & GRASPED:     t = %6.2f s  ◄-- [PAYLOAD ACQUIRED]\n', simEvents.t_pick);
    fprintf('  3. Obstacle Lift Phase:              t = %6.2f s\n', simEvents.t_lift);
    fprintf('  4. ■ OBSTACLE DROPPED IN BIN:        t = %6.2f s  ◄-- [PAYLOAD RELEASED]\n', simEvents.t_drop);
    fprintf('  5. Arm Restored to Home:             t = %6.2f s\n', simEvents.t_home);
    fprintf('------------------------------------------------------------------------\n');
    fprintf('PERFORMANCE METRICS:\n');
    fprintf('  • Peak Shoulder Torque (tau_1):      %6.2f Nm\n', max(abs(simData.tau1)));
    fprintf('  • Peak Elbow Torque (tau_2):         %6.2f Nm\n', max(abs(simData.tau2)));
    fprintf('  • Joint 1 RMS Tracking Error:        %6.3f deg\n', sqrt(mean(simData.e1.^2)));
    fprintf('  • Joint 2 RMS Tracking Error:        %6.3f deg\n', sqrt(mean(simData.e2.^2)));
    fprintf('========================================================================\n\n');
end
