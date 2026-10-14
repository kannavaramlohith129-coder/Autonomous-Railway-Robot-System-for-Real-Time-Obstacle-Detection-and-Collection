%% RUNTWOLINKARMANALYSIS - One-Click Launcher for Two-Link Manipulator Figures
%
% Generates the complete 6-figure suite matching MATLAB robotics standards:
%   - Figure 2: Joint 1 Error  (with pick timestamp event marker)
%   - Figure 3: Joint 2 Error  (with pick timestamp event marker)
%   - Figure 4: Tracking Errors (e1, e2, and ||e|| with phase regions)
%   - Figure 5: Control Torques (tau_1, tau_2 showing dynamic load jump at pick)
%   - Figure 6: Phase Plane    (q vs qdot orbital phase portraits)
%   - Figure 7: Two-Link Robot Animation (interactive 2-link motion canvas)
%
% All figures dock automatically as tabs across the top of the MATLAB figure container.
%
% Usage:
%   runTwoLinkArmAnalysis

clc;
addpath(genpath(fileparts(mfilename('fullpath'))));

fprintf('\n>>> Launching Two-Link Manipulator Movement & Dynamics Analysis...\n');
handles = plotTwoLinkManipulatorMovement('Dock', true, 'Animate', true);

fprintf('>>> Complete! All 6 docked tabs (Figures 2 to 7) are now open in your MATLAB window.\n\n');
