%% RUNMLOPTIMIZERANALYSIS - One-Click Launcher for ML & Optimizer Graphs
%
% Visualizes:
%   - Before vs After Gradient Descent optimization parameter changes (Kp, Kd)
%   - Tracking Error reduction under payload disturbance
%   - Loss convergence curve
%   - k-NN Action prediction accuracy
%
% Usage:
%   runMLOptimizerAnalysis

clc;
addpath(genpath(fileparts(mfilename('fullpath'))));

fprintf('\n>>> Launching ML & Gradient Descent Optimizer Performance Visualizer...\n');
fig = plotMLOptimizerAnalysis();
fprintf('>>> Complete! ML Optimizer Analysis figure is now open.\n\n');
