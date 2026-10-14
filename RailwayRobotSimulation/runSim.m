% RUNSIM.M - Always run this to launch the simulation.
% Clears MATLAB compiled-function cache so all file edits take effect.
%
%   >> runSim
%
clc; close all force;
clear functions;
clear classes;
rootDir = fileparts(mfilename('fullpath'));
addpath(genpath(rootDir));
rehash toolboxcache;
fprintf('\n========================================\n');
fprintf('  RAILWAY ROBOT SIMULATION - LAUNCHING  \n');
fprintf('  All functions reloaded from disk.     \n');
fprintf('========================================\n\n');
main();
