% Run once per MATLAB session so scripts in matlab/ subfolders can call each other
% (e.g. hardware/arm_control_app.m uses kinematics/build_robot_model and chess/Create*).
addpath(genpath(fileparts(mfilename('fullpath'))));
