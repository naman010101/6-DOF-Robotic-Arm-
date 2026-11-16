function robot = build_robot_model()
clc
clear
close all

robot = rigidBodyTree("DataFormat","row","MaxNumBodies",6);

%% Dimensions (m)

L_ground = 35e-3;

L1 = 78e-3;

dx23 = 47.5e-3;
dz23 = 195e-3;

L34 = 140e-3;
L45 = 41e-3;
L5T = 95e-3;

%% ============================
% Base Rotation (J1)
%% ============================

body1 = rigidBody("base_link");

joint = rigidBodyJoint("J1","revolute");

joint.JointAxis = [0 0 1];

setFixedTransform(joint,...
    trvec2tform([0 0 L_ground]));

joint.PositionLimits = deg2rad([-90 90]);

body1.Joint = joint;

addBody(robot,body1,robot.BaseName);

%% ============================
% Shoulder (J2)
%% ============================

body2 = rigidBody("link1");

joint = rigidBodyJoint("J2","revolute");

joint.JointAxis = [0 1 0];

setFixedTransform(joint,...
    trvec2tform([0 0 L1]));

joint.PositionLimits = deg2rad([-90 90]);

body2.Joint = joint;

addBody(robot,body2,"base_link");

%% ============================
% Elbow (J3)
%% ============================

body3 = rigidBody("link2");

joint = rigidBodyJoint("J3","revolute");

joint.JointAxis = [0 1 0];

setFixedTransform(joint,...
    trvec2tform([dx23 0 dz23]));

joint.PositionLimits = deg2rad([-90 90]);

body3.Joint = joint;

addBody(robot,body3,"link1");

%% ============================
% Elbow 2 (J4)
%% ============================

body4 = rigidBody("link3");

joint = rigidBodyJoint("J4","revolute");

joint.JointAxis = [0 1 0];

setFixedTransform(joint,...
    trvec2tform([L34 0 0]));

joint.PositionLimits = deg2rad([-90 90]);

body4.Joint = joint;

addBody(robot,body4,"link2");

%% ============================
% Wrist Rotation (J5)
%% ============================

body5 = rigidBody("link4");

joint = rigidBodyJoint("J5","revolute");

joint.JointAxis = [1 0 0];

setFixedTransform(joint,...
    trvec2tform([L45 0 0]));

joint.PositionLimits = deg2rad([-90 90]);

body5.Joint = joint;

addBody(robot,body5,"link3");

%% ============================
% Tool Frame
%% ============================

tool = rigidBody("tool0");

joint = rigidBodyJoint("tcp","fixed");

setFixedTransform(joint,...
    trvec2tform([L5T 0 0]));

tool.Joint = joint;

addBody(robot,tool,"link4");


