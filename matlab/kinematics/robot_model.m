clc;
clear;
close all;

%% Create Robot
robot = rigidBodyTree('DataFormat','column','MaxNumBodies',6);

%% Dimensions (meters)

L0 = 0.050578;   % Ground -> J1
L1 = 0.065984;   % J1 -> J2
L2 = 0.200117;
L3 = 0.140254;
L4 = 0.060171;
L5 = 0.061917;

body1 = rigidBody('link1');
joint1 = rigidBodyJoint('j1','revolute');

setFixedTransform(joint1,trvec2tform([0 0 L0]));

joint1.JointAxis = [0 0 1];

joint1.PositionLimits = deg2rad([-85 95]);

body1.Joint = joint1;

addBody(robot,body1,'base');


body2 = rigidBody('link2');
joint2 = rigidBodyJoint('j2','revolute');

setFixedTransform(joint2,trvec2tform([0 0 L1]));

joint2.JointAxis = [0 0 1];

joint2.PositionLimits = deg2rad([-90 90]);

body2.Joint = joint2;

addBody(robot,body2,'link1');


body3 = rigidBody('link3');
joint3 = rigidBodyJoint('j3','revolute');

setFixedTransform(joint3,trvec2tform([0 0 L2]));

joint3.JointAxis = [0 0 1];

joint3.PositionLimits = deg2rad([-90 90]);

body3.Joint = joint3;

addBody(robot,body3,'link2');


body4 = rigidBody('link4');
joint4 = rigidBodyJoint('j4','revolute');

setFixedTransform(joint4,trvec2tform([L3 0 0]));

joint4.JointAxis = [0 0 1];

joint4.PositionLimits = deg2rad([-90 90]);

body4.Joint = joint4;

addBody(robot,body4,'link3');


body5 = rigidBody('gripper_mount');
joint5 = rigidBodyJoint('j5','revolute');

setFixedTransform(joint5,trvec2tform([L4 0 0]));

joint5.JointAxis = [0 0 1];

joint5.PositionLimits = deg2rad([-90 90]);

body5.Joint = joint5;

addBody(robot,body5,'link4');


tool = rigidBody('tool');

fixJoint = rigidBodyJoint('fix1','fixed');

setFixedTransform(fixJoint,...
    trvec2tform([L5 0 0]));

tool.Joint = fixJoint;

addBody(robot,tool,'gripper_mount');


figure

show(robot);

axis equal

view(135,20)

showdetails(robot)


figure

config = homeConfiguration(robot);

show(robot,config);

axis equal
view(135,20)

config = homeConfiguration(robot);

config(1) = deg2rad(45);   % column data format: plain vector, not a struct

figure
show(robot,config);
axis equal


ik = inverseKinematics('RigidBodyTree',robot);

weights = [0.25 0.25 0.25 1 1 1];

initialGuess = homeConfiguration(robot);

targetPos = [0.20 0.10 0.25];

targetPose = trvec2tform(targetPos);

%% Solve IK for the target position (orientation weighted low)
endEffector = robot.BodyNames{end};
[configSol,solInfo] = ik(endEffector,targetPose,weights,initialGuess);

reached = tform2trvec(getTransform(robot,configSol,endEffector));
fprintf("IK status: %s\n",solInfo.Status);
fprintf("Target  : [%.3f %.3f %.3f] m\n",targetPos);
fprintf("Reached : [%.3f %.3f %.3f] m  (error %.2f mm)\n",reached,1000*norm(reached-targetPos));
fprintf("Joints  : %s deg\n",mat2str(round(rad2deg(configSol(:)')),4));

figure
show(robot,configSol);
hold on
plot3(targetPos(1),targetPos(2),targetPos(3),'r*','MarkerSize',12);
axis equal
title('IK solution')
