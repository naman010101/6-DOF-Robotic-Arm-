clc;
clear;
close all;

%% Import Robot
robot = importrobot(fullfile(fileparts(mfilename("fullpath")),"..","..","urdf","urdf","robot.urdf"));
robot.DataFormat = "row";

%% Create Ground Plane
ground = collisionBox(1.5,1.5,0.02);
ground.Pose = trvec2tform([0 0 -0.01]);

%% Check Collision Objects
fprintf('\n=== COLLISION OBJECTS ===\n');

for i = 1:robot.NumBodies
    body = robot.Bodies{i};

    fprintf('%s : %d collision objects\n', ...
        body.Name, ...
        numel(body.Collisions));
end

%% Home Configuration
config = homeConfiguration(robot);

%% Display Robot
figure;
show(robot,config,'Collisions','on');
hold on;
show(ground);

title('Home Configuration');
axis equal;
view(135,25);

%% Self Collision Check
[selfCollision,selfSepDist] = checkCollision(robot,config,"SkippedSelfCollisions","parent");

fprintf('\n=== SELF COLLISION (HOME) ===\n');

if selfCollision
    disp('SELF COLLISION DETECTED');
else
    disp('No self collision');
end

%% Ground Collision Check
[groundCollision,groundSepDist] = checkCollision(robot,config,{ground},"SkippedSelfCollisions","parent");

fprintf('\n=== GROUND COLLISION (HOME) ===\n');

if groundCollision
    disp('GROUND COLLISION DETECTED');
else
    disp('No ground collision');
end

%% Minimum Distances
fprintf('\n=== MINIMUM DISTANCES ===\n');

d1 = selfSepDist(isfinite(selfSepDist));

if ~isempty(d1)
    fprintf('Minimum self distance = %.5f m\n',min(d1));
end

d2 = groundSepDist(isfinite(groundSepDist));

if ~isempty(d2)
    fprintf('Minimum ground distance = %.5f m\n',min(d2));
end

%% Random Collision Search
fprintf('\n=== RANDOM CONFIGURATION TEST ===\n');

collisionFound = false;

for k = 1:5000

    q = randomConfiguration(robot);

    % Self collision
    [selfCol,~] = checkCollision(robot,q,"SkippedSelfCollisions","parent");

    if selfCol

        fprintf('\nSELF COLLISION FOUND\n');
        fprintf('Configuration number = %d\n',k);

        figure;
        show(robot,q,'Collisions','on');
        hold on;
        show(ground);

        title('Self Collision Found');
        axis equal;

        collisionFound = true;
        break;
    end

    % Ground collision
    [groundCol,~] = checkCollision(robot,q,{ground},"SkippedSelfCollisions","parent");

    if groundCol

        fprintf('\nGROUND COLLISION FOUND\n');
        fprintf('Configuration number = %d\n',k);

        figure;
        show(robot,q,'Collisions','on');
        hold on;
        show(ground);

        title('Ground Collision Found');
        axis equal;

        collisionFound = true;
        break;
    end

end

if ~collisionFound
    fprintf('\nNo collisions found in 5000 random configurations.\n');
end