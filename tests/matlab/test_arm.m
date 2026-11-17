function tests = test_arm
%TEST_ARM Hardware-free tests for the 6-DOF arm MATLAB code.
%   From the repo root:  results = runtests("tests/matlab/test_arm.m")
tests = functiontests(localfunctions);
end

function setupOnce(tc)
root = fileparts(fileparts(fileparts(mfilename("fullpath"))));
tc.TestData.root = root;
tc.TestData.oldPath = addpath(genpath(fullfile(root, "matlab")));
tc.TestData.oldVis = get(0, "DefaultFigureVisible");
set(0, "DefaultFigureVisible", "off");
tc.TestData.robot = build_robot_model();
tc.TestData.urdf = fullfile(root, "urdf", "urdf", "robot.urdf");
end

function teardownOnce(tc)
close all force
set(0, "DefaultFigureVisible", tc.TestData.oldVis);
path(tc.TestData.oldPath);
end

% ---------------------------------------------------------------- models
function testUrdfLoadsWithMeshes(tc)
u = importrobot(tc.TestData.urdf);
tc.verifyEqual(u.NumBodies, 6);
tc.verifyEqual(numel(homeConfiguration(u)), 5, "5 movable joints + fixed gripper");
txt = fileread(tc.TestData.urdf);
meshes = unique(string(regexp(txt, '(?<=meshes/)[^"]+\.stl', "match")));
for m = meshes
    tc.verifyTrue(isfile(fullfile(tc.TestData.root, "urdf", "meshes", m)), "missing mesh " + m);
end
end

function testBuildRobotModel(tc)
r = tc.TestData.robot;
tc.verifyEqual(numel(homeConfiguration(r)), 5);
tc.verifyTrue(any(r.BodyNames == "tool0"));
end

% ---------------------------------------------------------------- IK
function testIkOnRandomReachableTargets(tc)
r = tc.TestData.robot;
rng(1);
n = 12; err = zeros(n, 1);
for k = 1:n
    qTrue = randomConfiguration(r);
    target = tform2trvec(getTransform(r, qTrue, "tool0"));
    [~, err(k)] = solve_ik_dls(r, target, appSeed(r), "tool0");
end
report("IK random targets: median %.2f mm, worst %.2f mm, %d/%d under 1 mm\n", ...
    1000*median(err), 1000*max(err), sum(err < 1e-3), n);
% Random poses include elbow-down shapes the solver cannot reach from the app's fixed
% starting guess, so this is a coverage measurement rather than a strict pass/fail.
tc.verifyGreaterThanOrEqual(mean(err < 1e-3), 0.5, "at least half of random reachable targets within 1 mm");
end

function testIkReachesEveryChessSquare(tc)
chessIk(tc, 0, "on the board");
end

function testIkReachesEveryChessSquareFromAbove(tc)
chessIk(tc, 0.03, "30 mm above the board (approach height)");
end

function chessIk(tc, hover, label)
r = tc.TestData.robot;
Sq = CreateChessBoard();
names = string(fieldnames(Sq));
err = zeros(numel(names), 1);
for i = 1:numel(names)
    [~, err(i)] = solve_ik_dls(r, Sq.(names(i)) + [0 0 hover], appSeed(r), "tool0");
end
bad = names(err > 2e-3);
badTxt = "none"; if ~isempty(bad), badTxt = strjoin(bad, " "); end
report("IK chess squares %s: worst %.2f mm; >2 mm: %s\n", label, 1000*max(err), badTxt);
tc.verifyEmpty(bad, "squares the IK cannot reach within 2 mm");
end

function testServoMapping(tc)
tc.verifyEqual(joints_to_servo(deg2rad([-90 0 90 -45 30])), [0 90 180 45 120]);
tc.verifyEqual(joints_to_servo(deg2rad([-120 200 0 0 0])), [0 180 90 90 90], "clamped to 0..180");
end

% ---------------------------------------------------------------- chessboard
function testChessBoardGeometry(tc)
Sq = CreateChessBoard();
tc.verifyEqual(numel(fieldnames(Sq)), 64);
tc.verifyEqual(norm(Sq.b1 - Sq.a1), 0.033, "AbsTol", 1e-9);
tc.verifyEqual(norm(Sq.a2 - Sq.a1), 0.033, "AbsTol", 1e-9);
c = mean(cell2mat(struct2cell(Sq)), 1);
tc.verifyEqual(c(1:2), [0.125 0], "AbsTol", 1e-9, "board centred at x=125 mm");
end

function testServoTableCoversAllSquares(tc)
F = CreateServoInterpolation();
[f, rk] = meshgrid(1:8, 1:8);
measured = f(:) <= 4;   % only files a-d are in the measured table
nOut = 0;
for s = ["S1" "S2" "S3" "S4" "S5"]
    v = F.(s)(f(:), rk(:));
    tc.verifyTrue(all(isfinite(v)), s + " has NaN squares");
    tc.verifyTrue(all(v(measured) >= 0 & v(measured) <= 180), s + " outside 0..180 on a measured square");
    nOut = nOut + sum(v(~measured) < 0 | v(~measured) > 180);
end
report("servo table: %d extrapolated values (files e-h) outside 0..180, clamped by the app\n", nOut);
tc.verifyEqual(round([F.S1(1,1) F.S2(1,1) F.S3(1,1) F.S4(1,1) F.S5(1,1)]), [31 88 25 110 150], ...
    "measured square a1 reproduced exactly");
end

function testBoardToRobot(tc)
tc.verifyEqual(BoardToRobot([0.01; 0.02; 0.003]), [-0.07; 0.01; 0.003], "AbsTol", 1e-12);
end

function testGenerateRobotSquaresScript(tc)
% run() switches to the script's folder, so run a copy in a temp folder (keeps the repo clean)
d = tempname; mkdir(d);
copyfile(fullfile(tc.TestData.root, "matlab", "chess", "generateRobotSquares.m"), d);
runScript(fullfile(d, "generateRobotSquares.m"));
tc.verifyTrue(isfile(fullfile(d, "robotSquares.mat")));
S = load(fullfile(d, "robotSquares.mat")).Squares;
tc.verifyEqual(numel(fieldnames(S)), 64);
end

% ---------------------------------------------------------------- scripts run headless
function testCollisionCheckScript(tc)
out = runScript(fullfile(tc.TestData.root, "matlab", "simulation", "collision_check.m"));
tc.verifySubstring(out, "COLLISION OBJECTS");
lines = strtrim(splitlines(out));
keep = lines(~cellfun(@isempty, regexp(lines, "^(No |SELF|GROUND|Minimum|Configuration)", "once")));
report("collision_check: %s\n", strjoin(keep, " | "));
end

function testRobotModelScript(tc)
out = runScript(fullfile(tc.TestData.root, "matlab", "kinematics", "robot_model.m"));
tc.verifyNotEmpty(out);
end

% ---------------------------------------------------------------- serial packet format
function testFeedbackParsingMatchesFirmware(tc)
% moving_physical_arm.m parses lines like this; status lines start with '#'
line = "89,64,103,89,62,85";
a = str2double(split(line, ","));
tc.verifyEqual(numel(a), 6);
tc.verifyTrue(all(isfinite(a)));
tc.verifyTrue(any(isnan(str2double(split("# OK 1,2,3,4,5", ",")))), "status lines rejected");
end

% ---------------------------------------------------------------- gripper / pick and place
function testPickAndPlaceSequence(tc)
esp = FakeEsp32();
evalc("pick_and_place(esp, [31 88 25 110 150], [150 70 60 120 90])");
tc.verifyEqual(esp.Sent, ["OPEN" "31,88,25,110,150" "CLOSE" "HOME" "150,70,60,120,90" "OPEN" "HOME"], ...
    "open, pick, close, lift, place, open, home");
end

function testPickAndPlaceStopsOnError(tc)
esp = FakeEsp32();
esp.FailOn = "CLOSE";
tc.verifyError(@() pick_and_place(esp, [31 88 25 110 150], [150 70 60 120 90]), "pick_and_place:esp32");
tc.verifyEqual(esp.Sent, ["OPEN" "31,88,25,110,150" "CLOSE"], "stops at the rejected step");
end

function testAppBuildsWithoutHardware(tc)
before = findall(groot, "Type", "figure");
evalc("arm_control_app()");   % no ESP32 on COM9 -> runs in offline mode
figs = setdiff(findall(groot, "Type", "figure"), before);
c = onCleanup(@() delete(figs));
tc.verifyNotEmpty(figs, "app window created");
texts = string(get(findall(figs, "Type", "uibutton"), "Text"));
for t = ["Open Gripper" "Close Gripper" "Pick Square -> Box" "Load Square" "Solve IK"]
    tc.verifyTrue(any(texts == t), "missing button: " + t);
end
end

% ---------------------------------------------------------------- pick-and-place simulation
function testSimVisionFindsSquares(tc)
warning("off", "vision:calibrate:boardShouldBeAsymmetric");
for sq = ["a1" "d8" "b3" "h5"]
    S = sim_build_scene(Square=sq);
    det = sim_detect_object(S, sim_render_camera(S, Seed=7));
    tc.verifyEqual(det.square, sq, "camera found the wrong square");
    tc.verifyLessThan(norm(det.world(1:2) - S.cubeStart(1:2)), 0.003, "position error over 3 mm on " + sq);
end
end

function testSimReachMatchesMeasuredHalf(tc)
% pickable = fingertip can reach the cube AND the approach point 60 mm above it
S = sim_build_scene();
pickable = @(sq) pickOk(S, sq);
for sq = ["a1" "a8" "d1" "d8" "c5"]      % the half the real arm was calibrated on
    tc.verifyTrue(pickable(sq), sq + " should be pickable");
end
for sq = ["g4" "h1" "h8"]
    tc.verifyFalse(pickable(sq), sq + " should be out of reach");
end
end

function ok = pickOk(S, sq)
p = sim_square_to_world(S, sq) + [0 0 S.cubeSize/2];
[q, ok1] = sim_solve_pose(S, p);
[~, ok2] = sim_solve_pose(S, p + [0 0 0.06], q);
ok = ok1 && ok2;
end

function testSimPlanIsCollisionFree(tc)
S = sim_build_scene(Square="b3");
plan = sim_plan_pick_place(S, S.cubeStart);
tc.verifyTrue(plan.ok, strjoin(plan.messages, "; "));
tc.verifyEqual(plan.nCollisions, 0);
tc.verifyEqual(plan.waypoints, ["home" "above cube" "grasp" "lift" "above box" "release" "retreat" "home"]);
end

function testSimBacklashStrategies(tc)
S = sim_build_scene();
R = sim_backlash_study(S, ["b3" "c6"], Trials=150, Seed=3);
tc.verifyLessThan(mean(R.success(:,1)), 0.5, "uncompensated backlash should usually miss");
tc.verifyGreaterThan(mean(R.success(:,2)), 0.9, "same-direction approach should usually grasp");
end

% ---------------------------------------------------------------- helpers
function q = appSeed(robot)
% same starting guess arm_control_app uses for its first solve
q = homeConfiguration(robot);
q(2) = deg2rad(-20); q(3) = deg2rad(30); q(4) = deg2rad(10);
end

function out = runScript(file) %#ok<INUSD> (used inside the evalc string)
% scripts call clc/clear, so run them in this function's own workspace
out = evalc("run(file)");
end

function report(varargin)
% print a result line; also append it to $ARM_TEST_LOG when run_tests.m sets it
msg = sprintf(varargin{:});
fprintf("\n  %s", msg);
f = getenv("ARM_TEST_LOG");
if ~isempty(f)
    fid = fopen(f, "a"); fprintf(fid, "  %s", msg); fclose(fid);
end
end
