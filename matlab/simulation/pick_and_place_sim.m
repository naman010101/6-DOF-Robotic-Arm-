function result = pick_and_place_sim(square, opts)
%PICK_AND_PLACE_SIM One-button pick-and-place in simulation: camera -> square -> pick -> box.
%   result = pick_and_place_sim("b3")                 run and print the result
%   result = pick_and_place_sim("b3", Video="out.mp4") also render the animation
%
%   1. render the phone camera's view (real calibration, lens distortion) with the cube on `square`
%   2. vision finds the board and the cube -> square + position (sim_detect_object)
%   3. IK + quintic joint trajectory: above cube -> grasp -> lift -> above box -> release -> home
%   4. collision check along the whole path (self, board, box, carried cube)
%   See docs/simulation-plan.md.
arguments
    square (1,1) string = "b3"
    opts.Video (1,1) string = ""
    opts.FrameStep (1,1) double = 2
    opts.CameraImage (1,1) string = ""
end
warning("off", "vision:calibrate:boardShouldBeAsymmetric");
S = sim_build_scene(Square=square);
I = sim_render_camera(S);
if opts.CameraImage ~= "", imwrite(I, opts.CameraImage); end
det = sim_detect_object(S, I);
result = struct("square", square, "detected", det.square, "visionError_mm", NaN, ...
    "planned", false, "collisions", NaN, "messages", strings(0), "servoAngles", []);
if ~det.ok
    result.messages = "vision: " + det.msg; disp(result); return
end
result.visionError_mm = 1000*norm(det.world(1:2) - S.cubeStart(1:2));
plan = sim_plan_pick_place(S, det.world);
result.planned = plan.ok; result.collisions = plan.nCollisions; result.messages = plan.messages;
% servo commands the firmware would receive (S = joint angle + 90, whole degrees)
result.servoAngles = array2table(round(rad2deg(plan.Q) + 90), "RowNames", ...
    matlab.lang.makeUniqueStrings(cellstr(plan.waypoints)), ...
    "VariableNames", ["S1" "S2" "S3" "S4" "S5"]);
if opts.Video ~= ""
    sim_animate(S, plan, det, I, opts.Video, FrameStep=opts.FrameStep);
end
fprintf("cube on %s -> camera found %s (%.1f mm) -> plan %s, %d colliding samples\n", ...
    square, det.square, result.visionError_mm, string(plan.ok), plan.nCollisions);
end
