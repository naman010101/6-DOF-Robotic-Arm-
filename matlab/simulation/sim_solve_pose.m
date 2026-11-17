function [q, ok, posErr, angErr] = sim_solve_pose(S, p, qGuess)
%SIM_SOLVE_POSE IK for the fingertip at world point p with the gripper pointing straight down.
%   Yaw follows the base angle (the arm has 5 joints, so roll about the tool axis is fixed).
persistent ik robotId
if isempty(ik) || ~isequal(robotId, S.robot)
    ik = inverseKinematics("RigidBodyTree", S.robot, "SolverAlgorithm", "BFGSGradientProjection");
    ik.SolverParameters.MaxIterations = 400;
    robotId = S.robot;
end
if nargin < 3, qGuess = S.home; end
phi = atan2(p(2) - S.baseAxis(2), p(1) - S.baseAxis(1));
yaw = wrapToPi(phi - pi);                         % the arm reaches along -X at q = 0
R = axang2rotm([0 0 1 yaw]) * axang2rotm([0 1 0 -pi/2]);   % gripper -X axis -> world -Z
Tgoal = [R p(:); 0 0 0 1];
bestErr = inf;
guesses = [qGuess; S.home; 0 -0.6 1.2 1.0 0; 0 -0.3 1.5 0.4 0];
for g = 1:size(guesses, 1)
    g0 = guesses(g, :); g0(1) = max(min(yaw, pi/2), -pi/2);
    [qq, info] = ik(S.tool, Tgoal, [1 1 1 1 1 1], g0);
    T = getTransform(S.robot, qq, S.tool);
    pe = norm(T(1:3,4) - p(:));
    ae = acos(max(-1, min(1, dot(-T(1:3,1), [0 0 -1]))));   % tool axis vs straight down
    e = pe + 0.05*ae;
    if e < bestErr
        bestErr = e; q = qq; posErr = pe; angErr = ae;
    end
    if pe < 1e-3 && ae < deg2rad(2), break, end
    if info.ExitFlag == 1 && pe < 1e-3, break, end
end
ok = posErr < 2e-3 && angErr < deg2rad(5);
end
