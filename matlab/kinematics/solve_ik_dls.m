function [q, err, iters] = solve_ik_dls(robot, target, q0, endEffector, opts)
%SOLVE_IK_DLS Position-only damped-least-squares IK (the solver used by arm_control_app).
%   [q, err, iters] = solve_ik_dls(robot, target, q0, endEffector)
%   target : 1x3 position [m] in the robot base frame
%   q0     : 1xN starting configuration (row format)
%   err    : final position error [m]

arguments
    robot
    target (1,3) double
    q0 (1,:) double
    endEffector (1,1) string = "tool0"
    opts.Lambda (1,1) double = 0.1
    opts.Alpha (1,1) double = 1.0
    opts.Tol (1,1) double = 1e-4
    opts.MaxIter (1,1) double = 300
end

q = q0;
iters = opts.MaxIter;

for k = 1:opts.MaxIter
    p = tform2trvec(getTransform(robot, q, endEffector));
    e = target - p;

    if norm(e) < opts.Tol
        iters = k;
        break
    end

    J = geometricJacobian(robot, q, endEffector);
    Jv = J(4:6, :);
    dq = opts.Alpha * Jv' * ((Jv*Jv' + opts.Lambda^2*eye(3)) \ e');
    q = q + dq';

    for i = 1:numel(q)
        lim = robot.Bodies{i}.Joint.PositionLimits;
        q(i) = max(min(q(i), lim(2)), lim(1));
    end
end

err = norm(target - tform2trvec(getTransform(robot, q, endEffector)));
end
