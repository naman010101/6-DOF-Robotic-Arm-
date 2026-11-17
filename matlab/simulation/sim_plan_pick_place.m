function plan = sim_plan_pick_place(S, cubePos, opts)
%SIM_PLAN_PICK_PLACE Joint-space plan: home -> above cube -> grasp -> lift -> above box -> release -> home.
%   cubePos is where vision says the cube is (world, cube centre). Every sample is collision-checked
%   (self, board, box walls; the cube too while it is carried).
arguments
    S
    cubePos (1,3) double
    opts.Approach (1,1) double = 0.06      % m above the grasp point
    opts.StepDeg (1,1) double = 2          % max joint change between samples
end
boxTop = S.boxCentre(3) + S.boxSize(3);
dropTip = [S.boxCentre(1:2), boxTop + 0.015 + S.cubeSize/2];   % cube bottom 15 mm above the rim

names = ["above cube" "grasp" "lift" "above box" "release" "retreat"];
pts = [cubePos + [0 0 opts.Approach]; cubePos; cubePos + [0 0 opts.Approach]; ...
       dropTip + [0 0 0.03]; dropTip; dropTip + [0 0 0.04]];
grip = ["open" "close" "closed" "closed" "open" "open"];   % gripper state on arrival

q = S.home; Q = q; wpName = "home"; ok = true; msgs = strings(0);
for k = 1:size(pts, 1)
    [qk, okk, pe, ae] = sim_solve_pose(S, pts(k,:), q);
    if ~okk
        ok = false; msgs(end+1) = sprintf("%s: IK failed (%.1f mm, %.1f deg)", names(k), 1000*pe, rad2deg(ae)); %#ok<AGROW>
    end
    Q(end+1,:) = qk; wpName(end+1) = names(k); q = qk; %#ok<AGROW>
end
Q(end+1,:) = S.home; wpName(end+1) = "home";

% quintic joint-space segments
traj = zeros(0,5); seg = zeros(0,1); carrying = false(0,1);
carryFrom = find(wpName == "grasp"); carryTo = find(wpName == "release");
for k = 1:size(Q,1)-1
    n = max(8, ceil(max(abs(rad2deg(Q(k+1,:) - Q(k,:)))) / opts.StepDeg));
    s = linspace(0, 1, n)'; s = 10*s.^3 - 15*s.^4 + 6*s.^5;
    Qk = Q(k,:) + s .* (Q(k+1,:) - Q(k,:));
    if k > 1, Qk = Qk(2:end,:); end
    traj = [traj; Qk]; seg = [seg; k*ones(size(Qk,1),1)]; %#ok<AGROW>
    carrying = [carrying; repmat(k >= carryFrom && k < carryTo, size(Qk,1), 1)]; %#ok<AGROW>
end

% collision check along the whole path
nHit = 0; firstHit = "";
cube = collisionBox(S.cubeSize, S.cubeSize, S.cubeSize);
for i = 1:size(traj,1)
    env = S.envCollision;
    % while descending to / leaving the grasp the fingers are meant to be next to the cube,
    % so the board is only checked against the arm links, not the gripper tip region
    hitSelf = checkCollision(S.robot, traj(i,:), "SkippedSelfCollisions", "parent");
    hitEnv = checkCollision(S.robot, traj(i,:), env, "IgnoreSelfCollision", "on");
    hitCube = false;
    if carrying(i)
        T = getTransform(S.robot, traj(i,:), S.tool);
        cube.Pose = trvec2tform(T(1:3,4)');
        for e = 1:numel(env)
            hitCube = hitCube || checkCollision(cube, env{e});
        end
    end
    if hitSelf || hitEnv || hitCube
        nHit = nHit + 1;
        if firstHit == ""
            firstHit = sprintf("sample %d (%s -> %s): self=%d env=%d cube=%d", i, ...
                wpName(seg(i)), wpName(seg(i)+1), hitSelf, hitEnv, hitCube);
        end
    end
end
if nHit > 0
    ok = false; msgs(end+1) = sprintf("%d colliding samples, first at %s", nHit, firstHit);
end

plan = struct("ok", ok, "messages", msgs, "Q", Q, "waypoints", wpName, "grip", ["open" grip "open"], ...
    "traj", traj, "segment", seg, "carrying", carrying, "points", pts, "nCollisions", nHit);
end
