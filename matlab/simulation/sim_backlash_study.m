function R = sim_backlash_study(S, squares, opts)
%SIM_BACKLASH_STUDY Monte Carlo of grasp accuracy with hobby-servo errors.
%   For each square: IK grasp pose -> servo commands -> simulated servo errors -> fingertip
%   error -> success if the cube fits between the jaws and the fingers don't hit the board.
%   Strategies:
%     "A" current firmware: a joint may arrive from either side -> backlash error +/- b
%     "B" same-direction approach: every joint finishes its move from below -> constant +b,
%         removed by calibration; only dead band and 1-degree quantisation remain
%     "C" B plus 0.1-degree commands (writeMicroseconds instead of write)
arguments
    S
    squares (1,:) string
    opts.Backlash (1,5) double = [1.5 1.5 1.5 2.5 2.5]   % half-width, degrees (assumed)
    opts.DeadBand (1,1) double = 0.5                     % +/- degrees where the servo stops short
    opts.Trials (1,1) double = 300
    opts.Seed (1,1) double = 1
end
rng(opts.Seed);
lateralTol = (S.jawOpen - S.cubeSize)/2;      % 4.5 mm
zTol = [-0.008 0.006];                        % fingertip may sit 8 mm lower (board) to 6 mm higher
strategies = ["A" "B" "C"];
R.squares = squares; R.strategies = strategies; R.backlash = opts.Backlash; R.deadBand = opts.DeadBand;
R.success = nan(numel(squares), 3); R.medianErr = nan(numel(squares), 3); R.p95Err = nan(numel(squares), 3);
for i = 1:numel(squares)
    target = sim_square_to_world(S, squares(i)) + [0 0 S.cubeSize/2];
    [q, ok] = sim_solve_pose(S, target);
    if ~ok, continue, end
    qdeg = rad2deg(q);
    for k = 1:3
        nOk = 0; err = zeros(opts.Trials, 1);
        for t = 1:opts.Trials
            switch strategies(k)
                case "A", cmd = round(qdeg); e = sign(rand(1,5) - 0.5) .* opts.Backlash;
                case "B", cmd = round(qdeg); e = zeros(1,5);
                case "C", cmd = round(qdeg*10)/10; e = zeros(1,5);
            end
            qa = deg2rad(cmd + e + opts.DeadBand*(2*rand(1,5) - 1));
            T = getTransform(S.robot, qa, S.tool);
            dp = T(1:3,4)' - target;
            err(t) = norm(dp(1:2));
            nOk = nOk + (err(t) <= lateralTol && dp(3) >= zTol(1) && dp(3) <= zTol(2));
        end
        R.success(i,k) = nOk / opts.Trials;
        R.medianErr(i,k) = 1000*median(err); R.p95Err(i,k) = 1000*prctile(err, 95);
    end
end
end
