function export_packets()
%EXPORT_PACKETS Write the serial packets MATLAB would send for every chess square to
%tests/firmware_sim/packets.txt, so the firmware simulation can check it accepts them all.
%   Two packets per square: from the measured servo table (as arm_control_app's square
%   dropdown sends) and from IK on the square's position (as its X/Y/Z solve sends).
here = fileparts(mfilename("fullpath"));
root = fileparts(fileparts(here));
old = addpath(genpath(fullfile(root, "matlab"))); c = onCleanup(@() path(old));

robot = build_robot_model();
F = CreateServoInterpolation();
Sq = CreateChessBoard();
seed = homeConfiguration(robot); seed(2:4) = deg2rad([-20 30 10]);

fid = fopen(fullfile(root, "tests", "firmware_sim", "packets.txt"), "w");
files = 'abcdefgh';
for f = 1:8
    for rk = 1:8
        S = round([F.S1(f,rk) F.S2(f,rk) F.S3(f,rk) F.S4(f,rk) F.S5(f,rk)]);
        S = max(0, min(180, S));
        fprintf(fid, "%d,%d,%d,%d,%d\n", S);

        q = solve_ik_dls(robot, Sq.(sprintf('%c%d', files(f), rk)), seed, "tool0");
        fprintf(fid, "%d,%d,%d,%d,%d\n", joints_to_servo(q));
    end
end
fclose(fid);
fprintf("wrote 128 packets\n");
end
