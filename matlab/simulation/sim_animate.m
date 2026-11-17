function sim_animate(S, plan, det, camImage, videoFile, opts)
%SIM_ANIMATE Render the pick-and-place plan to a video: 3-D scene + the camera's view.
arguments
    S
    plan
    det
    camImage
    videoFile (1,1) string
    opts.FrameStep (1,1) double = 1      % use every n-th trajectory sample
    opts.FrameRate (1,1) double = 12
    opts.Hold (1,1) double = 14          % extra frames at grasp and release
end
fig = figure("Visible", "off", "Color", "w", "Position", [0 0 1280 720]);
ax = axes(fig, "Position", [0.01 0.02 0.62 0.94]);
hold(ax, "on"); axis(ax, "equal"); axis(ax, "off");
view(ax, 135, 28); camproj(ax, "perspective");
xlim(ax, [-0.45 0.09]); ylim(ax, [-0.2 0.30]); zlim(ax, [0.05 0.40]); camzoom(ax, 1.3);
light(ax, "Position", [1 1 2]); light(ax, "Position", [-1 -1 1], "Color", [0.5 0.5 0.5]);

drawStatic(ax, S);
hCube = drawCube(ax, S, S.cubeStart);
title(ax, "");

% camera view panel
axI = axes(fig, "Position", [0.645 0.30 0.34 0.66]);
J = undistortImage(camImage, S.intrinsics, OutputView="same");
imshow(J, "Parent", axI); hold(axI, "on");
if det.ok
    plot(axI, det.corners(:,1), det.corners(:,2), "g+", "MarkerSize", 4);
    plot(axI, det.redPixel(1), det.redPixel(2), "yo", "MarkerSize", 14, "LineWidth", 2);
end
title(axI, sprintf("Phone camera: object found on %s", det.square), "FontSize", 13, "Color", "k");
txt = annotation(fig, "textbox", [0.645 0.04 0.34 0.22], "String", "", "FontSize", 12, ...
    "EdgeColor", "none", "Interpreter", "none", "VerticalAlignment", "top", "Color", "k");

vw = VideoWriter(videoFile, "MPEG-4"); vw.FrameRate = opts.FrameRate; vw.Quality = 90; open(vw);
idx = 1:opts.FrameStep:size(plan.traj,1);
if idx(end) ~= size(plan.traj,1), idx(end+1) = size(plan.traj,1); end
holdAt = [find(diff(plan.carrying) == 1, 1); find(diff(plan.carrying) == -1, 1)];
first = true;
for i = idx
    q = plan.traj(i,:);
    if first
        show(S.robot, q, "Parent", ax, "PreservePlot", false, "FastUpdate", true, "Frames", "off");
        first = false;
    else
        show(S.robot, q, "Parent", ax, "PreservePlot", false, "FastUpdate", true, "Frames", "off");
    end
    T = getTransform(S.robot, q, S.tool);
    if plan.carrying(i)
        p = T(1:3,4)';
    elseif i > find(plan.carrying, 1, "last")
        p = [S.boxCentre(1:2), S.boxCentre(3) + 0.004 + S.cubeSize/2];   % dropped in the box
    else
        p = S.cubeStart;
    end
    set(hCube, "Matrix", makehgtform("translate", p));
    seg = plan.segment(i);
    stateTxt = sprintf("Step: %s -> %s\nGripper: %s\nTip: (%.0f, %.0f, %.0f) mm", plan.waypoints(seg), ...
        plan.waypoints(seg+1), ternary(plan.carrying(i), "closed (holding cube)", "open"), 1000*T(1:3,4));
    txt.String = stateTxt;
    drawnow limitrate;
    fr = getframe(fig);
    reps = 1 + opts.Hold * ismember(i, holdAt);
    for r = 1:reps, writeVideo(vw, fr); end
end
for r = 1:opts.FrameRate, writeVideo(vw, fr); end
close(vw); close(fig);
end

function drawStatic(ax, S)
tz = S.tableZ;
patch(ax, [-0.6 0.2 0.2 -0.6], [-0.35 -0.35 0.35 0.35], tz*[1 1 1 1], [0.78 0.78 0.80], "EdgeColor", "none");
toW = @(L) (S.boardT * [L, zeros(size(L,1),1), ones(size(L,1),1)]')';
b = 4*S.square + S.frameWidth;
fr = toW([-b -b; b -b; b b; -b b]);
patch(ax, fr(:,1), fr(:,2), fr(:,3) - 0.0005, [0.45 0.28 0.2], "EdgeColor", "none");
for r = 1:8
    for f = 1:8
        x0 = (r-5)*S.square; y0 = (f-5)*S.square;
        q = toW([x0 y0; x0+S.square y0; x0+S.square y0+S.square; x0 y0+S.square]);
        if mod(r+f, 2) == 0, col = [0.22 0.16 0.14]; else, col = [0.88 0.80 0.63]; end
        patch(ax, q(:,1), q(:,2), q(:,3), col, "EdgeColor", "none");
    end
end
% file letters along the near edge
for f = 1:8
    p = toW([-4.6*S.square, (f-4.5)*S.square]);
    text(ax, p(1), p(2), p(3)+0.002, char('a'+f-1), "FontSize", 9, "HorizontalAlignment", "center", "Color", "k");
end
fl = S.fiducialLocal; s = 0.008; fd = toW([fl(1)-s fl(2)-s; fl(1)+s fl(2)-s; fl(1)+s fl(2)+s; fl(1)-s fl(2)+s]);
patch(ax, fd(:,1), fd(:,2), fd(:,3)+0.0005, [0.16 0.67 0.27], "EdgeColor", "none");
% box
c = S.boxCentre; L = S.boxSize(1)/2; W = S.boxSize(2)/2; H = S.boxSize(3);
V = [-L -W 0; L -W 0; L W 0; -L W 0; -L -W H; L -W H; L W H; -L W H] + c;
F = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4];
patch(ax, "Vertices", V, "Faces", F, "FaceColor", [0.62 0.48 0.3], "FaceAlpha", 0.75, "EdgeColor", [0.35 0.25 0.15]);
% camera body and field of view
Tc = S.cameraPose; o = Tc(1:3,4)';
K = S.intrinsics.K; Wd = S.intrinsics.ImageSize(2); Hd = S.intrinsics.ImageSize(1);
cn = [0 0; Wd 0; Wd Hd; 0 Hd];
for k = 1:4
    d = Tc(1:3,1:3) * (K \ [cn(k,:) 1]'); d = d / norm(d);
    e = o + 0.18*d';
    plot3(ax, [o(1) e(1)], [o(2) e(2)], [o(3) e(3)], "-", "Color", [0.2 0.4 0.9]);
end
% (the camera itself sits outside this view; its image is shown on the right)
text(ax, S.boxCentre(1), S.boxCentre(2)+0.06, S.boxCentre(3)+S.boxSize(3), "box", "FontSize", 10, "Color", "k");
end

function h = drawCube(ax, S, p)
h = hgtransform(ax);
s = S.cubeSize/2;
V = [-s -s -s; s -s -s; s s -s; -s s -s; -s -s s; s -s s; s s s; -s s s];
F = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
patch("Parent", h, "Vertices", V, "Faces", F, "FaceColor", [0.85 0.1 0.1], "EdgeColor", [0.4 0 0]);
set(h, "Matrix", makehgtform("translate", p));
end

function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end
