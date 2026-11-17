function I = sim_render_camera(S, cubePos, opts)
%SIM_RENDER_CAMERA Synthetic phone-camera image of the board, cube, box and table.
%   Uses the calibrated intrinsics and lens distortion; polygons are drawn far-to-near.
arguments
    S
    cubePos (1,3) double = S.cubeStart
    opts.Noise (1,1) double = 4        % pixel noise (grey levels)
    opts.Seed (1,1) double = 0
end
E = rigidtform3d(inv(S.cameraPose));                  % world -> camera
H = S.imageSize(1); W = S.imageSize(2);
I = repmat(reshape(uint8([205 200 190]), 1, 1, 3), H, W);   % wall/background

polys = {};   % {worldVertices(Nx3), colour}
% table
tz = S.tableZ;
polys{end+1} = {[-0.9 -0.7 tz; 0.4 -0.7 tz; 0.4 0.7 tz; -0.9 0.7 tz], [60 62 66], 1};
% board frame and squares (board-local -> world)
toW = @(L) (S.boardT * [L, zeros(size(L,1),1), ones(size(L,1),1)]')';
b = 4*S.square + S.frameWidth;
fr = toW([-b -b; b -b; b b; -b b]); polys{end+1} = {fr(:,1:3), [95 60 45], 2};
for r = 1:8
    for f = 1:8
        x0 = (r-5)*S.square; y0 = (f-5)*S.square;
        q = toW([x0 y0; x0+S.square y0; x0+S.square y0+S.square; x0 y0+S.square]);
        if mod(r+f, 2) == 0, col = [55 40 35]; else, col = [225 205 160]; end   % a1 is dark
        polys{end+1} = {q(:,1:3), col, 3}; %#ok<AGROW>
    end
end
% green fiducial on the frame beside rank 8 (resolves the board's orientation ambiguity)
fl = S.fiducialLocal; s = S.frameWidth*0.35;
fid = toW([fl(1)-s fl(2)-s; fl(1)+s fl(2)-s; fl(1)+s fl(2)+s; fl(1)-s fl(2)+s]); polys{end+1} = {fid(:,1:3), [40 170 70], 4};
% box: walls as quads
c = S.boxCentre; L = S.boxSize(1)/2; Wd = S.boxSize(2)/2; Hh = S.boxSize(3);
faces = {[-L -Wd 0; L -Wd 0; L -Wd Hh; -L -Wd Hh], [-L Wd 0; L Wd 0; L Wd Hh; -L Wd Hh], ...
         [-L -Wd 0; -L Wd 0; -L Wd Hh; -L -Wd Hh], [L -Wd 0; L Wd 0; L Wd Hh; L -Wd Hh]};
for k = 1:4, polys{end+1} = {faces{k} + c, [150 120 80], 5}; end %#ok<AGROW>
% cube: five visible-candidate faces
h = S.cubeSize/2;
cf = {[-h -h h; h -h h; h h h; -h h h], [-h -h -h; h -h -h; h -h h; -h -h h], [-h h -h; h h -h; h h h; -h h h], ...
      [-h -h -h; -h h -h; -h h h; -h -h h], [h -h -h; h h -h; h h h; h -h h]};
shade = [215 30 30; 170 20 20; 170 20 20; 150 15 15; 150 15 15];
for k = 1:numel(cf), polys{end+1} = {cf{k} + cubePos, shade(k,:), 5}; end %#ok<AGROW>

% draw flat layers in order (table, frame, squares, fiducial), then the 3-D objects
% far-to-near (painter's algorithm); the cube's hidden faces are drawn first and covered
layer = cellfun(@(p) p{3}, polys);
dist = cellfun(@(p) norm(mean(p{1},1) - S.cameraPosition), polys);
[~, order] = sortrows([layer(:), -dist(:)]);
order = order';
for k = order
    Pc = transformPointsForward(E, polys{k}{1});
    % keep only the part inside the field of view (+20 % margin): the distortion model
    % folds points far outside the image back into it
    fx = S.intrinsics.FocalLength(1); fy = S.intrinsics.FocalLength(2); pp = S.intrinsics.PrincipalPoint;
    ax = 1.2*max(pp(1), W - pp(1))/fx; ay = 1.2*max(pp(2), H - pp(2))/fy;
    for n = {[0 0 1 -0.02], [-1 0 ax 0], [1 0 ax 0], [0 -1 ay 0], [0 1 ay 0]}
        Pc = clipPlane(Pc, n{1});
        if size(Pc,1) < 3, break, end
    end
    if size(Pc,1) < 3, continue, end
    V = transformPointsInverse(E, densify(Pc));
    uv = world2img(V, E, S.intrinsics, ApplyDistortion=true);
    mask = poly2mask(uv(:,1), uv(:,2), H, W);
    for ch = 1:3
        plane = I(:,:,ch); plane(mask) = polys{k}{2}(ch); I(:,:,ch) = plane;
    end
end
rng(opts.Seed);
I = imgaussfilt(I, 0.8);
I = uint8(double(I) + opts.Noise*randn(size(I)));
end

function out = clipPlane(P, n)
% Sutherland-Hodgman clip of a polygon (camera coordinates), keeping n(1:3)*p + n(4) >= 0
out = zeros(0,3); k = size(P,1);
f = @(p) n(1:3)*p' + n(4);
for i = 1:k
    a = P(i,:); b = P(mod(i,k)+1,:);
    fa = f(a); fb = f(b);
    if fa >= 0, out(end+1,:) = a; end %#ok<AGROW>
    if (fa >= 0) ~= (fb >= 0)
        out(end+1,:) = a + fa/(fa - fb)*(b - a); %#ok<AGROW>
    end
end
end

function V = densify(V)
% sample polygon edges so lens distortion bends them correctly
out = zeros(0,3);
for i = 1:size(V,1)
    a = V(i,:); b = V(mod(i, size(V,1)) + 1, :);
    t = linspace(0, 1, 20)'; out = [out; a + t(1:end-1).*(b - a)]; %#ok<AGROW>
end
V = out;
end
