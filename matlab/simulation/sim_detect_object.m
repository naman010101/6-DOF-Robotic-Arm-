function det = sim_detect_object(S, I)
%SIM_DETECT_OBJECT Find which square the red cube is on, from the camera image alone.
%   1. undistort with the calibrated intrinsics
%   2. find the 7x7 inner corners -> board pose (estimateExtrinsics)
%   3. resolve the board's orientation with the green fiducial beside rank 8
%   4. intersect the ray through the red blob with the plane at half the cube height
%   Board -> robot uses the fixed, measured board placement (S.boardT), as on the real arm.
det = struct("ok", false, "square", "", "boardXY", [NaN NaN], "world", [NaN NaN NaN], "msg", "");
J = undistortImage(I, S.intrinsics, OutputView="same");
K = S.intrinsics.K;

[pts, sz] = detectCheckerboardPoints(rgb2gray(J), PartialDetections=false);
if ~isequal(sz, [8 8])
    det.msg = "checkerboard not found"; return
end
patternPts = patternWorldPoints("checkerboard", sz, S.square);
noDist = cameraIntrinsics(S.intrinsics.FocalLength, S.intrinsics.PrincipalPoint, S.intrinsics.ImageSize);
ext = estimateExtrinsics(pts, patternPts, noDist);
R = ext.R; t = ext.Translation(:);          % pattern -> camera: Xc = R*Xp + t
C = -R' * t;                                 % camera centre in pattern coords

% blobs (in the undistorted image)
hsv = rgb2hsv(J);
red = (hsv(:,:,1) < 0.04 | hsv(:,:,1) > 0.96) & hsv(:,:,2) > 0.6 & hsv(:,:,3) > 0.35;
green = hsv(:,:,1) > 0.25 & hsv(:,:,1) < 0.45 & hsv(:,:,2) > 0.5 & hsv(:,:,3) > 0.35;
redC = blobCentre(red); greenC = blobCentre(green);
if any(isnan(redC)), det.msg = "object not found"; return, end
if any(isnan(greenC)), det.msg = "fiducial not found"; return, end

toPlane = @(uv, hgt) rayToPlane(uv, hgt, R, C, K);
fidP = toPlane(greenC, 0);
objP = toPlane(redC, S.cubeSize/2);

% pattern (X,Y) -> board-local (x = ranks, y = files): 8 candidate corner/axis assignments;
% keep the one that puts the fiducial at its known place
fidLocal = S.fiducialLocal;
best = inf;
for ox = [-3 3]*S.square
    for oy = [-3 3]*S.square
        for swap = [false true]
            M = @(P) mapPattern(P, ox, oy, swap);
            e = norm(M(fidP(1:2)) - fidLocal);
            if e < best, best = e; map = M; end
        end
    end
end
if best > 0.02, det.msg = sprintf("fiducial %.0f mm from where it should be", 1000*best); return, end

local = map(objP(1:2));
w = S.boardT * [local(:); S.cubeSize/2; 1];
det.boardXY = local;
det.world = w(1:3)';
det.square = sim_world_to_square(S, det.world);
det.ok = det.square ~= "";
det.redPixel = redC; det.corners = pts;
end

function c = blobCentre(mask)
mask = bwareaopen(mask, 30);
st = regionprops(mask, "Area", "Centroid");
if isempty(st), c = [NaN NaN]; return, end
[~, k] = max([st.Area]); c = st(k).Centroid;
end

function P = rayToPlane(uv, hgt, R, C, K)
d = R' * (K \ [uv(:); 1]);                   % ray direction in pattern coords
z = sign(C(3)) * hgt;                        % plane 'hgt' above the board, on the camera side
s = (z - C(3)) / d(3);
P = C + s*d;
end

function L = mapPattern(P, ox, oy, swap)
dx = -sign(ox); dy = -sign(oy);
if swap, P = P([2 1]); end
L = [ox + dx*P(1), oy + dy*P(2)];
end
