function S = sim_build_scene(opts)
%SIM_BUILD_SCENE Arm, table, chessboard, cube, box and camera for the pick-and-place simulation.
%   S = sim_build_scene()  uses the defaults described in docs/simulation-plan.md.

arguments
    opts.Square (1,1) string = "c3"          % where the cube starts
    opts.CameraDistance (1,1) double = 0.60  % m, camera to board centre
    opts.CameraHeight (1,1) double = 0.45    % m above the table
end

root = fileparts(fileparts(fileparts(mfilename("fullpath"))));

% ---------------- arm ----------------
robot = importrobot(fullfile(root, "urdf", "urdf", "robot.urdf"));
robot.DataFormat = "row";
tip = rigidBody("tool_tip");
setFixedTransform(tip.Joint, trvec2tform([-0.052 0.002 0.002]));  % fingertip centre, from the gripper mesh
addBody(robot, tip, "gripper_1");
S.robot = robot;
S.tool = "tool_tip";
S.baseAxis = [0 -0.002];        % base rotation axis (x, y): centre of the base disc
S.home = zeros(1, 5);

% ---------------- table and board ----------------
S.tableZ = 0.073;               % bottom of the base mesh
S.square = 0.033;
S.boardThickness = 0.012;
S.boardZ = S.tableZ + S.boardThickness;
S.boardNear = 0.110;            % nearest (file a) edge from the base axis: 60 mm base radius + the 50 mm
                                % gap in generateRobotSquares.m (measured from the base edge)
S.frameWidth = 0.025;
% green sticker on the frame beside file h (the edge nearest the camera, so a piece on the board
% can never hide it), level with rank 2: off the board's diagonal, so it fixes both the
% 180-degree and the file/rank (mirror) ambiguity of a square chessboard
S.fiducialLocal = [-2.5*S.square, 4*S.square + S.frameWidth/2];
% board frame: origin at the board centre, +x towards higher ranks (across the arm),
% +y towards higher files (away from the arm, world -X). Files a-h run away from the arm, as in
% matlab/chess/CreateChessBoard.m; the real arm could only reach (and was only calibrated on) a-d.
c = [S.baseAxis(1) - (S.boardNear + 4*S.square), S.baseAxis(2), S.boardZ];
S.boardT = trvec2tform(c) * axang2tform([0 0 1 pi/2]);  % board +y (files) = world -X

% ---------------- cube ----------------
S.cubeSize = 0.020;
S.cubeStart = sim_square_to_world(S, opts.Square) + [0 0 S.cubeSize/2];
S.cubeSquare = opts.Square;

% ---------------- box ----------------
S.boxSize = [0.08 0.08 0.05];
S.boxCentre = [S.baseAxis(1) - 0.06, 0.19, S.tableZ];      % centre of the box floor, beside the board
S.jawOpen = 0.029;

% ---------------- camera ----------------
K = readmatrix(fullfile(root, "vision", "calibration", "cameraMatrix.csv"));
d = readmatrix(fullfile(root, "vision", "calibration", "distCoeffs.csv"));
S.imageSize = [1600 1200];      % rows, cols (portrait phone image)
S.intrinsics = cameraIntrinsics([K(1,1) K(2,2)], [K(1,3) K(2,3)], S.imageSize, ...
    "RadialDistortion", d(1:2), "TangentialDistortion", d(3:4));
% camera on the far side of the board, looking back at it and down
target = c;
camPos = [c(1) - opts.CameraDistance*0.75, 0, S.tableZ + opts.CameraHeight];
S.cameraPose = lookAt(camPos, target);
S.cameraPosition = camPos;

% ---------------- collision objects ----------------
S.envCollision = sim_environment_collision(S);
end

function T = lookAt(pos, target)
% camera frame: z forward (optical axis), x right, y down in the image
z = (target - pos) / norm(target - pos);
x = cross(z, [0 0 1]); x = x / norm(x);
y = cross(z, x);
T = [x' y' z' pos'; 0 0 0 1];
end
