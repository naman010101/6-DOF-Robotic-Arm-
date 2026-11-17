# Pick-and-place simulation plan (MATLAB)

Goal: simulate the full intended task - the camera sees the chessboard, finds which square
the object is on, the arm picks it up and drops it in a box next to the board - using the
real arm model (URDF + meshes) and the real camera calibration, then use the simulation to
understand why the physical arm could not do it and what would fix it.

## v1 - scene and pipeline

| Part | How it is modelled |
|---|---|
| Arm | `urdf/urdf/robot.urdf` (Fusion export, STL meshes, 5 revolute joints, +/-90 deg) plus a fixed `tool_tip` frame 52 mm out from `gripper_1` (measured from the gripper mesh) |
| Table | plane at z = 0.073 m (bottom of the base mesh) |
| Chessboard | 8x8, 33 mm squares; nearest edge 50 mm from the base (v1 put it 50 mm from the base *axis*, see v4) |
| Object | 20 mm red cube on a chosen square (gripper jaws open ~29 mm) |
| Box | 80 x 80 x 50 mm open box beside the board |
| Camera | phone camera with the calibrated intrinsics (`cameraMatrix.csv`, `distCoeffs.csv`, 1200x1600), 0.6 m from the board, looking down at it |
| Vision | render the camera image (with lens distortion) -> undistort -> find the board with `detectCheckerboardPoints` -> board pose with `estimateExtrinsics` -> find the red blob -> intersect its ray with the cube's top plane -> square name |
| IK | `inverseKinematics` on the URDF; tool pointing straight down, yaw set by the base angle; poses: pre-grasp (+60 mm), grasp, lift, above box, release |
| Motion | joint-space quintic trajectories between poses; collision check (self, table, board, box) along every segment; cube attached to the tool while gripped |
| Servos | commands rounded to whole degrees (0-180 servo range), like the firmware |
| Output | animation video, camera image with detection, per-square reachability, metrics |

## v1 - experiments
1. Vision: detect the right square for every one of the 64 squares.
2. Reachability: which squares the arm can reach with the gripper pointing down, collision-free.
3. Backlash: Monte Carlo (servo dead-band per joint, random approach history) -> tip position
   error at the grasp -> success if the cube centre is inside the jaw tolerance.

## Acceptance
- Correct square detected for all 64 squares.
- At least one full pick -> box -> release run without collisions, shown on video.
- Success rate with and without backlash reported per square.

## Iterations

**v2 - camera rendering.** First renders had a dark bow-tie across the image: the lens-distortion
polynomial folds points far outside the field of view back into it. Fix: clip every polygon to the
camera's field of view (+20 %) before projecting. Then the table was drawn over the board (sorting
polygons by centre distance fails for a large plane); fix: draw flat layers in order, then sort
only the 3-D objects.

**v3 - board orientation.** With the camera working, 50/64 squares were found; the misses had file
and rank swapped. A square 8x8 board looks the same rotated or mirrored, and the first green
fiducial sat on the board's diagonal, which a mirror leaves unchanged. Moved it off the diagonal
(63/64), then to the frame edge nearest the camera so the cube can never hide it (b1 had covered
it). Result: **64/64 squares correct, median position error 0.58 mm, worst 1.12 mm.**

**v4 - placement and a URDF error.** The arm collided with the board at the home pose. Two causes:
the board's near edge was 50 mm from the base axis, inside the 60 mm base disc (the 50 mm in
`generateRobotSquares.m` only makes sense from the base edge -> now 110 mm from the axis); and the
URDF's base rotation axis was 50 mm away from the centre of the base disc (Fusion export error),
so every base rotation swung the arm around the wrong point. Fixed in `urdf/urdf/robot.urdf`
(axis through the disc centre, geometry at the zero pose unchanged).

**v5 - reach.** With the real placement only 50/64 grasps and fewer approaches were reachable, and
only on the near ranks - which did not match the real table (files a-d, all ranks). The app's
`CreateChessBoard.m` runs the files away from the arm; turned the board to match. Result:
**38/64 squares pickable (files a-d, most of e) - exactly the half the real arm was calibrated
on.** The box was moved 90 mm closer to be reachable with the gripper pointing down.

**v6 - full task.** One button: camera image -> square -> IK -> trajectory -> collision check ->
animation. **All 38 reachable squares get a collision-free pick -> box -> release plan.**
Video: `docs/media/sim_pick_and_place_b3.mp4`.

**v7 - why the real arm failed, and the fix.** Monte Carlo over the 38 squares (300 trials each)
with hobby-servo errors (assumed: +/-1.5 deg backlash on the MG995 joints, +/-2.5 deg on the
micro-servo wrist, +/-0.5 deg dead band, whole-degree commands). A grasp succeeds if the fingertip
lands within the 4.5 mm side clearance and does not hit the board.

| Strategy | Grasp success | Median miss |
|---|---|---|
| A: current firmware (a joint can arrive from either side) | 12 % | 9.0 mm |
| B: same-direction approach (overshoot, come back up) | 99 % | 1.9 mm |
| C: B + 0.1-degree commands | 100 % | 1.4 mm |

Sensitivity: with A, even 0.75 deg of backlash halves the success rate; B stays at 99 % as long as
the servo dead band is about 0.5 deg. **Implemented B in `firmware/esp32_main`** (`ANTI_BACKLASH`,
4 deg overshoot; covered by the firmware tests). C is optional: it needs `writeMicroseconds`.

## Limits of the simulation
- Servo errors are assumed typical values, not measured on this arm.
- The URDF gripper is one rigid part; the grasp is modelled as the cube following the fingertip
  once the fingers are within tolerance (no contact physics).
- The simulation uses the URDF for IK. The app's `build_robot_model.m` is a separate, simplified
  model and still disagrees with the hand-measured table; on the real arm the square table is used.
- The camera image is synthetic (flat-shaded); real lighting, glare and other objects are not
  modelled.
