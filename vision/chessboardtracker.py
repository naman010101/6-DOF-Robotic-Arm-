import cv2
import numpy as np
from pathlib import Path
CALIB_DIR = Path(__file__).resolve().parent / "calibration"
import json
import argparse

cameraMatrix = np.loadtxt(str(CALIB_DIR / "cameraMatrix.csv"), delimiter=",")
distCoeffs = np.loadtxt(str(CALIB_DIR / "distCoeffs.csv"), delimiter=",")

parser = argparse.ArgumentParser()
parser.add_argument("--rotate", type=int, default=0,
                    choices=[0, 90, 180, 270],
                    help="Rotation of the board orientation relative to the camera (clockwise). "
     "For example: camera on the right = 90°, camera behind = 180°, camera on the left = 270°."
)

parser.add_argument("--source", default="http://192.168.5.122:8080/video",
                    help="Camera: phone IP Webcam stream http://<phone-ip>:8080/video (default is the last-used IP), "
                         "a webcam index such as 0, or an image file.")
args = parser.parse_args()
CAM_ROT = args.rotate

# ===== Chessboard dimensions =====
SQUARE_SIZE = 33.0   # mm (change this to your actual square size)
BOARD_SIZE = 8 * SQUARE_SIZE
HALF = BOARD_SIZE / 2

# Board coordinate system
# Origin = center of chessboard
# +X -> files (a -> h)
# +Y -> ranks (1 -> 8)

objectPoints = np.array([
    [-HALF, -HALF, 0],
    [ HALF, -HALF, 0],
    [ HALF,  HALF, 0],
    [-HALF,  HALF, 0]
], dtype=np.float32)

points = []
   
def mouse_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(points) < 4:
            points.append((x, y))
            print(f"[INFO] Point {len(points)}: {x}, {y}")
        else:
            print("[INFO] 4 points have been selected. Press 'r' to reset or 's' to save.")

def remap_index(r_disp, c_disp, cam_rot):
    """Remap the (row, column) indices from the camera view to the standard chessboard notation."""
    if cam_rot == 0:
        r_std, c_std = r_disp, c_disp
    elif cam_rot == 90:
        r_std, c_std = c_disp, 7 - r_disp
    elif cam_rot == 180:
        r_std, c_std = 7 - r_disp, 7 - c_disp
    elif cam_rot == 270:
        r_std, c_std = 7 - c_disp, r_disp
    else:
        r_std, c_std = r_disp, c_disp
    return r_std, c_std


source = int(args.source) if args.source.isdigit() else args.source
still = cv2.imread(source) if isinstance(source, str) and not source.startswith("http") else None
cap = None if still is not None else cv2.VideoCapture(source)

if still is None and not cap.isOpened():

    print("❌ Camera could not be opened.")
    exit()

cv2.namedWindow("Chessboard Calibration")
cv2.setMouseCallback("Chessboard Calibration", mouse_click)

print("📸 Instructions:")
print(" 1️⃣ Point the camera to the position that will be used (e.g., from the side).")
print(" 2️⃣ Click on the 4 corner points of the board (in order: top-left, top-right, bottom-right, bottom-left).")
print(" 3️⃣ Press 'r' to reset, 's' to save the results to sqdict.json, 'q' to quit.")
print(f"[INFO] Board rotation relative to the camera: {CAM_ROT}° (CW)\n")

while True:
    if still is not None:
        ret, frame = True, still.copy()
    else:
        ret, frame = cap.read()
    if not ret:
        continue

    vis = frame.copy()

    # Draw the clicked points
    for idx, p in enumerate(points):
        cv2.circle(vis, p, 6, (0, 0, 255), -1)
        cv2.putText(vis, str(idx+1), (p[0]+8, p[1]-8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

    if len(points) == 4:
        # Draw the board outline and grid
        pts = np.array(points, dtype=np.int32)
        cv2.polylines(vis, [pts], True, (255,255,255), 2)

        src = np.array([[0,0],[8,0],[8,8],[0,8]], dtype=np.float32)
        dst = np.array(points, dtype=np.float32)
        # ===== Estimate board pose =====
        success, rvec, tvec = cv2.solvePnP(
        objectPoints,
        dst,
        cameraMatrix,
        distCoeffs
       )

        if success:
            cv2.putText(vis, f"X: {tvec[0][0]:.1f} mm", (10,30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0),2)

            cv2.putText(vis, f"Y: {tvec[1][0]:.1f} mm", (10,60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0),2)

            cv2.putText(vis, f"Z: {tvec[2][0]:.1f} mm", (10,90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0),2)
            
        H = cv2.getPerspectiveTransform(src, dst)

        src_grid = np.array([[[x,y] for x in range(9)] for y in range(9)], dtype=np.float32)
        dst_grid = cv2.perspectiveTransform(src_grid.reshape(-1,1,2), H).reshape(9,9,2)

        
        for r in range(9):
            cv2.polylines(vis, [dst_grid[r,:,:].astype(int)], False, (180,180,180), 1)
        for c in range(9):
            cv2.polylines(vis, [dst_grid[:,c,:].astype(int)], False, (180,180,180), 1)

       
        files = 'abcdefgh'
        ranks = '87654321'
        font = cv2.FONT_HERSHEY_SIMPLEX

        for r in range(8):
            for c in range(8):
                r_std, c_std = remap_index(r, c, CAM_ROT)
                file_letter = files[c_std]
                rank_char = ranks[r_std]
                label = f"{file_letter}{rank_char}"

                center = dst_grid[r, c] + (dst_grid[r+1, c+1] - dst_grid[r, c]) / 2
                cx, cy = int(center[0]), int(center[1])
                cv2.putText(vis, label, (cx-12, cy+5), font, 0.5, (0,255,255), 1, cv2.LINE_AA)

    cv2.imshow("Chessboard Calibration", vis)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        print("Exiting without saving.")
        break
    elif key == ord('r'):
        points = []
        print("[INFO] Reset the points.")
    elif key == ord('s'):
        if len(points) != 4:
            print("[WARN] Must click 4 points first before saving.")
            continue

        src = np.array([[0,0],[8,0],[8,8],[0,8]], dtype=np.float32)
        dst = np.array(points, dtype=np.float32)
        H = cv2.getPerspectiveTransform(src, dst)

        src_grid = np.array([[[x,y] for x in range(9)] for y in range(9)], dtype=np.float32)
        dst_grid = cv2.perspectiveTransform(src_grid.reshape(-1,1,2), H).reshape(9,9,2)

        displayed_squares = {}
        for r in range(8):
            for c in range(8):
                tl = dst_grid[r, c].tolist()
                tr = dst_grid[r, c+1].tolist()
                br = dst_grid[r+1, c+1].tolist()
                bl = dst_grid[r+1, c].tolist()
                displayed_squares[(r,c)] = [tl,tr,br,bl]

       
        files = 'abcdefgh'
        ranks = '87654321'
        squares_std = {}

for (r_disp, c_disp), poly in displayed_squares.items():

    r_std, c_std = remap_index(r_disp, c_disp, CAM_ROT)

    file_letter = files[c_std]
    rank_char = ranks[r_std]

    square_name = f"{file_letter}{rank_char}"

    # ==========================
    # Board coordinates (mm)
    # Origin at board center
    # ==========================

    board_x = (c_std - 3.5) * SQUARE_SIZE
    board_y = (r_std - 3.5) * SQUARE_SIZE

    squares_std[square_name] = {

        "pixel": poly,

        "board": [
            round(board_x,2),
            round(board_y,2),
            0.0
        ]

    }

    with open('sqdict.json', 'w') as f:
        json.dump(squares_std, f, indent=2)
    print(f"[✅] sqdict.json saved with orientation rotation {CAM_ROT}° (notation adjusted).")
    break

if cap is not None:
    cap.release()
cv2.destroyAllWindows()