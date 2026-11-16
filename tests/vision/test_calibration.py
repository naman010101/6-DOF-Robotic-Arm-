"""Checks the saved camera calibration against the 14 chessboard photos (no camera needed).

For every photo where the 7x7 inner corners are found, the board pose is solved with the
saved cameraMatrix/distCoeffs and the corners are re-projected; the pixel error shows how
well the calibration fits. The photos are also re-calibrated from scratch for comparison.

    python tests/vision/test_calibration.py
"""
import glob
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CAL = ROOT / "vision" / "calibration"
SQUARE_MM = 33.0
PATTERN = (7, 7)  # inner corners of an 8x8 board

K = np.loadtxt(CAL / "cameraMatrix.csv", delimiter=",")
dist = np.loadtxt(CAL / "distCoeffs.csv", delimiter=",").reshape(-1, 1)

grid = np.zeros((PATTERN[0] * PATTERN[1], 3), np.float32)
grid[:, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2) * SQUARE_MM

obj_pts, img_pts, errors = [], [], []
size = None
for f in sorted(glob.glob(str(CAL / "images" / "*.jpeg"))):
    gray = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
    size = gray.shape[::-1]
    found, corners = cv2.findChessboardCornersSB(gray, PATTERN)
    if not found:
        print(f"  {Path(f).name:12s} corners not found")
        continue
    ok, rvec, tvec = cv2.solvePnP(grid, corners, K, dist)
    proj, _ = cv2.projectPoints(grid, rvec, tvec, K, dist)
    err = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - corners.reshape(-1, 2)) ** 2, axis=1))))
    errors.append(err)
    obj_pts.append(grid)
    img_pts.append(corners)
    print(f"  {Path(f).name:12s} reprojection RMS {err:5.2f} px   board distance {np.linalg.norm(tvec)/10:5.1f} cm")

print(f"\nboards found: {len(errors)}/14")
if len(errors) < 3:
    sys.exit("FAIL: too few boards detected")

rms, K2, d2, _, _ = cv2.calibrateCamera(obj_pts, img_pts, size, None, None)
print(f"saved calibration : mean reprojection {np.mean(errors):.2f} px, worst {np.max(errors):.2f} px")
print(f"fresh calibration : RMS {rms:.2f} px, fx={K2[0,0]:.0f} fy={K2[1,1]:.0f} cx={K2[0,2]:.0f} cy={K2[1,2]:.0f}")
print(f"saved values      : fx={K[0,0]:.0f} fy={K[1,1]:.0f} cx={K[0,2]:.0f} cy={K[1,2]:.0f}")

ok = np.mean(errors) < 2.0
print("PASS" if ok else "FAIL: saved calibration re-projects worse than 2 px on average")
sys.exit(0 if ok else 1)
