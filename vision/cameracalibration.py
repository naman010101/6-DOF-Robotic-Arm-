import numpy as np
from pathlib import Path
CALIB_DIR = Path(__file__).resolve().parent / "calibration"

cameraMatrix = np.loadtxt(str(CALIB_DIR / "cameraMatrix.csv"), delimiter=",")
distCoeffs = np.loadtxt(str(CALIB_DIR / "distCoeffs.csv"), delimiter=",")

# Ensure OpenCV gets a 1×N vector
distCoeffs = distCoeffs.reshape(-1, 1)

print(cameraMatrix)
print(distCoeffs)