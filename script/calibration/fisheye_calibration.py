import cv2
import numpy as np
import glob
import json

# Settings for the checkerboard pattern
CHECKERBOARD = (6, 9)  # Specify the number of inner corners per a chessboard row and column
calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + \
                    cv2.fisheye.CALIB_CHECK_COND + \
                    cv2.fisheye.CALIB_FIX_SKEW

# Termination criteria for corner subpixel refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)

# 3D points in the real world space
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3D point in real world space
imgpoints = []  # 2D points in image plane.

# Load calibration images
images = glob.glob('C:/coding/git/EUROBOT_2025/script/samples_chessboard/*.jpg')  # Modify the path to your calibration images

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

    if ret:
        # Refine corners for higher accuracy
        refined_corners = cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), criteria)
        imgpoints.append(refined_corners)
        objpoints.append(objp)

# Calibration
N_OK = len(objpoints)
K = np.zeros((3, 3))  # Intrinsic camera matrix
D = np.zeros((4, 1))  # Distortion coefficients
rvecs = []  # Rotation vectors
tvecs = []  # Translation vectors

# Image dimensions (width and height)
h, w = gray.shape[:2]

# Perform calibration
retval, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
    objpoints,
    imgpoints,
    (w, h),
    K,
    D,
    rvecs,
    tvecs,
    calibration_flags,
    criteria
)

# Output calibration results
print("Calibration successful:", retval)
print("Camera Matrix (K):\n", K)
print("Distortion Coefficients (D):\n", D)

# Save calibration data to JSON file
calibration_data = {
    "camera_matrix": K.tolist(),
    "dist_coeff": D.tolist()
}

with open('C:/coding/git/EUROBOT_2025/script/fisheye_calibration_data.json', 'w') as json_file:
    json.dump(calibration_data, json_file, indent=4)

