import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

def read_config(config_path):
    with open(config_path, 'r') as file:
        data = yaml.safe_load(file)
    return data

class Camera:
    def __init__(self, config_path: str):
        self.config = read_config(config_path)
        self.camera_matrix = np.array(self.config["camera_matrix"], dtype=np.float64)
        self.dist_coefs = np.array(self.config["dist_coeff"], dtype=np.float64)
        self.newcameramatrix, _ = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coefs, (1600, 896), 0.5, (1600, 896))
        self.robot_id = self.config["robot_id"]

        self.arucoParams = cv2.aruco.DetectorParameters()
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.detector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)

        self.RotSideDict = {
            55: R.from_euler('y', 90, degrees=True).as_matrix(),    # Front: x down, z forward
            56: R.from_euler('x', 90, degrees=True).as_matrix(),    # Right: x forward, z right
            57: R.from_euler('y', -90, degrees=True).as_matrix(),   # Rear: x up, z backward
            58: R.from_euler('x', -90, degrees=True).as_matrix(),   # Left: x forward, z left
            964: np.eye(3),   # Top: no rotation (example)
            992: np.eye(3)    # Side: x forward, z right (example)
        }
        self.TvecSideDict = {
            55: np.array([-0.05, 0.0, 0.055]),  # From 55 to self.robot_id
            56: np.array([0.0, 0.05, 0.055]),   # From 56 to self.robot_id
            57: np.array([0.05, 0.0, 0.055]),   # From 57 to self.robot_id
            58: np.array([0.0, -0.05, 0.055]),  # From 58 to self.robot_id
            127: np.array([-0.043, 0.15, 0.135]),   # From 127 to self.robot_id 
            126: np.array([-0.043, -0.15, 0.135])   # From 126 to self.robot_id
        }

        self.field_markers = {
            20: np.array([-0.9, 0.4, 0.0]),
            21: np.array([0.9, 0.4, 0.0]),
            22: np.array([-0.9, -0.4, 0.0]),
            23: np.array([0.9, -0.4, 0.0])
        }

        self.last_tmatrix = None
        self.last_center = None
        self.initialized = False

    def prepare_image(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        # gray = clahe.apply(gray)
        # gray = cv2.GaussianBlur(gray, (3, 3), 0)
        # gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 21, 1)
        return gray

    def detect_markers(self, img):
        img_prepared = self.prepare_image(img)
        corners, ids, _ = self.detector.detectMarkers(img_prepared)
        if ids is not None:
            ids = list(map(lambda x: x[0], ids))
        return ids, corners

    def t_matrix_building(self, ids, corners):
        if not ids or not any(marker_id in self.field_markers for marker_id in ids):
            return self.last_tmatrix, self.last_center

        field_corners = []
        field_ids = []
        for i, marker_id in enumerate(ids):
            if marker_id in self.field_markers:
                field_corners.append(corners[i][0])
                field_ids.append(marker_id)

        if not field_ids:
            return self.last_tmatrix, self.last_center

        print(f"Visible field markers: {len(field_ids)}")

        marker_size = 0.1  # Size of field markers
        object_points = []
        image_points = []
        for mid, corners in zip(field_ids, field_corners):
            obj_pts = np.array([
                [-marker_size / 2, marker_size / 2, 0],
                [marker_size / 2, marker_size / 2, 0],
                [marker_size / 2, -marker_size / 2, 0],
                [-marker_size / 2, -marker_size / 2, 0]
            ], dtype=np.float32) + self.field_markers[mid]
            object_points.extend(obj_pts)
            image_points.extend(corners)

        object_points = np.array(object_points, dtype=np.float32)
        image_points = np.array(image_points, dtype=np.float32)

        if not self.initialized and len(field_ids) >= 3:
            success, rvec, tvec = cv2.solvePnP(
                object_points, image_points, self.camera_matrix, self.dist_coefs,
                flags=cv2.SOLVEPNP_SQPNP
            )
            if success:
                tmatrix = cv2.Rodrigues(rvec)[0]
                center = tvec.flatten()
                self.last_tmatrix = tmatrix
                self.last_center = center
                self.initialized = True
                print(f"Initialized tmatrix:\n{tmatrix}\ncenter: {center}")
            else:
                print("Failed to initialize with solvePnP")
                return self.last_tmatrix, self.last_center
        elif self.initialized and len(field_ids) >= 3:
            success, rvec, tvec = cv2.solvePnP(
                object_points, image_points, self.camera_matrix, self.dist_coefs,
                flags=cv2.SOLVEPNP_ITERATIVE, useExtrinsicGuess=True,
                rvec=cv2.Rodrigues(self.last_tmatrix)[0], tvec=self.last_center
            )
            if success:
                center = tvec.flatten()
                self.last_center = center
            else:
                print("Failed to update center with solvePnP")
            tmatrix = self.last_tmatrix
        else:
            return self.last_tmatrix, self.last_center

        return tmatrix, center

    def estimate_robot_pose(self, ids, corners, tmatrix, center, is_our_robot=True):
        if not ids or tmatrix is None or center is None:
            return None, None, None

        robot_corners = []
        robot_ids = []
        for i, marker_id in enumerate(ids):
            if is_our_robot:
                if marker_id != self.robot_id and marker_id not in self.RotSideDict:
                    continue
            else:
                if not (1 <= marker_id <= 10 and marker_id != self.robot_id):
                    continue
            robot_corners.append(corners[i][0])
            robot_ids.append(marker_id)

        if not robot_ids:
            return None, None, None

        print(f"Visible robot markers: {len(robot_ids)}")

        object_points = []
        image_points = []
        for mid, corners in zip(robot_ids, robot_corners):
            # Set marker size based on ID
            if mid in [127, 126]:
                marker_length = 0.085  # 8.5 cm for markers 964 and 992
            elif 1 <= mid <= 10:
                marker_length = 0.07   # 7 cm for enemy markers
            else:
                marker_length = 0.05   # 5 cm for other markers (55-58, self.robot_id)
            
            obj_pts = np.array([
                [-marker_length / 2, marker_length / 2, 0],
                [marker_length / 2, marker_length / 2, 0],
                [marker_length / 2, -marker_length / 2, 0],
                [-marker_length / 2, -marker_length / 2, 0]
            ], dtype=np.float32)
            if mid in self.RotSideDict:
                rot_side = self.RotSideDict[mid]
                tvec_side = self.TvecSideDict[mid]
                obj_pts = np.dot(obj_pts, rot_side.T) - tvec_side  # Transformation to self.robot_id system
            object_points.extend(obj_pts)
            image_points.extend(corners)

        object_points = np.array(object_points, dtype=np.float32)
        image_points = np.array(image_points, dtype=np.float32)

        success, rvec, tvec = cv2.solvePnP(
            object_points, image_points, self.camera_matrix, self.dist_coefs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        if not success:
            print("Failed to estimate robot pose with solvePnP")
            return None, None, None

        tvec_cam = tvec.flatten()
        rot_matrix = cv2.Rodrigues(rvec)[0]

        robot_tvec = np.dot(tmatrix.T, tvec_cam - center)
        robot_rot_matrix = np.dot(tmatrix.T, rot_matrix)
        quat = R.from_matrix(robot_rot_matrix).as_quat()

        _, jac = cv2.projectPoints(object_points, rvec, tvec, self.camera_matrix, self.dist_coefs)
        J = jac[:, :6]
        sigma2 = 1.0
        cov = np.linalg.pinv(J.T @ J) * sigma2

        return robot_tvec, quat, cov

    def robots_tracking(self, img):
        ids, corners = self.detect_markers(img)
        tmatrix, center = self.t_matrix_building(ids, corners)

        our_tvec, our_quat, our_cov = self.estimate_robot_pose(ids, corners, tmatrix, center, is_our_robot=True)
        enemy_tvec, enemy_quat, enemy_cov = self.estimate_robot_pose(ids, corners, tmatrix, center, is_our_robot=False)

        if our_tvec is not None:
            print(f"Our robot: x={our_tvec[0]:.3f}, y={our_tvec[1]:.3f}, z={our_tvec[2]:.3f}")
        if enemy_tvec is not None:
            print(f"Enemy robot: x={enemy_tvec[0]:.3f}, y={enemy_tvec[1]:.3f}, z={enemy_tvec[2]:.3f}")

        return our_tvec, our_quat, enemy_tvec, enemy_quat, our_cov, enemy_cov