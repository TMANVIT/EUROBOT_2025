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
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.detector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)

        self.RotSideDict = {
            55: R.from_euler('y', 90, degrees=True).as_matrix(),    # Front: x down, z forward
            56: R.from_euler('x', 90, degrees=True).as_matrix(),    # Right: x forward, z right
            57: R.from_euler('y', -90, degrees=True).as_matrix(),   # Rear: x up, z backward
            58: R.from_euler('x', -90, degrees=True).as_matrix()    # Left: x forward, z left
        }
        self.TvecSideDict = {
            55: np.array([-0.05, 0.0, 0.055]),  # From 55 to self.robot_id
            56: np.array([0.0, 0.05, 0.055]),   # From 56 to self.robot_id
            57: np.array([0.05, 0.0, 0.055]),   # From 57 to self.robot_id
            58: np.array([0.0, -0.05, 0.055])   # From 58 to self.robot_id
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

        marker_size = 0.1  # Specify the size of field markers
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
        
    def robots_tracking(self, ids, transMatrixDict, tvecDict, weightsDictionary, tmatrix, center):
        ourRobot = False
        enemyCoord = None
        enemyQuat = None
        tvecArray = []
        rvecArray = []
        weightsArray = []
        for i in ids:
            if i in list(range(11))+list(self.RotSideDict.keys()):
                rvec = tvecDict[i] - center
                robotCoord = [np.dot(rvec, tmatrix[0]), np.dot(rvec, tmatrix[1]), np.dot(rvec, tmatrix[2])]
                robotTransMatrix = np.linalg.inv(tmatrix) @ transMatrixDict[i]
                if i in self.RotSideDict.keys():
                    # robotCoord += self.TvecSideDict[i]
                    robotTransMatrix = robotTransMatrix @ self.RotSideDict[i]
                robotTransMatrix[0][2] = 0.0
                robotTransMatrix[0] = robotTransMatrix[0] / np.linalg.norm(robotTransMatrix[0])
                robotTransMatrix[1][2] = 0.0
                robotTransMatrix[1] = robotTransMatrix[1] / np.linalg.norm(robotTransMatrix[1])
                robotTransMatrix[2] = [0.0, 0.0, 1.0]
                
            
                
                if i == self.robot_id or i in self.RotSideDict.keys():
                    print(i, robotCoord, robotTransMatrix)
                    ourRobot = True
                    tvecArray.append(np.array(robotCoord))
                    rvecArray.append(robotTransMatrix)
                    weightsArray.append(weightsDictionary[i])
                else:
                    enemyCoord = robotCoord
                    r = R.from_matrix(robotTransMatrix)
                    enemyQuat = r.as_quat()

        if not ourRobot:
            robotCoordAver = None
            quaternion = None
        elif len(tvecArray) == 1:
            r = R.from_matrix(rvecArray[0])
            quaternion = r.as_quat()
            robotCoordAver = tvecArray[0]
        else:
            weightsSum = sum(weightsArray)
            for i in range(len(weightsArray)):
                weightsArray[i] =  weightsArray[i]/weightsSum
            
            print(tvecArray)
            robotCoordAver = np.average(tvecArray, axis = 0, weights= weightsArray)
            AverTrans = np.average(np.array(rvecArray), axis=0, weights=weightsArray)
            r = R.from_matrix(AverTrans)
            quaternion = r.as_quat()

        return our_tvec, our_quat, enemy_tvec, enemy_quat, our_cov, enemy_cov