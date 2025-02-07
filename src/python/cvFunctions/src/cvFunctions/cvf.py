import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

def read_config(config):
    # Load YAML configuration
    with open(config, 'r') as file:
        data = yaml.safe_load(file)  
    return data  

def angle_between_vectors(vector1, vector2):
    cos = np.dot(vector1, vector2) / (np.linalg.norm(vector1) * np.linalg.norm(vector2))
    sin = np.linalg.norm(np.cross(vector1, vector2)) / (np.linalg.norm(vector1) * np.linalg.norm(vector2))
    angle = np.angle(cos + sin * 1.j, deg=True)
    return angle

class Camera():
    def __init__(self, config_path: str):
        self.config = read_config(config_path)
        self.camera_matrix = np.array(self.config["camera_matrix"], dtype=np.float64)
        self.dist_coefs = np.array(self.config["dist_coeff"], dtype=np.float64)
        self.robot_id = self.config["robot_id"]

        self.arucoParams = cv2.aruco.DetectorParameters()
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.detector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)
    
    def prepare_image(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.normalize(gray, None, 1.0, 255, cv2.NORM_MINMAX, dtype = cv2.CV_8U)
        gray = cv2.medianBlur(gray, 3)
        ret, prepared_img = cv2.threshold(gray, 200,220,cv2.THRESH_BINARY)
        return prepared_img
    
    def detect_markers(self, img):
        tvecDictionary = {}
        transMatrixDictionary = {}
        imgToProduse = self.prepare_image(img)
        corners, ids, rejected = cv2.aruco.detectMarkers(imgToProduse, self.arucoDict, parameters=self.arucoParams)
    
        if ids is not None:
            ids = list(map(lambda x: x[0], ids))
            for i in range(len(ids)):
                if ids[i] in range(10):
                    marker_length = 0.069
                else:
                    marker_length = 0.1

                rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_length, cameraMatrix=self.camera_matrix, distCoeffs= None)
                tvecDictionary[ids[i]] = tvec[0][0]
                
                transMatrixDictionary[ids[i]] = cv2.Rodrigues(rvec)[0]
        
        return ids, transMatrixDictionary, tvecDictionary  
    
    def t_matrix_building(self, ids, tvecDict, transMatrixDict):
        corners = []
        tMatrices = []
        tmatrix = None
        center = None
        for i in ids:
            if i in range(11, 51) and i != 47:
                corners.append(tvecDict[i])
                tMatrices.append(transMatrixDict[i])

        if len(corners) == 4:
            center = sum(np.array(corners)) / 4
            corners = sorted(list(map(lambda x: x.tolist(), corners)))
            corners[:2] = [x for _, x in sorted(zip(list(map(lambda x: x[1], corners[:2])), corners[:2]))]
            corners[2:4] = [x for _, x in sorted(zip(list(map(lambda x: x[1], corners[2:4])), corners[2:4]))]

            for i in range(4):
                corners[i] = np.array(corners[i])

            xvec = (corners[2] + corners[3] - corners[1] - corners[0]) / 2
            yvec = (corners[1] + corners[3] - corners[2] - corners[0]) / 2
            xvec *= -1
            zvec = np.cross(xvec, yvec)

            tmatrix = sum(np.array(tMatrices)) / 4

        return tmatrix, center 
    
    def robots_tracking(self, ids, transMatrixDict, tvecDict, tmatrix, center):
        if self.robot_id in ids:
            robotCoord = np.dot(np.linalg.inv(tmatrix), np.array(tvecDict[self.robot_id] - center)) * 100
            #angle = angle_between_vectors(transMatrixDict[self.robot_id][0], tmatrix[0])
            r = R.from_matrix(np.dot(transMatrixDict[self.robot_id], np.linalg.inv(tmatrix)))
            quaternion = r.as_quat()
            return robotCoord, quaternion
        else:
            return None, None