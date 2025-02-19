import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

def read_config(config):
    # Load YAML configuration
    with open(config, 'r') as file:
        data = yaml.safe_load(file)  
    return data  

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
        # gray = cv2.medianBlur(gray, 3)
        ret, prepared_img = cv2.threshold(gray, 200,220,cv2.THRESH_BINARY)
        return gray
    
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
        tmatrix = None
        center = None


        if(set([20,21,22,23]).issubset(ids)):
            center = (tvecDict[22]+ tvecDict[20] + tvecDict[23] + tvecDict[21])/4
            
            xvec = (tvecDict[22]+ tvecDict[20] - tvecDict[23] - tvecDict[21])*-1
            xvec = xvec/np.linalg.norm(xvec)
            yvec = (tvecDict[22]+ tvecDict[23] - tvecDict[20] - tvecDict[21])*-1
            yvec = yvec/np.linalg.norm(yvec)
            zvec = np.cross(xvec, yvec)

            tmatrix = np.array([xvec, yvec, zvec])

        return tmatrix, center 
        
    def robots_tracking(self, ids, transMatrixDict, tvecDict, tmatrix, center):
        if self.robot_id in ids:
            rvec = tvecDict[self.robot_id]- center
            robotCoord = [np.dot(rvec, tmatrix[0]), np.dot(rvec, tmatrix[1]), np.dot(rvec, tmatrix[2])]
            robotTransMatrix = np.dot(transMatrixDict[self.robot_id], np.linalg.inv(tmatrix))
            robotTransMatrix[0][2] = 0.0
            robotTransMatrix[0] = robotTransMatrix[0]/np.linalg.norm(robotTransMatrix[0])
            robotTransMatrix[1][2] = 0.0
            robotTransMatrix[1] = robotTransMatrix[1]/np.linalg.norm(robotTransMatrix[1])
            robotTransMatrix[2] = [0.0, 0.0, 1.0]
            
            r = R.from_matrix(robotTransMatrix)
            quaternion = r.as_quat()
            return robotCoord, quaternion
        else:
            return None, None